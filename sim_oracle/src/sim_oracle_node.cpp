
#include "ros/ros.h"

#include "sim_oracle/SampleForceTorque.h"
#include "sim_architect/EvalForceTorque.h"
#include "sim_architect/GetNumberOfSimulations.h"

#include "geometry_msgs/Pose.h"

const int ALL_THREADS(0);

geometry_msgs::Pose IDEAL_POSE;

double loglikelihood_f(geometry_msgs::Pose& p) {
	ROS_INFO_STREAM("Position: \n x:" << p.position.x <<
	                " ideal x:" << IDEAL_POSE.position.x);
	double dist =  std::sqrt(std::pow(p.position.x - IDEAL_POSE.position.x, 2) +
	                         std::pow(p.position.y - IDEAL_POSE.position.y, 2) +
	                         std::pow(p.position.z - IDEAL_POSE.position.z, 2) );
	double ideal_norm = std::sqrt(std::pow(IDEAL_POSE.position.x, 2) +
	                              std::pow(IDEAL_POSE.position.y, 2) +
	                              std::pow(IDEAL_POSE.position.z, 2));
	ROS_INFO_STREAM("Dist: " << dist);
	return -log(dist);
	// if (dist <= ideal_norm)
	// 	return (dist / ideal_norm);
	// else
	// 	return std::max(1.0 - dist / ideal_norm, 0.0d);
}

double loglikelihood_f0(geometry_msgs::PoseArray& ps) {
	return loglikelihood_f(ps.poses[0]);
}

bool takeNewSample(double l1, double l2) {
	if (l1 == 0) return true;
	static std::default_random_engine generator;
	static std::uniform_real_distribution<double> distribution(0.0, 1.0);
	double draw_dist = distribution(generator);
	ROS_INFO_STREAM("The drawn var is: " << draw_dist << " prob " << l2 / l1);
	return (l2 / l1 >= draw_dist);
}

bool sample_callback(
    sim_oracle::SampleForceTorque::Request& req,
    sim_oracle::SampleForceTorque::Response& res) {

	sim_architect::GetNumberOfSimulations num_msg;
	if (!ros::service::call("sim_architect/getNumberOfSimulations", num_msg)) {
		ROS_WARN_STREAM("Cannot request number of simulations!");
		return false;
	}

	sim_architect::EvalForceTorque eft_msg;
	eft_msg.request.object_name = "cube0";
	eft_msg.request.distribution_type =
	    sim_architect::EvalForceTorque::Request::UNIFORM;
	eft_msg.request.params = std::vector<double>(2);
	eft_msg.request.params[
	    sim_architect::EvalForceTorque::Request::UNIFORM_MIN] = -3;
	eft_msg.request.params[
	    sim_architect::EvalForceTorque::Request::UNIFORM_MAX] = 3;
	eft_msg.request.names.push_back("cube0");
	// Generate 0 mean priors
	geometry_msgs::Wrench w;
	w.force.x = 0; w.force.y = 0; w.force.z = 0;
	w.torque.x = 0; w.torque.y = 0; w.torque.z = 0;
	for (int i = 0; i < num_msg.response.count; ++i) {
		eft_msg.request.wrench.push_back(w);
	}
	if (!ros::service::call("sim_architect/evalForceTorque", eft_msg)) {
		ROS_WARN_STREAM("Cannot call sim_architect for 'evalForceTorque'");
	}
	std::vector<double> llhs_old(eft_msg.response.poses.size(), 0);
	std::vector<double> llhs_new(eft_msg.response.poses.size(), 0);
	for (int i = 0; i < eft_msg.response.poses.size(); ++i) {
		llhs_old[i] = loglikelihood_f0(eft_msg.response.poses[i]);
		ROS_INFO_STREAM("For " << i << " the likelihood is : "
		                << llhs_old[i]);
	}

	// After generating uniform samples, continue an MCMC chain from each.
	// Update model
	eft_msg.request.distribution_type =
	    sim_architect::EvalForceTorque::Request::GAUSSIAN;
	eft_msg.request.params = std::vector<double>(2);
	eft_msg.request.params[
	    sim_architect::EvalForceTorque::Request::GAUSSIAN_MEAN] = 0;
	eft_msg.request.params[
	    sim_architect::EvalForceTorque::Request::GAUSSIAN_VAR] = 0.1;
	for (int i = 0; i < num_msg.response.count; ++i) {
		eft_msg.request.wrench[i] = eft_msg.response.wrench[i];
	}
	ROS_INFO_STREAM("Respose: " << eft_msg.response);
	for (int i = 0; i < req.burnout; ++i) {
		// Generate new draws
		if (!ros::service::call("sim_architect/evalForceTorque", eft_msg)) {
			ROS_WARN_STREAM("Cannot call sim_architect for 'evalForceTorque'" <<
			                " for attempt" << i << "!");
		}
		// Calculate log likelihood
		for (int i = 0; i < eft_msg.response.poses.size(); ++i) {
			llhs_new[i] = loglikelihood_f0(eft_msg.response.poses[i]);
			ROS_INFO_STREAM("For " << i << " the likelihood is : "
			                << llhs_new[i]);
		}

		// Copy last estimations
		for (int i = 0; i < num_msg.response.count; ++i) {
			// If new sample has better likelihood, take it!
			// Else remain with the same estimate
			if (takeNewSample(llhs_old[i], llhs_new[i])) {
				eft_msg.request.wrench[i] = eft_msg.response.wrench[i];
			}
		}
		llhs_old = llhs_new;

		ROS_WARN_STREAM("------------- " << i);
	}

	// Copy the estimates
	for (int i = 0; i < llhs_new.size(); ++i) {
		res.likelihood.push_back(llhs_new[i]);
		res.wrench.push_back(eft_msg.response.wrench[i]);
	}
	return true;
}

int main(int argc, char** argv) {
	ros::init(argc, argv, "sim_oracle");
	ros::NodeHandle node("~");

	ros::ServiceServer sample_service =
	    node.advertiseService("sample", sample_callback);

	// Generate ideal pose
	IDEAL_POSE.position.x = 0.25; IDEAL_POSE.position.y = 0;
	IDEAL_POSE.position.z = 0;


	ROS_INFO("Oracle initialized.");

	ros::AsyncSpinner spinner(ALL_THREADS);
	spinner.start();

	ros::waitForShutdown();

	return 0;
}
