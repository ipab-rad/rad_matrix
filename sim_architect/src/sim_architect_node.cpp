#include <ros/ros.h>
#include <ros/time.h>

#include <iostream>
#include <iomanip>
#include <string>
#include <map>
#include <random>
#include <cmath>
#include <thread>
#include <mutex>

#include <std_msgs/String.h>
#include <vrep_plugin_server/AddForce.h>
#include <vrep_plugin_server/AddForceTorque.h>
#include <vrep_common/simRosSetObjectPose.h>
#include <vrep_common/simRosGetObjectPose.h>
#include <vrep_common/simRosStopSimulation.h>
#include <vrep_common/simRosStartSimulation.h>
#include "sim_architect/GetNumberOfSimulations.h"
#include "sim_architect/GetObjectPose.h"
#include "sim_architect/GetSimulationNames.h"

#include <Eigen/Geometry>
#include <Eigen/Core>


const int WORLD_FRAME(-1);
double _2PI(2 * M_PI);

ros::NodeHandle* node;
std::map<std::string, ros::Time> sims;

// Service objects
ros::ServiceServer get_number_of_simulations_service;
ros::ServiceServer set_object_pose_service;
ros::ServiceServer stop_simulation_service;
ros::ServiceServer start_simulation_service;
ros::ServiceServer add_force_service;
ros::ServiceServer add_force_torque_service;
ros::ServiceServer get_object_pose_service;
ros::ServiceServer get_simulation_names_service;

// Random variation data
std::random_device rd;
std::mt19937 gen(rd());
const float NOISE_pos_variance(0.001);      // 1mm
const float NOISE_rotation_variance(0.1);   //
const float NOISE_force_variance(0.1);      // 0.1N
const float NOISE_torque_variance(0.01);    //

std::normal_distribution<> NOISE_POS(0, NOISE_pos_variance);
std::normal_distribution<> NOISE_ROT(0, NOISE_rotation_variance);
std::normal_distribution<> NOISE_FORCE(0, NOISE_force_variance);
std::normal_distribution<> NOISE_TORQUE(0, NOISE_torque_variance);

// Helper functions
Eigen::Quaterniond RPY2Quaternion(double r, double p, double y) {
    Eigen::AngleAxisd rollAngle(r, Eigen::Vector3d::UnitZ());
    Eigen::AngleAxisd yawAngle(y, Eigen::Vector3d::UnitY());
    Eigen::AngleAxisd pitchAngle(p, Eigen::Vector3d::UnitX());

    Eigen::Quaternion<double> q = rollAngle * yawAngle * pitchAngle;
    return q;
}

// Callbacks
void sim_heartbeat_message_callback(const std_msgs::StringConstPtr& msg) {
    if (sims.find(msg->data) == sims.end()) {
        ROS_INFO_STREAM("New simulation on: " << msg->data);
    }
    sims[msg->data] = ros::Time::now();
}

bool get_number_of_simulations_callback(
    sim_architect::GetNumberOfSimulations::Request& req,
    sim_architect::GetNumberOfSimulations::Response& res) {
    res.count = sims.size();
    return true;
}

bool set_object_pose_callback(
    vrep_common::simRosSetObjectPose::Request& req,
    vrep_common::simRosSetObjectPose::Response& res) {
    ros::WallTime begin = ros::WallTime::now();
    int all_ok = 0;
    std::vector<std::thread> ts;
    std::mutex all_ok_mutex;
    vrep_common::simRosSetObjectPose msg;
    msg.request = req;
    ROS_DEBUG_STREAM("Original: " << req.pose);
    for (auto& sim : sims) {
        ts.push_back(
            std::thread([sim, &all_ok, &all_ok_mutex]
        (vrep_common::simRosSetObjectPose nmsg) ->void {

            nmsg.request.pose.position.x += NOISE_POS(gen);
            nmsg.request.pose.position.y += NOISE_POS(gen);
            nmsg.request.pose.position.z += NOISE_POS(gen);

            Eigen::Quaternionf org(
                nmsg.request.pose.orientation.x,
                nmsg.request.pose.orientation.y,
                nmsg.request.pose.orientation.z,
                nmsg.request.pose.orientation.w);
            Eigen::Matrix3f m;
            m = Eigen::AngleAxisf(NOISE_ROT(gen), Eigen::Vector3f::UnitX())
            * Eigen::AngleAxisf(NOISE_ROT(gen),  Eigen::Vector3f::UnitY())
            * Eigen::AngleAxisf(NOISE_ROT(gen), Eigen::Vector3f::UnitZ());
            Eigen::Matrix3f final_mat = org.toRotationMatrix() * m;
            Eigen::Quaternionf final(final_mat);
            final.normalize();

            nmsg.request.pose.orientation.x = final.x();
            nmsg.request.pose.orientation.y = final.y();
            nmsg.request.pose.orientation.z = final.z();
            nmsg.request.pose.orientation.w = final.w();

            ROS_INFO_STREAM("Setting object pose on: " << sim.first);
            if (ros::service::call(sim.first + "/simRosSetObjectPose", nmsg) &&
            (nmsg.response.result != -1)) {
                all_ok_mutex.lock();
                ++all_ok;
                all_ok_mutex.unlock();
            } else {
                ROS_WARN_STREAM("Cannot set object position on " << sim.first <<
                " at: \n" << nmsg.request);
            }
        }, msg));
    }
    std::for_each(ts.begin(), ts.end(), [](std::thread & t) {
        t.join();
    });
    ROS_INFO_STREAM("[simRosSetObjectPose] Time elapsed: " <<
                    ros::WallTime::now() - begin);
    res.result = all_ok;
    return true;
}

bool stop_simulation_callback(vrep_common::simRosStopSimulation::Request& req,
                              vrep_common::simRosStopSimulation::Response& res) {
    ros::WallTime begin = ros::WallTime::now();
    int all_ok = 0;
    std::vector<std::thread> ts;
    std::mutex all_ok_mutex;
    vrep_common::simRosStopSimulation msg;
    for (auto& sim : sims) {
        ts.push_back(
            std::thread([sim, &all_ok, &all_ok_mutex]
        (vrep_common::simRosStopSimulation nmsg) ->void {
            ROS_INFO_STREAM("Stopping simulation on: " << sim.first);
            if (ros::service::call(sim.first + "/simRosStopSimulation", nmsg) &&
            (nmsg.response.result != -1)) {
                all_ok_mutex.lock();
                ++all_ok;
                all_ok_mutex.unlock();
            } else {
                ROS_WARN_STREAM("Could not stop simulation! (" << sim.first
                << " [" << sim.second << "])");
            }
        }, msg));
    }
    std::for_each(ts.begin(), ts.end(), [](std::thread & t) {
        t.join();
    });
    ROS_INFO_STREAM("[simRosStopSimulation] Time elapsed: " <<
                    ros::WallTime::now() - begin);
    res.result = all_ok;
    return true;
}

bool start_simulation_callback(vrep_common::simRosStartSimulation::Request& req,
                               vrep_common::simRosStartSimulation::Response& res) {
    ros::WallTime begin = ros::WallTime::now();
    int all_ok = 0;
    std::vector<std::thread> ts;
    std::mutex all_ok_mutex;
    vrep_common::simRosStartSimulation msg;
    for (auto& sim : sims) {
        ts.push_back(
            std::thread([sim, &all_ok, &all_ok_mutex]
        (vrep_common::simRosStartSimulation nmsg) ->void {
            ROS_INFO_STREAM("Starting simulation on: " << sim.first);
            if (ros::service::call(sim.first + "/simRosStartSimulation", nmsg) &&
            (nmsg.response.result != -1)) {
                all_ok_mutex.lock();
                ++all_ok;
                all_ok_mutex.unlock();
            } else {
                ROS_WARN_STREAM("Could not start simulation! (" << sim.first
                << " [" << sim.second << "])");
            }
        }, msg));
    }
    std::for_each(ts.begin(), ts.end(), [](std::thread & t) {
        t.join();
    });
    ROS_INFO_STREAM("[simRosStartSimulation] Time elapsed: " <<
                    ros::WallTime::now() - begin);
    res.result = all_ok;
    return true;
}

bool add_force_callback(
    vrep_plugin_server::AddForce::Request& req,
    vrep_plugin_server::AddForce::Response& res) {
    ros::WallTime begin = ros::WallTime::now();
    int all_ok = 0;
    std::vector<std::thread> ts;
    std::mutex all_ok_mutex;
    vrep_plugin_server::AddForce msg;
    msg.request = req;
    for (auto& sim : sims) {
        ts.push_back(
            std::thread([sim, &all_ok, &all_ok_mutex]
        (vrep_plugin_server::AddForce nmsg) ->void {
            nmsg.request.position.x += NOISE_POS(gen);
            nmsg.request.position.y += NOISE_POS(gen);
            nmsg.request.position.z += NOISE_POS(gen);

            nmsg.request.force.x += NOISE_FORCE(gen);
            nmsg.request.force.y += NOISE_FORCE(gen);
            nmsg.request.force.z += NOISE_FORCE(gen);

            if (ros::service::call(sim.first + "/addForce", nmsg) &&
            (nmsg.response.result != -1)) {
                all_ok_mutex.lock();
                ++all_ok;
                all_ok_mutex.unlock();
            } else {
                ROS_WARN_STREAM("Cannot add force to " << sim.first);
            }
        }, msg));
    }
    std::for_each(ts.begin(), ts.end(), [](std::thread & t) {
        t.join();
    });
    ROS_INFO_STREAM("[AddForce] Time elapsed: " << ros::WallTime::now() - begin);
    res.result = all_ok;
    return true;
}

bool add_force_torque_callback(
    vrep_plugin_server::AddForceTorque::Request& req,
    vrep_plugin_server::AddForceTorque::Response& res) {
    ros::WallTime begin = ros::WallTime::now();
    int all_ok = 0;
    std::vector<std::thread> ts;
    std::mutex all_ok_mutex;
    vrep_plugin_server::AddForceTorque msg;
    msg.request = req;
    for (auto& sim : sims) {
        ts.push_back(
            std::thread([sim, &all_ok, &all_ok_mutex]
        (vrep_plugin_server::AddForceTorque nmsg) ->void {
            nmsg.request.wrench.force.x += NOISE_FORCE(gen);
            nmsg.request.wrench.force.y += NOISE_FORCE(gen);
            nmsg.request.wrench.force.z += NOISE_FORCE(gen);

            nmsg.request.wrench.torque.x += NOISE_POS(gen);
            nmsg.request.wrench.torque.y += NOISE_POS(gen);
            nmsg.request.wrench.torque.z += NOISE_POS(gen);

            if (ros::service::call(sim.first + "/addForceTorque", nmsg) &&
            (nmsg.response.result != -1) ) {
                all_ok_mutex.lock();
                ++all_ok;
                all_ok_mutex.unlock();
            } else {
                ROS_WARN_STREAM("Cannot add force/torque to " << sim.first);
            }
        }, msg));
    }

    std::for_each(ts.begin(), ts.end(), [](std::thread & t) {
        t.join();
    });
    ROS_INFO_STREAM("[AddForceTorque] Time elapsed: " <<
                    ros::WallTime::now() - begin);

    res.result = all_ok;
    return true;
}

// TODO: CALC mean and covariance from all poses returned by the sims

bool get_object_pose_callback(
    sim_architect::GetObjectPose::Request& req,
    sim_architect::GetObjectPose::Response& res) {
    std::vector<std::vector<double>> poses(6, std::vector<double>(0, 0));
    enum vecOrder {x = 0, y = 1, z = 2, rr = 3, pp = 4, yy = 5};
    vrep_common::simRosGetObjectPose msg;
    msg.request.handle = req.handle;
    msg.request.relativeToObjectHandle = req.relativeToObjectHandle;
    ros::WallTime begin = ros::WallTime::now();
    int all_ok = 0;
    std::vector<std::thread> ts;
    std::mutex data_mutex;
    for (auto& sim : sims) {
        ts.push_back(
            std::thread([sim, &all_ok, &data_mutex, &poses]
        (vrep_common::simRosGetObjectPose nmsg) ->void {
            if (ros::service::call(sim.first + "/simRosGetObjectPose", nmsg) &&
            (nmsg.response.result != -1) ) {
                Eigen::Quaternionf orientation(
                    nmsg.response.pose.pose.orientation.x,
                    nmsg.response.pose.pose.orientation.y,
                    nmsg.response.pose.pose.orientation.z,
                    nmsg.response.pose.pose.orientation.w);
                Eigen::Matrix<float, 3, 1> rpy =
                orientation.toRotationMatrix().eulerAngles(0, 1, 2);

                ROS_DEBUG_STREAM(
                    "XYZ:" <<
                    nmsg.response.pose.pose.position.x << " " <<
                    nmsg.response.pose.pose.position.y << " " <<
                    nmsg.response.pose.pose.position.z);
                ROS_DEBUG_STREAM(
                    "RPY:" <<
                    fmod((rpy(0) + _2PI) , _2PI) << " " <<
                    fmod((rpy(1) + _2PI) , _2PI) << " " <<
                    fmod((rpy(2) + _2PI) , _2PI));

                data_mutex.lock();
                poses[vecOrder::x].push_back(nmsg.response.pose.pose.position.x);
                poses[vecOrder::y].push_back(nmsg.response.pose.pose.position.y);
                poses[vecOrder::z].push_back(nmsg.response.pose.pose.position.z);
                poses[vecOrder::rr].push_back(fmod(double(rpy(0) + _2PI), _2PI));
                poses[vecOrder::pp].push_back(fmod(double(rpy(1) + _2PI), _2PI));
                poses[vecOrder::yy].push_back(fmod(double(rpy(2) + _2PI), _2PI));
                ++all_ok;
                data_mutex.unlock();
            } else {
                ROS_WARN_STREAM("Cannot get_object_pose at " << sim.first);
            }
        }, msg));
    }

    std::for_each(ts.begin(), ts.end(), [](std::thread & t) {
        t.join();
    });
    ROS_INFO_STREAM("[simRosGetObjectPose] Time elapsed: " <<
                    ros::WallTime::now() - begin);

    if (poses[0].empty()) {
        ROS_WARN_STREAM("No positions recorded.");
        return false;
    }

    // Covnert to Eigen Mat
    Eigen::MatrixXd mat(poses.size(), poses[0].size());
    for (int i = 0; i < poses.size(); i++)
        mat.row(i) = Eigen::VectorXd::Map(poses[i].data(), poses[i].size());
    mat.transposeInPlace();

    // Find mean
    Eigen::VectorXd mean = mat.colwise().mean();

    // Find variances and covariances
    if (sims.size() > 1 ) {
        Eigen::MatrixXd centered = mat.rowwise() - mat.colwise().mean();
        Eigen::MatrixXd cov = (centered.adjoint() * centered) /
                              double(mat.rows() - 1);
        // ROS_INFO_STREAM("Mat: " << std::endl << mat);
        // ROS_INFO_STREAM("Cov: " << std::endl << cov);
        const auto variances = cov.diagonal();
        res.pose_xyz_variance.x = variances(0);
        res.pose_xyz_variance.y = variances(1);
        res.pose_xyz_variance.z = variances(2);
        res.pose_rpy_variance.row = variances(3);
        res.pose_rpy_variance.pitch = variances(4);
        res.pose_rpy_variance.yaw = variances(5);
        for (int i = 0; i < cov.size(); ++i) {
            res.pose_covariance.push_back(cov.data()[i]);
        }
    } else {
        // Cannot calcualte with a single sample
        res.pose_xyz_variance.x = 0;
        res.pose_xyz_variance.y = 0;
        res.pose_xyz_variance.z = 0;
        res.pose_rpy_variance.row = 0;
        res.pose_rpy_variance.pitch = 0;
        res.pose_rpy_variance.yaw = 0;
        res.pose_covariance.push_back(0);
    }

    // Fill in mean values
    res.pose_mean.position.x = mean[0];
    res.pose_mean.position.y = mean[1];
    res.pose_mean.position.z = mean[2];
    Eigen::Quaterniond mean_quaternion = RPY2Quaternion(mean[3], mean[4], mean[5]);
    res.pose_mean.orientation.x = mean_quaternion.x();
    res.pose_mean.orientation.y = mean_quaternion.y();
    res.pose_mean.orientation.z = mean_quaternion.z();
    res.pose_mean.orientation.w = mean_quaternion.w();
    res.sample_size = all_ok;

    return true;
}

bool get_simulation_names_callback(
    sim_architect::GetSimulationNames::Request& req,
    sim_architect::GetSimulationNames::Response& res) {
    for (auto& sim : sims) {
        res.names.push_back(sim.first);
    }
    return true;
}


void simulationCleanUpTimerCallback(const ros::TimerEvent& e) {
    for (auto it = sims.cbegin(); it != sims.cend();) {
        if ((*it).second < e.last_real) {
            ROS_WARN_STREAM("Cleanning up sims: " << (*it).first <<
                            ". Due to not sending a heartbeat signal.");
            // TODO: maybe kill the process number as well? (and relaunch one?)
            sims.erase(it++);
        } else {
            ++it;
        }
    }
}

//
int main(int argc, char** argv) {
    // Set up ROS.
    ros::init(argc, argv, "sim_architect");
    node = new ros::NodeHandle("~");

    // Subscribers
    ros::Subscriber sub_message = node->subscribe("/simulators_heartbeat", 124,
                                                  sim_heartbeat_message_callback);

    // Advertisers
    get_number_of_simulations_service =
        node->advertiseService("getNumberOfSimulations",
                               get_number_of_simulations_callback);
    set_object_pose_service =  node->advertiseService("setObjectPose",
                                                      set_object_pose_callback);
    stop_simulation_service =  node->advertiseService("stopSimulation",
                                                      stop_simulation_callback);
    start_simulation_service =  node->advertiseService("startSimulation",
                                                       start_simulation_callback);
    add_force_service =  node->advertiseService("addForce",
                                                add_force_callback);
    add_force_torque_service =  node->advertiseService("addForceTorque",
                                                       add_force_torque_callback);
    get_object_pose_service =  node->advertiseService("getObjectPose",
                                                      get_object_pose_callback);
    get_simulation_names_service =  node->advertiseService("getSimulationNames",
                                                           get_simulation_names_callback);

    ROS_INFO_STREAM("All top level servicees and topics advertised!");

    ros::Timer sim_cleanup_timer =
        node->createTimer(ros::Duration(5), simulationCleanUpTimerCallback);

    ros::Rate r(30);
    while (node->ok()) {
        ros::spinOnce();
        r.sleep();
    }

    set_object_pose_service.shutdown();
    return 0;
}
