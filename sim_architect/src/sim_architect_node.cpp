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
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Vector3.h>
#include <vrep_plugin_server/AddForce.h>
#include <vrep_plugin_server/AddForceTorque.h>
#include <vrep_plugin_server/IsSceneStatic.h>
#include <vrep_plugin_server/ResetScene.h>
#include <vrep_common/simRosSetObjectPose.h>
#include <vrep_common/simRosGetObjectPose.h>
#include <vrep_common/simRosStopSimulation.h>
#include <vrep_common/simRosStartSimulation.h>
#include <vrep_common/simRosGetObjectHandle.h>

#include "sim_architect/GetNumberOfSimulations.h"
#include "sim_architect/GetObjectPose.h"
#include "sim_architect/GetSimulationNames.h"
#include "sim_architect/EvalForceTorque.h"

#include <Eigen/Geometry>
#include <Eigen/Core>

using namespace std;

const int WORLD_FRAME(-1);
double _2PI(2 * M_PI);
enum vecOrder {x = 0, y = 1, z = 2, rr = 3, pp = 4, yy = 5};

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
ros::ServiceServer eval_force_torque_service;
ros::ServiceServer reset_scenes_service;

// Random variation data
std::random_device rd;
std::mt19937 gen(rd());
std::default_random_engine gen2;
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

template <class Tv, class Trand>
void addNoiseV3(Tv& v, Trand& rand) {
    v.x += rand(gen);
    v.y += rand(gen);
    v.z += rand(gen);
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

            addNoiseV3(nmsg.request.pose.position, NOISE_POS);
            // nmsg.request.pose.position.x += NOISE_POS(gen);
            // nmsg.request.pose.position.y += NOISE_POS(gen);
            // nmsg.request.pose.position.z += NOISE_POS(gen);

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
            addNoiseV3(nmsg.request.position, NOISE_POS);
            // nmsg.request.position.x += NOISE_POS(gen);
            // nmsg.request.position.y += NOISE_POS(gen);
            // nmsg.request.position.z += NOISE_POS(gen);

            addNoiseV3(nmsg.request.force, NOISE_FORCE);
            // nmsg.request.force.x += NOISE_FORCE(gen);
            // nmsg.request.force.y += NOISE_FORCE(gen);
            // nmsg.request.force.z += NOISE_FORCE(gen);

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
            addNoiseV3(nmsg.request.wrench.force, NOISE_FORCE);
            // nmsg.request.wrench.force.x += NOISE_FORCE(gen);
            // nmsg.request.wrench.force.y += NOISE_FORCE(gen);
            // nmsg.request.wrench.force.z += NOISE_FORCE(gen);

            addNoiseV3(nmsg.request.wrench.torque, NOISE_TORQUE);
            // nmsg.request.wrench.torque.x += NOISE_TORQUE(gen);
            // nmsg.request.wrench.torque.y += NOISE_TORQUE(gen);
            // nmsg.request.wrench.torque.z += NOISE_TORQUE(gen);

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

bool get_all_poses(std::vector<std::vector<double>>& poses,
                   int handle, int relativeToObjectHandle) {
    poses = std::vector<std::vector<double>>(6, std::vector<double>(0, 0));
    vrep_common::simRosGetObjectPose msg;
    msg.request.handle = handle;
    msg.request.relativeToObjectHandle = relativeToObjectHandle;
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

    ROS_INFO_STREAM("[get_all_poses] Time elapsed: " <<
                    ros::WallTime::now() - begin);
    return (!poses.empty());
}


bool get_object_pose_callback(
    sim_architect::GetObjectPose::Request& req,
    sim_architect::GetObjectPose::Response& res) {
    std::vector<std::vector<double>> poses; //(6, std::vector<double>(0, 0));
    get_all_poses(poses, req.handle, req.relativeToObjectHandle);

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
    res.sample_size = poses[0].size();

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

bool eval_force_torque_callback(
    sim_architect::EvalForceTorque::Request& req,
    sim_architect::EvalForceTorque::Response& res) {
    ros::WallTime begin = ros::WallTime::now();
    std::vector<geometry_msgs::Pose> poses(sims.size());
    int all_ok = 0;
    std::vector<std::thread> ts;
    std::mutex data_mutex;
    int id = 0;
    res.poses = std::vector<geometry_msgs::PoseArray>(sims.size());
    res.wrench = std::vector<geometry_msgs::Wrench>(sims.size());
    for (auto& sim : sims) {
        ts.push_back(
            std::thread([sim, &all_ok, &data_mutex, &poses, &req, &res]
        (int i) ->void {
            // TODO: use parallel for
            // ResetScene
            vrep_plugin_server::ResetScene reset_msg;
            reset_msg.request.filename = req.reset_scene_file;
            reset_msg.request.start_scene = true;
            if (!ros::service::call(sim.first + "/resetScene", reset_msg)) {
                ROS_WARN_STREAM("Cannot reset scenes for " << sim.first);
                return;
            }

            ROS_INFO_STREAM("ResetScene for " + sim.first + ": " +
            ((reset_msg.response.result) ? "success" : "fail"));

            // Convert names to handles. Keep order.
            vrep_common::simRosGetObjectHandle handle_msg;
            vector<int> handles(req.names.size(), -1);
            int hid = 0;
            for (auto& name : req.names) {
                handle_msg.request.objectName = name;
                if (!ros::service::call(sim.first + "/simRosGetObjectHandle", handle_msg)) {
                    ROS_WARN_STREAM("Cannot get handle of " << name << " at " << sim.first);
                } else {
                    handles[hid] = handle_msg.response.handle;
                }
                ++hid;
            }
            ROS_INFO_STREAM("simRosGetObjectHandle: success");

            // Get handle for object with added force torue
            int aft_handle = -1;
            handle_msg.request.objectName = req.object_name;
            if (!ros::service::call(sim.first + "/simRosGetObjectHandle", handle_msg)) {
                ROS_WARN_STREAM("Cannot get handle of object " << req.object_name <<
                                " to apply forceTorque at " << sim.first);
                return;
            } else {
                aft_handle = handle_msg.response.handle;
            }

            // Add forceTorque - id is 'i'
            if (req.wrench.size() < i) {
                ROS_INFO_STREAM("No wrench was given to this simulation.");
                return;
            }
            switch (req.distribution_type) {
                case (sim_architect::EvalForceTorque::Request::UNIFORM): {
                    std::uniform_real_distribution<double> distr(
                        req.params[sim_architect::EvalForceTorque::Request::UNIFORM_MIN],
                        req.params[sim_architect::EvalForceTorque::Request::UNIFORM_MAX]);
                    //ROS_INFO_STREAM("Created a noise distribution" << req.wrench[i].force);
                    addNoiseV3(req.wrench[i].force, distr);
                    //ROS_INFO_STREAM("Added to force");
                    // addNoiseV3(req.wrench[i].torque, distr);
                    //ROS_INFO_STREAM("Added to torque");
                    ROS_WARN_STREAM("Not applying any noise on TORQUE.");
                    break;
                }
                case (sim_architect::EvalForceTorque::Request::GAUSSIAN): {
                    std::normal_distribution<> distr(
                        req.params[sim_architect::EvalForceTorque::Request::GAUSSIAN_MEAN],
                        req.params[sim_architect::EvalForceTorque::Request::GAUSSIAN_VAR]);
                    addNoiseV3(req.wrench[i].force, distr);
                    // addNoiseV3(req.wrench[i].torque, distr);
                    ROS_WARN_STREAM("Not applying any noise on TORQUE.");
                    break;
                }
                default:
                    ROS_WARN_STREAM("Wrong distribution type!");
                    return;
            }
            ROS_INFO_STREAM("Adding noise: success");

            vrep_plugin_server::AddForceTorque force_t_msg;
            force_t_msg.request.handle = aft_handle;
            force_t_msg.request.wrench = req.wrench[i];
            if (!ros::service::call(sim.first + "/addForceTorque", force_t_msg)) {
                ROS_WARN_STREAM("Cannot add force torque!");
                return;
            }
            res.wrench[i] = force_t_msg.request.wrench;
            ROS_INFO_STREAM("AddForceTorque: success");

            // Wait while scene(i) is static
            vrep_plugin_server::IsSceneStatic static_msg;
            static_msg.request.max_speed = 0.001;
            while (ros::service::call(sim.first + "/isSceneStatic", static_msg) && !static_msg.response.is_static) {
                ros::Duration(0.005).sleep(); // 5 ms.
                ROS_INFO_STREAM("Scene still not static!");
            }

            // Get object poses
            vrep_common::simRosGetObjectPose pose_msg;
            hid = 0;
            res.poses[i].poses = std::vector<geometry_msgs::Pose>(handles.size());
            res.poses[i].header.frame_id = sim.first;
            for (auto& handle : handles) {
                if (handle == -1) {++hid; continue;}
                pose_msg.request.handle = handle;
                pose_msg.request.relativeToObjectHandle = WORLD_FRAME;
                if (ros::service::call(sim.first + "/simRosGetObjectPose", pose_msg) &&
                        (pose_msg.response.result != -1) ) {
                    data_mutex.lock();
                    res.poses[i].poses[hid] = pose_msg.response.pose.pose;
                    ++all_ok;
                    data_mutex.unlock();
                } else {
                    ROS_WARN_STREAM("Cannot get_object_pose at " << sim.first);
                }
                ++hid;
            }
            ROS_INFO_STREAM("simRosGetObjectPose: success");
        }, id++));
    }

    std::for_each(ts.begin(), ts.end(), [](std::thread & t) {
        t.join();
    });

    ROS_INFO_STREAM("[EvalForceTorque] Time elapsed: " <<
                    ros::WallTime::now() - begin);
    return true;
}

bool reset_scene_callback(vrep_plugin_server::ResetScene::Request& req,
                          vrep_plugin_server::ResetScene::Response& res) {
    ros::WallTime begin = ros::WallTime::now();
    int all_ok = 0;
    std::vector<std::thread> ts;
    std::mutex data_mutex;
    int id = 0;
    vrep_plugin_server::ResetScene reset_msg;
    reset_msg.request = req; // Copy request msg
    for (auto& sim : sims) {
        ts.push_back(
            std::thread([sim, &all_ok, &data_mutex, &reset_msg, &req, &res]
        (int i) ->void {
            // ResetScene
            if (!ros::service::call(sim.first + "/resetScene", reset_msg)) {
                ROS_WARN_STREAM("Cannot reset scenes for " << sim.first);
                return;
            }
            data_mutex.lock();
            all_ok += (reset_msg.response.result ? 1 : 0);
            data_mutex.unlock();
            ROS_INFO_STREAM("ResetScene for " + sim.first + ": " +
            ((reset_msg.response.result) ? "success" : "fail"));
        }, id++));
    }

    std::for_each(ts.begin(), ts.end(), [](std::thread & t) {
        t.join();
    });

    res.result = (all_ok == sims.size());
    ROS_INFO_STREAM("[ResetScene] Time elapsed: " <<
                    ros::WallTime::now() - begin);
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
    eval_force_torque_service = node->advertiseService("evalForceTorque",
                                                       eval_force_torque_callback);
    reset_scenes_service = node->advertiseService("resetScene",
                                                  reset_scene_callback);
    ROS_INFO_STREAM("All top level services and topics advertised!");

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
