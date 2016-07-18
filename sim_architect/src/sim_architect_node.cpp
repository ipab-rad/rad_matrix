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
#include <std_srvs/Trigger.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Vector3.h>
#include <vrep_plugin_server/AddForce.h>
#include <vrep_plugin_server/AddForceTorque.h>
#include <vrep_plugin_server/PushObject.h>
#include <vrep_plugin_server/IsSceneStatic.h>
#include <vrep_plugin_server/ResetScene.h>
#include <vrep_plugin_server/AreCubesSplit.h>
#include <vrep_plugin_server/ActionA.h>
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
ros::ServiceServer stop_simulations_service;
ros::ServiceServer start_simulations_service;
ros::ServiceServer are_scenes_static_service;
ros::ServiceServer add_force_service;
ros::ServiceServer push_object_service;
ros::ServiceServer add_force_torque_service;
ros::ServiceServer get_object_pose_service;
ros::ServiceServer get_simulation_names_service;
ros::ServiceServer eval_force_torque_service;
ros::ServiceServer reset_scenes_service;
ros::ServiceServer action_A_service;
ros::ServiceServer action_x_service;
ros::ServiceServer action_y_service;
ros::ServiceServer action_z_service;
ros::ServiceServer split_service;
ros::ServiceServer success_action_x_service;

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

template <typename T> int sgn(T val) {
    return (T(0) < val) - (val < T(0));
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
        ts.push_back(std::thread([sim, &all_ok, &all_ok_mutex]
        (vrep_common::simRosSetObjectPose nmsg) ->void {
            addNoiseV3(nmsg.request.pose.position, NOISE_POS);

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
    std::for_each(ts.begin(), ts.end(), [](std::thread & t) { t.join(); });

    ROS_INFO_STREAM("[simRosSetObjectPose] Time elapsed: " <<
                    ros::WallTime::now() - begin);
    res.result = all_ok;
    return true;
}

bool stop_simulations_callback(vrep_common::simRosStopSimulation::Request& req,
                               vrep_common::simRosStopSimulation::Response& res) {
    ros::WallTime begin = ros::WallTime::now();
    int all_ok = 0;
    std::vector<std::thread> ts;
    std::mutex all_ok_mutex;
    vrep_common::simRosStopSimulation msg;
    for (auto& sim : sims) {
        ts.push_back(std::thread([sim, &all_ok, &all_ok_mutex]
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
    std::for_each(ts.begin(), ts.end(), [](std::thread & t) { t.join(); });

    ROS_INFO_STREAM("[simRosStopSimulations] Time elapsed: " <<
                    ros::WallTime::now() - begin);
    res.result = all_ok;
    return true;
}

bool start_simulations_callback(vrep_common::simRosStartSimulation::Request&
                                req,
                                vrep_common::simRosStartSimulation::Response& res) {
    ros::WallTime begin = ros::WallTime::now();
    int all_ok = 0;
    std::vector<std::thread> ts;
    std::mutex all_ok_mutex;
    vrep_common::simRosStartSimulation msg;
    for (auto& sim : sims) {
        ts.push_back(std::thread([sim, &all_ok, &all_ok_mutex]
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
    std::for_each(ts.begin(), ts.end(), [](std::thread & t) { t.join(); });

    ROS_INFO_STREAM("[simRosStartSimulations] Time elapsed: " <<
                    ros::WallTime::now() - begin);
    res.result = all_ok;
    return true;
}

bool are_scenes_static_callback(
    vrep_plugin_server::IsSceneStatic::Request& req,
    vrep_plugin_server::IsSceneStatic::Response& res) {
    ros::WallTime begin = ros::WallTime::now();
    int all_ok = 0;
    std::vector<std::thread> ts;
    std::mutex all_ok_mutex;
    vrep_plugin_server::IsSceneStatic msg;
    msg.request = req;
    for (auto& sim : sims) {
        ts.push_back(std::thread([sim, &all_ok, &all_ok_mutex]
        (vrep_plugin_server::IsSceneStatic nmsg) ->void {
            ROS_DEBUG_STREAM("Checking scene static on: " << sim.first);
            if (ros::service::call(sim.first + "/isSceneStatic", nmsg) &&
            (nmsg.response.is_static)) {
                all_ok_mutex.lock();
                ++all_ok;
                all_ok_mutex.unlock();
            } else {
                ROS_INFO_STREAM("Scene is not static - " << sim.first);
            }
        }, msg));
    }
    std::for_each(ts.begin(), ts.end(), [](std::thread & t) {t.join(); });
    ROS_INFO_STREAM("[AreScenesStatic] Time elapsed: " <<
                    ros::WallTime::now() - begin);
    res.is_static = (all_ok == sims.size());
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
        ts.push_back(std::thread([sim, &all_ok, &all_ok_mutex]
        (vrep_plugin_server::AddForce nmsg) ->void {
            addNoiseV3(nmsg.request.position, NOISE_POS);
            addNoiseV3(nmsg.request.force, NOISE_FORCE);

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
    std::for_each(ts.begin(), ts.end(), [](std::thread & t) { t.join(); });

    ROS_INFO_STREAM("[AddForce] Time elapsed: " << ros::WallTime::now() - begin);
    res.result = all_ok;
    return true;
}

bool push_object_callback(
    vrep_plugin_server::PushObject::Request& req,
    vrep_plugin_server::PushObject::Response& res) {
    ros::WallTime begin = ros::WallTime::now();
    int all_ok = 0;
    std::vector<std::thread> ts;
    std::mutex all_ok_mutex;
    vrep_plugin_server::PushObject msg;
    msg.request = req;
    for (auto& sim : sims) {
        ts.push_back(std::thread([sim, &all_ok, &all_ok_mutex]
        (vrep_plugin_server::PushObject nmsg) ->void {
            addNoiseV3(nmsg.request.force_at_iter, NOISE_FORCE);
            // addNoiseV3(nmsg.request.wrench_at_iter.torque, NOISE_TORQUE);

            if (ros::service::call(sim.first + "/pushObject", nmsg) &&
            (nmsg.response.result != -1)) {
                all_ok_mutex.lock();
                ++all_ok;
                all_ok_mutex.unlock();
            } else {
                ROS_WARN_STREAM("Cannot push object at " << sim.first);
            }
        }, msg));
    }
    std::for_each(ts.begin(), ts.end(), [](std::thread & t) { t.join(); });

    ROS_INFO_STREAM("[PushObject] Time elapsed: " << ros::WallTime::now() - begin);
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
        ts.push_back(std::thread([sim, &all_ok, &all_ok_mutex]
        (vrep_plugin_server::AddForceTorque nmsg) ->void {
            addNoiseV3(nmsg.request.wrench.force, NOISE_FORCE);
            addNoiseV3(nmsg.request.wrench.torque, NOISE_TORQUE);

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

    std::for_each(ts.begin(), ts.end(), [](std::thread & t) { t.join(); });

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
        ts.push_back(std::thread([sim, &all_ok, &data_mutex, &poses]
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

    std::for_each(ts.begin(), ts.end(), [](std::thread & t) { t.join(); });

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
        // Cannot calculate with a single sample
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
        ts.push_back(std::thread([sim, &all_ok, &data_mutex, &poses, &req, &res]
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

            // Get handle for object with added force torque
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
                        (pose_msg.response.result != -1)) {
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

    std::for_each(ts.begin(), ts.end(), [](std::thread & t) { t.join(); });

    ROS_INFO_STREAM("[EvalForceTorque] Time elapsed: " <<
                    ros::WallTime::now() - begin);
    return true;
}

bool reset_scenes_callback(vrep_plugin_server::ResetScene::Request& req,
                           vrep_plugin_server::ResetScene::Response& res) {
    ros::WallTime begin = ros::WallTime::now();
    int all_ok = 0;
    std::vector<std::thread> ts;
    std::mutex data_mutex;
    int id = 0;
    vrep_plugin_server::ResetScene reset_msg;
    reset_msg.request = req; // Copy request msg
    for (auto& sim : sims) {
        ts.push_back(std::thread([sim, &all_ok, &data_mutex, &reset_msg, &req, &res]
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

    std::for_each(ts.begin(), ts.end(), [](std::thread & t) { t.join(); });

    res.result = (all_ok == sims.size());
    ROS_INFO_STREAM("[ResetScenes] Time elapsed: " <<
                    ros::WallTime::now() - begin);
    return true;
}

bool execute_action_on_sims(vrep_plugin_server::PushObject& rmsg,
                            vrep_plugin_server::ActionA::Request& req,
                            vrep_plugin_server::ActionA::Response& res) {
    ros::WallTime begin = ros::WallTime::now();
    int all_ok = 0;
    int failed = 0;
    std::vector<std::thread> ts;
    std::mutex all_ok_mutex;

    for (auto& sim : sims) {
        ts.push_back(std::thread([sim, &all_ok, &all_ok_mutex, &failed]
        (vrep_plugin_server::PushObject nmsg, int cube_id) ->void {
            // Get handle
            vrep_common::simRosGetObjectHandle handle_msg;
            do{
                handle_msg.request.objectName = "cube" + std::to_string(cube_id);

                if (ros::service::call(sim.first + "/simRosGetObjectHandle", handle_msg) &&
                (handle_msg.response.handle != -1)) {
                    nmsg.request.handle = handle_msg.response.handle;
                } else {
                    ROS_WARN_STREAM("Cannot get handle of '" <<
                    handle_msg.request.objectName << "' at " << sim.first);
                    all_ok_mutex.lock();
                    ++failed;
                    all_ok_mutex.unlock();
                    return; // The sim cannot perform any action if there are no cubes
                }
            } while (handle_msg.response.handle == -1);

            ROS_DEBUG_STREAM("[Action_] MSG: " << nmsg.request);

            // applying push maneuverer
            if (ros::service::call(sim.first + "/pushObject", nmsg) &&
            (nmsg.response.result > 0)) {
                all_ok_mutex.lock();
                ++all_ok;
                all_ok_mutex.unlock();
            } else {
                ROS_WARN_STREAM("Cannot pushObject on " << sim.first);
            }
        }, rmsg, req.cube_id));
    }

    std::for_each(ts.begin(), ts.end(), [](std::thread & t) { t.join(); });
    ROS_INFO_STREAM("[Action_] Time elapsed: " << ros::WallTime::now() - begin);

    if (failed == sims.size()) {
        ROS_ERROR_STREAM("All simulations failed. Cannot execute service (action_on_sim)!");
        return false;
    }
    res.success = (all_ok == sims.size());
    res.message = std::to_string(all_ok);

    return res.success;
}

bool action_x_callback(std_srvs::Trigger::Request& req,
                       std_srvs::Trigger::Response& res) {
    vrep_plugin_server::PushObject msg;
    msg.request.force_at_iter.x = 0.3;
    msg.request.duration = 0.05;

    // execute_action_on_sims(msg, req, res);

    return true;
}

bool action_y_callback(std_srvs::Trigger::Request& req,
                       std_srvs::Trigger::Response& res) {
    vrep_plugin_server::PushObject msg;
    msg.request.force_at_iter.y = 0.35;
    msg.request.duration = 0.08;

    // execute_action_on_sims(msg, req, res);

    return true;
}
bool action_z_callback(std_srvs::Trigger::Request& req,
                       std_srvs::Trigger::Response& res) {
    // vrep_plugin_server::ActionA r;

    // execute_action_on_sims(msg, req, res);

    return true;
}

bool action_A_callback(vrep_plugin_server::ActionA::Request& req,
                       vrep_plugin_server::ActionA::Response& res) {
    vrep_plugin_server::PushObject msg;
    float force_magnitude = 0.06;
    switch (req.direction) {
        case vrep_plugin_server::ActionA::Request::DIR_X:
        case vrep_plugin_server::ActionA::Request::DIR_nX:
            msg.request.force_at_iter.x = sgn(req.direction) * force_magnitude;
            break;
        case vrep_plugin_server::ActionA::Request::DIR_Y:
        case vrep_plugin_server::ActionA::Request::DIR_nY:
            msg.request.force_at_iter.y = sgn(req.direction) * force_magnitude;
            break;
        case vrep_plugin_server::ActionA::Request::DIR_Z:
        case vrep_plugin_server::ActionA::Request::DIR_nZ:
            msg.request.force_at_iter.z = sgn(req.direction) * force_magnitude;
            break;
        default:
            ROS_WARN_STREAM("Wrong direction!");
            return false;
    }

    msg.request.position = req.offset;
    msg.request.duration = 0.02; // deltaT is 0.01
    execute_action_on_sims(msg, req, res);
    return true;
}


float all_split_cubes(vrep_plugin_server::AreCubesSplit::Request& req) {
    ros::WallTime begin = ros::WallTime::now();
    int MAX_CUBE_COUNT = req.cube_count;
    double DIST_THR = req.min_distance;

    vrep_plugin_server::AreCubesSplit split_msg;
    split_msg.request.min_distance = DIST_THR;
    split_msg.request.cube_count = MAX_CUBE_COUNT;

    int all_ok = 0;
    std::vector<std::thread> ts;
    std::mutex all_ok_mutex;

    for (auto& sim : sims) {
        ts.push_back(std::thread([sim, &all_ok, &all_ok_mutex]
        (vrep_plugin_server::AreCubesSplit s_msg) ->void {
            if (ros::service::call(sim.first + "/areCubesSplit", s_msg)) {
                if (s_msg.response.are_split) {
                    all_ok_mutex.lock();
                    ++all_ok;
                    all_ok_mutex.unlock();
                } else {
                    ROS_INFO_STREAM("Cubes not split on: " << sim.first);
                }
            } else {
                ROS_WARN_STREAM("Cannot check cubes static-ness on " << sim.first);
            }
        }, split_msg));
    }

    std::for_each(ts.begin(), ts.end(), [](std::thread & t) { t.join(); });
    ROS_INFO_STREAM("[AreCubesSplit] Time elapsed: " <<
                    ros::WallTime::now() - begin);
    return float(all_ok) / sims.size();
}

bool split_callback(vrep_plugin_server::AreCubesSplit::Request& req,
                    vrep_plugin_server::AreCubesSplit::Response& res) {
    float c = all_split_cubes(req);

    res.are_split = (c > .9);
    res.ratio = c;

    return true;
}

bool halt_until_scenes_static() {
    vrep_plugin_server::IsSceneStatic scene_static;
    scene_static.request.max_speed = 0.0001;
    int scale = 1;
    do {
        ros::Duration(log(scale * scale) * 0.05).sleep();
        are_scenes_static_callback(scene_static.request, scene_static.response);
        ++scale;
    } while (!scene_static.response.is_static);

    return scene_static.response.is_static;
}

// enum Action { X, Y, Z, nX, nY, nZ };
bool success_action_all_callback(vrep_plugin_server::ActionA::Request& req,
                                 vrep_plugin_server::ActionA::Response& res) {
    ros::WallTime begin = ros::WallTime::now();
    ROS_INFO_STREAM("---");
    halt_until_scenes_static();

    // Trigger the action
    action_A_callback(req, res);

    // Wait until scene is static
    halt_until_scenes_static();

    // Check if cubes are split
    if (res.success) {
        vrep_plugin_server::AreCubesSplit split_trig;
        split_trig.request.min_distance = 0.026; //2.6cm
        split_trig.request.cube_count = 3;

        split_callback(split_trig.request, split_trig.response);

        ROS_INFO_STREAM("Action A: " << req.direction << " : " <<
                        ((split_trig.response.are_split) ? "success" : "fail") <<
                        " with success rate of " << split_trig.response.ratio);
        res.success = split_trig.response.are_split;
        res.message = "Success rate of " + std::to_string(split_trig.response.ratio);
    } else {
        ROS_WARN_STREAM("Cannot execute action A");
        res.success = false;
        res.message = "Cannot execute action A";
    }
    ROS_INFO_STREAM("[SuccessActionA] Time elapsed: " <<
                    ros::WallTime::now() - begin);
    ROS_INFO_STREAM("---");
    return true;


}

// bool success_action_x_callback(std_srvs::Trigger::Request& req,
//                                std_srvs::Trigger::Response& res) {
//     return success_action_all_callback(vrep_plugin_server::ActionA::Request::DIR_X,
//                                        req, res);
// }


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
    stop_simulations_service =  node->advertiseService("stopSimulations",
                                                       stop_simulations_callback);
    start_simulations_service =  node->advertiseService("startSimulations",
                                                        start_simulations_callback);
    are_scenes_static_service = node->advertiseService("areScenesStatic",
                                                       are_scenes_static_callback);
    add_force_service =  node->advertiseService("addForce",
                                                add_force_callback);
    push_object_service = node->advertiseService("pushObject",
                                                 push_object_callback);
    add_force_torque_service =  node->advertiseService("addForceTorque",
                                                       add_force_torque_callback);
    get_object_pose_service =  node->advertiseService("getObjectPose",
                                                      get_object_pose_callback);
    get_simulation_names_service =  node->advertiseService("getSimulationNames",
                                                           get_simulation_names_callback);
    eval_force_torque_service = node->advertiseService("evalForceTorque",
                                                       eval_force_torque_callback);
    reset_scenes_service = node->advertiseService("resetScenes",
                                                  reset_scenes_callback);
    action_A_service = node->advertiseService("action_A",
                                              action_A_callback);
    action_x_service = node->advertiseService("action_x",
                                              action_x_callback);
    action_y_service = node->advertiseService("action_y",
                                              action_y_callback);
    action_z_service = node->advertiseService("action_z",
                                              action_z_callback);
    split_service = node->advertiseService("areCubesSplit",
                                           split_callback);
    success_action_x_service = node->advertiseService("success_action_all",
                                                      success_action_all_callback);
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
