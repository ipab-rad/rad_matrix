#include "../include/vrep_plugin_server/ros_server.h"
#include "../include/v_repLib.h"

#include <random>
#include <chrono>
#include <unistd.h> // get pid
#include <fstream>

#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/distortion_models.h>
#include <image_transport/image_transport.h>
#include <geometry_msgs/PointStamped.h>

#include <Eigen/Geometry>

using namespace ros;
using namespace std;

bool ROSServer::initialize() {
    int argc = 0;
    char** argv = NULL;
    ros::init(argc, argv, "vrep_" + std::to_string(getpid()));

    if (!ros::master::check()) {
        ROS_ERROR("NO ROS MASTER!");
        return false;
    }
    node = new ros::NodeHandle("~");

    ROS_INFO_STREAM("STARTED THE ROS NODE with name: " <<
                    ros::this_node::getName());

    ROS_INFO("Advertising...");
    it = new image_transport::ImageTransport(*node);
    cam_pub = it->advertiseCamera("rgb/image_raw", 1);
    simulation_up_pub =
        node->advertise<std_msgs::String>("/simulators_heartbeat", 5);

    ROS_INFO_STREAM("Finished advertising.");

    add_shape_service =
        node->advertiseService("createPureShape",
                               &ROSServer::add_object_callback, this);
    set_colour_shape_service =
        node->advertiseService("setShapeColour",
                               &ROSServer::set_object_colour_callback, this);
    add_force_service =
        node->advertiseService("addForce",
                               &ROSServer::add_force_callback, this);
    add_force_torque_service =
        node->advertiseService("addForceTorque",
                               &ROSServer::add_force_torque_callback, this);
    is_scene_static_service =
        node->advertiseService("isSceneStatic",
                               &ROSServer::is_scene_static_callback, this);
    reset_scene_service =
        node->advertiseService("resetScene",
                               &ROSServer::reset_scene_callback, this);
    ROS_INFO_STREAM("Finished with server reconf.");




    // Load scene
    this->loadScene();
    //this->startSim();

    heartbeat_timer =
        node->createTimer(ros::Duration(0.5),
                          &ROSServer::simulationUpTimerCallback, this);

    return (true);
}

void ROSServer::loadScene() {
    if (simLoadScene(SCENE_PATH) == -1) {
        ROS_ERROR("Cannot load scene! Sleeping and attempting a final time...");
        ros::Duration(1.5).sleep();
        if (simLoadScene(SCENE_PATH) == -1) {
            ROS_ERROR("Cannot load scene! Stopping instance.");
            this->shutDown();
            system(("kill " + std::to_string(getpid())).c_str());
        }
    } else
        ROS_INFO("Loaded scene.");
}

void ROSServer::startSim() {
    this->simulationAboutToStart();

    // Start Sim
    ROS_INFO("Starting simulation... (%d/%d)", sim_iter, MAX_SIMS_COUNT);
    int st_res = simStartSimulation();
    if (st_res == -1) {
        ROS_ERROR("Cannot start scene!");
        ros::Duration(1.5).sleep();
        if (simStartSimulation() == -1) {
            ROS_ERROR("Second attempt at starting failed.");
        }
    } else {
        // Increase iteration counter
        ++sim_iter;
        ROS_INFO("Started scene (code: %d).", st_res);
        // Need to get the handlers, as starting it manually calls
    }
}

std::vector<int> ROSServer::getShapes() {
    ROS_DEBUG_STREAM(sim_object_shape_type << " " << sim_handle_all);
    std::vector<int> handles;
    int i = 0;
    do {
        int h = simGetObjects(i++, sim_object_shape_type); // or sim_handle_all
        ROS_DEBUG_STREAM("Object[" << i - 1 << "]=" << h);
        if (h != -1) {
            handles.push_back(h);
        } else {
            break;
        }
    } while (true);
    return handles;
}

bool ROSServer::isSceneStatic(float max_speed) {
    std::vector<int> handles = getShapes();
    for (const int& h : handles) {
        float linearVelocity[3], angularVelocity[3];
        int r =  simGetVelocity(h, linearVelocity, angularVelocity);
        if (r == -1)
            continue;
        for (const float& lv : linearVelocity)
            if (lv > max_speed)
                return false;
        for (const float& av : linearVelocity)
            if (av > max_speed)
                return false;
    }
    return true;
}

void ROSServer::simulationUpTimerCallback(const ros::TimerEvent& e) {
    std_msgs::StringPtr str(new std_msgs::String);
    str->data = ros::this_node::getName();
    simulation_up_pub.publish(str);
}

bool ROSServer::add_object_callback(
    vrep_plugin_server::AddShape::Request& request,
    vrep_plugin_server::AddShape::Response& response) {

    ROS_INFO_STREAM("Received a request for type:" << int(request.shape.type));
    if (request.shape.type == 0) {
        ROS_WARN("Please check ROS shape ID (id > 0)! "
                 "(as in http://docs.ros.org/indigo/api/shape_msgs/html/msg/SolidPrimitive.html)");
        return false;
    }

    int options = 0;
    options |= 1 << 1; // Edges visible
    options |= 1 << 3; // Shape is respondable
    float dims[] = {float(request.shape.dimensions[0]),
                    float(request.shape.dimensions[1]),
                    float(request.shape.dimensions[2])
                   };
    int handle =
        simCreatePureShape(
            request.shape.type - 1, // Currently ROS shapes are offset by 1 from V-Rep ones
            options,
            dims,
            request.mass,
            NULL); // Precision (number of sides) for cylinder or sphere - default
    if (handle != -1) {
        if (simSetObjectName(handle, request.name.c_str()) == -1) {
            ROS_WARN("Cannot set name!");
        }
        simSetObjectSpecialProperty(handle,
                                    sim_objectspecialproperty_renderable |
                                    sim_objectspecialproperty_collidable |
                                    sim_objectspecialproperty_detectable_all);
        response.handle = handle; // If use name simGetObjectHandle(request.name);
        return true;
    } else {
        ROS_WARN_STREAM("Cannot create a pure shape!");
        return false;
    }

}

bool ROSServer::set_object_colour_callback(
    vrep_plugin_server::SetShapeColour::Request& request,
    vrep_plugin_server::SetShapeColour::Response& response) {
    float clr[] = {request.colour.r, request.colour.g, request.colour.b};
    response.success = simSetShapeColor(request.handle,
                                        NULL,
                                        sim_colorcomponent_ambient_diffuse,
                                        clr);
    return true;
}

bool ROSServer::add_force_callback(
    vrep_plugin_server::AddForce::Request& request,
    vrep_plugin_server::AddForce::Response& response) {
    float position[] = {
        float(request.position.x),
        float(request.position.y),
        float(request.position.z)
    };
    float force[] = {
        float(request.force.x),
        float(request.force.y),
        float(request.force.z)
    };
    int result = simAddForce(request.handle, position, force);
    if (result == -1) {
        ROS_WARN("Cannot apply force!");
        return false;
    }
    response.result = result;
    return true;
}

bool ROSServer::add_force_torque_callback(
    vrep_plugin_server::AddForceTorque::Request& request,
    vrep_plugin_server::AddForceTorque::Response& response) {
    float force[] = {
        float(request.wrench.force.x),
        float(request.wrench.force.y),
        float(request.wrench.force.z)
    };
    float torque[] = {
        float(request.wrench.torque.x),
        float(request.wrench.torque.y),
        float(request.wrench.torque.z)
    };
    int result = simAddForceAndTorque(request.handle, force, torque);
    if (result == -1) {
        ROS_WARN("Cannot apply force/torque!");
        return false;
    }
    response.result = result;
    return true;
}

bool ROSServer::is_scene_static_callback(
    vrep_plugin_server::IsSceneStatic::Request& request,
    vrep_plugin_server::IsSceneStatic::Response& response) {
    response.is_static = isSceneStatic(request.max_speed);
    return true;
}

bool ROSServer::reset_scene_callback(
    vrep_plugin_server::ResetScene::Request& request,
    vrep_plugin_server::ResetScene::Response& response) {
    std::vector<std::string> instr;
    if (!request.filename.empty()) {
        readFileToString(request.filename, instr);
    } else if (!request.instructions.empty()) {
        instr = request.instructions;
    } else {
        readFileToString(DEF_SETTINGS_PATH, instr);
    }

    if (request.reload_scene) {
        if (simStopSimulation() == -1 || simCloseScene() == -1) {
            ROS_WARN_STREAM("Cannot stop to reload scene!");
            response.result = false;
            return true;
        }
        loadScene();
    }
    response.result = parseInstructions(instr);
    if (request.start_scene) {
        response.result &= (simStartSimulation() != -1);
    }
    return true;
}

bool ROSServer::parseInstructions(const std::vector<std::string>& instr) {
    // Reset the dynamics of the scene so far (won't move objects to initial position)
    if (simResetDynamicObject(sim_handle_all) == -1) {
        ROS_WARN_STREAM("Cannot reset dynamics of the objects!");
        return false;
    }

    for (auto i : instr) {
        size_t arg_op = i.find("[");
        size_t arg_end = i.find("]");
        std::string name = i.substr(0, arg_op);
        ROS_INFO_STREAM("name: " << name);

        std::replace(i.begin(), i.end(), ',', ' ');
        std::stringstream fs(i.substr(arg_op + 1, arg_end - 1));
        int shape;
        float x, y, z, xx, yy, zz, ww, height, width, depth, m, red, green, blue;
        ROS_INFO_STREAM("Details: " << i.substr(arg_op + 1, arg_end - 1).c_str());
        fs >> shape >>
           x >> y >> z >>
           xx >> yy >> zz >> ww >>
           height >> width >> depth >>
           m >>
           red >> green >> blue;
        // TODO: Add noise to all the dimensions
        ROS_DEBUG_STREAM("f: "  << shape << " " << x << " " << y << " " << z << " " <<
                         xx << " " << yy << " " << zz << " " << ww << " " <<
                         height << " " << width << " " << depth << " " <<
                         m << " " <<
                         red << " " << green << " " << blue);
        // Remove shape if it exists
        int handle = simGetObjectHandle(name.c_str());
        if (handle != -1) {
            if (simRemoveObject(handle) == -1) {
                ROS_WARN_STREAM("Cannot remove object " << name);
                return false;
            }
        }

        // Create shape
        int options = 0;
        options |= 1 << 1; // Edges visible
        options |= 1 << 3; // Shape is respondable
        float dims[] = {height, width, depth};
        handle =
            simCreatePureShape(
                shape - 1, // Currently ROS shapes are offset by 1 from V-Rep ones
                options, dims, m,
                NULL); // Precision (number of sides) for cylinder or sphere - default
        if (handle == -1) {
            ROS_WARN_STREAM("Cannot create shape");
            return false;
        }
        ROS_INFO_STREAM("Handle is: " << handle);

        if (simSetObjectName(handle, name.c_str()) == -1) {
            ROS_WARN("Cannot set name!");
            return false;
        }
        if (simSetObjectSpecialProperty(handle,
                                        sim_objectspecialproperty_renderable |
                                        sim_objectspecialproperty_collidable |
                                        sim_objectspecialproperty_detectable_all) == -1) {
            ROS_WARN("Cannot set special properties!");
            return false;
        }

        // Move shape
        float p[3] = {x, y, z};
        float q[4] = {xx, yy, zz, ww};
        if ((simSetObjectPosition(handle, WORLD_FRAME, p) == -1) ||
                (simSetObjectQuaternion(handle, WORLD_FRAME, q) == -1)) {
            ROS_WARN_STREAM("Cannot set position or orientation!");
            return false;
        }

        // Change colour
        float clr[] = {red / 255.f, green / 255.f, blue / 255.f};
        if (simSetShapeColor(handle,
                             NULL,
                             sim_colorcomponent_ambient_diffuse,
                             clr) == -1) {
            ROS_WARN_STREAM("Cannot set colour!");
            return false;
        }
    }

    return true;
}

bool ROSServer::readFileToString(const std::string file,
                                 std::vector<std::string>& data) {
    data.clear();
    std::ifstream f(file);
    std::string line;
    std::string comment("#");
    while (std::getline(f, line)) {
        if (line.empty() || line.compare(0, comment.length(), comment) == 0)
            continue;
        data.push_back(line);
    }
    return !data.empty();
}

void ROSServer::shutDown() {
    // Disable the publishers:
    cam_pub.shutdown();
    simulation_up_pub.shutdown();
    add_shape_service.shutdown();
    set_colour_shape_service.shutdown();
    add_force_service.shutdown();
    add_force_torque_service.shutdown();

    // Clean up parameters
    ros::param::del("~");

    // Shut down:
    ros::shutdown();
}

void ROSServer::instancePass() {
    // When simulation is not running, we "spinOnce" here:
    int simState = simGetSimulationState();
    if ((simState & sim_simulation_advancing) == 0) {
        spinOnce();
    }
}

void ROSServer::mainScriptAboutToBeCalled() {
    // When simulation is running, we "spinOnce" here:
    spinOnce();

    // Get cube position
    float pos[3];
    simGetObjectPosition(cube_handle, WORLD_FRAME, pos);
    ROS_DEBUG("Cube position: x: %f, y: %f, z: %f", pos[0], pos[1], pos[2]);

    if (cam_pub.getNumSubscribers() > 0) {
        sensor_msgs::Image frame;
        static sensor_msgs::CameraInfo ci;
        getImageMessage(frame, ci);
        cam_pub.publish(frame, ci);
    }
}

void ROSServer::simulationAboutToStart() {
    ROS_INFO("Grab handles");
    cube_handle = simGetObjectHandle("cube_0");
    simcam_handle = simGetObjectHandle("simcam");
    ROS_INFO_STREAM("[V-REP_ROS_SIM] Cube handle: " << cube_handle);
    ROS_INFO_STREAM("[V-REP_ROS_SIM] Camera handle: " << simcam_handle);
}

void ROSServer::simulationEnded() {
    // Restart simulation if needed
    // if (sim_iter < MAX_SIMS_COUNT) {
    //     // this->loadScene();
    //     // this->startSim();
    // } else {
    //     ROS_INFO("Reached maximum number of simultations. (%d)", MAX_SIMS_COUNT);
    //     ROS_INFO("Ugly killing of this vrep instance.");
    //     system(("kill " + std::to_string(getpid())).c_str());
    // }
}

void ROSServer::spinOnce() {
    // Disable error reporting (it is enabled in the service processing part, \
    // but we don't want error reporting for publishers/subscribers)
    int errorModeSaved;
    simGetIntegerParameter(sim_intparam_error_report_mode, &errorModeSaved);
    simSetIntegerParameter(sim_intparam_error_report_mode,
                           sim_api_errormessage_ignore);

    //Process all requested services and topic subscriptions
    ros::spinOnce();

    // Restore previous error report mode:
    simSetIntegerParameter(sim_intparam_error_report_mode, errorModeSaved);
}

//
//Generate image message
//
void ROSServer::getImageMessage(sensor_msgs::Image& image_msg,
                                sensor_msgs::CameraInfo& cam_info) {
    // Handle explicitly the vision sensor!
    int detections = simHandleVisionSensor(simcam_handle, NULL, NULL);
    if (detections == -1) {
        ROS_WARN("Cannot handle the vision sensor.");
    }
    int size[2], data_len;
    const float* image_buf = simGetVisionSensorImage(simcam_handle);
    simGetVisionSensorResolution(simcam_handle, size);

    image_msg.encoding = sensor_msgs::image_encodings::RGB8;
    image_msg.width  = size[0];
    image_msg.height = size[1];
    image_msg.step   = image_msg.width * 3; // image stride in bytes

    data_len = image_msg.step * image_msg.height;
    image_msg.data.resize(data_len);
    image_msg.is_bigendian = 0;

    int msg_idx, buf_idx;
    for (unsigned int i = 0; i < image_msg.height; ++i) {
        for (unsigned int j = 0; j < image_msg.step; ++j) {
            msg_idx = (image_msg.height - i - 1) * image_msg.step + j;
            buf_idx = i * image_msg.step + j;
            image_msg.data[msg_idx] = (unsigned char)(image_buf[buf_idx] * 255);
        }
    }

    image_msg.header.frame_id = simGetObjectName(simcam_handle);
    image_msg.header.stamp = ros::Time::now();
    simReleaseBuffer((char*)image_buf);

    cam_info = CreateCameraInfo(size[0], size[1]);
    cam_info.header = image_msg.header;
}

sensor_msgs::CameraInfo ROSServer::CreateCameraInfo(int width, int height) {
    sensor_msgs::CameraInfo ci;
    double fx = double(width) / 2;
    double fy = double(height) / 2;

    ci.K = {fx, 0, double(width) / 2,
            0, fy, double(height) / 2,
            0, 0, 1
           };

    for (int i = 0; i < 5; ++i)
        ci.D.push_back(0);

    ci.P = {fx, 0, double(width) / 2, 0,
            0, fy, double(height) / 2, 0,
            0, 0, 1, 0
           };

    ci.width = width;
    ci.height = height;
    ci.distortion_model = sensor_msgs::distortion_models::PLUMB_BOB;

    return ci;
}
