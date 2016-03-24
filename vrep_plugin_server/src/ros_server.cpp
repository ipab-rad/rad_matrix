#include "../include/vrep_plugin_server/ros_server.h"
#include "../include/v_repLib.h"

#include <random>
#include <chrono>
#include <unistd.h> // get pid

#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/distortion_models.h>
#include <image_transport/image_transport.h>
#include <geometry_msgs/PointStamped.h>

#include <Eigen/Geometry>

using namespace ros;

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
	if (sim_iter < MAX_SIMS_COUNT) {
		this->loadScene();
		this->startSim();
	} else {
		ROS_INFO("Reached maximum number of simultations. (%d)", MAX_SIMS_COUNT);
		ROS_INFO("Ugly killing of this vrep instance.");
		system(("kill " + std::to_string(getpid())).c_str());
	}
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
