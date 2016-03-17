#ifndef ROS_SERVER_H
#define ROS_SERVER_H

#include <ros/ros.h>

#include <std_msgs/Int32.h>
#include <std_msgs/String.h>
#include <sensor_msgs/Image.h>
#include <image_transport/image_transport.h>
#include <geometry_msgs/PointStamped.h>

#include "vrep_plugin_server/AddShape.h"
#include "vrep_plugin_server/SetShapeColour.h"
#include "vrep_plugin_server/AddForce.h"
#include "vrep_plugin_server/AddForceTorque.h"

#define WORLD_FRAME 	-1
#define MAX_SIMS_COUNT	1
#define SCENE_PATH		"scenes/rad_tabletop.ttt"

class ROSServer {
  public:
	ROSServer() {};

	bool initialize();
	void shutDown();

	void instancePass();
	void mainScriptAboutToBeCalled();

	void simulationAboutToStart();
	void simulationEnded();

  private:
	void spinOnce();
	void startSim();
	void loadScene();

	void getImageMessage(sensor_msgs::Image& image_msg,
	                     sensor_msgs::CameraInfo& cam_info);
	sensor_msgs::CameraInfo CreateCameraInfo(int width, int height);

	void streamAllData();
	void simulationUpTimerCallback(const ros::TimerEvent& e);
	bool add_object_callback(vrep_plugin_server::AddShape::Request& request,
	                         vrep_plugin_server::AddShape::Response& response);
	bool set_object_colour_callback(
	    vrep_plugin_server::SetShapeColour::Request& request,
	    vrep_plugin_server::SetShapeColour::Response& response);
	bool add_force_callback(
	    vrep_plugin_server::AddForce::Request& request,
	    vrep_plugin_server::AddForce::Response& response);
	bool add_force_torque_callback(
	    vrep_plugin_server::AddForceTorque::Request& request,
	    vrep_plugin_server::AddForceTorque::Response& response);

	// Data
	ros::NodeHandle* node;

	// Publishers:
	image_transport::ImageTransport* it;
	image_transport::CameraPublisher cam_pub;
	ros::Publisher simulation_up_pub;

	// Services
	ros::ServiceServer add_shape_service;
	ros::ServiceServer set_colour_shape_service;
	ros::ServiceServer add_force_service;
	ros::ServiceServer add_force_torque_service;

	ros::Timer heartbeat_timer;

	// Handles
	int simcam_handle;
	int cube_handle;

	unsigned int sim_iter;
	int sim_ID;
};

#endif
