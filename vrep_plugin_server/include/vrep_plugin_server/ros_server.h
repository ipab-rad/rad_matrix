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
#include "vrep_plugin_server/IsSceneStatic.h"
#include "vrep_plugin_server/ResetScene.h"
#include "vrep_plugin_server/PushObject.h"
#include "vrep_plugin_server/AreCubesSplit.h"

#define WORLD_FRAME 	-1
#define MAX_SIMS_COUNT	1
#define SCENE_PATH			"scenes/rad_tabletop.ttt"
#define DEF_SETTINGS_PATH	"/home/daniel/scene.config"
#define CUBE_SIZE		float(0.0254)

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
	bool readFileToString(const std::string file, std::vector<std::string>& data);
	bool cleanCubesFromScene();
	bool parseInstructions(const std::vector<std::string>& instr);

	bool isSceneStatic(float max_speed = 0.1f);
	std::vector<int> getShapes();
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
	bool is_scene_static_callback(
	    vrep_plugin_server::IsSceneStatic::Request& request,
	    vrep_plugin_server::IsSceneStatic::Response& response);
	bool reset_scene_callback(
	    vrep_plugin_server::ResetScene::Request& request,
	    vrep_plugin_server::ResetScene::Response& response);
	bool push_object_callback(
	    vrep_plugin_server::PushObject::Request& request,
	    vrep_plugin_server::PushObject::Response& response);
	bool are_cubes_split_callback(
	    vrep_plugin_server::AreCubesSplit::Request& req,
	    vrep_plugin_server::AreCubesSplit::Response& res);
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
	ros::ServiceServer is_scene_static_service;
	ros::ServiceServer reset_scene_service;
	ros::ServiceServer push_object_service;
	ros::ServiceServer are_cubes_split_service;


	ros::Timer heartbeat_timer;

	// Handles
	int simcam_handle;
	int tabletop_handle;

	int push_end_iters;
	vrep_plugin_server::AddForce force_push_req;

	unsigned int sim_iter;
	int sim_ID;
};

#endif
