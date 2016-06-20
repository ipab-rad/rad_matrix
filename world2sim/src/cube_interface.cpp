#include <ros/ros.h>

#include <sensor_msgs/PointCloud2.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/ros/conversions.h>

#include <vrep_plugin_server/ResetScene.h>

double cube_size;

struct SimCubeType {
	pcl::PointXYZ loc;
	double height;
	double width;
	double depth;
};

void new_cloud_2_process(const pcl::PCLPointCloud2::ConstPtr& msg) {
	// DO tf2 transform

	// Convert to pcl
	if (msg != nullptr) {
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(
		    new pcl::PointCloud<pcl::PointXYZRGB>);
		pcl::fromPCLPointCloud2(*msg, *cloud);
	} else {
		ROS_WARN("Cloud has an empty pointer!");
	}

	// Find bounding box
	// do pcl and axis align

	double height = 0.025;
	double width  = 0.025; // m
	double depth  = 0.075;

	const int cube_height = std::round(height / cube_size);
	const int cube_width  = std::round(width  / cube_size);
	const int cube_depth  = std::round(depth  / cube_size);
	ROS_INFO_STREAM("cube_height: " << cube_height << std::endl <<
	                "cube_width: " << cube_width << std::endl <<
	                "cube_depth: " << cube_depth);

	// Subdivide box into possible cube configurations
	std::vector<std::vector<SimCubeType>> configs;
	std::vector<SimCubeType> config_so_far;
	pcl::PointXYZ current_loc;
	for (int h = 0; h < cube_height; ++h) {
		for (int w = 0; w < cube_width; ++w) {
			for (int d = 0; d < cube_depth; ++d) {
				current_loc = pcl::PointXYZ(d * cube_size, w * cube_size, h * cube_size);

				ROS_INFO_STREAM("Loc: " << current_loc);
				// Merge all remaining
				std::vector<SimCubeType> new_config(config_so_far);
				SimCubeType c; // new block filling all remaining
				c.loc = current_loc;
				c.height = (cube_height - h) * cube_size;
				c.width = (cube_width - w) * cube_size;
				c.depth = (cube_depth - d) * cube_size;
				new_config.push_back(c);

				configs.push_back(new_config);

				// add current cube to the list
				SimCubeType std_cube; // Add a standard cube
				std_cube.loc = current_loc;
				std_cube.height = 1 * cube_size;
				std_cube.width = 1 * cube_size;
				std_cube.depth = 1 * cube_size;
				config_so_far.push_back(std_cube);
			}
		}
	}

	ROS_INFO_STREAM("size: " << configs.size());
	std::vector<std::vector<std::string>> instr_list;
	for (int i = 0; i < configs.size(); ++i) {
		std::vector<std::string> instrs;
		for (int j = 0; j < configs[i].size(); ++j) {
			std::string instr;
			instr += "cube" + std::to_string(j) + "["; // Name
			instr += "1, "; // ROS shape
			instr += std::to_string(configs[i][j].loc.x) + "," +
			         std::to_string(configs[i][j].loc.y) + "," +
			         std::to_string(configs[i][j].loc.z + 0.8) + ","; // position
			instr += "0, 0, 0, 1,"; // Orientation
			instr += std::to_string(configs[i][j].height) + "," +
			         std::to_string(configs[i][j].width) + "," +
			         std::to_string(configs[i][j].depth) + ","; // dimension
			instr += "0.024,"; // mass - 24g
			instr += "180, 240, " +
			         std::to_string(int((255.0) * (double(j) / configs[i].size()))); // colour
			instrs.push_back(instr);
			ROS_INFO_STREAM(instr);
		}
		ROS_INFO_STREAM("---");
		// ROS_INFO_STREAM("Instr: " << std::endl << instrs << std::endl << " --- ");
		instr_list.push_back(instrs);
	}

	// Load it in the sims
	vrep_plugin_server::ResetScene reset_msg;
	reset_msg.request.instructions = instr_list[instr_list.size() - 1];
	reset_msg.request.reload_scene = false;
	reset_msg.request.start_scene = true;
	ROS_INFO_STREAM("Sending command!");
	ros::service::call("/sim_architect/resetScene", reset_msg);

}


// Callback on new cloud
pcl::PCLPointCloud2::ConstPtr pc_msg;
bool new_pc;
void new_cloud_callback(const pcl::PCLPointCloud2::ConstPtr& msg) {
	ROS_INFO_STREAM("Got a new PointCloud!!!");
	pc_msg = msg;
	new_pc = true;
}

int main(int argc, char** argv) {
	ros::init(argc, argv, "cube_interface");
	ros::NodeHandle nh("~");

	ros::Subscriber pointcloud_sub =
	    nh.subscribe("/kinect2/qhd/points", 1, new_cloud_callback);

	ros::param::param("cube_size", cube_size, 0.0254);

	// FOR testing!
	ROS_INFO_STREAM("Begin testing");
	new_cloud_2_process(pc_msg);
	ROS_INFO_STREAM("---");
	// End for testing

	new_pc = false;
	ros::Rate r(30);
	while (ros::ok()) {
		ros::spinOnce();

		if (new_pc) {
			new_pc = false;
			ROS_INFO_STREAM("New msg!");
			new_cloud_2_process(pc_msg);
		}

		r.sleep();
	}

	return 0;
}

