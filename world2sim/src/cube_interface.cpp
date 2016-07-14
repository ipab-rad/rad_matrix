#include <ros/ros.h>

#include <sensor_msgs/PointCloud2.h>
#include <std_srvs/Trigger.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/ros/conversions.h>
#include <pcl/features/normal_3d.h>
#include <pcl/common/transforms.h>
#include <pcl/common/common.h>

#include <vrep_plugin_server/ResetScene.h>

#include <tf2_ros/transform_listener.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>
#include <geometry_msgs/TransformStamped.h>

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <Eigen/Geometry>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>

std::string world_frame;
double cube_size;
tf2_ros::Buffer* tfBuffer;
tf2_ros::TransformListener* tfListener;

visualization_msgs::MarkerArray axisMarkers;

template <typename T>
bool IsInBounds(const T& value, const T& low, const T& high) {
	return !(value < low) && (value < high);
}

struct SimCubeType {
	pcl::PointXYZ loc;
	double height;
	double width;
	double depth;
};

bool computeBoundingBox(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
                        double& height, double& width, double& depth, double& zrot,
                        Eigen::Quaternionf& bboxQuaternion,
                        Eigen::Vector3f& bboxTransform) {
#if COMPUTE_OBB
	ROS_WARN("Computing OBB");
	// Compute principal directions
	Eigen::Vector4f pcaCentroid;
	pcl::compute3DCentroid(*cloud, pcaCentroid);
	Eigen::Matrix3f covariance;
	computeCovarianceMatrixNormalized(*cloud, pcaCentroid, covariance);
	Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eigen_solver(
	    covariance, Eigen::ComputeEigenvectors);
	Eigen::Matrix3f eigenVectorsPCA = eigen_solver.eigenvectors();
	eigenVectorsPCA.col(2) =
	    eigenVectorsPCA.col(0).cross(eigenVectorsPCA.col(1));
	/// This line is necessary for proper orientation in some cases. The numbers come out the same without it, but ///    the signs are different and the box doesn't get correctly oriented in some cases.
	/* // Note that getting the eigenvectors can also be obtained via the PCL PCA interface with something like:
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudPCAprojection (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PCA<pcl::PointXYZ> pca;
	pca.setInputCloud(cloud);
	pca.project(*cloud, *cloudPCAprojection);
	std::cerr << std::endl << "EigenVectors: " << pca.getEigenVectors() << std::endl;
	std::cerr << std::endl << "EigenValues: " << pca.getEigenValues() << std::endl;
	// In this case, pca.getEigenVectors() gives similar eigenVectors to eigenVectorsPCA.
	*/
	// Transform the original cloud to the origin where the principal components correspond to the axes.
	Eigen::Matrix4f projectionTransform(Eigen::Matrix4f::Identity());
	projectionTransform.block<3, 3>(0, 0) = eigenVectorsPCA.transpose();
	projectionTransform.block<3, 1>(0, 3) =
	    -1.f * (projectionTransform.block<3, 3>(0, 0) * pcaCentroid.head<3>());
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudPointsProjected(
	    new pcl::PointCloud<pcl::PointXYZ>);
	pcl::transformPointCloud(*cloud, *cloudPointsProjected,
	                         projectionTransform);
	// Get the minimum and maximum points of the transformed cloud.
	pcl::PointXYZ minPoint, maxPoint;
	pcl::getMinMax3D(*cloudPointsProjected, minPoint, maxPoint);
	const Eigen::Vector3f meanDiagonal = 0.5f * (maxPoint.getVector3fMap() +
	                                             minPoint.getVector3fMap());
	// Final transform
	bboxQuaternion = Eigen::Quaternionf(eigenVectorsPCA);
	//Quaternions are a way to do rotations https://www.youtube.com/watch?v=mHVwd8gYLnI
	bboxTransform = eigenVectorsPCA * meanDiagonal + pcaCentroid.head<3>();

	// visu->addCube(bboxTransform, bboxQuaternion, maxPoint.x - minPoint.x,
	//               maxPoint.y - minPoint.y, maxPoint.z - minPoint.z, "bbox", mesh_vp_3);

	height = maxPoint.z - minPoint.z;
	width  = maxPoint.y - minPoint.y;
	depth  = maxPoint.x - minPoint.x;
#else
	ROS_INFO_STREAM("Computing AABB to planar surface");
	// Get minimum and max points (bottom and top)
	pcl::PointXYZ minPoint_raw, maxPoint_raw;
	pcl::getMinMax3D(*cloud, minPoint_raw, maxPoint_raw);
	// Get orientation of the rest of the cubes
	std::vector<cv::Point2f> points(cloud->size());
	for (int i = 0; i < cloud->size(); ++i) {
		points[i] = cv::Point2f(cloud->points[i].x, cloud->points[i].y);
	}
	cv::RotatedRect rotrect = cv::minAreaRect(points);
	if (rotrect.angle < 0)
		rotrect.angle += 360;

	// Get sizes
	height = maxPoint_raw.z - minPoint_raw.z;
	width = rotrect.size.height;
	depth = rotrect.size.width;
	// Swap around to a more human understandable format
	ROS_WARN_STREAM("angle: " << rotrect.angle);
	if (/*(IsInBounds(rotrect.angle, 0.0f, 45.0f) ||
	        IsInBounds(rotrect.angle, 315.0f, 360.0f) ||
	        IsInBounds(rotrect.angle, 135.0f, 225.0f)) &&*/
	    width < depth) {
		rotrect.angle += 90;
		std::swap(width, depth);
	}
	ROS_WARN_STREAM("size: :"  << rotrect.size);

	// Put data into vars
	bboxTransform.x() = rotrect.center.x;
	bboxTransform.y() = rotrect.center.y;
	bboxTransform.z() = (minPoint_raw.z + maxPoint_raw.z) * 0.5;

	zrot = rotrect.angle * M_PI / 180.0;
	Eigen::AngleAxisf rotAngle(zrot,
	                           Eigen::Vector3f::UnitZ());
	bboxQuaternion = rotAngle;

#endif

	ROS_DEBUG_STREAM("bboxTransform: " << bboxTransform << "\nbboxQuaternion: " <<
	                 bboxQuaternion.matrix());

	Eigen::Matrix<float, 3, 1> euler =
	    bboxQuaternion.matrix().eulerAngles(2, 1, 0);
	float yaw = euler(0, 0);
	float pitch = euler(1, 0);
	float roll = euler(2, 0);

	ROS_INFO_STREAM("rpy:\n r " << roll * 180.0 / M_PI <<
	                " p " <<  pitch * 180.0 / M_PI << " y " << yaw * 180.0 / M_PI);

	visualization_msgs::Marker marker;
	marker.header.frame_id = world_frame;
	marker.header.stamp = ros::Time();
	marker.ns = "plane_segmentator";
	marker.id = 0;
	marker.type = visualization_msgs::Marker::CUBE;
	marker.action = visualization_msgs::Marker::ADD;
	marker.pose.position.x = bboxTransform.x();
	marker.pose.position.y = bboxTransform.y();
	marker.pose.position.z = bboxTransform.z();
	marker.pose.orientation.x = bboxQuaternion.x();
	marker.pose.orientation.y = bboxQuaternion.y();
	marker.pose.orientation.z = bboxQuaternion.z();
	marker.pose.orientation.w = bboxQuaternion.w();
	marker.scale.x = depth;
	marker.scale.y = width;
	marker.scale.z = height;
	marker.color.a = 0.5; // Don't forget to set the alpha!
	marker.color.r = 0.0;
	marker.color.g = 1.0;
	marker.color.b = 0.0;

	axisMarkers.markers.push_back(marker);

	return true;
}

bool addCube(Eigen::Quaternionf bboxQuaternion,
             Eigen::Vector3f bboxTransform,
             double height, double width, double depth,
             int id = 0) {
	visualization_msgs::Marker marker;
	marker.header.frame_id = world_frame;
	marker.header.stamp = ros::Time();
	marker.ns = "plane_segmentator";
	marker.id = id;
	marker.type = visualization_msgs::Marker::CUBE;
	marker.action = visualization_msgs::Marker::ADD;
	marker.pose.position.x = bboxTransform.x();
	marker.pose.position.y = bboxTransform.y();
	marker.pose.position.z = bboxTransform.z();
	marker.pose.orientation.x = bboxQuaternion.x();
	marker.pose.orientation.y = bboxQuaternion.y();
	marker.pose.orientation.z = bboxQuaternion.z();
	marker.pose.orientation.w = bboxQuaternion.w();
	marker.scale.x = depth;
	marker.scale.y = width;
	marker.scale.z = height;
	marker.color.a = 0.5;
	marker.color.r = tanh(id * 0.1) / 2.0 + 0.5;
	marker.color.g = 1.0f;
	marker.color.b = 0.4;

	axisMarkers.markers.push_back(marker);
	return true;
}


/////

void new_cloud_2_process(sensor_msgs::PointCloud2::Ptr& msg) {
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(
	    new pcl::PointCloud<pcl::PointXYZ>);
	if (msg != nullptr) {
		// DO tf2 transform
		// Untested
		ROS_INFO_STREAM("Looking up tf");
		geometry_msgs::TransformStamped transform;
		transform = tfBuffer->lookupTransform(world_frame,
		                                      msg->header.frame_id,
		                                      ros::Time(0), //acquisition_time - ros::Duration().fromSec(4),
		                                      ros::Duration(0.1));
		ROS_DEBUG_STREAM("Will perform the transform: " << transform);

		// sensor_msgs::PointCloud2 cloud_out;
		sensor_msgs::PointCloud2::Ptr cloud_out(new sensor_msgs::PointCloud2);
		tf2::doTransform(*msg, *cloud_out, transform);

		ROS_INFO_STREAM("Performed the transform from " <<  msg->header.frame_id <<
		                " to " << cloud_out->header.frame_id);
		// Convert to pcl
		pcl::PCLPointCloud2 pcl_pc2;
		pcl_conversions::toPCL(*cloud_out, pcl_pc2);
		pcl::fromPCLPointCloud2(pcl_pc2, *cloud);
		ROS_INFO_STREAM("Converted to pcl");
	} else {
		ROS_WARN("Cloud has an empty pointer!");
		return;
	}

	// Find bounding box
	double height = 0.075;
	double width  = 0.025; // m
	double depth  = 0.025;
	double zrot = 0;
	Eigen::Quaternionf boxQuaternion;
	Eigen::Vector3f boxTransform;
	computeBoundingBox(cloud, height, width, depth, zrot,
	                   boxQuaternion, boxTransform);

	const int cube_height = std::round(height / cube_size);
	const int cube_width  = std::round(width  / cube_size);
	const int cube_depth  = std::round(depth  / cube_size);
	ROS_INFO_STREAM(
	    "\n\tcube_height:" << cube_height << " " << height << std::endl <<
	    "\tcube_width: " << cube_width << " " << width << std::endl <<
	    "\tcube_depth: " << cube_depth << " " << depth);

	if (cube_height <= 0 || cube_depth <= 0 || cube_width <= 0) {
		ROS_WARN_STREAM("Wrong cube parameters! Size <= 0.");
		return;
	}

	ROS_INFO_STREAM("BEFORE LOOP");

	// Subdivide box into possible cube configurations
	std::vector<std::vector<SimCubeType>> configs;
	std::vector<SimCubeType> config_so_far;
	pcl::PointXYZ current_loc;
	bool inited = false;
	zrot = M_PI / 2 - zrot;
	for (int h = 0; h < cube_height; ++h) {
		for (int w = 0; w < cube_width; ++w) {
			for (int d = 0; d < cube_depth; ++d) {
				ROS_INFO_STREAM("IN LOOP");
				pcl::PointXYZ last_c;
				if (inited)
					last_c = config_so_far.back().loc;
				ROS_INFO_STREAM("last c defined");

				// Merge all remaining
				std::vector<SimCubeType> new_config(config_so_far);
				SimCubeType c; // new block filling all remaining
				c.loc = pcl::PointXYZ(((cube_depth - d) / 2.0 + d) * cube_size,
				                      ((cube_width - w) / 2.0 + w) * cube_size,
				                      ((cube_height - h) / 2.0 + h) * cube_size);
				ROS_INFO_STREAM("c.loc ");

				c.height = (cube_height - h) * cube_size;
				c.width = (cube_width - w) * cube_size;
				c.depth = (cube_depth - d) * cube_size;

				ROS_INFO_STREAM("before change" );//<<
				// c.loc.x);// << " " << last_c.x << " " << zrot);
				std::cout << std::endl;
				if (inited) {
					// Move the x/y coordinates
					double dp = std::sqrt(std::pow(c.loc.x - last_c.x, 2) +
					                      std::pow(c.loc.y - last_c.y, 2));
					ROS_INFO_STREAM("dp" << dp);
					double w = std::max(c.width, c.depth);
					ROS_INFO_STREAM("w" << w);
					c.loc.x -= std::sin(zrot) * std::sqrt(w * w + dp * dp);
					c.loc.y -= -std::sin(zrot) * w;

					ROS_WARN_STREAM("c.loc: " << c.loc);

				}
				new_config.push_back(c);

				configs.push_back(new_config);

				// add current cube to the list
				SimCubeType std_cube; // Add a standard cube
				std_cube.loc = pcl::PointXYZ((d + 0.5) * cube_size,
				                             (w + 0.5) * cube_size,
				                             (h + 0.5 ) * cube_size);
				std_cube.height = 1 * cube_size;
				std_cube.width = 1 * cube_size;
				std_cube.depth = 1 * cube_size;

				if (inited) {
					// Move the x/y coordinates
					double dp = std::sqrt(std::pow(std_cube.loc.x - last_c.x, 2) +
					                      std::pow(std_cube.loc.y - last_c.y, 2));
					double w = std::max(std_cube.width, std_cube.depth);

					std_cube.loc.x -= std::sin(zrot) * std::sqrt(w * w + dp * dp);
					std_cube.loc.y -= -std::sin(zrot) * w;
				}
				config_so_far.push_back(std_cube);
				inited = true; // at least one cube is pushed
			}
		}
	}

	ROS_INFO_STREAM("size: " << configs.size());
	std::vector<std::vector<std::string>> instr_list;
	int k = 1;
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
			instr += "]";
			instrs.push_back(instr);
			// ROS_INFO_STREAM(instr);
			Eigen::Vector3f offset(configs[i][j].loc.x + 0.1 * (i + 1),
			                       configs[i][j].loc.y,
			                       configs[i][j].loc.z);
			addCube(boxQuaternion,
			        boxTransform + offset,
			        configs[i][j].height, configs[i][j].width, configs[i][j].depth,
			        k++);
		}
		// ROS_INFO_STREAM("---");

		instr_list.push_back(instrs);
	}

	// Load it in the sims
	for (int i = 0; i < instr_list.size(); ++i) {
		vrep_plugin_server::ResetScene reset_msg;
		reset_msg.request.instructions = instr_list[i];
		reset_msg.request.reload_scene = false;
		reset_msg.request.start_scene = true;
		ROS_INFO_STREAM("Sending command!");
		ros::service::call("/sim_architect/resetScene", reset_msg);

		// Test action X
		std_srvs::Trigger trig_msg;
		ros::service::call("/sim_architect/success_action_x", trig_msg);
		ROS_WARN_STREAM("Result: Option:" << i << " X Split: " <<
		                trig_msg.response.success << " " <<
		                trig_msg.response.message);
		break; // Test for now with only one;
	}
}


// Callback on new cloud
sensor_msgs::PointCloud2::Ptr pc_msg;
bool new_pc;
void new_cloud_callback(const sensor_msgs::PointCloud2::Ptr& msg) {
	ROS_INFO_STREAM("Got a new PointCloud!!!");
	pc_msg = msg;
	new_pc = true;
}

int main(int argc, char** argv) {
	ros::init(argc, argv, "cube_interface");
	ros::NodeHandle nh("~");

	tfBuffer = new tf2_ros::Buffer;
	tfListener = new tf2_ros::TransformListener(*tfBuffer);

	ros::Subscriber pointcloud_sub =
	    nh.subscribe("/plane_segmentator/clustered_object_cloud", 1,
	                 new_cloud_callback);

	ros::Publisher vis_pub =
	    nh.advertise<visualization_msgs::MarkerArray>("visualization_axisMarkers", 1);

	ros::param::param("cube_size", cube_size, 0.0254);
	ros::param::param<std::string>("world_frame", world_frame, "base_link");

	// // FOR testing!
	// ROS_INFO_STREAM("Begin testing");
	// new_cloud_2_process(pc_msg);
	// ROS_INFO_STREAM("---");
	// // End for testing

	new_pc = false;
	ros::Rate r(30);
	while (ros::ok()) {
		ros::spinOnce();

		if (new_pc) {
			new_pc = false;
			// axisMarkers.markers.clear();
			ROS_INFO_STREAM("New msg!");
			new_cloud_2_process(pc_msg);
			vis_pub.publish(axisMarkers);
		}

		r.sleep();
	}

	return 0;
}

