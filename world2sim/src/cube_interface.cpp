#include <ros/ros.h>

#include <sensor_msgs/PointCloud2.h>

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

std::string world_frame;
double cube_size;
tf2_ros::Buffer* tfBuffer;
tf2_ros::TransformListener* tfListener;

visualization_msgs::MarkerArray axisMarkers;

struct SimCubeType {
	pcl::PointXYZ loc;
	double height;
	double width;
	double depth;
};

////////////
void computePrincipalAxis(const pcl::PointCloud<pcl::PointXYZRGB> cloud,
                          Eigen::Vector4f& centroid,
                          Eigen::Matrix3f& evecs, Eigen::Vector3f& evals) {
	Eigen::Matrix3f covariance_matrix;
	pcl::computeCovarianceMatrix(cloud, centroid, covariance_matrix);
	pcl::eigen33(covariance_matrix, evecs, evals);
}

void createArrowMarker(Eigen::Vector3f& vec,
                       int id, double length,
                       Eigen::Vector4f& centroid) {
	visualization_msgs::Marker marker;
	marker.header.frame_id = world_frame;
	marker.header.stamp = ros::Time();
	marker.id = id;
	marker.type = visualization_msgs::Marker::ARROW;
	marker.action = visualization_msgs::Marker::ADD;

	float k = 0.1; // scale the length of the arrows

	// geometry_msgs::Point p;
	// p.x = centroid[0];
	// p.y = centroid[1];
	// p.z = centroid[2];
	// marker.points.push_back(p);
	// p.x += k * vec[0];
	// p.y += k * vec[1];
	// p.z += k * vec[2];
	// marker.points.push_back(p);
	marker.pose.position.x = centroid[0];
	marker.pose.position.y = centroid[1];
	marker.pose.position.z = centroid[2];
	marker.pose.orientation.x = vec[0];
	marker.pose.orientation.y = vec[1];
	marker.pose.orientation.z = vec[2];
	marker.pose.orientation.w = 1.0;

	marker.scale.x = length * k;
	marker.scale.y = 0.02;
	marker.scale.z = 0.02;
	marker.color.a = 1.0;
	marker.color.r = 1.0;
	marker.color.g = 1.0;
	marker.color.b = 0.5;
	axisMarkers.markers.push_back(marker);
}

////////////


bool computeBoundingBox(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
                        double& height, double& width, double& depth,
                        Eigen::Quaternionf& bboxQuaternion,
                        Eigen::Vector3f& bboxTransform) {
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
	// pcl::PointXYZ minPoint_raw, maxPoint_raw;
	// pcl::getMinMax3D(*cloud, minPoint_raw, maxPoint_raw);

	height = maxPoint.z - minPoint.z;
	width  = maxPoint.y - minPoint.y;
	depth  = maxPoint.x - minPoint.x;
	ROS_INFO_STREAM("bboxTransform: " << bboxTransform << "\n bboxQuaternion: " <<
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
	marker.color.r = 0.0;
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
		ROS_INFO_STREAM("Will perform the transform: " << transform);

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
	}

	// Find bounding box
	// do pcl and axis align
	double height = 0.025;
	double width  = 0.025; // m
	double depth  = 0.075;
	Eigen::Quaternionf boxQuaternion;
	Eigen::Vector3f boxTransform;
	computeBoundingBox(cloud, height, width, depth, boxQuaternion, boxTransform);
	////////
	// Untested
	// Eigen::Vector4f centroid;
	// Eigen::Matrix3f evecs;
	// Eigen::Vector3f evals;

	// // Table is the pointcloud of the table only.
	// pcl::compute3DCentroid(*cloud, centroid);
	// computePrincipalAxis(*cloud, centroid, evecs, evals);
	// ROS_INFO_STREAM("centroid: " << centroid);

	// Eigen::Vector3f vec;

	// vec << evecs.col(0);
	// createArrowMarker(vec, 1, evals[0], centroid);

	// vec << evecs.col(1);
	// createArrowMarker(vec, 2, evals[1], centroid);

	// vec << evecs.col(2);
	// createArrowMarker(vec, 3, evals[2], centroid);
	///////////////////

	// For testing
	// double height = 0.025;
	// double width  = 0.025; // m
	// double depth  = 0.075;

	const int cube_height = std::round(height / cube_size);
	const int cube_width  = std::round(width  / cube_size);
	const int cube_depth  = std::round(depth  / cube_size);
	ROS_INFO_STREAM("cube_height: " << cube_height << " " << height << std::endl <<
	                "\tcube_width: " << cube_width << " " << width << std::endl <<
	                "\tcube_depth: " << cube_depth << " " << depth);



	if (cube_height <= 0 || cube_depth <= 0 || cube_width <= 0) {
		ROS_WARN_STREAM("Wrong cube parameters! Size <= 0.");
		return;
	}

	// Subdivide box into possible cube configurations
	std::vector<std::vector<SimCubeType>> configs;
	std::vector<SimCubeType> config_so_far;
	pcl::PointXYZ current_loc;
	for (int h = 0; h < cube_height; ++h) {
		for (int w = 0; w < cube_width; ++w) {
			for (int d = 0; d < cube_depth; ++d) {
				current_loc = pcl::PointXYZ(d * cube_size, w * cube_size, h * cube_size);

				// ROS_INFO_STREAM("Loc: " << current_loc);
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
			// ROS_INFO_STREAM(instr);
			Eigen::Vector3f offset(configs[i][j].loc.x + 0.1 * (i + 1),
			                       configs[i][j].loc.y,
			                       configs[i][j].loc.z);
			addCube(boxQuaternion,
			        boxTransform + offset,
			        configs[i][j].height, configs[i][j].width, configs[i][j].depth,
			        j + 10 * (i + 1));
		}
		// ROS_INFO_STREAM("---");
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
	    nh.subscribe("/plane_segmentator/clustered_object", 1, new_cloud_callback);

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
			axisMarkers.markers.clear();
			ROS_INFO_STREAM("New msg!");
			new_cloud_2_process(pc_msg);
			vis_pub.publish(axisMarkers);
		}

		r.sleep();
	}

	return 0;
}

