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
#include <vrep_plugin_server/ActionA.h>

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

const float TABLE_OFFSET(0.72);

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
		rotrect.angle = rotrect.angle - 90;
		std::swap(width, depth);
	}
	ROS_WARN_STREAM("size: :"  << rotrect.size);

	// Put data into vars
	bboxTransform.x() = rotrect.center.x;
	bboxTransform.y() = rotrect.center.y;
	bboxTransform.z() = (minPoint_raw.z + maxPoint_raw.z) * 0.5;

	zrot = rotrect.angle * M_PI / 180.0;
	Eigen::AngleAxisf rotAngle(zrot, Eigen::Vector3f::UnitZ());
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
	marker.color.a = 0.2; // Don't forget to set the alpha!
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
enum Orientation { UNKNOWN, PLANAR, TOWER };
double delta_p = 0.001;
/////
std::vector<std::vector<SimCubeType> > get_possible_configs(int height,
                                                            int width,
                                                            int depth,
                                                            double& alpha,
                                                            std::vector<SimCubeType> config_so_far = std::vector<SimCubeType>(),
                                                            Orientation orient = Orientation::UNKNOWN) {
	ROS_INFO_STREAM("Inside function");
	std::vector<std::vector<SimCubeType> > res;
	if (height == 0 || width == 0 || depth == 0) {
		res.push_back(config_so_far);
		ROS_INFO_STREAM("Reached the end of the recursion");
		return res;
	}

	// Get orientation
	if (orient == Orientation::UNKNOWN) {
		if (height == 1 && (width != 1 || depth != 1)) // planar
			orient = Orientation::PLANAR;
		else
			orient = Orientation::TOWER;
	}

	if (orient == Orientation::PLANAR) {
		// the cubes are planar
		ROS_WARN_STREAM("Cubes are planar, not tested.");
		if (width < depth) {
			ROS_WARN_STREAM("Rotating!");
			alpha = M_PI / 2 - alpha;
			std::swap(width, depth);
		}
		for (int i = 1; i <= width; ++i) {
			ROS_INFO_STREAM("Attempting width: " << i);
			std::vector<std::vector<SimCubeType> > new_config;
			std::vector<SimCubeType> new_config_so_far(config_so_far);
			SimCubeType new_block;
			new_block.height = 1 * cube_size;
			new_block.width = i * cube_size;
			new_block.depth = 1 * cube_size;
			new_block.loc.x = config_so_far.empty() ? float(new_block.width) * 0.5 :
			                  config_so_far.back().loc.x -
			                  (config_so_far.back().width + new_block.width) * 0.5 * sin(alpha);

			new_block.loc.y = config_so_far.empty() ? float(new_block.width) * 0.5 :
			                  (config_so_far.back().loc.y +
			                   (config_so_far.back().width + new_block.width) * 0.5 * cos(alpha));
			new_block.loc.z = cube_size / 2.0 + delta_p;
			ROS_INFO_STREAM("loc.z: " << new_block.loc.z);
			new_config_so_far.push_back(new_block);
			ROS_INFO_STREAM("new_config_so_far.size()=" << new_config_so_far.size());
			new_config = get_possible_configs(height, width - i, depth, alpha,
			                                  new_config_so_far, orient);
			ROS_INFO_STREAM("new_config.size()=" << new_config.size());
			// res.insert(std::end(res), std::begin(new_config), std::end(new_config));
			std::copy(new_config.begin(), new_config.end(), std::back_inserter(res));

			ROS_INFO_STREAM("Copy, so big res has " << res.size() << " elements.");
		}
	} else if (orient == Orientation::TOWER) {
		// It is a tower
		for (int i = 1; i <= height; ++i) {
			ROS_INFO_STREAM("Attempting height: " << i);
			std::vector<std::vector<SimCubeType> > new_config;
			std::vector<SimCubeType> new_config_so_far(config_so_far);
			SimCubeType new_block;
			new_block.height = i * cube_size;
			new_block.width = 1 * cube_size;
			new_block.depth = 1 * cube_size;
			new_block.loc.x = 0;
			new_block.loc.y = 0;
			new_block.loc.z = config_so_far.empty() ? float(new_block.height) * 0.5 :
			                  config_so_far.back().loc.z + config_so_far.back().height * 0.5 +
			                  float(new_block.height) * 0.5 + delta_p;
			ROS_INFO_STREAM("loc.z: " << new_block.loc.z);
			new_config_so_far.push_back(new_block);
			ROS_INFO_STREAM("new_config_so_far.size()=" << new_config_so_far.size());
			new_config = get_possible_configs(height - i, width, depth, alpha,
			                                  new_config_so_far, orient);
			ROS_INFO_STREAM("new_config.size()=" << new_config.size());
			// res.insert(std::end(res), std::begin(new_config), std::end(new_config));
			std::copy(new_config.begin(), new_config.end(), std::back_inserter(res));

			ROS_INFO_STREAM("Copy, so big res has " << res.size() << " elements.");
		}
	} else {
		ROS_WARN_STREAM("Cannot decide how to orient the single block with no history");
	}
	return res;
}

/////
void new_cloud_2_process(sensor_msgs::PointCloud2::Ptr& msg) {
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(
	    new pcl::PointCloud<pcl::PointXYZ>);
	if (msg != nullptr) {
		// DO tf2 transform
		sensor_msgs::PointCloud2::Ptr cloud_out(new sensor_msgs::PointCloud2);
		if (world_frame != msg->header.frame_id) {
			ROS_INFO_STREAM("Looking up tf");
			geometry_msgs::TransformStamped transform;
			transform = tfBuffer->lookupTransform(world_frame,
			                                      msg->header.frame_id,
			                                      ros::Time(0), //acquisition_time - ros::Duration().fromSec(4),
			                                      ros::Duration(0.1));
			ROS_DEBUG_STREAM("Will perform the transform: " << transform);

			// sensor_msgs::PointCloud2 cloud_out;
			tf2::doTransform(*msg, *cloud_out, transform);

			ROS_INFO_STREAM("Performed the transform from " <<  msg->header.frame_id <<
			                " to " << cloud_out->header.frame_id);
		} else {
			cloud_out = msg;
		}

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
	    "\n\tcube_height: " << cube_height << " " << height << std::endl <<
	    "\tcube_width:  "   << cube_width << " " << width << std::endl <<
	    "\tcube_depth:  "   << cube_depth << " " << depth);

	if (cube_height <= 0 || cube_depth <= 0 || cube_width <= 0) {
		ROS_WARN_STREAM("Wrong cube parameters! Size <= 0.");
		return;
	} else if ((cube_height > 1 && (cube_depth > 1 || cube_width > 1)) ||
	           (cube_depth > 1 && (cube_height > 1 || cube_width > 1)) ||
	           (cube_width > 1 && (cube_height > 1 || cube_depth > 1))) {
		ROS_WARN_STREAM("Too big of an object! At least two parameters need to be 1");
		return;
	}

	ROS_INFO_STREAM("Getting subcube division!!!");
	std::vector<std::vector<SimCubeType>> configs;
	configs = get_possible_configs(cube_height, cube_width, cube_depth, zrot);
	boxQuaternion = Eigen::AngleAxisf(zrot, Eigen::Vector3f::UnitZ());

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
			         std::to_string(configs[i][j].loc.z + TABLE_OFFSET) + ","; // position
			instr += std::to_string(boxQuaternion.x()) + "," + // Orientation
			         std::to_string(boxQuaternion.y()) + "," +
			         std::to_string(boxQuaternion.z()) + "," +
			         std::to_string(boxQuaternion.w()) + ",";
			instr += std::to_string(configs[i][j].depth) + "," +
			         std::to_string(configs[i][j].width) + "," +
			         std::to_string(configs[i][j].height) + ","; // dimension
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
		instr_list.push_back(instrs);
	}


	// Outcome
	int sz = 6 * std::max(cube_depth, std::max(cube_width, cube_height));
	ROS_WARN_STREAM("sz: " << sz << " condfigs: " << configs.size());
	Eigen::MatrixXd success_matrix(configs.size(), sz);
	ROS_INFO_STREAM("size: " << success_matrix);
	// Load it in the sims
	for (int i = 0; i < instr_list.size(); ++i) { // equals configs[i]
		int action_num = 0;
		// Test action X
		for (int dir = -3; dir <= 3; ++dir) {
			if (dir == 0) continue;
			for (int cube_target = 0; cube_target < configs[i].size(); ++cube_target) {
				vrep_plugin_server::ActionA act_msg;
				act_msg.request.cube_id = cube_target;
				act_msg.request.direction = dir;
				// make an offset per cube based on dims
				for (int offset_pos_z = 0;
				        offset_pos_z < configs[i][cube_target].height / cube_size;
				        offset_pos_z++) {
					for (int offset_pos_y = 0;
					        offset_pos_y < configs[i][cube_target].depth / cube_size;
					        offset_pos_y++) {
						for (int offset_pos_x = 0;
						        offset_pos_x < configs[i][cube_target].width / cube_size;
						        offset_pos_x++) {
							geometry_msgs::Point pos;
							pos.x = configs[i][cube_target].loc.x;
							pos.y = configs[i][cube_target].loc.y;
							pos.z = configs[i][cube_target].loc.z + TABLE_OFFSET;

							// Adjust for cubes that consist of multiple sizes
							pos.z += cube_size * (offset_pos_z  + 0.5 -
							                      (configs[i][cube_target].height / cube_size / 2));
							pos.y += cube_size * (offset_pos_y  + 0.5 -
							                      (configs[i][cube_target].depth / cube_size / 2));
							pos.x += cube_size * (offset_pos_x  + 0.5 -
							                      (configs[i][cube_target].width / cube_size / 2));

							act_msg.request.offset = pos;
							ROS_DEBUG_STREAM("Position: " << pos);

							// execute all calls
							// Reset scene
							vrep_plugin_server::ResetScene reset_msg;
							reset_msg.request.instructions = instr_list[i];
							reset_msg.request.reload_scene = false;
							reset_msg.request.start_scene = true;
							ROS_INFO_STREAM("Sending reset command!");
							if (!ros::service::call("/sim_architect/resetScenes", reset_msg)) {
								ROS_ERROR_STREAM("Cannot execute reset scene with instruction: (1):" <<
								                 reset_msg.request.instructions[0]);
							}

							// Test action
							if (!ros::service::call("/sim_architect/success_action_all", act_msg)) {
								ROS_ERROR_STREAM("Cannot test the success of action X!");
							}
							ROS_INFO_STREAM("Result:\n\t\tOption: " << i << "/" << instr_list.size() <<
							                " Dir: " << dir << " X Split: " <<
							                ((act_msg.response.success) ? "success" : "fail") << " msg: " <<
							                act_msg.response.message);

							success_matrix(i, action_num++) = (act_msg.response.success) ? 1 : 0;

							// ros::Duration(0.005).sleep();
						}
					}
				}

			}
		}
	}
	ROS_INFO_STREAM("Executed all possible scenarios\n" << success_matrix);
	ROS_WARN_STREAM("sum: \n" << success_matrix.colwise().sum());
	ros::Duration(5).sleep();
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

