/*!
 * \file
 * \brief
 * \author Tomek Kornuta,,,
 */

#include <memory>
#include <string>

#include "ROSProxy.hpp"
#include "Common/Logger.hpp"

#include <boost/bind.hpp>

#include <pcl/common/transforms.h>
#include <pcl/PolygonMesh.h>

#include <tf_conversions/tf_eigen.h>
#include <eigen_conversions/eigen_msg.h>
#include <object_recognition_msgs/RecognizedObject.h>

#include <tf/transform_datatypes.h>
#include <geometry_msgs/Point.h>


namespace Processors {
namespace ROSProxy {

ROSProxy::ROSProxy(const std::string & name) :
	Base::Component(name),
	prop_parent_frame("parent_frame",std::string("/discode_kinect")),
	prop_ros_spin("ros.spin",true)
{
	// General properties.
	registerProperty(prop_parent_frame);
	registerProperty(prop_ros_spin);

}

ROSProxy::~ROSProxy() {
}

void ROSProxy::prepareInterface() {
	// Register data streams.
	registerStream("in_object_poses", &in_object_poses);
	registerStream("in_object_confidences", &in_object_confidences);
	registerStream("in_object_vertices_xyz", &in_object_vertices_xyz);
	registerStream("in_object_triangles", &in_object_triangles);

	registerStream("in_object_pose_covariances", &in_object_pose_covariances);
	registerStream("in_object_labels", &in_object_labels);

	// Register handlers
	registerHandler("spin", boost::bind(&ROSProxy::spin, this));
	addDependency("spin", NULL);

	registerHandler("publishPoses", boost::bind(&ROSProxy::publishPoses, this));
	addDependency("publishPoses", &in_object_poses);
	addDependency("publishPoses", &in_object_confidences);
	addDependency("publishPoses", &in_object_vertices_xyz);
	addDependency("publishPoses", &in_object_triangles);
	//addDependency("publishPoses", &in_object_pose_covariances); -> not obligatory.
	//addDependency("publishPoses", &in_object_labels); -> not obligatory.
}

bool ROSProxy::onInit() {
	// Initialize ROS node.
	static int argc;
	static char * argv = NULL;
	// Initialize ROS node.
	ros::init(argc, &argv, std::string("discode"), ros::init_options::NoSigintHandler);

	// Create ROS node handle.
	nh = new ros::NodeHandle;

	// Create publisherobj.
	pub = nh->advertise<object_recognition_msgs::RecognizedObject>("recognized_objects", 1000);

	return true;
}

bool ROSProxy::onFinish() {
	return true;
}

bool ROSProxy::onStop() {
	return true;
}

bool ROSProxy::onStart() {
	return true;
}

void ROSProxy::spin() {
	if(prop_ros_spin)
		ros::spinOnce();
}




/// Conversion from PointXYZ to geometry_msgs::Point
static inline geometry_msgs::Point pointToMsg(const pcl::PointXYZ& pt_xyz_){
  geometry_msgs::Point pt;
  pt.x = pt_xyz_.x;
  pt.y = pt_xyz_.y;
  pt.z = pt_xyz_.z;

  return pt;
}


static inline shape_msgs::MeshTriangle triangleToMsg(const pcl::Vertices& triangle_){
	shape_msgs::MeshTriangle mt;
	mt.vertex_indices[0] = triangle_.vertices[0];
	mt.vertex_indices[1] = triangle_.vertices[1];
	mt.vertex_indices[2] = triangle_.vertices[2];
	return mt;
}


void ROSProxy::publishPoses() {
	// Read OBLIGATORY data ports.
	std::vector<Types::HomogMatrix> object_poses = in_object_poses.read();
	std::vector<double> object_confidences = in_object_confidences.read();
	std::vector< pcl::PointCloud<pcl::PointXYZ>::Ptr> object_vertices = in_object_vertices_xyz.read();
	std::vector< std::vector<pcl::Vertices> > object_triangles = in_object_triangles.read();


	// Read object labels.
	std::vector< std::string> object_labels;
	if (!in_object_labels.empty())
		object_labels = in_object_labels.read();

	size_t i=0;
	// Fill vector of temporary names - if required.
	while(object_labels.size() < object_poses.size()) {
		std::ostringstream s;
		s << i++;
		std::string cname = "object_" + s.str();
		object_labels.push_back(cname);
	}//: while

	// Read object pose covariances.
	std::vector<boost::array<double, 36ul> > object_pose_covariances;
	if (!in_object_pose_covariances.empty())
		object_pose_covariances = in_object_pose_covariances.read();

	i=0;
	// Create "default" measurement (sensor) noise matrix.
	cv::Mat measurementNoiseCov (cv::Mat::eye(6, 6, CV_64F));
	cv::setIdentity(measurementNoiseCov, cv::Scalar::all(1e-4));
	// Fill vector of covariance matrix.
	while(object_pose_covariances.size() < object_poses.size()) {
		// Create matrix.
		boost::array<double, 36ul> def_cov;
		std::copy(measurementNoiseCov.begin<double>(), measurementNoiseCov.end<double>(), def_cov.begin());
		object_pose_covariances.push_back(def_cov);
	}//: while



	// Broadcasterobj.
	//static tf::TransformBroadcaster br;

	// Publish objects 1 by 1.
	for (size_t obj_i = 0; obj_i < object_poses.size(); ++obj_i) {
		CLOG(LDEBUG) << "Sending transform: " << object_poses[obj_i];

		// Transform pose to message.
		Eigen::Affine3d pose = object_poses[obj_i];
		geometry_msgs::Pose msg_pose;
		tf::poseEigenToMsg(pose, msg_pose);

		// Generate TF name - cut out the numbers!
		std::string name = "/"+	object_labels[obj_i].substr(0,object_labels[obj_i].rfind("_"));

		// Fill header structure - std_msgs/Header header
		std_msgs::Header hdr;
		// sequence ID: consecutively increasing ID - uint32 seq - not used.
		// time stamp
		hdr.stamp = ros::Time::now(); // Temporary solution - TODO
		//	string frame_id - set parent frame (name of sensor optical frame).
		hdr.frame_id = prop_parent_frame;

		// Fill Recognized object structure.
		object_recognition_msgs::RecognizedObject robj;
		// std_msgs/Header header
		robj.header = hdr;

		// object_recognition_msgs/ObjectType type
		// string key
		robj.type.key = name;
		// string db
		robj.type.db = "";

		// float32 confidence TODO!
		robj.confidence = object_confidences[obj_i];

		// sensor_msgs/PointCloud2[] point_clouds - not used.

		// shape_msgs/Mesh bounding_mesh TODO!
		// Set mesh vertices.
		//pcl::toPCLPointCloud2(*object_vertices[i], robj.bounding_mesh.vertices);
		//robj.bounding_mesh.vertices.push_back(object_vertices[i]->at(0));
		//pcl::PointXYZ bt_v(1,2,3);
		//geometry_msgs::Point msg_v = pointToMsg(bt_v);

		// Add vertices to mesh - ONE BY ONE:]
		for (size_t pt_i=0; pt_i < object_vertices[obj_i]->size(); pt_i++) {
			robj.bounding_mesh.vertices.push_back(pointToMsg(object_vertices[obj_i]->at(pt_i)));
		}//: for

		// Set mesh polygon.
		//robj.bounding_mesh.triangles = object_triangles[i];
		// Add triangles to mesh - ONE BY ONE:]
		for (size_t tr_i=0; tr_i < object_triangles[obj_i].size(); tr_i++) {
			robj.bounding_mesh.triangles.push_back(triangleToMsg(object_triangles[obj_i].at(tr_i)));
		}//: for

		// geometry_msgs/Point[] bounding_contours - not used.

		// geometry_msgs/PoseWithCovarianceStamped pose
		// std_msgs/Header header
		robj.pose.header = hdr;
		//geometry_msgs/PoseWithCovariance pose
		// geometry_msgs/Pose pose
		robj.pose.pose.pose = msg_pose;
		// float64[36] covariance
		robj.pose.pose.covariance = object_pose_covariances[obj_i];

		// Publish object data.
		pub.publish(robj);
	}//: for
}



} //: namespace ROSProxy
} //: namespace Processors
