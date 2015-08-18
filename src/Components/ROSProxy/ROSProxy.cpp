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

#include <tf_conversions/tf_eigen.h>
#include <eigen_conversions/eigen_msg.h>
#include <object_recognition_msgs/RecognizedObject.h>


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
	registerStream("in_object_labels", &in_object_labels);
	registerStream("in_object_poses", &in_object_poses);
	registerStream("in_object_pose_covariances", &in_object_pose_covariances);

	// Register handlers
	registerHandler("spin", boost::bind(&ROSProxy::spin, this));
	addDependency("spin", NULL);

	registerHandler("publishPoses", boost::bind(&ROSProxy::publishPoses, this));
	addDependency("publishPoses", &in_object_poses);
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

	// Create publisher.
	pub = nh->advertise<object_recognition_msgs::RecognizedObject>("RecognizedObjects", 1000);

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

void ROSProxy::publishPoses() {
	// Read object poses.
	std::vector<Types::HomogMatrix> object_poses = in_object_poses.read();

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
	// Create "default" measurement noise matrix.
	cv::Mat measurementNoiseCov (cv::Mat::eye(6, 6, CV_64F));
	cv::setIdentity(measurementNoiseCov, cv::Scalar::all(1e-4));
	// Fill vector of covariance matrix.
	while(object_pose_covariances.size() < object_poses.size()) {
		// Create matrix.
		boost::array<double, 36ul> def_cov;
		std::copy(measurementNoiseCov.begin<double>(), measurementNoiseCov.end<double>(), def_cov.begin());
		object_pose_covariances.push_back(def_cov);
	}//: while



	// Broadcaster.
	//static tf::TransformBroadcaster br;

	// Publish poses 1 by 1.
	for (size_t i = 0; i < object_poses.size(); ++i) {
		CLOG(LDEBUG) << "Sending transform: " << object_poses[i];

		// Transform pose to quaternion.
		Eigen::Affine3d pose = object_poses[i];
/*		tf::Transform transform;
		tf::poseEigenToTF(pose,transform);*/

		// Transform pose to message.
		geometry_msgs::Pose msg_pose;
		tf::poseEigenToMsg(pose, msg_pose);

		// Generate TF name.
		std::string name = "/"+object_labels[i];
		//name += ('0'+i);
//		br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), prop_parent_frame, name));

//		ros::Publisher pub = nh.advertise<std_msgs::String>("topic_name", 5);
		//std_msgs::StringPtr str(new std_msgs::String);
		//str->data = "hello world";



		// Fill header structure - std_msgs/Header header
		std_msgs::Header hdr;
		// sequence ID: consecutively increasing ID - uint32 seq - not used.
		// time stamp
		hdr.stamp = ros::Time::now(); // Temporary solution - TODO
		//	string frame_id - set parent frame (name of sensor optical frame).
		hdr.frame_id = prop_parent_frame;

		// Fill Recognized object structure.
		object_recognition_msgs::RecognizedObject r;
		// std_msgs/Header header
		r.header = hdr;

		// object_recognition_msgs/ObjectType type
		// string key
		r.type.key = name;
		// string db
		r.type.db = "";

		// float32 confidence TODO!
		r.confidence = 1;

		// sensor_msgs/PointCloud2[] point_clouds - not used.

		// shape_msgs/Mesh bounding_mesh TODO!

		// geometry_msgs/Point[] bounding_contours - not used.

		// geometry_msgs/PoseWithCovarianceStamped pose
		// std_msgs/Header header
		r.pose.header = hdr;
		//geometry_msgs/PoseWithCovariance pose
		// geometry_msgs/Pose pose
		r.pose.pose.pose = msg_pose;
		// float64[36] covariance
		r.pose.pose.covariance = object_pose_covariances[i];

		// Publish object data.
		pub.publish(r);
	}//: for
}



} //: namespace ROSProxy
} //: namespace Processors
