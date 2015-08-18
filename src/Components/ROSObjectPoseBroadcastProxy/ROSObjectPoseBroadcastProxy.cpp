/*!
 * \file
 * \brief
 * \author Tomek Kornuta,,,
 */

#include <memory>
#include <string>

#include "ROSObjectPoseBroadcastProxy.hpp"
#include "Common/Logger.hpp"

#include <boost/bind.hpp>

 #include <tf_conversions/tf_eigen.h>


namespace Processors {
namespace ROSObjectPoseBroadcastProxy {

ROSObjectPoseBroadcastProxy::ROSObjectPoseBroadcastProxy(const std::string & name) :
	Base::Component(name),
	prop_parent_frame("parent_frame",std::string("/discode_kinect"))
{
	// General properties.
	registerProperty(prop_parent_frame);

}

ROSObjectPoseBroadcastProxy::~ROSObjectPoseBroadcastProxy() {
}

void ROSObjectPoseBroadcastProxy::prepareInterface() {
	// Register data streams.
	registerStream("in_object_labels", &in_object_labels);
	registerStream("in_object_poses", &in_object_poses);

	// Register handlers
	registerHandler("spin", boost::bind(&ROSObjectPoseBroadcastProxy::spin, this));
	addDependency("spin", NULL);

	registerHandler("publishPoses", boost::bind(&ROSObjectPoseBroadcastProxy::publishPoses, this));
	addDependency("publishPoses", &in_object_poses);
	//addDependency("publishPoses", &in_object_labels); -> not obligatory.
}

bool ROSObjectPoseBroadcastProxy::onInit() {
	// Initialize ROS node.
	static int argc;
	static char * argv = NULL;
	ros::init(argc, &argv, std::string("discode"), ros::init_options::NoSigintHandler);

	return true;
}

bool ROSObjectPoseBroadcastProxy::onFinish() {
	return true;
}

bool ROSObjectPoseBroadcastProxy::onStop() {
	return true;
}

bool ROSObjectPoseBroadcastProxy::onStart() {
	return true;
}

void ROSObjectPoseBroadcastProxy::spin() {
	ros::spinOnce();
}

void ROSObjectPoseBroadcastProxy::publishPoses() {
	// Read inputs.
	std::vector< std::string> object_labels;
	std::vector<Types::HomogMatrix> object_poses = in_object_poses.read();

	// Read object labels.
	if (!in_object_labels.empty())
		object_labels = in_object_labels.read();

	int i=0;
	// Fill vector of temporary names - if required.
	while(object_labels.size() < object_poses.size()) {
		std::ostringstream s;
		s << i++;
		std::string cname = "object_" + s.str();
		object_labels.push_back(cname);
	}//: while

	// Broadcaster.
	static tf::TransformBroadcaster br;
	std::vector< tf::StampedTransform > transforms;

	// Publish poses 1 by 1.
	for (size_t i = 0; i < object_poses.size(); ++i) {
		CLOG(LDEBUG) << "Broadcasting transform: " << object_poses[i];

		// Transform pose to quaternion.
		Eigen::Affine3d pose = object_poses[i];
		tf::Transform transform;
		tf::poseEigenToTF(pose,transform);

		// Generate TF name.
		// std::string name = "/"+object_labels[i];
		std::string name = object_labels[i];

		transforms.push_back(tf::StampedTransform(transform, ros::Time::now(), prop_parent_frame, name));
		CLOG(LERROR) <<"Adding "<<name << " pose with parent " << prop_parent_frame << " to list";
	}//: for
	CLOG(LERROR) <<"Broadcasting "<<transforms.size() << " pose(s) at once";
	// Publish all stamped transforms at once.
	br.sendTransform(transforms);
}



} //: namespace ROSProxy
} //: namespace Processors
