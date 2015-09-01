/*!
 * \file
 * \brief 
 * \author Tomek Kornuta,,,
 */

#ifndef ROSPROXY_HPP_
#define ROSPROXY_HPP_

#include "Component_Aux.hpp"
#include "Component.hpp"
#include "DataStream.hpp"
#include "Property.hpp"
#include "EventHandler2.hpp"

#include <Types/HomogMatrix.hpp>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/Vertices.h>

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>

namespace Processors {
namespace ROSProxy {

/*!
 * \class ROSProxy
 * \brief ROSProxy processor class.
 *
 *
 */
class ROSProxy: public Base::Component {
public:
	/*!
	 * Constructor.
	 */
	ROSProxy(const std::string & name = "ROSProxy");

	/*!
	 * Destructor
	 */
	virtual ~ROSProxy();

	/*!
	 * Prepare components interface (register streams and handlers).
	 * At this point, all properties are already initialized and loaded to
	 * values set in config file.
	 */
	void prepareInterface();

protected:

	/*!
	 * Connects source to given device.
	 */
	bool onInit();

	/*!
	 * Disconnect source from device, closes streams, etc.
	 */
	bool onFinish();

	/*!
	 * Start component
	 */
	bool onStart();

	/*!
	 * Stop component
	 */
	bool onStop();


	/// Input data stream containing vector of objects ids.
	Base::DataStreamIn <std::vector< std::string>, Base::DataStreamBuffer::Newest, Base::Synchronization::Mutex> in_object_labels;

	/// Input data stream containing vector of poses of objects.
	Base::DataStreamIn<std::vector<Types::HomogMatrix>, Base::DataStreamBuffer::Newest, Base::Synchronization::Mutex>  in_object_poses;

	/// Input data stream containing vector of object vertices (used by polygons and boundingboxes).
	Base::DataStreamIn <std::vector< pcl::PointCloud<pcl::PointXYZ>::Ptr>, Base::DataStreamBuffer::Newest, Base::Synchronization::Mutex> in_object_vertices_xyz;

	/// Input data stream containing vector of object meshes - build on top of vertices.
	Base::DataStreamIn <std::vector< std::vector<pcl::Vertices> >, Base::DataStreamBuffer::Newest, Base::Synchronization::Mutex> in_object_triangles;

	/// Input data stream containing vector of covariances of poses of objects.
	Base::DataStreamIn<std::vector<boost::array<double, 36ul> >, Base::DataStreamBuffer::Newest, Base::Synchronization::Mutex>  in_object_pose_covariances;

	/// Input data stream containing vector of object confidences.
	Base::DataStreamIn<std::vector<double> >  in_object_confidences;


	/// Property: title of the window.
	Base::Property<std::string> prop_parent_frame;


	/// Property: title of the window.
	Base::Property<bool> prop_ros_spin;

	/// Handler - performs ROS spin.
	void spin();

	/// Handler - publishes received poses to ROS TF topics.
	void publishPoses();

	/// Handle to ROS.
	ros::NodeHandle * nh;

	/// ROS publisher.
	ros::Publisher pub;
};

} //: namespace ROSProxy
} //: namespace Processors

/*
 * Register processor component.
 */
REGISTER_COMPONENT("ROSProxy", Processors::ROSProxy::ROSProxy)

#endif /* ROSPROXY_HPP_ */
