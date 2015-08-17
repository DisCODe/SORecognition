/*!
 * \file
 * \brief 
 * \author Tomek Kornuta,,,
 */

#ifndef ROSOBJECTBROADCASTPROXY_HPP_
#define ROSOBJECTBROADCASTPROXY_HPP_

#include "Component_Aux.hpp"
#include "Component.hpp"
#include "DataStream.hpp"
#include "Property.hpp"
#include "EventHandler2.hpp"

#include <Types/HomogMatrix.hpp>

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>

namespace Processors {
namespace ROSObjectPoseBroadcastProxy {

/*!
 * \class ROSObjectPoseBroadcastProxy
 * \brief ROSObjectPoseBroadcastProxy processor class.
 *
 *
 */
class ROSObjectPoseBroadcastProxy: public Base::Component {
public:
	/*!
	 * Constructor.
	 */
	ROSObjectPoseBroadcastProxy(const std::string & name = "ROSProxy");

	/*!
	 * Destructor
	 */
	virtual ~ROSObjectPoseBroadcastProxy();

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


	/// Input data stream containing vector of objects/clusters/models ids.
	Base::DataStreamIn <std::vector< std::string>, Base::DataStreamBuffer::Newest, Base::Synchronization::Mutex> in_object_labels;

	/// Input data stream containing vector of poses of objects/clusters/models.
	Base::DataStreamIn<std::vector<Types::HomogMatrix>, Base::DataStreamBuffer::Newest, Base::Synchronization::Mutex>  in_object_poses;


	/// Property: title of the window.
	Base::Property<std::string> prop_parent_frame;


	/// Handler - performs ROS spin.
	void spin();

	/// Handler - publishes received poses to ROS TF topics.
	void publishPoses();

};

} //: namespace ROSProxy
} //: namespace Processors

/*
 * Register processor component.
 */
REGISTER_COMPONENT("ROSObjectPoseBroadcastProxy", Processors::ROSObjectPoseBroadcastProxy::ROSObjectPoseBroadcastProxy)

#endif /* ROSOBJECTBROADCASTPROXY_HPP_ */
