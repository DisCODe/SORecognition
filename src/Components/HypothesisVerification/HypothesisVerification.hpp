/*!
 * \file
 * \brief 
 * \author Tomek Kornuta,,,
 */

#ifndef HYPOTHESISVERIFICATION_HPP_
#define HYPOTHESISVERIFICATION_HPP_

#include "Component_Aux.hpp"
#include "Component.hpp"
#include "DataStream.hpp"
#include "Property.hpp"
#include "EventHandler2.hpp"

#include <pcl/correspondence.h>

#include <Types/HomogMatrix.hpp>
#include <Types/PointXYZSIFT.hpp>
#include <Types/SIFTFeatureRepresentation.hpp>


namespace Processors {
namespace HypothesisVerification {

/*!
 * \class HypothesisVerification
 * \brief HypothesisVerification processor class.
 *
 * HypothesisVerification processor.
 */
class HypothesisVerification: public Base::Component {
public:
	/*!
	 * Constructor.
	 */
	HypothesisVerification(const std::string & name = "HypothesisVerification");

	/*!
	 * Destructor
	 */
	virtual ~HypothesisVerification();

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

	/// Data stream with scene cloud of XYZRGB points.
	Base::DataStreamIn<pcl::PointCloud<pcl::PointXYZRGB>::Ptr, Base::DataStreamBuffer::Newest, Base::Synchronization::Mutex> in_cloud_xyzrgb;

	/// Data stream with scene cloud of XYZ SIFTs.
	Base::DataStreamIn<pcl::PointCloud<PointXYZSIFT>::Ptr, Base::DataStreamBuffer::Newest, Base::Synchronization::Mutex> in_cloud_xyzsift;

	/// Input data stream containing vector of clusters (matched models) labels.
	Base::DataStreamIn <std::vector< std::string>, Base::DataStreamBuffer::Newest, Base::Synchronization::Mutex> in_cluster_labels;

	/// Input data stream containing vector of clusters (matched models) XYZRGB clouds.
	Base::DataStreamIn <std::vector< pcl::PointCloud<pcl::PointXYZRGB>::Ptr>, Base::DataStreamBuffer::Newest, Base::Synchronization::Mutex> in_cluster_clouds_xyzrgb;

	/// Input data stream containing vector of clusters (matched models) XYZSIFT clouds.
	Base::DataStreamIn <std::vector< pcl::PointCloud<PointXYZSIFT>::Ptr>, Base::DataStreamBuffer::Newest, Base::Synchronization::Mutex> in_cluster_clouds_xyzsift;

	/// Input data stream containing vector of clusters (matched models) corners (each being a cloud containing 8 XYZ points).
	Base::DataStreamIn <std::vector< pcl::PointCloud<pcl::PointXYZ>::Ptr>, Base::DataStreamBuffer::Newest, Base::Synchronization::Mutex> in_cluster_corners_xyz;

	/// Input data stream containing vector of corespondences beetwen objects and scene clouds (clusters).
	Base::DataStreamIn<std::vector<pcl::CorrespondencesPtr>, Base::DataStreamBuffer::Newest, Base::Synchronization::Mutex> in_clusters_scene_correspondences;

	/// Input data stream containing vector of poses of clusters (matched models).
	Base::DataStreamIn<std::vector<Types::HomogMatrix>, Base::DataStreamBuffer::Newest, Base::Synchronization::Mutex>  in_cluster_poses;



	/// Output data stream containing vector of object labels.
	Base::DataStreamOut <std::vector< std::string> > out_object_labels;

	/// Output data stream containing vector of object XYZRGB clouds.
	Base::DataStreamOut <std::vector< pcl::PointCloud<pcl::PointXYZRGB>::Ptr> > out_object_clouds_xyzrgb;

	/// Output data stream containing vector of object XYZSIFT clouds.
	Base::DataStreamOut <std::vector< pcl::PointCloud<PointXYZSIFT>::Ptr> > out_object_clouds_xyzsift;

	/// Output data stream containing vector of object corners (each being a cloud containing 8 XYZ points).
	Base::DataStreamOut <std::vector< pcl::PointCloud<pcl::PointXYZ>::Ptr> > out_object_corners_xyz;

	/// Output data stream containing vector of corespondences beetwen objects and scene clouds.
	Base::DataStreamOut<std::vector<pcl::CorrespondencesPtr> > out_objects_scene_correspondences;

	/// Output data stream containing vector of poses of object.
	Base::DataStreamOut<std::vector<Types::HomogMatrix> >  out_object_poses;



	/// Property of the greedy verification algorithm: resolution?
	Base::Property<float> prop_greedy_resolution;

	/// Property of the greedy verification algorithm: inliers_threshold?
	Base::Property<float> prop_greedy_inlier_threshold;

	/// Property of the greedy verification algorithm:lambda?
	Base::Property<float> prop_greedy_lambda;

	/// Property: occlusion reasoning.
	Base::Property<bool> prop_occlusion_reasoning;


	/// Main handler - verify hypotheses on the basis of XYZRGB scene and model clouds.
	void verifyHypothesesXYZRGB();

	/// Executes the greedy verification algorithm.
	void greedyVerificationXYZRGB(pcl::PointCloud<pcl::PointXYZRGB>::Ptr scene_cloud_xyzrgb_, std::vector<pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr> cluster_clouds_xyzrgb_, std::vector<bool> & mask_hv_);

};

} //: namespace HypothesisVerification
} //: namespace Processors

/*
 * Register processor component.
 */
REGISTER_COMPONENT("HypothesisVerification", Processors::HypothesisVerification::HypothesisVerification)

#endif /* HYPOTHESISVERIFICATION_HPP_ */
