/*!
 * \file
 * \brief 
 * \author Tomek Kornuta,,,
 */

#ifndef CORRESPONDENCEGROUPING_HPP_
#define CORRESPONDENCEGROUPING_HPP_

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
namespace CorrespondenceGrouping {

/*!
 * \class CorrespondenceGrouping
 * \brief CorrespondenceGrouping processor class.
 *
 * CorrespondenceGrouping processor.
 */
class CorrespondenceGrouping: public Base::Component {
public:
	/*!
	 * Constructor.
	 */
	CorrespondenceGrouping(const std::string & name = "CorrespondenceGrouping");

	/*!
	 * Destructor
	 */
	virtual ~CorrespondenceGrouping();

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

	/// Input data stream containing vector of object/model labels.
	Base::DataStreamIn <std::vector< std::string>, Base::DataStreamBuffer::Newest, Base::Synchronization::Mutex> in_model_labels;

	/// Input data stream containing vector of XYZRGB clouds (objects/models).
	Base::DataStreamIn <std::vector< pcl::PointCloud<pcl::PointXYZRGB>::Ptr>, Base::DataStreamBuffer::Newest, Base::Synchronization::Mutex> in_model_clouds_xyzrgb;

	/// Input data stream containing vector of XYZSIFT clouds (objects/models).
	Base::DataStreamIn <std::vector< pcl::PointCloud<PointXYZSIFT>::Ptr>, Base::DataStreamBuffer::Newest, Base::Synchronization::Mutex> in_model_clouds_xyzsift;

	/// Input data stream containing vector of model corners (each being a cloud containing 8 XYZ points).
	Base::DataStreamIn <std::vector< pcl::PointCloud<pcl::PointXYZ>::Ptr>, Base::DataStreamBuffer::Newest, Base::Synchronization::Mutex> in_model_corners_xyz;

	/// Input data stream containing vector of corespondences beetwen models and scene clouds.
	Base::DataStreamIn<std::vector<pcl::CorrespondencesPtr>, Base::DataStreamBuffer::Newest, Base::Synchronization::Mutex> in_models_scene_correspondences;


	/// Output data stream containing vector of clusters (matched models) labels.
	Base::DataStreamOut <std::vector< std::string> > out_cluster_labels;

	/// Output data stream containing vector of clusters (matched models) XYZRGB clouds.
	Base::DataStreamOut <std::vector< pcl::PointCloud<pcl::PointXYZRGB>::Ptr> > out_cluster_clouds_xyzrgb;

	/// Output data stream containing vector of clusters (matched models) XYZSIFT clouds.
	Base::DataStreamOut <std::vector< pcl::PointCloud<PointXYZSIFT>::Ptr> > out_cluster_clouds_xyzsift;

	/// Output data stream containing vector of clusters (matched models) corners (each being a cloud containing 8 XYZ points).
	Base::DataStreamOut <std::vector< pcl::PointCloud<pcl::PointXYZ>::Ptr> > out_cluster_corners_xyz;

	/// Output data stream containing vector of corespondences beetwen objects and scene clouds (clusters).
	Base::DataStreamOut<std::vector<pcl::CorrespondencesPtr> > out_clusters_scene_correspondences;

	/// Output data stream containing vector of poses of clusters (matched models).
	Base::DataStreamOut<std::vector<Types::HomogMatrix> >  out_cluster_poses;

	/// Output data stream containing vector of cluster confidences.
	Base::DataStreamOut<std::vector<double> >  out_cluster_confidences;


	/// Property: the consensus set resolution, in metric units.
	Base::Property<double> prop_cg_consensus_resolution;
	/// Property: the minimum cluster size.
	Base::Property<double> prop_cg_minimal_cluster_size;

	/// Property: print results (cluster statistics) in console.
	Base::Property<bool> prop_print_cluster_statistics;

	/// Main handler - groups correspondences.
	void groupCorrespondences();

	/// Group correspondences between a single model and scene.
	void groupSingleModelCorrespondences(pcl::PointCloud<PointXYZSIFT>::Ptr model_clouds_xyzsift_, pcl::PointCloud<PointXYZSIFT>::Ptr cloud_xyzsift_, pcl::CorrespondencesPtr model_scene_correspondences_, std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> > & cluster_poses_, std::vector<pcl::CorrespondencesPtr> & cluster_correspondences_, std::vector<double> & cluster_confidences_);

	/// Prints results of clustering (number of clusters, their sizes, transformations etc.).
	void printCorrespondencesGroups (std::vector<Types::HomogMatrix> cluster_poses_, std::vector<pcl::CorrespondencesPtr> cluster_correspondences_, std::vector<std::string> cluster_labels_, std::vector<double> cluster_confidences_);

};

} //: namespace CorrespondenceGrouping
} //: namespace Processors

/*
 * Register processor component.
 */
REGISTER_COMPONENT("CorrespondenceGrouping", Processors::CorrespondenceGrouping::CorrespondenceGrouping)

#endif /* CORRESPONDENCEGROUPING_HPP_ */
