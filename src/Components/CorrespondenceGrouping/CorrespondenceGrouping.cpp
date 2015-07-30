/*!
 * \file
 * \brief
 * \author Tomek Kornuta,,,
 */

#include <memory>
#include <string>

#include "CorrespondenceGrouping.hpp"
#include "Common/Logger.hpp"

#include <boost/bind.hpp>

#include <pcl/recognition/cg/hough_3d.h>
#include <pcl/recognition/cg/geometric_consistency.h>

namespace Processors {
namespace CorrespondenceGrouping {

CorrespondenceGrouping::CorrespondenceGrouping(const std::string & name) :
	Base::Component(name),
	prop_cg_size("cluster_grouping.size", 0.01),
	prop_cg_thresh("cluster_grouping.threshold", 5.0)
{
	registerProperty(prop_cg_size);
	registerProperty(prop_cg_thresh);
}

CorrespondenceGrouping::~CorrespondenceGrouping() {
}

void CorrespondenceGrouping::prepareInterface() {
	// Register basic cloud input streams.
	registerStream("in_cloud_xyzrgb", &in_cloud_xyzrgb);
	registerStream("in_cloud_xyzsift", &in_cloud_xyzsift);

	// Register input cloud "scene" aliases.
	registerStream("in_scene_cloud_xyzrgb", &in_cloud_xyzrgb);
	registerStream("in_scene_cloud_xyzsift", &in_cloud_xyzsift);

	// Register cloud model input streams.
	registerStream("in_model_labels", &in_model_labels);
	registerStream("in_model_clouds_xyzrgb", &in_model_clouds_xyzrgb);
	registerStream("in_model_clouds_xyzsift", &in_model_clouds_xyzsift);
	registerStream("in_model_corners_xyz", &in_model_corners_xyz);

	// Register cloud model correspondences input streams.
	registerStream("in_models_scene_correspondences", &in_models_scene_correspondences);

	// Register clusters (matched models) output streams.
	registerStream("out_cluster_labels", &out_cluster_labels);
	registerStream("out_cluster_clouds_xyzrgb", &out_cluster_clouds_xyzrgb);
	registerStream("out_cluster_clouds_xyzsift", &out_cluster_clouds_xyzsift);
	registerStream("out_cluster_corners_xyz", &out_cluster_corners_xyz);
	registerStream("out_clusters_scene_correspondences", &out_clusters_scene_correspondences);
	registerStream("out_cluster_poses", &out_cluster_poses);

	// Register objects scene correspondences output streams.
	//registerStream("in_objects_scene_correspondences", &out_clusters_scene_correspondences);



	registerHandler("groupCorrespondences", boost::bind(&CorrespondenceGrouping::groupCorrespondences, this));
	addDependency("groupCorrespondences", &in_cloud_xyzsift);
	addDependency("groupCorrespondences", &in_model_clouds_xyzrgb);
	addDependency("groupCorrespondences", &in_model_clouds_xyzsift);
	addDependency("groupCorrespondences", &in_model_corners_xyz);
	addDependency("groupCorrespondences", &in_models_scene_correspondences);
//	addDependency("groupCorrespondences", &in_model_labels);

}

bool CorrespondenceGrouping::onInit() {

	return true;
}

bool CorrespondenceGrouping::onFinish() {
	return true;
}

bool CorrespondenceGrouping::onStop() {
	return true;
}

bool CorrespondenceGrouping::onStart() {
	return true;
}


void CorrespondenceGrouping::groupCorrespondences() {
	CLOG(LTRACE) << "groupCorrespondences";

	// This is executed when all required data streams are present - no need to check them! ASIDE of in_model_labels, which is not required!

	// Read inputs.
	pcl::PointCloud<PointXYZSIFT>::Ptr scene_cloud_xyzsift = in_cloud_xyzsift.read();
	std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> model_clouds_xyzrgb = in_model_clouds_xyzrgb.read();
	std::vector<pcl::PointCloud<PointXYZSIFT>::Ptr> model_clouds_xyzsift = in_model_clouds_xyzsift.read();
	std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> model_corners_xyz = in_model_corners_xyz.read();
	std::vector<pcl::CorrespondencesPtr> models_scene_correspondences = in_models_scene_correspondences.read();

	// Temporary variables containing names.
	std::vector<std::string> labels;

	// Read model labels.
	if (!in_model_labels.empty())
		labels = in_model_labels.read();

	std::string basis = "model";
	int i=0;

	// Fill vector of temporary names - if required.
	while(labels.size() < model_clouds_xyzsift.size()) {
		std::ostringstream s;
		s << i++;
		std::string cname = "model_" + s.str();
		labels.push_back(cname);
	}//: while

	// Temporary variables - for storing returned data.
	std::vector<std::string> all_clusters_labels;
	std::vector<Types::HomogMatrix> all_cluster_poses;
	std::vector<pcl::CorrespondencesPtr> all_cluster_correspondences;

	std::vector< pcl::PointCloud<pcl::PointXYZRGB>::Ptr> all_cluster_clouds_xyzrgb;
	std::vector< pcl::PointCloud<PointXYZSIFT>::Ptr> all_cluster_clouds_xyzsift;
	std::vector< pcl::PointCloud<pcl::PointXYZ>::Ptr> all_cluster_corners_xyz;


	// Iterate through model clouds.redbull_rgbdrainbow_kinect/
	for(size_t imd=0; imd< model_clouds_xyzsift.size(); imd++) {
		CLOG(LDEBUG) << "models_clouds_xyzsift.size()=" << model_clouds_xyzsift.size() << " models_scene_correspondences.size()=" << models_scene_correspondences.size() << "i=" << imd;

		// Variables containing cluster data for given model.
		std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> > cluster_poses;
		//std::vector<pcl::Correspondences> cluster_correspondences;
		std::vector<pcl::CorrespondencesPtr> cluster_correspondences;


		// Group correspondences for given model.
		groupSingleModelCorrespondences(model_clouds_xyzsift[imd], scene_cloud_xyzsift, models_scene_correspondences[imd], cluster_poses, cluster_correspondences);


		for (size_t igr = 0; igr < cluster_poses.size(); ++igr) {
			CLOG(LDEBUG) << "Processing " <<igr<<"-th cluster";

			// Get pose and correspondences.
			Eigen::Matrix4f pose = cluster_poses[igr];
			Types::HomogMatrix hm (pose);

			//CLOG(LERROR) << "hm.isIdentity()=" << hm.isIdentity() << " correspondences->size()="<< correspondences->size() << "\n" << hm;
			// Check them - if identity matrix and 2 correspondences - then it is invalid, hence skip it.
			if((hm.isIdentity()) && ((cluster_correspondences[igr])->size() <= 2)) {
					//CLOG(LERROR) << "Skipping!!";
					continue;
			}//: if
			// Otherwise - add cluster.

			// Generate group name - add postfix to model.
			std::ostringstream s;
			s << igr;
			std::string cname = labels[imd] + "_cluster_" + s.str();

			// Add cluster data to vectors.
			all_clusters_labels.push_back(cname);
			all_cluster_poses.push_back(hm);
			all_cluster_correspondences.push_back(cluster_correspondences[igr]);

			// Copy and add XYZRGB, XYZSIFT and corners clouds.
			pcl::PointCloud<pcl::PointXYZRGB>::Ptr tmp_cloud_xyzrgb (model_clouds_xyzrgb[imd]);
			all_cluster_clouds_xyzrgb.push_back(tmp_cloud_xyzrgb);

			pcl::PointCloud<PointXYZSIFT>::Ptr tmp_cloud_xyzsift (model_clouds_xyzsift[imd]);
			all_cluster_clouds_xyzsift.push_back(tmp_cloud_xyzsift);

			pcl::PointCloud<pcl::PointXYZ>::Ptr tmp_cloud_xyz (model_corners_xyz[imd]);
			all_cluster_corners_xyz.push_back(tmp_cloud_xyz);
		}//: for model clusters

	}//: for all models

	// Display results.
	displayCorrespondencesGroups(all_cluster_poses, all_cluster_correspondences, all_clusters_labels);

	// Write result to output ports.
	out_cluster_labels.write(all_clusters_labels);
	out_cluster_clouds_xyzrgb.write(all_cluster_clouds_xyzrgb);
	out_cluster_clouds_xyzsift.write(all_cluster_clouds_xyzsift);
	out_cluster_corners_xyz.write(all_cluster_corners_xyz);
	out_clusters_scene_correspondences.write(all_cluster_correspondences);
	out_cluster_poses.write(all_cluster_poses);

}




void CorrespondenceGrouping::groupSingleModelCorrespondences(pcl::PointCloud<PointXYZSIFT>::Ptr model_clouds_xyzsift_, pcl::PointCloud<PointXYZSIFT>::Ptr cloud_xyzsift_, pcl::CorrespondencesPtr model_scene_correspondences_, std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> > & cluster_poses_, std::vector<pcl::CorrespondencesPtr> & cluster_correspondences_) {
	CLOG(LTRACE) << "groupSingleModelCorrespondences";

	CLOG(LDEBUG) << "Model cloud size=" << model_clouds_xyzsift_->size() << " Scene cloud size=" << cloud_xyzsift_->size() << " Model2Scene correspondences size=" << model_scene_correspondences_->size();

	// Vector used for final enumeration index_mateches of grouped correspondences/
	std::vector<int> index_match_enumeration;

	// Resize the scene cloud so it contains only points having correspondences.
	pcl::PointCloud<PointXYZSIFT>::Ptr resized_cloud_xyzsift (new pcl::PointCloud<PointXYZSIFT>());
	// This also requires to reindex points in correspondences.
	pcl::CorrespondencesPtr resized_correspondences(new pcl::Correspondences()) ;

	// Iterate through the found correspondences.
	if (cloud_xyzsift_->size() > model_scene_correspondences_->size()) {
		for (size_t i = 0; i < model_scene_correspondences_->size(); ++i) {
			// Get i-th corresponcende.
			pcl::Correspondence corr = model_scene_correspondences_->at(i);
			int index = corr.index_match;
			// Remember index_match;
			index_match_enumeration.push_back(index);
			// CLOG(LDEBUG) << "i=" << i << " corr.index_query=" << corr.index_query << " corr.index_match=" << corr.index_match << " corr.index_match=" << corr.distance;
			// Copy point to resized cloud.
			resized_cloud_xyzsift->push_back( cloud_xyzsift_-> at(index) );
			// Change scene indices MODEL 2 SCENE fix.
			pcl::Correspondence corr2 (corr.index_query, static_cast<int> (i), corr.distance);
			// CLOG(LDEBUG) << "i=" << i << " corr2.index_query=" << corr2.index_query << " corr2.index_match=" << corr2.index_match << " corr2.index_match=" << corr.distance;
			// Add correspondence to resized correspondences set.
			resized_correspondences->push_back (corr2);
		}//: for
	} else {
		resized_cloud_xyzsift = cloud_xyzsift_;
		resized_correspondences = model_scene_correspondences_;
	}//: else

	CLOG(LDEBUG) << "Scene cloud after resize: " << resized_cloud_xyzsift->size ();



/*
//  Clustering by Hough 3D voting.
	/// Property: search radius for the potential Rf calculation.
	float prop_h3d_rf_rad (0.015f);

	if(use_hough3d){//nie działa :(
	CLOG(LINFO) << "Using Hough3DGrouping";
	pcl::Hough3DGrouping<PointXYZSIFT, PointXYZSIFT, pcl::ReferenceFrame, pcl::ReferenceFrame> clusterer;
	clusterer.setHoughBinSize (cg_size);
	clusterer.setHoughThreshold (cg_thresh);
	clusterer.setUseInterpolation (true);
	clusterer.setUseDistanceWeight (false);

	clusterer.setLocalRfSearchRadius(prop_h3d_rf_rad);
	clusterer.setInputCloud (models[i]->cloud_xyzsift);
	//clusterer.setInputRf (model_rf);
	clusterer.setSceneCloud (cloud_xyzsift);
	//clusterer.setSceneRf (scene_rf);
	clusterer.setModelSceneCorrespondences (correspondences);

//        clusterer.cluster (clustered_corrs);//tu sie wywala
	clusterer.recognize (poses, clustered_corrs);//tu sie wywala
*/

	CLOG(LDEBUG) << "Using GeometricConsistencyGrouping";
	// Using GeometricConsistency
	pcl::GeometricConsistencyGrouping<PointXYZSIFT, PointXYZSIFT> gc_clusterer;

	// Set algorithm parameters.
	gc_clusterer.setGCSize (prop_cg_size);
	gc_clusterer.setGCThreshold (prop_cg_thresh);

	// Set model and scene clouds.
	gc_clusterer.setInputCloud (model_clouds_xyzsift_);
	gc_clusterer.setSceneCloud (resized_cloud_xyzsift);

	// Cluster MODEL 2 SCENE correspondences.
	gc_clusterer.setModelSceneCorrespondences (resized_correspondences);

	std::vector<pcl::Correspondences> tmp_clusters_correspondences;

	// Perform correspondences grouping.
	gc_clusterer.recognize (cluster_poses_, tmp_clusters_correspondences);

	// Reenumerate index_matches - if clouds were resized.

	if (cloud_xyzsift_->size() > model_scene_correspondences_->size()) {
		// For each cluster.
		for (size_t clui = 0; clui < tmp_clusters_correspondences.size(); ++clui){
			// For each cluster correspondence.
			// Create new correspondences pointer.
			pcl::CorrespondencesPtr tmp_correspondences (new pcl::Correspondences());
			for (size_t cori = 0; cori < tmp_clusters_correspondences[clui].size(); ++cori){
				// Get (cor)i-th corresponcende from (clu)i-th cluster.
				pcl::Correspondence corr = (tmp_clusters_correspondences[clui]).at(cori);
				// CLOG(LDEBUG) << "i=" << i << " corr.index_query=" << corr.index_query << " corr.index_match=" << corr.index_match << " corr.index_match=" << corr.distance;

				// Change scene indices - back to original indices.
				pcl::Correspondence corr2 (corr.index_query, index_match_enumeration[corr.index_match], corr.distance);
				// CLOG(LDEBUG) << "i=" << i << " corr2.index_query=" << corr2.index_query << " corr2.index_match=" << corr2.index_match << " corr2.index_match=" << corr.distance;
				// Add correspondence to resized correspondences set.
				tmp_correspondences->push_back (corr2);
			}//: for
			// Add cluster of correspondences to vector.
			cluster_correspondences_.push_back(tmp_correspondences);
		}//: for
	} else {
		// If not, simply "point" the found vector of clusters of correspondences.
		for (size_t i = 0; i < tmp_clusters_correspondences.size(); ++i){
			pcl::Correspondences tmp_cluster = tmp_clusters_correspondences[i];
			pcl::CorrespondencesPtr tmp_cluster_ptr (new pcl::Correspondences ());
			*tmp_cluster_ptr = tmp_cluster;
			cluster_correspondences_.push_back(tmp_cluster_ptr);
		}//: for
	}//: else

	CLOG(LINFO) << "Groups found: " << cluster_poses_.size () ;

}


void CorrespondenceGrouping::displayCorrespondencesGroups (std::vector<Types::HomogMatrix> cluster_poses_, std::vector<pcl::CorrespondencesPtr> cluster_correspondences_, std::vector<std::string> cluster_labels_) {
	for (size_t k = 0; k < cluster_poses_.size (); ++k){
			  CLOG(LINFO) << "Group (" << k << "): " << cluster_labels_[k] << " (with " << cluster_correspondences_[k]->size () << " correspondences) \n" << cluster_poses_[k];
	}//: for
}



} //: namespace CorrespondenceGrouping
} //: namespace Processors
