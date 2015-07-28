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
		Base::Component(name)  {

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
	registerStream("in_model_ids", &in_model_ids);
	registerStream("in_model_clouds_xyzrgb", &in_model_clouds_xyzrgb);
	registerStream("in_model_clouds_xyzsift", &in_model_clouds_xyzsift);
	registerStream("in_model_corners_xyz", &in_model_corners_xyz);

	// Register cloud model correspondences input streams.
	registerStream("in_models_scene_correspondences", &in_models_scene_correspondences);

	// Register obcjet cloud output streams.
	registerStream("out_object_ids", &out_object_ids);
	registerStream("out_object_clouds_xyzrgb", &out_object_clouds_xyzrgb);
	registerStream("out_object_clouds_xyzsift", &out_object_clouds_xyzsift);
	registerStream("out_object_corners_xyz", &out_object_corners_xyz);

	// Register objects scene correspondences output streams.
	registerStream("in_objects_scene_correspondences", &out_objects_scene_correspondences);



	registerHandler("groupCorrespondences", boost::bind(&CorrespondenceGrouping::groupCorrespondences, this));
//	addDependency("groupCorrespondences", &in_model_ids);
	addDependency("groupCorrespondences", &in_model_clouds_xyzrgb);
	addDependency("groupCorrespondences", &in_model_clouds_xyzsift);
	addDependency("groupCorrespondences", &in_model_corners_xyz);
	addDependency("groupCorrespondences", &in_models_scene_correspondences);

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

	// This is executed when all required data streams are present - no need to check them!
	// TODO MODEL ID's!

	// Read inputs.
	pcl::PointCloud<PointXYZSIFT>::Ptr scene_cloud_xyzsift = in_cloud_xyzsift.read();
	std::vector<pcl::PointCloud<PointXYZSIFT>::Ptr> models_clouds_xyzsift = in_model_clouds_xyzsift.read();
	std::vector<pcl::CorrespondencesPtr> models_scene_correspondences = in_models_scene_correspondences.read();


	// Iterate through model clouds.
	for(int i=0; i< models_clouds_xyzsift.size(); i++) {
		groupSingleModelCorrespondences(models_clouds_xyzsift[i], scene_cloud_xyzsift, models_scene_correspondences[i]);
	}//: for
}



void CorrespondenceGrouping::groupSingleModelCorrespondences(pcl::PointCloud<PointXYZSIFT>::Ptr model_clouds_xyzsift_, pcl::PointCloud<PointXYZSIFT>::Ptr in_cloud_xyzsift_, pcl::CorrespondencesPtr model_scene_correspondences_) {

	double inlier_threshold = 0.001f;
	double cg_size = 0.01f;
	double cg_thresh = 5.0f;

	//Algorithm params
	float rf_rad_ (0.015f);

	std::vector< Eigen::Matrix<float, 4, 4>, Eigen::aligned_allocator<Eigen::Matrix<float, 4, 4> > > poses;
	std::vector<pcl::Correspondences> clustered_corrs;

//  Clustering
/*    if(use_hough3d){//nie działa :(
	CLOG(LINFO) << "Using Hough3DGrouping";
	pcl::Hough3DGrouping<PointXYZSIFT, PointXYZSIFT, pcl::ReferenceFrame, pcl::ReferenceFrame> clusterer;
	clusterer.setHoughBinSize (cg_size);
	clusterer.setHoughThreshold (cg_thresh);
	clusterer.setUseInterpolation (true);
	clusterer.setUseDistanceWeight (false);

	clusterer.setLocalRfSearchRadius(rf_rad_);
	clusterer.setInputCloud (models[i]->cloud_xyzsift);
	//clusterer.setInputRf (model_rf);
	clusterer.setSceneCloud (cloud_xyzsift);
	//clusterer.setSceneRf (scene_rf);
	clusterer.setModelSceneCorrespondences (correspondences);

//        clusterer.cluster (clustered_corrs);//tu sie wywala
	clusterer.recognize (poses, clustered_corrs);//tu sie wywala

	CLOG(LINFO) << "Model instances found: " << poses.size ();
	for (size_t j = 0; j < poses.size (); ++j)
	{
		Eigen::Matrix3f rotation = poses[j].block<3,3>(0, 0);
		Eigen::Vector3f translation = poses[j].block<3,1>(0, 3);
		if(rotation != Eigen::Matrix3f::Identity()){
			CLOG(LINFO) << "\n    Instance " << j + 1 << ":";
			CLOG(LINFO) << "        Correspondences belonging to this instance: " << clustered_corrs[j].size () ;
			// Print the rotation matrix and translation vector
			CLOG(LINFO) << "            | "<< rotation (0,0)<<" "<< rotation (0,1)<< " "<<rotation (0,2)<<" | ";
			CLOG(LINFO) << "        R = | "<< rotation (1,0)<<" "<< rotation (1,1)<< " "<<rotation (1,2)<<" | ";
			CLOG(LINFO) << "            | "<< rotation (2,0)<<" "<< rotation (2,1)<< " "<<rotation (2,2)<<" | ";
			CLOG(LINFO) << "        t = < "<< translation (0)<<" "<< translation (1)<< " "<< translation (2)<<" > ";
		}
	}
}
else */{
	CLOG(LINFO) << "Using GeometricConsistencyGrouping";
// Using GeometricConsistency
	pcl::GeometricConsistencyGrouping<PointXYZSIFT, PointXYZSIFT> gc_clusterer;
	gc_clusterer.setGCSize (cg_size);
	gc_clusterer.setGCThreshold (cg_thresh);
	gc_clusterer.setInputCloud (model_clouds_xyzsift_);
	gc_clusterer.setSceneCloud (in_cloud_xyzsift_);
	gc_clusterer.setModelSceneCorrespondences (model_scene_correspondences_);

	//gc_clusterer.cluster (clustered_corrs);
	gc_clusterer.recognize (poses, clustered_corrs);
	CLOG(LINFO) << "Model instances found: " << poses.size () ;
//            cout<<"clustered_corrs "<< clustered_corrs.size();
//            for(int j=0; j<clustered_corrs.size(); j++){
//                for(int k =0; k<clustered_corrs[j].size();k++){
//                    cout<<clustered_corrs[j][k].index_query<<"-"<<clustered_corrs[j][k].index_match<<", ";
//                }
//                cout<<"-----------"<<endl;
//            }
		for (size_t k = 0; k < poses.size (); ++k){
			  Eigen::Matrix3f rotation = poses[k].block<3,3>(0, 0);
			  Eigen::Vector3f translation = poses[k].block<3,1>(0, 3);
			  if(rotation != Eigen::Matrix3f::Identity()){
				  CLOG(LINFO) << "\n    Instance " << k + 1 << ":";
				  CLOG(LINFO) << "        Correspondences belonging to this instance: " << clustered_corrs[k].size () ;
				  //Print the rotation matrix and translation vector
				  CLOG(LINFO) << "            | "<< rotation (0,0)<<" "<< rotation (0,1)<< " "<<rotation (0,2)<<" | ";
				  CLOG(LINFO) << "        R = | "<< rotation (1,0)<<" "<< rotation (1,1)<< " "<<rotation (1,2)<<" | ";
				  CLOG(LINFO) << "            | "<< rotation (2,0)<<" "<< rotation (2,1)<< " "<<rotation (2,2)<<" | ";
				  CLOG(LINFO) << "        t = < "<< translation (0)<<" "<< translation (1)<< " "<< translation (2)<<" > ";
			  }//: if
		}//: for
	}//: else
}



} //: namespace CorrespondenceGrouping
} //: namespace Processors
