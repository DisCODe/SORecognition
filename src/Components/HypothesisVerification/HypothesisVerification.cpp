/*!
 * \file
 * \brief
 * \author Tomek Kornuta,,,
 */

#include <memory>
#include <string>

#include "HypothesisVerification.hpp"
#include "Common/Logger.hpp"

#include <boost/bind.hpp>

#include <pcl/recognition/hv/greedy_verification.h>

namespace Processors {
namespace HypothesisVerification {

HypothesisVerification::HypothesisVerification(const std::string & name) :
	Base::Component(name),
	prop_greedy_resolution("greedy.resolution", 0.005f),
	prop_greedy_inlier_threshold("greedy.inlier_threshold", 0.005f),
	prop_greedy_lambda("greedy.lambda", 1.5f),
	prop_occlusion_reasoning("occlusion_reasoning", true)
{
	registerProperty(prop_greedy_resolution);
	registerProperty(prop_greedy_inlier_threshold);
	registerProperty(prop_greedy_lambda);
	registerProperty(prop_occlusion_reasoning);


}

HypothesisVerification::~HypothesisVerification() {
}





void HypothesisVerification::prepareInterface() {
	// Register scene-related input streams.
	registerStream("in_scene_cloud_xyzrgb", &in_cloud_xyzrgb);
	registerStream("in_scene_cloud_xyzsift", &in_cloud_xyzsift);

	// Register cluster-related input streams.
	registerStream("in_cluster_labels", &in_cluster_labels);
	registerStream("in_cluster_clouds_xyzrgb", &in_cluster_clouds_xyzrgb);
	registerStream("in_cluster_clouds_xyzsift", &in_cluster_clouds_xyzsift);
	registerStream("in_cluster_corners_xyz", &in_cluster_corners_xyz);
	registerStream("in_clusters_scene_correspondences", &in_clusters_scene_correspondences);
	registerStream("in_cluster_poses", &in_cluster_poses);
	registerStream("in_cluster_confidences", &in_cluster_confidences);

	// Register object-related output streams.
	registerStream("out_object_labels", &out_object_labels);
	registerStream("out_object_clouds_xyzrgb", &out_object_clouds_xyzrgb);
	registerStream("out_object_clouds_xyzsift", &out_object_clouds_xyzsift);
	registerStream("out_object_corners_xyz", &out_object_corners_xyz);
	registerStream("out_objects_scene_correspondences", &out_objects_scene_correspondences);
	registerStream("out_object_poses", &out_object_poses);
	registerStream("out_object_confidences", &out_object_confidences);

	// Register main handler.
	registerHandler("verifyHypothesesXYZRGB", boost::bind(&HypothesisVerification::verifyHypothesesXYZRGB, this));
	addDependency("verifyHypothesesXYZRGB", &in_cloud_xyzrgb);
	addDependency("verifyHypothesesXYZRGB", &in_cluster_clouds_xyzrgb);
	addDependency("verifyHypothesesXYZRGB", &in_cluster_clouds_xyzsift);
	addDependency("verifyHypothesesXYZRGB", &in_cluster_corners_xyz);
	addDependency("verifyHypothesesXYZRGB", &in_clusters_scene_correspondences);
	addDependency("verifyHypothesesXYZRGB", &in_cluster_poses);
	addDependency("verifyHypothesesXYZRGB", &in_cluster_confidences);
//	addDependency("verifyHypothesesXYZRGB", &in_cluster_labels);

}

bool HypothesisVerification::onInit() {

	return true;
}

bool HypothesisVerification::onFinish() {
	return true;
}

bool HypothesisVerification::onStop() {
	return true;
}

bool HypothesisVerification::onStart() {
	return true;
}


void HypothesisVerification::verifyHypothesesXYZRGB() {
	CLOG(LTRACE) << "verifyHypothesesXYZRGB";

	// This is executed when all required data streams are present - no need to check them! ASIDE of in_cluster_labels, which is not required!

	// Read inputs.
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr scene_cloud_xyzrgb = in_cloud_xyzrgb.read();
	std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> cluster_clouds_xyzrgb = in_cluster_clouds_xyzrgb.read();
	std::vector<pcl::PointCloud<PointXYZSIFT>::Ptr> cluster_clouds_xyzsift = in_cluster_clouds_xyzsift.read();
	std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> cluster_corners_xyz = in_cluster_corners_xyz.read();
	std::vector<pcl::CorrespondencesPtr> clusters_scene_correspondences = in_clusters_scene_correspondences.read();
	std::vector<Types::HomogMatrix> cluster_poses = in_cluster_poses.read();
	std::vector<double> cluster_confidences  = in_cluster_confidences.read();

	if(cluster_clouds_xyzrgb.size() == 0){
		CLOG(LWARNING) << "There are no clusters to verify";
		return;
	}//: if


	// Temporary variables containing names.
	std::vector<std::string> labels;

	// Read model labels.
	if (!in_cluster_labels.empty())
		labels = in_cluster_labels.read();

	int i=0;
	// Fill vector of temporary names - if required.
	while(labels.size() < cluster_clouds_xyzsift.size()) {
		std::ostringstream s;
		s << i++;
		std::string cname = "cluster_" + s.str();
		labels.push_back(cname);
	}//: while

	// Mask determining accepted from rejected hypotheses.
	std::vector<bool> mask_hv;

	// Create vector of ConstPtr clouds :]

	std::vector<pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr> const_cluster_clouds_xyzrgb;
	for (size_t i = 0; i < cluster_clouds_xyzrgb.size(); ++i) {
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr tmp_ptr = cluster_clouds_xyzrgb[i];
		pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr tmp_const_ptr (tmp_ptr);
		const_cluster_clouds_xyzrgb.push_back(tmp_const_ptr);
	}

	// Run greedy.
	greedyVerificationXYZRGB(scene_cloud_xyzrgb, const_cluster_clouds_xyzrgb, mask_hv);


	// Temporary variables - for storing returned data.
	std::vector<std::string> all_object_labels;
	std::vector< pcl::PointCloud<pcl::PointXYZRGB>::Ptr> all_object_clouds_xyzrgb;
	std::vector< pcl::PointCloud<PointXYZSIFT>::Ptr> all_object_clouds_xyzsift;
	std::vector< pcl::PointCloud<pcl::PointXYZ>::Ptr> all_object_corners_xyz;
	std::vector<Types::HomogMatrix> all_object_poses;
	std::vector<pcl::CorrespondencesPtr> all_objects_scene_correspondences;
	std::vector<double> all_object_confidences;


	// Copy verified hypotheses data to output ports.
	int obji = 0;
	for(size_t i = 0; i < mask_hv.size(); i++){
		if(mask_hv[i]){
			CLOG(LNOTICE) << "GreedyVerification: Hypothesis " << i << " (" << labels[i] << ") is CORRECT";

			// Copy data.
			all_object_clouds_xyzrgb.push_back(cluster_clouds_xyzrgb[i]);
			all_object_clouds_xyzsift.push_back(cluster_clouds_xyzsift[i]);
			all_object_corners_xyz.push_back(cluster_corners_xyz[i]);
			all_object_poses.push_back(cluster_poses[i]);
			all_objects_scene_correspondences.push_back(clusters_scene_correspondences[i]);
			all_object_confidences.push_back(cluster_confidences[i]);


			// Process name.
			std::string basis = labels[i].substr(0,labels[i].rfind("cluster_"));
			std::ostringstream s;
			s << obji++;
			std::string cname = basis +"object_" + s.str();
			all_object_labels.push_back(cname);
		}
		else{
			CLOG(LINFO) << "GreedyVerification: Hypothese " << i << " is NOT correct";
		}

	}

	// Write result to output ports.
	out_object_clouds_xyzrgb.write(all_object_clouds_xyzrgb);
	out_object_clouds_xyzsift.write(all_object_clouds_xyzsift);
	out_object_corners_xyz.write(all_object_corners_xyz);
	out_object_poses.write(all_object_poses);
	out_objects_scene_correspondences.write(all_objects_scene_correspondences);
	out_object_labels.write(all_object_labels);
	out_object_confidences.write(all_object_confidences);

}


void HypothesisVerification::greedyVerificationXYZRGB(pcl::PointCloud<pcl::PointXYZRGB>::Ptr scene_cloud_xyzrgb_, std::vector<pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr> cluster_clouds_xyzrgb_, std::vector<bool> & mask_hv_) {
	CLOG(LTRACE) << "greedyVerificationXYZRGB";

	// Initialize verification algorithm.
	pcl::GreedyVerification<pcl::PointXYZRGB, pcl::PointXYZRGB> greedy_hv(prop_greedy_lambda);
	// Set parameters.
	greedy_hv.setResolution (prop_greedy_resolution);
	greedy_hv.setInlierThreshold (prop_greedy_inlier_threshold);
	// Add clouds.
	greedy_hv.setSceneCloud (scene_cloud_xyzrgb_);
	greedy_hv.addModels (cluster_clouds_xyzrgb_, prop_occlusion_reasoning);


	// Perform verification.
	greedy_hv.verify ();

	// Get results.
	greedy_hv.getMask (mask_hv_);
}


} //: namespace HypothesisVerification
} //: namespace Processors
