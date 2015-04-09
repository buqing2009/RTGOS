#include <pcl/common/angles.h>
#include "blu_rt_obj_seg.h"


#include <pcl/segmentation/extract_polygonal_prism_data.h>
#include <pcl/surface/convex_hull.h>


//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
OrganizedSegmentationDemo::OrganizedSegmentationDemo (CloudPtr cloud){

  // Set up Normal Estimation
  //ne.setNormalEstimationMethod (ne.SIMPLE_3D_GRADIENT);
  ne.setNormalEstimationMethod (ne.COVARIANCE_MATRIX);
  ne.setMaxDepthChangeFactor (0.02f);
  ne.setNormalSmoothingSize (20.0f);
  

  euclidean_cluster_comparator_ = pcl::EuclideanClusterComparator<PointT, pcl::Normal, pcl::Label>::Ptr (new pcl::EuclideanClusterComparator<PointT, pcl::Normal, pcl::Label> ());

  // Set up Organized Multi Plane Segmentation
  mps.setMinInliers (10000);
  mps.setAngularThreshold (pcl::deg2rad (3.0)); //3 degrees
  mps.setDistanceThreshold (0.02); //2cm
  

}

void
OrganizedSegmentationDemo::func (const CloudConstPtr& cloud)
{  
  //prev_cloud_=*cloud;
    
  // Estimate Normals
  pcl::PointCloud<pcl::Normal>::Ptr normal_cloud (new pcl::PointCloud<pcl::Normal>);
  ne.setInputCloud (cloud);
  ne.compute (*normal_cloud);
  
  // Segment Planes
  double mps_start = pcl::getTime ();
  std::vector<pcl::PlanarRegion<PointT>, Eigen::aligned_allocator<pcl::PlanarRegion<PointT> > > regions;
  std::vector<pcl::ModelCoefficients> model_coefficients;
  std::vector<pcl::PointIndices> inlier_indices;  
  pcl::PointCloud<pcl::Label>::Ptr labels (new pcl::PointCloud<pcl::Label>);
  std::vector<pcl::PointIndices> label_indices;
  std::vector<pcl::PointIndices> boundary_indices;
  mps.setInputNormals (normal_cloud);
  mps.setInputCloud (cloud);
  mps.segmentAndRefine (regions, model_coefficients, inlier_indices, labels, label_indices, boundary_indices);
  
  double mps_end = pcl::getTime ();
  std::cout << "MPS+Refine took: " << double(mps_end - mps_start) << std::endl;
    //Segment Objects
  pcl::PointCloud<PointT>::CloudVectorType clusters;

  if (regions.size () > 0)
  {
    std::vector<bool> plane_labels;
    plane_labels.resize (label_indices.size (), false);
    for (size_t i = 0; i < label_indices.size (); i++)
    {
      if (label_indices[i].indices.size () > 10000)
      {
        plane_labels[i] = true;
      }
    }  
    
    euclidean_cluster_comparator_->setInputCloud (cloud);
    euclidean_cluster_comparator_->setLabels (labels);
    euclidean_cluster_comparator_->setExcludeLabels (plane_labels);
    euclidean_cluster_comparator_->setDistanceThreshold (0.01f, false);
    
    pcl::PointCloud<pcl::Label> euclidean_labels;
    std::vector<pcl::PointIndices> euclidean_label_indices;
    pcl::OrganizedConnectedComponentSegmentation<PointT,pcl::Label> euclidean_segmentation (euclidean_cluster_comparator_);
    euclidean_segmentation.setInputCloud (cloud);
    euclidean_segmentation.segment (euclidean_labels, euclidean_label_indices);
    
    for (size_t i = 0; i < euclidean_label_indices.size (); i++)
    {
      if (euclidean_label_indices[i].indices.size () > 1000)
      {
        pcl::PointCloud<PointT> cluster;
        pcl::copyPointCloud (*cloud,euclidean_label_indices[i].indices,cluster);
        clusters.push_back (cluster);
      }    
    }
    std::cout<< "Got "<<clusters.size ()<<" euclidean clusters!"<<std::endl;
    if(clusters.size()>0)
    prev_clusters_ = clusters;
    
  }          
  
}

pcl::PointCloud<PointT>::CloudVectorType 
     OrganizedSegmentationDemo::getprevclusters(){
	return prev_clusters_;
  }



