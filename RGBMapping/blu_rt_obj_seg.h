#ifndef BLU_RT_OBJ_SEG_H_
#define BLU_RT_OBJ_SEG_H_
// PCL
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/common/time.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/integral_image_normal.h>
#include <pcl/segmentation/organized_multi_plane_segmentation.h>
#include <pcl/segmentation/planar_polygon_fusion.h>
#include <pcl/common/transforms.h>
//#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/segmentation/plane_coefficient_comparator.h>
#include <pcl/segmentation/euclidean_plane_coefficient_comparator.h>
#include <pcl/segmentation/rgb_plane_coefficient_comparator.h>
#include <pcl/segmentation/edge_aware_plane_comparator.h>
#include <pcl/segmentation/euclidean_cluster_comparator.h>
#include <pcl/segmentation/organized_connected_component_segmentation.h>
#include <iostream>
typedef pcl::PointXYZRGB PointT;


class OrganizedSegmentationDemo 
{
  
  public:
    typedef pcl::PointCloud<PointT> Cloud;
    typedef Cloud::Ptr CloudPtr;
    typedef Cloud::ConstPtr CloudConstPtr;
    typedef pcl::PointNormal PointNT;
    typedef pcl::PointCloud<PointNT> PointNCloudT;

    OrganizedSegmentationDemo(CloudPtr cloud);
    
    ~OrganizedSegmentationDemo ()
    {
	std::cout<<"Segment Thread is Over!"<<std::endl;
    }
  
    void func (const CloudConstPtr& cloud);//返回一个分割聚类的点云集合
    pcl::PointCloud<PointT>::CloudVectorType getprevclusters();
    //boost::shared_ptr<pcl::visualization::PCLVisualizer> vis_; 
  protected:
    
       
    //pcl::PointCloud<PointT> prev_cloud_;
    //pcl::PointCloud<pcl::Normal> prev_normals_;
    //std::vector<pcl::PlanarRegion<PointT>, Eigen::aligned_allocator<pcl::PlanarRegion<PointT> > > prev_regions_;


    pcl::PointCloud<PointT>::CloudVectorType prev_clusters_;
    pcl::IntegralImageNormalEstimation<PointT, pcl::Normal> ne;
    pcl::OrganizedMultiPlaneSegmentation<PointT, pcl::Normal, pcl::Label> mps;

    //bool capture_;
    //bool data_modified_;
    //size_t previous_data_size_;
    //size_t previous_clusters_size_;

    //bool display_normals_;
    //bool display_curvature_;
    //bool first_loop;

    pcl::EuclideanClusterComparator<PointT, pcl::Normal, pcl::Label>::Ptr euclidean_cluster_comparator_;



};

#endif /* BLU_RT_OBJ_SEG_H_ */
