#ifndef SEGMENTBUILDER_H_
#define SEGMENTBUILDER_H_
#ifndef Q_MOC_RUN
#include <QtGui/QVBoxLayout>

#include "rtabmap/gui/CloudViewer.h"
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/common/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

#include "blu_rt_obj_seg.h"
#include <QtCore/QMetaType>
#include "rtabmap/utilite/UStl.h"
#include "rtabmap/utilite/UConversion.h"
#include "rtabmap/utilite/UEventsHandler.h"
#include "rtabmap/utilite/ULogger.h"
#include "rtabmap/core/util3d.h"
#include "rtabmap/core/RtabmapEvent.h"
#include "rtabmap/core/OdometryEvent.h"
#include <pcl/common/angles.h>
#include <pcl/segmentation/extract_polygonal_prism_data.h>
#include <pcl/surface/convex_hull.h>

#include <pcl/segmentation/supervoxel_clustering.h>
#endif




//#include <vtkRenderWindow.h>
using namespace rtabmap;

// This class receives RtabmapEvent and construct/update a 3D Map
class SegmentBuilder : public QWidget, public UEventsHandler
{
    Q_OBJECT
public:
    SegmentBuilder() :
        _processingStatistics(false),
        _lastOdometryProcessed(true),
	_segCount(0),
	voxel_resolution(0.002f),
	seed_resolution(0.1f),
	color_importance(0.2f),
	spatial_importance(0.4f),
	normal_importance(1.0f),
	first_loop(true)
    {
        this->setWindowFlags(Qt::Dialog);
        this->setWindowTitle(tr("3D Segmentation Viewer"));
        this->setMinimumWidth(1600);
        this->setMinimumHeight(600);
	
	//bluking code***********
	//vis_.reset (new pcl::visualization::PCLVisualizer ("PointCloud Viewer"));
	//vis_->setBackgroundColor(0,0,0);
	//***********************
        cloudViewer_ = new CloudViewer(this);
	cloudViewer_2_=new CloudViewer(this);
	cloudViewer_3_=new CloudViewer(this);
// 	viewer.reset(new pcl::visualization::PCLVisualizer ("3D Viewer"));
// 	viewer->setBackgroundColor (0, 0, 0);
        QVBoxLayout *layout = new QVBoxLayout();
	layout->setDirection(QBoxLayout::LeftToRight);
        layout->addWidget(cloudViewer_);
	layout->addWidget(cloudViewer_2_);
	layout->addWidget(cloudViewer_3_);
        this->setLayout(layout);

        qRegisterMetaType<rtabmap::Statistics>("rtabmap::Statistics");
        qRegisterMetaType<rtabmap::SensorData>("rtabmap::SensorData");
	 
    }

    virtual ~SegmentBuilder()
    {
        this->unregisterFromEventsManager();
    }

private slots:
    void processOdometry(const rtabmap::SensorData & data)
    {
        if(!this->isVisible())
        {
            return;
        }

        Transform pose = data.pose();
        if(pose.isNull())
        {
            //Odometry lost
            //cloudViewer_->setBackgroundColor(Qt::darkRed);

            pose = lastOdomPose_;
        }
        else
        {
            cloudViewer_->setBackgroundColor(Qt::black);
	    cloudViewer_2_->setBackgroundColor(Qt::black);
	    cloudViewer_3_->setBackgroundColor(Qt::black);
        }
        if(!pose.isNull())
        {
            lastOdomPose_ = pose;

            // 3d cloud
            if(data.depth().cols == data.image().cols &&
               data.depth().rows == data.image().rows &&
               !data.depth().empty() &&
               data.fx() > 0.0f &&
               data.fy() > 0.0f)
            {
                pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud = util3d::cloudFromDepthRGB(
                    data.image(),
                    data.depth(),
                    data.cx(),
                    data.cy(),
                    data.fx(),
                    data.fy(),
                    2); // decimation // high definition
		//Segment code by bluking********************
		//UERROR("size= %d",cloud->size());
// 		if(first_loop)
// 		pcl::io::savePCDFile<pcl::PointXYZRGB>("1.pcd",*cloud);
// 		first_loop=false;
		if(cloud->size()){
                OrganizedSegmentationDemo osd(cloud);
		osd.func(cloud);
		pcl::PointCloud<PointT>::CloudVectorType clusters=osd.getprevclusters();
		pcl::PointCloud<PointT>::Ptr cluster_cloud_ptr;
		pcl::PointCloud<PointT>::Ptr cluster_sv_ptr(new pcl::PointCloud<PointT>);
// 		if(first_loop){
// 		vis_->addPointCloud (cloud,"original_cloud");
// 		first_loop=false;
// 		}
// 		vis_->updatePointCloud (cloud,"original_cloud");
// 		displayEuclideanClusters(clusters,vis_);
// 		vis_->spinOnce(10);
		//std::cout<<rand()<<std::endl;
		//std::cout<<clusters.size()<<std::endl;
		pcl::PCLPointCloud2Ptr binaryCloud_blu(new pcl::PCLPointCloud2);
		pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cluster_cloud_rgba(new pcl::PointCloud<pcl::PointXYZRGBA>);
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr cluster_sv_result_ptr;
		if(clusters.size()){
		  for(size_t i = 0; i < clusters.size (); i++){
		    if(clusters[i].size()){
		      //cluster_cloud_ptr = util3d::passThrough<pcl::PointXYZRGB>(boost::make_shared<pcl::PointCloud<pcl::PointXYZRGB> >(clusters[i]), "z", 0, 4.0f);//实用pcl中的直通滤波器进行滤波（过滤掉“z”字段下小于0大于4的所有点）
		      cluster_cloud_ptr=boost::make_shared<pcl::PointCloud<pcl::PointXYZRGB> >(clusters[i]);
		      if(cluster_cloud_ptr->size()){
                      cluster_cloud_ptr = util3d::transformPointCloud<pcl::PointXYZRGB>(cluster_cloud_ptr, data.localTransform());
// 		      pcl::copyPointCloud(*cluster_cloud_ptr,*cluster_sv_ptr); 
		      *cluster_sv_ptr+=*cluster_cloud_ptr;
		      }
		    }
		    
// 		    cluster_cloud_ptr=boost::make_shared<pcl::PointCloud<pcl::PointXYZRGB> >(clusters[i]);
//		    _segCount=cluster_cloud_ptr->size();
		
		   QColor qc=QColor::fromRgb(rand()%255,rand()%255,rand()%255);		   
		   if(!cloudViewer_2_->updateCloud("segcloud_"+i, cluster_cloud_ptr, pose))
		    {		 
			   pcl::toPCLPointCloud2(*cluster_cloud_ptr, *binaryCloud_blu);
			   if(!cloudViewer_2_->addCloud("segcloud_"+i, binaryCloud_blu, pose,false, qc))
			   {
			      UERROR("Adding segcloud to viewer failed!");
			   }
		    }
		  }
		    if(first_loop)
		pcl::io::savePCDFile<pcl::PointXYZRGB>("1.pcd",*cluster_sv_ptr);
		first_loop=false;
		  pcl::copyPointCloud(*cluster_sv_ptr,*cluster_cloud_rgba);
		  cluster_sv_result_ptr=getSupervoxelClusters(cluster_cloud_rgba);
		  if(cluster_sv_result_ptr->size())
                    {
                        cluster_sv_result_ptr = util3d::transformPointCloud<pcl::PointXYZRGB>(cluster_sv_result_ptr, data.localTransform());
                    }
		  if(!cloudViewer_3_->addOrUpdateCloud("spuervoxel_cloud", cluster_sv_result_ptr, pose))
                {
                    UERROR("Adding spuervoxel_cloud to viewer failed!");
                }
		  
		}
		
		}
		
		//*********************************************
                if(cloud->size())
                {
                    cloud = util3d::passThrough<pcl::PointXYZRGB>(cloud, "z", 0, 4.0f);//实用pcl中的直通滤波器进行滤波（过滤掉“z”字段下小于0大于4的所有点）
                    if(cloud->size())
                    {
                        cloud = util3d::transformPointCloud<pcl::PointXYZRGB>(cloud, data.localTransform());
                    }
                }
                
                if(!cloudViewer_->addOrUpdateCloud("cloudOdom", cloud, pose))
                {
                    UERROR("Adding cloudOdom to viewer failed!");
                }
//                  if(!cloudViewer_2_->addOrUpdateCloud("seg_ori_cloud", cloud, pose))
//                 {
//                     UERROR("Adding seg_ori_cloud to viewer failed!");
//                 } 
                
            }

            if(!data.pose().isNull())
            {
                // update camera position
                cloudViewer_->updateCameraTargetPosition(data.pose());
		cloudViewer_2_->updateCameraTargetPosition(data.pose());
            }
        }
        cloudViewer_->update();
	cloudViewer_2_->update();
	cloudViewer_3_->update();
        _lastOdometryProcessed = true;
	

    }


    void processStatistics(const rtabmap::Statistics & stats)
    {
        _processingStatistics = true;
	
        const std::map<int, Transform> & poses = stats.poses();
        QMap<std::string, Transform> clouds = cloudViewer_->getAddedClouds();
	//UERROR("clusters is here: %d",clouds.contains("segcloud_0"));

        for(std::map<int, Transform>::const_iterator iter = poses.begin(); iter!=poses.end(); ++iter)
        {
            if(!iter->second.isNull())
            {
	      //bluking code ********************************************
	      //*********************************************************
// 		for(size_t i=0;i<15;i++){ 
// 		std::string segcloudName = "segcloud_"+i;
// 		//UERROR("clusters contains: %d", clouds.contains(segcloudName));
// 		if(clouds.contains(segcloudName)){
// 		  // Update only if the pose has changed
//                     Transform tSegCloud;
//                     cloudViewer_->getPose(segcloudName, tSegCloud);
//                     if(tSegCloud.isNull() || iter->second != tSegCloud)
//                     {
//                         if(!cloudViewer_->updateCloudPose(segcloudName, iter->second))
//                         {
//                             UERROR("Updating pose segcloud_%d failed!", iter->first);
//                         }
//                     }
//                     cloudViewer_->setCloudVisibility(segcloudName, true);
// 			}
// 		}
	      //***************************************************************
	      //bluking code **************************************************
                // 3d point cloud
		//UERROR("cloud.contains(""segcloud_0"")= %d",clouds.contains("segcloud_0"));
		std::string cloudName = uFormat("cloud%d", iter->first);
                if(clouds.contains(cloudName))
                {
                    // Update only if the pose has changed
		
                    Transform tCloud;
                    cloudViewer_->getPose(cloudName, tCloud);
                    if(tCloud.isNull() || iter->second != tCloud)
                    {
                        if(!cloudViewer_->updateCloudPose(cloudName, iter->second))
                        {
                            UERROR("Updating pose cloud %d failed!", iter->first);
                        }
                    }
                    cloudViewer_->setCloudVisibility(cloudName, true);
                }
                else if(iter->first == stats.refImageId() &&
                        stats.getSignature().id() == iter->first)
                {
		   
                    Signature s = stats.getSignature();
                    s.uncompressData(); // make sure data is uncompressed
                    // Add the new cloud
                    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud = util3d::cloudFromDepthRGB(
                            s.getImageRaw(),
                            s.getDepthRaw(),
                            s.getDepthCx(),
                            s.getDepthCy(),
                            s.getDepthFx(),
                            s.getDepthFy(),
                           8); // decimation

                    if(cloud->size())
                    {
                        cloud = util3d::passThrough<pcl::PointXYZRGB>(cloud, "z", 0, 4.0f);
                        if(cloud->size())
                        {
                            cloud = util3d::transformPointCloud<pcl::PointXYZRGB>(cloud, stats.getSignature().getLocalTransform());
			    
                        }
                    }
                    if(!cloudViewer_->addOrUpdateCloud(cloudName, cloud, iter->second))
                    {
                        UERROR("Adding cloud %d to viewer failed!", iter->first);
                    }
                   
                }
                
            }
        }
       // UERROR("clusters contains: %d", clouds.contains("segcloud_0"));
       

        cloudViewer_->update();

        _processingStatistics = false;
    }
    
    
// void
// displayEuclideanClusters (const pcl::PointCloud<PointT>::CloudVectorType &clusters, 
//                           boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer)
// {
//   char name[1024];
//   unsigned char red [6] = {255,   0,   0, 255, 255,   0};
//   unsigned char grn [6] = {  0, 255,   0, 255,   0, 255};
//   unsigned char blu [6] = {  0,   0, 255,   0, 255, 255};
// 
//   for (size_t i = 0; i < clusters.size (); i++)
//   {
//     sprintf (name, "cluster_%d" , int (i));
//     pcl::visualization::PointCloudColorHandlerCustom<PointT> color0(boost::make_shared<pcl::PointCloud<PointT> >(clusters[i]),red[i%6],grn[i%6],blu[i%6]);
//     if (!viewer->updatePointCloud (boost::make_shared<pcl::PointCloud<PointT> >(clusters[i]),color0,name))
//       viewer->addPointCloud (boost::make_shared<pcl::PointCloud<PointT> >(clusters[i]),color0,name);
//     viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, name);
//     viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_OPACITY, 0.3, name);
//   }
// }

pcl::PointCloud<pcl::PointXYZRGB>::Ptr getSupervoxelClusters(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud){
    //////////////////////////////  //////////////////////////////
  ////// This is how to use supervoxels
  //////////////////////////////  //////////////////////////////

  pcl::SupervoxelClustering<pcl::PointXYZRGBA> super (voxel_resolution, seed_resolution);
  //if (disable_transform)
  //super.setUseSingleCameraTransform (false);
  super.setInputCloud (cloud);
  super.setColorImportance (color_importance);
  super.setSpatialImportance (spatial_importance);
  super.setNormalImportance (normal_importance);

  std::map <uint32_t, pcl::Supervoxel<pcl::PointXYZRGBA>::Ptr > supervoxel_clusters;

  UERROR ("Extracting supervoxels!\n");
  super.extract (supervoxel_clusters);
  UERROR("Found %d supervoxels\n", supervoxel_clusters.size ());

  

//   pcl::PointCloud<pcl::PointXYZRGBA>::Ptr voxel_centroid_cloud = super.getVoxelCentroidCloud ();
//   pcl::PointCloud<pcl::PointXYZRGB>::Ptr voxel_centroid_cloud_rgb(new pcl::PointCloud<pcl::PointXYZRGB>);
//   pcl::copyPointCloud(*voxel_centroid_cloud,*voxel_centroid_cloud_rgb);
     pcl::PointCloud<pcl::PointXYZRGBA>::Ptr colored_voxrl_cloud=super.getColoredVoxelCloud();
     pcl::PointCloud<pcl::PointXYZRGB>::Ptr colored_voxrl_cloud_rgb(new pcl::PointCloud<pcl::PointXYZRGB>);
     pcl::copyPointCloud(*colored_voxrl_cloud,*colored_voxrl_cloud_rgb);

  //pcl::PointCloud<pcl::PointXYZRGB>::Ptr labeled_voxel_cloud = super.getLabeledVoxelCloud ();
  //viewer->addPointCloud (labeled_voxel_cloud, "labeled voxels");
  //viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_OPACITY,0.8, "labeled voxels");

  //pcl::PointCloud<pcl::PointXYZRGB>::Ptr sv_normal_cloud = super.makeSupervoxelNormalCloud (supervoxel_clusters);
  //We have this disabled so graph is easy to see, uncomment to see supervoxel normals
  //viewer->addPointCloudNormals<PointNormal> (sv_normal_cloud,1,0.05f, "supervoxel_normals");

//   UERROR("Getting supervoxel adjacency\n");
//   std::multimap<uint32_t, uint32_t> supervoxel_adjacency;
//   
//   super.getSupervoxelAdjacency (supervoxel_adjacency);//邻接的区块的两两映射
//   //To make a graph of the supervoxel adjacency, we need to iterate through the supervoxel adjacency multimap
//   std::multimap<uint32_t,uint32_t>::iterator label_itr = supervoxel_adjacency.begin ();
//   for ( ; label_itr != supervoxel_adjacency.end (); )
//   {
//     //First get the label
//     uint32_t supervoxel_label = label_itr->first;
//     //Now get the supervoxel corresponding to the label
//     pcl::Supervoxel<pcl::PointXYZRGBA>::Ptr supervoxel = supervoxel_clusters.at (supervoxel_label);
//     //supervoxel->centroid_;
//     //pcl::copyPointCloud(*supervoxel,*voxel_centroid_cloud_rgb);
//     //Now we need to iterate through the adjacent supervoxels and make a point cloud of them
// //     pcl::PointCloud<pcl::PointXYZRGBA> adjacent_supervoxel_centers;
// //     std::multimap<uint32_t,uint32_t>::iterator adjacent_itr = supervoxel_adjacency.equal_range (supervoxel_label).first;
// //     for ( ; adjacent_itr!=supervoxel_adjacency.equal_range (supervoxel_label).second; ++adjacent_itr)
// //     {
// //       pcl::Supervoxel<pcl::PointXYZRGBA>::Ptr neighbor_supervoxel = supervoxel_clusters.at (adjacent_itr->second);
// //       adjacent_supervoxel_centers.push_back (neighbor_supervoxel->centroid_);
// //     }
//     //Now we make a name for this polygon
// //     std::stringstream ss;
// //     ss << "supervoxel_" << supervoxel_label;
//     //This function is shown below, but is beyond the scope of this tutorial - basically it just generates a "star" polygon mesh from the points given
//     //addSupervoxelConnectionsToViewer (supervoxel->centroid_, adjacent_supervoxel_centers, ss.str (), cloudViewer_3_);
//     //Move iterator forward to next label
// 
//     label_itr = supervoxel_adjacency.upper_bound (supervoxel_label);
//   }
 //return voxel_centroid_cloud_rgb;
     return colored_voxrl_cloud_rgb;
}

// void
// addSupervoxelConnectionsToViewer (pcl::PointXYZRGBA &supervoxel_center,
//                                   pcl::PointCloud<pcl::PointXYZRGBA> &adjacent_supervoxel_centers,
//                                   std::string supervoxel_name,
//                                   CloudViewer & viewer)
// {
//   vtkSmartPointer<vtkPoints> points = vtkSmartPointer<vtkPoints>::New ();
//   vtkSmartPointer<vtkCellArray> cells = vtkSmartPointer<vtkCellArray>::New ();
//   vtkSmartPointer<vtkPolyLine> polyLine = vtkSmartPointer<vtkPolyLine>::New ();
// 
//   //Iterate through all adjacent points, and add a center point to adjacent point pair
//   pcl::PointCloud<pcl::PointXYZRGBA>::iterator adjacent_itr = adjacent_supervoxel_centers.begin ();
//   for ( ; adjacent_itr != adjacent_supervoxel_centers.end (); ++adjacent_itr)
//   {
//     points->InsertNextPoint (supervoxel_center.data);
//     points->InsertNextPoint (adjacent_itr->data);
//   }
//   // Create a polydata to store everything in
//   vtkSmartPointer<vtkPolyData> polyData = vtkSmartPointer<vtkPolyData>::New ();
//   // Add the points to the dataset
//   polyData->SetPoints (points);
//   polyLine->GetPointIds  ()->SetNumberOfIds(points->GetNumberOfPoints ());
//   for(unsigned int i = 0; i < points->GetNumberOfPoints (); i++)
//     polyLine->GetPointIds ()->SetId (i,i);
//   cells->InsertNextCell (polyLine);
//   // Add the lines to the dataset
//   polyData->SetLines (cells);
//   viewer->addModelFromPolyData (polyData,supervoxel_name);
// }

protected:
    virtual void handleEvent(UEvent * event)
    {
        if(event->getClassName().compare("RtabmapEvent") == 0)
        {
            RtabmapEvent * rtabmapEvent = (RtabmapEvent *)event;
            const Statistics & stats = rtabmapEvent->getStats();
            // Statistics must be processed in the Qt thread
            if(this->isVisible())
            {
                QMetaObject::invokeMethod(this, "processStatistics", Q_ARG(rtabmap::Statistics, stats));
            }
        }
        else if(event->getClassName().compare("OdometryEvent") == 0)
        {
            OdometryEvent * odomEvent = (OdometryEvent *)event;
            // Odometry must be processed in the Qt thread
            if(this->isVisible() &&
               _lastOdometryProcessed &&
               !_processingStatistics)
            {
                _lastOdometryProcessed = false; // if we receive too many odometry events!
                QMetaObject::invokeMethod(this, "processOdometry", Q_ARG(rtabmap::SensorData, odomEvent->data()));
            }
        }
    }

private:
    CloudViewer * cloudViewer_;
    CloudViewer * cloudViewer_2_;
    CloudViewer * cloudViewer_3_;
    Transform lastOdomPose_;
    bool _processingStatistics;
    bool _lastOdometryProcessed;
    size_t _segCount;
    float voxel_resolution ;
    float seed_resolution ;
    float color_importance ;
    float spatial_importance;
    float normal_importance;
    //PointT supervoxel_center;
    //pcl::PointCloud<pcl::PointXYZRGB> adjacent_supervoxel_centers;
    //std::string supervoxel_name;
    bool first_loop;
    
};


#endif /* SEGMENTBUILDER_H_ */
