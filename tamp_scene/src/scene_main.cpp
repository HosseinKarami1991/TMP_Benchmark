#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/features/normal_3d.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <tamp_scene/cluster_point_cloud.h>
#include <tf/transform_broadcaster.h>
#include "tamp_scene.h"



using namespace std;
int nu_clusters=0;
typedef pcl::PointXYZ PointT;
	    pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT> ());

	  pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);
		pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZ>);
		 pcl::ModelCoefficients::Ptr coefficients_plane (new pcl::ModelCoefficients);
	  pcl::PointIndices::Ptr inliers_plane (new pcl::PointIndices);
	  pcl::PointCloud<PointT>::Ptr cloud_plane (new pcl::PointCloud<PointT> ());
	    pcl::PointCloud<PointT>::Ptr cloud_removed_plane (new pcl::PointCloud<PointT>);


std::vector<sensor_msgs::PointCloud2> clusters;



ros::Publisher pub,pub_filtered,pub_plane,pub_clusters;

void cloud_cb (const sensor_msgs::PointCloud2ConstPtr& input)
{
  // Create a container for the data.
  std::cerr << "PointCloud size is : " << input->data.size () << " data points." << std::endl;

  pcl::SACSegmentation<pcl::PointXYZ> seg;
  sensor_msgs::PointCloud2 output;
    pcl::ExtractIndices<PointT> extract;
  //pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>), cloud_f (new pcl::PointCloud<pcl::PointXYZ>);
  //pcl::PCLPointCloud2* cloud = new pcl::PCLPointCloud2; 
  // Do data processing here...
  output = *input;
  //pcl_conversions::toPCL(*input,*cloud);
   string frame = input->header.frame_id;

   pcl::PCLPointCloud2 pcl_pc2;
   pcl_conversions::toPCL(*input,pcl_pc2);
   
   pcl::fromPCLPointCloud2(pcl_pc2,*temp_cloud);
  
  //	viewer.addPointCloud(temp_cloud,"sf");
  // Publish the data.




  // Create the segmentation object for the planar model and set all the parameters
  seg.setOptimizeCoefficients (true);
  seg.setModelType (pcl::SACMODEL_PLANE);
  //seg.setNormalDistanceWeight (0.1);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setMaxIterations (100);
  seg.setDistanceThreshold (0.02);
  seg.setInputCloud (temp_cloud);
  //seg.setInputNormals (cloud_normals);
  // Obtain the plane inliers and coefficients
  seg.segment (*inliers_plane, *coefficients_plane);
  std::cerr << "Plane coefficients: " << *coefficients_plane << std::endl;

  // Extract the planar inliers from the input cloud
  extract.setInputCloud (temp_cloud);
  extract.setIndices (inliers_plane);
  extract.setNegative (false);

  // Write the planar inliers to disk
  
  extract.filter (*cloud_plane);
  std::cerr << "PointCloud representing the planar component: " << cloud_plane->points.size () << " data points." << std::endl;
  extract.setNegative (true);
  extract.filter (*cloud_removed_plane);

    std::cerr << "PointCloud Remaining: " << cloud_removed_plane->points.size () << " data points." << std::endl;


//Clustering


  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
  tree->setInputCloud (cloud_removed_plane);

  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
  ec.setClusterTolerance (0.02); // 2cm
  ec.setMinClusterSize (100);
  ec.setMaxClusterSize (25000);
  ec.setSearchMethod (tree);
  ec.setInputCloud (cloud_removed_plane);
  ec.extract (cluster_indices);
  nu_clusters = cluster_indices.size();
 clusters.resize(nu_clusters);
 int j = 0;
 Eigen::Vector4f centroid;
 for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
  {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);
    for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
        {cloud_cluster->points.push_back (cloud_removed_plane->points[*pit]);} //*
    cloud_cluster->width = cloud_cluster->points.size ();
    cloud_cluster->height = 1;
    cloud_cluster->is_dense = true;

    std::cout << "PointCloud representing the Cluster: " << cloud_cluster->points.size () << " data points." << std::endl;
    std::stringstream ss,sss,shape;
    ss << "cloud_cluster_" << j << ".pcd";
    sss << "cluster_" << j;
    pcl::compute3DCentroid (*cloud_cluster, centroid);
     cout<< BOLD(FBLU("cluster ")) <<j<<BOLD(FBLU(" x= "))<<centroid(0)<<BOLD(FCYN(" y= "))<<centroid(1)<<BOLD(FRED(" z= "))<<centroid(2)<<endl;

   pcl::toROSMsg(*cloud_cluster,clusters[j]);
   


   static tf::TransformBroadcaster br;
  tf::Transform transform;
  transform.setOrigin( tf::Vector3(centroid(0), centroid(1), centroid(2)) );
  tf::Quaternion q;
  q.setRPY(0, 0, 0);
  transform.setRotation(q);
  br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), frame, sss.str()));
   
  
 j++;
}








// pub_clusters.publish(clusters);

//End Clustering
  sensor_msgs::PointCloud2 pcl_removed_plane;
  sensor_msgs::PointCloud2 pcl_filtered;

//pcl::toPCLPointCloud2(*cloud_removed_plane,pcl_removed_plane);

//pcl::toPCLPointCloud2(*cloud_removed_plane,pcl_filtered);



pcl::toROSMsg(*cloud_removed_plane,pcl_removed_plane);

pcl::toROSMsg(*cloud_plane,pcl_filtered);
pcl_removed_plane.header.frame_id = frame;

pub_plane.publish (pcl_filtered);
pub_filtered.publish(pcl_removed_plane);


  pub.publish (output);

 
  
}


int main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "my_pcl_tutorial");
  ros::NodeHandle nh;

  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub = nh.subscribe ("/point_cloud/points", 1, cloud_cb);

  // Create a ROS publisher for the output point cloud
  pub = nh.advertise<sensor_msgs::PointCloud2> ("output", 1);
  pub_filtered = nh.advertise<sensor_msgs::PointCloud2> ("plane_removed", 1);
  pub_plane = nh.advertise<sensor_msgs::PointCloud2> ("plane_cloud", 1);
  pub_clusters = nh.advertise<tamp_scene::cluster_point_cloud>("clusters_all", 1);
 ros::Rate loop_rate(2);
   
    
    //This will get called once per visualization iteration
   //pcl::visualization::PCLVisualizer viewer ("input_cloud");
while(ros::ok())

{  
    
    loop_rate.sleep();
	ros::spinOnce ();
}
    
  // Spin
  

}
