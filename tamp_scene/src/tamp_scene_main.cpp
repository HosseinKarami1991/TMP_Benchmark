#include "tamp_scene.h"
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <tamp_scene/cluster_point_cloud.h>
#include <tf/transform_broadcaster.h>
#include "tamp_msgs/sceneobjects.h"
#include "geometry_msgs/Point.h"
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include"geometry_msgs/Transform.h"






string frame;
pcl::PointCloud<PointT>::Ptr temp_cloud(new pcl::PointCloud<PointT>);
cv_bridge::CvImagePtr cv_ptr;
cv_bridge::CvImage out_msg;

string address ="/home/hossein/catkin_ws/src/TAMP/tamp_scene/models";
tamp_scene_class tampScene(address,2);

void imageCb(const sensor_msgs::ImageConstPtr& msg){
     cout<<"***image_encodings is "<<msg->encoding<<endl;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
      tampScene.img_ = cv_ptr->image;
      tampScene.detectCircles();
      tampScene.detectSquares();
      //cv::namedWindow( "Display window", cv::WINDOW_AUTOSIZE );// Create a window for display.
      //cv::imshow( "Display window", tampScene.img_ );
      out_msg.header =  msg->header;
      out_msg.encoding = msg->encoding;
      out_msg.image =tampScene.img_;
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

}


void cloud_cb (const sensor_msgs::PointCloud2ConstPtr& input)
{

       std::cerr << "PointCloud size is : " << input->data.size () << " data points." << std::endl;
       frame = input->header.frame_id;
       pcl::PCLPointCloud2 pcl_pc2;
   	   pcl_conversions::toPCL(*input,pcl_pc2);
       pcl::fromPCLPointCloud2(pcl_pc2,*temp_cloud);



}


bool sceneQuery(tamp_msgs::sceneobjects::Request& req,tamp_msgs::sceneobjects::Response &res){

      if(req.update){

        
        tampScene.clusterCloud();
        //tampScene.filterCloud();
        tampScene.findCorrespondences();
        tampScene.printClusterObject();
        tampScene.findObjectCentre();
        //tampScene.boundBoxCube();
        std::vector<Eigen::Vector4f> objectposes = tampScene.centerofObjects_;
        std::vector<tf::Transform> objectposeswrtbase = tampScene.centerwrtbase_;
        std::vector<string> typeofobjects = tampScene.objectsNames_;
        std::vector<geometry_msgs::Point> objectposesgeo;
        std::vector<geometry_msgs::Transform> objectposeswrt;
        std::vector<std_msgs::String> typeofobjectsstr;
        for(int i=0;i<objectposes.size();i++){
        	Eigen::Vector4f centroid =objectposes[i] ;
          tf::Transform centerwrtbase = objectposeswrtbase[i];
           geometry_msgs::Transform msg;
         msg.translation.x = centerwrtbase.getOrigin().x();
          msg.translation.y = centerwrtbase.getOrigin().y();
         msg.translation.z = centerwrtbase.getOrigin().z();
        	geometry_msgs::Point objpos;
        	objpos.x = centroid(0);
        	objpos.y = centroid(1);
        	objpos.z = centroid(2);
        	objectposesgeo.push_back(objpos);
          geometry_msgs::Point objposwrtbase;
          //objposwrtbase.x=centerwrtbase.getX();
         // objposwrtbase.y=centerwrtbase.getY();
         // objposwrtbase.z=centerwrtbase.getZ();
          objectposeswrt.push_back(msg);
            std::stringstream ss;
            ss<<typeofobjects[i];
            std_msgs::String name;
            name.data = typeofobjects[i];
            typeofobjectsstr.push_back(name);
            static tf::TransformBroadcaster br;
			tf::Transform transform;
			transform.setOrigin( tf::Vector3(centroid(0), centroid(1), centroid(2)) );
			tf::Quaternion q;
			q.setRPY(0, 0, 0);
			transform.setRotation(q);
			br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), frame, ss.str()));


        }
        /*
        tf::Vector3 boxpos = tampScene.positionbox_;
        tf::Quaternion quatbox = tampScene.quatbox_;
        std::vector<double> boxsize = tampScene.boxsizebase_;
        std::vector<float> buondboxdtail;
        buondboxdtail.push_back(boxpos.getX());
        buondboxdtail.push_back(boxpos.getY());
        buondboxdtail.push_back(boxpos.getZ());
        buondboxdtail.push_back(quatbox.x());
        buondboxdtail.push_back(quatbox.y());
        buondboxdtail.push_back(quatbox.z());
        buondboxdtail.push_back(quatbox.w());
        buondboxdtail.push_back(boxsize[0]);
        buondboxdtail.push_back(boxsize[1]);
        buondboxdtail.push_back(boxsize[2]);
       
        res.boundbox = buondboxdtail;

        */
         res.types = typeofobjectsstr;
        res.poses = objectposesgeo;
        res.baseposes = objectposeswrt;
        






      }
      
      return true;
        






}






int main(int argc, char**argv)
{    


    ros::init (argc, argv, "tamp_scene_node");
    ros::NodeHandle nh;

  // Create a ROS subscriber for the input point cloud
    //string topic = argv[1];
    string topic;
    nh.getParam("input_pointcloud",topic);
    ros::Subscriber sub = nh.subscribe (topic, 1, cloud_cb);
    ros::Subscriber imgsub = nh.subscribe ("/camera/rgb/image_color", 1, imageCb);
    cout<<"Subscribinf to "<<topic<<endl;

    ros::Publisher  pub_removed_plane = nh.advertise<sensor_msgs::PointCloud2> ("plane_removed", 1);
    ros::Publisher  cropped_cloud = nh.advertise<sensor_msgs::PointCloud2> ("cropped_cloud", 1);
    ros::Publisher  out_img = nh.advertise<sensor_msgs::Image> ("edged_image", 1);
	
	
	ros::Rate loop_rate(1);
	 
     sensor_msgs::PointCloud2 pcl_removed_plane,pcl_cropped;
    ros::ServiceServer sceneService = nh.advertiseService("tamp_scene_service", sceneQuery);
    
    //This will get called once per visualization iteration
   //pcl::visualization::PCLVisualizer viewer ("input_cloud");
	while(ros::ok())

	{   
        tampScene.setToCrop();
		tampScene.setToFilter();

        tampScene.setInputCloud(temp_cloud);
        tampScene.removePlane();
        tampScene.cropCloud();
         pcl::toROSMsg(*tampScene.removedPlaneCloud_,pcl_removed_plane);
         pcl::toROSMsg(*tampScene.croppedCloud_,pcl_cropped);
        pub_removed_plane.publish(pcl_removed_plane);
        cropped_cloud.publish(pcl_cropped);
        out_img.publish(out_msg.toImageMsg());
        
   	 	loop_rate.sleep();
		ros::spinOnce ();
	}

	return 0;
}