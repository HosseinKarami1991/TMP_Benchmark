#ifndef TAMPSCENE_H
#define TAMPSCENE_H

#include <pcl/filters/crop_box.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/correspondence.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/shot_omp.h>
#include <pcl/features/board.h>
//#include <pcl/filters/uniform_sampling.h>
#include <pcl/keypoints/uniform_sampling.h>
#include <pcl/recognition/cg/hough_3d.h>
#include <pcl/recognition/cg/geometric_consistency.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/kdtree/impl/kdtree_flann.hpp>
#include <pcl/common/transforms.h>
#include <pcl/console/parse.h>
#include <pcl/pcl_base.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/features/moment_of_inertia_estimation.h>

//#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <opencv/cv.hpp>

#include "opencv2/core.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/imgcodecs.hpp"
#include "tf/tf.h"
#include "tf/tfMessage.h"




using namespace std;
using namespace cv;	


#define RST "\x1B[0m"
#define KRED  "\x1B[31m"
#define KGRN  "\x1B[32m"
#define KYLW  "\x1B[33m"
#define KBLU  "\x1B[34m"
#define KCYN  "\x1B[36m"

#define FRED(x)  KRED x RST
#define FGRN(x)  KGRN x RST
#define FBLU(x)  KBLU x RST
#define FYLW(x)  KYLW x RST
#define FCYN(x)  KCYN x RST
#define BOLD(x)  "\x1B[1m" x RST

typedef pcl::PointXYZRGBA PointType;
typedef pcl::Normal NormalType;
typedef pcl::ReferenceFrame RFType;
typedef pcl::SHOT352 DescriptorType;
typedef pcl::PointXYZ PointT;



class tamp_scene_class
{
public:
	 tamp_scene_class(string modesdirectory, int numobj);
	~tamp_scene_class();
     void setInputCloud(pcl::PointCloud<PointT>::Ptr &cloud);
     void clusterCloud();
     PointT & extractCoud(PointT &maincloud,PointT & cloud);
     void readObjectsCloud();
     void removePlane();
     void setToFilter();
     void filterCloud();
     void computeNormals();
     void extractKeyPoints();
     void computeDescriptor();
     void modelSceneCorrespondeces();
     void findCorrespondences();
     void findObjectCentre();
     void printClusterObject();
     void cropCloud();
     void setToCrop();
     void detectCircles();
     double findangle( cv::Point pt1, cv::Point pt2, cv::Point pt0);
     void findSquares();
     void drawSquares();
     void detectSquares();
     bool ifSquareinsideZone(vector<cv::Point> square);
     void boundBoxCube();
     void addViewer(pcl::PointCloud<PointT>::Ptr pc,PointT min,PointT max,Eigen::Vector3f position,Eigen::Quaternionf quat,PointT min2,PointT max2);

     pcl::PointCloud<PointT>::Ptr rawCloud_,rawCloudwithNAN_,croppedCloud_,filteredCloud_,filteredCloudbox_,remainingfilteredCloudbox_,objectCloud,planeCloud_,removedPlaneCloud_
     ,model_keypoints_,scene_keypoints_,rotated_model_;
     std::vector<pcl::PointCloud<PointT>::Ptr> model_keypoints_vector_,scene_keypoints_vector_;
     std::vector<pcl::PointCloud<NormalType>::Ptr> model_normals_vector_,scene_normals_vector_;
     std::vector<pcl::PointCloud<PointT>::Ptr> modelObjects;
     string modeladdress;
     pcl::PCDReader reader;
     int nuofobj;
     pcl::VoxelGrid<PointT> vg;
     pcl::SACSegmentation<PointT> seg;
     pcl::PointIndices::Ptr inliers_;
     pcl::ModelCoefficients::Ptr coefficients_;
     pcl::ExtractIndices<PointT> extract;
     bool tofilter_,tocrop_;
     pcl::search::KdTree<PointT>::Ptr tree_;
     std::vector<pcl::PointIndices> cluster_indices_;
     pcl::EuclideanClusterExtraction<PointT> ec;
     std::vector<pcl::PointCloud<PointT>::Ptr> clusters_vector_;
     bool clusteringFinshed_;
     pcl::NormalEstimationOMP<PointT, NormalType> norm_est;
     pcl::UniformSampling<PointT> uniform_sampling;
     float model_ss_,descr_rad_,rf_rad_,cg_size_,cg_thresh_;
     std::vector<int> cluster_object_vector;
 
     pcl::SHOTEstimationOMP<PointT, NormalType, DescriptorType> descr_est;
     std::vector<pcl::PointCloud<DescriptorType>::Ptr> model_descriptors_vector_,scene_descriptors_vector_;
     pcl::CorrespondencesPtr model_scene_corrs_;
     pcl::KdTreeFLANN<DescriptorType> match_search;
     std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> > rototranslations;
     std::vector<pcl::Correspondences> clustered_corrs;
     pcl::BOARDLocalReferenceFrameEstimation<PointT, NormalType, RFType> rf_est;
     pcl::Hough3DGrouping<PointT, PointT, RFType, RFType> clusterer;
     std::vector<Eigen::Vector4f> centerofObjects_;
     std::vector<string> objectsNames_;
     pcl::CropBox<PointT> boxFilter;
     cv::Mat img_,gray_;
     vector<cv::Vec3f> circles_;
     vector<vector<cv::Point>> squares;
     std::vector<tf::Transform> centerwrtbase_;
     pcl::MomentOfInertiaEstimation <PointT> feature_extractor;
     std::vector <float> moment_of_inertia_;
     std::vector <float> eccentricity_;
     pcl::PointXYZ min_point_AABB_;
     pcl::PointXYZ max_point_AABB_;
     pcl::PointXYZ min_point_OBB_;
     pcl::PointXYZ max_point_OBB_;
     pcl::PointXYZ position_OBB_;
     Eigen::Matrix3f rotational_matrix_OBB_;
     float major_value_, middle_value_, minor_value_;
     Eigen::Vector3f major_vector_, middle_vector_, minor_vector_;
     Eigen::Vector3f mass_center_;
     tf::Vector3 positionbox_;
     tf::Quaternion quatbox_;
     std::vector<double> boxsize_,boxsizebase_;
     pcl::visualization::PCLVisualizer::Ptr viewer;

};































#endif