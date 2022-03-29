#include "tamp_scene.h"




tamp_scene_class::tamp_scene_class(string modesdirectory, int numobj):rawCloud_(new pcl::PointCloud<PointT>)
                                             ,rawCloudwithNAN_(new pcl::PointCloud<PointT>)
                                             ,croppedCloud_(new pcl::PointCloud<PointT>)
                                             ,filteredCloud_(new pcl::PointCloud<PointT>)
                                             ,filteredCloudbox_(new pcl::PointCloud<PointT>)
                                             ,remainingfilteredCloudbox_(new pcl::PointCloud<PointT>)
                                             ,inliers_ (new pcl::PointIndices)
                                             ,coefficients_ (new pcl::ModelCoefficients)
                                             ,planeCloud_(new pcl::PointCloud<PointT>)
                                             ,removedPlaneCloud_(new pcl::PointCloud<PointT>)
                                             ,tree_(new pcl::search::KdTree<PointT>)
                                             ,model_keypoints_(new pcl::PointCloud<PointT>)
                                             ,scene_keypoints_(new pcl::PointCloud<PointT>)
                                             ,rotated_model_(new pcl::PointCloud<PointT>)
                                             ,model_scene_corrs_ (new pcl::Correspondences ())
                                              /*,viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"))*/
{ 


  cout<<BOLD(FCYN("tamp_scene_class::tamp_scene_class"))<<endl;
  modeladdress = modesdirectory;
  nuofobj = numobj;
  clusteringFinshed_=true;
  tofilter_ = false;
  tocrop_ = false;
  model_ss_ = 0.01f;
  descr_rad_ = 0.02f;
  rf_rad_=0.015f;
  cg_size_=0.01f;
  cg_thresh_ =5.0f;
  norm_est.setKSearch (10);
  descr_est.setRadiusSearch (descr_rad_);

  readObjectsCloud();
  

}
tamp_scene_class::~tamp_scene_class(){};

void tamp_scene_class::setInputCloud(pcl::PointCloud<PointT>::Ptr &cloud){
  *rawCloudwithNAN_ = *cloud;
  std::cout << "size with NAN: " << rawCloudwithNAN_->points.size () << std::endl;
  std::vector<int> indices;
  pcl::removeNaNFromPointCloud(*cloud, *rawCloud_, indices);
  std::cout << "size withoutNAN: " << rawCloud_->points.size () << std::endl;
   if(tocrop_){
    cout<<BOLD(FGRN("Cropping Cloud"))<<endl;
   cropCloud();

  }
  
  if(tofilter_){
    cout<<BOLD(FGRN("Filtering Cloud"))<<endl;
   filterCloud();

  }
  
// *rawCloudwithNAN_ = *cloud;

}

void tamp_scene_class::cropCloud(){

cout<<BOLD(FGRN("Cropping Cloud"))<<endl;
boxFilter.setMin(Eigen::Vector4f(-0.4, -2.5, 0.02, 1));
boxFilter.setMax(Eigen::Vector4f(0.5, 0.2, 1, 1));
boxFilter.setInputCloud(rawCloud_);
boxFilter.filter(*croppedCloud_);
cout<<BOLD(FRED("cropped filter size is "))<<croppedCloud_->points.size ()<<endl;
  

}


void tamp_scene_class::setToFilter(){
  
  tofilter_ = true;



}

void tamp_scene_class::setToCrop(){

  tocrop_=true;
}

void tamp_scene_class::readObjectsCloud(){

  for(int i=0; i<nuofobj;i++){
      
    string add = modeladdress +"/"+"object_" +to_string(i+1)+".pcd"; 
    pcl::PointCloud<PointT>::Ptr objectCloud(new pcl::PointCloud<PointT>);
  	reader.read(add,*objectCloud);
  	modelObjects.push_back(objectCloud);
  	cout<<BOLD(FGRN("Reading objects PointCloud"))<<" # "<<i+1<<endl;


  }
   std::cout << BOLD(FCYN("Computing Normals of Models ....... "))<<std::endl;
  
  //model_normals_vector_.clear();
  //compute normals of models


	for(int i =0;i<modelObjects.size();i++){
           pcl::PointCloud<NormalType>::Ptr model_normals(new pcl::PointCloud<NormalType> ());
	       norm_est.setInputCloud (modelObjects[i]);
	       norm_est.compute (*model_normals);
	      
	       model_normals_vector_.push_back(model_normals);
	  }



	 std::cout << BOLD(FCYN("Computing Descriptors of Models....... "))<<std::endl;
	  
	 

	  //model_descriptors_vector_.clear();
	for(int i =0;i<modelObjects.size();i++){
             pcl::PointCloud<DescriptorType>::Ptr model_descriptors(new pcl::PointCloud<DescriptorType> ());
	        descr_est.setInputCloud (modelObjects[i]);
			descr_est.setInputNormals (model_normals_vector_[i]);
			descr_est.setSearchSurface (modelObjects[i]);
			descr_est.compute (*model_descriptors);
	        model_descriptors_vector_.push_back(model_descriptors);

	  }
  	for(int i=0;i<model_descriptors_vector_.size();i++){

	    std::cout << BOLD(FRED("Descriptors of the Cluster of Model: ")) <<i+1<<" has size "<< model_descriptors_vector_[i]->size () << std::endl;

	}
  

}

void tamp_scene_class::filterCloud(){
  if(tocrop_){

      vg.setInputCloud (croppedCloud_);
      vg.setLeafSize (0.005f, 0.005f, 0.005f);
      vg.filter (*filteredCloud_);
      std::cout << "PointCloud after filtering has: " << filteredCloud_->points.size ()  << " data points." << std::endl;


  }
  else{

      vg.setInputCloud (rawCloud_);
      vg.setLeafSize (0.005f, 0.005f, 0.005f);
      vg.filter (*filteredCloud_);
      std::cout << "PointCloud after filtering has: " << filteredCloud_->points.size ()  << " data points." << std::endl;

  }
 


}


void tamp_scene_class::removePlane(){

	   std::cout << BOLD(FCYN("Removing Plane from Input Cloud ....... "))<<std::endl;
    
      seg.setOptimizeCoefficients (true);
	  seg.setModelType (pcl::SACMODEL_PLANE);
	  seg.setMethodType (pcl::SAC_RANSAC);
	  seg.setMaxIterations (100);
	  seg.setDistanceThreshold (0.02);
	  if(tofilter_){

	  	seg.setInputCloud (filteredCloud_);
	  }
    else if(!tofilter_ && tocrop_){
       seg.setInputCloud (croppedCloud_);
    }
	  else{

	  	seg.setInputCloud (rawCloud_);
	  }
	  
	  seg.segment (*inliers_, *coefficients_);
	  if (inliers_->indices.size () == 0)
	    {
	      std::cout << "Could not estimate a planar model for the given dataset." << std::endl;
	     
	    }
	    std::cerr << BOLD(FYLW("Plane coefficients_: ")) << *coefficients_ << std::endl;
     if(tofilter_){
     	extract.setInputCloud (filteredCloud_);

     }
     else if(!tofilter_ && tocrop_){

       extract.setInputCloud (croppedCloud_);
     }
     else{
        extract.setInputCloud (rawCloud_);

     }
	 
     extract.setIndices (inliers_);
     extract.setNegative (false);

    // Get the points associated with the planar surface
     extract.filter (*planeCloud_);
     std::cout << "PointCloud representing the planar component: " << planeCloud_->points.size () << " data points." << std::endl;

    // Remove the planar inliers_, extract the rest
     extract.setNegative (true);
     extract.filter (*removedPlaneCloud_);


}

void tamp_scene_class::clusterCloud(){
   

     std::cout << BOLD(FCYN("Clustering Cloud ....... "))<<std::endl;


    clusteringFinshed_=false;
    tree_->setInputCloud (removedPlaneCloud_);
    ec.setClusterTolerance (0.05); // 2cm
	ec.setMinClusterSize (50);
	ec.setMaxClusterSize (10000);
	ec.setSearchMethod (tree_);
	ec.setInputCloud (removedPlaneCloud_);
	ec.extract (cluster_indices_);
            centerofObjects_.clear();

    clusters_vector_.clear();
	int j = 1;
	for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices_.begin (); it != cluster_indices_.end (); ++it)
	{   
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);
		for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
		    {cloud_cluster->points.push_back (removedPlaneCloud_->points[*pit]);} //*
		cloud_cluster->width = cloud_cluster->points.size ();
		cloud_cluster->height = 1;
		cloud_cluster->is_dense = true;


        std::cout << BOLD(FYLW("PointCloud representing the Cluster: ")) <<j<<" has "<< cloud_cluster->points.size () << " data points." << std::endl;
        Eigen::Vector4f centroid;
      pcl::compute3DCentroid (*cloud_cluster, centroid);
      centerofObjects_.push_back(centroid);

        clusters_vector_.push_back(cloud_cluster);

        j++;
        
       // cloud_cluster_->points.clear();

}
            cluster_indices_.clear();
            
          // std::cout << BOLD(FRED("size of vector of clusters is "))<<clusters_vector_.size() <<std::endl;

   

}

void tamp_scene_class::computeNormals(){

    
   std::cout << BOLD(FCYN("Computing Normals of Clusters ....... "))<<std::endl;


  scene_normals_vector_.clear();
  for(int i =0;i<clusters_vector_.size();i++){
        pcl::PointCloud<NormalType>::Ptr scene_normals (new pcl::PointCloud<NormalType> ());

       norm_est.setInputCloud (clusters_vector_[i]);
       norm_est.compute (*scene_normals);

       scene_normals_vector_.push_back(scene_normals);
      // std::cout << BOLD(FGRN("Normals of the Cluster: ")) <<i+1<<" has "<< scene_normals_vector_[i]->points.size () << " data points." << std::endl;

  }

}

/*
void tamp_scene_class::extractKeyPoints(){
     


     model_keypoints_vector_.clear();
    for(int i =0;i<modelObjects.size();i++){

       uniform_sampling.setInputCloud (modelObjects[i]);
       uniform_sampling.setRadiusSearch (model_ss_);
       uniform_sampling.filter (*model_keypoints_);
       model_keypoints_vector_.push_back(*model_keypoints_);
  }


  

     scene_keypoints_vector_.clear();
    for(int i =0;i<modelObjects.size();i++){

       uniform_sampling.setInputCloud (clusters_vector_[i]);
       uniform_sampling.setRadiusSearch (model_ss_);
       uniform_sampling.filter (*scene_keypoints_);
       scene_keypoints_vector_.push_back(*scene_keypoints_);
  }








}
*/

void tamp_scene_class::computeDescriptor(){
   
	std::cout << BOLD(FCYN("Computing Descriptors of Scene ....... "))<<std::endl;

	scene_descriptors_vector_.clear();
	for(int i =0;i<clusters_vector_.size();i++){
	        //std::cout << BOLD(FYLW("Normals of the Cluster: ")) <<i+1<<" has "<< scene_normals_vector_[i]->points.size () << " data points." << std::endl;
	        //std::cout << BOLD(FRED("scene_normals_vector_.size () ")) <<" has "<< scene_normals_vector_.size () << std::endl;
	        pcl::PointCloud<DescriptorType>::Ptr scene_descriptors (new pcl::PointCloud<DescriptorType> ());
	        descr_est.setInputCloud (clusters_vector_[i]);
			descr_est.setInputNormals (scene_normals_vector_[i]);
			descr_est.setSearchSurface (clusters_vector_[i]);
			descr_est.compute (*scene_descriptors);
	        scene_descriptors_vector_.push_back(scene_descriptors);

	}
//	for(int i=0;i<scene_descriptors_vector_.size();i++){

//	    std::cout << BOLD(FYLW("Descriptors of the Cluster: ")) <<i+1<<" has size "<< scene_descriptors_vector_[i]->size () << std::endl;

//	}


}

void tamp_scene_class::modelSceneCorrespondeces(){
   cluster_object_vector.clear();
   cluster_object_vector.resize(clusters_vector_.size());
   for(int k=0;k<clusters_vector_.size();k++)
      {

      		for(int j=0;j<nuofobj;j++){
              
            	pcl::CorrespondencesPtr model_scene_corrs (new pcl::Correspondences ());
            	match_search.setInputCloud (model_descriptors_vector_[j]);

		  //  For each scene keypoint descriptor, find nearest neighbor into the model keypoints descriptor cloud and add it to the correspondences vector.
				  for (std::size_t i = 0; i < scene_descriptors_vector_[k]->size (); ++i)
				  {
				    std::vector<int> neigh_indices (1);
				    std::vector<float> neigh_sqr_dists (1);
				    if (!std::isfinite (scene_descriptors_vector_[k]->at (i).descriptor[0])) //skipping NaNs
				    {
				      continue;
				    }
				    int found_neighs = match_search.nearestKSearch (scene_descriptors_vector_[k]->at (i), 1, neigh_indices, neigh_sqr_dists);
				    if(found_neighs == 1 && neigh_sqr_dists[0] < 0.25f) //  add match only if the squared descriptor distance is less than 0.25 (SHOT descriptor distances are between 0 and 1 by design)
				    {
				      pcl::Correspondence corr (neigh_indices[0], static_cast<int> (i), neigh_sqr_dists[0]);
				      model_scene_corrs->push_back (corr);
				    }
				  }
				  std::cout << "Correspondences found: " << model_scene_corrs->size () << std::endl;
					               

                  


            pcl::PointCloud<RFType>::Ptr model_rf (new pcl::PointCloud<RFType> ());
    			  pcl::PointCloud<RFType>::Ptr scene_rf (new pcl::PointCloud<RFType> ());
    			  rf_est.setFindHoles (true);
				    rf_est.setRadiusSearch (rf_rad_);

				    rf_est.setInputCloud (modelObjects[j]);
				    rf_est.setInputNormals (model_normals_vector_[j]);
				    rf_est.setSearchSurface (modelObjects[j]);
				    rf_est.compute (*model_rf);

				    rf_est.setInputCloud (clusters_vector_[k]);
				    rf_est.setInputNormals (scene_normals_vector_[k]);
				    rf_est.setSearchSurface (clusters_vector_[k]);
				    rf_est.compute (*scene_rf);


                   
                    //Hough Clustere
				 	  clusterer.setHoughBinSize (cg_size_);
				    clusterer.setHoughThreshold (cg_thresh_);
				    clusterer.setUseInterpolation (true);
				    clusterer.setUseDistanceWeight (false);

				    clusterer.setInputCloud (modelObjects[j]);
				    clusterer.setInputRf (model_rf);
				    clusterer.setSceneCloud (clusters_vector_[k]);
				    clusterer.setSceneRf (scene_rf);
				    clusterer.setModelSceneCorrespondences (model_scene_corrs);

				    //clusterer.cluster (clustered_corrs);
				    clusterer.recognize (rototranslations, clustered_corrs);
				    std::cout <<  BOLD(FBLU("Model instances found of Cluster: "))<<k+1<<BOLD(FRED(" is for Object: "))<<j+1<<" is of size " <<rototranslations.size () << std::endl;

	/*			     for (std::size_t e = 0; e < rototranslations.size (); ++e)
  {
    std::cout << "\n    Instance " << e + 1 << ":" << std::endl;
    std::cout << "        Correspondences belonging to this instance: " << clustered_corrs[e].size () << std::endl;

    // Print the rotation matrix and translation vector
    Eigen::Matrix3f rotation = rototranslations[e].block<3,3>(0, 0);
    Eigen::Vector3f translation = rototranslations[e].block<3,1>(0, 3);

    printf ("\n");
    printf ("            | %6.3f %6.3f %6.3f | \n", rotation (0,0), rotation (0,1), rotation (0,2));
    printf ("        R = | %6.3f %6.3f %6.3f | \n", rotation (1,0), rotation (1,1), rotation (1,2));
    printf ("            | %6.3f %6.3f %6.3f | \n", rotation (2,0), rotation (2,1), rotation (2,2));
    printf ("\n");
    printf ("        t = < %0.3f, %0.3f, %0.3f >\n", translation (0), translation (1), translation (2));
  }*/
             if(rototranslations.size ()==1 || rototranslations.size ()>1)
          {

          	cluster_object_vector[k] = j+1;
          }

      		}



      }

   


}



void tamp_scene_class::findCorrespondences(){

	std::cout << BOLD(FCYN("Finding Correspondences ....... "))<<std::endl;
  
	computeNormals();
	computeDescriptor();
	modelSceneCorrespondeces();

}



void tamp_scene_class::findObjectCentre(){
 std::cout<<"centerofObjects_.clear()***************!!!!!!!!!!!!!!!"<<std::endl;
centerofObjects_.clear();
centerwrtbase_.clear();
//centerofObjects_.resize(cluster_object_vector.size());
//centerwrtbase_.resize(cluster_object_vector.size());
tf::Transform opticalframetobase;
opticalframetobase.setOrigin(tf::Vector3(0.301, 0.042, 0.723));
tf::Quaternion q(0.710, -0.687, 0.119, -0.098);
opticalframetobase.setRotation(q);
tf::Matrix3x3 Rab(q);
//Rab.getRotation(q);
//std::cout<<"before for()***************!!!!!!!!!!!!!!!"<<std::endl;
for(int i =0;i<cluster_object_vector.size();i++){
    Eigen::Vector4f centroid;
    pcl::compute3DCentroid (*clusters_vector_[i], centroid);
    centerofObjects_.push_back(centroid);
    tf::Transform objecttoopticalfarm;
    objecttoopticalfarm.setOrigin(tf::Vector3(centroid(0),centroid(1),centroid(2)));
    tf::Quaternion q;
    q.setRPY(0, 0, 0);
    objecttoopticalfarm.setRotation(q);
    tf::Vector3 ss = Rab * tf::Vector3(centroid(0),centroid(1),centroid(2)) + tf::Vector3(0.301, 0.042, 0.723);
    //[0.301, 0.042, 0.723]'+q*centroid
    //std::cout<<"objecttobase"<<std::endl;
    tf::Transform objecttobase;
    //std::cout<<"Multiplication Done!!!!!!!!!!!!!!!"<<std::endl;
    // 
    objecttobase.setOrigin(ss);
    //objecttobase.setOrigin(tf::Vector3(0.301-centroid(0),0.042-centroid(1),0.723-centroid(2)));
   // tf::Vector3 origin = objecttobase.getOrigin();
    //std::cout << "- Translation: [" << origin.getX() << ", " << origin.getY() << ", " << origin.getZ() << "]" << std::endl;
    centerwrtbase_.push_back(objecttobase);

}

    
}


void tamp_scene_class::boundBoxCube(){
    tf::Transform opticalframetobase;
    opticalframetobase.setOrigin(tf::Vector3(0.301, 0.042, 0.723));
    tf::Quaternion q(0.710, -0.687, 0.119, -0.098);
    opticalframetobase.setRotation(q);
    tf::Matrix3x3 Rcb(q);
   for(int i =0;i<cluster_object_vector.size();i++){
    

    string object;
  
   if(cluster_object_vector[i]==2)
   {  //vg.setInputCloud (clusters_vector_[i]);
      //vg.setLeafSize (0.01f, 0.01f, 0.01f);
      //vg.filter (*filteredCloudbox_);
      
     // cout<<BOLD(FGRN("Cropping Cloud"))<<endl;
     // boxFilter.setMin(Eigen::Vector4f(-0.4, -2.5, 0.02, 1));
    //  boxFilter.setMax(Eigen::Vector4f(0.5, 0.2, 0.7, 1));
     // boxFilter.setInputCloud(clusters_vector_[i]);
     // boxFilter.filter(*filteredCloudbox_);
      

      seg.setOptimizeCoefficients (true);
      seg.setModelType (pcl::SACMODEL_PLANE);
      seg.setMethodType (pcl::SAC_RANSAC);
      seg.setMaxIterations (100);
      seg.setDistanceThreshold (0.02);
      seg.setInputCloud (clusters_vector_[i]);
      seg.segment (*inliers_, *coefficients_);
      extract.setInputCloud (clusters_vector_[i]);
      extract.setIndices (inliers_);
      extract.setNegative (false);
      extract.filter (*filteredCloudbox_);
      extract.setNegative (true);
      extract.filter (*remainingfilteredCloudbox_);
      Eigen::Vector4f centroid1,centroid2;
      pcl::compute3DCentroid (*filteredCloudbox_, centroid1);
      pcl::compute3DCentroid (*remainingfilteredCloudbox_, centroid2);
      double dis1 = sqrt(centroid1(0)*centroid1(0)+centroid1(1)*centroid1(1)+centroid1(2)*centroid1(2));
      double dis2 = sqrt(centroid2(0)*centroid2(0)+centroid2(1)*centroid2(1)+centroid2(2)*centroid2(2));
      if(dis1<dis2){
        feature_extractor.setInputCloud (filteredCloudbox_);

      }
      else{
        feature_extractor.setInputCloud (remainingfilteredCloudbox_);
      }







      
      feature_extractor.compute ();
      feature_extractor.getMomentOfInertia (moment_of_inertia_);
      feature_extractor.getEccentricity (eccentricity_);
      feature_extractor.getAABB (min_point_AABB_, max_point_AABB_);
      feature_extractor.getOBB (min_point_OBB_, max_point_OBB_, position_OBB_, rotational_matrix_OBB_);
      feature_extractor.getEigenValues (major_value_, middle_value_, minor_value_);
      feature_extractor.getEigenVectors (major_vector_, middle_vector_, minor_vector_);
      feature_extractor.getMassCenter (mass_center_);

  Eigen::Vector3f position (position_OBB_.x, position_OBB_.y, position_OBB_.z);
 Eigen::Quaternionf quat2 (rotational_matrix_OBB_);



   if(dis1<dis2){
              addViewer(filteredCloudbox_,min_point_AABB_,max_point_AABB_,position,quat2,min_point_OBB_,max_point_OBB_);
;

      }
      else{
              addViewer(remainingfilteredCloudbox_,min_point_AABB_,max_point_AABB_,position,quat2,min_point_OBB_,max_point_OBB_);
;
      }







      boxsize_={max_point_OBB_.x - min_point_OBB_.x, max_point_OBB_.y - min_point_OBB_.y, max_point_OBB_.z - min_point_OBB_.z};
      
   



      tf::Transform objtocamera;
      Eigen::Quaternionf quat (rotational_matrix_OBB_);
 
      tf::Quaternion qobj(quat.x(),quat.y(), quat.z(), quat.w());
      tf::Matrix3x3 Roc(qobj);
      tf::Matrix3x3 Rob = Roc * Rcb;
      tf::Quaternion qo;
      
      double roll, pitch, yaw;
       //tf::Matrix3x3(q)
       Rob.getRPY(roll, pitch, yaw);
       tf::Matrix3x3 Rob2;
       Rob2.setEulerYPR(yaw,0.0,0.0);  
       Rob2.getRotation(qo);
      //objtocamera.setRotation(q);
     // objtocamera.setOrigin(tf::Vector3(position_OBB_.x, position_OBB_.y, position_OBB_.z));
      //tf::Transform objtobase =opticalframetobase*objtocamera ;
      //tf::Transform objtobaseinv = objtobase.inverse();
     // tf::Vector3 origin = objtobaseinv.getOrigin();
      //quatbox_ = objtobaseinv.getRotation();
      //positionbox_ ={origin.getX(), origin.getY(), origin.getZ()};
       positionbox_ = Rcb * tf::Vector3(position_OBB_.x, position_OBB_.y, position_OBB_.z) + tf::Vector3(0.301, 0.042, 0.723);
      tf::Vector3 ss = tf::Vector3(boxsize_[0],boxsize_[1],boxsize_[2]);

      // tf::Vector3 ss = Rcb * tf::Vector3(boxsize_[0],boxsize_[1],boxsize_[2]) + tf::Vector3(0.301, 0.042, 0.723);
       boxsizebase_ ={ss.getX(), ss.getY(), ss.getZ()};
       quatbox_ =qo;

   }
   
 
   //cout<<BOLD(FCYN("Cluster: "))<<i+1 <<" is "<< object<<" with center x: "<<centroid(0)<<" y: "<<centroid(1)<<" z: "<<centroid(2)<<endl;





 }


}


void tamp_scene_class::printClusterObject()
{
  
  //findObjectCentre();
  objectsNames_.clear();
  //objectsNames_.resize(cluster_object_vector.size());
  cout<<BOLD(FRED(" There are "))<<cluster_object_vector.size() <<BOLD(FRED(" Objects in the Scene"))<<endl;
  int cylcount=1;
  int cubecount=1;
  int uncount=1;
 for(int i =0;i<cluster_object_vector.size();i++){
    

 string object;
   if(cluster_object_vector[i]==1 )
   {
    object = "Cylinder_"+to_string(cylcount);
    cylcount++;
   }
   else if(cluster_object_vector[i]==2)
   {
    object = "Cube_"+to_string(cubecount);
    ++cubecount;
   }
   else
    {
     // object = "UnKnown_"+to_string(uncount);
      //++uncount;
       object = "Cylinder_"+to_string(cylcount);
       cylcount++;
    }

   Eigen::Vector4f centroid =centerofObjects_[i] ;
   objectsNames_.push_back(object);
   cout<<BOLD(FCYN("Cluster: "))<<i+1 <<" is "<< object<<" with center x: "<<centroid(0)<<" y: "<<centroid(1)<<" z: "<<centroid(2)<<endl;





 }
  

}

void tamp_scene_class::detectCircles(){

  cv::cvtColor(img_, gray_, CV_BGR2GRAY);
  cv::GaussianBlur( gray_, gray_, cv::Size(9, 9), 2, 2 );  
  cv::HoughCircles(gray_, circles_, CV_HOUGH_GRADIENT,2, gray_.rows/10, 200, 50,0,20 );
                 
    for( size_t i = 0; i < circles_.size(); i++ )
    {
         cv::Point center(cvRound(circles_[i][0]), cvRound(circles_[i][1]));
         int radius = cvRound(circles_[i][2]);
         // draw the circle center
         cv::circle( img_, center, 3, cv::Scalar(0,255,0), -1, 8, 0 );
         // draw the circle outline
         cv::circle( img_, center, radius, cv::Scalar(0,0,255), 3, 8, 0 );
         cout<<BOLD(FCYN("Circle center : "))<<i+1 <<" is "<< " with center x: "<<center.x<<" y: "<<center.y<<BOLD(FCYN(" With Radius: "))<<radius<<endl;

    }
   



}

 double tamp_scene_class::findangle( cv::Point pt1, cv::Point pt2, cv::Point pt0 )
{
    double dx1 = pt1.x - pt0.x;
    double dy1 = pt1.y - pt0.y;
    double dx2 = pt2.x - pt0.x;
    double dy2 = pt2.y - pt0.y;
    return (dx1*dx2 + dy1*dy2)/sqrt((dx1*dx1 + dy1*dy1)*(dx2*dx2 + dy2*dy2) + 1e-10);
}


 void tamp_scene_class::findSquares()
{
    squares.clear();
    int thresh = 50, N = 11;
    cv::Mat pyr, timg, gray0(img_.size(), CV_8U), gray;

    // down-scale and upscale the image to filter out the noise
    pyrDown(img_, pyr, Size(img_.cols/2, img_.rows/2));
    pyrUp(pyr, timg, img_.size());
    vector<vector<cv::Point> > contours;

    // find squares in every color plane of the image
    for( int c = 0; c < 3; c++ )
    {
        int ch[] = {c, 0};
        mixChannels(&timg, 1, &gray0, 1, ch, 1);

        // try several threshold levels
        for( int l = 0; l < N; l++ )
        {
            // hack: use Canny instead of zero threshold level.
            // Canny helps to catch squares with gradient shading
            if( l == 0 )
            {
                // apply Canny. Take the upper threshold from slider
                // and set the lower to 0 (which forces edges merging)
                cv::Canny(gray0, gray, 0, thresh, 5);
                // dilate canny output to remove potential
                // holes between edge segments
                cv::dilate(gray, gray, cv::Mat(), cv::Point(-1,-1));
            }
            else
            {
                // apply threshold if l!=0:
                //     tgray(x,y) = gray(x,y) < (l+1)*255/N ? 255 : 0
                gray = gray0 >= (l+1)*255/N;
            }

            // find contours and store them all as a list
            findContours(gray, contours, RETR_LIST, CHAIN_APPROX_SIMPLE);

            vector<cv::Point> approx;

            // test each contour
            for( size_t i = 0; i < contours.size(); i++ )
            {
                // approximate contour with accuracy proportional
                // to the contour perimeter
                approxPolyDP(contours[i], approx, arcLength(contours[i], true)*0.02, true);

                // square contours should have 4 vertices after approximation
                // relatively large area (to filter out noisy contours)
                // and be convex.
                // Note: absolute value of an area is used because
                // area may be positive or negative - in accordance with the
                // contour orientation
                if( approx.size() == 4 &&
                    fabs(contourArea(approx)) > 1000 &&
                    isContourConvex(approx) )
                {
                    double maxCosine = 0;

                    for( int j = 2; j < 5; j++ )
                    {
                        // find the maximum cosine of the angle between joint edges
                        double cosine = fabs(findangle(approx[j%4], approx[j-2], approx[j-1]));
                        maxCosine = MAX(maxCosine, cosine);
                    }

                    // if cosines of all angles are small
                    // (all angles are ~90 degree) then write quandrange
                    // vertices to resultant sequence
                    if( maxCosine < 0.3 )
                        squares.push_back(approx);
                }
            }
        }
    }
}

 void  tamp_scene_class::drawSquares()
{      
    for( size_t i = 0; i < squares.size(); i++ )
    {
        if(ifSquareinsideZone(squares[i])){
               
              const Point* p = &squares[i][0];
              int n = (int)squares[i].size();
              polylines(img_, &p, &n, 1, true, Scalar(0,255,0), 3, LINE_AA);

        }
        
    }

    //imshow(wndname, image);
}

void tamp_scene_class::detectSquares(){

  findSquares();
  drawSquares();

}

bool tamp_scene_class::ifSquareinsideZone(vector<cv::Point> square){
     
     int nurows =img_.rows;
     int nucols = img_.cols;
     bool allinside =true;
     for(std::size_t i=0;i<square.size();i++){
       
       if (square[i].x<nurows/4 || square[i].x>nurows*3/4 || square[i].y<nucols/4  || square[i].y>nucols*3/4)

          allinside =false;
          break;   
     }
     return allinside;

}

void tamp_scene_class::addViewer(pcl::PointCloud<PointT>::Ptr pc,PointT min,PointT max,Eigen::Vector3f position,Eigen::Quaternionf quat,PointT min2,PointT max2){

  viewer->setBackgroundColor (0, 0, 0);
  viewer->addCoordinateSystem (1.0);
  viewer->initCameraParameters ();
  viewer->addPointCloud<PointT> (pc, "sample cloud");
  viewer->addCube (min.x, max.x, min.y, max.y, min.z, max.z, 1.0, 1.0, 0.0, "AABB");
  viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION, pcl::visualization::PCL_VISUALIZER_REPRESENTATION_WIREFRAME, "AABB");
  
  
  //Eigen::Vector3f position (position_OBB.x, position_OBB.y, position_OBB.z);
  //Eigen::Quaternionf quat (rotational_matrix_OBB);
  viewer->addCube (position, quat, max2.x - min2.x, max2.y - min2.y, max2.z - min2.z, "OBB");
  viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION, pcl::visualization::PCL_VISUALIZER_REPRESENTATION_WIREFRAME, "OBB");





  while(!viewer->wasStopped())
  {
    viewer->spinOnce (100);
    //std::this_thread::sleep_for(100);
  }


}