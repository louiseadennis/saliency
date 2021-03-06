#include "matcher_surf/surf.h"
#include <iostream>
#include <sstream>
#include <fstream>
#include <math.h>


surf::surf(ros::NodeHandle *_n){
	first =true;
	ros::NodeHandle private_node_handle("~");
	private_node_handle.param("rate", rate_, int(10));

	//subscribers
	featureMap_sub_ = _n->subscribe("/saliency/features_Hou_Maps", 1, &surf::featureMapCallback, this);

	//publishers
	db_pub_ = _n->advertise<sscrovers_pmslam_common::featureUpdateArray>("SAL_db", 1);

};

surf::~surf(){};

void surf::featureMapCallback(const sscrovers_pmslam_common::featureMap& msg){
	if(first){

		first = false;
		for (int i=0; i<msg.imgs.size(); i++){
			//static cv::Scalar red_color[] ={0,0,255};
			cv_bridge::CvImagePtr cv_ptr;
			Mat image_;
			try
			{
			
				cv_ptr = cv_bridge::toCvCopy(msg.imgs[i], enc::MONO8);
			}
			catch (cv_bridge::Exception& e)
			{
				ROS_ERROR("cv_bridge exception: %s", e.what());
			}
			cv_ptr->image.copyTo(image_);

			int minHessian = 400;

			cv::SurfFeatureDetector detector( minHessian );

			std::vector<KeyPoint> keypoints_1;

			detector.detect( image_, keypoints_1 );


			//-- Step 2: Calculate descriptors (feature vectors)
			SurfDescriptorExtractor extractor;

			Mat descriptors_1;

			extractor.compute( image_, keypoints_1, descriptors_1 );


			for(int y=0; y<keypoints_1.size();y++){
				//descriptors_1.at<double>(y,0) = i;
				ROS_INFO("sdf1 %f", descriptors_1.at<double>(y,0));
				ROS_INFO("sdf2 %f", descriptors_1.at<double>(y,1));
				ROS_INFO("sdf3 %f", descriptors_1.at<double>(y,2));
			}
		}





	}else{



		for (int i=0; i<msg.imgs.size(); i++){
			//static cv::Scalar red_color[] ={0,0,255};
			cv_bridge::CvImagePtr cv_ptr;
			Mat image_;
			try
			{
			
				cv_ptr = cv_bridge::toCvCopy(msg.imgs[i], enc::MONO8);
			}
			catch (cv_bridge::Exception& e)
			{
				ROS_ERROR("cv_bridge exception: %s", e.what());
			}
			cv_ptr->image.copyTo(image_);

			int minHessian = 400;

			cv::SurfFeatureDetector detector( minHessian );

			std::vector<KeyPoint> keypoints_1;

			detector.detect( image_, keypoints_1 );


			for(int y=0; y<keypoints_1.size();y++){
				keypoints_1[y].class_id = i;
				//ROS_INFO("sdf %i", keypoints_1[y].class_id);
			}
		}




		step_ = msg.header.stamp.nsec;
		sscrovers_pmslam_common::featureUpdateArray tempArray;
		tempArray.header = msg.header;
		vector <sscrovers_pmslam_common::featureUpdate> tempVec;



	}
};




bool surf_bundle::inBundle(std::vector<KeyPoint>, double dist, double &outputValue){
	for (int i=0; i<bundle.size();i++){
		//outputValue = bundle.at(i).similarity(newM);
		//if ( outputValue < dist){
		//	return true;
		//}
	}
	return false;
};


void surf_bundle::add(std::vector<KeyPoint> newM){
	if (bundle.size()<2){
		bundle.push_back(newM);
	}else{

		bundle.push_back(newM);

	}
};

surf_bundle::surf_bundle(std::vector<KeyPoint> newM){
	bundle.push_back(newM);
}


int main(int argc, char **argv)
{

  // Set up ROS.
  ros::init(argc, argv, "hu_moments");
  ros::NodeHandle n;

  // Create a new NodeExample object.
  surf *surf_core = new surf(&n); // __atribute__... only for eliminate unused variable warning :)


  ros::Rate r(surf_core->rate_);

  // Main loop.

  while (n.ok())
  {
    ros::spinOnce();
    r.sleep();
  }

  return 0;


}

