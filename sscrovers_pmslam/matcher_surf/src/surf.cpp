#include "matcher_surf/surf.h"
#include <iostream>
#include <sstream>
#include <fstream>
#include <math.h>


surf::surf(ros::NodeHandle *_n): it_(*_n) {
	first =true;
	ros::NodeHandle private_node_handle("~");
	private_node_handle.param("rate", rate_, int(10));

	//subscribers
	feature_sub_ = _n->subscribe("/saliency/features_Hou", 1, &surf::featureCallback, this);
    	image_sub_ = it_.subscribe("/cam_image", 1, &surf::imageCallback, this);
	//publishers
	db_pub_ = _n->advertise<sscrovers_pmslam_common::featureUpdateArray>("SAL_db", 1);

	
	step1_ = 0;
	step2_ = 0;
};

surf::~surf(){};

void surf::featureCallback(const geometry_msgs::PoseArray& msg){
	step1_ = msg.header.stamp.nsec;
	latestFeat = msg.poses;
	process();
};


void surf::imageCallback(const sensor_msgs::ImageConstPtr& msg_ptr){

	cv_bridge::CvImagePtr cv_ptr;
	Mat image_;
	try
	{
		cv_ptr = cv_bridge::toCvCopy(msg_ptr, enc::BGR8);
	}
	catch (cv_bridge::Exception& e)
	{
		ROS_ERROR("cv_bridge exception: %s", e.what());
	}
	cv_ptr->image.copyTo(image_);
	step2_ = cv_ptr->header.stamp.nsec;
	//-- Step 1: Detect the keypoints using SURF Detector	
	int minHessian = 400;
	cv::SurfFeatureDetector detector( minHessian );
	detector.detect( image_, keypoints_1 );


	//-- Step 2: Calculate descriptors (feature vectors)
	SurfDescriptorExtractor extractor;
	extractor.compute( image_, keypoints_1, descriptors_1 );

/*
	Mat img_keypoints_1;
	drawKeypoints( image_, keypoints_1, img_keypoints_1, Scalar::all(-1), DrawMatchesFlags::DEFAULT );

	//-- Show detected (drawn) keypoints
	imshow("Keypoints 1", img_keypoints_1 );


	waitKey(0);
*/
}


void surf::process(){
	if(step1_ == step2_){	
		for ( int u=0; u<latestFeat.size(); u++ ) {
			int x1 = latestFeat[u].orientation.x;
			int y1 = latestFeat[u].orientation.y;
			int x2 = latestFeat[u].orientation.x + latestFeat[u].orientation.w; 
			int y2 = latestFeat[u].orientation.y + latestFeat[u].orientation.z;
			std::vector < descriptor > descs;
			descriptor desc = descriptor();
			for (int o=0; o<keypoints_1.size(); o++) {
				float xx = keypoints_1[o].pt.x;
				float yy = keypoints_1[o].pt.y;	

				
				if ( x1 < xx && xx < x2 && y1 < yy && yy < y2){
					desc.add(descriptors_1, o, xx, yy);
					ROS_INFO("%i", descriptors_1.cols);
					
				}
				descs.push_back(desc);
			}
			
		}

	}else{
		ROS_ERROR("Crash, Mismatch");
	}
}


bool surf_bundle::inBundle(std::vector<descriptor>, double dist, double &outputValue){
	for (int i=0; i<bundle.size();i++){
		//outputValue = bundle.at(i).similarity(newM);
		if ( outputValue < dist){
			return true;
		}
	}
	return false;
};


void surf_bundle::add(std::vector<descriptor> newM){
	if (bundle.size()<2){
		bundle.push_back(newM);
	}else{

		bundle.push_back(newM);

	}
};

surf_bundle::surf_bundle(std::vector<descriptor> newM){
	bundle.push_back(newM);
};




/*****	descriptor **************/

descriptor::descriptor(){
	d = Mat(1,64,CV_64FC1);
	x=0.0;
	y=0.0;
};
descriptor::~descriptor(){
};

void descriptor::add(Mat &ds, int i ,double xx, double yy){
	Mat temp(d.rows+1,64,CV_64FC1);

	for(int j=0;j<d.rows ;j++){
		 d.row(j).copyTo(temp.row(j));
	}
	ds.row(i).copyTo(temp.row(temp.rows));
	d = temp;
	x = xx;
	y = yy;
};





/********************************/




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

