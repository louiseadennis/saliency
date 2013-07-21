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
ROS_INFO("Got Image");
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
ROS_INFO("Got Surf");

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
			descriptor desc = descriptor(x1,y1);
			for (int o=0; o<keypoints_1.size(); o++) {
				float xx = keypoints_1[o].pt.x;
				float yy = keypoints_1[o].pt.y;	
				if ( x1 < xx && xx < x2 && y1 < yy && yy < y2){
					desc.add(descriptors_1, o);
				}
			}
			int out;
			if (database.inBundle(desc,out)){
			ROS_INFO("old %i", out);
			}else{
			ROS_INFO("Add");
				database.add(desc);
			}
			
		}

	}else{
		ROS_ERROR("Crash, Mismatch");
	}
}


/******* surf_bundle **************/





bool surf_bundle::inBundle(descriptor din, int &outputValue){
	double sum;
	int j=-1;
	double min = 1e6;
	bool test =false;
	for ( int i=0; i<bundle.size(); i++  ) {
		if( bundle[i].compare(din, sum)){
			test = true;
			if (min > sum){
				min = sum;
				j=i;
			}
		}
	}

	outputValue = j;
	return test;
};


void surf_bundle::add(descriptor newM){
	int y = bundle.size();
	newM.id = y;
	bundle.push_back(newM);
};


surf_bundle::surf_bundle(){

};


/*****	descriptor **************/

double compareSURFDescriptors(std::vector <double > d1, std::vector <double > d2, double best)
{
  double total_cost = 0;
  for (int i = 0; i < 64; i += 4)
  {
    double t0 = d1[i] - d2[i];
    double t1 = d1[i + 1] - d2[i + 1];
    double t2 = d1[i + 2] - d2[i + 2];
    double t3 = d1[i + 3] - d2[i + 3];
    total_cost += t0 * t0 + t1 * t1 + t2 * t2 + t3 * t3;
    if (total_cost > best)
      break;
  }
  return total_cost;
}

descriptor::descriptor(double xx, double yy){
	x=xx;
	y=yy;
};
descriptor::~descriptor(){
};

void descriptor::add(Mat &ds, int i){
	std::vector < double > fed;
	for(int j=0;j<64 ;j++){
		 fed.push_back(ds.at<float>(i,j));
	}
	d.push_back(fed);
};

bool descriptor::compare(descriptor dd, double &average){
	double sum = 0;
	bool test = false;
	double len =0;
	double dq, dist1 = 1e6, dist2 = 1e6;
	for (int i=0; i<d.size();i++){
		for (int j=0; j<dd.d.size();j++){
			dq = compareSURFDescriptors(d[i],dd.d[j],dist2);
			if (dq < dist1)
			{
				dist2 = dist1;
				dist1 = dq;
			}
			else if (dq < dist2)
				dist2 = dq;
		}
		if (dist1 < 0.6 * dist2) {
			sum+=dist1;
			ROS_INFO("dist %f", dist1);
			test = true;
			len++;
		}
	}
	average = sum/len;
	return test;
}



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

