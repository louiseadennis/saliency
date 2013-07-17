#include "hu_moments/hu.h"
#include <iostream>
#include <sstream>
#include <fstream>
#include <math.h>

hu::hu(ros::NodeHandle *_n){
	ros::NodeHandle private_node_handle("~");
	private_node_handle.param("rate", rate_, int(10));

	//subscribers
	featureMap_sub_ = _n->subscribe("/saliency/features_Hou_Maps", 1, &hu::featureMapCallback, this);

	//publishers
	db_pub_ = _n->advertise<sscrovers_pmslam_common::featureUpdateArray>("SAL_db", 1);

};

hu::~hu(){};

void hu::featureMapCallback(const sscrovers_pmslam_common::featureMap& msg){
	step_ = msg.header.stamp.nsec;
	sscrovers_pmslam_common::featureUpdateArray tempArray;
	tempArray.header = msg.header;
	vector <sscrovers_pmslam_common::featureUpdate> tempVec;
	for (int i=0; i<msg.imgs.size(); i++){
		cv_bridge::CvImagePtr cv_ptr;
		Mat image_;
		try
		{
			
			cv_ptr = cv_bridge::toCvCopy(msg.imgs[i], enc::BGR8);
		}
		catch (cv_bridge::Exception& e)
		{
			ROS_ERROR("cv_bridge exception: %s", e.what());
		}
		cv_ptr->image.copyTo(image_);

		vector<vector<Point> > contours;
		findContours(image_, contours, CV_RETR_LIST, CV_CHAIN_APPROX_NONE);
		double *huArray;
		Moments mm = moments(contours,true);
		HuMoments( mm , huArray);
		 
		moment newMoment = moment(huArray[0], huArray[1], huArray[2], huArray[3], huArray[4], huArray[5], huArray[6]);
		//matcher
		bool testIn = false;
		int intIn = -1;
		for (int j=0; j<database.size(); j++){
			testIn = database.at(j).inBundle(newMoment);
			if(testIn){
				intIn = j;
				break;
			}
		}
		sscrovers_pmslam_common::featureUpdate temp;
		if (testIn){
			database[intIn].add(newMoment);

			//id = intIn & exists = false
			temp.id = intIn;
			temp.exists = false;

			temp.x = msg.points[i].x;
			temp.y = msg.points[i].y;

		}else{
			database.push_back(moment_bundle(newMoment));

			// id = database.size() & exists =true
			temp.id = database.size();
			temp.exists = true;

			temp.x = msg.points[i].x;
			temp.y = msg.points[i].y;
		}
		tempVec.push_back(temp);
	}
	
	tempArray.features = tempVec;
	db_pub_.publish(tempArray);
};


moment::moment(){
};

moment::moment(double ah0,double ah1,double ah2,double ah3,double ah4,double ah5,double ah6){
	h0 = ah0;
	h1 = ah1;
	h2 = ah2;
	h3 = ah3;
	h4 = ah4;
	h5 = ah5;
	h6 = ah6;
};


double moment::similarity(moment h){

	double ma0 = (h0/abs(h0)) *  log10(h0);
	double mb0 = (h.h0/abs(h.h0)) *  log10(h.h0);

	double ma1 = (h1/abs(h1)) *  log10(h1);
	double mb1 = (h.h1/abs(h.h1)) *  log10(h.h1);

	double ma2 = (h2/abs(h2)) *  log10(h2);
	double mb2 = (h.h2/abs(h.h2)) *  log10(h.h2);

	double ma3 = (h3/abs(h3)) *  log10(h3);
	double mb3 = (h.h3/abs(h.h3)) *  log10(h.h3);

	double ma4 = (h4/abs(h4)) *  log10(h4);
	double mb4 = (h.h4/abs(h.h4)) *  log10(h.h4);

	double ma5 = (h5/abs(h5)) *  log10(h5);
	double mb5 = (h.h5/abs(h.h5)) *  log10(h.h5);

	double ma6 = (h6/abs(h6)) *  log10(h6);
	double mb6 = (h.h6/abs(h.h6)) *  log10(h.h6);
	
	return abs(ma0  - mb0) + abs(ma1  - mb1) + abs(ma2  - mb2) + abs(ma3  - mb3) + abs(ma4  - mb4) + abs(ma5  - mb5) + abs(ma6  - mb6); 
};


bool moment_bundle::inBundle(moment newM){
	for (int i=0; i<bundle.size();i++){
		if ( bundle.at(i).similarity(newM) < DIST){
			return true;
		}
	}
	return false;
};


void moment_bundle::add(moment newM){
	if (bundle.size()<2){
		bundle.push_back(newM);
	}else{
		if (bundle.at(bundle.size() -2).similarity(newM) >0.5 && bundle.at(bundle.size() -1).similarity(newM) <0.5){
			bundle.push_back(newM);
		}		
	}
};

moment_bundle::moment_bundle(moment newM){
	bundle.push_back(newM);
}


int main(int argc, char **argv)
{

  // Set up ROS.
  ros::init(argc, argv, "hu_moments");
  ros::NodeHandle n;

  // Create a new NodeExample object.
  hu *hu_core = new hu(&n); // __atribute__... only for eliminate unused variable warning :)


  ros::Rate r(hu_core->rate_);

  // Main loop.

  while (n.ok())
  {
    ros::spinOnce();
    r.sleep();
  }

  return 0;


}

