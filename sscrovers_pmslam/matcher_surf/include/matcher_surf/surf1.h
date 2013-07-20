#ifndef hu_H
#define hu_H

// ROS includes
#include "ros/ros.h"
#include "ros/time.h"
#include "tf/tf.h"

//ROS messages


#include "sscrovers_pmslam_common/featureUpdateArray.h"
#include "sscrovers_pmslam_common/featureUpdate.h"
#include "sscrovers_pmslam_common/control_vec.h"

#include "sscrovers_pmslam_common/featureUpdate3DArray.h"
#include "sscrovers_pmslam_common/featureUpdate3D.h"

#include "sscrovers_pmslam_common/featureMap.h"

#include "image_transport/image_transport.h"
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>

// OpenCV
#include "opencv/cv.h"
#include "opencv/highgui.h"
#include <opencv2/nonfree/features2d.hpp>

#include "geometry_msgs/Point32.h"



#include <math.h>


#define DIST 0.005

using std::string;
using namespace cv;
namespace enc = sensor_msgs::image_encodings;


class surf_bundle{
	public:
		surf_bundle();
		surf_bundle(std::vector<KeyPoint> newM);
		std::vector< std::vector <KeyPoint> > bundle;
		void add(std::vector<KeyPoint> newM);
		bool inBundle(std::vector<KeyPoint>, double dist,double &outputValue);
};


class surf{
	public:
		surf(ros::NodeHandle *_n);
		~surf();

		//! rate for node main loop
		int rate_;
	private:

		//! features subscriber
		ros::Subscriber featureMap_sub_;
		//! db publisher
		ros::Publisher db_pub_;
		int step_;

		void featureMapCallback(const sscrovers_pmslam_common::featureMap& msg);

		std::vector<surf_bundle> database;

		std::vector <geometry_msgs::Point32> prev, now;


		bool first;
};





#endif 
