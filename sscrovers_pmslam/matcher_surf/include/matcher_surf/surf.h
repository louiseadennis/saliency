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

#include <geometry_msgs/PoseArray.h>

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

class descriptor
{
public:
	double x,y;
	int id;
	std::vector < std::vector < double > > d;
	descriptor(double, double);
	void add(Mat&,int);
	~descriptor();
	bool compare(descriptor dd, double &average);
};

class surf_bundle{
public:
	surf_bundle();

	std::vector<descriptor> bundle;
	void add(descriptor newM);
	bool inBundle(descriptor in, int &outputValue);
};



class surf{
	public:
		surf(ros::NodeHandle *_n);
		~surf();

		int step1_, step2_;
		//! rate for node main loop
		int rate_;
	private:
    		image_transport::ImageTransport it_;
    		image_transport::Subscriber 	image_sub_;
		//! features subscriber
		ros::Subscriber feature_sub_;
		//! db publisher
		ros::Publisher db_pub_;
		int step_;

		void imageCallback(const sensor_msgs::ImageConstPtr& msg_ptr);
		void featureCallback(const geometry_msgs::PoseArray& msg);
		void process();
		surf_bundle database;

		std::vector <geometry_msgs::Point32> prev, now;
		std::vector<geometry_msgs::Pose> latestFeat;

		bool first;

		Mat descriptors_1;
		std::vector<KeyPoint> keypoints_1;
};





#endif 
