#ifndef rt_H
#define rt_H

// ROS includes
#include "ros/ros.h"
#include "ros/time.h"
#include "tf/tf.h"

//ROS messages
#include <sensor_msgs/image_encodings.h>
#include <nav_msgs/Path.h>
#include "sscrovers_pmslam_common/SALVector.h"
#include "sscrovers_pmslam_common/SPoint.h"
#include "sscrovers_pmslam_common/PairedPoints3D.h"
#include "sscrovers_pmslam_common/PtPairs.h"
#include "sscrovers_pmslam_common/DynamicArray.h"

#include "sscrovers_pmslam_common/featureUpdateArray.h"
#include "sscrovers_pmslam_common/featureUpdate.h"

#include "sscrovers_pmslam_common/featureUpdate3DArray.h"
#include "sscrovers_pmslam_common/featureUpdate3D.h"

//filter headers
#include "filter_new/EKFFilter.h"

using std::string;

class rosTop{
	public:
		rosTop(ros::NodeHandle *_n);
		~rosTop();
	private:
		//! publisher
		ros::Publisher map_pub_, pose_pub_;

		//! subscribers
		ros::Subscriber local_features_3D_sub_, control_vec_sub_ ;

		void control_vec_callBack(const sscrovers_pmslam_common::control_vec& msg);
		void local_features_3D_callBack(const sscrovers_pmslam_common::featureUpdate3D& msg);
		EKFFilter newFilter;
		vec current_ctrlVec;
		vector<features3D> features;
		int step_feat, step_vec; 

		void publish();
}



#endif 
