#ifndef DP_CORE_H
#define DP_CORE_H

// ROS includes
#include "ros/ros.h"
#include "ros/time.h"
#include "tf/tf.h"

//OpenCV
#include <cv.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

//ROS messages
#include <sensor_msgs/image_encodings.h>
#include "image_transport/image_transport.h"
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include "geometry_msgs/PoseArray.h"
#include "sscrovers_pmslam_common/featureMap.h"

#include "sscrovers_pmslam_common/extraFeature.h"
#include "sscrovers_pmslam_common/extraFeatures.h"
#include "geometry_msgs/Point32.h"


using std::string;

class DPCore
{
public:
  //! Constructor.
  DPCore(ros::NodeHandle *_n);

  //! Destructor.
  ~DPCore();

  //! rate for node main loop
  int rate_;

  //!Everything in this function is processed in node loop
  void process();


private:
  //! save to file flags
  bool traj_to_file_f_;
  bool features_to_file_f_;

  //! topic names
  string sub_features_topic_name_;

  //! publisher
  ros::Publisher features_pub_;

  //! subscribers
  ros::Subscriber features_sub_, featureMap_sub_;

  geometry_msgs::PoseArray before, after;
  bool test;
  sscrovers_pmslam_common::extraFeatures readIn;


  //! current step
  int step_;

  void addFeature(cv::Mat *image, float x, float y);

  void extraFeatureCallback(const sscrovers_pmslam_common::extraFeatures& msg);
};

#endif //DP_CORE_H
