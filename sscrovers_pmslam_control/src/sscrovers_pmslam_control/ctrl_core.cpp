#include "ctrl_core.h"
#include <math.h> 

using std::string;


CtrlCore::CtrlCore(ros::NodeHandle *_n) :
    it_(*_n)
{
  wait_time_ = ros::Duration(3.0);

  // Initialise node parameters from launch file or command line.
  // Use a private node handle so that multiple instances of the node can be run simultaneously
  // while using different parameters.
  ros::NodeHandle private_node_handle("~");
  private_node_handle.param("rate", rate_, int(10));
  private_node_handle.param("sub_image_topic_name", sub_image_topic_name_, string("cam_image"));
  private_node_handle.param("pub_image_topic_name", pub_image_topic_name_, string("output_image"));
  private_node_handle.param("sub_odometry_topic_name", sub_odom_topic_name_, string("odom"));
  private_node_handle.param("pub_ctrl_vec_topic_name", pub_ctrl_vec_topic_name_, string("ctrl_vec"));
  private_node_handle.param("xy_noise", xy_noise_, double(0.0));
  private_node_handle.param("z_noise", z_noise_, double(0.0));
  private_node_handle.param("yaw_noise", yaw_noise_, double(0.0));
  private_node_handle.param("disp_img", disp_input_image_, bool(false));


  //subscribers
  image_sub_ = it_.subscribe(sub_image_topic_name_.c_str(), 0, &CtrlCore::imageCallBack, this);
  odom_sub_ = _n->subscribe(sub_odom_topic_name_.c_str(), 0, &CtrlCore::odomCallBack, this);

  //publishers
  image_pub_ = it_.advertise(pub_image_topic_name_.c_str(), 1);
  ctrl_vec_pub_ = _n->advertise<sscrovers_pmslam_common::control_vec>(pub_ctrl_vec_topic_name_.c_str(), 1);

  cv_input_img_ptr_.reset(new cv_bridge::CvImage);
  cv_output_img_ptr_.reset(new cv_bridge::CvImage);


  img_callback_inv = false;
  odom_callback_inv = false;

  curr_step_ = 0;

}

CtrlCore::~CtrlCore()
{
  if (disp_input_image_)
    cv::destroyWindow(WINDOW);
}

void CtrlCore::resetTime() //TODO check this
{
  time_ = ros::Time::now();
}

bool CtrlCore::waitForData() //TODO check this
{
  //bool time_cond = ((ros::Time::now()-time_)>wait_time_);
  //if(time_cond) ROS_
  return !((img_callback_inv) && (odom_callback_inv));
}

void CtrlCore::process()
{
  curr_step_++; //TODO remove

  //process received data

  processImage();

  //publish everything
  publishCtrlVector();

  publishImage();

  img_callback_inv = false;
  odom_callback_inv = false;
}



void CtrlCore::processImage()
{
  cv_input_img_ptr_->header.stamp.nsec = curr_step_;

  *cv_output_img_ptr_ = *cv_input_img_ptr_;

}

void CtrlCore::publishImage()
{
  image_pub_.publish(cv_output_img_ptr_->toImageMsg());
}



void CtrlCore::publishCtrlVector()
{
  ctrl_vec_msg_.header.stamp.nsec = curr_step_;

  float d2 = curr_pose_msg_.pose.position.x + curr_pose_msg_.pose.position.y;
  float d = sqrt(d2);
  float theta = atan(curr_pose_msg_.pose.position.y/curr_pose_msg_.pose.position.x);
  ctrl_vec_msg_.d = d;
  ctrl_vec_msg_.theta = theta;
  ctrl_vec_pub_.publish(ctrl_vec_msg_);
}

void CtrlCore::odomCallBack(const nav_msgs::OdometryConstPtr& msg)
{
  odom_callback_inv = true;
  //! push forward time stamp
  curr_pose_msg_.header.stamp = msg->header.stamp;
  curr_pose_msg_.pose = msg->pose.pose;


}

void CtrlCore::imageCallBack(const sensor_msgs::ImageConstPtr& msg)
{
  img_callback_inv = true;
  //! push forward time stamp
  cv_input_img_ptr_->header.stamp = msg->header.stamp;

  try
  {
    cv_input_img_ptr_ = cv_bridge::toCvCopy(msg, enc::MONO8);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }

  if (disp_input_image_)
  {
    cv::imshow(WINDOW, cv_input_img_ptr_->image);
    cv::waitKey(3);
  }
}

//-----------------------------MAIN----------------------------
int main(int argc, char **argv)
{
  // Set up ROS.
  ros::init(argc, argv, "control_module");
  ros::NodeHandle n;

  // Create a new NodeExample object.
  CtrlCore *ctrl_core = new CtrlCore(&n);

  ros::Rate r(ctrl_core->rate_);

  // Main loop.

  while (n.ok())
  {
    while (n.ok() && ctrl_core->waitForData())
    {
      ros::spinOnce();
    }
    ctrl_core->process();
    r.sleep();
    //ROS_INFO("curr_step: %ld", ctrl_core->curr_step_);
  }

  return 0;
} // end main()
