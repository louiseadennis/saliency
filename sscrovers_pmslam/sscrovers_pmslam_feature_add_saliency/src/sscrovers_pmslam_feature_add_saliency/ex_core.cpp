#include "ex_core.h"
#include <iostream>
#include <sstream>
#include <fstream>

using namespace std;

EXCore::EXCore(ros::NodeHandle *_n)
{
  // Initialise node parameters from launch file or command line.
  // Use a private node handle so that multiple instances of the node can be run simultaneously
  // while using different parameters.
  ros::NodeHandle private_node_handle("~");
  private_node_handle.param("rate", rate_, int(10));
  //! topics name
private_node_handle.param("output_features_topic_name", sub_features_topic_name_, string("/saliency/features_Hou"));


  //! publishers
  features_pub_ = _n->advertise<sscrovers_pmslam_common::extraFeatures>("/saliency/features_added", 1);

  //! subscribers
  features_db_sub_ = _n->subscribe("SAL_db", 1, &EXCore::featuresDBCallBack, this);


  //pmslam functional module

}

EXCore::~EXCore()
{
}

void EXCore::featuresDBCallBack(const sscrovers_pmslam_common::SALVector& msg)
{
  //ROS_INFO("Sal_db found");
  if (msg.dims > 0)
  {
    sal_db.resize(msg.dims);
    memcpy(sal_db.data(), msg.data.data(), msg.dims * sizeof(sscrovers_pmslam_common::SPoint));
  }
  else
    ROS_ERROR("No data in database topic");

  process();

}



void EXCore::process()
{
	sscrovers_pmslam_common::extraFeatures out;
	out.number = sal_db.size();

	int size = sal_db.size();

	int i=0;
	for(int i=0;i<size;i++){

		sscrovers_pmslam_common::extraFeature tempExtra;
		float x = before.poses[i].orientation.x;
		float y = before.poses[i].orientation.y;
		float w = before.poses[i].orientation.w;
		float h = before.poses[i].orientation.z;
		

		//convert to extraFeature
		tempExtra.point1.x = x+w/2.0;
		tempExtra.point1.y = y+h/2.0;
		tempExtra.numbers=5;

		geometry_msgs::Point32 ptemp;
		ptemp.x = x;
		ptemp.y = y;
		tempExtra.extraPoints.push_back(ptemp);

		ptemp.x = x+w/2.0;
		ptemp.y = y+h/2.0;
		tempExtra.extraPoints.push_back(ptemp);

		ptemp.x = x;
		ptemp.y = y+h;
		tempExtra.extraPoints.push_back(ptemp);

		ptemp.x = x+w;
		ptemp.y = y;
		tempExtra.extraPoints.push_back(ptemp);

		ptemp.x = x+w;
		ptemp.y = y+h;
		tempExtra.extraPoints.push_back(ptemp);

		out.extras.push_back(tempExtra);
	}
	features_pub_.publish(out);
	


}





//-----------------------------MAIN------------------------------
int main(int argc, char **argv)
{
  // Set up ROS.
  ros::init(argc, argv, "ES");
  ros::NodeHandle n;

  // Create a new NodeExample object.
  EXCore *ex_core = new EXCore(&n);

  //until we use only img callback, below is needless
  // Tell ROS how fast to run this node.
  ros::Rate r(ex_core->rate_);

  // Main loop.

  while (n.ok())
  {
    //dp_core->Publish();
    ros::spinOnce();
    r.sleep();

  }

  //ros::spin();

  return 0;
} // end main()
