#include "dp_core.h"
#include <iostream>
#include <sstream>
#include <fstream>
#include <math.h>

using namespace std;

DPCore::DPCore(ros::NodeHandle *_n)
{
  // Initialise node parameters from launch file or command line.
  // Use a private node handle so that multiple instances of the node can be run simultaneously
  // while using different parameters.
  ros::NodeHandle private_node_handle("~");
  private_node_handle.param("rate", rate_, int(10));
  //! topics name
  private_node_handle.param("sub_traj_topic_name", sub_traj_topic_name_, string("est_traj"));
  private_node_handle.param("sub_pt_topic_name", sub_pt_topic_name_, string("ptpairs"));
  private_node_handle.param("pub_pt_topic_name", pub_pt_topic_name_, string("points3d"));
  private_node_handle.param("sub_db_topic_name", sub_db_topic_name_, string("sal_db"));
  private_node_handle.param("sub_features_topic_name", sub_features_topic_name_, string("features"));
  //! save to file flags
  private_node_handle.param("traj_to_file", traj_to_file_f_, bool(false));
  private_node_handle.param("features_to_file", features_to_file_f_, bool(false));

  //! publishers
  points3d_pub_ = _n->advertise<sscrovers_pmslam_common::featureUpdate3DArray>("localFeat3D", 1);

  //! subscribers
  features_db_sub_ = _n->subscribe("SAL_db", 1, &DPCore::featuresDBCallBack, this);

  //pmslam functional module

  step_ = -1;
}

DPCore::~DPCore()
{
}

void DPCore::process()
{
  //put here everything that should run with node
  if (step_ >= 0)
  {
      out.features.clear();
      out.header.stamp.nsec = step_;	
      double _rng = 10;
      int px = 320;
      int py = 240;
      double VFOV = 52.2;
      double HFOV = 66.7;
      double pan = 0.0;
      double tilt = 45.0;
      double CamH = 0.52;
      for(int i=0;i<arrt.features.size();i++){
	sscrovers_pmslam_common::featureUpdate3D temp = direct_depth(arrt.features[i], CamH, VFOV,HFOV, pan, tilt);
	out.features.push_back(temp);
      }	

  }
  publishPoints3D();
}



void DPCore::featuresDBCallBack(const sscrovers_pmslam_common::featureUpdateArray& msg)
{
  step_ = msg.header.stamp.nsec;
  arrt = msg;

  process();

}

sscrovers_pmslam_common::featureUpdate3D DPCore::direct_depth(sscrovers_pmslam_common::featureUpdate ft, double camh, double vfov, double hfov, double pan, double tilt)
{
	double u, v, x_coord_3D, y_coord_3D, z_coord_3D;
	sscrovers_pmslam_common::featureUpdate3D temp;
	double x_coord = ft.x;
	double y_coord = ft.y;

	u = (double)x_coord/320;//TODO
	v = (double)y_coord/240;//TODO

	temp.y = camh/tan(tilt+atan((2*v-1)*tan(vfov/2)));
	temp.x = temp.y * (2*u - 1) * tan(hfov/2);
	temp.z = 0;
	temp.id = ft.id;
	temp.exists = ft.exists;

	return temp;
}

void DPCore::publishPoints3D()
{
  points3d_pub_.publish(out);
}

//-----------------------------MAIN------------------------------
int main(int argc, char **argv)
{
  // Set up ROS.
  ros::init(argc, argv, "depth_perception");
  ros::NodeHandle n;

  // Create a new NodeExample object.
  DPCore *dp_core = new DPCore(&n);

  //until we use only img callback, below is needless
  // Tell ROS how fast to run this node.
  ros::Rate r(dp_core->rate_);

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
