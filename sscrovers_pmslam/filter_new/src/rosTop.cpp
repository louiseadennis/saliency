#include "filter_new/rosTop.h"
#include <iostream>
#include <sstream>
#include <fstream>
#include <math.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

using namespace std;
using namespace arma;

rosTop::rosTop(ros::NodeHandle *_n){

	ros::NodeHandle private_node_handle("~");
	private_node_handle.param("rate", rate_, int(10));

        ctrl_vec_sub_ = _n->subscribe("ctrl_vec", 1, &rosTop::control_vec_callBack, this);
	local_features_3D_sub_ = _n->subscribe("localFeat3D", 1, &rosTop::local_features_3D_callBack, this);

	marker_pub = _n->advertise<visualization_msgs::Marker>("rover_state", 1);
	marker_array_pub = _n->advertise<visualization_msgs::MarkerArray>("rocks", 1);

	checkVec = false;
	check3D = false;
	highID = 0;
}


rosTop::~rosTop(){}

void rosTop::process(){
	if((checkVec) && (check3D)){
		checkVec = false;
		check3D = false;
//ROS_INFO("%i  ==  %i",step_vec,step_feat);
		if(step_vec == step_feat){
//ROS_INFO("update");
		 	newFilter.update(features);
//ROS_INFO("augment");
		    	newFilter.augment(features);
		}else{
			ROS_ERROR("Crash, mismatch");
		}

		publish();
	}
}

void rosTop::control_vec_callBack(const sscrovers_pmslam_common::control_vec& msg){
	//ROS_INFO("Recieved control vector - %i", msg.header.stamp.nsec);
	step_vec  = msg.header.stamp.nsec; 
	vec temp ;
	temp << msg.d << endr << msg.theta;
	current_ctrlVec	= temp;	
	//ROS_INFO("control prediction");
	newFilter.predict(current_ctrlVec);
	//ROS_INFO("control predicted");
	checkVec=true;
	//ROS_INFO("Predicted ctrl");
}

void rosTop::local_features_3D_callBack(const sscrovers_pmslam_common::featureUpdate3DArray& msg){
	//ROS_INFO("Recieved features update - %i", msg.header.stamp.nsec);
	step_feat  = msg.header.stamp.nsec; 
	features.clear();
	for(int i=0; i<msg.features.size(); i++){
		features.push_back(features3D(msg.features[i].id,msg.features[i].x,msg.features[i].y,msg.features[i].z,msg.features[i].exists));
		if(msg.features[i].id > highID){
			highID = msg.features[i].id;
		}

		
	}
	check3D = true;
	process();
}


void rosTop::publish(){
	vec out = newFilter.getRoverState();
	//cout << " state prediction " << step_feat << endl << out;
	visualization_msgs::MarkerArray markerArray;

	std::cout << out;

	/******  Robot placer Rviz****************/
  	uint32_t shape = visualization_msgs::Marker::CUBE;
	visualization_msgs::Marker marker;
	// Set the frame ID and timestamp.  See the TF tutorials for information on these.
	marker.header.frame_id = "/map";
	marker.header.stamp = ros::Time::now();

	// Set the namespace and id for this marker.  This serves to create a unique ID
	// Any marker sent with the same namespace and id will overwrite the old one
	marker.ns = "basic_shapes";
	marker.id = 0;

	// Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
	marker.type = visualization_msgs::Marker::MESH_RESOURCE;

	marker.mesh_resource = "package://pr2_description/meshes/base_v0/base.dae";

	// Set the marker action.  Options are ADD and DELETE
	marker.action = visualization_msgs::Marker::ADD;

	marker.pose.position.x = out(0);
	marker.pose.position.y = out(1);
	marker.pose.position.z = 0.0;
	marker.pose.orientation.x = 0.0;
	marker.pose.orientation.y = 0.0;
	marker.pose.orientation.z = out(2);
	marker.pose.orientation.w = 1.0;

	// Set the scale of the marker -- 1x1x1 here means 1m on a side
	marker.scale.x = .10;
	marker.scale.y = .10;
	marker.scale.z = .10;

	// Set the color -- be sure to set alpha to something non-zero!
	marker.color.r = 1.0f;
	marker.color.g = 0.0f;
	marker.color.b = 0.1f;
	marker.color.a = 1.0;

	marker.lifetime = ros::Duration();

	// Publish the marker
	//marker_pub.publish(marker);
	markerArray.markers.push_back(marker);

	/****** rock placer Rviz****************

	for(int y=3;y<=out.size()-2; y+=2){

		visualization_msgs::Marker marker1;
		// Set the frame ID and timestamp.  See the TF tutorials for information on these.
		marker1.header.frame_id = "/map";
		marker1.header.stamp = ros::Time::now();

		// Set the namespace and id for this marker.  This serves to create a unique ID
		// Any marker sent with the same namespace and id will overwrite the old one
		marker1.ns = "basic_shapes";
		marker1.id = 0;

		// Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
		marker1.type = shape;

		// Set the marker action.  Options are ADD and DELETE
		marker1.action = visualization_msgs::Marker::ADD;
	

		marker1.pose.position.x = out(y);
		marker1.pose.position.y = out(y+1);
		marker1.pose.position.z = 0;
		marker1.pose.orientation.x = 0.0;
		marker1.pose.orientation.y = 0.0;
		marker1.pose.orientation.z = 0.0;
		marker1.pose.orientation.w = 1.0;


		marker1.color.r = 1.0f;
		marker1.color.g = 0.0f;
		marker1.color.b = 0.0f;
		marker1.color.a = 1.0;

		// Set the scale of the marker -- 1x1x1 here means 1m on a side
		marker1.scale.x = .10;
		marker1.scale.y = .10;
		marker1.scale.z = .10;
		marker1.lifetime = ros::Duration();

		markerArray.markers.push_back(marker1);

	}*/
	marker_array_pub.publish(markerArray);
	/**********************************************************/

}

//-----------------------------MAIN------------------------------
int main(int argc, char **argv)
{
	// Set up ROS.
	ros::init(argc, argv, "filter_new");
	ros::NodeHandle n;

	// Create a new NodeExample object.
	rosTop *rt = new rosTop(&n);

	//until we use only img callback, below is needless
	// Tell ROS how fast to run this node.
	ros::Rate r(rt->rate_);

	// Main loop.

	while (n.ok())
	{
	//rt->process();
	ros::spinOnce();
	r.sleep();

	}

	//ros::spin();

	return 0;
} // end main()
