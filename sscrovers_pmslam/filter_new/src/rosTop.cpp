#include "filter_new/rosTop.h"
#include <iostream>
#include <sstream>
#include <fstream>
#include <math.h>

using namespace std;
using namespace arma;

rosTop::rosTop(ros::NodeHandle *_n){
	ros::NodeHandle private_node_handle("~");
	private_node_handle.param("rate", rate_, int(10));

        ctrl_vec_sub_ = _n->subscribe("ctrl_vec", 1, &rosTop::control_vec_callBack, this);
	local_features_3D_sub_ = _n->subscribe("localFeat3D", 1, &rosTop::local_features_3D_callBack, this);

	checkVec = false;
	check3D = false;
}


rosTop::~rosTop(){}

void rosTop::process(){
ROS_INFO("%i  ==  %i",step_vec,step_feat);
	if(step_vec == step_feat){
	 	newFilter.update(features);
	    	newFilter.augment(features);
	}else{
		ROS_ERROR("Crash, mismatch");
	}

	publish();
}

void rosTop::control_vec_callBack(const sscrovers_pmslam_common::control_vec& msg){
	ROS_INFO("Recieved control vector - %i", msg.header.stamp.nsec);
	step_vec  = msg.header.stamp.nsec; 
	current_ctrlVec = vec(msg.d, msg.theta);
	ROS_INFO("control prediction");
	newFilter.predict(current_ctrlVec);
	checkVec=true;
}

void rosTop::local_features_3D_callBack(const sscrovers_pmslam_common::featureUpdate3DArray& msg){
	ROS_INFO("Recieved features update - %i", msg.header.stamp.nsec);
	step_feat  = msg.header.stamp.nsec; 
	features.clear();
	for(int i=0; i<msg.features.size(); i++){
		features.push_back(features3D(msg.features[i].id,msg.features[i].x,msg.features[i].y,msg.features[i].z,msg.features[i].exists));
	}
}


void rosTop::publish(){
	cout << newFilter.getRoverState();
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
ROS_INFO("waiting");
	rt->process();
	ros::spinOnce();
	r.sleep();

	}

	//ros::spin();

	return 0;
} // end main()
