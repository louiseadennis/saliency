#include "or_core.h"

#include <iostream>
#include <sstream>
#include <fstream>


using namespace std;


ORCore::ORCore(ros::NodeHandle *_n)
{
  step_ = 1;
  curr_pose_ptr_ = new RoverState;
  filter_keypoints_ptr_ = new FilterKeypointsOR(&step_, curr_pose_ptr_, &db_);

  NVec = SALPointVec();
  NMinusOneVec = SALPointVec();
  step=0;


  // Initialise node parameters from launch file or command line.
  // Use a private node handle so that multiple instances of the node can be run simultaneously
  // while using different parameters.
  ros::NodeHandle private_node_handle("~");
  private_node_handle.param("rate", rate_, int(10));
  // topic names
  private_node_handle.param("output_features_topic_name", sub_features_topic_name_, string("/saliency/features_Hou"));
  private_node_handle.param("sub_trajectory_topic_name", sub_trajectory_topic_name_, string("est_traj"));
  private_node_handle.param("pub_ptpairs_topic_name", pub_ptpairs_topic_name_, string("ptpairs"));
  //camera params - TODO from camera info?, camera image? param server?
  private_node_handle.param("px", filter_keypoints_ptr_->px_, int(320));
  private_node_handle.param("py", filter_keypoints_ptr_->py_, int(240));
  //pmslam params
  private_node_handle.param("lm_track", filter_keypoints_ptr_->lm_track_, int(94));
  private_node_handle.param("scale", filter_keypoints_ptr_->scale_, int(1));
  // debug flags
  private_node_handle.param("features_to_file", features_to_file_f_, bool(false));
  private_node_handle.param("ptpoints_to_file", ptpoints_to_file_f_, bool(false));
  private_node_handle.param("db_to_file", db_to_file_f_, bool(false));
  private_node_handle.param("traj_to_file", traj_to_file_f_, bool(false));



  //subscribers
  features_sub_ = _n->subscribe(sub_features_topic_name_.c_str(), 1, &ORCore::featuresCallback, this);
  trajectory_sub_ = _n->subscribe(sub_trajectory_topic_name_.c_str(), 1, &ORCore::trajectoryCallback, this);

  //publishers
  ptpairs_pub_ = _n->advertise<sscrovers_pmslam_common::PtPairs>(pub_ptpairs_topic_name_.c_str(), 1);
  db_pub_ = _n->advertise<sscrovers_pmslam_common::DynamicArray>("temp_db", 1);

  data_completed_f_ = false;
}

ORCore::~ORCore()
{

}

void ORCore::process()
{
  //put here everything that should run with node loop
  //if get complet data and step is updated
  if ((data_completed_f_) && (step_ > 0))
  {
    data_completed_f_ = false;
    //filter_keypoints_ptr_->filterKeypoints();
    filter();
    publishPtPairs();
    //publishDB(); //send data to DB
  }
}

void ORCore::sendToSurfDataBase()
{
  //publishing all db
}

void ORCore::publishPtPairs()
{
  // push forward stamp
  ptpairs_msg_.header.stamp = stamp_;
  // size of data to publish
  int size = pp.size();
  //reserve space for data;
  ptpairs_msg_.pairs.resize(size);
  // copy data to message structure
  memcpy(ptpairs_msg_.pairs.data(), pp.data(), size * sizeof(int)); 
  // publish msg
  ptpairs_pub_.publish(ptpairs_msg_);

}

void ORCore::featuresCallback(const geometry_msgs::PoseArray& msg)
{
	saliency_poses_vec.clear();
	SALPoint spTemp;
	NVec = SALPointVec(*curr_pose_ptr_, std::time(0));
	for(int i=0; i< msg.poses.size();i++){
		saliency_poses p;
		p.centroid_x = msg.poses[i].position.x;
		p.centroid_y = msg.poses[i].position.y;
		p.width = msg.poses[i].orientation.w;
		p.height = msg.poses[i].orientation.z;
		spTemp = SALPoint(msg.poses[i].position.x,msg.poses[i].position.y,msg.poses[i].orientation.w, msg.poses[i].orientation.z );
		NVec.push_back(spTemp);
		saliency_poses_vec.push_back(p);
	}

	process( );
	//filter();

}
 vector<int> ORCore::bubblesort(vector<int> w, vector<int> w2) 
{ 
	ROS_INFO("Bubblesort begin");
	int temp, temp2;
	bool finished = false;
	 while (!finished) 
	{ 
		finished = true;
		for (int i = 0; i < w.size()-1; i++) { 
			if (w[i] > w[i+1]) { 
				temp = w[i];
				temp2 = w2[i];
				 w[i] = w[i+1];
				 w2[i] = w2[i+1];
				 w[i+1] = temp;
				 w2[i+1] = temp2;
				 finished=false; 
			} 
		} 
	} 
	ROS_INFO("Bubblesort end");
	return w2; 
}

ptpairs ORCore::nearestNeighbour(){
	ROS_INFO("NN begin N = %i, N-1 = %i",NVec.size(),NMinusOneVec.size());
	int closest;
	ptpairs ptpairs_local;
	float dmin;	
	float d, area, areaMO;
	int framee;
	for(int i = 0; i<NVec.size(); i++){
		dmin=10000;
		closest=i;
		framee = 0;
		for(int j=0; j<NMinusOneVec.size(); j++){
			d = NMinusOneVec.at(j).dist(NVec.at(i));
			area = NVec.at(i).height * NVec.at(i).width;
			areaMO = NMinusOneVec.at(j).width * NMinusOneVec.at(j).height;
			if(d<dmin && area < 4*areaMO && 4 * area > areaMO){
				dmin=d;
				closest = j;
				framee = -1;
			}
		}
		
		ptpairs_local.add(closest,dmin,framee);
	}
	ROS_INFO("NN End");
	return ptpairs_local;
}


ptpairs ORCore::nearestNeighbour2(SALPointVec N, SALPointVec NM2, ptpairs in){
//TODO fix only points to unused points from n-1 frame
	ROS_INFO("NN2 begin");
	int closest;
	ptpairs ptpairs_local;
	float dmin;	
	float d, area, areaMO;
	for(int i = 0; i<N.size(); i++){
		dmin=10000;
		closest=i;
		if(in.atFrame(i)==0){
			for(int j=0; j<NM2.size(); j++){
				d = NM2.at(j).dist(N.at(i));
				area = N.at(i).height * N.at(i).width;
				areaMO = NM2.at(j).width * NM2.at(j).height;
				if(d<dmin && area < 4*areaMO && 4 * area > areaMO ){
					dmin=d;
					closest = j;
				}
			}
			ptpairs_local.add(closest,dmin,-2);
		}else{
			dmin = in.atD(i);
			closest = in.atJ(i);
			ptpairs_local.add(closest,dmin,in.atFrame(i));
		}

	}
	ROS_INFO("NN2 End");
	return ptpairs_local;
}

ptpairs ORCore::outlierRejection(ptpairs inP){
	vector<cv::Point2f> points1, points2;
	cv::Point2f pTemp;
	ROS_INFO("Outlier Rejection begin");
/*
	for(int r=0;r<inP.size();r++){
		ROS_INFO("Before i=%i, j=%i, frame=%i",r, inP.atJ(r), inP.atFrame(r));
	}
*/
	std::vector <int> lis;

	for(int i = 0; i<inP.size(); i++){
	ROS_INFO("%i = %i", inP.atJ(i), NMinusOneVec.size() );
		if(inP.pairs[i].frame == -1){
			pTemp.x = NMinusOneVec.atX(inP.atJ(i));
			pTemp.y = NMinusOneVec.atY(inP.atJ(i));
			points1.push_back(pTemp);
			pTemp.x = NVec.atX(i);
			pTemp.y = NVec.atY(i);
			points2.push_back(pTemp);
			lis.push_back(i);
		}
	}
	ROS_INFO("Outlier Rejection middle");
	vector<uchar> outliers;
	vector<cv::Point2f> points1_temp, points2_temp;
	Mat F = findFundamentalMat_local(Mat(points1), Mat(points2), FM_RANSAC, 5, 0.80, &outliers);
	for(int u=0; u<outliers.size(); u++){
		if (outliers[u] == 1){

			inP.pairs[lis[u]].j = lis[u];
			inP.pairs[lis[u]].d = 0;
			inP.pairs[lis[u]].frame = 0;
		}	
	}
/*
	for(int r=0;r<inP.size();r++){
		ROS_INFO("After i=%i, j=%i, frame=%i",r, inP.atJ(r), inP.atFrame(r));
	}
*/	
	ROS_INFO("Outlier Rejection end");
	return inP;
}



void ORCore::filter()
{

	if(SDB.size() < 1){
		//fill database
		//database = saliency_poses_vec;
		//database_all = saliency_poses_vec;
		NMinusOneVec = NVec;
		SDB.push_back(NVec);
		ptpair py;
		ptpairs pyVec;
		for(int i=0; i<NVec.size();i++){
			py.i = i;
			py.j = i;
			py.frame =0;
			py.d = 0;
			pyVec.push_back(py);
		}
		SDB.ptpars.push_back(pyVec);
		SDB.unadjusted.push_back(pyVec);

		for(int p=0; p<NVec.size();p++){
			vs.push_back(SALPoint(NVec.atX(p), NVec.atY(p), NVec.atW(p), NVec.atH(p), 1,0,step,0));
			ppm1.push_back(p);
		}
		
	}else{
		//nearest neighbour

		ptpairs outP = nearestNeighbour();

		//Outlier Rejection 
		
		outP = outlierRejection(outP);

		//k-nn tracking all frames
		if(SDB.size()>1){
			SALPointVec NMinusTwoVec = SDB.SALdata[SDB.size()-2];
			outP = nearestNeighbour2(NVec,NMinusTwoVec,outP);
		}

		//move database


		NMinusOneVec = NVec;
		ptpairs ptVec;

		SDB.push_back(NVec);
		for(int uu=0;uu<NVec.size();uu++ ){
			ptpair ppp;
			ppp.i = uu;
			pairInts pi = resolve(outP.at(uu));
			ppp.j = pi.j;
			ppp.d = outP.atD(uu); 
			ppp.frame = pi.frame;
			ROS_INFO("i=%i, j=%i, frame=%i",uu ,ppp.j,ppp.frame);
			ptVec.push_back(ppp);	
		}

		

		SDB.ptpars.push_back(ptVec);
		
		int ppUp=0;
		int start;
		for(int e=0;e<NVec.size();e++ ){
			ptpair plast = outP.at(e);
			if(plast.frame == -1){
			//n-1 frame
				int object = ppm1[plast.j];
				pp.push_back(object);
				vs[object].flag = 1;
				vs[object].step = step;
				vs[object].n += 1;
				vs[object].x = NVec.atX(e);
				vs[object].y = NVec.atY(e);
				vs[object].height = NVec.atH(e);
				vs[object].width = NVec.atW(e);
					
			}else if(plast.frame == -2){
			//n-2 frame
				int object = ppm1[plast.j];
				pp.push_back(object);
				vs[object].flag = 1;
				vs[object].step = step;
				vs[object].n += 1;
				vs[object].x = NVec.atX(e);
				vs[object].y = NVec.atY(e);
				vs[object].height = NVec.atH(e);
				vs[object].width = NVec.atW(e);

			}else{
			//new object
				pp.push_back(vs.size());
				vs.push_back(SALPoint(NVec.atX(e), NVec.atY(e), NVec.atW(e), NVec.atH(e), 1,0,step,0));
			}
		}
		//push  to publish
		//ROS_INFO("Conversion Started");
		//std::vector <SURFPoint> outDB = SDB.convert();
		//std::vector <int> ptps = SDB.ptpairConverted;
		
	}
	ppm2 = ppm1;
	ppm1 = pp;
	step++;	
}

pairInts ORCore::resolve(ptpair p){
	pairInts pI;
	if (p.frame != 0){
		ptpair match = SDB.ptpars.at(SDB.size()+p.frame-1).at(p.j);
		
		pI.frame = match.frame;
		pI.j = match.j;
			
	}else{
		pI.j = p.j;
		pI.frame = SDB.size()-1;
	}
		
	return pI;
}

cv::Mat ORCore::findFundamentalMat_local(const Mat& points1, const Mat& points2, int method, double param1, double param2, vector<
    uchar>* mask)
{
  CV_Assert(points1.checkVector(2) >= 0 && points2.checkVector(2) >= 0 &&
      (points1.depth() == CV_32F || points1.depth() == CV_32S) &&
      points1.depth() == points2.depth());  
  Mat F(3, 3, CV_64F);
  CvMat _pt1 = Mat(points1), _pt2 = Mat(points2);
  CvMat matF = F, _mask, *pmask = 0;
  if( mask )
  {
    mask->resize(points1.cols*points1.rows*points1.channels()/2);
    pmask = &(_mask = cvMat(1, (int)mask->size(), CV_8U, (void*)&(*mask)[0]));
  }
  int n = cvFindFundamentalMat( &_pt1, &_pt2, &matF, method, param1, param2, pmask );


  if( n <= 0 )
  F = Scalar(0);
  return F;
}


void ORCore::trajectoryCallback(const nav_msgs::PathConstPtr& msg)
{

  est_traj_msg_ = *msg;
  if (step_ >= 0)
  {
    if ((int)est_traj_msg_.poses.size() > step_)
    {
      curr_pose_ptr_->x = est_traj_msg_.poses[step_].pose.position.x;
      curr_pose_ptr_->y = est_traj_msg_.poses[step_].pose.position.y;
      curr_pose_ptr_->z = est_traj_msg_.poses[step_].pose.position.z;

      double _roll, _pitch, _yaw;
#if ROS_VERSION_MINIMUM(1, 8, 0) // if current ros version is >= 1.8.0 (fuerte)
      //min. fuerte
      tf::Quaternion _q;
      tf::quaternionMsgToTF(est_traj_msg_.poses[step_].pose.orientation, _q);
      tf::Matrix3x3(_q).getRPY(_roll, _pitch, _yaw);
#else
      //electric and older
      btQuaternion _q;
      tf::quaternionMsgToTF(est_traj_msg_.poses[step_].pose.orientation, _q);
      btMatrix3x3(_q).getRPY(_roll, _pitch, _yaw);
#endif

      curr_pose_ptr_->roll = _roll;
      curr_pose_ptr_->pitch = _pitch;
      curr_pose_ptr_->yaw = _yaw;

      data_completed_f_ = true;

    }
    else
    {
      ROS_WARN("There is no trajectory point corresponding to features data...");
      data_completed_f_ = false;
    }
  }

  
}

void ORCore::publishDB()
{
  //ROS_STATIC_ASSERT(sizeof(MyVector3) == 24);

  sscrovers_pmslam_common::SALVector sal_db;

  sal_db.header.stamp.nsec = step;

  sal_db.data.resize(sizeof(sscrovers_pmslam_common::SPoint) * vs.size());
  for(int i=0; i<vs.size();i++){
	sal_db.data[i].x = vs[i].x;
	sal_db.data[i].y = vs[i].y;
	sal_db.data[i].width = vs[i].width;
	sal_db.data[i].height = vs[i].height;
	sal_db.data[i].n = vs[i].n;
	sal_db.data[i].id = vs[i].id;
	sal_db.data[i].step = vs[i].step;
	sal_db.data[i].flag = vs[i].flag;
  }

  db_pub_.publish(sal_db);
}

//-----------------------------MAIN-------------------------------

int main(int argc, char **argv)
{
  // Set up ROS.
  ros::init(argc, argv, "outlier_rejection");
  ros::NodeHandle n;

  // Create a new NodeExample object.
  ORCore *or_core = new ORCore(&n); // __atribute__... only for eliminate unused variable warning :)

  //until we use only img callback, below is needless
  // Tell ROS how fast to run this node.
  ros::Rate r(or_core->rate_);

  // Main loop.

  while (n.ok())
  {
    //or_core->Process();
    //fd_core->Publish();
    ros::spinOnce();
    r.sleep();
  }

  return 0;
} // end main()
