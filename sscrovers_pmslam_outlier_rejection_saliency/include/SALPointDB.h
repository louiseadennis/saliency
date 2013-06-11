#ifndef SALPOINTDB_H
#define SALPOINTDB_H

#include <vector>
#include "RoverState.h"
#include "SALPoint.h"
#include "SURFPoint.h"
#include "ptpairs.h"
#include "SALPointVec.h"
#include<ctime>




class SALPointDB{

	public:
		
		std::vector <SALPointVec> SALdata;
		std::vector <ptpairs> ptpars;
		std::vector <ptpairs> unadjusted;
		std::vector <int> startPoint;
		int last;
		
		SALPointDB(){
			upto=0;
			last =0;
		};	
		
		void push_back(SALPointVec in){
			if(SALdata.size()<1){
				startPoint.push_back(0);
			}else{
				int y = startPoint[startPoint.size()-1]+SALdata[SALdata.size()-1].size();
				startPoint.push_back(1+y);
			}
			SALdata.push_back(in);
			
		};
		
		void new_vec(RoverState stateIn, std::time_t tIn){
			SALPointVec latest(stateIn, tIn);
			SALdata.push_back(latest);
		};

		void add_point(SALPoint sa){
			SALdata.at(SALdata.size()-1).push_back(sa);
		};

		int size(){
			return SALdata.size();
		}

		//TODO Eventually conversion stuff HERE
		
		int upto;
		std::vector <SURFPoint> converted;
		std::vector <int> ptpairConverted;

		std::vector <SURFPoint> convert(){
			if(SALdata.size()>upto){
				for(int i=0; i<SALdata.size()-upto;i++){
				//convert
					SALPointVec t = SALdata[SALdata.size()-1-i];
					ptpairs p = ptpars[SALdata.size()-1-i];
					ROS_INFO("In here");
					for(int j=0; j<t.size(); j++){
						//Convert ptpairs
						if(p.pairs[j].frame==0){
							ptpairConverted.push_back(last);
							last++;
						}else{
							ptpairConverted.push_back(ptpairConverted[startPoint[p.pairs[j].frame]+p.pairs[j].j]);	
						}
						//Convert DB
						SURFPoint sp;
						sp.pt.pt.x = t.atX(j);
						sp.pt.pt.y = t.atY(j);
						sp.state = t.state;
						sp.pt.size = t.atH(j) * t.atW(j);
						sp.index = converted.size();
						converted.push_back(sp);
					}
				}
				upto = SALdata.size();
			}
			return converted;
		}
		

};

#endif
