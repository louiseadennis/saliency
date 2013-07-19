#include "filter_new/map.h"
	
using namespace std;
using namespace PMSLAM;

map::map() {}
map::~map() {}

void map::add(mapFeature feature)
{
	features.push_back(feature);
}

void map::add(double p_x, double p_y, double p_z)
{
	features.push_back(mapFeature(p_x, p_y, p_z));
}

void map::update(int index, mapFeature feature)
{
	if(index<features.size())
		features[index] = feature;
}

void map::update(int index, double p_x, double p_y, double p_z)
{
	if(index<features.size())
	{
		features[index].setX(p_x);
		features[index].setY(p_y);
		features[index].setZ(p_z);
	}
}

mapFeature map::getFeature(int index)
{
	if(index < features.size())
		return features[index];
	return mapFeature();
}
vector<vector<double> > map::getMap()
{
	vector<vector<double> > returnVector;
	for(int i=0;i<features.size(); i++)
	{
		vector<double> feature;
		feature.push_back(features[i].getX());
		feature.push_back(features[i].getY());
		feature.push_back(features[i].getZ());
		returnVector.push_back(feature);
	}
	return returnVector;
}
