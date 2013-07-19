#include "filter_new/mapFeature.h"

mapFeature::mapFeature()
{
	m_x = 0;	
	m_y = 0;	
	m_z = 0;	
	m_empty = false;
}

mapFeature::mapFeature(double p_x, double p_y, double p_z)
{ 
	m_x = p_x;
	m_y = p_y;
	m_z = p_z;
	m_empty = true;
}
	
mapFeature::~mapFeature(){}
