#include "filter_new/features3D.h"

//	Constructors/Destructor

features3D::features3D(int id, float x, float y, float z, bool exists)
{
	m_id = id;
	m_x = x;
	m_y = y;
	m_z = z;
	m_exists = exists;
	cartesianToPolar();
}

features3D::features3D(int id, float range, float bearing, bool exists)
{
	m_id = id;
	m_range = range;
	m_bearing = bearing;
	m_exists = exists;
	polarToCartesian();
}

features3D::~features3D()
{
}

//	Accessors

void features3D::setPoint(float x, float y, float z)
{
	m_x = x;
	m_y = y;
	m_z = z;
	cartesianToPolar();
}

void features3D::setPoint(float range, float bearing)
{
	m_range = range;
	m_bearing = bearing;
	polarToCartesian();
}

void features3D::setExists(bool exists)	{ m_exists = exists; }

int   features3D::getIndex()    { return m_id; }
float features3D::getX()	    { return m_x; }
float features3D::getY()	    { return m_y; }
float features3D::getZ()	    { return m_z; }
float features3D::getBearing()	{ return m_bearing; }
float features3D::getRange()	{ return m_range; }
bool  features3D::getExists()	{ return m_exists; }

//	Private Members

void features3D::polarToCartesian()
{
	m_x = m_range*sin(m_bearing);
	m_y = m_range*cos(m_bearing);
	m_z = 0;
}

void features3D::cartesianToPolar()
{
	m_range = sqrt(pow(m_x, 2) + pow(m_y, 2));
	m_bearing = atan2(m_x, m_y);
}
