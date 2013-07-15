#include <math.h>

class features3D
{

private:
	int m_id;
	float m_x,
	      m_y,
  	      m_z,
	      m_range,
	      m_bearing;
	bool m_exists;

	void polarToCartesian();
	void cartesianToPolar();
public:
	features3D(int id, float x, float y, float z, bool exists);
	features3D(int id, float range, float bearing, bool exists);
	~features3D();

	void setPoint(float x, float y, float z);
	void setPoint(float range, float bearing);
	void setExists(bool exists);
	int getIndex();
	float getX();
	float getY();
	float getZ();
	float getBearing();
	float getRange();
	bool getExists();
};
