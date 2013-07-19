class mapFeature{

private:
	double m_x, m_y, m_z;
	bool m_empty;

public:
	mapFeature();
	mapFeature(double x, double y, double z);
	~mapFeature();

	//Accessors
	double getX(){ return m_x;}
	double getY(){ return m_y;}
	double getZ(){ return m_z;}
	bool isEmpty(){ return m_empty; }
	void setX(double p_x){ m_x=p_x;}
	void setY(double p_y){ m_y=p_y;}
	void setZ(double p_z){ m_z=p_z;}
};
