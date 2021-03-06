#include "filter_new/EKFFilter.h"
#include "ros/ros.h"

EKFFilter::EKFFilter()
{
	m_state = zeros<vec>(3);
	m_covariance = eye<mat>(3,3) * 0.001;
	m_controlNoise << 0.00000001 << 0 << 0 << endr << 0 << 0.00000001 << 0 << endr << 0 << 0 << 0.000000000003;
	m_measurementNoise << 0.0025 << 0 << endr << 0 << 0.0012;
}

EKFFilter::~EKFFilter()
{
}

vector<vector<double> > getMap()
{
}

void EKFFilter::predict(vec controlData)
{
	int stateLength = m_state.n_rows;
	mat A = eye<mat>(3,3);
	A(0,2) = -controlData(0)*sin(m_state(2));
	A(1,2) = controlData(0)*cos(m_state(2));
	mat predictedCovariance = m_covariance;
	predictedCovariance(span(0,2), span(0,2)) = A * m_covariance(span(0,2), span(0,2))  * A.t() + m_controlNoise;
	if(stateLength > 3)
	{
		predictedCovariance(span(0,2), span(3,stateLength-1)) = A*m_covariance(span(0,2), span(3,stateLength-1));
		predictedCovariance(span(3,stateLength-1), span(0,2)) = predictedCovariance(span(0,2), span(3,stateLength-1)).t();
	}
	m_covariance = predictedCovariance;
	m_state(0) = m_state(0) + controlData(0)*cos(m_state(2));
	m_state(1) = m_state(1) + controlData(0)*sin(m_state(2));
	m_state(2) = m_state(2) + controlData(1);
}

void EKFFilter::updateHeading(double heading, double headingNoise)
{
}

void EKFFilter::update(vector<features3D> features)
{

    int featureCount = 0;
	for(int i=0; i<(int)features.size(); i++)
		if(features[i].getExists()) featureCount++;	
    mat H = zeros<mat>(m_state.n_rows, m_state.n_rows);
    vec difference = zeros<vec>(m_state.n_rows);
    mat measurementCovariance = zeros<mat>(m_state.n_rows, m_state.n_rows);

    for(int i=0; i<(int)features.size(); i++)
    {
        if(features[i].getExists())
        {
            int featureIndex = 2*features[i].getIndex() - 2;
            vec predictedFeatures;
            H(span(featureIndex, featureIndex+1), span::all) = observeModel(featureIndex, &predictedFeatures);
            difference(featureIndex) = features[i].getRange() - predictedFeatures(0);
            difference(featureIndex+1) = features[i].getBearing() - predictedFeatures(1);
            measurementCovariance(span(featureIndex, featureIndex+1), span(featureIndex, featureIndex+1)) = m_measurementNoise;
        }
    }
    choleskyUpdate(difference, measurementCovariance, H);
}

void EKFFilter::augment(vector<features3D> features)
{
    for(int i=0; i<(int)features.size(); i++)
    {
        if(!features[i].getExists())
        {
            int stateSize = m_state.n_rows;
            float range = features[i].getRange();
            float bearing = features[i].getBearing();
            float s = sin(m_state(2) - bearing);
            float c = cos(m_state(2) - bearing);
            m_state.resize(stateSize+2);
            m_state(stateSize) = m_state(0) + range*c;
            m_state(stateSize+1) = m_state(1) + range*s;
	    featureMap.add(m_state(stateSize), m_state(stateSize+1), 0);
            mat vehicleJacobian;
            vehicleJacobian << 1 << 0 << -range*s << endr
                            << 0 << 1 << range*c << endr;
            mat measurementJacobian;
            measurementJacobian << c << -range*s << endr
                                << s << range*c << endr;
            m_covariance.resize(stateSize+2, stateSize+2);
            m_covariance(span(stateSize, stateSize+1), span(stateSize, stateSize+1)) = vehicleJacobian*m_covariance(span(0,2), span(0,2))*vehicleJacobian.t() + measurementJacobian*m_measurementNoise*measurementJacobian.t();
            m_covariance(span(stateSize, stateSize+1), span(0,2)) = vehicleJacobian*m_covariance(span(0,2), span(0,2));
            m_covariance(span(0,2), span(stateSize, stateSize+1)) = m_covariance(span(stateSize, stateSize+1), span(0,2)).t();
            if(stateSize>3)
            {
                m_covariance(span(stateSize, stateSize+1), span(3,stateSize-1)) = vehicleJacobian*m_covariance(span(0,2), span(3,stateSize-1));
                m_covariance(span(3,stateSize-1), span(stateSize, stateSize+1)) = m_covariance(span(stateSize, stateSize+1), span(3,stateSize-1)).t();
            }
        }
    }
}

mat EKFFilter::observeModel(int featureIndex, mat* predictedFeature)
{


    int stateSize = m_state.n_rows;
    featureIndex = featureIndex+3; //Rover state size = 3
    mat measurementJacobian = zeros<mat>(2, stateSize);
    vec delta = m_state(span(featureIndex, featureIndex+1)) - m_state(span(0,1));
    float deltaSquared = pow(delta(0),2) + pow(delta(1),2);
    float range = sqrt(deltaSquared);
    vec deltaByRange = delta / range;
    vec deltaByDeltaSquared = delta / deltaSquared;
    *predictedFeature << range << endr << m_state(2) - atan2(delta(1), delta(0));
    mat jacobianTop;
    jacobianTop << -deltaByRange(0) << -deltaByRange(1) << 0 << endr << deltaByDeltaSquared(1) << -deltaByDeltaSquared(0) << 1;
    mat jacobianBottom ;
    jacobianBottom << deltaByRange(0) << deltaByRange(1) << endr << deltaByDeltaSquared(1) << -deltaByDeltaSquared(0);
    measurementJacobian(span::all, span(0,2)) = jacobianTop;
    measurementJacobian(span::all, span(featureIndex, featureIndex+1)) = jacobianBottom;
    return measurementJacobian;
}

void EKFFilter::choleskyUpdate(vec difference, mat measurementCovariance, mat measurementJacobian)
{
    int stateLength = m_state.n_rows;
    mat PHt = m_covariance(span(0, stateLength-1), span(0, stateLength-1))*measurementJacobian.t();
    mat S = measurementJacobian * PHt + measurementCovariance;
    S = (S + S.t())*0.5;
    mat SChol = chol(S);
    mat SCholInv = inv(SChol);
    mat W1 = PHt * SCholInv;
    mat W = W1 * SCholInv.t();
    m_state = m_state + W*difference;
    m_covariance = m_covariance - W1*W1.t();
}
