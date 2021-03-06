#include <armadillo>
#include <vector>
#include "features3D.h"
#include "map.h"

using namespace arma;
using namespace std;
using namespace PMSLAM;

class EKFFilter //:SLAMFilter
{
	private:
		vec m_state;
		mat m_covariance;
		mat m_controlNoise;
		mat m_measurementNoise;
		PMSLAM::map featureMap;


        void choleskyUpdate(vec difference, mat measurementCovariance, mat measurementJacobian);

	public:
		EKFFilter();
		~EKFFilter();
		mat observeModel(int featureIndex, mat* predictedFeature);
		vec getState() { return m_state; }
		vec getRoverState() { return m_state(span(0,2)); }
//		vector<vector<double> > getMap();

		void predict(vec controlData);
		void updateHeading(double heading, double headingNoise);
		void update(vector<features3D> features);
		void augment(vector<features3D> features);
};
