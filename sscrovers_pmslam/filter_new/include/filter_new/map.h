#include <vector>
#include "filter_new/mapFeature.h"

using namespace std;
namespace PMSLAM{

	class map{

	private:
		vector<mapFeature> features;

	public:
		map();
		~map();

		void add(mapFeature feature);
		void add(double p_x, double p_y, double p_z);
		void update(int index, mapFeature feature);
		void update(int index, double p_x, double p_y, double p_z);
	
		mapFeature getFeature(int index);
		vector<vector<double> > getMap();
	};
}
