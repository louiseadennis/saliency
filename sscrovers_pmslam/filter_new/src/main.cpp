#include <iostream>
#include <fstream>
#include <sstream>
#include <iterator>
#include <armadillo>
#include "filter_new/EKFFilter.h"

using namespace std;
using namespace arma;

vector<vec> getControlData(string filePath);
vector<features3D> getFeatures(int step, string path);

int main(int argc, char** argv)
{
	EKFFilter testFilter;
	vector<vec> controlVectors = getControlData("input/controldata.csv");
	cout << "Controls loaded: " << controlVectors.size() << endl;
	ofstream outputFile("output.csv");
	string path = "input/features_";
	if(outputFile.is_open())
	{
		for(int i=0; i < (int)controlVectors.size(); i++)
		{
			cout << "STEP " << i << endl;
			vector<features3D> features = getFeatures(i, path);
			cout << "Features loaded: " << features.size() << endl;
	    	testFilter.predict(controlVectors[i]);
	    	testFilter.update(features);
	    	testFilter.augment(features);
	    	outputFile << testFilter.getRoverState().t();
		    cout << testFilter.getRoverState();
		}
		outputFile.close();
	}
	else
		cout << "Could not output to file" << endl;
	return 0;
}


vector<vec> getControlData(string filePath)
{
    ifstream controlFile(filePath.c_str());
    string controlLine;
    vector<vec> controlVectors;
    if(controlFile.is_open())
    {
        while (controlFile.good())
        {
            getline (controlFile, controlLine);
            vec controlVector;
            vector<string> tokens;
            istringstream iss(controlLine);
            copy(istream_iterator<string>(iss),
                istream_iterator<string>(),
                back_inserter<vector<string> >(tokens));
            controlVector << atof(tokens[0].c_str()) << endr << atof(tokens[1].c_str());
            controlVectors.push_back(controlVector);

        }
        controlFile.close();
    }
    else
        cout << "Could not open control file" << endl;

    return controlVectors;
}

vector<features3D> getFeatures(int step, string path)
{
	stringstream fileName;
	fileName << path << step << ".csv";
	ifstream featureFile(fileName.str().c_str());
	vector<features3D> features;
	string featureLine;
	if(featureFile.is_open())
	{
		while(featureFile.good())
		{
			getline (featureFile, featureLine);
			vector<string> tokens;
			stringstream lineStream;
			lineStream << featureLine;
			while(!lineStream.eof())
			{
				string tokenString;
				getline(lineStream, tokenString, ',');
				tokens.push_back(tokenString);
			}
			if(tokens.size() == 10)
				features.push_back(features3D(atoi(tokens[0].c_str()), atoi(tokens[5].c_str()), atoi(tokens[6].c_str()), atoi(tokens[7].c_str()), ((atoi(tokens[1].c_str()) == 1) ? false : true)) );
		}
	}
	else
		cout << "Could not open features file " << step << endl;
	return features;
}
