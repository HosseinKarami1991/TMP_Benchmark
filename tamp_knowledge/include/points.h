#include <stdlib.h>
#include <iostream>
#include <stdio.h>
#include <fstream>
#include <vector>
#include <string>
#include <math.h>
#include <boost/algorithm/string.hpp>
#include "world.h"



using namespace std;


void readPointsVector(string pointsPath, vector<world> &pointsVector){
	ifstream file_path_ifStr(pointsPath.c_str());
	string line;
	vector<string> line_list;

	string delim_type=" ";
	if (file_path_ifStr.is_open())
	{
		while(getline(file_path_ifStr,line))
		{
			boost::split(line_list, line, boost::is_any_of(delim_type));
			if(line_list[0]!="#")
			{
				vector<float> Pose;
				vector<string> Name;
				for(int i=1;i<line_list.size();i++)
				{
					Pose.push_back( stof(line_list[i]) );
				}

				Name.push_back(line_list[0]);
				Name.push_back("Pose");
				world temp_point(Name,Pose);
				pointsVector.push_back(temp_point);
			}
		}
	}

	for(int i=0;i<pointsVector.size();i++)
		pointsVector[i].print();
};

