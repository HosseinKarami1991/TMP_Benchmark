/*
 * Objects.hpp
 *
 *  Created on: Mar 9, 2017
 *      Author: Kourosh Darvish
 */
#ifndef WORLD_H
#define WORLD_H

#include <stdlib.h>
#include <iostream>
#include <stdio.h>
#include <fstream>
#include <vector>
#include <string>
#include <math.h>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>


using namespace std;

class world{
public:
	vector<string> name;
	vector<float> value;
	world(void){}
	world(vector<string> Name, vector<float> Value){
		name=Name;
		value=Value;
	}

	~world(){}
	void print(){


		cout<<"name: ";
		for(int i=0;i<name.size();i++)
			cout<<name[i]<<" ";
		cout<<endl;
		cout<<"value: ";

		for(int i=0;i<value.size();i++)
			cout<<value[i]<<" ";
		cout<<endl;
};
};

#endif
