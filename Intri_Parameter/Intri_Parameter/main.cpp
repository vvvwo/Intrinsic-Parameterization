/*********************************************************************************

					 Main for Point Cloud Denoising

						Updating in 2022/06/30

						   By Dr. Chenlei Lv

			The functions includes:
			1. main function for pre-processing of intrinsic
			clearn net (ICN) 

*********************************************************************************/


#pragma once

#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include "tri_geodesic.hpp"
#include "intri_para.hpp"

using namespace std;

int main(int argc, char* argv[]) {

	//nose index: pjanic(18504, 4858, 1.2); face(736, 242, 10); pogba(2245, 48, 1.3); skarllet(7332,105.0)
	char* fileName = NULL;
	int nosalPoint = -1;
	int eyemiddle = -1;
	float radius_searching = -1;

	if (argc == 5) {
		fileName = argv[1];
		nosalPoint = (int)atoi(argv[2]);
		eyemiddle = (int)atoi(argv[3]);
		radius_searching = (float)atof(argv[4]);
	}
	else if (argc == 4) {
		fileName = argv[1];
		nosalPoint = (int)atoi(argv[2]);
		radius_searching = (float)atof(argv[3]);
	}
	else {
		cout << "error! the parameters are incorrect!" << endl;
	}

	string fileNameStr(fileName);
	int findIndex = fileNameStr.find_last_of(".");
	string fileName_2 = fileNameStr.substr(0, findIndex)+"_cut.ply";	
	vector<Point3f> point;
	

	if (1) {		
		
		cout << "start facial cropping." << endl;
		int t0 = clock();

		TriGeodesic_Mymesh tm;
		tm.TriGeodesic_Mymesh_init(fileName, nosalPoint, eyemiddle);
		tm.TriGeodesic_Mymesh_Extract(radius_searching);
		tm.TriGeodesic_Mymesh_Save_PLY(fileName_2);

		vector<vector<int>> face_Temp = tm.face_Temp;
		vector<Point3f> point_Temp = tm.point_Temp;
		vector<int> point_Border_Index = tm.point_Border_Index;
		int point_center =tm.point_center;
		
		int t1 = clock();
		cout << "Finished facial cropping."<< endl;	
		cout << "Computation Time:" << float(t1 - t0) / CLOCKS_PER_SEC << "s" << endl;	

		cout << "start intri-parameterization." << endl;
		t0 = clock();

		IntriniscParameterization ip;
		ip.IP_init(point_Temp, face_Temp, point_Border_Index, point_center);
		ip.IP_start_fixedBorder();

		t1 = clock();
		cout << "finished intri-parameterization." << endl;
		cout << "Computation Time:" << float(t1 - t0) / CLOCKS_PER_SEC << "s" << endl;

	}

	cout << "Finished."<<endl;	

}

