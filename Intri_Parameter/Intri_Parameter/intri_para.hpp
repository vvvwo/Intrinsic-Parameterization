/*********************************************************************************

					   Intrinisc Parameterization

						 Updating in 2022/08/30

						   By Dr. Chenlei Lv

			The functions includes:
			1. Input mesh and generate intrinisc parameterization
			2. The parameterization includes:
			   2.1 fixed borders and keypoints
			   2.2 fixed keypoints with natural borders
			   2.3 2D visualization for the parameterization

*********************************************************************************/
#pragma once
#include <vcg/complex/complex.h>
#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <Eigen/IterativeLinearSolvers>
#include <vector>
#include "drawing2D.hpp"
#define Pi 3.1415926

using namespace vcg;
using namespace std;

class IntriniscParameterization {

private:

	//defien 2D domain
	int Para2D_Width = 1000; // width of 2D parameterization domain
	int Para2D_Height = 1000; // height of 2D parameterization domain
	int radius = 400;

	vector<vector<int>> LaplaceNeighbor;
	vector<vector<int>> pointNeighbor;
	vector<Point3f> point;
	vector<vector<int>> face;
	vector<int> pointBorder_Index;
	vector<int> pointBorder_Index_Global;
	int point_center;
	int border_start = 0;

	//vector<Point2f> U_2D; // store 2D position result in 2D parameterization domain	
	vector<Point2f> C_2D; // store border 2D position result in 2D parameterization domain

	//weights to balance angle and area distortion
	float lamda = 1;
	float mu = 0;

public:

	void IP_init(vector<Point3f> point_input, vector<vector<int>> face_input, 
		vector<int> pointBorder_Index_input, int point_center_input) {

		point = point_input;
		face = face_input;
		pointBorder_Index = pointBorder_Index_input;
		point_center = point_center_input;

		IP_Mapping_AdjustOrder();

		//U_2D.resize(point.size());
		C_2D.resize(point.size());

		//Compute pointBorder_Index_Global
		pointBorder_Index_Global.resize(point.size(), 0);
		for (int i = 0; i < pointBorder_Index.size(); i++) {
			pointBorder_Index_Global[pointBorder_Index[i]] = 1;		
		}

		//compute pointNeighbor
		LaplaceNeighbor.resize(point.size());
		pointNeighbor.resize(point.size());
		for (int i = 0; i < LaplaceNeighbor.size(); i++) {
			vector<int> pointNeighbor_i(point.size(), 0);		
			LaplaceNeighbor[i] = pointNeighbor_i;
		}

		for (int i = 0; i < face.size(); i++) {
			int b1 = face[i][0];
			int b2 = face[i][1];
			int b3 = face[i][2];
			LaplaceNeighbor[b1][b2] = 1;
			LaplaceNeighbor[b1][b3] = 1;
			LaplaceNeighbor[b2][b3] = 1;
			LaplaceNeighbor[b2][b1] = 1;
			LaplaceNeighbor[b3][b1] = 1;
			LaplaceNeighbor[b3][b2] = 1;
			pointNeighbor[b1].push_back(b2);
			pointNeighbor[b1].push_back(b3);
			pointNeighbor[b2].push_back(b3);
			pointNeighbor[b2].push_back(b1);
			pointNeighbor[b3].push_back(b1);
			pointNeighbor[b3].push_back(b2);
		}		
	
	}

	void IP_start_fixedBorder() {

		IP_Mapping_Border();//achieve border serialization
		
		//order the border
		float sum_border = 0;
		vector<float> weight_bordfer(pointBorder_Index.size() - 1);
		weight_bordfer[0] = 0;

		for (int i = 1; i < pointBorder_Index.size() - 1; i++) {
			int b1 = pointBorder_Index[i - 1];
			int b2 = pointBorder_Index[i];
			float b12 = IP_DisPointIndex(b1, b2);
			weight_bordfer[i] = weight_bordfer[i - 1] + b12;
			sum_border = sum_border + b12;		
		}

		//compute the weight (arc length)
		for (int i = 0; i < weight_bordfer.size(); i++) {

			weight_bordfer[i] = weight_bordfer[i] / sum_border;		
		
		}

		//compute accurate 2D positions for border, clockwise rotation
		Point2f point_zero;
		point_zero[0] = -Para2D_Width;
		point_zero[1] = -Para2D_Width;

		for (int i = 0; i < C_2D.size(); i++) {

			C_2D[i] = point_zero;

		}

		C_2D[point_center][0] = 0;
		C_2D[point_center][1] = 0;


		for (int i = 0; i < pointBorder_Index.size() - 1; i++) {

			int point_index = pointBorder_Index[i];
			float weight_i = weight_bordfer[i];

			if (weight_i <= 0.25) {

				//0->90;//0.25->0
				float angle_i = (Pi / 2) * (1 - weight_i / 0.25);
				//radius
				C_2D[point_index][0] = radius * cos(angle_i);
				C_2D[point_index][1] = radius * sin(angle_i);
							
			}
			else if(weight_i <= 0.5) {				
				//0->0
				float angle_i = - (Pi / 2) * ((weight_i - 0.25) / 0.25);
				//radius
				C_2D[point_index][0] = radius * cos(angle_i);
				C_2D[point_index][1] = radius * sin(angle_i);
			
			}
			else if (weight_i <= 0.75) {

				//0->0
				float angle_i = -(Pi / 2) - (Pi / 2) * ((weight_i - 0.5) / 0.25);
				//radius
				C_2D[point_index][0] = radius * cos(angle_i);
				C_2D[point_index][1] = radius * sin(angle_i);
			
			}
			else {

				float angle_i = - Pi - (Pi / 2) * ((weight_i - 0.75) / 0.25);
				//radius
				C_2D[point_index][0] = radius * cos(angle_i);
				C_2D[point_index][1] = radius * sin(angle_i);			
			
			}		
		}	

		cout << "achieve C_2D." << endl;

		//Matrix computation
		//IP_Mapping_Matrix();
		IP_Mapping_SparseMatrix();

		//Preparing Drawing picture
		IP_Mapping_Drawing();
		
	}

private:

	void IP_Mapping_AdjustOrder() {

		vector<Point3f> pointNew;
		vector<int> pointNewIndex(point.size());
		for (int i = 0; i < pointNewIndex.size(); i++) {
			pointNewIndex[i] = i;
		}
		for (int i = 0; i < pointBorder_Index.size(); i++) {
			int index_i = pointBorder_Index[i];			
			pointNewIndex[index_i] = -1;
		}
		int index_count = 0;
		for (int i = 0; i < pointNewIndex.size(); i++) {
			if (pointNewIndex[i] >= 0) {
				pointNew.push_back(point[i]);
				pointNewIndex[i] = index_count;
				index_count++;			
			}		
		}
		for (int i = 0; i < pointNewIndex.size(); i++) {
			if (pointNewIndex[i] == -1) {
				pointNew.push_back(point[i]);
				pointNewIndex[i] = index_count;
				index_count++;
			}		
		}

		point.clear();
		point = pointNew;
		for (int i = 0; i < face.size(); i++) {
			face[i][0] = pointNewIndex[face[i][0]];
			face[i][1] = pointNewIndex[face[i][1]];
			face[i][2] = pointNewIndex[face[i][2]];		
		}
		for (int i = 0; i < pointBorder_Index.size(); i++) {
			pointBorder_Index[i] = pointNewIndex[pointBorder_Index[i]];			
		}
		point_center = pointNewIndex[point_center];	
	}

	void IP_Mapping_Matrix() {

		Eigen::MatrixXf LaplaceMatrix(LaplaceNeighbor.size(), LaplaceNeighbor.size());
		Eigen::MatrixXf C_Matrix(C_2D.size(), 2);
		
		//init LaplaceMatrix
		for (int i = 0; i < LaplaceNeighbor.size(); i++) {
			//non-inside point
			if (C_2D[i][0] <= -Para2D_Width && C_2D[i][1] <= -Para2D_Width) {				
				vector<int> LaplaceNeighbor_i = LaplaceNeighbor[i];
				float sum_i = 0;
				for (int j = 0; j < LaplaceNeighbor_i.size(); j++) {
					if (LaplaceNeighbor_i[j] == 1) {
						float cotweight_ij = IP_Mapping_CotWeight(i, j);
						float distance_ij = IP_DisPointIndex(i, j);
						LaplaceMatrix(i, j) = lamda * cotweight_ij + mu * cotweight_ij * distance_ij * distance_ij;
						sum_i = sum_i + LaplaceMatrix(i, j);
					}
					else {
						LaplaceMatrix(i, j) = 0;
					}
				}
				for (int j = 0; j < LaplaceNeighbor_i.size(); j++) {
					if (LaplaceNeighbor_i[j] == 1) {
						LaplaceMatrix(i, j) = LaplaceMatrix(i, j) / sum_i;
					}
				}
				LaplaceMatrix(i, i) = -1;
			}
			
			else if (C_2D[i][0] == 0 && C_2D[i][1] == 0) {				
				vector<int> LaplaceNeighbor_i = LaplaceNeighbor[i];
				for (int j = 0; j < LaplaceNeighbor_i.size(); j++) {					
					//if (LaplaceNeighbor_i[j] == 1) {
						//LaplaceMatrix(i, j) = 1;						
					//}					
					if (i == j) {
						LaplaceMatrix(i, j) = 1;
					}
					else {
						LaplaceMatrix(i, j) = 0;
					}
				}				
			}
			else {//inside point
				for (int j = 0; j < LaplaceNeighbor[i].size(); j++) {
					if (i == j) {
						LaplaceMatrix(i, j) = 1;
					}
					else {
						LaplaceMatrix(i, j) = 0;

					}
				}
			}						
		}

		//init C_Matrix
		for (int i = 0; i < C_2D.size(); i++) {

			if (C_2D[i][0] <= -Para2D_Width && C_2D[i][1] <= -Para2D_Width) {

				C_Matrix(i, 0) = 0;
				C_Matrix(i, 1) = 0;

			}
			else {

				C_Matrix(i, 0) = C_2D[i][0];
				C_Matrix(i, 1) = C_2D[i][1];

			}	

			//cout << C_Matrix(i, 0) << "," << C_Matrix(i, 1) << endl;
		
		}

		//cout << "point_center: " << point_center << endl;

		//for (int i = 0; i < pointBorder_Index.size(); i++) {
			//int index_i = pointBorder_Index[i];			
			//for (int j = 0; j < LaplaceNeighbor.size(); j++) {
				//if (LaplaceMatrix(index_i,j) != 0) {
					//cout << LaplaceMatrix(index_i, j) << ",";
				//}			
			//}
			//cout << endl;		
		//}

		cout << "start compute linear system." << endl;
		//Eigen::MatrixXf U_Matrix = LaplaceMatrix.ldlt().solve(C_Matrix);
		Eigen::MatrixXf U_Matrix = LaplaceMatrix.fullPivLu().solve(C_Matrix);
		//Eigen::MatrixXf U_Matrix = LaplaceMatrix.householderQr().solve(C_Matrix);
		Eigen::MatrixXf C_Test_Matrix = LaplaceMatrix * U_Matrix;

		//check
		//float t = 0;
		//for (int i = 0; i < LaplaceNeighbor.size(); i++) {
			//cout << "C_Test_Matrix:" << C_Test_Matrix(i, 0) << "," << C_Test_Matrix(i, 1) << endl;	
			//cout << "C_Matrix:" << C_Matrix(i, 0) << "," << C_Matrix(i, 1) << endl;
		//}

		for (int i = 0; i < C_2D.size(); i++) {

			C_2D[i][0] = U_Matrix(i, 0);
			C_2D[i][1] = U_Matrix(i, 1);
		
		}

		LaplaceMatrix.resize(0, 0);
		U_Matrix.resize(0, 0);
		C_Matrix.resize(0, 0);
		
		//for (int i = 0; i < pointBorder_Index.size(); i++) {

			//int index_i = pointBorder_Index[i];
			//cout << "border " << index_i << ": " << (bool)(C_2D[index_i][0] == U_2D[index_i][0]) << ";";
			//cout << (bool)(C_2D[index_i][1] == U_2D[index_i][1]) << endl;
			//cout << C_2D[index_i][0] <<","<< C_2D[index_i][1] << endl;
			//cout << U_2D[index_i][0] <<","<< U_2D[index_i][1] << endl;		
		//}		

		//free the matrix
		cout << "finish compute linear system." << endl;
		
	
	}

	void IP_Mapping_SparseMatrix() {

		Eigen::SparseMatrix<float> A1_sparse(LaplaceNeighbor.size(), LaplaceNeighbor.size());
		Eigen::MatrixXf b1_sparse(LaplaceNeighbor.size(), 2);
		Eigen::MatrixXf x1_sparse;
		std::vector<Eigen::Triplet<float>> tripletlist;		

		//init LaplaceMatrix
		for (int i = 0; i < LaplaceNeighbor.size(); i++) {
			//non-inside point
			if (C_2D[i][0] <= -Para2D_Width && C_2D[i][1] <= -Para2D_Width) {
				vector<int> LaplaceNeighbor_i = LaplaceNeighbor[i];
				float sum_i = 0;
				std::vector<Eigen::Triplet<float>> t_i;
				for (int j = 0; j < LaplaceNeighbor_i.size(); j++) {
					if (LaplaceNeighbor_i[j] == 1) {
						float cotweight_ij = IP_Mapping_CotWeight(i, j);
						float distance_ij = IP_DisPointIndex(i, j);
						float LaplaceMatrix_ij = lamda * cotweight_ij + mu * cotweight_ij * distance_ij * distance_ij;
						t_i.push_back(Eigen::Triplet<float>(i, j, LaplaceMatrix_ij));
						//cout << t_i[0].col() << "," << t_i[0].row() << "," << t_i[0].value() << endl;
						sum_i = sum_i + LaplaceMatrix_ij;
					}					
				}	
				for (int j = 0; j < t_i.size(); j++) {
					//cout << t_i[j].col() << "," << t_i[j].row() << "," << t_i[j].value() << endl;
					tripletlist.push_back(Eigen::Triplet<float>(t_i[j].row(), t_i[j].col(), t_i[j].value()/ sum_i));
				}
				tripletlist.push_back(Eigen::Triplet<float>(i, i, -1));				
			}

			else if (C_2D[i][0] == 0 && C_2D[i][1] == 0) {
				//vector<int> LaplaceNeighbor_i = LaplaceNeighbor[i];
				//for (int j = 0; j < LaplaceNeighbor_i.size(); j++) {
					//if (LaplaceNeighbor_i[j] == 1) {
						//tripletlist.push_back(Eigen::Triplet<float>(i, j, 1));						
					//}					
				//}
				tripletlist.push_back(Eigen::Triplet<float>(i, i, 1));
			}
			else {//inside point
				tripletlist.push_back(Eigen::Triplet<float>(i, i, 1));
			}
		}

		A1_sparse.setFromTriplets(tripletlist.begin(), tripletlist.end());
		A1_sparse.makeCompressed();

		//init C_Matrix
		for (int i = 0; i < C_2D.size(); i++) {

			if (C_2D[i][0] <= -Para2D_Width && C_2D[i][1] <= -Para2D_Width) {
				b1_sparse(i, 0) = 0;
				b1_sparse(i, 1) = 0;
			}
			else {

				b1_sparse(i, 0) = C_2D[i][0];
				b1_sparse(i, 1) = C_2D[i][1];

			}			

		}

		cout << "start compute sparse linear system." << endl;

		//Eigen::SimplicialCholesky<Eigen::SparseMatrix<float>> solver(A1_sparse);
		//Eigen::SparseLU<Eigen::SparseMatrix<float>> solver(A1_sparse);
		//Eigen::SparseQR<Eigen::SparseMatrix<float>, COLAMDOrdering<int>> solver(A1_sparse);
		//x1_sparse = solver.solve(b1_sparse);

		//Eigen::LeastSquaresConjugateGradient<Eigen::SparseMatrix<float> > Solver_sparse;
		//Eigen::ConjugateGradient<Eigen::SparseMatrix<float> > Solver_sparse;
		Eigen::BiCGSTAB<Eigen::SparseMatrix<float> > Solver_sparse;
		Solver_sparse.setTolerance(0.00001);
		Solver_sparse.compute(A1_sparse);
		x1_sparse = Solver_sparse.solve(b1_sparse);

		for (int i = 0; i < C_2D.size(); i++) {

			C_2D[i][0] = x1_sparse(i, 0);
			C_2D[i][1] = x1_sparse(i, 1);

		}
		cout << "finish compute sparse linear system." << endl;
	}

	void IP_Mapping_Drawing() {
		
		vector<vector<float>> pointF;
		//vector<vector<int>> facesF;

		/*
		for (int i = 0; i < C_2D.size(); i++) {
			vector<float> point_2d_i;
			if (C_2D[i][0] <= -Para2D_Width && C_2D[i][1] <= -Para2D_Width) {
				continue;
			}
			else {
				point_2d_i.push_back(C_2D[i][0]);
				point_2d_i.push_back(C_2D[i][1]);
				pointF.push_back(point_2d_i);
			}

		}
		*/

		for (int i = 0; i < C_2D.size(); i++) {
			vector<float> point_2d_i;
			point_2d_i.push_back(C_2D[i][0]);
			point_2d_i.push_back(C_2D[i][1]);
			pointF.push_back(point_2d_i);
		}
	

		Draw2D d2d;
		d2d.Draw2D_init(pointF, face, Para2D_Width, Para2D_Height);
	
	}

	//border serialization
	void IP_Mapping_Border() {

		//start point: border_start;
		int startBorder = pointBorder_Index[border_start];
		//int startBorder = 8987;
		int borderSum = pointBorder_Index.size();
		pointBorder_Index.clear();
		pointBorder_Index.push_back(startBorder);		

		int temp_before = IP_Mapping_Border_Direction(startBorder);
		int temp = startBorder;

		if (temp_before < 0) {

			cout << "border searching error!" << endl;
			return;
		
		}		

		//order border
		while (true) {

			bool judge = false;
			vector<int> LaplaceNeighbor_i = LaplaceNeighbor[temp];
			vector<int> LaplaceBorder_i;

			for (int i = 0; i < LaplaceNeighbor_i.size(); i++) {
				if (LaplaceNeighbor_i[i] == 1 && pointBorder_Index_Global[i] == 1 && i != temp_before) {
					LaplaceBorder_i.push_back(i);
				}
			}

			float dis_max = 0;	
			int temp_select = -1;
			if (LaplaceBorder_i.size() > 1) {
				for (int i = 0; i < LaplaceBorder_i.size(); i++) {
					float dis_i = IP_DisPointIndex(LaplaceBorder_i[i], temp);
					if (dis_i > dis_max) {
						dis_max = dis_i;
						temp_select = LaplaceBorder_i[i];
					}				
				}	
				temp_before = temp;
				temp = temp_select;
				judge = true;
				
			}
			else if(LaplaceBorder_i.size() == 1) {
				temp_select = LaplaceBorder_i[0];	
				temp_before = temp;
				temp = temp_select;
				judge = true;
				
			}			

			if (judge) {
				pointBorder_Index.push_back(temp);
				if (temp == startBorder) {					
					judge = false;
				}				
			}
			
			if (!judge) {
				break;			
			}
		
		}	

		if (pointBorder_Index[0] == pointBorder_Index[pointBorder_Index.size()-1]) {

			cout << "Serialize pointBorder_Index finished!" << endl;
		
		}
		else {

			cout << "Serialize pointBorder_Index failed!" << endl;
		
		}
	
	}

	//define direction for border serialization
	int IP_Mapping_Border_Direction(int startBorder) {

		int temp_before = -1;
		vector<int> LaplaceNeighbor_temp = LaplaceNeighbor[startBorder];
		for (int i = 0; i < LaplaceNeighbor_temp.size(); i++) {
			if (LaplaceNeighbor_temp[i] == 1 && pointBorder_Index_Global[i] == 1) {
				temp_before = i;
				return temp_before;
			}
		}
		return temp_before;
	
	}

	//compute distance between two points
	float IP_DisPointIndex(int a, int b) {

		float distance_ab = sqrt((point[a][0] - point[b][0]) * (point[a][0] - point[b][0]) +
			(point[a][1] - point[b][1]) * (point[a][1] - point[b][1]) +
			(point[a][2] - point[b][2]) * (point[a][2] - point[b][2]));

		return distance_ab;
	
	
	}

	//compute cotweight
	float IP_Mapping_CotWeight(int a, int b) {

		vector<int> a_N = pointNeighbor[a];
		vector<int> b_N = pointNeighbor[b];
		vector<int> sharedNeighbor;
		for (int i = 0; i < a_N.size()/2; i++) {
			int a_N1 = a_N[2 * i];
			int a_N2 = a_N[2 * i + 1];
			if (a_N1 == b) {
				sharedNeighbor.push_back(a_N2);			
			}
			if (a_N2 == b) {
				sharedNeighbor.push_back(a_N1);			
			}		
		}

		float cot_sum = 0;
		for (int i = 0; i < sharedNeighbor.size(); i++) {
			int ab_n = sharedNeighbor[i];
			//vector angle
			Point3f v1;
			v1[0] = point[a][0] - point[ab_n][0];
			v1[1] = point[a][1] - point[ab_n][1];
			v1[2] = point[a][2] - point[ab_n][2];
			Point3f v2;
			v2[0] = point[b][0] - point[ab_n][0];
			v2[1] = point[b][1] - point[ab_n][1];
			v2[2] = point[b][2] - point[ab_n][2];
			float theta = (v1[0] * v2[0] + v1[1] * v2[1] + v1[2] * v2[2]) /
				sqrt(v1[0] * v1[0] + v1[1] * v1[1] + v1[2] * v1[2]) /
				sqrt(v2[0] * v2[0] + v2[1] * v2[1] + v2[2] * v2[2]);
			float tan_i = tan(theta);
			float cot_i;
			if (tan_i == 0) {
				cot_i = 10;			
			}
			else {
				cot_i = 1 / tan_i;			
			}
			if (cot_i > 10) {
				cot_i = 10;			
			}
			if (cot_i < 0) {
				cot_i = 0;			
			}
			cot_sum = cot_sum + cot_i;		
		}		
		return cot_sum;		
	
	}
};