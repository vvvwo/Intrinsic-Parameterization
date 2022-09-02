/*********************************************************************************

					  Darwing 2D Image by OpenCV

						Updating in 2022/09/01

						   By Dr. Chenlei Lv

			The functions includes:
			1. Darwing 2D image
			2. Visualize 2D domain for parameterization

*********************************************************************************/

#pragma once
#include <iostream>  
#include <opencv2/core/core.hpp>  
#include <opencv2/highgui/highgui.hpp>  
#include <opencv2/imgproc/imgproc.hpp>

using namespace std;
//using namespace cv;

class Draw2D {

private:

	cv::Mat image;
	vector<vector<float>> points;
	vector<vector<int>> faces; 
	string imageFilePath = "draw2D.jpg";
	int row_center;
	int col_center;	

public:

	void Draw2D_init(vector<vector<float>> points_input, vector<vector<int>> faces_input, int width, int height) {

		points = points_input;
		faces = faces_input;
		image.create(height, width, CV_8UC3);

		//init image to be white
		for (int i = 0; i < height; i++)
		{
			for (int j = 0; j < width; j++)
			{
				image.at<cv::Vec3b>(i, j)[0] = 255;
				image.at<cv::Vec3b>(i, j)[1] = 255;
				image.at<cv::Vec3b>(i, j)[2] = 255;
			}
		}

		col_center = height / 2;
		row_center = width / 2;

		Draw2D_Drawing();
	
	}

private:

	void Draw2D_Drawing() {

		for (int i = 0; i < points.size(); i++) {

			cv::Point p_i;
			p_i.x = points[i][0] + row_center;
			p_i.y = - points[i][1] + col_center;

			cv::circle(image, p_i, 1, cv::Scalar(112, 25, 25), -1);  // drawing point
		
		}

		for (int i = 0; i < faces.size(); i++) {

			int b1 = faces[i][0];
			int b2 = faces[i][1];
			int b3 = faces[i][2];
			cv::Point p_1;
			p_1.x = points[b1][0] + row_center;
			p_1.y = -points[b1][1] + col_center;
			cv::Point p_2;
			p_2.x = points[b2][0] + row_center;
			p_2.y = -points[b2][1] + col_center;
			cv::Point p_3;
			p_3.x = points[b3][0] + row_center;
			p_3.y = -points[b3][1] + col_center;
			//b1,b2; b2,b3; b3,b1
			cv::line(image, p_1, p_2, cv::Scalar(0, 0, 0), 1);
			cv::line(image, p_2, p_3, cv::Scalar(0, 0, 0), 1);
			cv::line(image, p_3, p_1, cv::Scalar(0, 0, 0), 1);
			
		}

		imwrite(imageFilePath, image);
	
	}

};


