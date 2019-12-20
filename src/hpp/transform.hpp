#include<iostream>
#include<vector>
#include <librealsense/rs.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>
using namespace std;
using namespace cv;
int const DEPTH_WIDTH  = 640;
int const DEPTH_HEIGHT = 480;
double  theta_cam = 0;
vector <vector<int>> transform(vector<unsigned short> depthBuffer, Mat colorImage){
    Mat color_PT = Mat::zeros(Size(DEPTH_WIDTH, DEPTH_HEIGHT), CV_8UC3);
    Mat depth_HSV = Mat::zeros(Size(DEPTH_WIDTH, DEPTH_HEIGHT), CV_8UC3);
    double depth_th = 0;
}
