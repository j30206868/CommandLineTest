#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <opencv2/calib3d/calib3d.hpp>
#include <iostream>
#include <sstream>
#include <stdio.h>

#include "ReadAndApplyCalib.cpp"

using namespace cv;
using namespace std;
void save_stereo_frame(cv::Mat stereo_frame, const char *path_and_prefix, int frame_num){
	std::stringstream sstm;
	if(frame_num < 10){
		sstm << path_and_prefix << "0" << frame_num << ".bmp";
	}else{
		sstm << path_and_prefix << frame_num << ".bmp";
	}
	cv::imwrite(sstm.str(), stereo_frame);
}
void save_left_right_of_comb_img(cv::Mat src, const char *path, int frame_num, int single_h, int single_w, int channel = 3){
	int total_w = single_w * 2;
	std::stringstream sstm;
	std::string suffix = "";
	//set filename suffix
	if(frame_num < 10){
		sstm << "0" << frame_num << ".bmp"; 
	}else{
		sstm << frame_num << ".bmp"; 
	}
	suffix = sstm.str();
	//init img
	cv::Mat img;
	if(channel == 1)
		img = Mat(single_h, single_w, CV_8UC1);
	else
		img = Mat(single_h, single_w, CV_8UC3);
	//copy left img
	img = src(Rect(0, 0, single_w, single_h));
	sstm.str("");
	sstm << path << "left" << suffix;
	cv::imwrite(sstm.str(), img);
	//copy right img
	img = src(Rect(single_w, 0, single_w, single_h));
	sstm.str("");
	sstm << path << "right" << suffix;
	cv::imwrite(sstm.str(), img);
}

void read_pair_then_split_and_save(const char *readpath, const char *savepath){
	stringstream sstm;
	int w_for_each_img = 640;
	int h_for_each_img = 400;
	int channel = 3;

	int total_image = 20;

	for(int i=1 ; i<=total_image; i++)
    {
		sstm.str("");
		if(i<10)
			sstm << readpath << "0" << i << ".bmp";
		else
			sstm << readpath << i << ".bmp";
		
        Mat frame;
        //cap >> frame; // get a new frame from camera
		std::cout << sstm.str() << std::endl;
		frame = cv::imread(sstm.str(), CV_LOAD_IMAGE_COLOR);

		save_left_right_of_comb_img(frame, savepath, i, h_for_each_img, w_for_each_img, channel);
	}
}

inline void split_left_right_frame_stereo_frame(cv::Mat stereo_frame, cv::Mat &left, cv::Mat &right, int w, int h){
	left = stereo_frame(Rect(0, 0, w, h));
	right = stereo_frame(Rect(w, 0, w, h));
}

inline void copy_left_right_into_view(cv::Mat left, cv::Mat right, cv::Mat &view){
	for (int i=0;i<view.cols;i++) {
		if (i < left.cols) {
				view.col(i) = left.col(i);
		} else {
				view.col(i) = right.col(i - left.cols);
		}
	}
}

int main( int argc, char** argv )
{
	//read_pair_then_split_and_save("stereo_calib/", "test/");
	//system("PAUSE");

	VideoCapture cap(0); // open the default camera
	if(!cap.isOpened())  // check if we succeeded
		return -1;	   

	int w_for_each_img = 640;
	int h_for_each_img = 400;
	Size imageSize(w_for_each_img, h_for_each_img);
	int channel = 3;
	int cal_img_cout = 20;
	bool cal_end = false;
	int width = 640;
	int height = 400;
	Size borderSize(7, 5);

	//single calibration
	Mat l_cameraMatrix, l_distCoeffs, r_cameraMatrix, r_distCoeffs;
	readCameraParams(l_cameraMatrix, l_distCoeffs, r_cameraMatrix, r_distCoeffs);
	Mat l_map1, l_map2, r_map1, r_map2;
    initUndistortRectifyMap(l_cameraMatrix, l_distCoeffs, Mat(),
        getOptimalNewCameraMatrix(l_cameraMatrix, l_distCoeffs, imageSize, 1, imageSize, 0),
        imageSize, CV_16SC2, l_map1, l_map2);
	initUndistortRectifyMap(r_cameraMatrix, r_distCoeffs, Mat(),
        getOptimalNewCameraMatrix(r_cameraMatrix, r_distCoeffs, imageSize, 1, imageSize, 0),
        imageSize, CV_16SC2, r_map1, r_map2);
	//stereo calibration
	Mat rmap[2][2];
	Mat cameraMatrix[2], distCoeffs[2];
	Mat R, T, R1, R2, P1, P2, Q;
	readCameraIntrAndExtr(cameraMatrix, distCoeffs, R, T, R1, R2, P1, P2, Q);
	initUndistortRectifyMap(cameraMatrix[0], distCoeffs[0], R1, P1, imageSize, CV_16SC2, rmap[0][0], rmap[0][1]);
    initUndistortRectifyMap(cameraMatrix[1], distCoeffs[1], R2, P2, imageSize, CV_16SC2, rmap[1][0], rmap[1][1]);

	cv::Mat left, right;
	Mat l_edges, r_edges, s_edges;
    namedWindow("Stereo",1);
    while(!cal_end)
    {
        Mat frame;
        cap >> frame; // get a new frame from camera

		if(frame.rows <= 0 || frame.cols <= 0)continue;

		split_left_right_frame_stereo_frame(frame, left, right, width, height);

		//single calibration
		Mat l_t = left.clone();
		remap(l_t, left, l_map1, l_map2, INTER_LINEAR);
		Mat r_t = right.clone();
		remap(r_t, right, r_map1, r_map2, INTER_LINEAR);
		//stereo calibration
		l_t = left.clone();
		r_t = right.clone();
		remap(l_t, left, rmap[0][0], rmap[0][1], CV_INTER_LINEAR);
		remap(r_t, right, rmap[1][0], rmap[1][1], CV_INTER_LINEAR);

		/*cvtColor(left, l_edges, CV_BGR2GRAY);
        GaussianBlur(l_edges, l_edges, Size(7,7), 1.5, 1.5);
		Canny(l_edges, l_edges, 0, 30, 3);

		cvtColor(right, r_edges, CV_BGR2GRAY);
        GaussianBlur(r_edges, r_edges, Size(7,7), 1.5, 1.5);
		Canny(r_edges, r_edges, 0, 30, 3);

		imshow("lefti", l_edges);
		cvWaitKey(30);
		imshow("righti", r_edges);
		cvWaitKey(30);*/
		copy_left_right_into_view(left, right, frame);
		
		for( int j = 0; j < frame.rows; j += 16 )
                line(frame, Point(0, j), Point(frame.cols, j), Scalar(0, 255, 0), 1, 8);

        imshow("Stereo", frame);
		cvWaitKey(30);
	}
    // the camera will be deinitialized automatically in VideoCapture destructor

   return 0;
}