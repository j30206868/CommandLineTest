#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <opencv2/calib3d/calib3d.hpp>
#include <iostream>
#include <sstream>
#include <stdio.h>

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

void split_left_right_frame_stereo_frame(cv::Mat stereo_frame, cv::Mat &left, cv::Mat &right, int w, int h){
	left = stereo_frame(Rect(0, 0, w, h));
	right = stereo_frame(Rect(w, 0, w, h));
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
	int channel = 3;
	int cal_img_cout = 20;
	bool cal_end = false;
	int width = 640;
	int height = 400;
	Size borderSize(7, 5);

	cv::Mat left, right;
	Mat edges;
    namedWindow("edges",1);
    while(!cal_end)
    {
        Mat frame;
        cap >> frame; // get a new frame from camera

		if(frame.rows <= 0 || frame.cols <= 0)continue;

		split_left_right_frame_stereo_frame(frame, left, right, width, height);
		vector<Point2f> l_pointBuf;
		vector<Point2f> r_pointBuf;

		bool l_found = findChessboardCorners( left , borderSize, l_pointBuf,
												CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_FAST_CHECK | CV_CALIB_CB_NORMALIZE_IMAGE);
		bool r_found = findChessboardCorners( right, borderSize, r_pointBuf,
												CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_FAST_CHECK | CV_CALIB_CB_NORMALIZE_IMAGE);
		if (l_found && r_found)                // If both found
		{
			// improve the found corners' coordinate accuracy for chessboard
			Mat l_viewGray;
			Mat r_viewGray;

			cvtColor(left , l_viewGray, COLOR_BGR2GRAY);
			cvtColor(right, r_viewGray, COLOR_BGR2GRAY);

			cornerSubPix( l_viewGray, l_pointBuf, Size(11,11),
				Size(-1,-1), TermCriteria( CV_TERMCRIT_EPS+CV_TERMCRIT_ITER, 30, 0.1 ));
			cornerSubPix( r_viewGray, r_pointBuf, Size(11,11),
				Size(-1,-1), TermCriteria( CV_TERMCRIT_EPS+CV_TERMCRIT_ITER, 30, 0.1 ));

			//draw
			vector<Point2f> shift_r_pointBuf;
			for ( Point2f &i : r_pointBuf ) {
				shift_r_pointBuf.push_back(Point2f(i.x+width, i.y));
			}
			drawChessboardCorners( frame, borderSize, Mat(l_pointBuf), true );
			drawChessboardCorners( frame, borderSize, Mat(shift_r_pointBuf), true );
		}

        imshow("edges", frame);
		switch(waitKey(30))
		{
			case 'e':
				cal_end = true;
				break;
			case 's':
				//save this frame to be ready for calibration
				save_left_right_of_comb_img(frame, "test/", cal_img_cout + 1, h_for_each_img, w_for_each_img, channel);
				//save_stereo_frame(frame, "stereo_calib/", cal_img_cout + 1);
				printf("Save image %2dth image\n", cal_img_cout+1);
				cal_img_cout++;
				break;
		}
    }
    // the camera will be deinitialized automatically in VideoCapture destructor
    return 0;

   return 0;
}