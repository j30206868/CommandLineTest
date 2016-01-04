#include <opencv2/calib3d/calib3d.hpp>
#include <iostream>

using namespace std;
using namespace cv;

static void readCameraParams(Mat &l_cameraMatrix, Mat &l_distCoeffs, Mat &r_cameraMatrix, Mat &r_distCoeffs){
	FileStorage fs("./xmls/left_single_calib.yml", FileStorage::READ);
	fs["Camera_Matrix"] >> l_cameraMatrix;
	fs["Distortion_Coefficients"] >> l_distCoeffs;
	fs.release();
	FileStorage fs2("./xmls/right_single_calib.yml", FileStorage::READ);
	fs2["Camera_Matrix"] >> r_cameraMatrix;
	fs2["Distortion_Coefficients"] >> r_distCoeffs;
	fs2.release();
}

static void readCameraIntrAndExtr(Mat cameraMatrix[2], Mat distCoeffs[2], Mat &R, Mat &T, Mat &R1, Mat &R2, Mat &P1, Mat &P2, Mat &Q){
	FileStorage fsi("./xmls/intrinsics.yml", FileStorage::READ);
	fsi["M1"] >> cameraMatrix[0];
	fsi["M2"] >> cameraMatrix[1];
	fsi["D1"] >> distCoeffs[0];
	fsi["D2"] >> distCoeffs[1];
	fsi.release();
	FileStorage fse("./xmls/extrinsics.yml", FileStorage::READ);
	fse["R"] >> R;
	fse["T"] >> T;
	fse["R1"] >> R1;
	fse["R2"] >> R2;
	fse["P1"] >> P1;
	fse["P2"] >> P2;
	fse["Q"] >> Q;
	fse.release();
}