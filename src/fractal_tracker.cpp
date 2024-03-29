#include "ros/ros.h"
#include <iostream>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <ros/package.h>

#include <std_msgs/Float32.h>
#include <std_msgs/Bool.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/CameraInfo.h>

//#include "aruco.h"
#include <opencv2/imgproc/imgproc.hpp>
#include <sstream>
#include <tf/transform_broadcaster.h>

#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/aruco.hpp>
//#include <ps4_ros/PID_gains.h>
//#include <rehabilitador/error_ejes.h>
#include "geometry_msgs/Twist.h"
#include <math.h>
#include <triangulacion/PoseAruco.h>
#include <triangulacion/VectorAruco.h>

#define pi 3.14159

using namespace std;
using namespace cv;
using namespace aruco;

VideoCapture cap(0);
Mat Image;
//Mat camMatrix = (Mat_<double>(3,3) << 993.9147556551712, 0, 623.3000840807752, 0, 1000.499558902831, 358.8895289373344, 0, 0, 1);
//Mat distCoeffs = (Mat_<double>(1,5)<< 0.1498811233861027, -0.3082727411825578, -0.001258945407272281, -0.001153500340496638, 0 );
Mat camMatrix = (Mat_<double>(3,3) << 1024.276091, 0, 620.614256, 0, 1033.081372, 372.047331, 0, 0, 1);
//Mat distCoeffs = (Mat_<double>(1,5)<< 0.183718, -0.319692, -0.000977, 0.002573, 0.000000);
Mat distCoeffs = (Mat_<double>(1,5)<< 0.183718, -0.319692, -0.039, -0.0110, 0.000000);
Ptr<aruco::DetectorParameters> detectorParams = aruco::DetectorParameters::create();
Ptr<aruco::Dictionary> dictionary = aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50);
vector< vector< Point2f > > corners, rejected;
vector< Vec3d > rvecs, tvecs;
vector < Mat> MTF;
Mat M;
int counter[23] = {};
float aru_x[23] = {}, aru_y[23] = {};
vector<int> ids;
double markerLength = 0.0865;
bool cam_info=false;
static bool readDetectorParameters(Ptr<aruco::DetectorParameters> &params);
Mat mat_transform(Vec3d rvecs, Vec3d tvecs);
Mat mat_transform_I(Mat transform);
ros::Publisher pose_aruco;
geometry_msgs::Twist msg;

void timerCallback(const ros::TimerEvent&)
  {  
    cap.grab();
    cap.retrieve(Image);
    aruco::detectMarkers(Image, dictionary, corners, ids, detectorParams, rejected); 
    for(int i = 0; i < ids.size(); i++)
    {
      MTF.push_back(Mat(4, 4, DataType<double>::type));
    }   

    triangulacion::VectorAruco msg;
    msg.aruco.clear();

    if(ids.size() > 0)
    {
      aruco::estimatePoseSingleMarkers(corners, markerLength, camMatrix, distCoeffs, rvecs, tvecs);
      aruco::drawDetectedMarkers(Image, corners, ids);
      for(unsigned int i = 0; i < ids.size(); i++)
      {
        //aruco::drawAxis(Image, camMatrix, distCoeffs, rvecs[i], tvecs[i], markerLength/2.0  );
        M = mat_transform(rvecs[i],tvecs[i]);
        MTF[i] = mat_transform_I(M);
        //cout << M << endl;
        //cout << MTF[i] << endl;
        //cout <<"ID: "<< ids[i]<<endl;
        //cout <<"X: "<< MTF[i].at<double>(0,3) /*<< " Y: " << MTF[i].at<double>(1,3)*/ << " Z: "<< MTF[i].at<double>(2,3)<<endl;
        cout << endl;
        triangulacion::PoseAruco aruco;

        counter[ids[i]]++;

        aru_x[ids[i]] = (aru_x[ids[i]]*(counter[ids[i]]-1) + MTF[i].at<double>(0,3))/(counter[ids[i]]);
        aru_y[ids[i]] = (aru_y[ids[i]]*(counter[ids[i]]-1) + MTF[i].at<double>(2,3))/(counter[ids[i]]);
        aruco.id  = ids[i];
        //aruco.theta = (atan2(MTF[i].at<double>(2,3), MTF[i].at<double>(0,3))-0.5*pi)*180.0/pi;
        aruco.theta = (atan2(aru_y[ids[i]], aru_x[ids[i]])-0.5*pi)*180.0/pi;
        cout <<"ID: "<< ids[i]<<endl;
        cout <<"X: "<< aru_x[ids[i]] /*<< " Y: " << MTF[i].at<double>(1,3)*/ << endl;
        cout <<"Y: "<< aru_y[ids[i]]<<endl;
        msg.aruco.push_back(aruco);
        if (counter[ids[i]] == 100) counter[ids[i]] = 0;
      }
    pose_aruco.publish(msg);
  }
	 /* cv::rectangle(Image,Point(620+40,372-40),Point(620-40,372+40),Scalar(0,255,0),1); 
    line(Image, Point(620,372-40), Point(620,372+40), Scalar(0, 50, 125), 1, LINE_8);
    line(Image, Point(620+40,372), Point(620-40,372), Scalar(0, 50, 125), 1, LINE_8);

    cv::imshow("Detector",  Image);
    cv::waitKey(5);*/
  }

 
int main(int argc, char** argv)
{

  ros::init(argc, argv, "fractal_tracker");
  ros::NodeHandle nh;
  pose_aruco = nh.advertise<triangulacion::VectorAruco>("/Alg_TG", 1000); 
  /*ros::Subscriber Image = nh.subscribe("/image_cam_rpi", 1, estimacion);*/
  cap.set(CAP_PROP_FRAME_WIDTH,1280);
  cap.set(CAP_PROP_FRAME_HEIGHT,720);
  
  readDetectorParameters(detectorParams);
  ros::Timer timer =  nh.createTimer(ros::Duration(1.0/30.0),timerCallback);
  ros::spin();
  return 0;
}

static bool readDetectorParameters(Ptr<aruco::DetectorParameters> &params) {
  //params->nmarkers=1024;
  params->adaptiveThreshWinSizeMin=3;
	params->adaptiveThreshWinSizeMax=23;
	params->adaptiveThreshWinSizeStep=10;
	//params->adaptiveThreshWinSize=21;
	params->adaptiveThreshConstant=7;
	params->minMarkerPerimeterRate=0.03;
	params->maxMarkerPerimeterRate=4.0;
	params->polygonalApproxAccuracyRate=0.05;
	//params->minCornerDistanceRate=10.0;
	params->minDistanceToBorder=3;
	//params->minMarkerDistance=10.0;
	params->minMarkerDistanceRate=0.05;
	//params->cornerRefinementMethod=aruco::CORNER_REFINE_SUBPIX;
	params->cornerRefinementWinSize=5;
	params->cornerRefinementMaxIterations=30;
	params->cornerRefinementMinAccuracy=0.1;
	params->markerBorderBits=1;
	params->perspectiveRemovePixelPerCell=8;
	params->perspectiveRemoveIgnoredMarginPerCell=0.13;
	params->maxErroneousBitsInBorderRate=0.04;
	params->minOtsuStdDev=5.0;
	params->errorCorrectionRate=0.6;
    return true;
}

Mat mat_transform(Vec3d rvecs, Vec3d tvecs){
  
	Mat trans, rot;
    Rodrigues(rvecs,rot);
	trans = (Mat_<double>(4,4) << rot.at<double>(0,0),rot.at<double>(0,1),rot.at<double>(0,2),tvecs[0],
							      rot.at<double>(1,0),rot.at<double>(1,1),rot.at<double>(1,2),tvecs[1],
			                   	  rot.at<double>(2,0),rot.at<double>(2,1),rot.at<double>(2,2),tvecs[2],
			                   	  0.0,0.0,0.0,1.0);

	return trans;
}

Mat mat_transform_I(Mat transform){
	Mat rot = (Mat_<double>(3,3) << transform.at<double>(0,0),transform.at<double>(0,1),transform.at<double>(0,2),
				         transform.at<double>(1,0),transform.at<double>(1,1),transform.at<double>(1,2),
					 transform.at<double>(2,0),transform.at<double>(2,1),transform.at<double>(2,2));
	Mat rotI;
	transpose(rot,rotI);
	Mat vecRD=(Mat_<double>(3,1) << transform.at<double>(0,3),transform.at<double>(1,3),transform.at<double>(2,3));
        Mat inv = (Mat_<double>(4,4) << rotI.at<double>(0,0),rotI.at<double>(0,1),rotI.at<double>(0,2),vecRD.at<double>(0),
 					rotI.at<double>(1,0),rotI.at<double>(1,1),rotI.at<double>(1,2),-vecRD.at<double>(1),
					rotI.at<double>(2,0),rotI.at<double>(2,1),rotI.at<double>(2,2),vecRD.at<double>(2),
					0.0,                 0.0,                 0.0,                 1.0);
	return inv;
}
