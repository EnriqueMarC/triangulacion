#include <iostream>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <ros/package.h>
#include <std_msgs/Float32.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/CameraInfo.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <sstream>
#include <tf/transform_broadcaster.h>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/aruco.hpp>
#include <triangulacion/PoseAruco.h>
#include <triangulacion/VectorAruco.h>

using namespace std;
using namespace cv;
using namespace aruco;
cv::Mat Image,im;
cv::Mat camMatrix, distCoeffs;
int height;
int width;
ros::Subscriber cam_info_sub;

ros::Publisher pose_aruco;
float markerLength = 0.10;
bool cam_info=false;
static bool readDetectorParameters(Ptr<aruco::DetectorParameters> &params);
Mat mat_transform(Vec3d rvecs, Vec3d tvecs);
Mat mat_transform_I(Mat transform);
void infoCallback(const sensor_msgs::CameraInfoConstPtr& msg)
{
  height = msg->height;
	width = msg->width;
	camMatrix = cv::Mat::zeros(3, 3, CV_32F);
  distCoeffs = cv::Mat::zeros(1, 5, CV_32F);
	if (msg->K != boost::array<double, 9>({0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0})) {
		for (int i=0; i<3; i++) {
			for (int j=0; j<3; j++) {
				camMatrix.at<float>(i, j) = msg->K[i*3+j];
			}
		}
		for (int i=0; i<5; i++) {
			distCoeffs.at<float>(0,i) = msg->D[i];
		}
	}
	else {
		ROS_INFO("CameraInfo message has invalid intrinsics, K matrix all zeros");
	}
	cam_info=true;
	cam_info_sub.shutdown();
}
void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{  
	static tf::TransformBroadcaster broadcaster_base,broadcaster;
  static tf::TransformBroadcaster br,br_1,br_2,br_3;
	if (cam_info){
    cv_bridge::CvImagePtr cv_ptr;
    try 
	  {
		  cv_ptr = cv_bridge::toCvCopy(msg,sensor_msgs::image_encodings::BGR8);
		  Image = cv_ptr->image;
		  
		  Size size(width,height);
      Ptr<aruco::DetectorParameters> detectorParams = aruco::DetectorParameters::create();
      readDetectorParameters(detectorParams);
      Ptr<aruco::Dictionary> dictionary = aruco::getPredefinedDictionary(aruco::PREDEFINED_DICTIONARY_NAME(0));
      vector< int > ids;
      vector< vector< Point2f > > corners, rejected;
      vector< Vec3d > rvecs, tvecs;
      vector< Mat > MTF;
			Mat M;
     
      aruco::detectMarkers(Image, dictionary, corners, ids, detectorParams, rejected); 
      for(int i = 0; i < ids.size(); i++){
      	MTF.push_back(Mat(4, 4, DataType<double>::type));
			}   
			triangulacion::VectorAruco msg;
			msg.aruco.clear();
      if(ids.size() > 0)
			{
				aruco::estimatePoseSingleMarkers(corners, markerLength, camMatrix, distCoeffs, rvecs, tvecs);
 				//aruco::drawDetectedMarkers(Image, corners, ids);		
				for(unsigned int i = 0; i < ids.size(); i++)
				{
					
					aruco::drawAxis(Image, camMatrix, distCoeffs, rvecs[i], tvecs[i], markerLength/2.0  );
          MTF[i] = mat_transform(rvecs[i],tvecs[i]);
	//				MTF[i] = mat_transform_I(M);
					/*aruco.id = ids[i];
					aruco.x = tvecs[i][0];
					aruco.y = tvecs[i][1];
					aruco.pitch = -asin(MTF[i].at<double>(2,0));
					pose_aruco.publish(aruco);*/
					//cout <<"X: "<< ids[i]<<endl;
					//cout <<"X: "<< MTF[i].at<double>(0,3) << " X: "<<tvecs[i][0]<<endl;
					//cout <<"Y: "<< MTF[i].at<double>(1,3)<<endl;
					//cout <<"Z: "<< MTF[i].at<double>(2,3)<<endl;
					//cout << "ATAN: "<< atan2(MTF[i].at<double>(2,3), MTF[i].at<double>(0,3))<<endl;
					triangulacion::PoseAruco aruco;
					aruco.id  = ids[i];
					aruco.theta = atan2(MTF[i].at<double>(2,3), MTF[i].at<double>(0,3));
					
					msg.aruco.push_back(aruco);
				}
				pose_aruco.publish(msg);
			}        
      cv::imshow("Detector",  Image);
      cv::waitKey(1); 
    }
    catch(cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }
  }	
}
int main(int argc, char** argv)
{

	ros::init(argc, argv, "image_listener");
  ros::NodeHandle nh;
  image_transport::ImageTransport it(nh);	
  image_transport::Subscriber sub = it.subscribe("/usb_cam/image_raw", 1, &imageCallback); 
  cam_info_sub = nh.subscribe("/usb_cam/camera_info", 1, &infoCallback);
  pose_aruco = nh.advertise<triangulacion::VectorAruco>("/Alg_TG",1);   
	ros::spin();
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
	//params->minCornerDistance=10.0;
	params->minDistanceToBorder=3;
	//params->minMarkerDistance=10.0;
	params->minMarkerDistanceRate=0.05;
	//params->cornerRefinementWinSize=aruco::CORNER_REFINE_CONTOUR;
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
	Mat vecRD=rotI*(Mat_<double>(3,1) << transform.at<double>(0,3),transform.at<double>(1,3),transform.at<double>(2,3));

  Mat inv = (Mat_<double>(4,4) << rotI.at<double>(0,0),rotI.at<double>(0,1),rotI.at<double>(0,2),-vecRD.at<double>(0),
				         									rotI.at<double>(1,0),rotI.at<double>(1,1),rotI.at<double>(1,2),-vecRD.at<double>(1),
					 												rotI.at<double>(2,0),rotI.at<double>(2,1),rotI.at<double>(2,2),-vecRD.at<double>(2),
					 												0.0,0.0,0.0,1.0);
	return inv;
}
