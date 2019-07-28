
#include "ros/ros.h"
#include "slam/GetCameraImage.h"
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include "sensor_msgs/Image.h"
#include <signal.h>
#include <stdlib.h>
#include <stdio.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/core.hpp"
#include "opencv2/features2d.hpp"
#include "opencv2/xfeatures2d.hpp"
#include <cstdlib>
#include <ctime>


void handle(int a){ exit(1);}

static const std::string OPENCV_WINDOW = "Image window";


int main(int argc, char **argv)
{


  ros::init(argc, argv, "img_rec_client");

  ros::NodeHandle n;
  ros::ServiceClient client = n.serviceClient<slam::GetCameraImage>("get_camera_view");
  ros::Rate loop_rate(100);


    signal (SIGINT,handle);


    cv::Mat prev_img;
    std::vector<cv::KeyPoint> prev_keypoints;  
    while(ros::ok())
    {
      std::cout<<"loop begin"<<std::endl;
      slam::GetCameraImage srv;
      client.call(srv);

      std::cout<<"Received image"<<std::endl;
      int minHessian = 4000;
      cv_bridge::CvImagePtr InImage;
      InImage = cv_bridge::toCvCopy(srv.response.image);

      cv::Mat gray;

      cv::cvtColor(InImage->image, gray, CV_BGR2GRAY);

      cv::Mat kp;


      cv::Ptr<cv::xfeatures2d::SURF> detector = cv::xfeatures2d::SURF::create( minHessian );

      std::vector<cv::KeyPoint> keypoints;
      detector->detect(gray, keypoints);

      drawKeypoints( InImage->image, keypoints, kp, cv::Scalar::all(-1), cv::DrawMatchesFlags::DEFAULT );


      cv::imshow("test", kp);
      cv::waitKey(1);

      loop_rate.sleep();
    }


  return 0;
}
