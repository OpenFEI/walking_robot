//roslaunch openni_launch openni.launch
//rosrun p2os_driver p2os _port:=/dev/ttyUSB0
//rosrun p2os_dashboard p2os_dashboard


#include <stdio.h>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "std_msgs/String.h"
#include <sstream>
#include <geometry_msgs/Twist.h>

namespace enc = sensor_msgs::image_encodings;

class DepthInfo
{
  ros::NodeHandle nh_;
  
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Subscriber depth_sub_;
  image_transport::Publisher depth_pub_;
    
    /*
     image_transport should always be used to subscribe to and publish images. 
     It provides transparent support for transporting images in low-bandwidth compressed formats.  
     //www.ros.org/wiki/image_transport
     */
  


public:
  
  
  cv::Mat rgb_frame; //coloca na forma matricial
  cv::Mat depth_frame; //coloca na forma forma matricial
    
  ros::Publisher cmd_vel_pub_;
  geometry_msgs::Twist base_cmd;

  

  
  DepthInfo()  
    : it_(nh_)
  {
    cmd_vel_pub_  = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 1); //publishing
    depth_pub_ = it_.advertise("out", 1); //
    depth_sub_ = it_.subscribe("camera/depth/image", 1, &DepthInfo::depthInfoCb, this); //subscribes
    image_sub_ = it_.subscribe("camera/rgb/image_color", 1, &DepthInfo::imageCb, this); //subscribes   
  } 
    
  //perfumaria ///////
  void imageCb(const sensor_msgs::ImageConstPtr & msg)
  {
    cv_bridge::CvImagePtr cv_ptr_rgb;
    //cv_bridge::CvImagePtr is a pointer type that points to NULL by default. You have to allocate storage before you can actually use it
    try
    {
      cv_ptr_rgb = cv_bridge::toCvCopy(msg, enc::BGR8); //make a copy of the ROS message data. //bgra8: BGR color image with an alpha channel
                                                         //Note that mono8 and bgr8 are the two image encodings expected by most OpenCV functions.
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

    rgb_frame = cv_ptr_rgb->image; //
    cv::circle(rgb_frame, cv::Point(20, 240), 5, cv::Scalar(255, 0, 0), 2);
    cv::circle(rgb_frame, cv::Point(320, 240), 5, cv::Scalar(255, 0, 0), 2);
    cv::circle(rgb_frame, cv::Point(620, 240), 5, cv::Scalar(255, 0, 0), 2);
    //circle(Mat& img, Point center, int radius, const Scalar& color, int lineType)
    cv::imshow("rgb_image", rgb_frame);
    cv::waitKey(3);
  }
    
 //fim da perfumaria ///


  void depthInfoCb(const sensor_msgs::ImageConstPtr & msg)
  {
    cv_bridge::CvImagePtr cv_ptr_depth;

    try {cv_ptr_depth = cv_bridge::toCvCopy(msg, enc::TYPE_32FC1);}
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

    depth_frame = cv_ptr_depth->image; //
    cv::imshow("depth_image", depth_frame);
    int info=get_depthInfo();

    //perfumaria - mostra circulo
    if(info==1) cv::circle(depth_frame, cv::Point(20, 240), 5, cv::Scalar(0, 0, 0), 2);
    else if(info==2) cv::circle(depth_frame, cv::Point(320, 240), 5, cv::Scalar(0, 0, 0), 2);
    else if(info==3) cv::circle(depth_frame, cv::Point(620, 240), 5, cv::Scalar(0, 0, 0), 2);
    else
    {
     cv::circle(depth_frame, cv::Point(20, 240), 5, cv::Scalar(255, 255, 255), 2);
     cv::circle(depth_frame, cv::Point(320, 240), 5, cv::Scalar(255, 255, 255), 2);
     cv::circle(depth_frame, cv::Point(620, 240), 5, cv::Scalar(255, 255, 255), 2);
    }// fim da perfumaria    

    cv::imshow("depth_image", depth_frame); //mostra a imagem
    depth_pub_.publish(cv_ptr_depth->toImageMsg()); 

    
//***************    comandos de controle do robo *****************************************

    ///limpa variaveis /////
     base_cmd.angular.x=0;
     base_cmd.angular.y=0;
     base_cmd.angular.z=0;
     base_cmd.linear.x=0;
     base_cmd.linear.y=0;
     base_cmd.linear.z=0;        

    if(info==1)//turn right (yaw) and drive forward at the same time
    {    
      base_cmd.angular.z = -0.75;
      base_cmd.linear.x = 0.25;
    }
    
    else if(info==2) //move forward
    base_cmd.linear.x = 0.25;

    else if(info==3)//turn left (yaw) and drive forward at the same time
    {
    base_cmd.angular.z = 0.75;
    base_cmd.linear.x = 0.25;
    }
   else //stop
   {
      base_cmd.angular.x=0;
      base_cmd.angular.y=0;
      base_cmd.angular.z=0;
      base_cmd.linear.x=0;
      base_cmd.linear.y=0;
      base_cmd.linear.z=0;

   }

   cmd_vel_pub_.publish(base_cmd);  //publica
    
//*******************************************************************    

  }


  int get_depthInfo()
  {
    int posX_d = 20;              // cols
    int posY_d = 240;             // rows
    int posX_c = 320;             // cols
    int posY_c = 240;             // rows
    int posX_e = 620;             // cols
    int posY_e = 240;             // rows
    
      double depthInfo_e,depthInfo_d,depthInfo_c;


    depthInfo_e = depth_frame.at<float>(posY_e, posX_e);
    depthInfo_d = depth_frame.at<float>(posY_d, posX_d);
    depthInfo_c = depth_frame.at<float>(posY_c, posX_c);


     if(depthInfo_d>0.600 && depthInfo_d<1.000) // retorna3
       {
         std::cout << "depthInfo Direita [" << posX_e << ", " << posY_e << "] : "  << depthInfo_e << std::endl;
         return 3;
       }

       else if(depthInfo_c>0.600 && depthInfo_c<1.000) //retorna2
       {
         std::cout << "depthInfo Centro [" << posX_c << ", " << posY_c << "] : "  << depthInfo_d << std::endl;
         return 2;
       }

       else if(depthInfo_e>0.600 && depthInfo_e<1.000) //retorn1
       {
         std::cout << "depthInfo Esquerda [" << posX_d << ", " << posY_d << "] : "  << depthInfo_d << std::endl;
         return 1;
         

       }
      else{return 0;}
  }

  



};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "depth_info"); 

  DepthInfo ic;
  ros::spin();
  return 0;
}
