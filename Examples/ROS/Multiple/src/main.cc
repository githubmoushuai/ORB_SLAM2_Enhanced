
#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>

#include<ros/ros.h>
#include <cv_bridge/cv_bridge.h>

#include<opencv2/core/core.hpp>
#include<opencv2/imgproc/imgproc.hpp>

#include"../../../include/System.h"

#include "TCPServer.h"
#include <strstream>
std::strstream ss;
std::string res;


TCPServer tcp1;
TCPServer tcp2;
using namespace std;





class ImageGrabber
{
public:
    ImageGrabber(ORB_SLAM2::System* pSLAM,TCPServer* tcp):mpSLAM(pSLAM),tcp(tcp) {}

    void GrabImage(const sensor_msgs::ImageConstPtr& msg);

    ORB_SLAM2::System* mpSLAM;
    TCPServer* tcp;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "Mono");
    ros::start();

    if(argc != 4)
    {
        cerr << endl << "Usage: rosrun ORB_SLAM2 Mono path_to_vocabulary path_to_settings" << endl;        
        ros::shutdown();
        return 1;
    }
    bool save=false;
    if(strcmp(argv[3],"true")==0)
    {
        save=true;
    }

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM2::System SLAM1(argv[1],argv[2],ORB_SLAM2::System::MONOCULAR,false,save);
    ORB_SLAM2::System SLAM2(argv[1],argv[2],ORB_SLAM2::System::MONOCULAR,false,false);

    
    cout << endl << endl;
    cout << "-----------------------" << endl;
    cout << "Augmented Reality Demo" << endl;
    cout << "1) Translate the camera to initialize SLAM." << endl;
    cout << "2) Look at a planar region and translate the camera." << endl;
    cout << "3) Press Insert Cube to place a virtual cube in the plane. " << endl;
    cout << endl;
    cout << "You can place several cubes in different planes." << endl;
    cout << "-----------------------" << endl;
    cout << endl;


    

    ImageGrabber igb1(&SLAM1,&tcp1);
    ImageGrabber igb2(&SLAM2,&tcp2);
    tcp1.setup(8088);
    tcp2.setup(8089);
    
    ros::NodeHandle nodeHandler;
    ros::Subscriber sub1 = nodeHandler.subscribe("/camera/image_raw1", 1, &ImageGrabber::GrabImage,&igb1);
    ros::Subscriber sub2 = nodeHandler.subscribe("/camera/image_raw2", 1, &ImageGrabber::GrabImage,&igb2);




	
	thread receiver1=thread(&TCPServer::receive,&tcp1);
	thread receiver2=thread(&TCPServer::receive,&tcp2);
	
    ros::spin();

    // Stop all threads
    SLAM1.Shutdown();
    SLAM2.Shutdown();

    ros::shutdown();

    return 0;
}

void ImageGrabber::GrabImage(const sensor_msgs::ImageConstPtr& msg)
{
    // Copy the ros image message to cv::Mat.
    cv_bridge::CvImageConstPtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvShare(msg);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
    cv::Mat im = cv_ptr->image.clone();
    cv::Mat imu;
    cv::Mat Tcw = mpSLAM->TrackMonocular(cv_ptr->image,cv_ptr->header.stamp.toSec());
    string response="";
    if(!Tcw.empty())
    {
        for(int i=0;i<4;i++)
    {
        for(int j=0;j<4;j++)
        {
            ss << Tcw.at<float>(i,j);
            ss >> res;
            ss.clear(); 
            response=response+res+",";
        }
    }
    }
    else
    {
        response="none";
    }
    tcp->Send(response);
    tcp->clean();
}


