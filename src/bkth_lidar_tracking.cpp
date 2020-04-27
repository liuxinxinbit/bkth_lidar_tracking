
#include "ros/ros.h"
#include <vector>
#include <math.h>
#include <sstream>
#include <iostream>
#include "std_msgs/String.h"
#include <sensor_msgs/PointCloud2.h> //ROS message type to publish a pointCloud
// #include "pointcloud_process/boatpose.h"
// #include "pointcloud_process/stereo_vision_msg.h"
// #include "pointcloud_process/obstacal_msg.h"
// #include "pointcloud_process/xyz_msg.h"
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/registration/icp.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/common/common.h>
#include <time.h>
#include "gnss/GPHCD_msg.h"
#include <thread>
#include <QVTKWidget.h>
#include <QtWidgets>
#include <pcl/common/centroid.h>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>       // 核心组件
#include <opencv2/highgui/highgui.hpp> // GUI
#include <opencv2/imgproc/imgproc.hpp> // 图像处理




using namespace std;
using namespace cv;
//参数初始化
long file_index = 0;//激光雷达帧数
std::string save_data_dir = "/home/liuxinxin/ToolKit/result";//数据保存路径

pcl::PointCloud<pcl::PointXYZI>::Ptr Cloud_Received(new pcl::PointCloud<pcl::PointXYZI>);//激光雷达数据



//显示结果
void show_result(pcl::PointCloud<pcl::PointXYZI>::Ptr Cloud_Received)
{
    Mat img(200, 200, CV_8UC3, Scalar(0, 0, 0));                                                 //定义结果图片
    //点云转换成二维图像
    for (size_t i = 0; i < Cloud_Received->points.size(); i++)
    {
        int x = (int)Cloud_Received->points[i].x ;
        int y = (int)Cloud_Received->points[i].y;
        int color = (Cloud_Received->points[i].z + 3) * 80;
        color = color > 255 ? 255 : color;
        circle(img, Point((100 + x), (100 - y)), 1, cv::Scalar(255, 255, 255));
    }
    // 显示结果并保存图片
    cv::imshow("Lidar window", img);
    std::string file_head = "/lidar_data_";
    std::string file_tail = ".png";
    std::string file_tail2 = ".pcd";
    std::string file_name = save_data_dir + file_head + std::to_string(file_index) + file_tail;
    cv::imwrite(file_name, img);

    std::string file_name2=save_data_dir+file_head+std::to_string(file_index)+file_tail2;
    if(Cloud_Received->points.size()>0)
    pcl::io::savePCDFileASCII (file_name2, *Cloud_Received);
    waitKey(1);
}


//接收和发布ros消息类
class SubscribeAndPublish
{
public:
    //订阅激光雷达和定位信息，定义目标信息发布器
    SubscribeAndPublish()
    {
        sub_pointcloud = nh.subscribe("/rfans_driver/rfans_points", 1, &SubscribeAndPublish::getpointcloudcallback, this);
        // sub_pointcloud = nh.subscribe("/pandar_points", 1, &SubscribeAndPublish::getpointcloudcallback, this);
        // sub_pointcloud = nh.subscribe("/livox_lidar_publisher/livox/lidar", 1, &SubscribeAndPublish::getpointcloudcallback, this); 

        // msg_pub = nh.advertise<pointcloud_process::stereo_vision_msg>("pointcloud_tracking", 1);
    }

    //节点接收激光雷达点云数据，加入点云数据处理流，产生结果发布消息
    void getpointcloudcallback(const sensor_msgs::PointCloud2::ConstPtr &msg)
    {
        pcl::PointCloud<pcl::PointXYZI> Cloud; //定义点云数据
        pcl::fromROSMsg(*msg, Cloud);         //激光雷达消息转换为点云
        *Cloud_Received = Cloud;
        show_result(Cloud_Received);
        file_index++;
        cout<<"pointcloud number -> "<<file_index<<endl;
    }

private:
    ros::NodeHandle nh;
    ros::Publisher msg_pub;         //目标检测和跟踪结果发布器
    ros::Subscriber sub_pointcloud; //激光雷达点云订阅器
    ros::Subscriber sub_boatpose;   //船体定位信息订阅器
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "bkth_Lidar_tracking");
    cout << "lidar tracking start!" << endl;
    //正式实施激光雷达目标检测跟踪
    SubscribeAndPublish sap;
    ros::spin();
    return 0;
}
