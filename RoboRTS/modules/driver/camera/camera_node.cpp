///****************************************************************************
// *  Copyright (C) 2018 RoboMaster.
// *
// *  This program is free software: you can redistribute it and/or modify
// *  it under the terms of the GNU General Public License as published by
// *  the Free Software Foundation, either version 3 of the License, or
// *  (at your option) any later version.
// *
// *  This program is distributed in the hope that it will be useful,
// *  but WITHOUT ANY WARRANTY; without even the implied warranty of 
// *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// *  GNU General Public License for more details.
// *
// *  You should have received a copy of the GNU General Public License
// *  along with this program. If not, see <http://www.gnu.org/licenses/>.
// ***************************************************************************/
//
//#include <cv_bridge/cv_bridge.h>
//
//#include "modules/driver/camera/camera_node.h"
//#include "common/main_interface.h"
//
//namespace rrts {
//namespace driver {
//namespace camera {
//CameraNode::CameraNode(std::string name): rrts::common::RRTS::RRTS(name) {
//  camera_num_ = camera_param_.GetCameraParam().size();
//  publishers_.resize(camera_num_);
//  camera_threads_.resize(camera_num_);
//  camera_driver_.resize(camera_num_);
//  image_transport::ImageTransport it(nh);
//  for (unsigned int i = 0; i < camera_num_; i++) {
//    std::string topic_name = "camera_" + std::to_string(i);
//    publishers_[i] = it.advertise(topic_name, 2);
//    //create the selected camera driver
//    std::string camera_type = camera_param_.GetCameraParam()[i].camera_type;
//    camera_driver_[i] = rrts::common::AlgorithmFactory<CameraBase>::CreateAlgorithm(camera_type);
//  }
//  running_ = true;
//  StartThread();
//}
//void CameraNode::StartThread() {
//  for (unsigned int i = 0; i < camera_num_; i++) {
//    camera_threads_[i] = std::thread(&CameraNode::Update, this, i);
//  }
//}
///**
// *
// */
//void CameraNode::Update(const unsigned int index) {
//  cv::Mat img;
//  while(running_) {
//    camera_driver_[index]->StartReadCamera(index, img);
//    if(!img.empty()) {
//      sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", img).toImageMsg();
//      publishers_[index].publish(msg);
//    }
//  }
//}
///**
// * @brief Stop to read image.
// */
//void CameraNode::StoptThread() {
//
//}
//
//CameraNode::~CameraNode() {
//  running_ = false;
//  for (auto &iter: camera_threads_) {
//    if (iter.joinable())
//      iter.join();
//  }
//}
//} //namespace camera
//} //namespace drivers
//} //namespace rrts
//
//MAIN(rrts::driver::camera::CameraNode, "camera_node")

#include "opencv2/core/core.hpp"
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>
#include <string>
#include <vector>
#include "ros/ros.h"
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
using namespace cv;
using namespace std;
static void help(char** av)
{
  cout << endl
       << av[0] << " shows the usage of the OpenCV serialization functionality."         << endl
       << "usage: "                                                                      << endl
       <<  av[0] << " outputfile.yml.gz"                                                 << endl
       << "The output file may be either XML (xml) or YAML (yml/yaml). You can even compress it by "
       << "specifying this in its extension like xml.gz yaml.gz etc... "                  << endl
       << "With FileStorage you can serialize objects in OpenCV by using the << and >> operators" << endl
       << "For example: - create a class and have it serialized"                         << endl
       << "             - use it to read and write matrices."                            << endl;
}
double realTime_distance_x=-1;
double realTime_distance_y=-1;
double realTime_orient_z=-1;
double realTime_orient_w=-1;
class MyData
{
public:
    MyData() : A(-1), X(-1), Y(-1),Z(-1),W(-1),id()
    {}
    explicit MyData(int) : A(0), X(0),Y(0),Z(0),W(0), id("mydata1234") // explicit to avoid implicit conversion
    {}
    void write(FileStorage& fs) const                        //Write serialization for this class
    {
      fs << "{" << "A" << A << "X" << X << "Y" << Y << "Z" << Z << "W" << W << "id" << id << "}";
    }
    void read(const FileNode& node)                          //Read serialization for this class
    {
      A = (int)node["A"];
      X = (double)node["X"];
      Y = (double)node["Y"];
      Z = (double)node["Z"];
      W = (double)node["W"];
      id = (string)node["id"];
    }
public:   // Data Members
    char* getNum;
    int A;
    double X;
    double Y;
    double Z;
    double W;
    string id;
};
//These write and read functions must be defined for the serialization in FileStorage to work
static void write(FileStorage& fs, const std::string&, const MyData& x)
{
  x.write(fs);
}
static void read(const FileNode& node, MyData& x, const MyData& default_value = MyData()){
  if(node.empty())
    x = default_value;
  else
    x.read(node);
}
// This function will print our custom class to the console
static ostream& operator<<(ostream& out, const MyData& m)
{
  out << "{ id = " << m.id << ", ";
  out << "X = " << m.X << ", ";
  out << "Y = " << m.X << ", ";
  out << "Z = " << m.X << ", ";
  out << "W = " << m.X << ", ";
  out << "A = " << m.A << "}";
  return out;
}
//void odomCallback(const nav_msgs::Odometry &odom)
//{
//  realTime_distance_x=odom.pose.pose.position.x;
//  realTime_distance_y=odom.pose.pose.position.y;
//  realTime_orient_z=odom.pose.pose.orientation.z;
//  realTime_orient_w=odom.pose.pose.orientation.w;
//  std::cout<<"now odom "<<odom.pose.pose.position<<std::endl;
//}
void StarCB (const geometry_msgs::PoseStamped::ConstPtr & goal)
{
    geometry_msgs::PoseStamped setGoal;
    setGoal=*goal;

    realTime_distance_x=setGoal.pose.position.x;
    realTime_distance_y=setGoal.pose.position.y;
    realTime_orient_z=setGoal.pose.orientation.z;
    realTime_orient_w=setGoal.pose.orientation.w;
}
int main(int ac, char** av)
{
  ros::init(ac, av, "capture_position");
  ros::NodeHandle goal_nh;

  ros::Rate loop_rate(10);
//  ros::Subscriber sub_odom=goal_nh.subscribe("odom",10,odomCallback);
    ros::Subscriber sub_goal = goal_nh.subscribe<geometry_msgs::PoseStamped>("move_base_simple/goal", 1, StarCB);
  MyData m(1);
  std::string xml_name;
  m.getNum=*(av+1);
  xml_name.insert(0,"/home/cv/Desktop/linyicheng/starPos.yaml");
  xml_name.insert(35,m.getNum);
  int endFlag=0;
  while(ros::ok()&&endFlag==0)
  {
    { //write
      FileStorage fs(xml_name, FileStorage::WRITE);
      m.X=realTime_distance_x;
      m.Y=realTime_distance_y;
      m.W=realTime_orient_w;
      m.Z=realTime_orient_z;
      fs << "MyData" << m;                                // your own data structures
      fs.release();                                       // explicit close
      cout << "Write Done." << endl;
    }
    {//read
      cout << endl << "Reading: " << endl;
      FileStorage fs;
      fs.open(xml_name, FileStorage::READ);
      if (!fs.isOpened()) {
        cerr << "Failed to open " << xml_name << endl;
        help(av);
        return 1;
      }
      MyData read_m;
      fs["MyData"] >> read_m;                                 // Read your own structure_
      cout << "MyData = " << endl << read_m << endl << endl;
      if(read_m.X!=0&&read_m.X!=-1)
        endFlag=1;
    }
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}