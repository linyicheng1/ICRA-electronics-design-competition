#include <thread>
#include "curltest.h"
#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include "messages/GlobalPlannerAction.h"
#include "common/error_code.h"
#include "std_msgs/String.h"
#include <string>
#include <iostream>
#include "opencv2/core/core.hpp"
#include <opencv2/opencv.hpp>
#include "std_msgs/UInt8MultiArray.h"
#include "std_msgs/Float32MultiArray.h"
#include "std_msgs/MultiArrayDimension.h"

#include <vector>
#include "std_msgs/MultiArrayDimension.h"
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Odometry.h>
#include <string>
using namespace std;
using namespace cv;
double goalPoint[20][4]={};

uint8_t request_state;// every time receive
uint8_t upload_foodID;// every time receive  upload to the cloud
uint8_t upload_foodID_number;// every time receive  upload to the cloud


uint8_t final_state;// every time send
uint8_t requestfoodID;// every time send  //download from the cloud
uint8_t foodstatus;//app send once ,change ,every time send
uint8_t paynum;
uint8_t isArrval;//every time

uint8_t goalNum=0;
class MyData_Read
{
public:
    MyData_Read() : A(-1), X(-1), Y(-1),Z(-1),W(-1),id()
    {}
    explicit MyData_Read(int) : A(0), X(0),Y(0),Z(0),W(0), id("mydata1234") // explicit to avoid implicit conversion
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
static void write(FileStorage& fs, const std::string&, const MyData_Read& x)
{
    x.write(fs);
}
static void read(const FileNode& node, MyData_Read& x, const MyData_Read& default_value = MyData_Read()){
    if(node.empty())
        x = default_value;
    else
        x.read(node);
}
// This function will print our custom class to the console
static ostream& operator<<(ostream& out, const MyData_Read& m)
{
    out << "{ id = " << m.id << ", ";
    out << "X = " << m.X << ", ";
    out << "Y = " << m.Y << ", ";
    out << "Z = " << m.Z << ", ";
    out << "W = " << m.W << ", ";
    out << "A = " << m.A << "}";
    return out;
}
double get_Distance(double x0,double y0,double x1,double y1)
{
    double distance=(x0-x1+1)*(x0-x1+1)+(y0-y1+1)*(y0-y1+1);
    if(distance>=0)
        return distance;
    else if(distance<0)
        return -distance;
}
void state_Callback(const nav_msgs::Odometry &state)
{
    printf("upload_foodID_number%f\n",state.pose.pose.position.y);
    printf("upload_foodID %f\n",state.pose.pose.orientation.x);

//    std::cout<<"get request msg!!"<<(uint8_t)state.pose.pose.position.x<<std::endl;
    request_state=(uint8_t)state.pose.pose.position.x;
    upload_foodID=(uint8_t)state.pose.pose.orientation.x;
    upload_foodID_number=(uint8_t)state.pose.pose.position.y;

    printf("upload_foodID%x\n",upload_foodID);
    printf("upload_foodID_number%x\n", upload_foodID_number);
}

void Odom_Callback(const nav_msgs::Odometry &odom)
{
    double distance=get_Distance(-odom.pose.pose.position.x,odom.pose.pose.position.y,
                                 goalPoint[goalNum+1][1],goalPoint[goalNum+1][0]);
//    if(distance<0.2)
//        isArrval=1;
//    else
//        isArrval=0;
}
void globalPathCallback(const nav_msgs::Path &path)
{
    nav_msgs::Path receiveGlobalPath=path;
    std::cout<<"size of path is"<<receiveGlobalPath.poses.size();
    if(receiveGlobalPath.poses.size()<10)
    {
        isArrval=1;
    }
    else
        isArrval=0;
    printf("is arrive=%x\n",isArrval);
}
int main(int argc, char **argv) {
    ORDER order_app[10];
    int count=0;

    final_state=0;// every time send
    request_state=-1;// every time receive
    int app_request_state=-1;
    foodstatus=0;//app send once ,change ,every time send
    paynum=0;
    isArrval=0;//every time
    goalNum=0;
    int ordernum;
    int orderN;
    int uploadFN=0;
    int loop_free=1;
    int addflag=0;
    int appOrderFlag=0;
    upload_foodID=0;
    upload_foodID_number=0;
    std::string objID="",goalstr="";
//    char num='0';

    getorder(order_app,count);
    for(int i=0;i<count;i++)
    {
        if(order_app[i].status==4)
        {

            orderN=order_app[i].ordernum;
            break;
        }
    }
    ros::init(argc, argv, "USB_commutation");
    ros::NodeHandle n;
    ros::Publisher pub = n.advertise<geometry_msgs::PoseStamped>("myGoal", 10);
    ros::Publisher goalPosition_pub = n.advertise<std_msgs::Float32MultiArray>("goal_position", 10);
    ros::Subscriber sub_odom=n.subscribe("odom",20,Odom_Callback);
    ros::Publisher send_pub = n.advertise<nav_msgs::Odometry>("sendMsg", 10);
    ros::Publisher order_pub = n.advertise<std_msgs::UInt8MultiArray>("order", 10);
    ros::Subscriber state_sub = n.subscribe("state", 100, state_Callback);
    ros::Subscriber sub_global_path=n.subscribe("global_planner_path",10,globalPathCallback);
    ros::Rate loop_rate(1);
    //read the data
    FileStorage fs;
    char num[10][3]={{"0"},{"1"},{"2"},{"3"},{"4"},{"5"},{"6"},{"7"},{"8"},{"9"}};
    for (int i = 0; i < 10; i++)
    {
        std::string xml_name;
        xml_name.insert(0, "/home/cv/Desktop/linyicheng/starPos.yaml");
        MyData_Read read_m;
        read_m.getNum=num[i];
        xml_name.insert(35,read_m.getNum);
        {//read
            cout << endl << "Reading: " <<xml_name<< endl;
            FileStorage fs;
            fs.open(xml_name, FileStorage::READ);
            if (!fs.isOpened()) {
                cerr << "Failed to open " << xml_name << endl;
            }

            fs["MyData"] >> read_m;                                 // Read your own structure_
            cout << "MyData = " << endl << read_m << endl << endl;
        }
//        fs.open(xml_name, FileStorage::READ);
//        if (!fs.isOpened()) {
//            std::cerr << "Failed to open " << xml_name << std::endl;
////            return 1;
//        } else
//        {
//            std::cout<<"open success!!"<<std::endl;
            goalPoint[i][0] = read_m.X;
            goalPoint[i][1] = read_m.Y;
            goalPoint[i][2] = read_m.Z;
            goalPoint[i][3] = read_m.W;
            std::cout<<"read x:"<<goalPoint[i][0]<<"y "<<goalPoint[i][1]<<"z ";
            std::cout<<goalPoint[i][2]<<"w "<<goalPoint[i][3]<<std::endl;
//        }
    }

    int begin=0;
    while (ros::ok())
    {

        printf(" final mode is %d\n",final_state);
      geometry_msgs::PoseStamped goal;


        getorder(order_app,count);
        printf("app count is%d\n",count);
        if(final_state==1) {
            for (int i = 0; i < count; i++) {
                if (order_app[i].status == 1) {
                    app_request_state = 2;
                    requestfoodID=order_app[i].foodID;

                    break;
                }
            }
        }

        nav_msgs::Odometry send;


        /*****
         * luoji
         */
       // int re=
        if(final_state==1)
        {

            if(isArrval==1) {
                isArrval=0;
                loop_free++;
                if (loop_free == 7)
                    loop_free = 1;
                printf("you have arrived \n");


            }
            begin=loop_free;
            goalNum=loop_free;
        }
        else if(final_state==2)
        {
            begin=goalNum;
        }
        else if(final_state==4)
        {
            begin=1;
            goalNum=1;
        }
        else
        {
            begin=0;
        }


          if(final_state==0&&request_state==1)
          {
              final_state=1;
              goalNum=loop_free;
              std::cout<<"loop ="<<loop_free<<std::endl;
              begin=loop_free;
          }
          if(final_state==1&&request_state==3)
              final_state=3;
        printf("app re state is%x\n",app_request_state);
        std::cout<<app_request_state;
        if(final_state==1&&request_state==4)
        {
            begin=1;
            goalNum=1;
            final_state=2;
            isArrval=0;
            addflag=1;
            //app_request_state=0;
        }
       printf("app request state =%x\n",app_request_state);
          if(final_state==1&&app_request_state==2) {
              final_state = 2;
              isArrval=0;
              app_request_state=-1;
              appOrderFlag=1;
              checkStatus(objID,goalstr,ordernum);
              goalNum=goalstr[0]-'0'+1;
              begin=goalNum;

          }
          if(final_state==2&&isArrval==1&&addflag==1) {

              final_state = 4;

              isArrval=0;
              goalNum=0;
              begin=0;
              addflag=0;
          }
        if(final_state==2&&isArrval==1&&appOrderFlag==1) {
            final_state = 3;
            appOrderFlag=0;
            isArrval=0;
            goalNum=0;
            begin=0;
            arrived(objID);
        }
          if(final_state==3&&request_state==1)
          {
              goalNum=loop_free;
              final_state=1;
              begin=loop_free;

          }
         if(final_state==4&&request_state==1)
         {
             goalNum=loop_free;
             final_state=1;
             begin=loop_free;


         }
        printf("upload number=%d,and FN=%d\n",upload_foodID_number,uploadFN);

        if(upload_foodID_number>uploadFN)//upload food
        {
            printf("post new order !");
            uploadFN=upload_foodID_number;
          //  post(upload_foodID,1);
        }
        if(ordernum>orderN)
        {
            orderN=ordernum;
            paynum++;
        }
        std::cout<<final_state<<std::endl;
        send.pose.pose.position.x=(double)final_state;
        send.pose.pose.position.y=(double)requestfoodID;//have get
        send.pose.pose.position.z=(double)foodstatus;// get from the cloud
        send.pose.pose.orientation.x=(double)paynum;
        send.pose.pose.orientation.y=(double)goalNum;
//        send.pose.pose.orientation.z=(double);
        printf("begin %x\n",begin);
        printf("goal Num %x\n",goalNum);
        send_pub.publish(send);

//       cin>>begin;
        /***********************************/

      if(begin!=0)
      {
          goal.pose.position.x = goalPoint[begin-1][0];
          goal.pose.position.y = goalPoint[begin-1][1];
          goal.pose.orientation.z = goalPoint[begin-1][2];
          goal.pose.orientation.w = goalPoint[begin-1][3];
          goal.header.frame_id="map";
          goal.header.seq=0;
          goal.header.stamp=ros::Time::now();
          pub.publish(goal);
    }
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}


