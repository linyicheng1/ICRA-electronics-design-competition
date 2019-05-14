#include <sys/types.h>
#include <sys/socket.h>
#include <stdio.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <string.h>
#include "std_msgs/Int8.h"
#include "std_msgs/UInt8MultiArray.h"
#include "std_msgs/MultiArrayDimension.h"
#include <stdlib.h>
#include <fcntl.h>
#include <sys/shm.h>
//ros header
#include "std_msgs/String.h"
#include <string>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include<geometry_msgs/Twist.h>
#include <nav_msgs/MapMetaData.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
//eigen
#include <iostream>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#define MYPORT  6000
#define QUEUE   20
//#define BUFFER_SIZE 1024
#define BUFFER_SIZE 4096

/************************************************/
/************************CRC************************/
uint16_t CRC_INIT = 0xffff;
const uint16_t wCRC_Table[256] =
        {
                0x0000, 0x1189, 0x2312, 0x329b, 0x4624, 0x57ad, 0x6536, 0x74bf,
                0x8c48, 0x9dc1, 0xaf5a, 0xbed3, 0xca6c, 0xdbe5, 0xe97e, 0xf8f7,
                0x1081, 0x0108, 0x3393, 0x221a, 0x56a5, 0x472c, 0x75b7, 0x643e,
                0x9cc9, 0x8d40, 0xbfdb, 0xae52, 0xdaed, 0xcb64, 0xf9ff, 0xe876,
                0x2102, 0x308b, 0x0210, 0x1399, 0x6726, 0x76af, 0x4434, 0x55bd,
                0xad4a, 0xbcc3, 0x8e58, 0x9fd1, 0xeb6e, 0xfae7, 0xc87c, 0xd9f5,
                0x3183, 0x200a, 0x1291, 0x0318, 0x77a7, 0x662e, 0x54b5, 0x453c,
                0xbdcb, 0xac42, 0x9ed9, 0x8f50, 0xfbef, 0xea66, 0xd8fd, 0xc974,
                0x4204, 0x538d, 0x6116, 0x709f, 0x0420, 0x15a9, 0x2732, 0x36bb,
                0xce4c, 0xdfc5, 0xed5e, 0xfcd7, 0x8868, 0x99e1, 0xab7a, 0xbaf3,
                0x5285, 0x430c, 0x7197, 0x601e, 0x14a1, 0x0528, 0x37b3, 0x263a,
                0xdecd, 0xcf44, 0xfddf, 0xec56, 0x98e9, 0x8960, 0xbbfb, 0xaa72,
                0x6306, 0x728f, 0x4014, 0x519d, 0x2522, 0x34ab, 0x0630, 0x17b9,
                0xef4e, 0xfec7, 0xcc5c, 0xddd5, 0xa96a, 0xb8e3, 0x8a78, 0x9bf1,
                0x7387, 0x620e, 0x5095, 0x411c, 0x35a3, 0x242a, 0x16b1, 0x0738,
                0xffcf, 0xee46, 0xdcdd, 0xcd54, 0xb9eb, 0xa862, 0x9af9, 0x8b70,
                0x8408, 0x9581, 0xa71a, 0xb693, 0xc22c, 0xd3a5, 0xe13e, 0xf0b7,
                0x0840, 0x19c9, 0x2b52, 0x3adb, 0x4e64, 0x5fed, 0x6d76, 0x7cff,
                0x9489, 0x8500, 0xb79b, 0xa612, 0xd2ad, 0xc324, 0xf1bf, 0xe036,
                0x18c1, 0x0948, 0x3bd3, 0x2a5a, 0x5ee5, 0x4f6c, 0x7df7, 0x6c7e,
                0xa50a, 0xb483, 0x8618, 0x9791, 0xe32e, 0xf2a7, 0xc03c, 0xd1b5,
                0x2942, 0x38cb, 0x0a50, 0x1bd9, 0x6f66, 0x7eef, 0x4c74, 0x5dfd,
                0xb58b, 0xa402, 0x9699, 0x8710, 0xf3af, 0xe226, 0xd0bd, 0xc134,
                0x39c3, 0x284a, 0x1ad1, 0x0b58, 0x7fe7, 0x6e6e, 0x5cf5, 0x4d7c,
                0xc60c, 0xd785, 0xe51e, 0xf497, 0x8028, 0x91a1, 0xa33a, 0xb2b3,
                0x4a44, 0x5bcd, 0x6956, 0x78df, 0x0c60, 0x1de9, 0x2f72, 0x3efb,
                0xd68d, 0xc704, 0xf59f, 0xe416, 0x90a9, 0x8120, 0xb3bb, 0xa232,
                0x5ac5, 0x4b4c, 0x79d7, 0x685e, 0x1ce1, 0x0d68, 0x3ff3, 0x2e7a,
                0xe70e, 0xf687, 0xc41c, 0xd595, 0xa12a, 0xb0a3, 0x8238, 0x93b1,
                0x6b46, 0x7acf, 0x4854, 0x59dd, 0x2d62, 0x3ceb, 0x0e70, 0x1ff9,
                0xf78f, 0xe606, 0xd49d, 0xc514, 0xb1ab, 0xa022, 0x92b9, 0x8330,
                0x7bc7, 0x6a4e, 0x58d5, 0x495c, 0x3de3, 0x2c6a, 0x1ef1, 0x0f78
        };
/*
** Descriptions: CRC16 checksum function
** Input: Data to check,Stream length, initialized checksum
** Output: CRC checksum
*/
uint16_t Get_CRC16_Check_Sum(uint8_t *pchMessage,uint32_t dwLength,uint16_t wCRC)
{
    uint8_t chData;
    if (pchMessage == 0)
    {
        return 0xFFFF;
    }
    while(dwLength--)
    {
        chData = *pchMessage++;
        (wCRC) = ((uint16_t)(wCRC) >> 8) ^ wCRC_Table[((uint16_t)(wCRC) ^ (uint16_t)(chData)) &0x00ff];
    }
    return wCRC;
}
/*
** Descriptions: CRC16 Verify function
** Input: Data to Verify,Stream length = Data + checksum（dwLength=数据长度+校验和长度2，以字节为单位）
** Output: True or False (CRC Verify Result)
*/
uint32_t Verify_CRC16_Check_Sum(uint8_t *pchMessage, uint32_t dwLength)
{
    uint16_t wExpected = 0;
    if ((pchMessage == 0) || (dwLength <= 2))
    {
        return 0;
    }
    wExpected = Get_CRC16_Check_Sum ( pchMessage, dwLength - 2, CRC_INIT);
    return ((wExpected & 0xff) == pchMessage[dwLength - 2] && ((wExpected >> 8) & 0xff) == pchMessage[dwLength - 1]);
}
/*
** Descriptions: append CRC16 to the end of data
** Input: Data to CRC and append,Stream length = Data + checksum（dwLength=数据长度+校验和长度2，以字节为单位）
** Output: True or False (CRC Verify Result)
*/
void Append_CRC16_Check_Sum(uint8_t * pchMessage,uint32_t dwLength)
{
    uint16_t wCRC = 0;
    if ((pchMessage == 0) || (dwLength <= 2))
    {
        return;
    }
    wCRC = Get_CRC16_Check_Sum ( (unsigned char *)pchMessage, dwLength-2, CRC_INIT );
    pchMessage[dwLength-2] = (unsigned char)(wCRC & 0x00ff);
    pchMessage[dwLength-1] = (unsigned char)((wCRC >> 8)& 0x00ff);
}

/**
***8位CRC校验：同16位使用说明，适用于单个或者多个字节
**/
const unsigned char CRC8_INIT = 0xff;
const unsigned char CRC8_TAB[256] =
        {
                0x00, 0x5e, 0xbc, 0xe2, 0x61, 0x3f, 0xdd, 0x83, 0xc2, 0x9c, 0x7e, 0x20, 0xa3, 0xfd, 0x1f, 0x41,
                0x9d, 0xc3, 0x21, 0x7f, 0xfc, 0xa2, 0x40, 0x1e, 0x5f, 0x01, 0xe3, 0xbd, 0x3e, 0x60, 0x82, 0xdc,
                0x23, 0x7d, 0x9f, 0xc1, 0x42, 0x1c, 0xfe, 0xa0, 0xe1, 0xbf, 0x5d, 0x03, 0x80, 0xde, 0x3c, 0x62,
                0xbe, 0xe0, 0x02, 0x5c, 0xdf, 0x81, 0x63, 0x3d, 0x7c, 0x22, 0xc0, 0x9e, 0x1d, 0x43, 0xa1, 0xff,
                0x46, 0x18, 0xfa, 0xa4, 0x27, 0x79, 0x9b, 0xc5, 0x84, 0xda, 0x38, 0x66, 0xe5, 0xbb, 0x59, 0x07,
                0xdb, 0x85, 0x67, 0x39, 0xba, 0xe4, 0x06, 0x58, 0x19, 0x47, 0xa5, 0xfb, 0x78, 0x26, 0xc4, 0x9a,
                0x65, 0x3b, 0xd9, 0x87, 0x04, 0x5a, 0xb8, 0xe6, 0xa7, 0xf9, 0x1b, 0x45, 0xc6, 0x98, 0x7a, 0x24,
                0xf8, 0xa6, 0x44, 0x1a, 0x99, 0xc7, 0x25, 0x7b, 0x3a, 0x64, 0x86, 0xd8, 0x5b, 0x05, 0xe7, 0xb9,
                0x8c, 0xd2, 0x30, 0x6e, 0xed, 0xb3, 0x51, 0x0f, 0x4e, 0x10, 0xf2, 0xac, 0x2f, 0x71, 0x93, 0xcd,
                0x11, 0x4f, 0xad, 0xf3, 0x70, 0x2e, 0xcc, 0x92, 0xd3, 0x8d, 0x6f, 0x31, 0xb2, 0xec, 0x0e, 0x50,
                0xaf, 0xf1, 0x13, 0x4d, 0xce, 0x90, 0x72, 0x2c, 0x6d, 0x33, 0xd1, 0x8f, 0x0c, 0x52, 0xb0, 0xee,
                0x32, 0x6c, 0x8e, 0xd0, 0x53, 0x0d, 0xef, 0xb1, 0xf0, 0xae, 0x4c, 0x12, 0x91, 0xcf, 0x2d, 0x73,
                0xca, 0x94, 0x76, 0x28, 0xab, 0xf5, 0x17, 0x49, 0x08, 0x56, 0xb4, 0xea, 0x69, 0x37, 0xd5, 0x8b,
                0x57, 0x09, 0xeb, 0xb5, 0x36, 0x68, 0x8a, 0xd4, 0x95, 0xcb, 0x29, 0x77, 0xf4, 0xaa, 0x48, 0x16,
                0xe9, 0xb7, 0x55, 0x0b, 0x88, 0xd6, 0x34, 0x6a, 0x2b, 0x75, 0x97, 0xc9, 0x4a, 0x14, 0xf6, 0xa8,
                0x74, 0x2a, 0xc8, 0x96, 0x15, 0x4b, 0xa9, 0xf7, 0xb6, 0xe8, 0x0a, 0x54, 0xd7, 0x89, 0x6b, 0x35,
        };
unsigned char Get_CRC8_Check_Sum(unsigned char *pchMessage,unsigned int dwLength,unsigned char ucCRC8)
{
    unsigned char ucIndex;
    while (dwLength--)
    {
        ucIndex = ucCRC8^(*pchMessage++);
        ucCRC8 = CRC8_TAB[ucIndex];
    }
    return(ucCRC8);
}
/*
** Descriptions: CRC8 Verify function
** Input: Data to Verify,Stream length = Data + checksum
** Output: True or False (CRC Verify Result)
*/
short sss;
unsigned int Verify_CRC8_Check_Sum(unsigned char *pchMessage, unsigned int dwLength)
{
    unsigned char ucExpected = 0;
    if ((pchMessage == 0) || (dwLength <= 2))
        return 0;
    sss=ucExpected = Get_CRC8_Check_Sum (pchMessage, dwLength-1, CRC8_INIT);
    return ( ucExpected == pchMessage[dwLength-1] );
}
/*
** Descriptions: append CRC8 to the end of data
** Input: Data to CRC and append,Stream length = Data + checksum
** Output: True or False (CRC Verify Result)
*/
void Append_CRC8_Check_Sum(unsigned char *pchMessage, unsigned int dwLength)
{
    unsigned char ucCRC = 0;
    if ((pchMessage == 0) || (dwLength <= 2))
        return;
    ucCRC = Get_CRC8_Check_Sum ( (unsigned char *)pchMessage, dwLength-1, CRC8_INIT);
    pchMessage[dwLength-1] = ucCRC;
}
/*************************CRC***********************/
/*******DEBUG VISUAL 发布里程计的可视化信息***********/
void  pub_msg( Eigen::Vector3d& pose,nav_msgs::Path &path,ros::Publisher &mcu_path_pub_)
{

    ros::Time current_time = ros::Time::now();
    geometry_msgs::PoseStamped this_pose_stamped;
    this_pose_stamped.pose.position.x = pose(0);
    this_pose_stamped.pose.position.y = pose(1);

    geometry_msgs::Quaternion goal_quat = tf::createQuaternionMsgFromYaw(pose(2));
    this_pose_stamped.pose.orientation.x = goal_quat.x;
    this_pose_stamped.pose.orientation.y = goal_quat.y;
    this_pose_stamped.pose.orientation.z = goal_quat.z;
    this_pose_stamped.pose.orientation.w = goal_quat.w;

    this_pose_stamped.header.stamp=current_time;
    this_pose_stamped.header.frame_id="odom";
    path.poses.push_back(this_pose_stamped);
    mcu_path_pub_.publish(path);

}
float vx_cmd, vy_cmd, vz_cmd;
/*
 * y:正数 向前
 * x:正数 向左
 * z:正数 左转
 * */
int sendNewFlag=0;
void cmd_velCallback(const  geometry_msgs::Twist& cmd_vel)
{
    printf("get cmd");
    std::cout<<"get cmd !!"<<std::endl;
    vx_cmd = cmd_vel.linear.y*8;
    vy_cmd =cmd_vel.linear.x*8;
    vz_cmd = cmd_vel.angular.z*8;

    std::cout<<"x"<<vx_cmd<<std::endl;
    std::cout<<"y"<<vy_cmd<<std::endl;
    std::cout<<"z"<<vz_cmd<<std::endl;
    ROS_INFO("x :%f",vx_cmd);
    ROS_INFO("y :%f",vy_cmd);
    ROS_INFO("z :%f",vz_cmd);
    sendNewFlag=0;
}
/**********里程计的数据结构**************/
double  distance_x;
double  distance_y;
double  pos_w;
double  VecX;
double  VecY;
double  VecW;

void odom_Callback(const nav_msgs::Odometry &odom)
{
    distance_x=odom.pose.pose.position.x;
    distance_y=odom.pose.pose.position.y;
    pos_w=odom.pose.pose.position.z+180;
    printf(" printf get odom！\n");
    std::cout<<"cout get odom !"<<std::endl;
    std::cout<<"distance_x: "<<distance_x<<"distance_y"<<distance_y<<"pos_w"<<pos_w<<std::endl;
}
void float2char(float sendFloat, unsigned char *sendChar)
{
    union change
    {
        float d;
        unsigned char dat[4];
    }r1;
    r1.d = sendFloat;
    *sendChar = r1.dat[0];
    *(sendChar + 1) = r1.dat[1];
    *(sendChar + 2) = r1.dat[2];
    *(sendChar + 3) = r1.dat[3];
}
uint8_t finalMode=0,requestFoodID=0,foodState=0,payNum=0,requestMode=0,goalNum=0;
void send_Callback(const nav_msgs::Odometry &send)
{
    finalMode=(uint8_t)send.pose.pose.position.x;
    requestFoodID=(uint8_t)send.pose.pose.position.y;
    foodState=(uint8_t)send.pose.pose.position.z;
    payNum=(uint8_t)send.pose.pose.orientation.x;
    goalNum=(uint8_t)send.pose.pose.orientation.y;
    printf("get goal num %x\n",goalNum);
    std::cout<<"get send "<<std::endl;
}
int main(int argc, char** argv)
{
    /************ROS*****************/
    ros::init(argc, argv, "odom_publisher");
    ros::NodeHandle n;
    ros::Subscriber cmd_vel_sub = n.subscribe("cmd_vel", 100, cmd_velCallback);
    ros::Subscriber sendMsg_sub = n.subscribe("sendMsg", 100, send_Callback);
    ros::Publisher odom_path_pub_;
    ros::Publisher goalARM;
    geometry_msgs::PoseWithCovarianceStamped initialForAMCL;
    ros::Publisher initialPose=n.advertise<geometry_msgs::PoseWithCovarianceStamped>("initialpose",100);
    //ros::Subscriber odom_sub=n.subscribe("odom",10,odom_Callback);
    ros::Rate loop_rate(600);
    ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("odom", 600);
    ros::Publisher state_pub = n.advertise<nav_msgs::Odometry>("state", 100);
    // velocity
    double vx = 0.4;
    double vy = 0.0;
    double vth = 0.4;
    int pubBEG=0;
    int firstFrame=0;
    int errorTimes=0;
    uint8_t orderNum;
    uint8_t orderID;
    uint8_t payNum=0;
    uint8_t upload_foodID;
    ros::Time current_time;
    ros::Time last_time;
    current_time = ros::Time::now();
    last_time = ros::Time::now();

    tf::TransformBroadcaster broadcaster;
    ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter", 600);
    const double degree = M_PI/180;

    // message declarations
    geometry_msgs::TransformStamped odom_trans;
    odom_trans.header.frame_id = "odom";
    odom_trans.child_frame_id = "base_link";

    /************ROS*****************/
    ///定义sockfd
    int server_sockfd = socket(AF_INET,SOCK_STREAM, 0);

    ///定义sockaddr_in
    struct sockaddr_in server_sockaddr;
    server_sockaddr.sin_family = AF_INET;
    server_sockaddr.sin_port = htons(MYPORT);
    server_sockaddr.sin_addr.s_addr = htonl(INADDR_ANY);

    ///bind，成功返回0，出错返回-1
    if(bind(server_sockfd,(struct sockaddr *)&server_sockaddr,sizeof(server_sockaddr))==-1)
    {
        perror("bind");
        exit(1);
    }

    printf("监听%d端口\n",MYPORT);
    ///listen，成功返回0，出错返回-1
    if(listen(server_sockfd,QUEUE) == -1)
    {
        perror("listen");
        exit(1);
    }

    ///客户端套接字
    unsigned char buffer[BUFFER_SIZE];
    struct sockaddr_in client_addr;
    socklen_t length = sizeof(client_addr);

    printf("等待客户端连接\n");
    ///成功返回非负描述字，出错返回-1
    int conn = accept(server_sockfd, (struct sockaddr*)&client_addr, &length);
    if(conn<0)
    {
        perror("connect");
        exit(1);
    }
    printf("客户端成功连接\n");

    while(ros::ok())
    {
        printf("request mode %x\n",requestMode);
        printf("final mode %x",finalMode);
        ros::spinOnce();//TODO without this ,cannot receive the cmd messages
        memset(buffer,0,sizeof(buffer));
        int len = recv(conn, buffer, sizeof(buffer),0);
        //客户端发送exit或者异常结束时，退出
       // if(strcmp(buffer,"exit\n")==0 || len<=0)
         //   break;
        /****************对数据进行处理****************/
        int receiveSequence=-1;
        uint8_t receiveData[32];
        uint8_t receiveData_final[32];
        for(int i=0;i<len;i++)
        {
           // printf(" %d %x\n",i,buffer[i]);
        }
       // printf("\n");
        union testReceive
        {
            float d;
            unsigned char dat[4];
        }r[6];
        if (buffer[0] == '!')
        {
            if(buffer[1]==0x51)
            {
            //    printf("data receive begin!\n");
                for(int i=0;i<30;i++)
                    receiveData_final[i]=buffer[i];
            }
        }
        for (int i = 0; i<6; i++)
        {
            for (int j = 0; j<4; j++)
            {
                r[i].dat[j] = receiveData_final[i*4+j+2];
            }
        }
        uint8_t goalFramARM;
        if(receiveData_final[26]==0x52)
        {
//            printf("get 0x52!!");
            goalFramARM=receiveData_final[27];
//            printf(" goal is :%x\n",goalFramARM);
        }
        /***************对接收到的命令进行处理**************/

//        std_msgs::UInt8MultiArray  msg;

//        if(receiveData_final[26]==0x52)
//        {
            switch(receiveData_final[27])
            {
                case 0x00:requestMode=0;break;
                case 0x01:requestMode=1;break;
                case 0x02:requestMode=2;break;
                case 0x03:requestMode=3;break;
                case 0x04:requestMode=4;break;
                case 0x05:requestMode=5;break;
                default:break;
            }
            orderNum=receiveData_final[28];
            orderID=receiveData_final[29];
//            switch (receiveData_final[28])
//            {
//                case 0x00:isArrive=0;break;
//                case 0x01:isArrive=1;break;
//
//            }
//
//            switch (receiveData_final[29])
//            {
//                case 0x00:foodInfo=0;break;
//                case 0x01:foodInfo=1;break;
//                case 0x02:foodInfo=2;break;
//                case 0x03:foodInfo=3;break;
//                case 0x04:foodInfo=4;break;
//                case 0x05:foodInfo=5;break;
//                case 0x06:foodInfo=6;break;
//                default:break;
//            }
//
//        }
        distance_x=r[0].d;
        distance_y=r[1].d;
        pos_w=r[2].d/360.0f*3.14159*2+1.5708;
        VecX=r[3].d;
        VecY=r[4].d;
        VecW=r[5].d/360.0f*3.14159*2;
//        printf("pass CRC\n");
//        for(int i=0;i<30;i++)
//        {
//            printf(" %x",receiveData_final[i]);
//        }
//        printf("\n");



        /****************对数据进行处理****************/
        /****************给下位机发送数据****************/
//        unsigned char send_data[2];
        if(sendNewFlag>=200)
        {
            vx_cmd=0;
            vy_cmd=0;
            vz_cmd=0;
            sendNewFlag=0;
        }
        else 
          sendNewFlag++;
        unsigned char sendData[20];
        {
            //printf("vx_cmd%f",vx_cmd);
            //printf("vy_cmd%f",vy_cmd);
            // printf("vz_cmd%f",vz_cmd);
            sendData[0] = '!';
            sendData[1] = '#';
            float2char(vx_cmd, &sendData[2]);
            float2char(vy_cmd,&sendData[6]);
            float2char(vz_cmd,&sendData[10] );
            sendData[15]=finalMode;
            sendData[16]=requestFoodID;
            sendData[17]=foodState;
            sendData[18]=payNum;
            sendData[19]=goalNum;

            printf("request food id = %x\n",requestFoodID);
//            printf("goal num %x\n",goalNum);
            //Append_CRC8_Check_Sum(sendData,18);
        }
        
        send(conn, sendData, 20, 0);
        /****************给下位机发送数据****************/
        /****************把数据发送到ROS上面****************/
        //发布路径
        static nav_msgs::Path path_odom;
        Eigen::Vector3d odom_pos_cal;
        odom_path_pub_ = n.advertise<nav_msgs::Path>("odom_path_pub_",100,true);
        //赋值
        path_odom.header.stamp=current_time;
        path_odom.header.frame_id="odom";

        current_time = ros::Time::now();

        geometry_msgs::Quaternion odom_quat;
        odom_quat = tf::createQuaternionMsgFromRollPitchYaw(0,0,pos_w);

        // update transform
        odom_trans.header.stamp = current_time;
        odom_trans.transform.translation.z = 0.0;
        odom_trans.transform.rotation = tf::createQuaternionMsgFromYaw(pos_w);


        //filling the odometry
        nav_msgs::Odometry odom;
        nav_msgs::Odometry state;
        state.header.stamp=current_time;
        odom.header.stamp = current_time;
        odom.header.frame_id = "odom";
        odom.child_frame_id = "map";
        state.pose.pose.position.x=requestMode;
        state.pose.pose.orientation.x=(double)orderNum;//upload food id
        state.pose.pose.position.y=(double)orderID;//upload food id number flag
        printf("order id %f\n",orderID);
        printf("order Num %f\n",orderNum);
        state_pub.publish(state);
        // position
        odom.pose.pose.position.z = 0.0;
        odom.pose.pose.orientation = odom_quat;

        //velocity
        odom.twist.twist.linear.x = VecX;
        odom.twist.twist.linear.y = VecY;
        odom.twist.twist.linear.z = 0.0;
        odom.twist.twist.angular.x = 0.0;
        odom.twist.twist.angular.y = 0.0;
        odom.twist.twist.angular.z = VecW;
        odom.pose.pose.position.x = 0;
        odom.pose.pose.position.y = 0;
        odom.pose.pose.position.z = 0.0;
        odom.pose.pose.orientation = odom_quat;

        //velocity
        odom.twist.twist.linear.x = 0;
        odom.twist.twist.linear.y = 0;
        odom.twist.twist.linear.z = 0.0;
        odom.twist.twist.angular.x = 0.0;
        odom.twist.twist.angular.y = 0.0;
        odom.twist.twist.angular.z = 0;
        last_time = current_time;
//        chatter_pub.publish(msg);

        broadcaster.sendTransform(odom_trans);

        odom_pos_cal(0)=-distance_y;
        odom_pos_cal(1)=distance_x;
        odom_pos_cal(2)=pos_w;
        pub_msg(odom_pos_cal,path_odom,odom_path_pub_);

        /*printf("get result: distance_x:---:%f\n dx%f",last_x,last_x2-last_x);
        printf("get result: distance_x:---:%f\n dx%f",last_x,last_x2-last_x);
        printf("get result: distance_y:---:  %f\n dy%f",last_y,last_y2-last_y);
        printf("get result: pos_w:---:  %f\n dz%f",last_th,last_th2-last_th);
        printf("get result: Vec_x:---%f\n", VecX);
        printf("get result: Vec_y:---%f\n", VecY);
        printf("get result: Vec_w:---%f\n", VecW);*/
        odom_trans.transform.translation.x =-distance_y;
        odom_trans.transform.translation.y =distance_x;
        odom.pose.pose.position.x =-distance_y;
        odom.pose.pose.position.y =distance_x;
        broadcaster.sendTransform(odom_trans);

        odom_pos_cal(0)=distance_x;
        odom_pos_cal(1)=distance_y;
        odom_pos_cal(2)=pos_w;
        pub_msg(odom_pos_cal,path_odom,odom_path_pub_);

        /*printf("get result: distance_x:---:%f\n ",distance_x);
         printf("get result: distance_y:---:  %f\n ",distance_y);
         printf("get result: pos_w:---:  %f\n ",pos_w);
         printf("get result: Vec_x:---%f\n", VecX);
         printf("get result: Vec_y:---%f\n", VecY);
         printf("get result: Vec_w:---%f\n", VecW);*/
        initialForAMCL.header.stamp=ros::Time::now();
        initialForAMCL.pose.pose=odom.pose.pose;
        for(int i=0;i<36;i++)
        {
            initialForAMCL.pose.covariance[i]=0.2;
        }
        //initialPose.publish(initialForAMCL);

        odom_pub.publish(odom);

        ros::spinOnce();
        loop_rate.sleep();
        /****************把数据发送到ROS上面****************/
    }
    close(conn);
    close(server_sockfd);
    return 0;
}
