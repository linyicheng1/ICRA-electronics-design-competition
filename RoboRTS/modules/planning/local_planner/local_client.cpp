/****************************************************************************
 *  Copyright (C) 2018 RoboMaster.
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of 
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program. If not, see <http://www.gnu.org/licenses/>.
 ***************************************************************************/

#include <thread>

#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <actionlib/client/simple_action_client.h>
#include <thread>
#include "messages/LocalPlannerAction.h"
#include "iostream"
#include "common/log.h"
#include <tf/transform_broadcaster.h>
#include <thread>
#include <vector>
#include <nav_msgs/Odometry.h>
/*
 * @brief 将输入限幅后输出
 */
double MAX_MIN(int max,int min, int x)
{
    if(max<min)
        printf("error !! MAX<MIN");
    if(x>max)
        return max;
    else if(x<min)
        return min;
    else
        return x;
}
/*
 * @brief 蒙特卡罗定位的初始位置
 */
#define X_OFFSET 1.01246473991
#define Y_OFFSET 0.984946173604
//double X_OFFSET=0;
//double Y_OFFSET=0;

nav_msgs::Path plan_info;
bool new_goal_flag = false;//是否有新的目标点
double local_planner_radio=1.5;//局部路径规划的半径值
int closeToGoal=0;//是否接近目标位置
double  distance_x;//当前机器人的位置
double  distance_y;

geometry_msgs::PoseStamped  setGoal;//目标位置
void GoalCB (const geometry_msgs::PoseStamped::ConstPtr & goal);
void myGoalCB (const geometry_msgs::PoseStamped::ConstPtr & goal);
void action_client ();
void globalPath_Callback(const nav_msgs::Path &path);//全局路径规划
void odom_Callback(const nav_msgs::Odometry &odom);//里程计信息
void chargeGoal(const nav_msgs::Path &path,double x,double y);
double getDistance(double x0,double y0,double z0,double x1,double y1,double z1);
double getAtanTh(double x0,double y0,double x1,double y1);

int main(int argc, char **argv) {
    ros::init(argc, argv, "local_test_client");

    ros::NodeHandle goal_nh;
    //订阅当前的目标点，当前的位置信息，当前的全局路径规划信息
    ros::Subscriber sub_goal = goal_nh.subscribe<geometry_msgs::PoseStamped>("move_base_simple/goal", 1, GoalCB);
    ros::Subscriber myGoal = goal_nh.subscribe<geometry_msgs::PoseStamped>("myGoal", 10, myGoalCB);
    ros::Subscriber sub_global_path=goal_nh.subscribe("global_planner_path",10,globalPath_Callback);
    ros::Subscriber sub_odom=goal_nh.subscribe("odom",20,odom_Callback);

    auto thread = std::thread(action_client);
    ros::spin();
    return 0;
}
/*
 * @brief 计算两个点的距离，返回绝对值
 */
double getDistance(double x0,double y0,double x1,double y1)
{
    double distance=(x0-x1+X_OFFSET)*(x0-x1+X_OFFSET)+(y0-y1+Y_OFFSET)*(y0-y1+Y_OFFSET);
    if(distance>=0)
        return distance;
    else if(distance<0)
        return -distance;
}
/*
 * @brief 每次从rviz中设置新的目标点后调用一次，判断目标点是否在
 *        当前位置的附近如果是则直接把目标给局部路径规划
 */
void GoalCB (const geometry_msgs::PoseStamped::ConstPtr & goal) {
    if (!plan_info.poses.empty()) {
        plan_info.poses.clear();
    }
    setGoal=*goal;
    if(getDistance(setGoal.pose.position.x,setGoal.pose.position.y,
                   distance_x,distance_y)>1)
    {
        plan_info.poses.push_back(setGoal);
        new_goal_flag = true;
        closeToGoal = 0;
    }
    else
    {
        new_goal_flag= true;
        closeToGoal=2;
    }
}
/*
 * @brief 每次发布新的目标点后调用一次，判断目标点是否在
 *        当前位置的附近如果是则直接把目标给局部路径规划
 */
void myGoalCB (const geometry_msgs::PoseStamped::ConstPtr & goal) {
    if (!plan_info.poses.empty()) {
        plan_info.poses.clear();
    }
    setGoal=*goal;
    if(getDistance(-distance_x,distance_y,setGoal.pose.position.y,setGoal.pose.position.x)>1.5)
    {
        plan_info.poses.push_back(setGoal);
        new_goal_flag = true;
        closeToGoal = 0;
    }
    else
    {
        new_goal_flag= true;
        closeToGoal=0;
    }
}
/*
 * @brief 实时获取机器人的位置信息
 * */
void odom_Callback(const nav_msgs::Odometry &odom)
{
    distance_x=odom.pose.pose.position.x;
    distance_y=odom.pose.pose.position.y;
//  std::cout<<"get odom x :"<<distance_x<<" y: "<<distance_y<<std::endl;
}
/*
 * @brief 只要有全局路径规划存在就会一直调用，计算当前局部路径规划的目标点
 */
void globalPath_Callback(const nav_msgs::Path &path)
{
    nav_msgs::Path receiveGlobalPath=path;
//  std::cout<<"                    get global path !"<<std::endl;
//  for(int i=0;i< receiveGlobalPath.poses.size();i++)
//  {
//    std::cout<<receiveGlobalPath.poses.at(i);
//  }
//  std::cout<<"                    over size is:"<<receiveGlobalPath.poses.size()<<std::endl;
    //TODO 实际场景下的坐标系反了(和单片机程序有关，注意修改)
    chargeGoal(receiveGlobalPath,distance_x,distance_y);
}
/*
 * @brief 程序开始时调用，检测局部路径规划算法是否正确运行
 */
void action_client () {
    actionlib::SimpleActionClient<messages::LocalPlannerAction> ac("local_planner_node_action", true);
    LOG_INFO<<"Waiting for action server to start.";
    ac.waitForServer();
    LOG_INFO<<"Start.";
    messages::LocalPlannerGoal goal;

    char command = '0';

    while (ros::ok()) {
        if (new_goal_flag) {
            goal.route = plan_info;
            ac.sendGoal(goal);
            new_goal_flag = false;
        }
    }
}
/*
 * @brief 根据目前的全局路径规划的路径得出局部路径规划的目标点位置
 *
 */
void chargeGoal(const nav_msgs::Path &path,double x,double y)
{
    if (!plan_info.poses.empty()) {
        plan_info.poses.clear();
    }
    int pathSize=(int)path.poses.size();
    std::cout<<"odom x: "<<closeToGoal<<" y: "<<distance_y<<std::endl;
//    std::cout<<"first path point x: "<<path.poses.at(0).pose.position.x<<
//             " y: "<<path.poses.at(0).pose.position.y<<std::endl;
    if(closeToGoal==0)
    {//TODO 寻找全局路径内第一个超出半径的点,由于路径和odom不在一个坐标系下，这里转换为减一
        for(int i=0;i<pathSize;i++)
        {
            double distance_i=getDistance(-distance_x,distance_y,path.poses.at(MAX_MIN(path.poses.size()-1,0,i)).pose.position.y, path.poses.at(MAX_MIN(path.poses.size()-1,0,i)).pose.position.x);
//            double distance_i=getDistance(path.poses.at(MAX_MIN(path.poses.size()-1,0,i)).pose.position.x,x,
//                                          path.poses.at(MAX_MIN(path.poses.size()-1,0,i)).pose.position.y,y);
            /*
             * for debug
             */
            std::cout<<"------------"<<i<<"------"<<i<<"------------"<<std::endl;
            std::cout<<"now position x: "<<distance_x<<"y: "<<distance_y<<std::endl;
            std::cout<<"now path     x: "<<path.poses.at(MAX_MIN(path.poses.size()-1,0,i)).pose.position.x
                     <<"y: "<<path.poses.at(MAX_MIN(path.poses.size()-1,0,i)).pose.position.y<<std::endl;
            std::cout<<"now distance is "<<distance_i<<std::endl;
            std::cout<<"------------"<<"-"<<"------"<<"-"<<"------------"<<std::endl;
            if(distance_i>local_planner_radio)
            {
//                double pose=getAtanTh(path.poses.at(MAX_MIN(path.poses.size()-1,0,i)).pose.position.x,path.poses.at(MAX_MIN(path.poses.size()-1,0,i)).pose.position.y,
//                                      path.poses.at(MAX_MIN(path.poses.size()-1,0,i+1)).pose.position.x,path.poses.at(MAX_MIN(path.poses.size()-1,0,i+1)).pose.position.y);
//                geometry_msgs::Quaternion goal_quat = tf::createQuaternionMsgFromYaw(pose);
////                //transform to quat
//                geometry_msgs::PoseStamped Goal;
//                Goal.pose.orientation.x=goal_quat.x;
//                Goal.pose.orientation.y = goal_quat.y;
//                Goal.pose.orientation.z = goal_quat.z;
//                Goal.pose.orientation.w = goal_quat.w;
//                Goal.pose.position=path.poses.at(MAX_MIN(path.poses.size()-1,0,i)).pose.position;

                plan_info.poses.push_back(path.poses.at(MAX_MIN(path.poses.size() - 1, 0, i)));
                new_goal_flag = true;
                break;
            }
            else if(i>=pathSize-1)
            {
                std::cout<<"final point!!"<<std::endl;
                plan_info.poses.push_back(setGoal);
                new_goal_flag = true;
                closeToGoal=0;
            }
        }
    }

}
double getAtanTh(double x0,double y0,double x1,double y1)
{
    double th=atan2(x1-x0,y1-y0);
    return MAX_MIN(0.9999,-0.9999,th);
}
//#include <thread>
//
//#include <ros/ros.h>
//#include <nav_msgs/Path.h>
//#include <geometry_msgs/PoseStamped.h>
//#include <actionlib/client/simple_action_client.h>
//#include <thread>
//#include "messages/LocalPlannerAction.h"
//#include "iostream"
//#include "common/log.h"
//
//#include <thread>
//#include <vector>
//#include <eigen3/Eigen/Core>
//#include <eigen3/Eigen/Dense>

//#include <nav_msgs/Odometry.h>
///*
// * @brief 将输入限幅后输出
// */
//double MAX_MIN(double max,double min, double x)
//{
//    if(max<min)
//        printf("error !! MAX<MIN");
//    if(x>max)
//        return max;
//    else if(x<min)
//        return min;
//    else
//        return x;
//}
///*
// * @brief 蒙特卡罗定位的初始位置
// */
//#define X_OFFSET 1.01246473991
//#define Y_OFFSET 0.984946173604
////double X_OFFSET=0;
////double Y_OFFSET=0;
//
//nav_msgs::Path plan_info;
//bool new_goal_flag = false;//是否有新的目标点
//double local_planner_radio=1.5;//局部路径规划的半径值
//int closeToGoal=0;//是否接近目标位置
//double  distance_x;//当前机器人的位置
//double  distance_y;
//
//geometry_msgs::PoseStamped  setGoal;//目标位置
//void GoalCB (const geometry_msgs::PoseStamped::ConstPtr & goal);
//void myGoalCB (const geometry_msgs::PoseStamped::ConstPtr & goal);
//void action_client ();
//void globalPath_Callback(const nav_msgs::Path &path);//全局路径规划
//void odom_Callback(const nav_msgs::Odometry &odom);//里程计信息
//void chargeGoal(const nav_msgs::Path &path,double x,double y);
//double getDistance(double x0,double y0,double z0,double x1,double y1,double z1);
//double getAtanTh(double x0,double y0,double x1,double y1);
//int main(int argc, char **argv) {
//    ros::init(argc, argv, "local_test_client");
//
//    ros::NodeHandle goal_nh;
//    //订阅当前的目标点，当前的位置信息，当前的全局路径规划信息
//    ros::Subscriber sub_goal = goal_nh.subscribe<geometry_msgs::PoseStamped>("move_base_simple/goal", 1, GoalCB);
//    ros::Subscriber myGoal = goal_nh.subscribe<geometry_msgs::PoseStamped>("myGoal", 10, myGoalCB);
//    ros::Subscriber sub_global_path=goal_nh.subscribe("global_planner_path",10,globalPath_Callback);
//    ros::Subscriber sub_odom=goal_nh.subscribe("odom",20,odom_Callback);
//
//    auto thread = std::thread(action_client);
//    ros::spin();
//    return 0;
//}
///*
// * @brief 计算两个点的距离，返回绝对值
// */
//double getDistance(double x0,double y0,double x1,double y1)
//{
//    double distance=(x0-x1+X_OFFSET)*(x0-x1+X_OFFSET)+(y0-y1+Y_OFFSET)*(y0-y1+Y_OFFSET);
//    if(distance>=0)
//        return distance;
//    else if(distance<0)
//        return -distance;
//}
///*
// * @brief 每次从rviz中设置新的目标点后调用一次，判断目标点是否在
// *        当前位置的附近如果是则直接把目标给局部路径规划
// */
//void GoalCB (const geometry_msgs::PoseStamped::ConstPtr & goal) {
//    if (!plan_info.poses.empty()) {
//        plan_info.poses.clear();
//    }
//    setGoal=*goal;
//    if(getDistance(setGoal.pose.position.x,setGoal.pose.position.y,
//                   distance_x,distance_y)<1.5)
//    {
//        std::cout<<"------------"<<"-"<<"------"<<"-"<<"------------"<<std::endl;
//        std::cout<<"now position x: "<<distance_x<<"y: "<<distance_y<<std::endl;
//        std::cout<<"now path     x: "<<setGoal.pose.position.x
//                 <<"y: "<<setGoal.pose.position.y<<std::endl;
//        std::cout<<"d            x: "<<setGoal.pose.position.x<<"y: "<<distance_y-setGoal.pose.position.y<<std::endl;
//        std::cout<<"now distance is "<<getDistance(setGoal.pose.position.x,setGoal.pose.position.y,
//                                                   distance_x,distance_y)<<std::endl;
//        std::cout<<"------------"<<"-"<<"------"<<"-"<<"------------"<<std::endl;
//
//        plan_info.poses.push_back(setGoal);
//        new_goal_flag = true;
//        closeToGoal = 1;
//    }
//    else
//    {
//        new_goal_flag= true;
//        closeToGoal=0;
//    }
//}
///*
// * @brief 每次发布新的目标点后调用一次，判断目标点是否在
// *        当前位置的附近如果是则直接把目标给局部路径规划
// */
//void myGoalCB (const geometry_msgs::PoseStamped::ConstPtr & goal) {
//    if (!plan_info.poses.empty()) {
//        plan_info.poses.clear();
//    }
//    setGoal=*goal;
//    if(getDistance(setGoal.pose.position.x,setGoal.pose.position.y,
//                   distance_x,distance_y)>1)
//    {
////        plan_info.poses.push_back(setGoal);
//        new_goal_flag = true;
//        closeToGoal = 0;
//    }
//    else
//    {
//        new_goal_flag= true;
//        closeToGoal=0;
//    }
//}
///*
// * @brief 实时获取机器人的位置信息
// * */
//void odom_Callback(const nav_msgs::Odometry &odom)
//{
//    distance_x=odom.pose.pose.position.x;
//    distance_y=odom.pose.pose.position.y;
////  std::cout<<"get odom x :"<<distance_x<<" y: "<<distance_y<<std::endl;
//}
///*
// * @brief 只要有全局路径规划存在就会一直调用，计算当前局部路径规划的目标点
// */
//void globalPath_Callback(const nav_msgs::Path &path)
//{
//    nav_msgs::Path receiveGlobalPath=path;
////  std::cout<<"                    get global path !"<<std::endl;
////  for(int i=0;i< receiveGlobalPath.poses.size();i++)
////  {
////    std::cout<<receiveGlobalPath.poses.at(i);
////  }
////  std::cout<<"                    over size is:"<<receiveGlobalPath.poses.size()<<std::endl;
//    //TODO 实际场景下的坐标系反了(和单片机程序有关，注意修改)
//    chargeGoal(receiveGlobalPath,distance_x,distance_y);
//}
///*
// * @brief 程序开始时调用，检测局部路径规划算法是否正确运行
// */
//void action_client () {
//    actionlib::SimpleActionClient<messages::LocalPlannerAction> ac("local_planner_node_action", true);
//    LOG_INFO<<"Waiting for action server to start.";
//    ac.waitForServer();
//    LOG_INFO<<"Start.";
//    messages::LocalPlannerGoal goal;
//
//    char command = '0';
//
//    while (ros::ok()) {
//        if (new_goal_flag) {
//            goal.route = plan_info;
//            ac.sendGoal(goal);
//            new_goal_flag = false;
//        }
//    }
//}
///*
// * @brief 根据目前的全局路径规划的路径得出局部路径规划的目标点位置
// *
// */
//void chargeGoal(const nav_msgs::Path &path,double x,double y)
//{
//    if (!plan_info.poses.empty()) {
//        plan_info.poses.clear();
//    }
//    int pathSize=(int)path.poses.size();
////    std::cout<<"odom x: "<<distance_x<<" y: "<<distance_y<<std::endl;
////    std::cout<<"first path point x: "<<path.poses.at(0).pose.position.x<<
////             " y: "<<path.poses.at(0).pose.position.y<<std::endl;
//    if(closeToGoal==0)
//    {//TODO 寻找全局路径内第一个超出半径的点,由于路径和odom不在一个坐标系下，这里转换为减一
//        for(int i=0;i<pathSize;i++)
//        {
//            double distance_i=getDistance(-distance_x,distance_y,path.poses.at(MAX_MIN(path.poses.size()-1,0,i)).pose.position.y, path.poses.at(MAX_MIN(path.poses.size()-1,0,i)).pose.position.x);
//            /*
//             * for debug
//             */
//            /*
//            std::cout<<"------------"<<i<<"------"<<i<<"------------"<<std::endl;
//            std::cout<<"now position x: "<<distance_x<<"y: "<<distance_y<<std::endl;
//            std::cout<<"now path     x: "<<path.poses.at(MAX_MIN(path.poses.size()-1,0,i)).pose.position.x
//                     <<"y: "<<path.poses.at(MAX_MIN(path.poses.size()-1,0,i)).pose.position.y<<std::endl;
//            std::cout<<"d            x: "<<-distance_x-path.poses.at(MAX_MIN(path.poses.size()-1,0,i)).pose.position.x<<"y: "<<-distance_y-path.poses.at(MAX_MIN(path.poses.size()-1,0,i)).pose.position.y<<std::endl;
//            std::cout<<"now distance is "<<distance_i<<std::endl;
//            std::cout<<"------------"<<"-"<<"------"<<"-"<<"------------"<<std::endl;
//             */
//            if(distance_i>local_planner_radio)
//            {
//                double pose=getAtanTh(path.poses.at(MAX_MIN(path.poses.size()-1,0,i)).pose.position.x,path.poses.at(MAX_MIN(path.poses.size()-1,0,i)).pose.position.y,
//                                      path.poses.at(MAX_MIN(path.poses.size()-1,0,i+1)).pose.position.x,path.poses.at(MAX_MIN(path.poses.size()-1,0,i+1)).pose.position.y);
//                // TODO  get now director
//                geometry_msgs::Quaternion goal_quat = tf::createQuaternionMsgFromYaw(pose);
//                //transform to quat
//                geometry_msgs::PoseStamped Goal;
//                Goal.pose.orientation.x=goal_quat.x;
//                Goal.pose.orientation.y = goal_quat.y;
//                Goal.pose.orientation.z = goal_quat.z;
//                Goal.pose.orientation.w = goal_quat.w;
//                Goal.pose.position=path.poses.at(MAX_MIN(path.poses.size()-1,0,i)).pose.position;
//                plan_info.poses.push_back(path.poses.at(i));
//                //std::cout<<path.poses.at(i)<<std::endl;
//                new_goal_flag = true;
//                break;
//            }
//            else if(i>=pathSize-1)
//            {
//                std::cout<<"final point!!"<<std::endl;
////                plan_info.poses.push_back(setGoal);
////                path.poses.at(i).pose.orientation=setGoal.pose.orientation;
//                plan_info.poses.push_back(path.poses.at(pathSize-1));
//                std::cout<<path.poses.at(pathSize-1)<<std::endl;
//                new_goal_flag = true;
//                closeToGoal=2;
//            }
//        }
//    }
//}
//double getAtanTh(double x0,double y0,double x1,double y1)
//{
//    double th=atan2(x1-x0,y1-y0);
//    return MAX_MIN(0.9999,-0.9999,th);
//}