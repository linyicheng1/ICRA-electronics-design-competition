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

#include "modules/decision/behavior_tree/behavior_tree.h"
#include "modules/decision/behavior_tree/planner_behavior.h"
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "modules/decision/behavior_tree/condition_behavior.h"
#include "std_msgs/UInt8MultiArray.h"
#include "std_msgs/Float32MultiArray.h"
#include "std_msgs/MultiArrayDimension.h"
void chatterCallback(const std_msgs::UInt8MultiArray::ConstPtr& msg)
{
 printf("count:%d\n",msg.get()->data[0]);
  for(int i=0;i<msg.get()->data[0];i++)
  {
    printf("I heard: foodID:[%d],status:[%d]\n",msg.get()->data[2*i+1],msg.get()->data[2*i+2]);

  }

}


int main(int argc, char **argv){
  ros::init(argc, argv, "decision_node");

  ros::NodeHandle n;
  // 告诉master需要订阅chatter topic消息
  ros::Subscriber sub = n.subscribe("order", 10, chatterCallback);
  ros::spin(); // 自循环
  return 0;
}
