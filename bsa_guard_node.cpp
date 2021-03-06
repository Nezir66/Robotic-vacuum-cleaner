// Copyright 1996-2020 Cyberbotics Ltd.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "ros/ros.h"
#include <webots_ros/set_float.h>
#include <webots_ros/set_int.h>
#include <webots_ros/set_bool.h>
#include <sensor_msgs/Range.h>
#include <webots_ros/SetVelocityGuard.h>

#include <signal.h>
#include <stdio.h>
#include <string>



ros::ServiceClient leftVelocityClient, rightVelocityClient;
webots_ros::set_float leftVelocitySrv, rightVelocitySrv;

static double lspeed = 0;
static double rspeed = 0;
static double lspeedNew = 0;
static double rspeedNew = 0;
static double speed = 5;
double lastSpeedSetTime = 0;

bool sensor_front_left1;
bool sensor_front_left2;
bool sensor_front_middle;
bool sensor_front_right1;
bool sensor_front_right2;
bool sensor_back_left;
bool sensor_back_right;

void quit(int sig) {
  ROS_INFO("User stopped the node.");
  ros::shutdown();
  exit(sig);
}



void setSpeed(double lspeedNew, double rspeedNew) {
    lastSpeedSetTime = ros::Time::now().toSec();
    leftVelocitySrv.request.value = lspeedNew;
    rightVelocitySrv.request.value = rspeedNew;
    if (!leftVelocityClient.call(leftVelocitySrv) || !rightVelocityClient.call(rightVelocitySrv) || !leftVelocitySrv.response.success || !rightVelocitySrv.response.success) {
        ROS_ERROR("Failed to send new speed commands to the robot.");
    }
}

bool VelocityCallback(webots_ros::SetVelocityGuard::Request &req,
                webots_ros::SetVelocityGuard::Response &res)
{
        double rspeedNew = req.right;
        double lspeedNew = req.left;
        bool front = req.front;
        bool back = req.back;
        bool EinAus = req.EinAus;
        
        ROS_INFO("got Value right: %f", rspeedNew);
        ROS_INFO("got Value left: %f", lspeedNew);
        ROS_INFO("setVelocity is called ");
        if(EinAus == false) {
        if ((sensor_front_left1 == false) && (sensor_front_left2 == false) && (sensor_front_middle == false) && (sensor_front_right1 == false) && (sensor_front_right2 == false) && (front == true)) {
        setSpeed(lspeedNew,rspeedNew);
        }
        else if ((sensor_back_left == false) && (sensor_back_right == false) && (back == true)) {
        setSpeed(lspeedNew,rspeedNew);
        }
        else {setSpeed(0, 0);}
        }
        else {setSpeed(lspeedNew,rspeedNew);}
        return true;
}

void proxCallbackFrontleft1(const sensor_msgs::Range::ConstPtr &value){
    ROS_INFO("got value from prox sensors_front_left1: %f", value->range);
    if(value->range > 250){
       sensor_front_left1 = true;
       ROS_INFO("can't move forward");
    }
    else{sensor_front_left1 = false;} 
    return;
}
void proxCallbackFrontleft2(const sensor_msgs::Range::ConstPtr &value){
    ROS_INFO("got value from prox sensors_front_left2: %f", value->range);
    if(value->range > 250){
       sensor_front_left2 = true;
       ROS_INFO("can't move forward");
    }
    else{sensor_front_left2 = false;} 
    
    return;
}
void proxCallbackFrontmiddle(const sensor_msgs::Range::ConstPtr &value){
    ROS_INFO("got value from prox sensors_front_middle: %f", value->range);
    if(value->range > 250){
       sensor_front_middle = true;
       ROS_INFO("can't move forward");
    }else{
      sensor_front_middle = false; 
    }
    return;
}
void proxCallbackFrontright2(const sensor_msgs::Range::ConstPtr &value){
    ROS_INFO("got value from prox sensors_front_right2: %f", value->range);
    if(value->range > 250){
       sensor_front_right1 = true;
       ROS_INFO("can't move forward");
    }else{
      sensor_front_right1 = false; 
    }
    return;
}
void proxCallbackFrontright1(const sensor_msgs::Range::ConstPtr &value){
    ROS_INFO("got value from prox sensors_front_right1: %f", value->range);
    if(value->range > 250){
       sensor_front_right2 = true;
       ROS_INFO("can't move forward");
    }else{
      sensor_front_right2 = false; 
    }
    return;
}
void proxCallbackBackleft(const sensor_msgs::Range::ConstPtr &value){
     ROS_INFO("got value from prox sensors_front: %f", value->range);
    if(value->range > 250){
       sensor_back_left = true;
       ROS_INFO("can't move forward");
    }else{
      sensor_back_left = false; 
    }
     return;
}
void proxCallbackBackright(const sensor_msgs::Range::ConstPtr &value){
     ROS_INFO("got value from prox sensors_front: %f", value->range);
    if(value->range > 250){
       sensor_back_right = true;
       ROS_INFO("can't move forward");
    }else{
      sensor_back_right = false; 
    }
     return;
}


int main(int argc, char **argv) {
    signal(SIGINT, quit);

    ros::init(argc, argv, "guard_thymio", ros::init_options::AnonymousName);
    ros::NodeHandle n;
    
    ros::ServiceServer service = n.advertiseService("SetVelocityGuard", VelocityCallback);
    

    leftVelocityClient = n.serviceClient<webots_ros::set_float>("thymioII/motor_left/set_velocity");
    rightVelocityClient = n.serviceClient<webots_ros::set_float>("thymioII/motor_right/set_velocity");
    

    ros::Subscriber subs_prox[7];
    std::string topicName0 = "thymioII/prox_horizontal_0/value";
    subs_prox[0] = n.subscribe(topicName0, 1, proxCallbackFrontleft1);
    
    std::string topicName1 = "thymioII/prox_horizontal_1/value";
    subs_prox[1] = n.subscribe(topicName1, 1, proxCallbackFrontleft2);
    
    std::string topicName2 = "thymioII/prox_horizontal_2/value";
    subs_prox[2] = n.subscribe(topicName2, 1, proxCallbackFrontmiddle);
    
    std::string topicName3 = "thymioII/prox_horizontal_3/value";
    subs_prox[3] = n.subscribe(topicName3, 1, proxCallbackFrontright2);
    
    std::string topicName4 = "thymioII/prox_horizontal_4/value";
    subs_prox[4] = n.subscribe(topicName4, 1, proxCallbackFrontright1);
    
    std::string topicName5 = "thymioII/prox_horizontal_5/value";
    subs_prox[5] = n.subscribe(topicName5, 1, proxCallbackBackleft);
    
    std::string topicName6 = "thymioII/prox_horizontal_6/value";
    subs_prox[6] = n.subscribe(topicName6, 1, proxCallbackBackright);
    

    while(ros::ok()) {

        if(subs_prox[0].getNumPublishers() == 0) {
            bool success = true;
            for (int i=0; i<=6; i++) {
                std::string serviceName = "thymioII/prox_horizontal_" + std::to_string(i) +"/enable";
                ros::ServiceClient serviceClient = n.serviceClient<webots_ros::set_int>(serviceName);
                webots_ros::set_int clientSrv;
                clientSrv.request.value = 1;
                serviceClient.call(clientSrv);
                success = success && clientSrv.response.success; 
            }
            if(success) {
                ROS_INFO ("prox sensor successfully enabled!");
        
            } else {
                ROS_INFO ("prox sensor could not be enabled!"); 
            }
         
            ros::Duration(0.5).sleep();
        }
        ros::spinOnce();
        ros::Duration(0.05).sleep();
    }
    quit(0);
}
