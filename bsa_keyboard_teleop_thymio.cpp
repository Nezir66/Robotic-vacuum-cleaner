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

#include <webots_ros/Int32Stamped.h>
#include <webots_ros/set_float.h>
#include <webots_ros/set_int.h>
#include <webots_ros/SetVelocityGuard.h>

#include <std_msgs/String.h>

#include <signal.h>
#include <stdio.h>

#define TIME_STEP 32

static double lspeed = 0;
static double rspeed = 0;
static double lspeedNew = 0;
static double rspeedNew = 0;
static double speed = 5;
double lastSpeedSetTime = 0;


ros::ServiceClient leftWheelClient, rightWheelClient;
webots_ros::set_float leftWheelSrv, rightWheelSrv;

ros::ServiceClient enableKeyboardClient;
webots_ros::set_int enableKeyboardSrv;

ros::ServiceClient client;

webots_ros::SetVelocityGuard srv;

void initMovement() {
    leftWheelSrv.request.value = INFINITY;
    rightWheelSrv.request.value = INFINITY;
    if (!leftWheelClient.call(leftWheelSrv) || !rightWheelClient.call(rightWheelSrv) || !leftWheelSrv.response.success ||
        !rightWheelSrv.response.success) {
        ROS_ERROR("Failed to send init movement commands to the robot.");
    }
}

void setSpeed(double lspeedNew, double rspeedNew) {
    lastSpeedSetTime = ros::Time::now().toSec();
    if(lspeedNew == lspeed && rspeedNew == rspeed) {
        return;
    }

    initMovement();
    lspeed = lspeedNew;
    rspeed = rspeedNew;
    srv.request.left = lspeed;
    srv.request.right = rspeed;
    client.call(srv);
}

void quit(int sig) {
  enableKeyboardSrv.request.value = 0;
  enableKeyboardClient.call(enableKeyboardSrv);
  ROS_INFO("User stopped the 'keyboard_teleop' node.");
  ros::shutdown();
  exit(sig);
}

/**
* inits movement by setting target position for both wheels to infinity. Movement is controlled by setting wheel speed afterwards.
* also see: https://cyberbotics.com/doc/reference/motor#velocity-control
*/

void keyboardCallback(const webots_ros::Int32Stamped::ConstPtr &value) {
  int high=1;
  int low=-1;
  int key = value->data;
  
  switch (key) {
    case 69: //EinAus
      srv.request.EinAus = false;
      break;
    case 65: //EinAus
      srv.request.EinAus = true;
      break;
    case 314: //left
      lspeedNew = -speed;
      rspeedNew = speed;
      srv.request.front = true;
      srv.request.back = false;
      break;
    case 316: //right
      lspeedNew = speed;
      rspeedNew = -speed;
      srv.request.front = true;
      srv.request.back = false;
      break;
    case 315: //forward
      lspeedNew = speed;
      rspeedNew = speed;
      srv.request.front = true;
      srv.request.back = false;
      break;
    case 317: //back
      lspeedNew = -speed;
      rspeedNew = -speed;
      srv.request.front = false;
      srv.request.back = true;
      break;
    case 312:
      ROS_INFO("END.");
      quit(0);
      break;
    case 32: // Space - Stop
      lspeedNew = 0;
      rspeedNew = 0;
      break;
    //TODO: add speed control with +/- keys
    case 43: // + speed up
      speed = speed+high;
      if (speed == 9) {
        speed = speed - high;
      }
      break;
    case 45: // - lower speed
      speed = speed + low;
      if (speed == 0) {
        speed = speed + high;
      }
      break;
    default:
        ROS_INFO("pressed unused key: %d", key);
        break;
  }

  setSpeed(lspeedNew, rspeedNew);
  return;
  
}


int main(int argc, char **argv) {
    ros::init(argc, argv, "keyboard_teleop_thymio", ros::init_options::AnonymousName);
    ros::NodeHandle n;
    client = n.serviceClient<webots_ros::SetVelocityGuard>("SetVelocityGuard");
    signal(SIGINT, quit);
    
    leftWheelClient = n.serviceClient<webots_ros::set_float>("thymioII/motor_left/set_position");
    rightWheelClient = n.serviceClient<webots_ros::set_float>("thymioII/motor_right/set_position");
    
    ros::ServiceClient client = n.serviceClient<webots_ros::SetVelocityGuard>("SetVelocityGuard");
    webots_ros::SetVelocityGuard srv;
    srv.request.left = lspeedNew;
    srv.request.right = rspeedNew;
    client.call(srv);


    enableKeyboardClient = n.serviceClient<webots_ros::set_int>("thymioII/keyboard/enable");
    enableKeyboardSrv.request.value = TIME_STEP;
    
    ros::Subscriber sub_keyboard;
    sub_keyboard = n.subscribe("thymioII/keyboard/key", 1, keyboardCallback);
    

    while(ros::ok()) {
        if (sub_keyboard.getNumPublishers() == 0) {
            rspeedNew = 0;
            lspeedNew = 0;
            if (enableKeyboardClient.call(enableKeyboardSrv) && enableKeyboardSrv.response.success) {
                ROS_INFO("Keyboard enabled.");
                ROS_INFO("Use the arrows in Webots window to move the robot.");
                ROS_INFO("Press the End key to stop the node.");
            } else {
                ROS_ERROR("Could not enable keyboard!");
            }
            ros::Duration(0.5).sleep();
        }
        ros::spinOnce();
        if (ros::Time::now().toSec() - lastSpeedSetTime > 0.1) {
            setSpeed(0, 0);
        }
        ros::Duration(0.05).sleep();
    }

    quit(0);
}

