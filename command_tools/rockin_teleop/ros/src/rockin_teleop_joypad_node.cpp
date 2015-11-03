/*
 * teleop_joypad_node.cpp
 *
 *  Created on: Dec 15, 2011
 *      Author: Frederik Hegger
 */

#include "rockin_teleop/rockin_teleop_joypad.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "rockin_teleop_joypad");
    ros::NodeHandle nh("~");

    TeleOpJoypad* teleop = new TeleOpJoypad(nh);

    ros::Rate loop_rate(50);

    ros::spin();

    delete teleop;
}

