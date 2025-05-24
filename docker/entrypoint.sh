#!/bin/bash


busybox devmem 0x0c303000 32 0x0000C400
busybox devmem 0x0c303008 32 0x0000C458
busybox devmem 0x0c303010 32 0x0000C400
busybox devmem 0x0c303018 32 0x0000C458

modprobe can
modprobe can_raw
modprobe mttcan

ip link set down can0
ip link set down can1

ip link set can0 type can bitrate 1000000 
ip link set can1 type can bitrate 1000000 

ip link set up can0
ip link set up can1

if [[ $1 == "autostart" ]]
then
	ros2 launch controller

	bash

else
    alias controller="ros2 launch controller controller.launch.py"
	bash
fi

