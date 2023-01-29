#!/bin/bash

rostopic pub /limo_1_node/battery_state sensor_msgs/BatteryState -r 5 -f battery.yaml