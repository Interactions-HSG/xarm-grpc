#!/bin/bash

echo "Start xarm routine"
xarm-commander -v -i 130.82.171.9 motion_enable
xarm-commander -v -i 130.82.171.9 set_mode
xarm-commander -v -i 130.82.171.9 get_version
xarm-commander -v -i 130.82.171.9 set_state
xarm-commander -v -i 130.82.171.9 get_state
xarm-commander -v -i 130.82.171.9 get_position

# Motion!
xarm-commander -v -i 130.82.171.9 set_position -y 200
xarm-commander -v -i 130.82.171.9 set_position -y -200
xarm-commander -v -i 130.82.171.9 set_position -y 0
