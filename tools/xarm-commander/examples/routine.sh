#!/bin/bash

XARM_IP=$1
XARM_COMMANDER="bin/xarm-commander"

echo "Start xarm routine"
${XARM_COMMANDER} -v -i ${XARM_IP} motion_enable
${XARM_COMMANDER} -v -i ${XARM_IP} set_mode
${XARM_COMMANDER} -v -i ${XARM_IP} get_version
${XARM_COMMANDER} -v -i ${XARM_IP} set_state
${XARM_COMMANDER} -v -i ${XARM_IP} get_state
${XARM_COMMANDER} -v -i ${XARM_IP} get_position

# Motion!
${XARM_COMMANDER} -v -i ${XARM_IP} set_position -y 200
${XARM_COMMANDER} -v -i ${XARM_IP} set_position -y -200
${XARM_COMMANDER} -v -i ${XARM_IP} set_position -y 0
