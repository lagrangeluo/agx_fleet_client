#!/bin/bash
#version: 0.1
#Author: Junyu Luo(Lagrange.L)

_GREEN='\e[32m'
_NORMAL='\e[0m'
_BOLD='\e[33m'
_RED='\e[31m'
_Yellow="\033[0;36m"
Blue="\033[0;34m"         # Blue
Purple="\033[0;35m"       # Purple
Red="\033[0;31m"          # Red
CHOOSE=1
VERSION=1

function PRINT_MENU()
{
    echo -e "${_BOLD}--------------------------${_NORMAL}"
    echo -e "${_Yellow} 1.Auto start (Recommend)${_NORMAL}"
    echo -e "${Blue} 2.Build image${_NORMAL}"
    echo -e "${Purple} 3.Start Container${_NORMAL}"
    echo -e "${Red} 4.Delete Container${_NORMAL}"
    echo -e "${_GREEN} 5.Backup environment${_NORMAL}"
    echo -e "${_GREEN} 6.Restore environment${_NORMAL}"
    echo -e "${_BOLD}--------------------------${_NORMAL}"
    echo -n "Your chose(1-6):"
}


function BUILD_IMAGE() {
    Docker_file=Dockerfile
    docker build --file $Docker_file --tag $image_tag .
    echo -e "${_GREEN} image agx_fleet_client build success!${_NORMAL}"
}

function start_image()
{
    docker run \
        -it \
        --network host \
        --privileged \
        --name agx_fleet_client \
        --device-cgroup-rule="a *:* rmw" \
        -v /dev:/dev \
        -v /home/agilex/agx_fleet_client_ws:/home/agx_fleet_client_ws \
	-v /home/agilex/freego_ws:/home/freego_ws \
        --volume=/tmp/.X11-unix:/tmp/.X11-unix \
        -e DISPLAY=${DISPLAY} \
        $image_tag

    echo -e "${_GREEN} Container agx_fleet_client start success!${_NORMAL}"
    echo -e "${_GREEN} Now you can now connect to the container via SSH by using 'ssh -p 10022 root@ip' the password is 'agx'${_NORMAL}"
}

PRINT_MENU

read CHOOSE

    echo -e "${_Yellow} chosen get! ${_NORMAL}"
    echo -e "${_BOLD}-------------------${_NORMAL}"
    echo -e "${_GREEN} chose your version ${_NORMAL}"
    echo -e "${_Yellow} 1.x86_64 amd64${_NORMAL}"
    echo -e "${Blue} 2.arm64 ${_NORMAL}"
    echo -n "Your chose(1-2):"

read VERSION

case "${VERSION}" in
    1)
    image_tag=agx_fleet_client:v1
    ;;
    2)
    image_tag=agx_fleet_client:v2
    ;;
esac

case "${CHOOSE}" in
    1)
    BUILD_IMAGE
    start_image
    ;;
    2)
    BUILD_IMAGE
    ;;
    3)
    start_image
    ;;
    4)
    #docker rm -f limo_dev
    ;;
    5)
    #backup_container
    ;;
    6)
    #restore_image
    ;;

esac
