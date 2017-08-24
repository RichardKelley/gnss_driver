#!/usr/bin/env bash

if pidof -o %PPID -x "roscore" > /dev/null; then

    if rosnode list | grep stream_nodelet > /dev/null; then
	rosnode kill /stream_nodelet
    fi

    if rosnode list | grep parser_nodelet > /dev/null; then
	rosnode kill /parser_nodelet
    fi

    if rosnode list | grep gnss_nodelet_manager > /dev/null; then
	rosnode kill /gnss_nodelet_manager
    fi

fi

roslaunch gnss_driver gnss_driver.launch
