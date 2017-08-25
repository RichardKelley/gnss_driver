#!/usr/bin/env bash

PB_DIR=../proto

protoc -I=$PB_DIR $PB_DIR/error_code.proto --cpp_out=$PB_DIR
protoc -I=$PB_DIR $PB_DIR/header.proto --cpp_out=$PB_DIR
protoc -I=$PB_DIR $PB_DIR/geometry.proto --cpp_out=$PB_DIR
protoc -I=$PB_DIR $PB_DIR/pose.proto --cpp_out=$PB_DIR
protoc -I=$PB_DIR $PB_DIR/gps.proto --cpp_out=$PB_DIR
protoc -I=$PB_DIR $PB_DIR/imu.proto --cpp_out=$PB_DIR
protoc -I=$PB_DIR $PB_DIR/gnss_status.proto --cpp_out=$PB_DIR
protoc -I=$PB_DIR $PB_DIR/config.proto --cpp_out=$PB_DIR
protoc -I=$PB_DIR $PB_DIR/gpgga.proto --cpp_out=$PB_DIR
protoc -I=$PB_DIR $PB_DIR/imu.proto --cpp_out=$PB_DIR
protoc -I=$PB_DIR $PB_DIR/ins.proto --cpp_out=$PB_DIR
protoc -I=$PB_DIR $PB_DIR/gnss.proto --cpp_out=$PB_DIR
