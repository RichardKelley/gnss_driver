#!/usr/bin/env bash

###############################################################################
# Copyright 2017 Board of Regents of the
# Nevada System of Higher Education, on behalf of the University of
# Nevada, Reno. All Rights Reserved.
# 
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
###############################################################################

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
