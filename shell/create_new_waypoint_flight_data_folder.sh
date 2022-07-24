#!/bin/bash

root_dir="/media/amsl/96fde31e-3b9b-4160-8d8a-a4b913579ca21/flight_airsim_image"
deg0="degree_0"
deg5="degree_5"
deg10="degree_10"

deg0_path=$root_dir"/"$1"/"$deg0"/camera_image"
deg5_path=$root_dir"/"$1"/"$deg5"/camera_image"
deg10_path=$root_dir"/"$1"/"$deg10"/camera_image"

mkdir -p $deg0_path
mkdir -p $deg5_path
mkdir -p $deg10_path
