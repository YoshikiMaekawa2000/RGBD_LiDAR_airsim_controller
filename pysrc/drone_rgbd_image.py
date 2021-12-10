import cv2
import PIL.Image as Image
import math
import numpy as np
import time
import argparse
from numpy.core.fromnumeric import argmin
import yaml
import os
import csv
import random
import itertools
import matplotlib.pyplot as plt
import matplotlib.cm as cm
import random

import airsim

class DroneRGBDImage:
    def __init__(self,
        save_data_top_path,
        rgb_image_directory,
        depth_image_directory,
        lidar_data_directory,
        save_csv_file_name,
        place_csv_root_path,
        place_csv_name,
        num_total,
        wait_time_milisec,
        color_checker,
        roll_max,
        roll_min,
        pitch_max,
        pitch_min,
        yaw_max,
        yaw_min,
        resolution):

        self.save_data_top_path = save_data_top_path
        self.rgb_image_directory = rgb_image_directory
        self.depth_image_directory = depth_image_directory
        self.lidar_data_directory = lidar_data_directory
        self.save_csv_file_name = save_csv_file_name
        self.place_csv_root_path = place_csv_root_path
        self.place_csv_name = place_csv_name
        self.num_total = num_total
        self.wait_time_milisec = wait_time_milisec
        self.color_checker = color_checker
        self.roll_max = roll_max
        self.roll_min = roll_min
        self.pitch_max = pitch_max
        self.pitch_min = pitch_min
        self.yaw_max = yaw_max
        self.yaw_min = yaw_min
        self.resolution = resolution

        print("Starting Drone RGBD Image")

        self.client = airsim.MultirotorClient() #Get Drone Client
        self.client.confirmConnection()
        self.client.enableApiControl(True)
        self.client.armDisarm(True)

        self.client.reset()

        self.list_camera = ["camera_rgb", "camera_depth"]
        self.csv_data = []
        self.csv_data_size = 0

    def spin(self):
        csv_checker = bool(self.load_csv())

        if(csv_checker==False):
            print("ERROR LOADING CSV FILE")
            exit(1)

        self.save_data()

    def load_csv(self):
        checker = True
        place_csv_path = os.path.join(self.place_csv_root_path, self.place_csv_name)
        with open(place_csv_path) as f:
            reader = csv.reader(f)
            for row in reader:

                row_float = []
                for value in row:
                    row_float.append(float(value))
                
                self.csv_data.append(row_float)

        self.csv_data_size = len(self.csv_data)

        return checker

    def save_data(self):
        print("Saving RGB-D image")
        num = 0

        while num < self.num_total:
            place_index = random.randint(0, self.csv_data_size)
            tmp_place_data = self.csv_data[place_index]

            responses = self.client.simGetImages([
            # png format
            airsim.ImageRequest(0, airsim.ImageType.Scene), 
            # uncompressed RGB array bytes
            airsim.ImageRequest(1, airsim.ImageType.Scene, False, False),
            # floating point uncompressed image
            airsim.ImageRequest(1, airsim.ImageType.DepthPlanar, True)])





if __name__ == '__main__':
    parser = argparse.ArgumentParser("./drone_rgbd_image.py")

    parser.add_argument(
        '--config', '-c',
        type=str,
        required=False,
        default='../pyyaml/drone_rgbd_image.yaml',
    )

    FLAGS, unparsed = parser.parse_known_args()

    #Load yaml file
    try:
        print("Opening train config file %s", FLAGS.config)
        CFG = yaml.safe_load(open(FLAGS.config, 'r'))
    except Exception as e:
        print(e)
        print("Error opening train config file %s", FLAGS.config)
        quit()

    save_data_top_path = CFG["save_data_top_path"]
    rgb_image_directory = CFG["rgb_image_directory"]
    depth_image_directory = CFG["depth_image_directory"]
    lidar_data_directory = CFG["lidar_data_directory"]
    save_csv_file_name = CFG["save_csv_file_name"]

    place_csv_root_path = CFG["place_csv_root_path"]
    place_csv_name = CFG["place_csv_name"]

    num_total = int(CFG["num_total"])
    wait_time_milisec = int(CFG["wait_time_milisec"])
    color_checker = bool(CFG["color_checker"])

    roll_max = float(CFG["roll_max"]) * 3.141592/180.0
    roll_min = float(CFG["roll_min"]) * 3.141592/180.0
    pitch_max = float(CFG["pitch_max"]) * 3.141592/180.0
    pitch_min = float(CFG["pitch_min"]) * 3.141592/180.0
    yaw_max = float(CFG["yaw_max"]) * 3.141592/180.0
    yaw_min = float(CFG["yaw_min"]) * 3.141592/180.0
    resolution = float(CFG["resolution"])

    drone_rgbd_image = DroneRGBDImage(
        save_data_top_path,
        rgb_image_directory,
        depth_image_directory,
        lidar_data_directory,
        save_csv_file_name,
        place_csv_root_path,
        place_csv_name,
        num_total,
        wait_time_milisec,
        color_checker,
        roll_max,
        roll_min,
        pitch_max,
        pitch_min,
        yaw_max,
        yaw_min,
        resolution
    )

    drone_rgbd_image.spin()
