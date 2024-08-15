#! /usr/bin/env python3 
# -*- coding: utf-8 -*-
import os
import sys
import shutil
import rospy
import rospkg
import subprocess

class SaveMap:
    def __init__(self):
        rospy.init_node(os.path.basename(__file__).split(".")[0])
        rospack = rospkg.RosPack()
        pkg_dir_path = rospack.get_path("navigation_start")
        self.maps_dir_path = os.path.join(pkg_dir_path, "maps", "maps")
        self.prohibition_maps_dir_path = os.path.join(pkg_dir_path, "maps", "prohibition_maps")
        self.ros_maps_dir_path = os.path.expanduser("~/.ros/map")
        
        self.p_map_name = rospy.get_param("~map_name", None)
        
        self.run()
    
    def run(self):
        if self.p_map_name is None:
            print("\033[31m" + "The parameter map_name is not set. _map_name:=<map name>" + "\033[0m")
            sys.exit(1)
        
        # Check if the map name already exists in either maps directory
        if os.path.exists(os.path.join(self.maps_dir_path, self.p_map_name)) or os.path.exists(os.path.join(self.prohibition_maps_dir_path, self.p_map_name)):
            print("\033[31m" + f"Error: Map name '{self.p_map_name}' already exists." + "\033[0m")
            sys.exit(1)
        
        # Create the directories if they don't exist and save the map
        os.makedirs(os.path.join(self.maps_dir_path, self.p_map_name), exist_ok=True)
        result = subprocess.run(["rosrun", "map_server", "map_saver", "-f", os.path.join(self.maps_dir_path, self.p_map_name, 'map')], check=True, text=True)
        if result.stdout or result.stderr:
            print("\033[31m" + "Error stdout: " + str(result.stdout) + "\033[0m")
            print("\033[31m" + "Error stderr: " + str(result.stderr) + "\033[0m")
        
        if os.path.exists(os.path.join(self.maps_dir_path, self.p_map_name, "map.yaml")) and os.path.exists(os.path.join(self.maps_dir_path, self.p_map_name, "map.pgm")):
            os.makedirs(os.path.join(self.prohibition_maps_dir_path, self.p_map_name), exist_ok=True)
            shutil.copy(os.path.join(self.maps_dir_path, self.p_map_name, "map.yaml"), os.path.join(self.prohibition_maps_dir_path, self.p_map_name))
            shutil.copy(os.path.join(self.maps_dir_path, self.p_map_name, "map.pgm"), os.path.join(self.prohibition_maps_dir_path, self.p_map_name))
        else:
            print("\033[31m" + "Error: map.yaml or map.pgm not found." + "\033[0m")
            sys.exit(1)
        
        # Check if the ~/.ros/map directory exists, if not, create it
        if not os.path.exists(self.ros_maps_dir_path):
            os.makedirs(self.ros_maps_dir_path, exist_ok=True)

        # Copy the map files directly to ~/.ros/map/ (overwrite if exists)
        shutil.copy(os.path.join(self.maps_dir_path, self.p_map_name, "map.yaml"), os.path.join(self.ros_maps_dir_path, "map.yaml"))
        shutil.copy(os.path.join(self.maps_dir_path, self.p_map_name, "map.pgm"), os.path.join(self.ros_maps_dir_path, "map.pgm"))
        
        # Verify the files in both the prohibition maps directory and ~/.ros/map
        if os.path.exists(os.path.join(self.prohibition_maps_dir_path, self.p_map_name, "map.yaml")) and os.path.exists(os.path.join(self.prohibition_maps_dir_path, self.p_map_name, "map.pgm")):
            if os.path.exists(os.path.join(self.ros_maps_dir_path, "map.yaml")) and os.path.exists(os.path.join(self.ros_maps_dir_path, "map.pgm")):
                print("\033[34m" + "Successfully saved the map!!" + "\033[0m")
                sys.exit(0)
            else:
                print("\033[31m" + "Failed to copy map.yaml and map.pgm to ~/.ros/map" + "\033[0m")
                sys.exit(1)
        else:
            print("\033[31m" + "Failed to copy map.yaml and map.pgm to the prohibition maps directory" + "\033[0m")
            sys.exit(1)

def main():
    save_map = SaveMap()
    save_map.run()

if __name__ == "__main__":
    main()

