import numpy as np
from os import path
import glob
import os

data_set_path = 'C:\\Users\\zj7xsg\\git\\KITTI\\GTA_sequence\\set1\\GTA_sequence_1'

input_directory = data_set_path
point_cloud_dir = os.path.join(input_directory, "velodyne")
calibration_dir = os.path.join(input_directory, "calibration")
ground_truth_dir = os.path.join(input_directory, "label_2")

if(path.exists(point_cloud_dir) and path.exists(ground_truth_dir)):
    ground_truth_files = sorted(glob.glob(os.path.join(ground_truth_dir, "*.txt")))
    point_cloud_files = sorted(glob.glob(os.path.join(point_cloud_dir, "*.bin")))
    assert(len(point_cloud_files) == len(ground_truth_files))
    filename = '001_sequence.txt'  # e.g. NOTE this is sequence number
    filename = os.path.join(data_set_path, filename)
    sequence_data = []
    for file_number in range(len(ground_truth_files)):
        current_ground_truth_filename = os.path.splitext(os.path.basename(ground_truth_files[file_number]))[0]
        current_point_cloud_filename = os.path.splitext(os.path.basename(point_cloud_files[file_number]))[0]
        assert current_ground_truth_filename == current_point_cloud_filename
        for line in open(ground_truth_files[file_number], 'r').readlines():
            line = str(file_number) + " " + line
            sequence_data.append(line)

    with open(filename, 'w') as file_handler:
        for entry in sequence_data:
            file_handler.write("{}".format(entry))
    print('[INFO] Sequence ground truth file has been generated.')
else:
    print("[INFO] The given dataset does not exist. Please check the paths provided. . .")

print('[INFO] Exiting now!')