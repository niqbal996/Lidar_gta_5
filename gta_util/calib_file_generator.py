from os import path
import glob
import os
import shutil

data_set_path = r'E:\GTA_sequence\fixed_dataset_raycast\set7\training'
example_calib_file = 'E:\\GTA_sequence\\000000.txt'
example_image_file = 'E:\\GTA_sequence\\000000.png'
#Preprocessed DATA OUTPUT DIRECTORY
# output_directory = r''
output_directory = os.path.join(data_set_path)
output_calib_dir = os.path.join(output_directory, "calib")
output_image_dir = os.path.join(output_directory, "image_2")
output_point_cloud_dir = os.path.join(output_directory, "velodyne")

os.makedirs(output_calib_dir, exist_ok=True)
os.makedirs(output_image_dir, exist_ok=True)

if path.exists(output_point_cloud_dir):
    point_cloud_files = sorted(glob.glob(os.path.join(output_point_cloud_dir, "*.bin")))
    if os.path.isfile(example_calib_file):
        print('[INFO] Sample file found. Continuing. . .')
        for file_number in range(len(point_cloud_files)):
            basename = os.path.basename(point_cloud_files[file_number])[:-4]
            output_filename = (output_calib_dir + '\\' +basename + '.txt')
            output_image_filename = (output_image_dir + '\\' +basename + '.png')
            shutil.copy2(example_calib_file, output_filename)
            shutil.copy2(example_image_file, output_image_filename)
    else:
        print('[INFO] No Sample file found. Exiting!')


