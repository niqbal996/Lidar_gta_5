import numpy as np
from matplotlib import pyplot
from mpl_toolkits.mplot3d import Axes3D
from os import path
import glob
import os
import math
import pickle

data_set_path = r'E:\GTA_sequence\fixed_dataset_raycast\set7'

#Preprocessed DATA OUTPUT DIRECTORY
output_directory = r'E:\GTA_sequence\fixed_dataset_raycast\set7\sequence'
output_directory = os.path.join(data_set_path, output_directory)
output_point_cloud_dir = os.path.join(output_directory, "velodyne")
output_ground_truth_dir = os.path.join(output_directory, "labels")
output_ground_truth_kitti_dir = os.path.join(output_directory, "label_2")
os.makedirs(output_point_cloud_dir, exist_ok=True)
os.makedirs(output_ground_truth_dir, exist_ok=True)
os.makedirs(output_ground_truth_kitti_dir, exist_ok=True)

#Source directory
point_cloud_dir = os.path.join(data_set_path, "velodyne")
ground_truth_kitti_dir = os.path.join(data_set_path, "kitti_labels")
points_counter = 0

number_of_values_per_line = 5
bounding_box_vertices = np.zeros((8, 3))     #Spatial coordinates for Eight points for each cuboid(x,y,z).


def lidar_2_cam(vec1):
    x, y, z = float(vec1[0]), float(vec1[1]), float(vec1[2])
    R_xyz = np.array([[0.0, -1.0, 0.0],
                      [0.0, 0.0, -1.0],
                      [1.0, 0.0, 0.0]])

    x1 = np.transpose([[x, 0, 0]])
    y1 = np.transpose([[0, y, 0]])
    z1 = np.transpose([[0, 0, z]])
    x1 = R_xyz.dot(x1)
    y1 = R_xyz.dot(y1)
    z1 = R_xyz.dot(z1)
    return float(y1[0]), float(z1[1]), float(x1[2])

def dotproduct(v1, v2):
  return sum((a*b) for a, b in zip(v1, v2))

def length(v):
  return math.sqrt(dotproduct(v, v))

def angle(v1, v2):
  return math.acos(dotproduct(v1, v2) / (length(v1) * length(v2)))

def angle_tan(a, b):
    sine_theta = np.linalg.norm(np.cross(a, b))
    cos_theta = np.dot(a, b)
    tan_theta = sine_theta / cos_theta

    angle = math.atan(tan_theta)
    return angle

def rot_a2b(vec1, vec2):
    '''Rotates the given vector1 to align with vector2 but only in the xy-plane

    :argument vec1 a numpy array containing 3 dimensional values
            vec2 a numpy array containing 3 dimensional values
    :return 3D vector1 rotated parallel to vec2 in xy-plane
    '''

    x1, y1, z1 = vec1[0], vec1[1], vec1[2]
    x2, y2, z2 = vec2[0], vec2[1], vec2[2]
    print('Angle between vectors is: ', angle(vec1, vec2) * 180 / 3.14)
    vec1_ = vec1[0:2]
    G = np.array([[(x1*x2+y1*y2), -(x1*y2 - x2*y1)],
                  [(x1*y2 - x2*y1), (x1*x2+y1*y2)]])

    rotated_a2D = np.matmul(G, vec1_)
    return np.append(rotated_a2D, vec1[2])


def rot_angle(point, vec2):
    vec1 = np.array([1, 0, 0])      #positive x axis unit vector
    theta = angle(vec1, vec2)

    R = np.array(((np.cos(theta), -np.sin(theta), 0),
                  (np.sin(theta), np.cos(theta), 0),
                  (0, 0, 1)))

    vec1 = R.dot(point)
    vec1[0] = float("{0:.4f}".format(vec1[0]))
    vec1[1] = float("{0:.4f}".format(vec1[1]))
    vec1[2] = float("{0:.4f}".format(vec1[2]))
    return vec1

def rot_angle_2(point, vec2):
    vec1 = point      # position vector of given point
    theta = angle(vec1, vec2)

    R = np.array(((np.cos(theta), -np.sin(theta), 0),
                  (np.sin(theta), np.cos(theta), 0),
                  (0, 0, 1)))

    vec1 = R.dot(point)
    vec1[0] = float("{0:.4f}".format(vec1[0]))
    vec1[1] = float("{0:.4f}".format(vec1[1]))
    vec1[2] = float("{0:.4f}".format(vec1[2]))
    return vec1

if path.exists(point_cloud_dir) and  path.exists(ground_truth_kitti_dir):

    point_cloud_files = sorted(glob.glob(os.path.join(point_cloud_dir, "*.txt")))
    ground_truth_kitti_files = sorted(glob.glob(os.path.join(ground_truth_kitti_dir, "*.txt")))
    assert(len(point_cloud_files) == len(ground_truth_kitti_files))

    for file_number in range(82, len(point_cloud_files)):
        current_point_cloud_file = os.path.join(point_cloud_dir, point_cloud_files[file_number])
        current_ground_truth_kitti_file = os.path.join(ground_truth_kitti_dir, ground_truth_kitti_files[file_number])

        #For file smaller than 1KB , ignore. These are empty files
        if os.stat(current_ground_truth_kitti_file).st_size < 1000:
            continue
        else:
            point_cloud_array =[]
            ground_truth_array = []
            ground_truth_kitti_array = []
            #Process the point cloud data first
            for line in open(current_point_cloud_file, 'r').readlines():
                # Delete the empty entries from the array, non-hit points from the ray cast
                if list(map(float, line.split(" "))) == [0.0, 0.0, 0.0, 0] or \
                   np.abs(list(map(float, line.split(" ")))[0]) > 120.0:                #When the labels are  0 0 0 the values skyrocket to -300 something
                    points_counter += 1
                    continue
                else:
                    x =list(map(float, line.split(" ")))[0:4]
                    point_cloud_array.append(list(map(float, line.split(" ")))[0:4])

            #print("[INFO] " , str(points_counter), " number of points have been removed.")
            points_counter_2 = 0 #reset
            point_cloud_array = np.asarray(point_cloud_array, dtype='float32')

            #Process the kitti labels file now.
            lines = open(current_ground_truth_kitti_file, 'r').readlines()
            for line in lines:
                current_entry = list(map(str, line.split(" ")))
                float_list = np.abs(np.array(current_entry[1:], dtype=float))
                # condition = (any((float_list) > 120.0)) ==True
                if(float(current_entry[2]) > 1):              #Must be 0 or 1. anything greater than that, discard it.
                    continue
                elif((any(np.abs(float_list) > 120.0)) ==True):
                    continue
                else:
                    ground_truth_kitti_array.append(current_entry)

            # Same naming convention as KITTI
            ground_truth_kitti_array = np.asarray(ground_truth_kitti_array)
            unique_examples = []
            if len(ground_truth_kitti_array) == 0:
                print("[INFO] Skipped a bad file  {}".format(str(os.path.basename(point_cloud_files[file_number]))))
                continue
            for entry in range(len(np.unique(ground_truth_kitti_array[:, 11]))):
                x_1 = np.where(ground_truth_kitti_array[:, 11] == np.unique(ground_truth_kitti_array[:,11])[entry] )
                if len(x_1[0]) > 1:
                    unique_examples.append(x_1[0][1])  #need only one entry of this unique ex_1ample
                else:
                    unique_examples.append(x_1[0][0])    # only 1 dimensional data.
            ground_truth_kitti_array = ground_truth_kitti_array[unique_examples, :]

            for entry in range(ground_truth_kitti_array.shape[0]): # number of labels in the current sample
                current_entry = ground_truth_kitti_array[entry, :]
                # lidar to camera coordinates transformation
                # b = current_entry[11], current_entry[12], current_entry[13]
                ego_forward_vector = (ground_truth_kitti_array[0, :][22:25]).astype(np.float32)
                current_entry[11:14] = rot_angle(current_entry[11:14].astype(np.float32), ego_forward_vector)
                current_entry[11:14] = lidar_2_cam(current_entry[11:14])

                a = current_entry[11], current_entry[12], current_entry[13]
                # TODO positive y axis needs 180 counter clock rotation
                # Angle ry w.r.t y axis
                try:
                    current_entry[14] = angle(np.array([current_entry[16],  # labelled vehicle forward vector
                                                        current_entry[17],
                                                        current_entry[18]]).astype(np.float16),
                                              np.array([current_entry[19],  # ego vehicle right vector
                                                        current_entry[20],
                                                        current_entry[21]]).astype(np.float16))
                except:
                    current_entry[14] = 0.0
            # # Forward direction vector of host vehicle
            ego_forward_vector = (ground_truth_kitti_array[0, :][22:25]).astype(np.float32)
            mag_ego_forward_vector = math.sqrt(ego_forward_vector[0]**2 + ego_forward_vector[1]**2)
            rotated_pc = np.zeros(shape=point_cloud_array.shape, dtype=np.float32)
            rotated_pc[:, 3] = 1.0
            for point in range(len(point_cloud_array)):
                rotated_pc[point, 0:3] = rot_angle(point_cloud_array[point, 0:3], ego_forward_vector)
            filename = '{:06d}.bin'.format(file_number)  # e.g. 000001.bin
            filename = os.path.join(output_point_cloud_dir, filename)
            fileObject = open(filename, 'wb')

            rotated_pc = np.reshape(rotated_pc, (1, (np.shape(rotated_pc)[0] * np.shape(rotated_pc)[1])))
            number_of_points = int(rotated_pc.shape[1] / 4)

            # Transform (1,number_of_points) dimensional 2D array into (number_of_points,) 1D array
            # Vectorize the array before saving into binary file since the KITTI data is also vectorize.
            rotated_pc = rotated_pc.flatten()
            # Use to file to save into a bin file instead of pickle.dump
            rotated_pc.tofile(fileObject)
            fileObject.close()
            filename = '{:06d}.txt'.format(file_number)  # e.g. 000001.txt
            filename = os.path.join(output_ground_truth_kitti_dir, filename)
            ground_truth_kitti_array = ground_truth_kitti_array[:, 0:16] #Dont push ur luck, tone it back to original format
            item_counter = 0
            with open(filename, 'w') as f:
                for item in ground_truth_kitti_array:
                    for entry in item:
                        if item_counter == 15:
                            f.write('%s\n' % entry)
                            item_counter = 0
                        else:
                            item_counter +=1
                            f.write('%s ' % entry)

            # TODO get the bounding box values from instance values in ground truth files.
            print("[INFO] ", str("{0:.2f}".format((file_number / len(point_cloud_files) * 100.0))),
                  " percent of the files have been processed. . . {} ".format(points_counter),
                  str(os.path.basename(point_cloud_files[file_number])))
    print("[INFO] Total ", points_counter, " number of points have been removed")
    print("[INFO] JOB DONE ! ! ! ")
else:
    print('[INFO] The given sub directories were not found. please check the names . . .')


