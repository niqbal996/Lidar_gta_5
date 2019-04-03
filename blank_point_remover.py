import numpy as np
from matplotlib import pyplot
from mpl_toolkits.mplot3d import Axes3D
from os import path
import glob
import os
import pickle

data_set_path = "D:\\Applications\\steamapps\\common\\Grand Theft Auto V\\LiDAR GTA V"

output_directory = "synthetic\\"
output_point_cloud_dir = os.path.join(output_directory, "velodyne")
output_ground_truth_dir = os.path.join(output_directory, "labels")
output_ground_truth_kitti_dir = os.path.join(output_directory, "kitti_labels")
os.makedirs(output_point_cloud_dir, exist_ok=True)
os.makedirs(output_ground_truth_dir, exist_ok=True)
os.makedirs(output_ground_truth_kitti_dir, exist_ok=True)

point_cloud_dir = os.path.join(data_set_path, "velodyne")
ground_truth_dir = os.path.join(data_set_path, "labels")
ground_truth_kitti_dir = os.path.join(data_set_path, "kitti_labels")
points_counter = 0

number_of_values_per_line = 5
bounding_box_vertices = np.zeros((8,3))     #Spatial coordinates for Eight points for each cuboid(x,y,z).

if(path.exists(point_cloud_dir) and path.exists(ground_truth_dir) and path.exists(ground_truth_kitti_dir)):
    point_cloud_files = glob.glob(os.path.join(point_cloud_dir , "*.txt"))
    ground_truth_files = glob.glob(os.path.join(ground_truth_dir,"*.txt"))
    ground_truth_kitti_files = glob.glob(os.path.join(ground_truth_kitti_dir, "*.txt"))
    assert(len(point_cloud_files) == len(ground_truth_files) ==len(ground_truth_kitti_files))
    for file_number in range(len(point_cloud_files)):
        current_point_cloud_file  = os.path.join(point_cloud_dir, point_cloud_files[file_number])
        current_ground_truth_file = os.path.join(ground_truth_dir, ground_truth_files[file_number])
        current_ground_truth_kitti_file = os.path.join(ground_truth_dir, ground_truth_kitti_files[file_number])
        #For file smaller than 1KB , ignore. These are empty files
        if os.stat(current_ground_truth_file).st_size < 1000:
            continue
        else:
            point_cloud_array =[]
            ground_truth_array = []
            ground_truth_kitti_array = []
            #Process the point cloud data first
            for line in open(current_point_cloud_file, 'r').readlines():
                # Delete the empty entries from the array, non-hit points from the ray cast
                if list(map(float, line.split(" "))) == [0.0, 0.0, 0.0, 0]:
                    points_counter += 1
                    continue
                else:
                    #x =list(map(float, line.split(" ")))[0:4]
                    point_cloud_array.append(list(map(float, line.split(" ")))[0:4])

            print("[INFO] " , str(points_counter), " number of points have been removed.")
            points_counter = 0 #reset
            point_cloud_array = np.asarray(point_cloud_array)
            #Same naming convention as KITTI
            filename = '{:06d}.bin'.format(file_number)                 #e.g. 000001.bin
            filename = os.path.join(output_point_cloud_dir,filename)
            fileObject = open(filename, 'wb')
            #Vectorize the array before saving into binary file since the KITTI data is also vectorize.
            pickle.dump(point_cloud_array, fileObject)
            fileObject.close()


            #Process the ground truth data now
            for line in open(current_ground_truth_file, 'r').readlines():
                #Delete the empty entries from the array, non-hit points from the ray cast
                x = list(map(int, line.split(" ")))
                if list(map(int, line.split(" "))) == [0, 0, 0, 0]:
                    points_counter += 1;
                    continue
                else:
                    ground_truth_array.append(list(map(int, line.split(" "))))

            print("[INFO] " , str(points_counter), " number of points have been removed from ground truth file.")
            ground_truth_array = np.asarray(ground_truth_array)
            assert(len(ground_truth_array) == len(point_cloud_array))
            instances = ground_truth_array[:, 3]
            unique_instances = np.unique(instances)
            unique_instances = unique_instances[1:,] #ignore 0 because its not needed. len(unique_instances) = number of cars in the current snapshot
            first_occurence = np.zeros(len(unique_instances),dtype="int8")

            for instance in range(len(unique_instances)):
                all_occurences = np.where(instances==unique_instances[instance])[0]
                first_occurence[instance] = int(all_occurences[0])
            # Same naming convention as KITTI
            filename = '{:06d}.txt'.format(file_number)  # e.g. 000001.txt
            filename = os.path.join(output_ground_truth_dir, filename)
            fileObject = open(filename, 'wb')
            # Vectorize the array before saving into binary file since the KITTI data is also vectorize.
            pickle.dump(ground_truth_array, fileObject)
            fileObject.close()

            #Process the KITTI ground truth data now
            lines = open(current_ground_truth_kitti_file, 'r').readlines()
            for line in open(current_ground_truth_kitti_file, 'r').readlines():
                ground_truth_kitti_array.append(list(map(float, line.split(" "))))
            # Same naming convention as KITTI
            ground_truth_kitti_array = np.asarray(ground_truth_kitti_array)
            ground_truth_kitti_array = ground_truth_kitti_array[first_occurence,:]
            filename = '{:06d}.txt'.format(file_number)  # e.g. 000001.txt
            filename = os.path.join(output_ground_truth_kitti_dir, filename)
            fileObject = open(filename, 'wb')
            # Vectorize the array before saving into binary file since the KITTI data is also vectorize.
            pickle.dump(ground_truth_array, fileObject)
            fileObject.close()

            fig2 = pyplot.figure()
            blue_points_array = np.where(instances == 0)    #all blue
            red_points_array = np.where(instances != 0)     #all reds
            pyplot.scatter(point_cloud_array[red_points_array, 0],
                           point_cloud_array[red_points_array, 1], s=0.01, marker=".",c='r')
            pyplot.scatter(point_cloud_array[blue_points_array, 0],
                           point_cloud_array[blue_points_array, 1], s=0.01, marker=".", c='b')
            pyplot.xlabel("X axis")
            pyplot.ylabel('Y axis')
            pyplot.show()
            # fig = pyplot.figure()
            # ax = Axes3D(fig)
            # blue_points_array = np.where(instances == 0)    #all blue
            # red_points_array = np.where(instances != 0)     #all reds
            # ax.scatter(point_cloud_array[red_points_array, 0],
            #            point_cloud_array[red_points_array, 1],
            #            point_cloud_array[red_points_array, 2],
            #            s=0.01, marker=".", c='r')
            # ax.scatter(point_cloud_array[blue_points_array, 0],
            #            point_cloud_array[blue_points_array, 1],
            #            point_cloud_array[blue_points_array, 2],
            #            s=0.01, marker=".", c='b')
            # ax.set_xlabel("X axis")
            # ax.set_ylabel("Y axis")
            # ax.set_zlabel("Z axis")
            # pyplot.show()

            # fig2 = pyplot.figure()
            # pyplot.scatter(point_cloud_array[:, 0], point_cloud_array[:, 1], s=0.01, marker=".")
            # pyplot.xlabel("X axis")
            # pyplot.ylabel('Y axis')
            # assert(len(unique_instances) == ground_truth_kitti_array.shape[0])
            # # for rectangle in range(len(unique_instances)):
            # #       pyplot.arrow(ground_truth_kitti_array[rectangle, 0],   #x1              #vertex 1 and 2
            # #               ground_truth_kitti_array[rectangle,  1],  #y1
            # #               ground_truth_kitti_array[rectangle,  3],  #x2
            # #               ground_truth_kitti_array[rectangle,  4])  #y2
            # #       pyplot.arrow(ground_truth_kitti_array[rectangle, 0],  # x1              #vertex 1 and 7
            # #               ground_truth_kitti_array[rectangle,  1],  # y1
            # #               ground_truth_kitti_array[rectangle,   18],  # x2
            # #               ground_truth_kitti_array[rectangle,   19])  # y2
            # #       pyplot.arrow(ground_truth_kitti_array[rectangle,   21],  # x1              #vertex 8 and 7
            # #               ground_truth_kitti_array[rectangle,   22],  # y1
            # #               ground_truth_kitti_array[rectangle,   18],  # x2
            # #               ground_truth_kitti_array[rectangle,   19])  # y2
            # #       pyplot.arrow(ground_truth_kitti_array[rectangle,   3],  # x1              #vertex 2 and 8
            # #               ground_truth_kitti_array[rectangle,   4],  # y1
            # #               ground_truth_kitti_array[rectangle,   21],  # x2
            # #               ground_truth_kitti_array[rectangle,   22])  # y2
            # pyplot.show()

            # TODO get the bounding box values from instance values in ground truth files.
            print("[INFO] ", str(round(file_number / len(point_cloud_files) * 100.0)), " percent of the files have been processed. . .")

    print (points_counter , " number of points have been removed")
    print("[INFO]JOB DONE ! ! ! ")




# number_of_points = int(cloud.shape[0] / 4)
#
# #data contains 4*num values, where the first 3 values correspond to x,y and z, and the last value is the reflectance information.
# points = np.reshape(cloud, (number_of_points , 4))
#
# print("Points shape is" , points[:,0].shape)
# x= points[:,0]
# y = points[:,1]
# z = points[:,2]
#
# fig = pyplot.figure()
# ax = Axes3D(fig)
# ax.scatter(points[:,0],points[:,1],points[:,2], marker=".")
# ax.set_xlabel("X axis")
# ax.set_ylabel("Y axis")
# ax.set_zlabel("Z axis")
#
# fig2 = pyplot.figure()
# pyplot.scatter(x,y, marker=".")
# pyplot.xlabel("X axis")
# pyplot.ylabel('Y axis')
# pyplot.show()
# print("number of points are:" + str(number_of_points))
#
