#!/usr/bin/env python
# -*- coding:UTF-8 -*-

# File Name : train3.py

import argparse
import os
from os.path import join
import numpy as np
import sys
from matplotlib import pyplot as  plt
import cv2
from PIL import Image


scriptPath = os.path.realpath(os.path.dirname(__file__))
sys.path.append(scriptPath + "/../../..")  # PeRL

from Lidar.python.VoxelNet.config import cnfg
from Lidar.python.VoxelNet.voxelnet_local_config import VN_LOCCONF
from Lidar.python.VoxelNet.voxelnet_utils.loader import Loader

parser = argparse.ArgumentParser(description='training')
parser.add_argument('-i', '--max-epoch', type=int, nargs='?', default=200,
                    help='max epoch')
parser.add_argument('-n', '--tag', type=str, nargs='?', default='CycleGAN_image_GAN',
                    help='set log tag')
parser.add_argument('-b', '--single-batch-size', type=int, nargs='?', default=1,
                    help='set batch size for each gpu')
parser.add_argument('-l', '--lr', type=float, nargs='?', default=0.0002,
                    help='set learning rate')
parser.add_argument('-dtrain_A', '--data_A', type=str, nargs='?', default=VN_LOCCONF.data_train,
                    help='location of train_A data')
parser.add_argument('-dtrain_B', '--data_B', type=str, nargs='?', default=VN_LOCCONF.data_test,
                    help='location of train_B data')
parser.add_argument('-log', '--logloc', type=str, nargs='?', default=VN_LOCCONF.log_loc,
                    help='location to store data for tensorboard')
parser.add_argument('-mloc', '--modelloc', type=str, nargs='?', default=VN_LOCCONF.model_loc,
                    help='location to store model')
parser.add_argument('-g', '--g', type=str, nargs='?', default=16,
                    help='number of generator filters')
parser.add_argument('-d', '--d', type=str, nargs='?', default=8,
                    help='number of discriminator filters')
parser.add_argument('-images', '--image', type=str, nargs='?', default='output_images',
                    help='location to store figures for debugging')
parser.add_argument('--L1_lambda', dest='L1_lambda', type=float, default=10.0, help='weight on L1 term in objective')
args = parser.parse_args()

def scale_to_255(a, min, max, dtype=np.uint8):
    """ Scales an array of values from specified min, max range to 0-255
        Optionally specify the data type of the output (default is uint8)
    """
    return (((a - min) / float(max - min)) * 255).astype(dtype)

def cam_2_lidar(vec1):
    x, y, z = float(vec1[0]), float(vec1[1]), float(vec1[2])
    R_xyz = np.array([[0.0, -1.0, 0.0],
                      [0.0, 0.0, -1.0],
                      [1.0, 0.0, 0.0]])

    x1 = np.transpose([[x, 0, 0]])
    y1 = np.transpose([[0, y, 0]])
    z1 = np.transpose([[0, 0, z]])
    x1 = np.linalg.inv(R_xyz).dot(x1)
    y1 = np.linalg.inv(R_xyz).dot(y1)
    z1 = np.linalg.inv(R_xyz).dot(z1)
    return float(z1[0]), float(x1[1]), float(y1[2])


def point_cloud_2_birdseye(points,
                           res=0.1,
                           side_range=(-40., 40.),  # left-most to right-most
                           fwd_range=(0., 40.),     # back-most to forward-most
                           height_range=(-3., 1.),  # bottom-most to upper-most
                           ):
    """ Creates an 2D birds eye view representation of the point cloud data.

    Args:
        points:     (numpy array)
                    N rows of points data
                    Each point should be specified by at least 3 elements x,y,z
        res:        (float)
                    Desired resolution in metres to use. Each output pixel will
                    represent an square region res x res in size.
        side_range: (tuple of two floats)
                    (-left, right) in metres
                    left and right limits of rectangle to look at.
        fwd_range:  (tuple of two floats)
                    (-behind, front) in metres
                    back and front limits of rectangle to look at.
        height_range: (tuple of two floats)
                    (min, max) heights (in metres) relative to the origin.
                    All height values will be clipped to this min and max value,
                    such that anything below min will be truncated to min, and
                    the same for values above max.
    Returns:
        2D numpy array representing an image of the birds eye view.
    """
    # EXTRACT THE POINTS FOR EACH AXIS
    x_points = points[:, 0]
    y_points = points[:, 1]
    z_points = points[:, 2]

    # FILTER - To return only indices of points within desired cube
    # Three filters for: Front-to-back, side-to-side, and height ranges
    # Note left side is positive y axis in LIDAR coordinates
    f_filt = np.logical_and((x_points > fwd_range[0]), (x_points < fwd_range[1]))
    s_filt = np.logical_and((y_points > -side_range[1]), (y_points < -side_range[0]))
    filter = np.logical_and(f_filt, s_filt)
    indices = np.argwhere(filter).flatten()

    # KEEPERS
    x_points = x_points[indices]
    y_points = y_points[indices]
    z_points = z_points[indices]

    # CONVERT TO PIXEL POSITION VALUES - Based on resolution
    x_img = (-y_points / res).astype(np.int32)  # x axis is -y in LIDAR
    y_img = (-x_points / res).astype(np.int32)  # y axis is -x in LIDAR

    # SHIFT PIXELS TO HAVE MINIMUM BE (0,0)
    # floor & ceil used to prevent anything being rounded to below 0 after shift
    x_img -= int(np.floor(side_range[0] / res))
    y_img += int(np.ceil(fwd_range[1] / res))

    # CLIP HEIGHT VALUES - to between min and max heights
    pixel_values = np.clip(a=z_points,
                           a_min=height_range[0],
                           a_max=height_range[1])

    # RESCALE THE HEIGHT VALUES - to be between the range 0-255
    pixel_values = scale_to_255(pixel_values,
                                min=height_range[0],
                                max=height_range[1])

    # INITIALIZE EMPTY ARRAY - of the dimensions we want
    x_max = 1 + int((side_range[1] - side_range[0]) / res)
    y_max = 1 + int((fwd_range[1] - fwd_range[0]) / res)
    im = np.zeros([y_max, x_max], dtype=np.uint8)

    # FILL PIXEL VALUES IN IMAGE ARRAY
    im[y_img, x_img] = pixel_values

    return im

def point_cloud_2_birdseye_3(points,
                           res=0.1,
                           side_range=(-40., 40.),  # left-most to right-most
                           fwd_range=(0., 40.),     # back-most to forward-most
                           height_range=(-3., 1.),  # bottom-most to upper-most
                           ):
    """ Creates an 2D birds eye view representation of the point cloud data.

    Args:
        points:     (numpy array)
                    N rows of points data
                    Each point should be specified by at least 3 elements x,y,z
        res:        (float)
                    Desired resolution in metres to use. Each output pixel will
                    represent an square region res x res in size.
        side_range: (tuple of two floats)
                    (-left, right) in metres
                    left and right limits of rectangle to look at.
        fwd_range:  (tuple of two floats)
                    (-behind, front) in metres
                    back and front limits of rectangle to look at.
        height_range: (tuple of two floats)
                    (min, max) heights (in metres) relative to the origin.
                    All height values will be clipped to this min and max value,
                    such that anything below min will be truncated to min, and
                    the same for values above max.
    Returns:
        2D numpy array representing an image of the birds eye view.
    """
    # EXTRACT THE POINTS FOR EACH AXIS
    x_points = points[:, 0]
    y_points = points[:, 1]
    z_points = points[:, 2]

    # FILTER - To return only indices of points within desired cube
    # Three filters for: Front-to-back, side-to-side, and height ranges
    # Note left side is positive y axis in LIDAR coordinates
    f_filt = np.logical_and((x_points > fwd_range[0]), (x_points < fwd_range[1]))
    s_filt = np.logical_and((y_points > -side_range[1]), (y_points < -side_range[0]))
    filter = np.logical_and(f_filt, s_filt)
    indices = np.argwhere(filter).flatten()

    # KEEPERS
    x_points = x_points[indices]
    y_points = y_points[indices]
    z_points = z_points[indices]

    # CONVERT TO PIXEL POSITION VALUES - Based on resolution
    x_img = (-y_points / res).astype(np.int32)  # x axis is -y in LIDAR
    y_img = (-x_points / res).astype(np.int32)  # y axis is -x in LIDAR

    # SHIFT PIXELS TO HAVE MINIMUM BE (0,0)
    # floor & ceil used to prevent anything being rounded to below 0 after shift
    x_img -= int(np.floor(side_range[0] / res))
    y_img += int(np.ceil(fwd_range[1] / res))

    # CLIP HEIGHT VALUES - to between min and max heights
    pixel_values = np.clip(a=z_points,
                           a_min=height_range[0],
                           a_max=height_range[1])

    # RESCALE THE HEIGHT VALUES - to be between the range 0-255
    pixel_values = scale_to_255(pixel_values,
                                min=height_range[0],
                                max=height_range[1])

    # INITIALIZE EMPTY ARRAY - of the dimensions we want
    x_max = 1 + int((side_range[1] - side_range[0]) / res)
    y_max = 1 + int((fwd_range[1] - fwd_range[0]) / res)
    im = np.zeros([y_max, x_max], dtype=np.uint8)

    # FILL PIXEL VALUES IN IMAGE ARRAY
    im[y_img, x_img] = pixel_values

    return im
def main():
    global save_model_dir
    with Loader(data_spec=args.data_A, queue_size=50, require_shuffle=True,
                is_testset=False, batch_size=args.single_batch_size * cnfg.GPU_USE_COUNT,
                use_multi_process_num=VN_LOCCONF.multi_process_num, multi_gpu_sum=cnfg.GPU_USE_COUNT,
                aug=True) as train_A_loader:

        for sample in range(len(train_A_loader)):
            current_sample = train_A_loader.load()
            image = point_cloud_2_birdseye(current_sample[6][0].LIDARS[0].pointcloud.T)
            image[np.isnan(image)] = 0.0
            image = np.reshape(image, newshape=(image.shape[0], image.shape[1], 1))
            image = np.concatenate([image, image, image], axis=2)
            labels = current_sample[6][0].LABEL['GT']
            # cv2.imshow("Show", image)
            for object in range(len(labels)):
                type = ''
                position = [0, 0, 0]
                if labels[object].type == 'car_or_van_or_suv':
                    current_object = labels[object]
                    # bounding box dimensions
                    length = (current_object.dim3d[0, 1]/0.1).astype(np.int32)
                    width = (current_object.dim3d[0, 2]/0.1).astype(np.int32)
                    # x and y position in camera coordinates
                    position[0] = current_object.loc3d[0, 0]
                    position[1] = current_object.loc3d[0, 1]
                    position[2] = current_object.loc3d[0, 2]

                    # position[0], position[1], position[2] = cam_2_lidar(position)
                    # x and y will be interchanged now
                    # x_img = (-position[1] / 0.1).astype(np.int32)  # x axis is -y in LIDAR
                    # y_img = (-position[0] / 0.1).astype(np.int32)  # y axis is -x in LIDAR
                    x_img = int(-position[1] / 0.1)  # x axis is -y in LIDAR
                    y_img = int(-position[0] / 0.1)  # y axis is -x in LIDAR

                    x_img -= int(np.floor(-40.0 / 0.1))
                    y_img += int(np.ceil(40.0 / 0.1))
                    c1 = int(round(x_img - width/2)), int(round(y_img - length/2))
                    c2 = int(round(x_img + width/2)), int(round(y_img + length/2))
                    cv2.rectangle(image, c1, c2, (0, 255, 0), 2)
                    cv2.putText(image, 'CAR', (c1[0]-3, c1[1]-3), 0, 0.3, (0, 255, 0))
                    # cv2.imshow("Show", image)
                    # cv2.waitKey()
                    print('hold')
                else:
                    continue

            label = current_sample
            plt.imshow(image, cmap='BrBG', vmin=0, vmax=255)
            plt.imshow(image, vmin=0, vmax=255)
            plt.show()
            # cv2.rectangle(image, (10, 10), (40, 40), (255, 255, 0), 5)
            # cv2.imshow("Show", image)
            cv2.waitKey()
            cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
