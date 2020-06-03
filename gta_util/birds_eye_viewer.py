import numpy as np
import os
import sys
import argparse
import cv2
import glob
import pickle
from PIL import Image
import matplotlib.pyplot as plt
from Lidar.python.lidar_utils.data_provider.kitti.kitti_load_scan import *
import pptk

scriptPath = os.path.realpath(os.path.dirname(__file__))
sys.path.append(scriptPath + "/../../..")# PeRL

pickle_fileA = r'C:\Users\zj7xsg\git\KITTI\normed_real_full_size.pkl'
# pickle_fileA = r'C:\Users\zj7xsg\git\KITTI\cars_kitti_single.pkl'
# pickle_fileA = r'C:\Users\zj7xsg\git\KITTI\normed_synthetic_full_size.pkl'
path = r'E:\GTA_sequence\GTA_ref_dataset\velodyne\000255.bin'
samples = r'C:\Users\zj7xsg\git\Mini_cycle_GAN\CycleGAN_logs_reduced_size_Scaled_2019-08-05_15_35\Output_samples'
output_dataset = r'E:\KITTI_car_images\real'
# output_dataset = r'E:\KITTI_car_images\synthetic'
gen_samples = os.path.join(samples, 'generated*.bin')
input_samples = os.path.join(samples, 'input*.bin')
gen_samples = glob.glob(gen_samples)
input_samples = glob.glob(input_samples)
# ==============================================================================
#                                                                   SCALE_TO_255
# ==============================================================================
def scale_to_255(a, min, max, dtype=np.uint8):
    """ Scales an array of values from specified min, max range to 0-255
        Optionally specify the data type of the output (default is uint8)
    """
    return (((a - min) / float(max - min)) * 255).astype(dtype)


# ==============================================================================
#                                                         POINT_CLOUD_2_BIRDSEYE
# ==============================================================================
def point_cloud_2_birdseye(points,
                           res=0.1,
                           side_range=(-40., 40.),  # left-most to right-most
                           fwd_range = (0, 40.), # back-most to forward-most
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


# for file in range(len(gen_samples)):
#     with open(gen_samples[file], 'rb') as f:
#         sample = pickle.load(f)
#         # sample = np.transpose(sample[0]['kitti_single']['Car'][49].pointcloud)
#         sample.pointcloud = np.reshape(sample.pointcloud, newshape=(8192, 4))
#         sample = sample.pointcloud[sample.mask, :]
#         # sample[0:4, :] = 0.0
#         sample[:, 2] = sample[:, 2] * -1.0
#         v = pptk.viewer(sample[:, 0:3])
#         v.set(point_size=0.001)
    # image = point_cloud_2_birdseye(sample)
    # plt.imshow(image, cmap='BrBG', vmin=0, vmax=255)
    # plt.imshow(image, vmin=0, vmax=255)
    # plt.show()
    #
    # sample = np.transpose(load_scan(path))
    # v = pptk.viewer(sample[:, 0:3])
    # v.set(point_size=0.001)
ctr = 0
# with open(pickle_fileA, 'rb') as f:
#     samples = pickle.load(f)
#     for sample in range(len(samples)):
#         pc = samples[sample].pointcloud[:, 0:3]
#         if len(samples[sample].mask) > 2048:
#             pc[0:4, :] = 0.0
#             image = point_cloud_2_birdseye(pc)
#             image = image[1000-256:1000+256, 1000-256:1000+256]
#             output_image_file_name = '{:06d}.jpg'.format(sample)
#             im = Image.fromarray((image.reshape((512, 512))).astype(np.uint8))
#             im.save(os.path.join(output_dataset, output_image_file_name))
#             ctr += 1
#             print('[INFO] :::::: {} / {}'.format(ctr, len(samples)))
#         else:
#             print('Ignored sample')
#     print('hold')

pc = load_scan(path)
pc[np.isnan(pc)] = 0
image = point_cloud_2_birdseye(np.transpose(pc))
im = Image.fromarray(image.astype(np.uint8))
im.save('000019.png')
