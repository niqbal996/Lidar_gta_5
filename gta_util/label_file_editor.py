import numpy as np
import os
import glob

data_set_path = r'D:\GTA_sequence_2_fixed'

label_files = os.path.join(data_set_path, 'label_2')
pc_files = os.path.join(data_set_path, 'velodyne')
calib_files = os.path.join(data_set_path, 'calib')
image_files = os.path.join(data_set_path, 'image_2')

label_files_array = glob.glob(os.path.join(label_files, '*.txt'))
pc_array = glob.glob(os.path.join(pc_files, '*.bin'))
im_array = glob.glob(os.path.join(image_files, '*.png'))
calib_array = glob.glob(os.path.join(calib_files, '*.txt'))
bad_labels = 0
for file in range(len(label_files_array)):
    new_labels = []
    for line in open(label_files_array[file], 'r').readlines():
        # Delete the empty entries from the array, non-hit points from the ray cast
        current_label = line.split(' ')
        if current_label == ['\n']:
            continue
        if float(current_label[13]) < 2.0:
            bad_labels += 1
            print('[INFO] Found {} number of bad labels till now . . .'.format(bad_labels))
            continue
        else:
            new_labels.append(current_label)

    if new_labels == []:
        os.remove(pc_array[file])
        os.remove(label_files_array[file])
        os.remove(im_array[file])
        os.remove(calib_array[file])
        print('[INFO] Sample with filename {} has been removed from dataset'.format(os.path.basename(pc_array[file])))
    else:
        filename = label_files_array[file]
        item_counter = 0
        with open(filename, 'w') as f:
            for item in new_labels:
                for entry in item:
                    if item_counter == 15:
                        f.write('%s' % entry)
                        item_counter = 0
                    else:
                        item_counter += 1
                        f.write('%s ' % entry)
