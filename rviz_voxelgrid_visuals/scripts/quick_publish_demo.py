#!/usr/bin/env python
import pickle

import numpy as np
import rospkg
import rospy
from rviz_voxelgrid_visuals_msgs.msg import VoxelgridStamped
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import ColorRGBA

from rviz_voxelgrid_visuals import conversions

"""
This is an example showing how to use the voxelgrid publisher
Open Rviz with the included `example_voxelgrid.rviz` file, then run this script
"""

if __name__ == "__main__":

    rospy.init_node("quick_voxelgrid_publisher_demo")
    pub_1 = rospy.Publisher('/voxelgrid_1', VoxelgridStamped, queue_size=1)
    pub_2 = rospy.Publisher('/voxelgrid_2', VoxelgridStamped, queue_size=1)
    point_pub = rospy.Publisher('/pointcloud', PointCloud2, queue_size=1)

    i = 1
    while pub_1.get_num_connections() == 0 or pub_2.get_num_connections() == 0 or point_pub.get_num_connections() == 0:
        i += 1
        if i % 10 == 0:
            rospy.loginfo("Waiting for publishers to connect")
        rospy.sleep(0.1)

    # Display some random voxels in rviz
    vg_1 = np.clip(np.random.random((64, 64, 64)) * 100.0 - 99.0, 0, 1)  # Only 1% of voxels have a non-zero value
    pub_1.publish(conversions.vox_to_voxelgrid_stamped(vg_1,
                                                       scale=0.01,  # Each voxel is a 1cm cube
                                                       frame_id='world',  # In frame "world", same as rviz fixed frame
                                                       origin=[-32 * 0.01] * 3))  # With origin centering the voxelgrid

    # Load a voxelgrid of a mug and display in rviz
    mug_path = rospkg.RosPack().get_path('rviz_voxelgrid_visuals') + '/examples/mug_voxelgrid.pkl'
    with open(mug_path, 'rb') as f:
        vg_2 = pickle.load(f, encoding='latin1')
    vg_msg_2 = conversions.vox_to_voxelgrid_stamped(vg_2, scale=0.02, frame_id='world', origin=(-.64, -.64, 0))
    pub_2.publish(vg_msg_2)

    point_pub.publish(conversions.vox_to_pointcloud2_msg(vg_2, scale=0.02, frame='world', origin=(-.64, -.64, 0),
                                                         density_factor=2))
    rospy.sleep(1)
    vg_empty = np.zeros((64, 64, 64))
    print(f"Clearing voxelgrid")
    pub_1.publish(conversions.vox_to_voxelgrid_stamped(vg_empty,
                                                       scale=0.01,  # Each voxel is a 1cm cube
                                                       frame_id='world',  # In frame "world", same as rviz fixed frame
                                                       origin=[-32 * 0.01] * 3))

    # Setting the color via code
    vg_msg_2.has_color = True
    vg_msg_2.origin.x += 0.5
    vg_msg_2.color = ColorRGBA(r=1, g=0, b=0, a=1)
    pub_1.publish(vg_msg_2)
    rospy.sleep(1)
