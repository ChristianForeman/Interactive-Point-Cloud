#!/usr/bin/env python
import rospy
import utils
import math
import numpy as np
from std_msgs.msg import Float32, Float32MultiArray
from arc_utilities.ros_helpers import get_connected_publisher
from sensor_msgs.msg import PointCloud2


class Pc:
    def __init__(self):
        self.sel_pub = get_connected_publisher("selected", PointCloud2, queue_size=10)
        self.pc_pub = get_connected_publisher("pc", PointCloud2, queue_size=10)

        # Read in the point cloud
        pc_source = utils.load_pc('cloud_icp_source.csv')
        self.points = (np.array(utils.convert_pc_to_matrix(pc_source)) * 8).T

        # Publish the initial points of the cloud
        msg = utils.points_to_pc2_msg(self.points, "world")
        self.pc_pub.publish(msg)

        # Set default values
        self.CENTER = np.array([0, 0, 0])
        self.RADIUS = 0.25

        # Highlight the starting area
        self.highlight()

    def highlight(self):
        sel_pc = []
        for i in range(self.points.shape[0]):
            cur_point = self.points[i, :]
            dist = math.sqrt(np.sum((cur_point - self.CENTER) ** 2))
            if dist < self.RADIUS:
                sel_pc.append(self.points[i, :])
        # If no points in the range, vstack throws an error
        if len(sel_pc) > 0:
            sel_pc = np.vstack(sel_pc)

        msg = utils.points_to_pc2_msg(sel_pc, "world")
        self.sel_pub.publish(msg)


rospy.init_node("highlighter")
my_pointcloud = Pc()


def handle_interactive_marker(point):
    # Gets point which is the new center of the pointcloud
    my_pointcloud.CENTER = np.array(point.data)
    my_pointcloud.highlight()


def handle_slider(radius):
    # Gets the radius which is the range to select the points
    my_pointcloud.RADIUS = radius.data
    my_pointcloud.highlight()


def main():
    rospy.Subscriber("center", Float32MultiArray, handle_interactive_marker)
    rospy.Subscriber("radius", Float32, handle_slider)
    rospy.spin()


if __name__ == '__main__':
    main()
