#!/usr/bin/env python
#  ______                   __  __              __
# /\  _  \           __    /\ \/\ \            /\ \__
# \ \ \L\ \  __  __ /\_\   \_\ \ \ \____    ___\ \ ,_\   ____
#  \ \  __ \/\ \/\ \\/\ \  /'_` \ \ '__`\  / __`\ \ \/  /',__\
#   \ \ \/\ \ \ \_/ |\ \ \/\ \L\ \ \ \L\ \/\ \L\ \ \ \_/\__, `\
#    \ \_\ \_\ \___/  \ \_\ \___,_\ \_,__/\ \____/\ \__\/\____/
#     \/_/\/_/\/__/    \/_/\/__,_ /\/___/  \/___/  \/__/\/___/
#  Copyright 2016, Avidbots Corp.
#
#  To use:
#  rosrun
#  asdqwe to translate
#  insert, home, pageup, delete, end, pagedown to rotate
#
#  Escape to quit.
#
#  @name  calibrate_camera_manual.py
#  @brief  A program that manually calibrates kinect position/rotation
#  @author Joseph Duchesne
#

import roslib
import rospy
import json

import curses

import tf
from sensor_msgs.msg import PointCloud2
import tf.transformations as transformations

# Default values for the calibration. These are based on the Neo24 Calibration, but are otherwise arbitrary.
position = {"left": {"rpy": [0.024626, 0.207318, 0.418879], "xyz": [0.026, 0.161, 0.9484]},
            "right": {"rpy": [-0.037126, 0.237318, -0.518879], "xyz": [0.135, -0.077, 0.9884]}}


camera = None

camera_rgb_to_depth = None  # The transform from camera base frame to camera depth frame

cloud_publisher = None


def handle_point_cloud(cloud):
    # print "Read %s cloud\n" % camera
    br = tf.TransformBroadcaster()

    # Generate translation and rotation quaternion based on calibration values
    translation = [position[camera]['xyz'][0], position[camera]['xyz'][1], position[camera]['xyz'][2]]
    rotation = tf.transformations.quaternion_from_euler(position[camera]['rpy'][0],
                                                        position[camera]['rpy'][1],
                                                        position[camera]['rpy'][2])

    # Translate and rotate by camera base to depth
    translation = [x + y for x, y in zip(translation, camera_rgb_to_depth[0])]
    rotation = transformations.quaternion_multiply(rotation, camera_rgb_to_depth[1])

    cloud.header.frame_id = "/camera_%s_calibration" % camera

    br.sendTransform(translation,
                     rotation,
                     rospy.Time.now(),
                     cloud.header.frame_id,
                     "base_link")

    cloud_publisher.publish(cloud)


def curses_loop(screen):
    global position

    # Curses loop
    key = ''
    stdscr.addstr(5, 5, "Press ESC to quit")
    stdscr.addstr(6, 0, "Calibration: %s" % json.dumps(position[camera]))
    stdscr.refresh()

    curses.noecho()
    while key != 27:
        key = stdscr.getch()

        if key == ord('q'):  # q
            position[camera]['xyz'][2] -= 0.005
        elif key == ord('e'):  # e
            position[camera]['xyz'][2] += 0.005
        if key == ord('w'):  # w
            position[camera]['xyz'][0] += 0.005
        elif key == ord('s'):  # s
            position[camera]['xyz'][0] -= 0.005
        if key == ord('a'):  # a
            position[camera]['xyz'][1] += 0.005
        elif key == ord('d'):  # d
            position[camera]['xyz'][1] -= 0.005
        elif key == curses.KEY_IC:  # insert
            position[camera]['rpy'][0] -= 0.005
        elif key == curses.KEY_PPAGE:  # page up
            position[camera]['rpy'][0] += 0.0025
        if key == curses.KEY_HOME:  # home
            position[camera]['rpy'][1] += 0.0025
        elif key == curses.KEY_END:  # end
            position[camera]['rpy'][1] -= 0.0025
        if key == curses.KEY_DC:  # delete
            position[camera]['rpy'][2] += 0.0025
        elif key == curses.KEY_NPAGE:  # page down
            position[camera]['rpy'][2] -= 0.0025

        stdscr.addstr(6, 0, "Calibration: %s" % json.dumps(position[camera]))
        stdscr.refresh()

    curses.endwin()

if __name__ == '__main__':
    rospy.init_node('calibrate_camera_manual')

    try:
        camera = rospy.get_param('~camera')
    except:
        print "Please run with: rosrun pcl_calibration calibrate_camera_manual.py _camera:=left|right"
        exit()

    stdscr = curses.initscr()

    listener = tf.TransformListener()

    camera_base_frame = "/camera_%s_rgb_frame" % camera
    camera_depth_frame = "/camera_%s_depth_optical_frame" % camera
    listener.waitForTransform(camera_base_frame, camera_depth_frame, rospy.Time(0), rospy.Duration(3.0))

    camera_rgb_to_depth = listener.lookupTransform(camera_base_frame, camera_depth_frame, rospy.Time(0))

    stdscr.addstr(3, 0, "Manual calibration for camera %s\n" % camera)
    rospy.Subscriber('/camera_%s/depth/points_raw' % camera, PointCloud2, handle_point_cloud)
    cloud_publisher = rospy.Publisher('/camera_%s_calibration_cloud' % camera, PointCloud2, queue_size=1)

    curses.wrapper(curses_loop)
    print "Calibration: %s" % json.dumps(position[camera])
