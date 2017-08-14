#!/usr/bin/python
#   ______                   __  __              __
#  /\  _  \           __    /\ \/\ \            /\ \__
#  \ \ \L\ \  __  __ /\_\   \_\ \ \ \____    ___\ \ ,_\   ____
#   \ \  __ \/\ \/\ \\/\ \  /'_` \ \ '__`\  / __`\ \ \/  /',__\
#    \ \ \/\ \ \ \_/ |\ \ \/\ \L\ \ \ \L\ \/\ \L\ \ \ \_/\__, `\
#     \ \_\ \_\ \___/  \ \_\ \___,_\ \_,__/\ \____/\ \__\/\____/
#      \/_/\/_/\/__/    \/_/\/__,_ /\/___/  \/___/  \/__/\/___/
#  Copyright 2015, Avidbots Corp.
#  @name    fake_mc_driver.py
#  @brief   Pretends to be the MC driver
#  @author  Joseph Duchesne
##
import rospy
import math
from avidbots_msgs.msg import mc_status, mc_cmd, motor_value
from geometry_msgs.msg import Twist
import numpy as np

params = {}

mc_status_pub = None
velocity_pub = None
jacobian = None
inv_jacobian = None
old_status = None

motor_ticks = [0, 0, 0]
old_slip_ticks = [0, 0, 0]
slip_ticks = [0, 0, 0]
motor_rpm = [0, 0, 0]
old_motor_rpm = [0, 0, 0]


def apply_ticks(event):
    global motor_ticks, old_slip_ticks, slip_ticks, motor_rpm, old_motor_rpm
    global mc_status_pub, velocity_pub, jacobian, inv_jacobian, params, old_status

    # Update ticks based on RPMs
    for i in range(3):
        # Add ticks, converting rpm into ticks per delta period
        delta = motor_rpm[i]*params['rpm_to_ticks_per_sec']*params['pub_period']
        # print "Motor ", i, " norm ticks going from", motor_ticks[i],
        # "to", int(motor_ticks[i] + delta), " delta: ", delta
        motor_ticks[i] += int(delta)
        delta *= (1-params['slip_const'])  # Factor in static slip
        # Subtract slip caused acceleration
        slip = (motor_rpm[i]-old_motor_rpm[i])*params['rpm_to_ticks_per_sec']*params['pub_period']*params['slip_delta']
        delta -= slip
        # print "Motor ", i, " slip ticks going from", slip_ticks[i],
        # "to", int(slip_ticks[i] + delta), " delta: ", delta, params['slip_const']
        slip_ticks[i] += int(delta)

    # Todo: Delay here?

    # Publish the new tick/rpm values
    mval = motor_value()
    status = mc_status()

    status.rpm = [0, 0, 0]
    status.ticks = [0, 0, 0]

    for i in range(3):
        status.ticks[i] = motor_ticks[i]*params['invert'][i]
        status.rpm[i] = motor_rpm[i]*params['invert'][i]

    if old_status is not None:
        old_status.stamp = rospy.Time.now()
        # Beware, if the pub period is not a multiple of the Gazebo simulation max_step_size,
        # there will be aliasing effects in this timer callback.
        mc_status_pub.publish(old_status)

        # rospy.loginfo("Stamp time: %.15f", old_status.stamp.to_sec())

    old_status = status

    # Convert slip tick delta back into velocity twist, and publish to gazebo
    wheel_velocity = (np.array(slip_ticks)-np.array(old_slip_ticks))*params['ticks_to_m']/params['pub_period']
    # rospy.loginfo("Wheel Movement: %.3f", wheel_velocity)
    robot_velocity = inv_jacobian.dot(wheel_velocity)
    # rospy.loginfo("Robot Movement: %.3f", robot_velocity)
    velocity = Twist()
    velocity.linear.x = robot_velocity[0]
    velocity.linear.y = robot_velocity[1]
    velocity.angular.z = robot_velocity[2]
    velocity_pub.publish(velocity)

    old_motor_rpm = motor_rpm[:]
    old_slip_ticks = slip_ticks[:]


def mc_command_in(cmd):
    global motor_rpm

    # rospy.loginfo("Received command at %.4f", rospy.Time.now().to_sec())

    # rospy.loginfo("Received motor command %.3f, %.3f, %.3f",
    # cmd.commands[0].value, cmd.commands[1].value, cmd.commands[2].value)
    for command in cmd.commands:
        if command.motor < 3:
            motor_rpm[command.motor] = command.value*params['invert'][command.motor]


def init_jacobian():
    global jacobian, inv_jacobian
    jacobian = np.zeros((3, 3))

    jacobian[0, 0] = 0
    jacobian[1, 0] = -1.0 * math.sin(rospy.get_param("/robot_properties/kiwi_th2") * math.pi / 180.0)
    jacobian[2, 0] = -1.0 * math.sin(rospy.get_param("/robot_properties/kiwi_th3") * math.pi / 180.0)
    jacobian[0, 1] = 1
    jacobian[1, 1] = math.cos(rospy.get_param("/robot_properties/kiwi_th2") * math.pi / 180.0)
    jacobian[2, 1] = math.cos(rospy.get_param("/robot_properties/kiwi_th3") * math.pi / 180.0)
    jacobian[0, 2] = rospy.get_param("/robot_properties/kiwi_l1")
    jacobian[1, 2] = rospy.get_param("/robot_properties/kiwi_l2")
    jacobian[2, 2] = rospy.get_param("/robot_properties/kiwi_l3")

    inv_jacobian = np.linalg.inv(jacobian)

    # print "Inverse Jacobian Matrix:", inv_jacobian

# Initialize the node
if __name__ == '__main__':
    # Start ROS

    rospy.init_node('fake_mc_driver')

    params['slip_const'] = rospy.get_param("/robot_properties/simulated_slip_constant", 0.0)
    params['slip_delta'] = 0.00  # 0% Not tested, do not use
    params['pub_period'] = 1.0 / rospy.get_param("/mc_properties/mc_status_pub_rate", 40.0)
    params['ticks_per_rev'] = rospy.get_param("/robot_properties/ticks_per_rev", 1000)
    params['wheel_radius'] = rospy.get_param("/robot_properties/wheel_radius")
    params['gear_ratio'] = rospy.get_param("/robot_properties/gear_ratio")
    params['rpm_to_ticks_per_sec'] = 1.0/60*params['ticks_per_rev']
    params['ticks_to_m'] = 1.0/params['ticks_per_rev']/params['gear_ratio']*(2.0*math.pi*params['wheel_radius'])

    rospy.loginfo("Fake MC driver -- pub period is: %.3f", params['pub_period'])

    mval = motor_value()
    params['invert'] = [0, 0, 0]
    # WARNING: The order for front, left and right wheels are hard-coded due to the fact that
    #          kMotorFront, kMotorLeft and kMotorRight constants are removed from avidbots_msgs and
    #          have been moved into avidbots_msgs/constants.h to support enums for both shiny and wheely
    params['invert'][0] = rospy.get_param("/robot_properties/invert_front")
    params['invert'][1] = rospy.get_param("/robot_properties/invert_left")
    params['invert'][2] = rospy.get_param("/robot_properties/invert_right")

    init_jacobian()

    mc_status_pub = rospy.Publisher('avidbots/mc/mc_status', mc_status, queue_size=10)
    velocity_pub = rospy.Publisher('mobile_base/commands/velocity', Twist, queue_size=10)
    rospy.Subscriber("avidbots/mc/mc_cmd", mc_cmd, mc_command_in)

    rospy.Timer(rospy.Duration(params['pub_period']), apply_ticks)

    # Wait for input
    try:
        rospy.spin()
    except Exception:
        rospy.loginfo("Exiting...")

    # Other MC Driver subscriptions
    # static const std::string mc_manual_override_msg         ("avidbots/mc/manual_override_cmd");
    # Other MC Driver publishings
    # static const std::string mc_voltage_average_topic       ("avidbots/mc/mc_voltage_average");
    # static const std::string mc_voltage_status_topic        ("avidbots/mc/mc_voltage_status");
