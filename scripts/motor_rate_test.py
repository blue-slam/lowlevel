#!/usr/bin/env python

import rospy

from std_msgs.msg import Byte
from mip_rover_lowlevel.msg import MotorRate
from mip_rover_lowlevel.msg import MotorCmd

current_left_motor_rate = 0
current_right_motor_rate = 0


def motor_rate_handler(msg):
    global current_left_motor_rate
    global current_right_motor_rate
    current_left_motor_rate = msg.left_motor_rate
    current_right_motor_rate = msg.right_motor_rate


if __name__ == "__main__":
    try:
        rospy.init_node('motor_rate_test')

        motor_volts = rospy.get_param('/mip_rover/motors/volts')
        controller_cmd_mag = rospy.get_param('/mip_rover/controller/cmd_mag')
        motor_cmd_scale = int((motor_volts * controller_cmd_mag) / 8.4)

        motor_rate_sub = rospy.Subscriber('/mip_rover/motors/rates', MotorRate, motor_rate_handler)
        motor_cmd_pub = rospy.Publisher('/mip_rover/motors/cmd', MotorCmd, queue_size=10)
        green_led_pub = rospy.Publisher('/mip_rover/leds/green', Byte, queue_size=10)

        rate = rospy.Rate(4)
        motor_cmd = MotorCmd()
        motor_cmd.left_motor = 0
        motor_cmd.right_motor = 0
        green_led_cmd = Byte()
        green_led_cmd.data = 0
        while not rospy.is_shutdown():
            print('{:d} {:d} {:f} {:f}'.format(motor_cmd.left_motor, motor_cmd.right_motor,
                                               current_left_motor_rate, current_right_motor_rate))
            green_led_pub.publish(green_led_cmd)
            motor_cmd.header.stamp = rospy.Time.now()
            motor_cmd_pub.publish(motor_cmd)
            motor_cmd.left_motor += 10
            motor_cmd.right_motor += 10
            green_led_cmd.data = green_led_cmd.data == 0
            if motor_cmd.left_motor > motor_cmd_scale or motor_cmd.right_motor > motor_cmd_scale:
                break
            rate.sleep()

        while not rospy.is_shutdown():
            print('{:d} {:d} {:f} {:f}'.format(motor_cmd.left_motor, motor_cmd.right_motor,
                                               current_left_motor_rate, current_right_motor_rate))
            green_led_pub.publish(green_led_cmd)
            motor_cmd.header.stamp = rospy.Time.now()
            motor_cmd_pub.publish(motor_cmd)
            motor_cmd.left_motor -= 10
            motor_cmd.right_motor -= 10
            green_led_cmd.data = green_led_cmd.data == 0
            if motor_cmd.left_motor <= 0 or motor_cmd.right_motor <= 0:
                break
            rate.sleep()

        motor_cmd.left_motor = 0
        motor_cmd.right_motor = 0
        motor_cmd_pub.publish(motor_cmd)

    except rospy.ROSInterruptException:
        pass
