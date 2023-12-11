import rospy
from mavros_msgs.msg import OverrideRCIn
from mavros_msgs.srv import CommandBool
import time

global throttle_channel
global steering_channel
throttle_channel = 1
steering_channel = 0

def move_forward(pub, speed):
    msg = OverrideRCIn()

    if speed == 'MEDIUM':
        msg.channels[throttle_channel] = 1800
    elif speed == 'FAST':
        msg.channels[throttle_channel] = 2000
    else:
        msg.channels[throttle_channel] = 1650

    pub.publish(msg)

def move_backward(pub, speed):
    msg = OverrideRCIn()

    if speed == 'MEDIUM':
        msg.channels[throttle_channel] = 1200
    elif speed == 'FAST':
        msg.channels[throttle_channel] = 1000
    else:
        msg.channels[throttle_channel] = 1350

    pub.publish(msg)

def rotate_left(pub, speed):
    msg = OverrideRCIn()

    if speed == 'MEDIUM':
        msg.channels[steering_channel] = 1200
    elif speed == 'FAST':
        msg.channels[steering_channel] = 1000
    else:
        msg.channels[steering_channel] = 1350

    pub.publish(msg)

def rotate_right(pub, speed):
    msg = OverrideRCIn()

    if speed == 'MEDIUM':
        msg.channels[steering_channel] = 1800
    elif speed == 'FAST':
        msg.channels[steering_channel] = 2000
    else:
        msg.channels[steering_channel] = 1650

    pub.publish(msg)

def stop(pub):
    msg = OverrideRCIn()

    msg.channels[steering_channel] = 1500
    msg.channels[throttle_channel] = 1500

    pub.publish(msg)


if __name__ == '__main__':

    speeds = ["SLOW", "MEDIUM", "FAST"]

    rospy.init_node('control_test', anonymous=True)
    pub = rospy.Publisher('mavros/rc/override', OverrideRCIn)

    rospy.wait_for_service('/mavros/cmd/arming')
    try:
        armService = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
        armResponse = armService(True)
        rospy.loginfo(armResponse)
    except rospy.ServiceException as e:
        print("Service call failed: %s" %e)

    time.sleep(1)

    while True:

    # print("Moving forward...")

        move_forward(pub, speeds[1])
        time.sleep(1)

    # stop(pub)

    # print("Moving backward...")

    # move_backward(pub, speeds[1])

    # time.sleep(5)

    # print("Moving right...")

    # rotate_right(pub, speeds[1])

    # time.sleep(5)


    # print("Moving left...")

    # rotate_left(pub, speeds[1])

    # time.sleep(5)