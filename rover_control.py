import rospy
from mavros_msgs.msg import OverrideRCIn
from mavros_msgs.srv import CommandBool
import time

class Rover():

    def __init__(self) -> None:

        # rospy.init_node('control_test', anonymous=True)
        # self.rc_override = rospy.Publisher('mavros/rc/override', OverrideRCIn)

        self._throttle_channel = 1
        self._steering_channel = 0

        self.speeds = ["SLOW", "MEDIUM", "FAST"]

    def arm(self):
        rospy.wait_for_service('/mavros/cmd/arming')
        try:
            armService = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
            armResponse = armService(True)
            rospy.loginfo(armResponse)
        except rospy.ServiceException as e:
            print("Service call failed: %s" %e)
        time.sleep(1)

    def disarm(self):
        rospy.wait_for_service('/mavros/cmd/arming')
        try:
            armService = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
            armResponse = armService(False)
            rospy.loginfo(armResponse)
        except rospy.ServiceException as e:
            print("Service call failed: %s" %e)

    def move_forward(self, speed):
        msg = OverrideRCIn()

        if speed == 'MEDIUM':
            msg.channels[self._throttle_channel] = 1800
        elif speed == 'FAST':
            msg.channels[self._throttle_channel] = 2000
        else:
            msg.channels[self._throttle_channel] = 1650

        self.rc_override.publish(msg)

    def move_backward(self, speed):
        msg = OverrideRCIn()

        if speed == 'MEDIUM':
            msg.channels[self._throttle_channel] = 1200
        elif speed == 'FAST':
            msg.channels[self._throttle_channel] = 1000
        else:
            msg.channels[self._throttle_channel] = 1350

        self.rc_override.publish(msg)

    def rotate_left(self, speed):
        msg = OverrideRCIn()

        if speed == 'MEDIUM':
            msg.channels[self._steering_channel] = 1200
        elif speed == 'FAST':
            msg.channels[self._steering_channel] = 1000
        else:
            msg.channels[self._steering_channel] = 1350

        self.rc_override.publish(msg)

    def rotate_right(self, speed):
        msg = OverrideRCIn()

        if speed == 'MEDIUM':
            msg.channels[self._steering_channel] = 1800
        elif speed == 'FAST':
            msg.channels[self._steering_channel] = 2000
        else:
            msg.channels[self._steering_channel] = 1650

        self.rc_override.publish(msg)

    def stop(self):
        msg = OverrideRCIn()

        msg.channels[self._steering_channel] = 1500
        msg.channels[self._throttle_channel] = 1500

        self.rc_override.publish(msg)

if __name__ == '__main__':
    myRover = Rover()

    print("Arming Rover...")
    myRover.arm()

    try:
        print("Moving forward")
        while True:
            myRover.move_forward(speed=myRover.speeds[2])
    except:
        print("Exiting...")
        myRover.stop()
        time.sleep(1)
            