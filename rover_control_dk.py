from dronekit import connect, VehicleMode,LocationGlobalRelative
import time
from pymavlink import mavutil

class Rover():
    def __init__(self) -> None:
        self.rate = 1
        self.speeds = ["SLOW", "MEDIUM", "FAST"]
        self.current_speed = self.speeds[0]       

    def connect(self, port):
        self.vehicle = connect(port, baud=57600, wait_ready=True)

    def arm(self):
        while self.vehicle.is_armable != True:
            print("Waiting for vehicle to become armable.")
            time.sleep(self.rate)
        print("Arming vehicle...")

        self.vehicle.mode = VehicleMode("GUIDED")

        while self.vehicle.mode != 'GUIDED':
            print("Waiting for vehicle to enter Guided Mode.")
            time.sleep(self.rate)
        print("Vehicle is in GUIDED mode.")

        self.vehicle.armed = True
        while self.vehicle.armed == False:
            time.sleep(self.rate)
        print("Vehicle is ARMED.")

    def disarm(self):

        self.vehicle.mode = VehicleMode("GUIDED")
        print("Disarming vehicle...")
        self.vehicle.armed = False
        while self.vehicle.armed == True:
            time.sleep(self.rate)
        print("Vehicle is DISARMED.")

    def send_local_ned_velocity(self, vx, vy, vz):
        msg = self.vehicle.message_factory.set_position_target_local_ned_encode(
            0,
            0, 0,
            mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED,
            0b0000111111000111,
            0, 0, 0,
            vx, vy, vz,
            0, 0, 0,
            0, 0)
        self.vehicle.send_mavlink(msg)
        self.vehicle.flush()

    def reverse_cmd(self, direction):
        # create the CONDITION_YAW command using command_long_encode()
        msg = self.vehicle.message_factory.command_long_encode(
            0, 0, # target system, target component
            mavutil.mavlink.MAV_CMD_DO_SET_REVERSE, #command
            0, #confirmation
            direction, #Param 1, 0 for forward 1 for backward.
            0,  #Param 2, yaw speed deg/s
            0, #Param 3, Direction -1 ccw, 1 cw
            0, # Param 4, relative offset 1, absolute angle 0
            0,0, 0) # Param 5-7 not used
        self.vehicle.send_mavlink(msg)
        self.vehicle.flush()

    ##Send a velocity command with +x being the heading of the drone.
    def send_global_ned_velocity(self, vx, vy, vz):
        msg = self.vehicle.message_factory.set_position_target_local_ned_encode(
            0, # time_boot_ms (not used)
            0, 0, # target system, target component
            mavutil.mavlink.MAV_FRAME_LOCAL_NED, #frame
            0b0000111111000111, #type_mask (only speeds enabled)
            0, 0, 0, # x, y, z positions (not used)
            vx, vy, vz, # x, y, z velocity in m/s
            0, 0, 0, #x, y, z acceleration (not supported yet, ignored in GCS_Mavlink)
            0, 0) #yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink)
        self.vehicle.send_mavlink(msg)
        self.vehicle.flush()

    def manual_forward(self):
        if self.vehicle.mode != 'MANUAL':
            # Switch Rover to Manual Mode
            self.vehicle.mode = VehicleMode("MANUAL")
            while self.vehicle.mode != 'MANUAL':
                time.sleep(self.rate)

        if self.current_speed == 'MEDIUM':
            self.vehicle.channels.overrides = {'2':1800}
        elif self.current_speed == 'FAST':
            self.vehicle.channels.overrides = {'2':2000}
        else:
            self.vehicle.channels.overrides = {'2':1650}
        time.sleep(self.rate)

    def manual_reverse(self):
        if self.vehicle.mode != 'MANUAL':
            # Switch Rover to Manual Mode
            self.vehicle.mode = VehicleMode("MANUAL")
            while self.vehicle.mode != 'MANUAL':
                time.sleep(self.rate)
        if self.current_speed == 'MEDIUM':
            self.vehicle.channels.overrides = {'2':1200}
        elif self.current_speed == 'FAST':
            self.vehicle.channels.overrides = {'2':1000}
        else:
            self.vehicle.channels.overrides = {'2':1350}
        time.sleep(self.rate)

    def manual_turn_right(self):
        if self.vehicle.mode != 'MANUAL':
            # Switch Rover to Manual Mode
            self.vehicle.mode = VehicleMode("MANUAL")
            while self.vehicle.mode != 'MANUAL':
                time.sleep(self.rate)
        if self.current_speed == 'MEDIUM':
            self.vehicle.channels.overrides = {'1':1800}
        elif self.current_speed == 'FAST':
            self.vehicle.channels.overrides = {'1':2000}
        else:
            self.vehicle.channels.overrides = {'1':1650}
        time.sleep(self.rate)
    
    def manual_turn_left(self):
        if self.vehicle.mode != 'MANUAL':
            # Switch Rover to Manual Mode
            self.vehicle.mode = VehicleMode("MANUAL")
            while self.vehicle.mode != 'MANUAL':
                time.sleep(self.rate)
        if self.current_speed == 'MEDIUM':
            self.vehicle.channels.overrides = {'1':1200}
        elif self.current_speed == 'FAST':
            self.vehicle.channels.overrides = {'1':1000}
        else:
            self.vehicle.channels.overrides = {'1':1350}
        time.sleep(self.rate)

    def manual_stop(self):
        if self.vehicle.mode != 'MANUAL':
            # Switch Rover to Manual Mode
            self.vehicle.mode = VehicleMode("MANUAL")
            while self.vehicle.mode != 'MANUAL':
                time.sleep(self.rate)
        self.vehicle.channels.overrides = {'1':1500, '2':1500}

if __name__ == '__main__':
    myRover = Rover()
    myRover.connect(port='/dev/ttyACM0')

    print("Arming Rover...")
    myRover.arm()
    myRover.current_speed = myRover.speeds[1]
    myRover.rate = 0.5

    try:
        print("Moving forward")
        while True:
            myRover.manual_forward()
    except:
        print("Exiting...")
        myRover.manual_stop()
        myRover.disarm()
        time.sleep(1)
            
