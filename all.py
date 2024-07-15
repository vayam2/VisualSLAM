#!/usr/bin/env python3
import rospy,cv2,time,dronekit,math
from cv_bridge import CvBridge
from sensor_msgs.msg import Image,Imu
from std_msgs.msg import Float64
from nav_msgs.msg import Odometry
from mavros_msgs.msg import State, PositionTarget
from mavros_msgs.srv import CommandBool, SetMode
from dronekit import connect, VehicleMode, LocationGlobalRelative, LocationGlobal, Command
from pymavlink import mavutil

class Drone:


    def __init__(self):
        rospy.init_node('aruco_detector', anonymous=True)
        self.bridge = CvBridge()
        self.mavros_sub = rospy.Subscriber('/mavros/state', State, self.state_callback)
        self.image_sub = rospy.Subscriber('/roscam/cam/image_raw', Image, self.image_callback)  # Change topic name here
        self.odom_sub = rospy.Subscriber("mavros/imu/data", Imu, self.imudata_callback) 
        self.alt_sub = rospy.Subscriber("/mavros/global_position/rel_alt", Float64, self.altitude_callback) 

        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_5X5_100)
        self.parameters = cv2.aruco.DetectorParameters()
        self.detector = cv2.aruco.ArucoDetector(self.aruco_dict, self.parameters) # declare a detector with the selected params and dict
        self.vehicle = connect('127.0.0.1:14560', wait_ready=True)  # Replace with your connection string

        self.takeoff_alti = 1.5

        self.state=None
        self.imu=None
        self.altitude=0

############################################################################################################################################
# ###########################################################   CALLBACKS    ###############################################################
# ##########################################################################################################################################
        
    def state_callback(self,data):
        # rospy.loginfo("State Data Received:\n%s", data)
        rospy.loginfo("State Data Received")
        self.State=data

    def image_callback(self, data):
        cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        height, width, _ = cv_image.shape  # Get frame size
        #rospy.loginfo("Frame size: {} x {}".format(width, height))
        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        # corners, ids, rejectedImgPoints = cv2.aruco.detectMarkers(gray, self.aruco_dict, parameters=self.parameters)
        corners, ids, rejectedImgPoints = self.detector.detectMarkers(gray)
        
        if ids is None:
            self.flag = 1

        if ids is not None:
            rospy.loginfo("Detected ArUco markers: {}".format(ids))
            for i in range(len(ids)):
                marker_id = ids[i]

                if marker_id == 1:
                    self.send_velocities(0, 1, 0)

                elif marker_id == 2:
                    # self.vehicle.mode = VehicleMode("LAND")
                    self.send_velocities(0, 0, 1)
        
    
    def imudata_callback(self,data):
        # This function will be called whenever new odometry data is received
        #rospy.loginfo("IMU Data Received:\n%s", data)
        # rospy.loginfo("IMU Data Received")
        self.imu=data
        self.orientation = data.orientation
        # Convert quaternion to Euler angles
        self.yaw = math.atan2(2 * (self.orientation.w * self.orientation.z + self.orientation.x * self.orientation.y),
                     1 - 2 * (self.orientation.y ** 2 + self.orientation.z ** 2))
        print(" yaw ",self.yaw)

    def altitude_callback(self,data):
        # This function will be called whenever new altitude data is received
        rospy.loginfo("Altitude Data Received:\n%s", data)
        self.altitude=data.data

############################################################################################################################################
########################################################### DRONEKIT FUNCTIONS #############################################################
############################################################################################################################################

    def arm_drone(self):
        rospy.wait_for_service('/mavros/cmd/arming')
        try:
            arm_service = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
            response = arm_service(True)
            if response.success:
                rospy.loginfo("Drone armed successfully.")
            else:
                rospy.logerr("Failed to arm the drone")
        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: %s", e)
            
    def set_mode(self, mode):
        rospy.wait_for_service('/mavros/set_mode')
        try:
            set_mode = rospy.ServiceProxy('/mavros/set_mode', SetMode)
            response = set_mode(custom_mode=mode)
            if response.mode_sent:
                rospy.loginfo("Mode set to %s successfully.", mode)
            else:
                rospy.logerr("Failed to set mode to %s", mode)
        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: %s", e)

    def arm_and_takeoff_nogps(self):
        """
        Arms vehicle and fly to self.takeoff_alti without GPS data.
        """

        ##### CONSTANTS #####
        DEFAULT_TAKEOFF_THRUST = 0.7
        SMOOTH_TAKEOFF_THRUST = 0.6

        print("Basic pre-arm checks")
        # Don't let the user try to arm until autopilot is ready
        # If you need to disable the arming check,
        # just comment it with your own responsibility.
        # while not vehicle.is_armable:
        #     print(" Waiting for vehicle to initialise...")
        #     time.sleep(1)


        print("Arming motors")
        # Copter should arm in GUIDED_NOGPS mode

        # vehicle.mode = VehicleMode("GUIDED_NOGPS")
        detector.set_mode('GUIDED_NOGPS')
        # vehicle.armed = True
        detector.arm_drone()


        # while not self.state.armed:
        #     print(" Waiting for arming...")
        #     detector.arm_drone()
        #     time.sleep(1)

        print("Taking off!")

        thrust = DEFAULT_TAKEOFF_THRUST
        while True:
            current_altitude = self.altitude
            print(" Altitude: %f  Desired: %f" %
                  (current_altitude, self.takeoff_alti))
            if current_altitude >= self.takeoff_alti*0.95: # Trigger just below target alt.
                print("Reached target altitude")
                break
            elif current_altitude >= self.takeoff_alti*0.6:
                thrust = SMOOTH_TAKEOFF_THRUST
            detector.set_attitude(thrust = thrust)
            time.sleep(0.2)

    def send_attitude_target(self, roll_angle = 0.0, pitch_angle = 0.0,
                             yaw_angle = None, yaw_rate = 0.0, use_yaw_rate = False,
                             thrust = 0.5):
        """
        use_yaw_rate: the yaw can be controlled using yaw_angle OR yaw_rate.
                      When one is used, the other is ignored by Ardupilot.
        thrust: 0 <= thrust <= 1, as a fraction of maximum vertical thrust.
                Note that as of Copter 3.5, thrust = 0.5 triggers a special case in
                the code for maintaining current altitude.
        """
        if yaw_angle is None:
            # this value may be unused by the vehicle, depending on use_yaw_rate
            yaw_angle = self.yaw
        # Thrust >  0.5: Ascend
        # Thrust == 0.5: Hold the altitude
        # Thrust <  0.5: Descend
        msg = self.vehicle.message_factory.set_attitude_target_encode(
            0, # time_boot_ms
            1, # Target system
            1, # Target component
            0b00000000 if use_yaw_rate else 0b00000100,
            detector.to_quaternion(roll_angle, pitch_angle, yaw_angle), # Quaternion
            0, # Body roll rate in radian
            0, # Body pitch rate in radian
            math.radians(yaw_rate), # Body yaw rate in radian/second
            thrust  # Thrust
        )
        self.vehicle.send_mavlink(msg)
    
    def set_attitude(self, roll_angle = 0.0, pitch_angle = 0.0,
                     yaw_angle = None, yaw_rate = 0.0, use_yaw_rate = False,
                     thrust = 0.5, duration = 0):
        """
        Note that from AC3.3 the message should be re-sent more often than every
        second, as an ATTITUDE_TARGET order has a timeout of 1s.
        In AC3.2.1 and earlier the specified attitude persists until it is canceled.
        The code below should work on either version.
        Sending the message multiple times is the recommended way.
        """
        detector.send_attitude_target(roll_angle, pitch_angle,
                             yaw_angle, yaw_rate, False,
                             thrust)
        start = time.time()
        while time.time() - start < duration:
            self.send_attitude_target(roll_angle, pitch_angle,
                                 yaw_angle, yaw_rate, False,
                                 thrust)
            time.sleep(0.1)
        # Reset attitude, or it will persist for 1s more due to the timeout
        detector.send_attitude_target(0, 0,
                             0, 0, True,
                             thrust)
    
    def to_quaternion(self,roll = 0.0, pitch = 0.0, yaw = 0.0):
        """
        Convert degrees to quaternions
        """
        t0 = math.cos(math.radians(yaw * 0.5))
        t1 = math.sin(math.radians(yaw * 0.5))
        t2 = math.cos(math.radians(roll * 0.5))
        t3 = math.sin(math.radians(roll * 0.5))
        t4 = math.cos(math.radians(pitch * 0.5))
        t5 = math.sin(math.radians(pitch * 0.5))
    
        w = t0 * t2 * t4 + t1 * t3 * t5
        x = t0 * t3 * t4 - t1 * t2 * t5
        y = t0 * t2 * t5 + t1 * t3 * t4
        z = t1 * t2 * t4 - t0 * t3 * t5
    
        return [w, x, y, z]
 
    def send_velocities(self, velocities:list):
        print(f"Velocities sent to the drone: {velocities[0]} m/s, {velocities[1]} m/s, {velocities[2]} m/s")
        message = self.vehicle.message_factory.set_position_target_local_ned_encode(
            0,       # time_boot_ms (not used)
            0, 0,    # target system, target component
            mavutil.mavlink.MAV_FRAME_LOCAL_NED, # frame
            0b0000111111000111, # type_mask (only speeds enabled)
            0, 0, 0, # x, y, z positions (not used)
            velocities[0],velocities[1], velocities[2], # x, y, z velocity in m/s
            0, 0, 0, # x, y, z acceleration (not supported yet, ignored in GCS_Mavlink)
            0, 0)    # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink)
        self.vehicle.send_mavlink(message)


if __name__ == '__main__':
    try:
        detector = Drone()
        # detector.arm_and_takeoff_nogps(5)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass