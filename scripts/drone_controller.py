#!/usr/bin/env python
import rospy
from dronekit import connect, VehicleMode
from pymavlink import mavutil
from geometry_msgs.msg import Twist
from drone_autonomous.msg import ObjectPosition
from std_msgs.msg import Bool

class DroneController:
    def __init__(self):
        rospy.init_node('drone_controller')
        
        # Parameters
        self.connection_string = rospy.get_param('~connection_string', '/dev/ttyACM0')
        self.default_altitude = rospy.get_param('~default_altitude', 5.0)  # meters
        
        # Drone connection
        self.vehicle = connect(self.connection_string, wait_ready=True)
        
        # Publishers and Subscribers
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.landing_pub = rospy.Publisher('/landing_status', Bool, queue_size=1)
        self.object_sub = rospy.Subscriber('/object_position', ObjectPosition, self.object_callback)
        
        # Control parameters
        self.k_p = 0.01  # Proportional gain for position correction
        self.landing_speed = 0.3  # m/s
        
    def arm_and_takeoff(self, altitude):
        rospy.loginfo("Arming and taking off")
        self.vehicle.mode = VehicleMode("GUIDED")
        self.vehicle.armed = True
        
        while not self.vehicle.armed:
            rospy.sleep(0.1)
        
        self.vehicle.simple_takeoff(altitude)
        
        while self.vehicle.location.global_relative_frame.alt < altitude*0.95:
            rospy.sleep(0.5)
        
        rospy.loginfo("Reached target altitude")
        
    def goto_position(self, lat, lon, alt):
        rospy.loginfo(f"Going to position: {lat}, {lon} at {alt}m")
        target = LocationGlobalRelative(lat, lon, alt)
        self.vehicle.simple_goto(target)
        
    def object_callback(self, msg):
        if not msg.detected:
            return
            
        cmd = Twist()
        
        if msg.centered:
            # Descend
            cmd.linear.z = -self.landing_speed
            rospy.loginfo("Object centered - descending")
        else:
            # Adjust position
            cmd.linear.x = -msg.dy * self.k_p
            cmd.linear.y = -msg.dx * self.k_p
            rospy.loginfo(f"Adjusting position: dx={msg.dx:.1f}, dy={msg.dy:.1f}")
        
        self.cmd_vel_pub.publish(cmd)
        
    def land(self):
        rospy.loginfo("Landing")
        self.vehicle.mode = VehicleMode("LAND")
        self.landing_pub.publish(True)

    def send_nav_command(self, vx, vy, vz, rot):
        if self.simulation_mode:
            # Untuk simulasi menggunakan Twist
            twist = Twist()
            twist.linear.x = vx
            twist.linear.y = vy
            twist.linear.z = vz
            twist.angular.z = rot
            self.cmd_vel_pub.publish(twist)
        else:
            # Implementasi asli untuk drone fisik
            pass
        
    def run(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            rate.sleep()
        
        self.vehicle.close()

if __name__ == '__main__':
    dc = DroneController()
    dc.run()