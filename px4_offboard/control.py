#!/usr/bin/env python
############################################################################
#
#   Copyright (C) 2022 PX4 Development Team. All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
# 1. Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in
#    the documentation and/or other materials provided with the
#    distribution.
# 3. Neither the name PX4 nor the names of its contributors may be
#    used to endorse or promote products derived from this software
#    without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
# OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
# AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
############################################################################

__author__ = "Braden Wagstaff"
__contact__ = "braden@arkelectron.com"

import rclpy
from rclpy.node import Node
import numpy as np

import serial

from rclpy.clock import Clock
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy

from px4_msgs.msg import VehicleStatus
#from px4_msgs.msg import VehicleAttitude
from px4_msgs.msg import VehicleLocalPosition


from geometry_msgs.msg import Twist, Vector3
from math import pi
from std_msgs.msg import Bool

from pyudev import Context

def find_serial_port():
    context = Context()
    for device in context.list_devices(subsystem='tty'):
        if 'ID_VENDOR_ID' in device.properties and device.properties['ID_VENDOR_ID'] == '1a86':
            if 'ID_MODEL_ID' in device.properties and device.properties['ID_MODEL_ID'] == '7523':
                return device.device_node

serial_port = find_serial_port()
while not serial_port:
    serial_port = find_serial_port()

ser = serial.Serial(serial_port, baudrate=9600)



class kmsenseControl(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )

        #Create subscriptions
        self.status_sub = self.create_subscription(
            VehicleStatus,
            '/fmu/out/vehicle_status',
            self.vehicle_status_callback,
            qos_profile)

        """self.attitude_sub = self.create_subscription(
            VehicleAttitude,
            '/fmu/out/vehicle_attitude',
            self.attitude_callback,
            qos_profile)"""
        
        self.vlp_sub = self.create_subscription(
            VehicleLocalPosition,
            '/fmu/out/vehicle_local_position',
            self.local_position_callback,
            qos_profile)
        
        """self.velctl_sub = self.create_subscription(
            Twist,
            '/kmsense_feedback',
            self.velctl_callback,qos_profile)"""


        self.ser = serial.Serial(serial_port, baudrate=9600)

        self.ser.read_until(b'\n').decode('utf-8')
        #Create publishers
        self.publisher_position = self.create_publisher(Twist, '/kmsense_velocity_cmd', qos_profile)
        
        timer1_period = 1.0  # seconds
        self.timer = self.create_timer(timer1_period, self.send_speed_callback)
        
        timer2_period = 0.02  # seconds
        self.timer = self.create_timer(timer2_period, self.offboard_position_callback)
        self.counter=0
        self.arm_state = -1
        self.last_arming_state = -1
        #self.yaw_bias=0.0
        #self.trueYaw=0.0
        self.groundspeed=0.0
        self.ser = serial.Serial(serial_port, baudrate=9600)

        #received_data = self.ser.read_until(b'\n').decode('utf-8') ## if speed is low, one of them will be delete
        #received_data = self.ser.read_until(b'\n').decode('utf-8')
        self.set_sensor('H')
        received_data = self.ser.read_until(b'\n').decode('utf-8') ## if speed is low, one of them will be delete
        received_data = self.ser.read_until(b'\n').decode('utf-8')
        self.get_logger().info(str(received_data.split("|")[0]))
        #self.set_sensor('Z')
        self.send_speed=1.0
    def set_sensor(self,parameter):                                      # Send to Sensor & Log function
        #f = open(file_name , 'a')
        self.ser = serial.Serial(serial_port, baudrate=9600)                  #Open port with baud rate | Sensor Port
        self.ser.write((parameter+"*/").encode())                      # P:pos Z:angle C:angular.velocity
        #self.get_logger().info(f"####################################Written to Sensor: {parameter}")
        self.ser.read_until(b'\n').decode('utf-8')
        #stamped_data = str(datetime.now())+"|"
        #stamped_data += parameter+"*/"
        #f.write(stamped_data+"\n")
        #print("Written to Sensor: ",(parameter+"\r\n").encode())    # 
                                                                    #receives and sets vehicle status values 
    def vehicle_status_callback(self, msg):
        #self.get_logger().info(f"NAV_STATUS: {msg.nav_state}")
        #self.get_logger().info(f"ARM STATUS: {msg.arming_state}")
        # self.get_logger().info(f"FlightCheck: {msg.pre_flight_checks_pass}")
        if (self.last_arming_state != msg.arming_state and self.last_arming_state!=-1 ):
            #self.yaw_bias=(self.trueYaw*180/np.pi+360)%360
            self.set_sensor('P')
            
        self.last_arming_state=msg.arming_state
        

    """def attitude_callback(self, msg):
        orientation_q = msg.q

        #trueYaw is the drones current yaw value
        self.trueYaw = -(np.arctan2(2.0*(orientation_q[3]*orientation_q[0] + orientation_q[1]*orientation_q[2]), 
                                  1.0 - 2.0*(orientation_q[0]*orientation_q[0] + orientation_q[1]*orientation_q[1])))"""
    
    def local_position_callback(self, msg):
        self.groundspeed=np.sqrt(msg.vx * msg.vx + msg.vy * msg.vy)
    
    """def velctl_callback(self, msg):
        self.send_speed = msg.angular.z"""
    
    #publishes offboard control modes and velocity as trajectory setpoints
    def send_speed_callback(self):
        #self.get_logger().info(f"SPEED: {self.groundspeed}")
        self.set_sensor("A"+str(float("{:.1f}".format(self.groundspeed))))

    def offboard_position_callback(self):
        self.ser = serial.Serial(serial_port, baudrate=9600)
        received_data = self.ser.read_until(b'\n').decode('utf-8') ## if speed is low, one of them will be delete
        received_data = self.ser.read_until(b'\n').decode('utf-8')
        #print(float(received_data.split("|")[0]))
        twist = Twist()



        if(len(received_data.split("|"))==9):
            sensor_x=float(received_data.split("|")[4])
            sensor_y=float(received_data.split("|")[5])
            sensor_yaw=float(received_data.split("|")[8]) # NED frame
            fc_yaw_local=  (sensor_yaw)%360
            if sensor_x>=0:
                if sensor_y>=0:
                    dest_yaw=((np.arctan2(sensor_x,sensor_y)*180/np.pi + 90)%360)
                else:
                    dest_yaw=((np.arctan2(-sensor_y,sensor_x)*180/np.pi + 180)%360)
                
            else:
                if sensor_y>=0:
                    dest_yaw=((np.arctan2(sensor_y,-sensor_x)*180/np.pi )%360)
                else:
                    dest_yaw=((np.arctan2(-sensor_x,-sensor_y)*180/np.pi + 270)%360)
            
            set_yaw=0    
            if (dest_yaw-fc_yaw_local)>0:
                if (dest_yaw-fc_yaw_local)<180:
                    set_yaw=dest_yaw-fc_yaw_local
                else:
                    set_yaw=(dest_yaw-fc_yaw_local)-360.0
            else:
                if abs(dest_yaw-fc_yaw_local)<180:
                    set_yaw=dest_yaw-fc_yaw_local
                else:
                    set_yaw=(dest_yaw-fc_yaw_local)+360.0
            
            distance=np.sqrt(sensor_x*sensor_x+sensor_y*sensor_y)
            

            self.get_logger().info(f"XYy: {sensor_x,sensor_y,set_yaw,dest_yaw,fc_yaw_local}")

            twist.linear.x = float(distance) # used for transfer distance # n_val ## NED
            twist.linear.y = 0.0# e_val ## NED
            twist.linear.z = 0.0
            twist.angular.x = 0.0
            twist.angular.y = 0.0
            twist.angular.z = set_yaw
            self.publisher_position.publish(twist)
            """if self.counter>10:
                stamped_data = str(datetime.now())+"|"
                stamped_data += received_data
                stamped_data = stamped_data.replace('|', ',')
                f = open(file_name, 'a')                                                # Log stamped data
                f.write(str(stamped_data)+"\n")
                self.counter=0
            self.counter=self.counter+1"""
        #else:
            #self.get_logger().info(f"XYy:Reject")

def main(args=None):
    rclpy.init(args=args)

    kmsense_control = kmsenseControl()

    rclpy.spin(kmsense_control)

    kmsense_control.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    try:
        main()
    except Exception as e:
        print(e)