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
from rclpy.clock import Clock
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy


from px4_msgs.msg import OffboardControlMode
#from px4_msgs.msg import TrajectorySetpoint
from px4_msgs.msg import VehicleRatesSetpoint
from px4_msgs.msg import VehicleStatus
from px4_msgs.msg import VehicleLocalPosition
from px4_msgs.msg import VehicleAttitude
from px4_msgs.msg import VehicleCommand
from px4_msgs.msg import SensorGps
from px4_msgs.msg import InputRc
from px4_msgs.msg import ManualControlSetpoint
from px4_msgs.msg import TimesyncStatus
#from px4_msgs.msg import HoverThrustEstimate
#from px4_msgs.msg import AirspeedValidated


from geometry_msgs.msg import Twist, Vector3
from math import pi
from std_msgs.msg import Bool

class OffboardControl(Node):

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
            self.vehicle_status_callback,qos_profile)
        
        self.kmsense_velocity_sub = self.create_subscription(
            Twist,
            '/kmsense_velocity_cmd',
            self.kmsense_velocity_callback,qos_profile)
        
        self.RC_sub = self.create_subscription(
            InputRc,
            '/fmu/out/input_rc',
            self.rc_callback,qos_profile)

        self.manual_control_sw_sub = self.create_subscription(
              ManualControlSetpoint  ,
            '/fmu/out/manual_control_setpoint',
            self.mcs_callback,qos_profile)
            
        self.vlp_sub = self.create_subscription(
            VehicleLocalPosition,
            '/fmu/out/vehicle_local_position',
            self.local_position_callback,qos_profile)
            
        self.timesync_sub_ = self.create_subscription(
            TimesyncStatus,
            '/fmu/out/timesync_status',
            self.timestamp_callback,qos_profile)
        
        self.attitude_sub = self.create_subscription(
            VehicleAttitude,
            '/fmu/out/vehicle_attitude',
            self.attitude_callback,
            qos_profile)
        
        """self.aspd_sub = self.create_subscription(
            AirspeedValidated,
            '/fmu/out/airspeed_validated',
            self.aspd_callback,
            qos_profile)"""

        """self.hover_thrust_sub = self.create_subscription(
            HoverThrustEstimate,
            '/fmu/out/hover_thrust_estimate',
            self.hoverthrust_callback,
            qos_profile)"""

        #Create publishers
        self.publisher_offboard_mode = self.create_publisher(OffboardControlMode, '/fmu/in/offboard_control_mode',qos_profile)#, qos_profile)
        #self.publisher_velocity = self.create_publisher(Twist, '/fmu/in/in/setpoint_velocity/cmd_vel_unstamped', qos_profile)
        #self.publisher_trajectory = self.create_publisher(TrajectorySetpoint, '/fmu/in/trajectory_setpoint',qos_profile)#, qos_profile)
        self.publisher_rates = self.create_publisher(VehicleRatesSetpoint, '/fmu/in/vehicle_rates_setpoint',qos_profile)#, qos_profile)
        self.vehicle_command_publisher_ = self.create_publisher(VehicleCommand, "/fmu/in/vehicle_command", 10)
        """self.kmsense_speed_publisher = self.create_publisher(Twist, "/kmsense_feedback", qos_profile)"""

        
        #creates callback function for the arm timer
        # period is arbitrary, just should be more than 2Hz
        arm_timer_period = 0.1 # seconds
        self.arm_timer_ = self.create_timer(arm_timer_period, self.offboard_callback)

        # creates callback function for the command loop
        # period is arbitrary, just should be more than 2Hz. Because live controls rely on this, a higher frequency is recommended
        # commands in cmdloop_callback won't be executed if the vehicle is not in offboard mode
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.cmdloop_callback)
        #self.nav_state = VehicleStatus.NAVIGATION_STATE_MAX
        self.arm_state = VehicleStatus.ARMING_STATE_INIT
        self.distance = 0.0
        self.th_rate = -0.56    #-0.75 -0.648
        self.hover_thrust=0.56  #0.75  0.648
        self.timestamp=0
        self.sticks_moving = False
        self.yaw_diff = 0.0  #yaw value we send as command
        self.true_roll=0.0
        self.true_pitch=0.0
        self.true_yaw=0.0
        self.counter_land = 0
        self.counter_climb = 0
        self.counter_eq=0
        self.offboard_mode = 0
        self.last_offboard_mode =-1
        #self.reset_rate_integral=False
        self.altitude_set_point_delta=-15
        self.altitude_set_point=-100
        self.altitude=0
        self.last_altitude=0
        

        self.jamming_sim_switch		=False

        #states with corresponding callback functions that run once when state switches
        self.states = {
            "IDLE": self.state_init,
            "OFFBOARD": self.state_offboard,
            "MANUAL": self.state_manual,
            "HOLD": self.state_loiter
            
        }
        self.current_state = "IDLE"

        

    """def gps_callback(self,msg):
        
        #self.spoofing_state = msg.spoofing_state"""



    def local_position_callback(self, msg):
        self.altitude=msg.z
        #self.groundspeed=np.sqrt(msg.vx * msg.vx + msg.vy * msg.vy)
    """def aspd_callback(self, msg):
        self.get_logger().info(f"ASPD:ind-true|GSPD: {msg.indicated_airspeed_m_s,msg.true_airspeed_m_s,self.groundspeed}")
    """
    def timestamp_callback(self, msg):
        self.timestamp=msg.timestamp

    def rc_callback(self,msg):
        self.jamming_sim_switch	=True if msg.values[4]>1500 else False
        """if(self.jamming_sim_switch):
            self.get_logger().info(f"jamming on")
        else:
            self.get_logger().info(f"jamming off")"""
        

    def mcs_callback(self,msg):
        self.sticks_moving = msg.sticks_moving
        

    #callback function that arms, takes off, and switches to offboard mode
    #implements a finite state machine
    def offboard_callback(self):
        """self.get_logger().info(f"navst: {self.nav_state}")"""
        #self.get_logger().info(f"OJNAS_NEY: {self.yaw,self.distance,self.current_state,self.jamming_sim_switch,self.nstates[self.nav_state],self.arm_state,self.sticks_moving}")
        
        if(self.jamming_sim_switch ):
                self.current_state = "OFFBOARD"

        if (self.current_state== "OFFBOARD"):
            self.state_offboard()
            if self.sticks_moving:
                self.current_state = "MANUAL"
                self.get_logger().info(f"Pilot took over control")
                self.state_manual()
            if (not self.jamming_sim_switch ):
                self.current_state = "HOLD"
                self.get_logger().info("Offboard Finish"+str(self.hover_thrust))
                self.state_loiter()
        elif (self.current_state== "MANUAL"):
            self.state_manual()
        elif (self.current_state== "HOLD"):
            self.state_loiter()
            self.current_state = "IDLE"
        elif (self.current_state== "IDLE"):
            self.state_init()
            self.current_state = "IDLE"
        
        
    def state_init(self):
        self.offboard_mode=0

    def state_offboard(self):
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.,6.)
        self.offboard_mode = 1
    
    def state_manual(self):
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1., 1.)
        self.offboard_mode = 0
    def state_loiter(self):
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_LOITER_UNLIM  , param7=25.0)#(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1., 4.)# 4 for altctl #(VehicleCommand.VEHICLE_CMD_NAV_LOITER_UNLIM  , param7=5.0)
        self.offboard_mode = 0

    #publishes command to /fmu/in/vehicle_command
    def publish_vehicle_command(self, command, param1=0.0, param2=0.0, param7=0.0):
        msg = VehicleCommand()
        msg.param1 = param1
        msg.param2 = param2
        msg.param7 = param7    # altitude value in takeoff command
        msg.command = command  # command ID
        msg.target_system = 1  # system which should execute the command
        msg.target_component = 1  # component which should execute the command, 0 for all components
        msg.source_system = 1  # system sending the command
        msg.source_component = 1  # component sending the command
        msg.from_external = True
        msg.timestamp = self.timestamp #int(Clock().now().nanoseconds / 1000) # time in microseconds
        self.vehicle_command_publisher_.publish(msg)
    
    #receives and sets vehicle status values 
    def vehicle_status_callback(self, msg):
        #self.nav_state = msg.nav_state
        self.arm_state = msg.arming_state

    """def hoverthrust_callback(self,msg):
        if msg.hover_thrust!='nan':
            self.hover_thrust=float(msg.hover_thrust)
        self.get_logger().info("thrust: "+str(self.hover_thrust)+str(msg.hover_thrust))"""
    #receives Twist commands from Teleop and converts NED -> FLU
    def kmsense_velocity_callback(self, msg):
        #implements NED -> FLU Transformation
        self.distance = msg.linear.x
        #self.velocity_e = msg.linear.y
        #self.velocity_d = msg.linear.z
        self.yaw_diff = msg.angular.z
        #self.get_logger().info(f"XYy: {self.yaw_diff}")

    #receives current trajectory values from drone and grabs the yaw value of the orientation
    def attitude_callback(self, msg):
        q = msg.q
        w=q[0]
        x=q[1]
        y=q[2]
        z=q[3]
        self.true_roll = np.arctan2(2 * (w * x + y * z), 1 - 2 * (x**2 + y**2))
        self.true_pitch = np.arcsin(2 * (w * y - z * x))
        self.true_yaw = np.arctan2(2 * (w * z + x * y), 1 - 2 * (y**2 + z**2))

        
    #publishes offboard control modes and velocity as trajectory setpoints
    def publish_rate_message(self,yw_rate,pt_rate):
        rates_message=VehicleRatesSetpoint()
        rates_message.timestamp = self.timestamp #int(Clock().now().nanoseconds / 1000)
        rates_message.roll  = -self.true_roll
        rates_message.pitch = pt_rate
        rates_message.yaw   = yw_rate

        rates_message.thrust_body	[0] = 0.0
        rates_message.thrust_body	[1] = 0.0
        if self.th_rate < -1.0:
            rates_message.thrust_body	[2] = -1.0
        else:
            rates_message.thrust_body	[2] = self.th_rate
        #rates_message.reset_integral    = self.reset_rate_integral
        self.publisher_rates.publish(rates_message)
    """def speed_publisher(self,send_speed):
        twist = Twist()
        twist.linear.x = 0.0 # used for transfer distance # n_val ## NED
        twist.linear.y = 0.0# e_val ## NED
        twist.linear.z = 0.0
        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = float(send_speed)
        self.kmsense_speed_publisher.publish(twist)"""
    
    def cmdloop_callback(self):
        if (self.last_offboard_mode != self.offboard_mode and self.last_offboard_mode!=-1 ):
            self.altitude_set_point=self.altitude+self.altitude_set_point_delta
            self.get_logger().info("Offboard Switched"+str(self.altitude_set_point))
        self.last_offboard_mode=self.offboard_mode
        if(self.offboard_mode == 1):
            # Publish offboard control modes
            offboard_msg = OffboardControlMode()
            offboard_msg.timestamp = self.timestamp #int(Clock().now().nanoseconds / 1000)
            offboard_msg.position = False
            offboard_msg.velocity = False
            offboard_msg.acceleration = False
            offboard_msg.attitude = False
            offboard_msg.body_rate = True
            
            self.publisher_offboard_mode.publish(offboard_msg)            
            
            rates_message=VehicleRatesSetpoint()
            rates_message.timestamp = self.timestamp #int(Clock().now().nanoseconds / 1000)
            altitude_difference=self.altitude-self.altitude_set_point   # + -> drone below sp.
                                                                        # - -> drone above sp.

            if(altitude_difference>=0): # drone below setpoint
                if self.last_altitude<self.altitude: #drone land
                    self.counter_land=self.counter_land + 1
                    if self.counter_land >3:
                        self.hover_thrust=self.hover_thrust+0.01
                        self.counter_land = 0
                else:
                    self.counter_land = 0
                thrust_rate=(-1*self.hover_thrust)*(1.0+abs(altitude_difference/self.altitude_set_point_delta))
                self.th_rate=thrust_rate
                self.publish_rate_message(0.0,-self.true_pitch)
                self.get_logger().info("Taking Off")
                self.last_altitude=self.altitude
                #self.speed_publisher(1.0)
                return 0

            if(abs(self.yaw_diff)>10):
                #self.speed_publisher(0.0)
                if self.last_altitude>(self.altitude): #drone climbing
                    self.counter_land = 0
                    self.counter_eq=0
                    self.counter_climb=self.counter_climb + 1
                    if self.counter_climb >5:
                        self.hover_thrust=self.hover_thrust-0.01
                        self.counter_climb = 0
                elif self.last_altitude<self.altitude: #drone land
                    self.counter_climb = 0
                    self.counter_eq=0
                    self.counter_land=self.counter_land + 1
                    if self.counter_land >3:
                        self.hover_thrust=self.hover_thrust+0.01
                        self.counter_land = 0
                else:
                    if self.counter_eq >3:
                        self.counter_climb = 0   
                        self.counter_land = 0
                        self.counter_eq=0

                self.th_rate=(-1*self.hover_thrust)
                if(self.yaw_diff>0):
                    self.publish_rate_message(0.8,-self.true_pitch)
                else:
                    self.publish_rate_message(-0.8,-self.true_pitch)
                self.get_logger().info("Yaw Settling"+str(self.yaw_diff))
                self.last_altitude=self.altitude
                return 0
            
            #self.speed_publisher(1.0)
            if self.last_altitude>(self.altitude): #drone climbing
                self.counter_land = 0
                self.counter_eq=0
                self.counter_climb=self.counter_climb + 1
                if self.counter_climb >5:
                    self.hover_thrust=self.hover_thrust-0.01
                    self.counter_climb = 0
            elif self.last_altitude<self.altitude: #drone land
                self.counter_climb = 0
                self.counter_eq=0
                self.counter_land=self.counter_land + 1
                if self.counter_land >3:
                    self.hover_thrust=self.hover_thrust+0.01
                    self.counter_land = 0
            else:
                self.counter_eq=self.counter_eq + 1
                if self.counter_eq >3:
                    self.counter_climb = 0   
                    self.counter_land = 0
                    self.counter_eq=0
            self.th_rate=(-1*self.hover_thrust)

            yw_rate=2*self.yaw_diff*np.pi/180
            pitch_rate=0.0
            if self.distance>10.0:
                if self.true_pitch> -0.10:
                    pitch_rate=-0.15
                elif self.true_pitch> -0.15:
                    pitch_rate=-0.05
                else:
                    pitch_rate=+0.05           
            elif self.distance>7.0:
                #self.get_logger().info(str(self.true_pitch))
                if self.true_pitch> -0.05:
                    pitch_rate=-0.15
                elif self.true_pitch> -0.10:
                    pitch_rate=-0.05
                else:
                    pitch_rate=+0.05
            elif self.distance>1:
                if self.true_pitch<0.05:
                    pitch_rate = +0.01
                elif self.true_pitch>0.05:
                    pitch_rate = -0.01
            else:
                if self.true_pitch<0.0:
                    pitch_rate = +0.01
                elif self.true_pitch>0.0:
                    pitch_rate = -0.01
            
            self.get_logger().info("Yaw:"+str(yw_rate)+"Pitch:"+str(pitch_rate)+"Thrust:"+str(self.th_rate))
            self.publish_rate_message(yw_rate,pitch_rate)
            self.last_altitude=self.altitude
        

def main(args=None):
    rclpy.init(args=args)

    offboard_control = OffboardControl()

    rclpy.spin(offboard_control)

    offboard_control.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    try:
        main()
    except Exception as e:
        print(e)