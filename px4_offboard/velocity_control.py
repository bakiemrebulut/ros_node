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
from px4_msgs.msg import TrajectorySetpoint
from px4_msgs.msg import VehicleStatus
from px4_msgs.msg import VehicleLocalPosition
from px4_msgs.msg import VehicleAttitude
from px4_msgs.msg import VehicleCommand
from px4_msgs.msg import SensorGps
from px4_msgs.msg import InputRc
from px4_msgs.msg import ManualControlSetpoint


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
            '/fmu/vehicle_status/out',
            self.vehicle_status_callback,
            qos_profile)
        
        self.kmsense_velocity_sub = self.create_subscription(
            Twist,
            '/kmsense_velocity_cmd',
            self.kmsense_velocity_callback,
            qos_profile)
        
        """self.attitude_sub = self.create_subscription(
            VehicleAttitude,
            '/fmu/out/vehicle_attitude',
            self.attitude_callback,
            qos_profile)
        """
        self.GPS_sub = self.create_subscription(
            SensorGps,
            '/fmu/sensor_gps/out',
            self.gps_callback,
            qos_profile)        

        
        self.RC_sub = self.create_subscription(
            InputRc,
            '/fmu/input_rc/out',
            self.rc_callback,
            qos_profile)

        self.manual_control_sw_sub = self.create_subscription(
              ManualControlSetpoint  ,
            '/fmu/manual_control_setpoint/out',
            self.mcs_callback,
            qos_profile)
        self.attitude_sub = self.create_subscription(
            VehicleLocalPosition,
            '/fmu/vehicle_local_position/out',
            self.local_position_callback,
            qos_profile)

        #Create publishers
        self.publisher_offboard_mode = self.create_publisher(OffboardControlMode, '/fmu/offboard_control_mode/in', qos_profile)
        #self.publisher_velocity = self.create_publisher(Twist, '/fmu/in/setpoint_velocity/cmd_vel_unstamped', qos_profile)
        self.publisher_trajectory = self.create_publisher(TrajectorySetpoint, '/fmu/trajectory_setpoint/in', qos_profile)
        self.vehicle_command_publisher_ = self.create_publisher(VehicleCommand, "/fmu/vehicle_command/in", 10)

        
        #creates callback function for the arm timer
        # period is arbitrary, just should be more than 2Hz
        arm_timer_period = .1 # seconds
        self.arm_timer_ = self.create_timer(arm_timer_period, self.offboard_callback)

        # creates callback function for the command loop
        # period is arbitrary, just should be more than 2Hz. Because live controls rely on this, a higher frequency is recommended
        # commands in cmdloop_callback won't be executed if the vehicle is not in offboard mode
        timer_period = 0.02  # seconds
        self.timer = self.create_timer(timer_period, self.cmdloop_callback)

        self.nav_state = VehicleStatus.NAVIGATION_STATE_MAX
        self.arm_state = VehicleStatus.ARMING_STATE_INIT
        self.velocity_n = 0.0
        self.velocity_e = 0.0
        self.velocity_d = 0.0
        
        self.sticks_moving = False
        self.yaw = 0.0  #yaw value we send as command
        """self.trueYaw = 0.0  #current yaw value of drone"""
        self.offboard_mode = 0
        self.last_offboard_mode =-1
        
        self.altitude_set_point_delta=-10

        self.myCnt = 0
        self.arm_message = False
        self.failsafe = False
        self.jamming_indicator		=0
        #states with corresponding callback functions that run once when state switches
        self.states = {
            "IDLE": self.state_init,
            "OFFBOARD": self.state_offboard,
            "MANUAL": self.state_manual,
            "HOLD": self.state_loiter
            
        }
        self.current_state = "IDLE"

        self.nstates = [
            "NAVIGATION_STATE_MANUAL",	# Manual mode
            "NAVIGATION_STATE_ALTCTL",	# Altitude control mode
            "NAVIGATION_STATE_POSCTL",	# Position control mode
            "NAVIGATION_STATE_AUTO_MISSION",	# Auto mission mode
            "NAVIGATION_STATE_AUTO_LOITER",	# Auto loiter mode
            "NAVIGATION_STATE_AUTO_RTL",	# Auto return to launch mode
            "NAVIGATION_STATE_UNUSED11",	# Free slot
            "NAVIGATION_STATE_UNUSED10",	# Free slot
            "NAVIGATION_STATE_AUTO_LANDENGFAIL",	# Auto land on engine failure
            "NAVIGATION_STATE_UNUSED",        # Free slot
            "NAVIGATION_STATE_ACRO",		# Acro mode
            "NAVIGATION_STATE_UNUSED1",		# Free slot
            "NAVIGATION_STATE_DESCEND",		# Descend mode (no position control)
            "NAVIGATION_STATE_TERMINATION",		# Termination mode
            "NAVIGATION_STATE_OFFBOARD",
            "NAVIGATION_STATE_STAB",		# Stabilized mode
            "NAVIGATION_STATE_UNUSED2",		# Free slot
            "NAVIGATION_STATE_AUTO_TAKEOFF",	# Takeoff
            "NAVIGATION_STATE_AUTO_LAND",		# Land
            "NAVIGATION_STATE_AUTO_FOLLOW_TARGET",	# Auto Follow
            "NAVIGATION_STATE_AUTO_PRECLAND",	# Precision land with landing target
            "NAVIGATION_STATE_ORBIT",       # Orbit in a circle
            "NAVIGATION_STATE_AUTO_VTOL_TAKEOFF",   # Takeoff, transition, establish loiter
            "NAVIGATION_STATE_MAX"
        ]

    def gps_callback(self,msg):
        self.jamming_indicator	=msg.jamming_indicator		
        #self.spoofing_state = msg.spoofing_state



    def local_position_callback(self, msg):
        self.altitude=msg.z

    def rc_callback(self,msg):
        self.get_logger().info(f"RC: {msg}")

    def mcs_callback(self,msg):
        self.sticks_moving = msg.sticks_moving
        

    #callback function that arms, takes off, and switches to offboard mode
    #implements a finite state machine
    def offboard_callback(self):
        """self.get_logger().info(f"navst: {self.nav_state}")"""
        #self.get_logger().info(f"OJNAS_NEY: {self.current_state,self.jamming_indicator,self.nstates[self.nav_state],self.arm_state,self.sticks_moving}")
        if(self.jamming_indicator > 30):
                self.current_state = "OFFBOARD"
        if (self.current_state== "OFFBOARD"):
            self.state_offboard()
            if self.sticks_moving:
                self.current_state = "MANUAL"
                self.get_logger().info(f"Pilot took over control")
                self.state_manual()
            if (self.jamming_indicator < 30):
                self.current_state = "HOLD"
        elif (self.current_state== "MANUAL"):
            self.state_manual()
        elif (self.current_state== "HOLD"):
            self.state_loiter()
        
    def state_init(self):
        self.get_logger().info(f"wait")
    def state_offboard(self):
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1., 6.)
        self.offboard_mode = 1
    
    def state_manual(self):
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1., 4.)
        self.offboard_mode = 0
    def state_loiter(self):
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_LOITER_UNLIM  , param7=5.0)
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
        msg.timestamp = int(Clock().now().nanoseconds / 1000) # time in microseconds
        self.vehicle_command_publisher_.publish(msg)
    
    #receives and sets vehicle status values 
    def vehicle_status_callback(self, msg):


        self.nav_state = msg.nav_state
        self.arm_state = msg.arming_state
        self.failsafe = msg.failsafe

    

    #receives Twist commands from Teleop and converts NED -> FLU
    def kmsense_velocity_callback(self, msg):
        #implements NED -> FLU Transformation
        self.velocity_n = msg.linear.x
        self.velocity_e = msg.linear.y
        self.velocity_d = msg.linear.z
        self.yaw = msg.angular.z


    #receives current trajectory values from drone and grabs the yaw value of the orientation
    """def attitude_callback(self, msg):
        orientation_q = msg.q

        #trueYaw is the drones current yaw value
        self.trueYaw = -(np.arctan2(2.0*(orientation_q[3]*orientation_q[0] + orientation_q[1]*orientation_q[2]), 
                                  1.0 - 2.0*(orientation_q[0]*orientation_q[0] + orientation_q[1]*orientation_q[1])))"""
        
    #publishes offboard control modes and velocity as trajectory setpoints
    def cmdloop_callback(self):
        if (self.last_offboard_mode != self.offboard_mode and self.last_offboard_mode!=-1 ):
            self.altitude_set_point=self.altitude+self.altitude_set_point_delta
        self.last_offboard_mode=self.offboard_mode
        if(self.offboard_mode == 1):
            # Publish offboard control modes
            offboard_msg = OffboardControlMode()
            offboard_msg.timestamp = int(Clock().now().nanoseconds / 1000)
            offboard_msg.position = False
            offboard_msg.velocity = True
            offboard_msg.acceleration = False
            self.publisher_offboard_mode.publish(offboard_msg)            


            # Create and publish TrajectorySetpoint message with NaN values for position and acceleration
            trajectory_msg = TrajectorySetpoint()
            trajectory_msg.timestamp = int(Clock().now().nanoseconds / 1000)
            
            z_val=0.0
            if(self.altitude>(self.altitude_set_point) or self.altitude<(self.altitude_set_point-5)):
                z_val=(self.altitude_set_point-self.altitude)/5.0
            trajectory_msg.vx = self.velocity_n
            trajectory_msg.vy = self.velocity_e
            trajectory_msg.vz = z_val
            trajectory_msg.x = float('nan')
            trajectory_msg.y = float('nan')
            trajectory_msg.z = float('nan')
            trajectory_msg.acceleration[0] = float('nan')
            trajectory_msg.acceleration[1] = float('nan')
            trajectory_msg.acceleration[2] = float('nan')
            trajectory_msg.jerk[0] = float('nan')
            trajectory_msg.jerk[1] = float('nan')
            trajectory_msg.jerk[2] = float('nan')
            trajectory_msg.thrust[0] = float('nan')
            trajectory_msg.thrust[1] = float('nan')
            trajectory_msg.thrust[2] = float('nan')
            

            trajectory_msg.yaw = float('nan')
            trajectory_msg.yawspeed = self.yaw

            self.publisher_trajectory.publish(trajectory_msg)


def main(args=None):
    rclpy.init(args=args)

    offboard_control = OffboardControl()

    rclpy.spin(offboard_control)

    offboard_control.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()