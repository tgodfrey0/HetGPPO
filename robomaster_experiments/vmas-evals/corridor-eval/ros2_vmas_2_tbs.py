#  Copyright (c) 2022.
#  ProrokLab (https://www.proroklab.org/)
#  All rights reserved.

import math

# import torch
import os
from dataclasses import dataclass
from typing import Tuple

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

import evaluate_model as MDEval
from math import pi
from grid import Grid

STARTING_POS: str = "B0"
TARGET_POS: str = "B7"

AREA_WIDTH: float = 3.0
AREA_LENGTH: float = 3.0 

@dataclass
class State():
    grid = Grid(STARTING_POS, Grid.Heading.NORTH, 3, 8)
    yaw_count: float = 0
    actual_pos: Tuple[float, float, float] # x(x,y,heading)


class ReferenceStateMock():
    def __init__(self):
        self.pn: float
        self.pe: float
        self.pd: float
        self.vn: float
        self.ve: float
        self.vd: float
        self.yaw: float
        self.an: float
        self.ae: float
        self.ad: float
        

class Translator():
    @staticmethod
    def grid_to_abs() -> ReferenceStateMock:
        # Assume centre of grid
        rs = ReferenceStateMock()
        pos = (State.grid.col, State.grid.row)
    
    @staticmethod
    def abs_to_grid(rs: ReferenceStateMock) -> Tuple[str, int, Grid.Heading]:
        # length
        # ^
        # 0 -> width
        col = chr(math.floor((rs.pe / AREA_WIDTH) * State.grid.max_width) + ord('A'))
        row = math.floor((rs.pn / AREA_LENGTH) * State.grid.max_height)
        
        new_heading: Grid.Heading = State.grid.heading
        
        State.yaw_count += rs.yaw
        
        # Clockwise positive
        if(State.yaw_count >= pi/2):
            State.grid.clockwise()
            State.yaw_count = 0
        elif(State.yaw_count <= -pi/2):
            State.grid.anticlockwise()
            State.yaw_count = 0
        
        # discretised_yaw: Grid.Heading
        # #? 0 -> 2pi?
        # if(rs.yaw <= (1/4 * pi) or rs.yaw > (7/4 * pi)):
        #     discretised_yaw = Grid.Heading.NORTH
        # elif(rs.yaw <= (3/4 * pi) and rs.yaw > (1/4 * pi)):
        #     discretised_yaw = Grid.Heading.EAST
        # elif(rs.yaw <= (5/4 * pi) and rs.yaw > (3/4 * pi)):
        #     discretised_yaw = Grid.Heading.SOUTH
        # elif(rs.yaw <= (7/4 * pi) and rs.yaw > (5/4 * pi)):
        #     discretised_yaw = Grid.Heading.WEST
        # else:
        #     raise RuntimeError(f"Unrecognised yaw angle: {rs.yaw}")
        
        return (col, row, new_heading)
        

class MinimalPublisher(Node):
    # self.pn0 = 3.0;
    # self.pn1 = 1.0;
    # self.RS = ReferenceStateMock();

    def __init__(self, style, a_range, v_range, model_name):
        super().__init__("lawnmower_publisher")

        self.a_range = a_range
        self.v_range = v_range
        self.model_name = model_name

        import torch

        dname = os.path.dirname(os.path.realpath(__file__))
        mdlpath = dname + f"/{self.model_name}"
        
        self.get_logger().info(f"Loading model: {mdlpath}")

        self.model = torch.load(mdlpath)
        print("Model loaded!")
        self.model.eval()
        print("Model ready!")

        self.CS1 = ReferenceStateMock() # Current State?
        self.CS2 = ReferenceStateMock()
        self.RS1 = ReferenceStateMock() # Reference State? I.e. state to get to?
        self.RS2 = ReferenceStateMock()
        self.lastRS1 = ReferenceStateMock()
        self.lastRS2 = ReferenceStateMock()

        self.dt = 1.0 / 20.0
        self.mytime = 0
        self.subscriber1 = self.create_subscription(
            CurrentState, "/robomaster_1/current_state", self.current_state_callback_robot_1, 1
        )
        self.publisher1 = self.create_publisher(
            ReferenceStateMock, "/robomaster_1/reference_state", 1
        )
        self.subscriber2 = self.create_subscription(
            CurrentState, "/robomaster_2/current_state", self.current_state_callback_robot_2, 1
        )
        self.publisher2 = self.create_publisher(
            ReferenceStateMock, "/robomaster_2/reference_state", 1
        )
        timer_period = self.dt  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        print("ROS2 starting ..")

    # At 20 Hz publish the new positions of the agents
    def timer_callback(self):
        # evaluate model
        x = 1  # index into "pos" and "vel"
        y = 0

        pos1 = [self.CS1.pn, self.CS1.pe]
        vel1 = [self.CS1.vn, self.CS1.ve]
        pos2 = [self.CS2.pn, self.CS2.pe]
        vel2 = [self.CS2.vn, self.CS2.ve]

        print(pos1)
        print(pos2)

        cmd_vels = MDEval.compute_action_corridor(
            pos1[x],
            pos1[y],
            vel1[x],
            vel1[y],
            pos2[x],
            pos2[y],
            vel2[x],
            vel2[y],
            model=self.model,
            u_range=self.v_range,
            deterministic=True,
        )

        cmd_vel1 = cmd_vels[0]
        cmd_vel2 = cmd_vels[1]
        # print( cmd_vels, cmd_vel1, cmd_vel2 );

        self.RS1.vn = cmd_vel1[x]
        self.RS1.ve = cmd_vel1[y]
        self.RS1.yaw = math.pi / 2  # math.pi / 4.0; # math.atan2( a[1], a[0] );
        self.RS1.an = self.RS1.ae = self.a_range
        self.RS1.header.stamp = self.get_clock().now().to_msg()

        self.RS2.vn = cmd_vel2[x]
        self.RS2.ve = cmd_vel2[y]
        self.RS2.yaw = 3 * math.pi / 2  # math.pi / 4.0; # math.atan2( a[1], a[0] );
        self.RS2.an = self.RS2.ae = self.a_range
        self.RS2.header.stamp = self.get_clock().now().to_msg()

        self.publisher1.publish(self.RS1)
        self.publisher2.publish(self.RS2)

        # print( [cmd_vel1[0]-self.RS1.ve, cmd_vel1[1]-self.RS1.vn], [cmd_vel2[0]-self.RS2.ve, cmd_vel2[1]-self.RS2.vn] );

        # book-keeping
        self.lastRS1 = self.RS1
        self.lastRS2 = self.RS2
        self.mytime += self.dt

    # Update the locations on publish
    def current_state_callback_robot_1(self, msg):
        self.CS1.pn = msg.state_vector[0]  # + self.off_pn
        self.CS1.pe = msg.state_vector[1]  # + self.off_pe
        self.CS1.vn = msg.state_vector[3]
        self.CS1.ve = msg.state_vector[4]
        # self.CS.yaw = msg.state_vector[8]

    def current_state_callback_robot_2(self, msg):
        self.CS2.pn = msg.state_vector[0]  # + self.off_pn
        self.CS2.pe = msg.state_vector[1]  # + self.off_pe
        self.CS2.vn = msg.state_vector[3]
        self.CS2.ve = msg.state_vector[4]
        # self.CS.yaw = msg.state_vector[8]
        
    def process_rs(self, rs: ReferenceStateMock):
        
        
    def delay(self, t_target):
        t0 = self.get_clock().now()
        while(self.get_clock().now() - t0 < rclpy.duration.Duration(seconds=t_target)):
        pass
        self.get_logger().info(f"Delayed for {t_target} seconds")
    
    def _publish_cmd(self, msg: Twist):
        self.publisher_.publish(msg)
        self.get_logger().info(f"Publishing to /cmd_vel")
    
    def _publish_zero(self):
        self.get_logger().info(f"Zero velocity requested")
        msg = Twist()
        
        msg.linear.x = 0.0
        msg.linear.y = 0.0
        msg.linear.z = 0.0
        
        msg.angular.x = 0.0
        msg.angular.y = 0.0
        msg.angular.z = 0.0
        
        self._publish_cmd(msg)
        
    def _pub_linear(self, dir: int, dist: float):
        if(dist == 0):
        return 
        
        msg = Twist()
        
        msg.linear.x = dir * LINEAR_SPEED
        msg.linear.y = 0.0
        msg.linear.z = 0.0
        
        msg.angular.x = 0.0
        msg.angular.y = 0.0
        msg.angular.z = 0.0
        
        self._publish_cmd(msg)    
        self.delay((dist / LINEAR_SPEED) + 0.05) # Extra delay for the spinup of motors -- this may need tuning/removing
        self._publish_zero()
    
    def _pub_rotation(self, dir: float, angle: float):
        if(angle == 0):
        return
        
        msg = Twist()
        
        msg.linear.x = 0.0
        msg.linear.y = 0.0
        msg.linear.z = 0.0
        
        msg.angular.x = 0.0
        msg.angular.y = 0.0
        msg.angular.z = dir * ANGULAR_SPEED
        
        self._publish_cmd(msg)
        self.delay((angle / ANGULAR_SPEED) + 0.05) # Extra delay for the spinup of motors
        self._publish_zero()
        
    def pub_forwards(self, dist: float): # m
        self.get_logger().info(f"Forwards {dist} m command")
        self._pub_linear(1, abs(dist))
        
    def pub_backwards(self, dist: float): # m
        self.get_logger().info(f"Backwards {dist} m command")
        self._pub_linear(-1, abs(dist))
        
    def pub_anticlockwise(self, angle: float): # rads
        self.get_logger().info(f"Anticlockwise {angle} rad command")
        self._pub_rotation(1, abs(angle))
        
    def pub_clockwise(self, angle: float): # rads
        self.get_logger().info(f"Clockwise {angle} rad command")
        self._pub_rotation(-1, abs(angle))


def main(s):
    rclpy.init(args=None)

    minimal_publisher = MinimalPublisher(
        style=args.style,
        a_range=args.a_range,
        v_range=args.v_range,
        model_name=args.model_name,
    )

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser(description="Runs a vmas model.")

    parser.add_argument("--v_range", type=float)
    parser.add_argument("--a_range", type=float)
    parser.add_argument("--model_name", type=str)
    parser.add_argument("--style", type=int)

    args = parser.parse_args()

    main(args)
