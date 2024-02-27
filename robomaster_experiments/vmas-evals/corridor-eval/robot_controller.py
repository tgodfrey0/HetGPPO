import base64
import csv
import datetime
import openai
import os
import sys
import yaml
from math import pi, radians
from openai import OpenAI, ChatCompletion
from swarmnet import SwarmNet, Log_Level, set_log_level
from tenacity import retry, stop_after_attempt, wait_random_exponential
from threading import Lock
from typing import Dict, Optional, List, Tuple

# ROS2 imports
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

# Local package imports
from .grid import Grid

valid_grid_positions: List[str] = ["A3", "B0", "B1", "B2", "B3", "B4", "B5", "B6", "B7", "C4"]
dl = [("192.168.0.1", 51000)]
AGENT_NAME = "Alice"

ANGULAR_SPEED = pi / 6


class VelocityPublisher(Node):
  def __init__(self):
    super().__init__("velocity_publisher")
        
    self.publisher_ = self.create_publisher(Twist, "/cmd_vel", 10)
    
    # qos = QoSProfile(
    #   reliability=ReliabilityPolicy.BEST_EFFORT,
    #   history=HistoryPolicy.KEEP_LAST,
    #   depth=10
    # )
    
    # self.subscription = self.create_subscription(
    #   LaserScan,
    #   "/scan",
    #   self.listener_callback,
    #   qos_profile=qos
    # )
    # self.subscription
    
    self.sn_ctrl = SwarmNet({"VT": self.vt_recv, "INFO": None}, device_list = dl) #! Publish INFO messages which can then be subscribed to by observers
    self.sn_ctrl.start()
    self.get_logger().info(f"SwarmNet initialised") 
    set_log_level(self.SN_LOG_LEVEL)
    self.info("SwarmNet initialised successfully")
    
  def vt_recv(self, msg):
    p = msg.split(" ")
    
    if(p[0] != AGENT_NAME):
      return
    
    if(float(p[2]) < 0):
      self.pub_anticlockwise(abs(float(p[2])))
    else:
      self.pub_clockwise(abs(float(p[2])))
      
    if(float(p[1]) < 0):
      self.pub_backwards(abs(float(p[1])))
    else:
      self.pub_forwards(abs(float(p[1])))
    
  def info(self, s: str) -> None:
    self.get_logger().info(s)
    self.sn_ctrl.send(f"INFO {self.AGENT_NAME}: {s}")
    
  # def listener_callback(self, msg: LaserScan) -> None:
  #   self.info(f"{msg}") 
  #   # Clip ranges to those in front
  #   angle_increment = msg.angle_increment
  #   angle_min = msg.angle_min
  #   angle_max = msg.angle_max
  #   ranges = msg.ranges

  #   # Calculate the start and end indices for the 90-degree cone
  #   start_index = int((45 - angle_min) / angle_increment)
  #   end_index = int((45 - angle_max) / angle_increment)

  #   # Ensure indices are within bounds
  #   start_index = max(0, start_index)
  #   end_index = min(len(ranges) - 1, end_index)
      
  #   with self.scan_mutex:
  #     self.scan_ranges = ranges[start_index:end_index + 1]
        
  def _delay(self, t_target):
    t0 = self.get_clock().now()
    while(self.get_clock().now() - t0 < rclpy.duration.Duration(seconds=t_target)):
      pass
    self.get_logger().info(f"Delayed for {t_target} seconds")
    
  def linear_delay(self):
    self._delay(self.LINEAR_TIME)
    
  def angular_delay(self):
    self._delay(self.ANGULAR_TIME)
    
  def wait_delay(self):
    self._delay(self.WAITING_TIME)
    
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
    
  def _pub_linear(self, dir: int, v):
    msg = Twist()
    
    msg.linear.x = float(dir * v)
    msg.linear.y = 0.0
    msg.linear.z = 0.0
    
    msg.angular.x = 0.0
    msg.angular.y = 0.0
    msg.angular.z = 0.0
    
    self._publish_cmd(msg)
  
  def _pub_rotation(self, dir: float, theta):
    msg = Twist()
    
    msg.linear.x = 0.0
    msg.linear.y = 0.0
    msg.linear.z = 0.0
    
    msg.angular.x = 0.0
    msg.angular.y = 0.0
    msg.angular.z = float(dir * ANGULAR_SPEED) #? X Y or Z
    
    self._publish_cmd(msg)
    self._delay(theta / ANGULAR_SPEED)
    self._publish_zero()
    
  def pub_forwards(self, v):
    self.info(f"Forwards command")
    self._pub_linear(1, v)
    
  def pub_backwards(self, v):
    self.info(f"Backwards command")
    self._pub_linear(-1, v)
    
  def pub_anticlockwise(self, theta):
    self.info(f"Anticlockwise command")
    self._pub_rotation(1, theta)
    
  def pub_clockwise(self, theta):
    self.info(f"Clockwise command")
    self._pub_rotation(-1, theta)
