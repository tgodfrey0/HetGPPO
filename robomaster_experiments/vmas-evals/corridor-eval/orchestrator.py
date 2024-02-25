import swarmnet
from swarmnet import SwarmNet

import math

import torch
import os
from dataclasses import dataclass
from typing import Tuple

import rclpy
from rclpy.node import Node

import evaluate_model as MDEval
from math import pi
from grid import Grid
from typing import List, Tuple, Optional, Dict
from threading import Lock
from time import sleep
import argparse

finished_lock = Lock()
finished = False

class Agent():
  def __init__(self, n, starting_loc, starting_heading, starting_pos, starting_angular_heading) -> None:
    self.lock = Lock()
    self.name = n
    self.finished = False
    self.pn = starting_pos[1]
    self.pe = starting_pos[0]
    self.pd = 0.0
    self.vn = 0.0
    self.ve = 0.0
    self.vd = 0.0
    self.yaw = 0.0
    self.an = 0.0
    self.ae = 0.0
    self.ad = 0.0
    self.grid = Grid(starting_loc, starting_heading, 3, 8)
    self.yaw_count = 0.0
    self.angular_heading = starting_angular_heading
    
alice: Agent = Agent("Alice", "B0", Grid.Heading.NORTH, (0,-5), 0)
bob: Agent = Agent("Bob", "B7", Grid.Heading.SOUTH, (0, 5), pi)

dl: List[Tuple[str, int]] = [()]

sn: SwarmNet
model = None

def finished_recv(str: Optional[str]):
  global finished
  
  if str.lower() == "alice":
    with alice.lock():
      alice.finished = True
  elif str.lower() == "bob":
    with bob.lock():
      bob.finished = True
      
  if(alice.finished and bob.finished):
    with finished_lock:
      finished = True
      
def get_finished():
  with finished_lock:
    return finished

# def timer_callback(self): #TODO Play around with this to see how it behaves
#         # evaluate model
#         x = 1  # index into "pos" and "vel"
#         y = 0

#         pos1 = [self.CS1.pn, self.CS1.pe]
#         vel1 = [self.CS1.vn, self.CS1.ve]
#         pos2 = [self.CS2.pn, self.CS2.pe]
#         vel2 = [self.CS2.vn, self.CS2.ve]

#         print(pos1)
#         print(pos2)

#         cmd_vels = MDEval.compute_action_corridor(
#             pos1[x],
#             pos1[y],
#             vel1[x],
#             vel1[y],
#             pos2[x],
#             pos2[y],
#             vel2[x],
#             vel2[y],
#             model=self.model,
#             u_range=self.v_range,
#             deterministic=True,
#         )

#         cmd_vel1 = cmd_vels[0]
#         cmd_vel2 = cmd_vels[1]
#         # print( cmd_vels, cmd_vel1, cmd_vel2 );

#         self.RS1.vn = cmd_vel1[x]
#         self.RS1.ve = cmd_vel1[y]
#         self.RS1.yaw = math.pi / 2  # math.pi / 4.0; # math.atan2( a[1], a[0] );
#         self.RS1.an = self.RS1.ae = self.a_range
#         self.RS1.header.stamp = self.get_clock().now().to_msg()

#         self.RS2.vn = cmd_vel2[x]
#         self.RS2.ve = cmd_vel2[y]
#         self.RS2.yaw = 3 * math.pi / 2  # math.pi / 4.0; # math.atan2( a[1], a[0] );
#         self.RS2.an = self.RS2.ae = self.a_range
#         self.RS2.header.stamp = self.get_clock().now().to_msg()

#         self.publisher1.publish(self.RS1)
#         self.publisher2.publish(self.RS2)

#         # print( [cmd_vel1[0]-self.RS1.ve, cmd_vel1[1]-self.RS1.vn], [cmd_vel2[0]-self.RS2.ve, cmd_vel2[1]-self.RS2.vn] );

#         # book-keeping
#         self.lastRS1 = self.RS1
#         self.lastRS2 = self.RS2
#         self.mytime += self.dt

if __name__=="__main__":
  parser = argparse.ArgumentParser(description="Runs a vmas model.")

  parser.add_argument("--v_range", type=float)
  parser.add_argument("--a_range", type=float)
  parser.add_argument("--model_name", type=str)
  parser.add_argument("--style", type=int)

  args = parser.parse_args()
  
  sn = SwarmNet({"FORWARD": None, "BACKWARD": None, "CLOCKWISE": None, "ANTICLOCKWISE": None, "FINISHED": finished_recv}, device_list=dl)
  #! sn.start()
  
  dname = os.path.dirname(os.path.realpath(__file__))
  # mdlpath = dname + f"/{args.model_name}"
  mdlpath = dname + "/give_way_export.pt"
  
  print(f"Loading model: {mdlpath}")

  model = torch.load(mdlpath)
  print("Model loaded!")
  model.eval()
  print("Model ready!")
  
  res = MDEval.compute_action_corridor(1, 1, 0, 0, 1, 1, 0, 0, model, 0.5, deterministic=True)
  print(res)
  
  # while(not get_finished()):
  #   pass
  
  # sn.kill()