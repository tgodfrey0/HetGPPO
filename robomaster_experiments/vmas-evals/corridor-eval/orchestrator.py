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
import time
import argparse

finished_lock = Lock()
finished = False

timestamp_lock = Lock()
timestamp = None

#* a_range = max accel?
#* I think yaw is just desired heading
U_RANGE = 0.5

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
    self.grid = Grid(starting_loc, starting_heading, 9, 3)
    self.yaw_count = 0.0
    self.angular_heading = starting_angular_heading
    self.v = 0.0
    
alice: Agent = Agent("Alice", "B0", Grid.Heading.NORTH, (-5,0), pi/2)
bob: Agent = Agent("Bob", "B7", Grid.Heading.SOUTH, (5, 0), 3*pi/2)

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
  
def compose_vel(v) -> Tuple[float, float]:
  vn = v[1]
  ve = v[0]
  
  combined_vel = math.sqrt((vn**2) + (ve**2))
  angle: float
  
  if(vn == 0):
    angle = math.pi / 2
  else:
    angle = math.atan(math.abs(ve) / math.abs(vn))
  
  angle = (2*pi)-angle if ve < 0 else angle
  #! SOmething if the north val is negative
  
  return (combined_vel, angle)

def decompose_vel(vt) -> Tuple[float, float]:
  v = vt[0]
  theta = vt[1]
  
  ve = v*math.sin(abs(theta))
  vn = v*math.cos(abs(theta)) if abs(theta) != (pi / 2) else 0
  
  ve = -ve if theta > pi else ve
  
  return (vn, ve)

def send_vels():
  global timestamp
  
  a_v, a_t = decompose_vel((alice.vn, alice.ve))
  b_v, b_t = decompose_vel((bob.vn, bob.ve))
  
  alice.v = a_v
  a_d_heading = a_t - alice.angular_heading
  alice.angular_heading = a_t
  bob.v = b_v
  b_d_heading = b_t - bob.angular_heading
  bob.angular_heading = b_t
  
  # Send speed and rotation
  sn.send(f"VT ALICE {alice.v} {a_d_heading}")
  sn.send(f"VT BOB {bob.v} {b_d_heading}")
  
  with timestamp_lock:
    timestamp = time.perf_counter()
  
def get_pos():
  x = 1
  y = 0
  
  with timestamp_lock:
    if timestamp is not None:
      t_n = time.perf_counter()
      
      alice.pn += alice.vn * (t_n - timestamp)
      alice.pe += alice.ve * (t_n - timestamp)
      bob.pn += bob.vn * (t_n - timestamp)
      bob.pe += bob.ve * (t_n - timestamp)
  
  cmd_vels = MDEval.compute_action_corridor(
    alice.pe, 
    alice.pn, 
    alice.ve, 
    alice.vn, 
    bob.pe, 
    bob.pn, 
    bob.ve, 
    bob.vn, 
    model=model, 
    u_range=U_RANGE, 
    deterministic=True
  )
  
  print(cmd_vels)
  
  cmd_vel_0 = cmd_vels[0]
  cmd_vel_1 = cmd_vels[1]
  
  alice.vn = cmd_vel_0[x]
  alice.ve = cmd_vel_0[y]
  
  bob.vn = cmd_vel_1[x]
  bob.ve = cmd_vel_1[y]
  
  #* Record time since last velocities were sent then calculate the position 
  
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
  
  sn = SwarmNet({"VT": None, "INFO": lambda s: print(f"Info from {s.split(' ', 2)[0]}: {s.split(' ', 2)[1]}")}, device_list=dl)
  #! sn.start()
  
  dname = os.path.dirname(os.path.realpath(__file__))
  # mdlpath = dname + f"/{args.model_name}"
  mdlpath = dname + "/give_way_export.pt"
  
  print(f"Loading model: {mdlpath}")

  model = torch.load(mdlpath)
  print("Model loaded!")
  model.eval()
  print("Model ready!")
  
  res = MDEval.compute_action_corridor(1, 2, 3, 4, 5, 6, 7, 8, model, 0.5, deterministic=True)
  print(res)
  
  # while(not get_finished()):
  #   get_pos()
  #   send_vels()
  # Delay
  
  # sn.kill()