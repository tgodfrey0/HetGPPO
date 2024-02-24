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

finished_lock = Lock()
finished = False

class Agent():
  def __init__(self, n, starting_loc, starting_heading) -> None:
    self.lock = Lock()
    self.name = n
    self.finished = False
    self.pn = 0.0
    self.pe = 0.0
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
    
alice: Agent = Agent("Alice", "B0", Grid.Heading.NORTH)
bob: Agent = Agent("Bob", "B7", Grid.Heading.SOUTH)

dl: List[Tuple[str, int]] = [()]

sn: SwarmNet

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
    

if __name__=="__main__":
  sn = SwarmNet({"FORWARD": None, "BACKWARD": None, "CLOCKWISE": None, "ANTICLOCKWISE": None, "FINISHED": finished_recv}, device_list=dl)
  sn.start()
  
  while(not get_finished()):
    pass
  
  sn.kill()