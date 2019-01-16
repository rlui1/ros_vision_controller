#!/usr/bin/env python
# Copyright (c) 2014 Hanson Robotics, Ltd. 
import os
import yaml
import rospy
from ros_nmpt_saliency.msg import targets
from ros_pololu_servo.msg import servo_pololu

def read_config(config):
    dirname, filename = os.path.split(os.path.abspath(__file__))

    stream = open(os.path.join(dirname, config), 'r')
    conflist = yaml.load(stream)

    #Return a dict instead of a list
    confdict = {}
    for entry in conflist:
      confdict[entry['name']] = entry
    return confdict

def create_cmd(confentry, angle, speed=None):
  """Argument 'val' takes values in rad."""
  cmd = servo_pololu()
  cmd.id = confentry['motorid']
  cmd.angle = angle
  cmd.speed = speed or confentry['speed']
  cmd.acceleration = confentry['acceleration']
  return cmd

class EmaPoint:
  """Exponential moving average point"""
  x = 0.5
  y = 0.5
  alpha = 1.0 # Coefficient for how fast for the average to move

  def __init__(self, N):
    """Alpha coefficient is caluclated from N (the samples to average over)."""
    self.alpha = 2.0/(N+1)

  def update(self, sample):
    """Update the average with a new sample point."""
    self.x = self.alpha*point.x + (1-self.alpha)*self.x
    self.y = self.alpha*point.y + (1-self.alpha)*self.y

SURPRISED = 'surprised'
HAPPY = 'happy'
NEUTRAL = 'neutral'

class EinsteinCtrl:

  curface = '' # Holds string of the current face expression

  def surprised(self):
    if (self.curface == SURPRISED):
      return
    self.send_cmd(self.config['smile'], 0.26)
    self.send_cmd(self.config['jaw'], -0.21)
    self.send_cmd(self.config['brow'], -1.363)
    self.curface = SURPRISED

  def happy(self):
    if (self.curface == HAPPY):
      return
    self.send_cmd(self.config['smile'], -1.556, 10)
    self.send_cmd(self.config['jaw'], -0.54, 10)
    self.send_cmd(self.config['brow'], -1.363, 10)
    self.curface = HAPPY

  def neutral(self):
    if (self.curface == NEUTRAL):
      return
    self.send_cmd(self.config['smile'], 0.017)
    self.send_cmd(self.config['jaw'], -1.271)
    self.send_cmd(self.config['brow'], -0.977)
    self.curface = NEUTRAL

  def send_cmd_norm(self, confentry, angle_norm):
    """Valid 'angle_norm' values include 0 to 1 (they are mapped to the motor's min to max range)."""
    angle = angle_norm * (confentry['max'] - confentry['min']) + confentry['min']
    self.pub.publish(create_cmd(confentry, angle))

  def send_cmd(self, confentry, angle, speed=None):
    """Takes 'angle' values in radians"""
    self.pub.publish(create_cmd(confentry, angle, speed))

  def updateEmas(self, point):
    for ema in [self.fastEma, self.medEma, self.slowEma]:
      ema.update(point)

  def on_point(self, data):
    """Called when a new saliency point is received"""
    point = data.positions[0]

    self.updateEmas(point)
    
    self.send_cmd_norm(self.config['neck_base'], self.fastEma.x)
    self.send_cmd_norm(self.config['neck_pitch'], self.fastEma.y)

    if (pow(self.fastEma.x-self.medEma.x, 2) + pow(self.fastEma.y-self.medEma.y, 2) > 0.01):
      self.surprised()
    elif (pow(self.fastEma.x-self.slowEma.x, 2) + pow(self.fastEma.y-self.slowEma.y, 2) <= 0.01):
      self.neutral()
    else:
      self.happy()

  def init(self):
    self.config = read_config("einstein.yaml")
    self.fastEma = EmaPoint(10)
    self.medEma = EmaPoint(20)
    self.slowEma = EmaPoint(150)
    
    rospy.init_node('einstein_ctrl')
    self.pub = rospy.Publisher('/cmd_pololu', servo_pololu, queue_size=10)
    rospy.Subscriber("/nmpt_saliency_point", targets, self.on_point)

    self.happy()
    self.send_cmd_norm(self.config['neck_roll'], 0.5)

if __name__ == '__main__':
    einstein_ctrl().init()
    rospy.spin()