""" HW1 behavior."""

from __future__ import print_function
from __future__ import division
from __future__ import absolute_import

from task import Task
import core

JOINT_NAMES = [
  "HeadYaw",
  "HeadPitch",
  "LHipYawPitch",
  "LHipRoll",
  "LHipPitch",
  "LKneePitch",
  "LAnklePitch",
  "LAnkleRoll",
  "RHipYawPitch",
  "RHipRoll",
  "RHipPitch",
  "RKneePitch",
  "RAnklePitch",
  "RAnkleRoll",
  "LShoulderPitch",
  "LShoulderRoll",
  "LElbowYaw",
  "LElbowRoll",
  "RShoulderPitch",
  "RShoulderRoll",
  "RElbowYaw",
  "RElbowRoll"
]

SENSOR_NAMES = [
  "gyroX",
  "gyroY",
  "gyroZ",
  "accelX",
  "accelY",
  "accelZ",
  "angleX",
  "angleY",
  "angleZ",
  "battery",
  "fsrLFL",
  "fsrLFR",
  "fsrLRL",
  "fsrLRR",
  "fsrRFL",
  "fsrRFR",
  "fsrRRL",
  "fsrRRR",
  "centerButton",
  "bumperLL",
  "bumperLR",
  "bumperRL",
  "bumperRR",
  "headFront",
  "headMiddle",
  "headRear"
]

class Playing(Task):
    def run(self):
        for _ in range(100):
            for i, name in enumerate(JOINT_NAMES):
                print ("J[%10s]: %.5f" % (name, core.joint_values[i]))
            for i, name in enumerate(SENSOR_NAMES):
                print ("S[%10s]: %.5f" % (name, core.sensor_values[i]))
