from numpy.lib.shape_base import take_along_axis
import constants as c
import numpy
import pyrosim.pyrosim as pyrosim
import pybullet as p

class MOTOR:
    def __init__(self, jointName, robot):
        self.jointName = jointName
        self.theRobot = robot

    def Set_Value(self, desiredAngle):
        pyrosim.Set_Motor_For_Joint(
        bodyIndex = self.theRobot,
        jointName = self.jointName,
        controlMode = p.POSITION_CONTROL,
        targetPosition = desiredAngle,
        maxForce = 40)