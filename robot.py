import pybullet as p
import pyrosim.pyrosim as pyrosim
from sensor import SENSOR
from motor import MOTOR
from pyrosim.neuralNetwork import NEURAL_NETWORK
import os
import time
import constants as c
import numpy

class ROBOT:
    def __init__(self, solutionID):
        self.robot = p.loadURDF("body.urdf")
        self.solutionID = solutionID
        pyrosim.Prepare_To_Simulate("body.urdf")
        self.Prepare_To_Sense()
        self.Prepare_To_Act(self.robot)
        self.nn = NEURAL_NETWORK("brain"+str(self.solutionID)+".nndf")
        # deletes the nndf file after it has been read
        # but why do we have to run search.py first???
        # i think it's because search.py (re)creates the brainID.nndf files? 
        os.system("rm brain"+str(self.solutionID)+".nndf")
        
    def Prepare_To_Sense(self):
        self.sensors = {}
        for linkName in pyrosim.linkNamesToIndices:
            self.sensors[linkName] = SENSOR(linkName)

    def Sense(self, t):
        for i in self.sensors:
            self.sensors[i].Get_Value(t)

    def Prepare_To_Act(self, robot):
        self.motors = {}
        for jointName in pyrosim.jointNamesToIndices:
            self.motors[jointName] = MOTOR(jointName, robot)

    # essentially what's going on = synapseWeight (generated randomly) * sensorNeuronValue added to motorNeuronValue
    # and then the motorNeuronValue is thresholded and converted to a target angle
    # so the AI will test weights until it gets to the optimal weight (influence sensor value has on motor value)
    def Act(self):
        for neuronName in self.nn.Get_Neuron_Names():
            if self.nn.Is_Motor_Neuron(neuronName):
                self.jointName = self.nn.Get_Motor_Neurons_Joint(neuronName)
                self.desiredAngle = self.nn.Get_Value_Of(neuronName)
                self.motors[self.jointName].Set_Value(self.desiredAngle*c.motorJointRange)

    def Think(self):
        self.nn.Update()
        # self.nn.Print()

    def Get_Fitness(self):
        # self.robot is the robot, 0 is the link, p.getLinkState gets the position of the link

        # but we need all the legs to be off the ground so fix that
        timeStampConsNegOnes = 0
        oneLiftOff = 0
        longestLiftOff = 0
        for j in range (1000):
            for i in self.sensors:
                if self.sensors[i].values[j] == -1:
                    timeStampConsNegOnes += 1
                else:
                    timeStampConsNegOnes = 0
                    oneLiftOff = 0
                    break 
                timeStampConsNegOnes = 0
            if timeStampConsNegOnes == len(self.sensors):
                oneLiftOff += 1
                if oneLiftOff > longestLiftOff:
                    longestLiftOff = oneLiftOff
        f = open("tmp"+str(self.solutionID)+".txt", "w")
        f.write(str(longestLiftOff))
        f.close()
        os.system("mv" + " " + "tmp"+str(self.solutionID)+".txt" + " " + "fitness"+str(self.solutionID)+".txt") 
        # basePositionAndOrientation = p.getBasePositionAndOrientation(self.robot)
        # basePosition = basePositionAndOrientation[0]
        # xPosition = basePosition[2]
        
