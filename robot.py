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
        # oneLiftOff = 0
        # longestLiftOff = 0
        # # print(self.sensors['FrontLowerLeg'].values, self.sensors['BackLowerLeg'].values, self.sensors['LeftLowerLeg'].values, self.sensors['RightLowerLeg'].values)
        # for j in range (1000):
        #     if self.sensors['FrontLowerLeg'].values[j] == -1 and self.sensors['BackLowerLeg'].values[j] == -1 and self.sensors['LeftLowerLeg'].values[j] == -1 and self.sensors['RightLowerLeg'].values[j] == -1:
        #         oneLiftOff += 1
        #     else:
        #         oneLiftOff = 0
        #     if oneLiftOff > longestLiftOff:
        #         longestLiftOff = oneLiftOff
        
        zTorso = p.getLinkState(self.robot,0)[0][2]
        zFrontLowerLeg = p.getLinkState(self.robot,1)[0][2]
        zBackLowerLeg = p.getLinkState(self.robot,2)[0][2]
        zLeftLowerLeg = p.getLinkState(self.robot,3)[0][2]
        zRightLowerLeg = p.getLinkState(self.robot,4)[0][2]
        
        # worldPosTorso = stateOfTorso[0]
        # worldPosFrontLowerLeg = stateOfFrontLowerLeg[0]
        # worldPosBackLowerLeg = stateOfBackLowerLeg[0]
        # worldPosLeftLowerLeg = stateOfLeftLowerLeg[0]
        # worldPosRightLowerLeg = stateOfRightLowerLeg[0]

        # zTorso = worldPosTorso[2]
        # zFrontLowerLeg = worldPosFrontLowerLeg[2]
        # zBackLowerLeg = worldPosBackLowerLeg[2]
        # zLeftLowerLeg = worldPosLeftLowerLeg[2]
        # zRightLowerLeg = worldPosRightLowerLeg[2]

        if zTorso == 0.5:
            robotFitness = (zFrontLowerLeg+zBackLowerLeg+zLeftLowerLeg+zRightLowerLeg)/4
        else:
            robotFitness = 0

        f = open("tmp"+str(self.solutionID)+".txt", "w")
        f.write(str(robotFitness))
        f.close()
        os.system("mv" + " " + "tmp"+str(self.solutionID)+".txt" + " " + "fitness"+str(self.solutionID)+".txt") 
        
