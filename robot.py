import pybullet as p
import pyrosim.pyrosim as pyrosim
from sensor import SENSOR
from motor import MOTOR
from pyrosim.neuralNetwork import NEURAL_NETWORK
import os

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
                self.motors[self.jointName].Set_Value(self.desiredAngle)

    def Think(self):
        self.nn.Update()
        # self.nn.Print()

    def Get_Fitness(self):
        # self.robot is the robot, 0 is the link, p.getLinkState gets the position of the link
        stateOfLinkZero = p.getLinkState(self.robot,0)
        # print(stateOfLinkZero)
        positionOfLinkZero = stateOfLinkZero[0]
        # print(positionOfLinkZero)
        xCoordinateOfLinkZero = positionOfLinkZero[0]
        # print(xCoordinateOfLinkZero)
        f = open("tmp"+str(self.solutionID)+".txt", "w")
        f.write(str(xCoordinateOfLinkZero))
        os.system("mv tmp"+str(self.solutionID)+".txt fitness"+str(self.solutionID)+".txt") 

