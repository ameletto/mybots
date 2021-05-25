import numpy
import pyrosim.pyrosim as pyrosim
import os
import random
import time

length=1
width=1
height=1

class SOLUTION:
    def __init__(self, nextAvailableID):
        self.weights = numpy.random.rand(3, 2)
        self.weights = self.weights * 2 - 1
        self.myID = nextAvailableID
        
    def Start_Simulation(self, directOrGUI):
        self.Create_World()
        self.Create_Body()
        self.Create_Brain()
        # for multiple simulations to run at the same time
        # also starts simulate.py and immediately starts running the next statement without waiting for simulate.py to finish
        os.system("python3 simulate.py " + directOrGUI + " " + str(self.myID) +" &")

    def Wait_For_Simulation_To_End(self):
        while not os.path.exists("fitness"+str(self.myID)+".txt"):
            time.sleep(0.01)
        fitnessFile = open("fitness"+str(self.myID)+".txt", "r")
        self.fitness = fitnessFile.read()
        fitnessFile.close()
        os.system("rm fitness"+str(self.myID)+".txt")
        
    def Create_World(self):
        pyrosim.Start_SDF("world.sdf")
        pyrosim.Send_Cube(name="Box", pos=[-3,3,0.5] , size=[length,width,height])
        pyrosim.End()

    def Create_Body(self):
        pyrosim.Start_URDF("body.urdf")
        pyrosim.Send_Cube(name="BackLeg", pos=[-0.5,0,-0.5] , size=[length,width,height])
        pyrosim.Send_Cube(name="Torso", pos=[1.5,0,1.5] , size=[length,width,height])
        pyrosim.Send_Cube(name="FrontLeg", pos=[0.5,0,-0.5] , size=[length,width,height])
        pyrosim.Send_Joint( name = "Torso_BackLeg" , parent= "Torso" , child = "BackLeg" ,
        type = "revolute", position = "1 0 1")
        pyrosim.Send_Joint( name = "Torso_FrontLeg" , parent= "Torso" , child = "FrontLeg" ,
        type = "revolute", position = "2 0 1")
        pyrosim.End()

    def Create_Brain(self):
        pyrosim.Start_NeuralNetwork("brain"+str(self.myID)+".nndf")
        pyrosim.Send_Sensor_Neuron(name = 0 , linkName = "Torso")
        pyrosim.Send_Sensor_Neuron(name = 1 , linkName = "BackLeg")
        pyrosim.Send_Sensor_Neuron(name = 2 , linkName = "FrontLeg")
        pyrosim.Send_Motor_Neuron( name = 3 , jointName = "Torso_BackLeg")
        pyrosim.Send_Motor_Neuron( name = 4 , jointName = "Torso_FrontLeg")
        # synapses don't have IDs because they are the last type of components will be generating; nothing else will have to refer to them
        # 0 is the presynaptic neuron, 3 is the postsynaptic neuron
        for currentRow in range (0, 3):
            for currentColumn in range (0, 2):
                pyrosim.Send_Synapse( sourceNeuronName = currentRow , targetNeuronName = currentColumn+3 , weight = self.weights[currentRow][currentColumn] )
        pyrosim.End()

    def Mutate(self):
        randomRow = random.randint(0, 2)
        randomColumn = random.randint(0, 1)
        self.weights[randomRow,randomColumn] = random.random() * 2 - 1

    def Set_ID(self):
        return self.myID