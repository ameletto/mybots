from world import WORLD
from robot import ROBOT
import pybullet as p
import pybullet_data
import pyrosim.pyrosim as pyrosim
import time

class SIMULATION:
    def __init__(self, directOrGUI, solutionID):
        self.directOrGUI = directOrGUI
        self.solutionID = solutionID
        if directOrGUI == "DIRECT":
            p.connect(p.DIRECT)
        elif directOrGUI == "GUI":
            p.connect(p.GUI)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.setGravity(0,0,-9.8)
        self.world = WORLD()
        self.robot = ROBOT(self.solutionID)

    def Run(self):
        for t in range (1000):
            p.stepSimulation()
            self.robot.Sense(t)
            self.robot.Think()
            self.robot.Act()
            if self.directOrGUI == "GUI":
                time.sleep(1/1000000)

    def Get_Fitness(self):
        self.robot.Get_Fitness()

    def __del__(self):
        p.disconnect()