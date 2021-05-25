from solution import SOLUTION
import constants as c
import copy
import os

class PARALLEL_HILL_CLIMBER:
    def __init__(self):
        os.system("rm brain*.nndf")
        os.system("rm fitness*.txt")
        self.parents = {}
        self.nextAvailableID = 0
        for i in range (0, c.populationSize):
            self.parents[i] = SOLUTION(self.nextAvailableID)
            self.nextAvailableID += 1

    def Evolve(self):
        self.Evaluate(self.parents)
        # this for loop will spawn a mutated copy of self.parent, evaluate that child solution's fitness, 
        # and replace self.parent with this child, if it achieves a better fitness <-- repeat for several generations
        for currentGeneration in range(c.numberOfGenerations):
            self.Evolve_For_One_Generation()

    def Evolve_For_One_Generation(self):
        self.Spawn()
        self.Mutate()
        self.Evaluate(self.children)
        self.Print()
        # self.Select()

    def Spawn(self):
        # self.child receives a copy of self.parent's weights, as well as its fitness
        self.children = {}
        for i in self.parents:
            self.children[i] = copy.deepcopy(self.parents[i])
            self.children[i].Set_ID()
            self.nextAvailableID += 1

    def Mutate(self):
        for j in self.children:
            self.children[j].Mutate()

    def Evaluate(self, solutions):
        for j in solutions:
            solutions[j].Start_Simulation("DIRECT")
        for j in solutions:
            solutions[j].Wait_For_Simulation_To_End()

    def Print(self):
        for i in self.parents:
            print("")
            print(self.parents[i].fitness, self.children[i].fitness)
            print("")

    def Select(self):
        # we want the robot to move as far away from the camera as possible (smallest x value)
        if self.parent.fitness > self.child.fitness:
            self.parent = self.child

    def Show_Best(self):
        pass
        # self.parent.Evaluate("GUI")