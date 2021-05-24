from solution import SOLUTION
import constants as c
import copy

class HILL_CLIMBER:
    def __init__(self):
        self.parent = SOLUTION()

    def Evolve(self):
        self.parent.Evaluate("GUI")
        # this for loop will spawn a mutated copy of self.parent, evaluate that child solution's fitness, 
        # and replace self.parent with this child, if it achieves a better fitness <-- repeat for several generations
        for currentGeneration in range(c.numberOfGenerations):
            self.Evolve_For_One_Generation()

    def Evolve_For_One_Generation(self):
        self.Spawn()
        self.Mutate()
        self.child.Evaluate("DIRECT")
        self.Print()
        self.Select()

    def Spawn(self):
        # self.child receives a copy of self.parent's weights, as well as its fitness
        self.child = copy.deepcopy(self.parent)

    def Mutate(self):
        self.child.Mutate()

    def Select(self):
        # we want the robot to move as far away from the camera as possible (smallest x value)
        if self.parent.fitness > self.child.fitness:
            self.parent = self.child

    def Print(self):
        print(self.parent.fitness, self.child.fitness)

    def Show_Best(self):
        self.parent.Evaluate("GUI")