from solution import SOLUTION
import constants as c

class HILL_CLIMBER:
    def __init__(self):
        self.parent = SOLUTION()

    def Evolve(self):
        self.parent.Evaluate()
        # this for loop will spawn a mutated copy of self.parent, evaluate that child solution's fitness, 
        # and replace self.parent with this child, if it achieves a better fitness <-- repeat for several generations
        for currentGeneration in range(c.numberOfGenerations):
            self.Evolve_For_One_Generation()

    def Evolve_For_One_Generation(self):
        self.Spawn()
        self.Mutate()
        self.child.Evaluate()
        self.Select()

    def Spawn(self):
        pass

    def Mutate(self):
        pass

    def Select(self):
        pass