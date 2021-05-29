import os
from parallelHillClimber import PARALLEL_HILL_CLIMBER

phc = PARALLEL_HILL_CLIMBER()
phc.Evolve()
phc.Show_Best()
# really hard to actually get it to move forward in oscillatory motion bc it can take such large steps, gets trapped in local minima.