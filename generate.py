import pyrosim.pyrosim as pyrosim

length=1
width=1
height=1

def Generate_Body():
    pyrosim.Start_SDF("world.sdf")
    pyrosim.Send_Cube(name="Box", pos=[-3,3,0.5] , size=[length,width,height])
    pyrosim.End()

    pyrosim.Start_URDF("body.urdf")
    pyrosim.Send_Cube(name="BackLeg", pos=[-0.5,0,-0.5] , size=[length,width,height])
    pyrosim.Send_Cube(name="Torso", pos=[1.5,0,1.5] , size=[length,width,height])
    pyrosim.Send_Cube(name="FrontLeg", pos=[0.5,0,-0.5] , size=[length,width,height])
    pyrosim.Send_Joint( name = "Torso_BackLeg" , parent= "Torso" , child = "BackLeg" ,
    type = "revolute", position = "1 0 1")
    pyrosim.Send_Joint( name = "Torso_FrontLeg" , parent= "Torso" , child = "FrontLeg" ,
    type = "revolute", position = "2 0 1")
    pyrosim.End()

def Generate_Brain():
    # pyrosim.Start_SDF("world.sdf")
    # pyrosim.End()

    pyrosim.Start_NeuralNetwork("brain.nndf")
    pyrosim.Send_Sensor_Neuron(name = 0 , linkName = "Torso")
    pyrosim.End()

Generate_Body()
Generate_Brain()

