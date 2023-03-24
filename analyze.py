import numpy
import matplotlib.pyplot

# backLegSensorValues=numpy.load('data/backLegSensorValues.npy') 
# frontLegSensorValues=numpy.load('data/frontLegSensorValues.npy')
# matplotlib.pyplot.plot(backLegSensorValues, label='Back Leg', linewidth=2)
# matplotlib.pyplot.plot(frontLegSensorValues, label='Front Leg')

motorValuesFront=numpy.load('data/motorValuesFront.npy')
motorValuesBack=numpy.load('data/motorValuesBack.npy')
matplotlib.pyplot.plot(motorValuesFront, label='Front Leg Motor Values')
matplotlib.pyplot.plot(motorValuesBack, label='Back Leg Motor Values')

matplotlib.pyplot.legend()
matplotlib.pyplot.show()