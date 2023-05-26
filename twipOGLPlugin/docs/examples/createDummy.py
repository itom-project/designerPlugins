#Import lena-image from scipy.misc
from scipy.misc import lena
import numpy as np
from numpy import linspace

#The image is 8bit, so create a dataObject from lena with 8-bit
intensity = dataObject(lena()).astype('uint8')

#Create a sphere with the size of lena and a radius of 10 mm
radius = 10
x = np.linspace(-5, 5, intensity.shape[1])
y = np.linspace(-5, 5, intensity.shape[0])
xx, yy = np.meshgrid(x, y, sparse=True)
topo = dataObject(np.sqrt(radius**2 - xx**2 - yy**2))

# create a bumb using an inverted parable
x = np.linspace(-0.25, 0.25, int(intensity.shape[1] / 20))
y = np.linspace(-0.25, 0.25, int(intensity.shape[0] / 20))
xx, yy = np.meshgrid(x, y, sparse=True)

bump = -1*(xx + yy)**2 + 0.25 ** 2

# delete values below zero
bump[np.where(bump < 0)] = 0

x0 = int(intensity.shape[1] / 2)
x1 = int(intensity.shape[1] / 2) +bump.shape[1]
y0 = int(intensity.shape[0] / 2)
y1 = int(intensity.shape[0] / 2) + bump.shape[0]

# add the bumb to the current sphere
topo[y0 : y1, x0 : y1] = topo[y0 : y1, x0 : y1] + dataObject(bump[:,:])

# add some noise to make it more interesting
topo += dataObject.randN([topo.shape[0], topo.shape[1]], topo.dtype) * 0.01

# add a lateral scale to the objects

topo.axisScales = (10.0 / topo.shape[0], 10.0 / topo.shape[1])
#topo.axisOffsets = (topo.shape[0] / 2.0, topo.shape[1] / 2.0)

topo.axisUnits = ('mm', 'mm')
topo.valueUnit = 'mm'

del x0
del x1
del y0
del y1
del bump
del x
del y
del xx
del yy
del radius
