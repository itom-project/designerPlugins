# First we create a pointcloude from a topography measurement
myPointCloud = pointCloud.fromDisparity(topo, intensity)

#Just plot the data
[number, handle] = plot(myPointCloud, "twipOGLFigure")

#Set the overlay and the topography color to 50 / 50
handle["overlayAlpha"] = 128
