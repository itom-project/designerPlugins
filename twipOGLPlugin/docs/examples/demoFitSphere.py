# First we create a pointcloude from a topography measurement
myPointCloud = pointCloud.fromDisparity(topo, intensity)

# We take the random samples using an itom filter from the pclTools-plugin
# The parameters are inputCloud, outputCloud, number of samples
myRandomPoints = pointCloud()
filter("pclRandomSample", myPointCloud, myRandomPoints, 65000)

# We fit the sphere into the random points using an itom filter from the pclTools-plugin
# The parameters are inputCloud, [min Radius, max Radius]
# The return values are the center point in xyz, the radius, and the number of points within approx 0.05 distance to the fitted sphere
[cPt, cRad, cInl] = filter("pclFitSphere", myRandomPoints,  [9.0, 11.0], optimizeParameters=1)

print('The fitted radius is ' + str(cRad) + ' mm')

# We calculate the distance between the sphere and the raw data
myPointsDistance = pointCloud()
filter("pclDistanceToModel", myPointCloud, myPointsDistance, 4, cPt, [], cRad)

#Just plot the data
[number, handle] = plot(myPointsDistance, "twipOGLFigure")

#Set the overlay and the topography color to 50 / 50
handle["overlayAlpha"] = 128

#We change the false color rendering from topography (z-Value) to curvature (curvature-Value of each point)
handle["curvatureToColor"] = True

#We change the color palette to "hotIron" and turn of the colorBar
handle["colorMap"] = "falseColor"
handle["colorBarVisible"] = True
