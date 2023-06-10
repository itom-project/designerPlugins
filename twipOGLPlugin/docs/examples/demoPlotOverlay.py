#Drag and drop the measurement example form the twip download section to your itom workspace
#You will have 2 variables, intensity and topo

#Plot topo and store the plot handle within handle
[number, handle] = plot(topo, "twipOGLFigure")

#Add the overlay to the plot
handle["overlayImage"] = intensity

#Set the overlay and the topography color to 50 / 50
handle["overlayAlpha"] = 128
