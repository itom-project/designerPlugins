project(itom_designerPlugin)

cmake_minimum_required(VERSION 2.8)

OPTION(BUILD_UNICODE "Build with unicode charset if set to ON, else multibyte charset." ON)
OPTION(BUILD_SHARED_LIBS "Build shared library." ON)
OPTION(BUILD_TARGET64 "Build for 64 bit target if set to ON or 32 bit if set to OFF." OFF)
SET (ITOM_SDK_DIR "" CACHE PATH "base path to itom_sdk")

ADD_SUBDIRECTORY(qwt)
ADD_SUBDIRECTORY(itom1DQwtFigurePlugin)
ADD_SUBDIRECTORY(itom2DGVFigurePlugin)
ADD_SUBDIRECTORY(itom2DQwtFigurePlugin)
#ADD_SUBDIRECTORY(itom2dQwtPlot) # NICHT!
ADD_SUBDIRECTORY(itomIsoGLFigurePlugin)
ADD_SUBDIRECTORY(matplotlibFigure)
ADD_SUBDIRECTORY(motorController)
ADD_SUBDIRECTORY(dataObjectTable)
