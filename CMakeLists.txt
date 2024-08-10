cmake_minimum_required(VERSION 3.12...3.29)

option(BUILD_TARGET64 "Build for 64 bit target if set to ON or 32 bit if set to OFF." ON)

#this is to automatically detect the SDK subfolder of the itom build directory.
if(NOT EXISTS ${ITOM_SDK_DIR})
    find_path(ITOM_SDK_DIR "cmake/itom_sdk.cmake"
    HINTS "$ENV{ITOM_SDK_ROOT}"
          "${CMAKE_CURRENT_BINARY_DIR}/../itom/SDK"
    DOC "Path of SDK subfolder of itom root (build) directory")
endif(NOT EXISTS ${ITOM_SDK_DIR})

if(NOT EXISTS ${ITOM_SDK_DIR})
    message(FATAL_ERROR "ITOM_SDK_DIR is invalid. Provide itom SDK directory path first")
endif(NOT EXISTS ${ITOM_SDK_DIR})

message(
    STATUS "------------------- PROJECT itom_designerPlugin ---------------------")
# Retrieve Version Number and Identifier from GIT-TAG
include(${ITOM_SDK_DIR}/cmake/VersionFromGit.cmake)
version_from_git(
  LOG       ON
  TIMESTAMP "%Y%m%d%H%M%S"
)

# define cmake project and version number
project(itom_designerPlugin VERSION ${GIT_VERSION})
set(itom_designerPlugin_VERSION_IDENTIFIERS ${GIT_IDENTIFIERS})

option(BUILD_TARGET64 "Build for 64 bit target if set to ON or 32 bit if set to OFF." ON)
set(ITOM_SDK_DIR NOTFOUND CACHE PATH "path of SDK subfolder of itom root (build) directory")

message(
    STATUS "------------------- Version = ${itom_designerPlugin_VERSION} -----------------------\n")

set(CMAKE_DEBUG_POSTFIX d CACHE STRING "Adds a postfix for debug-built libraries.")

# PLUGIN BUILD OPTIONS
set(PLUGIN_BUILD_OPTION "default" CACHE STRING "If ITOM plugin build options: default, setup, test")
set_property(CACHE PLUGIN_BUILD_OPTION PROPERTY STRINGS default setup test)

#include itom_plugin_option macro
include(${CMAKE_CURRENT_SOURCE_DIR}/cmake/plugins_options.cmake)

# evaluateGeometrics
itom_plugin_option(PLUGIN_evaluateGeometrics)
if(PLUGIN_evaluateGeometrics)
    add_subdirectory(evaluateGeometrics)
endif(PLUGIN_evaluateGeometrics)

# itom1DQwtPlot & itom2DQwtPlot
itom_plugin_option(PLUGIN_itom1DQwtPlot)
itom_plugin_option(PLUGIN_itom2DQwtPlot)

if(PLUGIN_itom1DQwtPlot OR PLUGIN_itom2DQwtPlot)
    add_subdirectory(qwt)
    add_subdirectory(itomQwtPlotBase)
endif(PLUGIN_itom1DQwtPlot OR PLUGIN_itom2DQwtPlot)

if(PLUGIN_itom1DQwtPlot)
    add_subdirectory(itom1DQwtPlot)
endif(PLUGIN_itom1DQwtPlot)

if(PLUGIN_itom2DQwtPlot)
    add_subdirectory(itom2DQwtPlot)
endif(PLUGIN_itom2DQwtPlot)

# itomIsoGLFigurePlugin
itom_plugin_option(PLUGIN_itomIsoGLFigurePlugin)
if(PLUGIN_itomIsoGLFigurePlugin)
    add_subdirectory(itomIsoGLFigurePlugin)
endif(PLUGIN_itomIsoGLFigurePlugin)

# matplotlibPlot
itom_plugin_option(PLUGIN_matplotlibPlot)
if(PLUGIN_matplotlibPlot)
    add_subdirectory(matplotlibPlot)
endif(PLUGIN_matplotlibPlot)

# motorController
itom_plugin_option(PLUGIN_motorController)
if(PLUGIN_motorController)
    add_subdirectory(motorController)
endif(PLUGIN_motorController)

# plotlyPlot
itom_plugin_option(PLUGIN_plotlyPlot)
if(PLUGIN_plotlyPlot)
    add_subdirectory(plotlyPlot)
endif(PLUGIN_plotlyPlot)

# PLUGIN_slider2D
itom_plugin_option(PLUGIN_slider2D)
if(PLUGIN_slider2D)
    add_subdirectory(slider2D)
endif(PLUGIN_slider2D)

# twipOGLPlugin
if(BUILD_WITH_PCL)
    itom_plugin_option(PLUGIN_twipOGLPlugin)
    if(PLUGIN_twipOGLPlugin)
        add_subdirectory(twipOGLPlugin)
    endif(PLUGIN_twipOGLPlugin)
endif(BUILD_WITH_PCL)

# vtk3dVisualizer
if(BUILD_WITH_PCL)
    itom_plugin_option(PLUGIN_vtk3dVisualizer)
    if(PLUGIN_vtk3dVisualizer)
        add_subdirectory(vtk3dVisualizer)
    endif(PLUGIN_vtk3dVisualizer)
endif(BUILD_WITH_PCL)