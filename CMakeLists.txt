project(itom_designerPlugin)
 
cmake_minimum_required(VERSION 3.1...3.15)

option(BUILD_TARGET64 "Build for 64 bit target if set to ON or 32 bit if set to OFF." ON)
set(ITOM_SDK_DIR NOTFOUND CACHE PATH "path of SDK subfolder of itom root (build) directory")

#this is to automatically detect the SDK subfolder of the itom build directory.
if(NOT ITOM_SDK_DIR)
    find_path(ITOM_SDK_DIR "cmake/itom_sdk.cmake"
        HINTS "C:/itom/build/itom/SDK"
              "${CMAKE_CURRENT_BINARY_DIR}/../itom/SDK"
        DOC "path of SDK subfolder of itom root (build) directory")
endif()

if(NOT ITOM_SDK_DIR)
    message(SEND_ERROR "ITOM_SDK_DIR is invalid. Provide itom SDK directory path first")
endif()

set(CMAKE_DEBUG_POSTFIX d CACHE STRING "Adds a postfix for debug-built libraries.")
option(PLUGIN_itom1DQwtPlot "Build with this solution." ON)
option(PLUGIN_itom2DQwtPlot "Build with this solution." ON)
option(PLUGIN_itomIsoGLFigurePlugin "Build with this solution." ON)
option(PLUGIN_matplotlibPlot "Build with this solution." ON)
option(PLUGIN_plotlyPlot "Build with this solution." ON)
option(PLUGIN_motorController "Build with this solution." ON)
option(PLUGIN_evaluateGeometrics "Build with this solution." ON)
option(PLUGIN_vtk3dVisualizer "Build with this solution." ON)
option(PLUGIN_slider2D "Build with this solution." ON)

if(PLUGIN_itom1DQwtPlot OR PLUGIN_itom2DQwtPlot)
    add_subdirectory(qwt)
    add_subdirectory(itomQwtPlotBase)
endif()

if(PLUGIN_slider2D)
    add_subdirectory(slider2D)
endif()

if(PLUGIN_itom1DQwtPlot)
    add_subdirectory(itom1DQwtPlot)
endif()

if(PLUGIN_itom2DQwtPlot)
    add_subdirectory(itom2DQwtPlot)
endif()

if(PLUGIN_itomIsoGLFigurePlugin)
    add_subdirectory(itomIsoGLFigurePlugin)
endif()

if(PLUGIN_matplotlibPlot)
    add_subdirectory(matplotlibPlot)
endif()

if(PLUGIN_plotlyPlot)
    add_subdirectory(plotlyPlot)
endif()

if(PLUGIN_motorController)
    add_subdirectory(motorController)
endif()

if(PLUGIN_evaluateGeometrics)
    add_subdirectory(evaluateGeometrics)
endif()

if(PLUGIN_vtk3dVisualizer)
    add_subdirectory(vtk3dVisualizer)
endif()
