project(itom_designerPlugin)       

cmake_minimum_required(VERSION 3.0.2)

option(BUILD_UNICODE "Build with unicode charset if set to ON, else multibyte charset." ON)
option(BUILD_SHARED_LIBS "Build shared library." ON)
option(BUILD_TARGET64 "Build for 64 bit target if set to ON or 32 bit if set to OFF." ON)

#this is to automatically detect the SDK subfolder of the itom build directory.
find_path(ITOM_SDK_DIR "itom_sdk.cmake"
    HINTS "C:/itom/build/itom/SDK"
          "${CMAKE_CURRENT_BINARY_DIR}/../itom/SDK"
    DOC "path of SDK subfolder of itom root (build) directory")

if("${ITOM_SDK_DIR}" STREQUAL "ITOM_SDK_DIR-NOTFOUND")
    message(FATAL_ERROR "ITOM_SDK_DIR is invalid. Provide itom SDK directory path first")
endif()

load_cache(${ITOM_SDK_DIR}/../[EXCLUDE ${VTK_DIR}])

if(WIN32)
    # add /LTCG flag to remove MSVC linker warning in release build
    set(CMAKE_SHARED_LINKER_FLAGS_RELEASE "${CMAKE_SHARED_LINKER_FLAGS_RELEASE} /LTCG")
    set(CMAKE_STATIC_LINKER_FLAGS_RELEASE "${CMAKE_STATIC_LINKER_FLAGS_RELEASE} /LTCG")
endif(WIN32)

option(PLUGIN_itom1DQwtPlot "Build with this solution." ON)
option(PLUGIN_itom2DQwtPlot "Build with this solution." ON)
option(PLUGIN_itom2DGraphicView "Build with this solution." OFF)
option(PLUGIN_itomIsoGLFigurePlugin "Build with this solution." ON)
option(PLUGIN_matplotlibPlot "Build with this solution." ON)
option(PLUGIN_motorController "Build with this solution." ON)
option(PLUGIN_dataObjectTable "Build with this solution." ON)
option(PLUGIN_dObMetaDataTable "Build with this solution." ON)
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

if(PLUGIN_itom2DGraphicView)
    add_subdirectory(itom2DGraphicView)
endif()

if(PLUGIN_itomIsoGLFigurePlugin)
    add_subdirectory(itomIsoGLFigurePlugin)
endif()

if(PLUGIN_matplotlibPlot)
    add_subdirectory(matplotlibPlot)
endif()

if(PLUGIN_motorController)
    add_subdirectory(motorController)
endif()

if(PLUGIN_dataObjectTable)
    add_subdirectory(dataObjectTable)
endif()

if(PLUGIN_dObMetaDataTable)
    add_subdirectory(dObMetaDataTable)
endif()

if(PLUGIN_evaluateGeometrics)
    add_subdirectory(evaluateGeometrics)
endif()

if(PLUGIN_vtk3dVisualizer)
    add_subdirectory(vtk3dVisualizer)
endif()
