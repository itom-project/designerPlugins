project(itom_designerPlugin)       

cmake_minimum_required(VERSION 2.8)

OPTION(BUILD_UNICODE "Build with unicode charset if set to ON, else multibyte charset." ON)
OPTION(BUILD_SHARED_LIBS "Build shared library." ON)
OPTION(BUILD_TARGET64 "Build for 64 bit target if set to ON or 32 bit if set to OFF." OFF)
SET (ITOM_SDK_DIR "" CACHE PATH "base path to itom_sdk")

load_cache(${ITOM_SDK_DIR}/../[EXCLUDE ${VTK_DIR}])

if (WIN32)
    # add /LTCG flag to remove MSVC linker warning in release build
    set(CMAKE_SHARED_LINKER_FLAGS_RELEASE "${CMAKE_SHARED_LINKER_FLAGS_RELEASE} /LTCG")
    set(CMAKE_STATIC_LINKER_FLAGS_RELEASE "${CMAKE_STATIC_LINKER_FLAGS_RELEASE} /LTCG")
endif (WIN32)

OPTION(PLUGIN_itom1DQwtPlot "Build with this solution." ON)
OPTION(PLUGIN_itom2DQwtPlot "Build with this solution." ON)
OPTION(PLUGIN_itom2DGraphicView "Build with this solution." ON)
OPTION(PLUGIN_itomIsoGLFigurePlugin "Build with this solution." ON)
OPTION(PLUGIN_matplotlibPlot "Build with this solution." ON)
OPTION(PLUGIN_motorController "Build with this solution." ON)
OPTION(PLUGIN_dataObjectTable "Build with this solution." ON)
OPTION(PLUGIN_dObMetaDataTable "Build with this solution." ON)
OPTION(PLUGIN_evaluateGeometrics "Build with this solution." ON)
OPTION(PLUGIN_vtk3dVisualizer "Build with this solution." ON)
OPTION(PLUGIN_slider2D "Build with this solution." ON)

if (PLUGIN_itom1DQwtPlot OR PLUGIN_itom2DQwtPlot)
    ADD_SUBDIRECTORY(qwt)
endif ()

if (PLUGIN_slider2D)
    ADD_SUBDIRECTORY(slider2D)
endif ()

if (PLUGIN_itom1DQwtPlot)
    ADD_SUBDIRECTORY(itom1DQwtPlot)
endif ()

if (PLUGIN_itom2DQwtPlot)
    ADD_SUBDIRECTORY(itom2DQwtPlot)
endif ()

if (PLUGIN_itom2DGraphicView)
    ADD_SUBDIRECTORY(itom2DGraphicView)
endif ()

if (PLUGIN_itomIsoGLFigurePlugin)
    ADD_SUBDIRECTORY(itomIsoGLFigurePlugin)
endif ()

if (PLUGIN_matplotlibPlot)
    ADD_SUBDIRECTORY(matplotlibPlot)
endif ()

if (PLUGIN_motorController)
    ADD_SUBDIRECTORY(motorController)
endif ()

if (PLUGIN_dataObjectTable)
    ADD_SUBDIRECTORY(dataObjectTable)
endif ()

if (PLUGIN_dObMetaDataTable)
    ADD_SUBDIRECTORY(dObMetaDataTable)
endif ()

if (PLUGIN_evaluateGeometrics)
    ADD_SUBDIRECTORY(evaluateGeometrics)
endif ()

if (PLUGIN_vtk3dVisualizer)
    ADD_SUBDIRECTORY(vtk3dVisualizer)
endif ()
