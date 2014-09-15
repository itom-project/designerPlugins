project(itom_designerPlugin)

cmake_minimum_required(VERSION 2.8)

OPTION(BUILD_UNICODE "Build with unicode charset if set to ON, else multibyte charset." ON)
OPTION(BUILD_SHARED_LIBS "Build shared library." ON)
OPTION(BUILD_TARGET64 "Build for 64 bit target if set to ON or 32 bit if set to OFF." OFF)
SET (ITOM_SDK_DIR "" CACHE PATH "base path to itom_sdk")

OPTION(PLUGIN_itom1DQwtPlot "Build with this plugin." ON)
OPTION(PLUGIN_itom2DQwtPlot "Build with this plugin." ON)
OPTION(PLUGIN_itom2DGraphicView "Build with this plugin." ON)
OPTION(PLUGIN_itomIsoGLFigurePlugin "Build with this plugin." ON)
OPTION(PLUGIN_matplotlibPlot "Build with this plugin." ON)
OPTION(PLUGIN_motorController "Build with this plugin." ON)
OPTION(PLUGIN_dataObjectTable "Build with this plugin." ON)
OPTION(PLUGIN_dObMetaDataTable "Build with this plugin." ON)
OPTION(PLUGIN_evaluateGeometrics "Build with this plugin." ON)

if (PLUGIN_itom1DQwtPlot AND PLUGIN_itom2DQwtPlot)
    ADD_SUBDIRECTORY(qwt)
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
