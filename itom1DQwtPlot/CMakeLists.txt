SET (target_name itom1DQwtPlot)
SET (figure_name itom1DQwtPlot)
project(${target_name})
 
cmake_minimum_required(VERSION 2.8)

OPTION(BUILD_UNICODE "Build with unicode charset if set to ON, else multibyte charset." ON)
OPTION(BUILD_SHARED_LIBS "Build shared library." ON)
OPTION(BUILD_TARGET64 "Build for 64 bit target if set to ON or 32 bit if set to OFF." OFF)
OPTION(UPDATE_TRANSLATIONS "Update source translation translation/*.ts files (WARNING: make clean will delete the source .ts files! Danger!)")
SET (ITOM_SDK_DIR "" CACHE PATH "base path to itom_sdk")
SET (CMAKE_DEBUG_POSTFIX "d" CACHE STRING "Adds a postfix for debug-built libraries.")
SET (ITOM_LANGUAGES "de" CACHE STRING "semicolon separated list of languages that should be created (en must not be given since it is the default)")

SET (CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${PROJECT_SOURCE_DIR} ${ITOM_SDK_DIR})

IF(BUILD_SHARED_LIBS)
    SET(LIBRARY_TYPE SHARED)
ELSE(BUILD_SHARED_LIBS)
    SET(LIBRARY_TYPE STATIC)
ENDIF(BUILD_SHARED_LIBS)

set(CMAKE_INCLUDE_CURRENT_DIR ON)

find_package(ITOM_SDK COMPONENTS dataobject itomCommonLib itomCommonQtLib itomWidgets itomShapeLib REQUIRED)
include("${ITOM_SDK_DIR}/ItomBuildMacros.cmake")
FIND_PACKAGE_QT(ON Core Widgets Xml Designer PrintSupport Svg LinguistTools PrintSupport)

find_package(VisualLeakDetector QUIET)

IF (BUILD_UNICODE)
    ADD_DEFINITIONS(-DUNICODE -D_UNICODE)
ENDIF (BUILD_UNICODE)
ADD_DEFINITIONS(-DCMAKE -DITOMSHAREDDESIGNER)

IF(VISUALLEAKDETECTOR_FOUND AND VISUALLEAKDETECTOR_ENABLED)
    ADD_DEFINITIONS(-DVISUAL_LEAK_DETECTOR_CMAKE)
ENDIF(VISUALLEAKDETECTOR_FOUND AND VISUALLEAKDETECTOR_ENABLED)

# enable some qt stuff
#SET (QT_USE_QTXML TRUE)

# default build types are None, Debug, Release, RelWithDebInfo and MinRelSize
IF (DEFINED CMAKE_BUILD_TYPE)
    SET(CMAKE_BUILD_TYPE ${CMAKE_BUILD_TYPE} CACHE STRING "Choose the type of build, options are: None(CMAKE_CXX_FLAGS or CMAKE_C_FLAGS used) Debug Release RelWithDebInfo MinSizeRel.")
ELSE(CMAKE_BUILD_TYPE)
    SET (CMAKE_BUILD_TYPE Debug CACHE STRING "Choose the type of build, options are: None(CMAKE_CXX_FLAGS or CMAKE_C_FLAGS used) Debug Release RelWithDebInfo MinSizeRel.")
ENDIF (DEFINED CMAKE_BUILD_TYPE)

message(STATUS ${CMAKE_CURRENT_BINARY_DIR})

INCLUDE_DIRECTORIES(
    ${CMAKE_CURRENT_BINARY_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${QT_QTCORE_INCLUDE_DIR}
    ${ITOM_SDK_INCLUDE_DIRS}
    ${CMAKE_CURRENT_SOURCE_DIR}/icons
    ${CMAKE_CURRENT_SOURCE_DIR}/../qwt/src
    ${VISUALLEAKDETECTOR_INCLUDE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/../sharedFiles
)

LINK_DIRECTORIES(
    ${CMAKE_CURRENT_SOURCE_DIR}/..
)

set(plugin_HEADERS
    ${CMAKE_CURRENT_SOURCE_DIR}/dataObjectSeriesData.h
    ${CMAKE_CURRENT_SOURCE_DIR}/dialog1DScale.h
    ${CMAKE_CURRENT_SOURCE_DIR}/widgetCurveProperties.h
    ${CMAKE_CURRENT_SOURCE_DIR}/itom1DQwtPlot.h
    ${CMAKE_CURRENT_SOURCE_DIR}/itom1DQwtPlotPlugin.h
    ${CMAKE_CURRENT_SOURCE_DIR}/plot1DWidget.h
    ${CMAKE_CURRENT_SOURCE_DIR}/qwtPlotCurveDataObject.h
    ${CMAKE_CURRENT_SOURCE_DIR}/qwtPlotCurveProperty.h
    ${CMAKE_CURRENT_SOURCE_DIR}/valuepicker1d.h
    ${CMAKE_CURRENT_SOURCE_DIR}/pluginVersion.h
    ${CMAKE_CURRENT_SOURCE_DIR}/itomPlotMarker.h
    ${CMAKE_CURRENT_SOURCE_DIR}/../sharedFiles/dialogExportProperties.h
    ${CMAKE_CURRENT_SOURCE_DIR}/../sharedFiles/multiPointPickerMachine.h
    ${CMAKE_CURRENT_SOURCE_DIR}/../sharedFiles/userInteractionPlotPicker.h
    ${CMAKE_CURRENT_SOURCE_DIR}/../sharedFiles/drawItem.h
    ${CMAKE_CURRENT_SOURCE_DIR}/../sharedFiles/itomPlotMagnifier.h
    ${CMAKE_CURRENT_SOURCE_DIR}/../sharedFiles/itomPlotZoomer.h
    ${CMAKE_CURRENT_SOURCE_DIR}/../sharedFiles/itomLogLogScaleEngine.h
    ${CMAKE_CURRENT_SOURCE_DIR}/../sharedFiles/itomQwtPlot.h
    ${CMAKE_CURRENT_SOURCE_DIR}/../sharedFiles/itomQwtDObjFigure.h
    ${CMAKE_CURRENT_SOURCE_DIR}/../sharedFiles/itomQwtPlotEnums.h
)

set(plugin_HEADERS_COPY
    ${CMAKE_CURRENT_SOURCE_DIR}/itom1DQwtPlot.h
    ${CMAKE_CURRENT_SOURCE_DIR}/itom1DQwtPlotEnums.h
    ${CMAKE_CURRENT_SOURCE_DIR}/../sharedFiles/itomQwtPlotEnums.h
    ${CMAKE_CURRENT_SOURCE_DIR}/../sharedFiles/itomQwtDObjFigure.h
)

set(plugin_UI
    ${CMAKE_CURRENT_SOURCE_DIR}/dialog1DScale.ui
    ${CMAKE_CURRENT_SOURCE_DIR}/widgetCurveProperties.ui
    ${CMAKE_CURRENT_SOURCE_DIR}/../sharedFiles/dialogExportProperties.ui
)

set(plugin_RCC
    ${CMAKE_CURRENT_SOURCE_DIR}/itom1DQwtPlotPlugin.qrc
    ${CMAKE_CURRENT_SOURCE_DIR}/../itomDesignerPlugins.qrc
)

set(plugin_SOURCES 
    ${CMAKE_CURRENT_SOURCE_DIR}/dataObjectSeriesData.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/dialog1DScale.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/widgetCurveProperties.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/itom1DQwtPlot.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/itom1DQwtPlotPlugin.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/plot1DWidget.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/valuepicker1d.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/qwtPlotCurveDataObject.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/qwtPlotCurveProperty.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/itomPlotMarker.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/../sharedFiles/dialogExportProperties.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/../sharedFiles/multiPointPickerMachine.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/../sharedFiles/userInteractionPlotPicker.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/../sharedFiles/drawItem.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/../sharedFiles/itomPlotMagnifier.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/../sharedFiles/itomPlotZoomer.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/../sharedFiles/itomLogLogScaleEngine.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/../sharedFiles/itomQwtPlot.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/../sharedFiles/itomQwtDObjFigure.cpp
)

#If you want to use automatical metadata for dlls under windows use the following if-case.
if(MSVC)
    list(APPEND plugin_SOURCES ${ITOM_SDK_INCLUDE_DIR}/../designerPluginLibraryVersion.rc)
endif(MSVC)

if (QT5_FOUND)
    #if automoc if OFF, you also need to call QT5_WRAP_CPP here
    QT5_WRAP_UI(plugin_ui_MOC ${plugin_UI})
    QT5_ADD_RESOURCES(plugin_rcc_MOC ${plugin_RCC})
else (QT5_FOUND)
    QT4_WRAP_CPP_ITOM(plugin_HEADERS_MOC ${plugin_HEADERS})
    QT4_WRAP_UI_ITOM(plugin_ui_MOC ${plugin_UI})
    QT4_ADD_RESOURCES(plugin_rcc_MOC ${plugin_RCC})
endif (QT5_FOUND)

file (GLOB EXISTING_TRANSLATION_FILES "translation/*.ts")

ADD_LIBRARY(${target_name} ${LIBRARY_TYPE} ${plugin_SOURCES} ${plugin_HEADERS} ${plugin_HEADERS_MOC} ${plugin_ui_MOC} ${plugin_rcc_MOC} ${EXISTING_TRANSLATION_FILES})

TARGET_LINK_LIBRARIES(${target_name} ${QT_LIBRARIES} ${ITOM_SDK_LIBRARIES} ${VISUALLEAKDETECTOR_LIBRARIES} qwtsrc)
if (QT5_FOUND)
    qt5_use_modules(${target_name} ${QT_COMPONENTS})
endif (QT5_FOUND)

#translation
set (FILES_TO_TRANSLATE ${plugin_SOURCES} ${plugin_HEADERS} ${plugin_UI})
PLUGIN_TRANSLATION(QM_FILES ${target_name} ${UPDATE_TRANSLATIONS} "${EXISTING_TRANSLATION_FILES}" ITOM_LANGUAGES "${FILES_TO_TRANSLATE}")

configure_file(${CMAKE_CURRENT_SOURCE_DIR}/docs/doxygen/doxygen.dox.in ${CMAKE_CURRENT_BINARY_DIR}/docs/doxygen/doxygen.dox )
IF(EXISTS ${ITOM_SDK_DIR}/docs/plotDoc/plot_doc_config.cfg.in)
    configure_file(${ITOM_SDK_DIR}/docs/plotDoc/plot_doc_config.cfg.in ${CMAKE_CURRENT_BINARY_DIR}/docs/userDoc/plot_doc_config.cfg )
ELSEIF()
    message(WARNING "Could not find and configure auto doc config file. Auto is not enabled")
ENDIF()

# COPY SECTION
set(COPY_SOURCES "")
set(COPY_DESTINATIONS "")
ADD_DESIGNERLIBRARY_TO_COPY_LIST(${target_name} COPY_SOURCES COPY_DESTINATIONS)
ADD_DESIGNER_QM_FILES_TO_COPY_LIST(QM_FILES COPY_SOURCES COPY_DESTINATIONS)
ADD_DESIGNERHEADER_TO_COPY_LIST(${target_name} plugin_HEADERS_COPY COPY_SOURCES COPY_DESTINATIONS)
POST_BUILD_COPY_FILES(${target_name} COPY_SOURCES COPY_DESTINATIONS)
