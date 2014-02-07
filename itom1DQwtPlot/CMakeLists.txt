SET (target_name itom1DQwtPlot)

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

find_package(Qt4 REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core)
#find_package(Glut REQUIRED)
find_package(ITOM_SDK REQUIRED)
find_package(VisualLeakDetector QUIET)

include("${ITOM_SDK_DIR}/ItomBuildMacros.cmake")

ADD_DEFINITIONS(${QT_DEFINITIONS})
IF (BUILD_UNICODE)
    ADD_DEFINITIONS(-DUNICODE -D_UNICODE)
ENDIF (BUILD_UNICODE)
ADD_DEFINITIONS(-DCMAKE -DITOMSHAREDDESIGNER)

IF(VISUALLEAKDETECTOR_FOUND AND VISUALLEAKDETECTOR_ENABLED)
    ADD_DEFINITIONS(-DVISUAL_LEAK_DETECTOR_CMAKE)
ENDIF(VISUALLEAKDETECTOR_FOUND AND VISUALLEAKDETECTOR_ENABLED)

# enable some qt stuff
SET (QT_USE_QTXML TRUE)

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
    ${OpenCV_DIR}/include
    ${ITOM_SDK_INCLUDE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/icons
    ${CMAKE_CURRENT_SOURCE_DIR}/../qwt/src
    ${VISUALLEAKDETECTOR_INCLUDE_DIR}
)

LINK_DIRECTORIES(
    ${CMAKE_CURRENT_SOURCE_DIR}/..
    ${OpenCV_DIR}/lib
)

INCLUDE(${QT_USE_FILE})

set(designer_HEADERS
    ${ITOM_SDK_INCLUDE_DIR}/common/sharedStructures.h
    ${ITOM_SDK_INCLUDE_DIR}/common/sharedStructuresGraphics.h
    ${ITOM_SDK_INCLUDE_DIR}/common/sharedStructuresPrimitives.h
    ${ITOM_SDK_INCLUDE_DIR}/common/sharedStructuresQt.h
    ${ITOM_SDK_INCLUDE_DIR}/common/typeDefs.h
    ${ITOM_SDK_INCLUDE_DIR}/DataObject/dataObjectFuncs.h
    ${ITOM_SDK_INCLUDE_DIR}/plot/AbstractDObjFigure.h
    ${ITOM_SDK_INCLUDE_DIR}/plot/AbstractFigure.h
    ${ITOM_SDK_INCLUDE_DIR}/plot/AbstractItomDesignerPlugin.h
    ${ITOM_SDK_INCLUDE_DIR}/plot/AbstractNode.h
    ${CMAKE_CURRENT_SOURCE_DIR}/dataObjectSeriesData.h
    ${CMAKE_CURRENT_SOURCE_DIR}/dialog1DScale.h
    ${CMAKE_CURRENT_SOURCE_DIR}/itom1DQwtPlot.h
    ${CMAKE_CURRENT_SOURCE_DIR}/itom1DQwtPlotPlugin.h
    ${CMAKE_CURRENT_SOURCE_DIR}/plot1DWidget.h
    ${CMAKE_CURRENT_SOURCE_DIR}/qwtPlotCurveDataObject.h
    ${CMAKE_CURRENT_SOURCE_DIR}/valuepicker1d.h
    ${CMAKE_CURRENT_SOURCE_DIR}/pluginVersion.h
    ${CMAKE_CURRENT_SOURCE_DIR}/../sharedFiles/dialogExportProperties.h
    ${CMAKE_CURRENT_SOURCE_DIR}/../sharedFiles/multiPointPickerMachine.h
    ${CMAKE_CURRENT_SOURCE_DIR}/../sharedFiles/userInteractionPlotPicker.h
    ${CMAKE_CURRENT_SOURCE_DIR}/../sharedFiles/drawItem.h
)

QT4_WRAP_CPP_ITOM(designer_HEADERS_MOC ${designer_HEADERS})

set(designer_ui
    ${CMAKE_CURRENT_SOURCE_DIR}/dialog1DScale.ui
     ${CMAKE_CURRENT_SOURCE_DIR}/../sharedFiles/dialogExportProperties.ui
)

QT4_WRAP_UI_ITOM(designer_ui_MOC ${designer_ui})

set(designer_HEADERS_COPY
	${CMAKE_CURRENT_SOURCE_DIR}/itom1DQwtPlot.h
)

set(designer_rcc
    ${CMAKE_CURRENT_SOURCE_DIR}/itom1DQwtPlotPlugin.qrc
    ${CMAKE_CURRENT_SOURCE_DIR}/../itomDesignerPlugins.qrc
)

set(designer_SOURCES 
    ${CMAKE_CURRENT_SOURCE_DIR}/dataObjectSeriesData.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/dialog1DScale.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/itom1DQwtPlot.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/itom1DQwtPlotPlugin.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/plot1DWidget.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/valuepicker1d.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/qwtPlotCurveDataObject.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/../sharedFiles/dialogExportProperties.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/../sharedFiles/multiPointPickerMachine.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/../sharedFiles/userInteractionPlotPicker.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/../sharedFiles/drawItem.cpp
)

#If you want to use automatical metadata for dlls under windows use the following if-case.
if(MSVC)
    list(APPEND designer_SOURCES ${ITOM_SDK_INCLUDE_DIR}/../designerPluginLibraryVersion.rc)
endif(MSVC)

#handle translations BEGIN STEP 1
set (FILES_TO_TRANSLATE ${designer_SOURCES} ${designer_HEADERS} ${designer_ui})
set (TRANSLATION_OUTPUT_FILES)
set (TRANSLATIONS_FILES)

file (GLOB EXISTING_TRANSLATION_FILES "translation/*.ts")
#handle translations END STEP 1

QT4_ADD_RESOURCES(designer_rcc_MOC ${designer_rcc})
ADD_LIBRARY(${target_name} ${LIBRARY_TYPE} ${designer_SOURCES} ${designer_HEADERS} ${designer_HEADERS_MOC} ${designer_ui_MOC} ${designer_rcc_MOC} ${EXISTING_TRANSLATION_FILES})

TARGET_LINK_LIBRARIES(${target_name} ${QT_LIBRARIES} ${QT_QTUITOOLS_LIBRARY} ${QT_QTXML_LIBRARY} ${QT_QTDESIGNER_LIBRARY} ${QT_QTOPENGL_LIBRARY} ${QT_QTSVG_LIBRARY} ${OpenCV_LIBS} ${ITOM_SDK_dataobject_LIBRARY} ${ITOM_SDK_qpropertyeditor_LIBRARY} ${VISUALLEAKDETECTOR_LIBRARIES} qwtsrc ${ITOM_SDK_itomCommonLib_LIBRARY} ${ITOM_SDK_itomCommonQtLib_LIBRARY})

#handle translations BEGIN STEP 2
if (UPDATE_TRANSLATIONS)
    QT4_CREATE_TRANSLATION_ITOM(TRANSLATION_OUTPUT_FILES TRANSLATIONS_FILES ${target_name} ITOM_LANGUAGES ${FILES_TO_TRANSLATE} ${TRANSLATIONS_FILES})
    
    add_custom_target (_${target_name}_translation DEPENDS ${TRANSLATION_OUTPUT_FILES})
    add_dependencies(${target_name} _${target_name}_translation)
    
    QT4_ADD_TRANSLATION_ITOM(QM_FILES "${CMAKE_CURRENT_BINARY_DIR}/translation" ${target_name} ${TRANSLATIONS_FILES})
else (UPDATE_TRANSLATIONS)
    QT4_ADD_TRANSLATION_ITOM(QM_FILES "${CMAKE_CURRENT_BINARY_DIR}/translation" ${target_name} ${EXISTING_TRANSLATION_FILES})
endif (UPDATE_TRANSLATIONS)
#handle translations END STEP 2

# COPY SECTION
set(COPY_SOURCES "")
set(COPY_DESTINATIONS "")
ADD_DESIGNERLIBRARY_TO_COPY_LIST(${target_name} COPY_SOURCES COPY_DESTINATIONS)
ADD_DESIGNER_QM_FILES_TO_COPY_LIST(QM_FILES COPY_SOURCES COPY_DESTINATIONS)
ADD_DESIGNERHEADER_TO_COPY_LIST(${target_name} designer_HEADERS_COPY COPY_SOURCES COPY_DESTINATIONS)
POST_BUILD_COPY_FILES(${target_name} COPY_SOURCES COPY_DESTINATIONS)
