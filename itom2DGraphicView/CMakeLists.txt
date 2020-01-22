set(target_name itomGraphicView)
project(${target_name})

cmake_minimum_required(VERSION 3.0.2)

message(STATUS "\n--------------- PLUGIN ${target_name} ---------------")

option(BUILD_UNICODE "Build with unicode charset if set to ON, else multibyte charset." ON)
option(BUILD_SHARED_LIBS "Build shared library." ON)
option(BUILD_TARGET64 "Build for 64 bit target if set to ON or 32 bit if set to OFF." ON)
option(UPDATE_TRANSLATIONS "Update source translation translation/*.ts files (WARNING: make clean will delete the source .ts files! Danger!)")
set(ITOM_SDK_DIR "" CACHE PATH "base path to itom_sdk")
set(CMAKE_DEBUG_POSTFIX "d" CACHE STRING "Adds a postfix for debug-built libraries.")
set(ITOM_LANGUAGES "de" CACHE STRING "semicolon separated list of languages that should be created (en must not be given since it is the default)")

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${PROJECT_SOURCE_DIR} ${ITOM_SDK_DIR})

include("${ITOM_SDK_DIR}/ItomBuildMacros.cmake")
INIT_ITOM_LIBRARY() #apply general project settings, CMake configurations, policies... for itom and its libraries

if(BUILD_SHARED_LIBS)
    set(LIBRARY_TYPE SHARED)
else(BUILD_SHARED_LIBS)
    set(LIBRARY_TYPE STATIC)
endif(BUILD_SHARED_LIBS)

set(CMAKE_INCLUDE_CURRENT_DIR ON)

find_package(ITOM_SDK COMPONENTS dataobject itomCommonLib itomCommonQtLib itomCommonPlotLib REQUIRED)
FIND_PACKAGE_QT(ON Core Widgets Xml Designer LinguistTools)

find_package(VisualLeakDetector QUIET)

if(BUILD_UNICODE)
    add_definitions(-DUNICODE -D_UNICODE)
endif(BUILD_UNICODE)
add_definitions(-DITOMSHAREDDESIGNER)

if(VISUALLEAKDETECTOR_FOUND AND VISUALLEAKDETECTOR_ENABLED)
    add_definitions(-DVISUAL_LEAK_DETECTOR_CMAKE)
endif(VISUALLEAKDETECTOR_FOUND AND VISUALLEAKDETECTOR_ENABLED)

# enable some qt stuff
#set(QT_USE_QTOPENGL TRUE)
#set(QT_USE_QTXML TRUE)

# default build types are None, Debug, Release, RelWithDebInfo and MinRelSize
if(DEFINED CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE ${CMAKE_BUILD_TYPE} CACHE STRING "Choose the type of build, options are: None(CMAKE_CXX_FLAGS or CMAKE_C_FLAGS used) Debug Release RelWithDebInfo MinSizeRel.")
else()
    set(CMAKE_BUILD_TYPE Debug CACHE STRING "Choose the type of build, options are: None(CMAKE_CXX_FLAGS or CMAKE_C_FLAGS used) Debug Release RelWithDebInfo MinSizeRel.")
endif()

message(STATUS ${CMAKE_CURRENT_BINARY_DIR})

include_directories(
    ${CMAKE_CURRENT_BINARY_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${ITOM_SDK_INCLUDE_DIRS}
    ${CMAKE_CURRENT_SOURCE_DIR}/icons
    ${CMAKE_CURRENT_SOURCE_DIR}/../qwt/src
    ${VISUALLEAKDETECTOR_INCLUDE_DIR}
)

link_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}/..
)

set(plugin_HEADERS
    ${ITOM_SDK_INCLUDE_DIR}/common/apiFunctionsGraphInc.h
    ${ITOM_SDK_INCLUDE_DIR}/common/apiFunctionsInc.h
    ${ITOM_SDK_INCLUDE_DIR}/common/sharedStructures.h
    ${ITOM_SDK_INCLUDE_DIR}/common/sharedStructuresGraphics.h
    ${ITOM_SDK_INCLUDE_DIR}/common/sharedStructuresQt.h
    ${ITOM_SDK_INCLUDE_DIR}/common/typeDefs.h
    ${ITOM_SDK_INCLUDE_DIR}/plot/AbstractDObjFigure.h
    ${ITOM_SDK_INCLUDE_DIR}/plot/AbstractFigure.h
    ${ITOM_SDK_INCLUDE_DIR}/plot/AbstractItomDesignerPlugin.h
    ${ITOM_SDK_INCLUDE_DIR}/plot/AbstractNode.h
    ${CMAKE_CURRENT_SOURCE_DIR}/dialog2DScale.h
    ${CMAKE_CURRENT_SOURCE_DIR}/graphicViewPlot.h
    ${CMAKE_CURRENT_SOURCE_DIR}/graphicViewPlugin.h
    ${CMAKE_CURRENT_SOURCE_DIR}/plotWidget.h
    ${CMAKE_CURRENT_SOURCE_DIR}/dObjToQImage.h
    ${CMAKE_CURRENT_SOURCE_DIR}/pluginVersion.h
)

set(plugin_UI
    ${CMAKE_CURRENT_SOURCE_DIR}/dialog2DScale.ui
)

set(plugin_RCC
    ${CMAKE_CURRENT_SOURCE_DIR}/../itomDesignerPlugins.qrc
)

set(plugin_SOURCES
    ${CMAKE_CURRENT_SOURCE_DIR}/dialog2DScale.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/graphicViewPlot.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/graphicViewPlugin.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/plotWidget.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/dObjToQImage.cpp
)

#If you want to use automatical metadata for dlls under windows use the following if-case.
if(MSVC)
    list(APPEND plugin_SOURCES ${ITOM_SDK_INCLUDE_DIR}/../designerPluginLibraryVersion.rc)
endif()

if(QT5_FOUND)
    #if automoc if OFF, you also need to call QT5_WRAP_CPP here
    QT5_WRAP_UI(plugin_ui_MOC ${plugin_UI})
    QT5_ADD_RESOURCES(plugin_rcc_MOC ${plugin_RCC})
else()
    message(SEND_ERROR "Currently only Qt5 is supported")
endif()

file(GLOB EXISTING_TRANSLATION_FILES "translation/*.ts")

add_library(${target_name} ${LIBRARY_TYPE} ${plugin_SOURCES} ${plugin_HEADERS} ${plugin_HEADERS_MOC} ${plugin_ui_MOC} ${plugin_rcc_MOC} ${EXISTING_TRANSLATION_FILES})

target_link_libraries(${target_name} ${QT_LIBRARIES} ${ITOM_SDK_LIBRARIES} ${QT5_LIBRARIES} ${VISUALLEAKDETECTOR_LIBRARIES})


#translation
set(FILES_TO_TRANSLATE ${plugin_SOURCES} ${plugin_HEADERS} ${plugin_UI})
PLUGIN_TRANSLATION(QM_FILES ${target_name} ${UPDATE_TRANSLATIONS} "${EXISTING_TRANSLATION_FILES}" ITOM_LANGUAGES "${FILES_TO_TRANSLATE}")

configure_file(${CMAKE_CURRENT_SOURCE_DIR}/docs/doxygen/doxygen.dox.in ${CMAKE_CURRENT_BINARY_DIR}/docs/doxygen/doxygen.dox )
if(EXISTS ${ITOM_SDK_DIR}/docs/plotDoc/plot_doc_config.cfg.in)
    configure_file(${ITOM_SDK_DIR}/docs/plotDoc/plot_doc_config.cfg.in ${CMAKE_CURRENT_BINARY_DIR}/docs/userDoc/plot_doc_config.cfg )
elseif()
    message(WARNING "Could not find and configure auto doc config file. Auto is not enabled")
endif()

# COPY SECTION
set(COPY_SOURCES "")
set(COPY_DESTINATIONS "")
ADD_DESIGNERLIBRARY_TO_COPY_LIST(${target_name} COPY_SOURCES COPY_DESTINATIONS)
ADD_DESIGNER_QM_FILES_TO_COPY_LIST(QM_FILES COPY_SOURCES COPY_DESTINATIONS)
POST_BUILD_COPY_FILES(${target_name} COPY_SOURCES COPY_DESTINATIONS)
