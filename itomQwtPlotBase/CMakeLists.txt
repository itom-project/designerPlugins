set(target_name itomQwtPlotBase)     

project(${target_name})

cmake_minimum_required(VERSION 3.0.2)

message(STATUS "\n--------------- BASE LIBRARY ${target_name} ---------------")

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

find_package(ITOM_SDK COMPONENTS dataobject itomCommonLib itomCommonQtLib itomCommonPlotLib itomWidgets itomShapeLib REQUIRED)
FIND_PACKAGE_QT(ON Core Widgets Xml Designer PrintSupport Svg Concurrent OpenGL LinguistTools)

find_package(VisualLeakDetector QUIET)

if(BUILD_UNICODE)
    add_definitions(-DUNICODE -D_UNICODE)
endif(BUILD_UNICODE)
add_definitions(-DITOMQWTPLOTBASE_DLL)
add_definitions(-DQWT_DLL) #use the shared library version of qwt

if(VISUALLEAKDETECTOR_FOUND AND VISUALLEAKDETECTOR_ENABLED)
    add_definitions(-DVISUAL_LEAK_DETECTOR_CMAKE)
endif(VISUALLEAKDETECTOR_FOUND AND VISUALLEAKDETECTOR_ENABLED)

# enable some qt stuff
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
    ${CMAKE_CURRENT_SOURCE_DIR}/version
    ${ITOM_SDK_INCLUDE_DIRS}
    ${CMAKE_CURRENT_SOURCE_DIR}/../qwt/src
    ${VISUALLEAKDETECTOR_INCLUDE_DIR}
)

link_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}/..
)

set(plugin_HEADERS
    ${CMAKE_CURRENT_SOURCE_DIR}/version/pluginVersion.h
    ${CMAKE_CURRENT_SOURCE_DIR}/dialogExportProperties.h
    ${CMAKE_CURRENT_SOURCE_DIR}/multiPointPickerMachine.h
    ${CMAKE_CURRENT_SOURCE_DIR}/userInteractionPlotPicker.h
    ${CMAKE_CURRENT_SOURCE_DIR}/drawItem.h
    ${CMAKE_CURRENT_SOURCE_DIR}/itomPlotMagnifier.h
    ${CMAKE_CURRENT_SOURCE_DIR}/itomPlotZoomer.h
    ${CMAKE_CURRENT_SOURCE_DIR}/itomLogLogScaleEngine.h
    ${CMAKE_CURRENT_SOURCE_DIR}/itomQwtPlot.h
    ${CMAKE_CURRENT_SOURCE_DIR}/itomQwtDObjFigure.h
    ${CMAKE_CURRENT_SOURCE_DIR}/itomQwtPlotEnums.h
    ${CMAKE_CURRENT_SOURCE_DIR}/itomColorMap.h
    ${CMAKE_CURRENT_SOURCE_DIR}/itomPlotPicker.h
    ${CMAKE_CURRENT_SOURCE_DIR}/itomQwtPlotPanner.h   
)

set(plugin_HEADERS_COPY
    ${CMAKE_CURRENT_SOURCE_DIR}/itomQwtPlotEnums.h
    ${CMAKE_CURRENT_SOURCE_DIR}/itomQwtDObjFigure.h
    ${CMAKE_CURRENT_SOURCE_DIR}/itomQwtPlotBase.h
)

set(plugin_UI
    ${CMAKE_CURRENT_SOURCE_DIR}/dialogExportProperties.ui
)

set(plugin_RCC
    #${CMAKE_CURRENT_SOURCE_DIR}/itom1DQwtPlotPlugin.qrc
    #${CMAKE_CURRENT_SOURCE_DIR}/../itomDesignerPlugins.qrc
)

set(plugin_SOURCES 
    ${CMAKE_CURRENT_SOURCE_DIR}/dialogExportProperties.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/multiPointPickerMachine.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/userInteractionPlotPicker.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/drawItem.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/itomPlotMagnifier.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/itomPlotZoomer.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/itomLogLogScaleEngine.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/itomQwtPlot.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/itomQwtDObjFigure.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/itomColorMap.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/itomPlotPicker.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/itomQwtPlotPanner.cpp
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

target_link_libraries(${target_name} ${QT_LIBRARIES} ${ITOM_SDK_LIBRARIES} ${QT5_LIBRARIES} ${VISUALLEAKDETECTOR_LIBRARIES} qwt)


#translation
set(FILES_TO_TRANSLATE ${plugin_SOURCES} ${plugin_HEADERS} ${plugin_UI})
PLUGIN_TRANSLATION(QM_FILES ${target_name} ${UPDATE_TRANSLATIONS} "${EXISTING_TRANSLATION_FILES}" ITOM_LANGUAGES "${FILES_TO_TRANSLATE}")

# COPY SECTION
set(COPY_SOURCES "")
set(COPY_DESTINATIONS "")
ADD_DESIGNERLIBRARY_TO_COPY_LIST(${target_name} COPY_SOURCES COPY_DESTINATIONS)
ADD_DESIGNER_QM_FILES_TO_COPY_LIST(QM_FILES COPY_SOURCES COPY_DESTINATIONS)
ADD_DESIGNERHEADER_TO_COPY_LIST(${target_name} plugin_HEADERS_COPY COPY_SOURCES COPY_DESTINATIONS)
POST_BUILD_COPY_FILES(${target_name} COPY_SOURCES COPY_DESTINATIONS)
