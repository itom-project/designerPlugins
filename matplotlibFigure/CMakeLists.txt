SET (target_name matplotlibFigure)

project(${target_name})

cmake_minimum_required(VERSION 2.8)

OPTION(BUILD_UNICODE "Build with unicode charset if set to ON, else multibyte charset." ON)
OPTION(BUILD_SHARED_LIBS "Build shared library." ON)
OPTION(BUILD_TARGET64 "Build for 64 bit target if set to ON or 32 bit if set to OFF." OFF)
SET (ITOM_SDK_DIR "" CACHE PATH "base path to itom_sdk")
SET (CMAKE_DEBUG_POSTFIX "d" CACHE STRING "Adds a postfix for debug-built libraries.")

SET (CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${PROJECT_SOURCE_DIR} ${ITOM_SDK_DIR})

IF(BUILD_SHARED_LIBS)
	SET(LIBRARY_TYPE SHARED)
ELSE(BUILD_SHARED_LIBS)
	SET(LIBRARY_TYPE STATIC)
ENDIF(BUILD_SHARED_LIBS)

find_package(Qt4 REQUIRED)
find_package(ITOM_SDK REQUIRED)

include("${ITOM_SDK_DIR}/ItomBuildMacros.cmake")

ADD_DEFINITIONS(${QT_DEFINITIONS})
IF (BUILD_UNICODE)
	ADD_DEFINITIONS(-DUNICODE -D_UNICODE)
ENDIF (BUILD_UNICODE)
ADD_DEFINITIONS(-DCMAKE)

# enable some qt stuff
#SET (QT_USE_QTOPENGL TRUE)
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
    ${ITOM_SDK_INCLUDE_DIR}
    ${QT_QTCORE_INCLUDE_DIR}
	${CMAKE_CURRENT_SOURCE_DIR}/icons
 )

LINK_DIRECTORIES(
	${CMAKE_CURRENT_SOURCE_DIR}/..
)

INCLUDE(${QT_USE_FILE})

set(designer_HEADERS
    ${ITOM_SDK_INCLUDE_DIR}/plot/AbstractItomDesignerPlugin.h
	${CMAKE_CURRENT_SOURCE_DIR}/matplotlibfigure.h
	${CMAKE_CURRENT_SOURCE_DIR}/matplotlibfigurefactory.h
    ${CMAKE_CURRENT_SOURCE_DIR}/matplotlibSubfigConfig.h
    ${CMAKE_CURRENT_SOURCE_DIR}/matplotlibWidget.h
)

QT4_WRAP_CPP_ITOM(designer_HEADERS_MOC ${designer_HEADERS})

set(designer_ui
    ${CMAKE_CURRENT_SOURCE_DIR}/matplotlibSubfigConfig.ui
)

QT4_WRAP_UI_ITOM(designer_ui_MOC ${designer_ui})

set(designer_rcc
    ${CMAKE_CURRENT_SOURCE_DIR}/matplotlibFigure.qrc
)

set(designer_SOURCES 
	${CMAKE_CURRENT_SOURCE_DIR}/matplotlibfigure.cpp
	${CMAKE_CURRENT_SOURCE_DIR}/matplotlibfigurefactory.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/matplotlibWidget.cpp
)

QT4_ADD_RESOURCES(designer_rcc_MOC ${designer_rcc})
ADD_LIBRARY(${target_name} ${LIBRARY_TYPE} ${designer_SOURCES} ${designer_HEADERS} ${designer_HEADERS_MOC} ${designer_ui_MOC} ${designer_rcc_MOC})

TARGET_LINK_LIBRARIES(${target_name} ${QT_LIBRARIES} ${QT_QTUITOOLS_LIBRARY} ${QT_QTXML_LIBRARY} ${QT_QTDESIGNER_LIBRARY} ${QT_QTOPENGL_LIBRARY} ${QT_QTSVG_LIBRARY})

# COPY SECTION
set(COPY_SOURCES "")
set(COPY_DESTINATIONS "")

# 1. add default file to list
ADD_DESIGNERLIBRARY_TO_COPY_LIST(${target_name} COPY_SOURCES COPY_DESTINATIONS)

# 2. add any file from source to list (EXAMPLE)
#LIST(APPEND COPY_SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/matplotlibWidget.h)
#LIST(APPEND COPY_DESTINATIONS ${ITOM_DIR}/designer)

# copy file of list
POST_BUILD_COPY_FILES(${target_name} COPY_SOURCES COPY_DESTINATIONS)
