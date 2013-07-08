SET (target_name itom2dQwtPlot)

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
find_package(OpenCV REQUIRED COMPONENTS core)
#find_package(Glew REQUIRED)
#find_package(Glut REQUIRED)
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

# using custom macro for qtCreator compability, i.e. put ui files into GeneratedFiles/ folder
MACRO (QT4_WRAP_UI outfiles )
  QT4_EXTRACT_OPTIONS(ui_files ui_options ${ARGN})

  file(MAKE_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/../GeneratedFiles)

  FOREACH (it ${ui_files})
    GET_FILENAME_COMPONENT(outfile ${it} NAME_WE)
    GET_FILENAME_COMPONENT(infile ${it} ABSOLUTE)
    SET(outfile ${CMAKE_CURRENT_SOURCE_DIR}/../GeneratedFiles/ui_${outfile}.h) # Here we set output
    ADD_CUSTOM_COMMAND(OUTPUT ${outfile}
      COMMAND ${QT_UIC_EXECUTABLE}
      ARGS ${ui_options} -o ${outfile} ${infile}
      MAIN_DEPENDENCY ${infile})
    SET(${outfiles} ${${outfiles}} ${outfile})
  ENDFOREACH (it)

ENDMACRO (QT4_WRAP_UI)

# default build types are None, Debug, Release, RelWithDebInfo and MinRelSize
IF (DEFINED CMAKE_BUILD_TYPE)
  SET(CMAKE_BUILD_TYPE ${CMAKE_BUILD_TYPE} CACHE STRING "Choose the type of build, options are: None(CMAKE_CXX_FLAGS or CMAKE_C_FLAGS used) Debug Release RelWithDebInfo MinSizeRel.")
ELSE(CMAKE_BUILD_TYPE)
  SET (CMAKE_BUILD_TYPE Debug CACHE STRING "Choose the type of build,
options are: None(CMAKE_CXX_FLAGS or CMAKE_C_FLAGS used) Debug Release
RelWithDebInfo MinSizeRel.")
ENDIF (DEFINED CMAKE_BUILD_TYPE)

INCLUDE_DIRECTORIES(
    ${CMAKE_CURRENT_BINARY_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${QT_QTCORE_INCLUDE_DIR}
    ${OpenCV_DIR}/include
    ${ITOM_SDK_INCLUDE_DIR}
	${CMAKE_CURRENT_SOURCE_DIR}/icons
	${CMAKE_CURRENT_SOURCE_DIR}/../qwt/src
 )

LINK_DIRECTORIES(
    ${CMAKE_CURRENT_BINARY_DIR}/../../DataObject
	${CMAKE_CURRENT_SOURCE_DIR}/..
    ${OpenCV_DIR}/lib
#	${CMAKE_CURRENT_SOURCE_DIR}/../3rdParty/glew/lib
)

INCLUDE(${QT_USE_FILE})

message(STATUS ${CMAKE_CURRENT_BINARY_DIR})

set(designer_HEADERS
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
    ${CMAKE_CURRENT_SOURCE_DIR}/dataObjItem.h
    ${CMAKE_CURRENT_SOURCE_DIR}/dataObjRasterData.h
    ${CMAKE_CURRENT_SOURCE_DIR}/itom2dqwtplot.h
    ${CMAKE_CURRENT_SOURCE_DIR}/itom2dqwtplotplugin.h
	${CMAKE_CURRENT_SOURCE_DIR}/valuePicker2d.h
	${CMAKE_CURRENT_SOURCE_DIR}/dialog2DScale.h
    ${CMAKE_CURRENT_SOURCE_DIR}/plotCanvas.h
    ${CMAKE_CURRENT_SOURCE_DIR}/pluginVersion.h
)

QT4_WRAP_CPP_ITOM(designer_HEADERS_MOC ${designer_HEADERS})

set(designer_ui
    ${CMAKE_CURRENT_SOURCE_DIR}/dialog2DScale.ui
)

QT4_WRAP_UI_ITOM(designer_ui_MOC ${designer_ui})

#set(designer_rcc
#    ${CMAKE_CURRENT_SOURCE_DIR}/itom2dQwtPlot.qrc
#)

set(designer_SOURCES 
    ${ITOM_SDK_INCLUDE_DIR}/common/sharedStructures.cpp
    ${ITOM_SDK_INCLUDE_DIR}/common/sharedStructuresGraphics.cpp
    ${ITOM_SDK_INCLUDE_DIR}/common/sharedStructuresQt.cpp
    ${ITOM_SDK_INCLUDE_DIR}/plot/AbstractDObjFigure.cpp
    ${ITOM_SDK_INCLUDE_DIR}/plot/AbstractFigure.cpp
    ${ITOM_SDK_INCLUDE_DIR}/plot/AbstractNode.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/dataObjItem.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/dataObjRasterData.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/itom2dqwtplot.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/itom2dqwtplotplugin.cpp
	${CMAKE_CURRENT_SOURCE_DIR}/valuePicker2d.cpp
	${CMAKE_CURRENT_SOURCE_DIR}/dialog2DScale.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/plotCanvas.cpp
)

#If you want to use automatical metadata for dlls under windows use the following if-case.
if(MSVC)
    list(APPEND designer_SOURCES ${ITOM_SDK_INCLUDE_DIR}/../pluginLibraryVersion.rc)
endif(MSVC)

QT4_ADD_RESOURCES(designer_rcc_MOC ${designer_rcc})
ADD_LIBRARY(${target_name} ${LIBRARY_TYPE} ${designer_SOURCES} ${designer_HEADERS_MOC} ${designer_HEADERS} ${designer_ui_MOC} ${designer_rcc_MOC})

TARGET_LINK_LIBRARIES(${target_name} ${QT_LIBRARIES} ${QT_QTUITOOLS_LIBRARY} ${QT_QTXML_LIBRARY} ${QT_QTDESIGNER_LIBRARY} ${QT_QTOPENGL_LIBRARY} ${QT_QTSVG_LIBRARY} ${OpenCV_LIBS} ${ITOM_SDK_dataobject_LIBRARY} qwtsrc)

# COPY SECTION
set(COPY_SOURCES "")
set(COPY_DESTINATIONS "")
ADD_DESIGNERLIBRARY_TO_COPY_LIST(${target_name} COPY_SOURCES COPY_DESTINATIONS)
POST_BUILD_COPY_FILES(${target_name} COPY_SOURCES COPY_DESTINATIONS)