project(itom2dQwtPlot)

cmake_minimum_required(VERSION 2.8)

#OPTION(BUILD_SHARED_LIBS "Build shared library." ON)
OPTION(BUILD_TARGET64 "Build for 64 bit target if set to ON or 32 bit if set to OFF." OFF) 

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
#find_package(PCL 1.6 REQUIRED)
#find_package(NumPy REQUIRED)

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
    #${ITOM_SDK_DIR}/common/addInGrabber.h
    #${ITOM_SDK_DIR}/common/addInInterface.h
    ${ITOM_SDK_DIR}/common/apiFunctionsGraphInc.h
    ${ITOM_SDK_DIR}/common/apiFunctionsInc.h
    #${ITOM_SDK_DIR}/common/helperActuator.h
    #${ITOM_SDK_DIR}/common/helperCommon.h
    #${ITOM_SDK_DIR}/common/helperGrabber.h
    #${ITOM_SDK_DIR}/common/pclHelperFunctions.h
    #${ITOM_SDK_DIR}/common/pclStructures.h
    #${ITOM_SDK_DIR}/common/sharedFunctionsQt.h
    ${ITOM_SDK_DIR}/common/sharedStructures.h
    ${ITOM_SDK_DIR}/common/sharedStructuresGraphics.h
    ${ITOM_SDK_DIR}/common/sharedStructuresQt.h
    #${ITOM_SDK_DIR}/common/typeDefs.h
    ${CMAKE_CURRENT_SOURCE_DIR}/../AbstractDObjFigure.h
    ${CMAKE_CURRENT_SOURCE_DIR}/../AbstractFigure.h
    ${CMAKE_CURRENT_SOURCE_DIR}/../abstractItomDesignerPlugin.h
    ${CMAKE_CURRENT_SOURCE_DIR}/../AbstractNodeV2.h
	${CMAKE_CURRENT_SOURCE_DIR}/dataObjItem.h
	${CMAKE_CURRENT_SOURCE_DIR}/itom2dqwtplot.h
    ${CMAKE_CURRENT_SOURCE_DIR}/itom2dqwtplotplugin.h
    ${CMAKE_CURRENT_SOURCE_DIR}/plotCanvas.h
)

QT4_WRAP_CPP_ITOM(designer_HEADERS_MOC ${designer_HEADERS})

#set(designer_ui
#    ${CMAKE_CURRENT_SOURCE_DIR}/dialog2DScale.ui
#)

QT4_WRAP_UI_ITOM(designer_ui_MOC ${designer_ui})

#set(designer_rcc
#    ${CMAKE_CURRENT_SOURCE_DIR}/itom2dQwtPlot.qrc
#)

set(designer_SOURCES 
    #${ITOM_SDK_DIR}/common/addInGrabber.cpp
    #${ITOM_SDK_DIR}/common/addInInterface.cpp
    #${ITOM_SDK_DIR}/common/helperActuator.cpp
    #${ITOM_SDK_DIR}/common/helperCommon.cpp
    #${ITOM_SDK_DIR}/common/helperGrabber.cpp
    #${ITOM_SDK_DIR}/common/pclStructures.cpp
    #${ITOM_SDK_DIR}/common/sharedFunctionsQt.cpp
    ${ITOM_SDK_DIR}/common/sharedStructures.cpp
    ${ITOM_SDK_DIR}/common/sharedStructuresGraphics.cpp
    ${ITOM_SDK_DIR}/common/sharedStructuresQt.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/../AbstractDObjFigure.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/../AbstractFigure.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/../AbstractNodeV2.cpp
	${CMAKE_CURRENT_SOURCE_DIR}/dataObjItem.cpp
	${CMAKE_CURRENT_SOURCE_DIR}/itom2dqwtplot.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/itom2dqwtplotplugin.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/plotCanvas.cpp
)

SET(CMAKE_RUNTIME_OUTPUT_DIRECTORY ../../designer)
SET(CMAKE_LIBRARY_OUTPUT_DIRECTORY ../../designer)
QT4_ADD_RESOURCES(designer_rcc_MOC ${designer_rcc})
ADD_LIBRARY(itom2dQwtPlot ${LIBRARY_TYPE} ${designer_SOURCES} ${designer_HEADERS_MOC} ${designer_ui_MOC} ${designer_rcc_MOC})

#IF (WIN32)
#  set(OpenCVVerStr ${OpenCV_VERSION_MAJOR}${OpenCV_VERSION_MINOR}${OpenCV_VERSION_PATCH})
#  set(OpenCVCoreLib opencv_core${OpenCVVerStr})
#ELSE (WIN32)
#  set(OpenCVCoreLib opencv_core)  
#ENDIF (WIN32)

TARGET_LINK_LIBRARIES(itom2dQwtPlot ${QT_LIBRARIES} ${QT_QTUITOOLS_LIBRARY} ${QT_QTXML_LIBRARY} ${QT_QTDESIGNER_LIBRARY} ${QT_QTOPENGL_LIBRARY} ${QT_QTSVG_LIBRARY} dataobject glu32 opengl32 qwtsrc)