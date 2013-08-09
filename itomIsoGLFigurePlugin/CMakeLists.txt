SET (target_name itomIsoGLFigurePlugin)

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
find_package(OpenGL REQUIRED)
#find_package(Glut REQUIRED)
find_package(ITOM_SDK REQUIRED)
find_package(GLEW REQUIRED)
find_package(3DCONNEXION)

IF(GLEW_FOUND)

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
        ${QT_QTCORE_INCLUDE_DIR}
        ${OpenCV_DIR}/include
        ${ITOM_SDK_INCLUDE_DIR}
        ${CMAKE_CURRENT_SOURCE_DIR}/icons
        ${CMAKE_CURRENT_SOURCE_DIR}/../qwt/src
        ${GLEW_INCLUDE_PATH}
        ${3DCONNEXION_INCLUDE_PATH}
    )


    LINK_DIRECTORIES(
        ${CMAKE_CURRENT_SOURCE_DIR}/..
        ${OpenCV_DIR}/lib
        ${3DCONNEXION_LIBRARIES}
    )

    INCLUDE(${QT_USE_FILE})

    set(designer_HEADERS
        ${ITOM_SDK_INCLUDE_DIR}/common/apiFunctionsGraphInc.h
        ${ITOM_SDK_INCLUDE_DIR}/common/apiFunctionsInc.h
        ${ITOM_SDK_INCLUDE_DIR}/common/sharedStructures.h
        ${ITOM_SDK_INCLUDE_DIR}/common/sharedStructuresGraphics.h
        ${ITOM_SDK_INCLUDE_DIR}/common/sharedStructuresQt.h
        ${ITOM_SDK_INCLUDE_DIR}/common/typeDefs.h
        ${ITOM_SDK_INCLUDE_DIR}/DataObject/dataObjectFuncs.h
        ${ITOM_SDK_INCLUDE_DIR}/plot/AbstractDObjFigure.h
        ${ITOM_SDK_INCLUDE_DIR}/plot/AbstractFigure.h
        ${ITOM_SDK_INCLUDE_DIR}/plot/AbstractItomDesignerPlugin.h
        ${ITOM_SDK_INCLUDE_DIR}/plot/AbstractNode.h
        ${CMAKE_CURRENT_SOURCE_DIR}/itomIsoGLFigure.h
        ${CMAKE_CURRENT_SOURCE_DIR}/itomIsoGLFigurePlugin.h
        ${CMAKE_CURRENT_SOURCE_DIR}/plotIsoGLWidget.h
        ${CMAKE_CURRENT_SOURCE_DIR}/pluginVersion.h
)

    set(designer_SOURCES 
        ${ITOM_SDK_INCLUDE_DIR}/common/sharedStructures.cpp
        ${ITOM_SDK_INCLUDE_DIR}/common/sharedStructuresGraphics.cpp
        ${ITOM_SDK_INCLUDE_DIR}/common/sharedStructuresQt.cpp
        ${ITOM_SDK_INCLUDE_DIR}/plot/AbstractDObjFigure.cpp
        ${ITOM_SDK_INCLUDE_DIR}/plot/AbstractFigure.cpp
        ${ITOM_SDK_INCLUDE_DIR}/plot/AbstractNode.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/itomIsoGLFigure.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/itomIsoGLFigurePlugin.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/plotIsoGLWidget.cpp
    )

    IF(3DCONNEXION_FOUND)
        ADD_DEFINITIONS(-DCONNEXION_FOUND)
        #list(APPEND  ${LINK_DIRECTORIES} ${3DCONNEXION_LIBRARIES})
        #list(APPEND  ${INCLUDE_DIRECTORIES} ${3DCONNEXION_INCLUDE_PATH})
        #list(APPEND designer_HEADERS ${3DCONNEXION_LIBRARIES}/designerPluginLibraryVersion.rc)
    ENDIF(3DCONNEXION_FOUND)
    
    #If you want to use automatical metadata for dlls under windows use the following if-case.
    if(MSVC)
        list(APPEND designer_SOURCES ${ITOM_SDK_INCLUDE_DIR}/../designerPluginLibraryVersion.rc)
    endif(MSVC) 
    
    QT4_WRAP_CPP_ITOM(designer_HEADERS_MOC ${designer_HEADERS})

    QT4_WRAP_UI_ITOM(designer_ui_MOC ${designer_ui})
    
    
    set(designer_rcc
        ${CMAKE_CURRENT_SOURCE_DIR}/itomIsoGLFigurePlugin.qrc
    )
    
    QT4_ADD_RESOURCES(designer_HEADERS ${designer_rcc})
    
    
    ADD_LIBRARY(${target_name} ${LIBRARY_TYPE} ${designer_SOURCES} ${designer_HEADERS} ${designer_HEADERS_MOC} ${designer_ui_MOC} ${designer_rcc_MOC})

    IF(3DCONNEXION_FOUND)
        Message(STATUS "3DCONNEXION was found ${3DCONNEXION_SIAPP}.")
        
        TARGET_LINK_LIBRARIES(${target_name} ${QT_LIBRARIES} ${3DCONNEXION_SIAPP} ${QT_QTUITOOLS_LIBRARY} ${QT_QTXML_LIBRARY} ${QT_QTDESIGNER_LIBRARY} ${QT_QTOPENGL_LIBRARY} ${QT_QTSVG_LIBRARY} ${OpenCV_LIBS} ${OPENGL_LIBRARIES} ${GLEW_LIBRARIES} ${ITOM_SDK_dataobject_LIBRARY}) # ${GLEW_LIBRARIES}) # glu32 opengl32 qwtsrc)
    ELSE(3DCONNEXION_FOUND)
        Message(STATUS "WARNING: 3DCONNEXION was not found.")
        TARGET_LINK_LIBRARIES(${target_name} ${QT_LIBRARIES} ${QT_QTUITOOLS_LIBRARY} ${QT_QTXML_LIBRARY} ${QT_QTDESIGNER_LIBRARY} ${QT_QTOPENGL_LIBRARY} ${QT_QTSVG_LIBRARY} ${OpenCV_LIBS} ${OPENGL_LIBRARIES} ${GLEW_LIBRARIES} ${ITOM_SDK_dataobject_LIBRARY}) # ${GLEW_LIBRARIES}) # glu32 opengl32 qwtsrc)
    ENDIF(3DCONNEXION_FOUND)
    
    # COPY SECTION
    set(COPY_SOURCES "")
    set(COPY_DESTINATIONS "")
    ADD_DESIGNERLIBRARY_TO_COPY_LIST(${target_name} COPY_SOURCES COPY_DESTINATIONS)
    
    IF(WIN32)
        POST_BUILD_COPY_FILE_TO_LIB_FOLDER(${target_name} GLEW_RUNTIME_LIBRARIES)
    ENDIF(WIN32)
    
    POST_BUILD_COPY_FILES(${target_name} COPY_SOURCES COPY_DESTINATIONS)

ELSE(GLEW_FOUND)
    message(WARNING "glew could not be found. ${target_name} will not be build")
ENDIF(GLEW_FOUND)
