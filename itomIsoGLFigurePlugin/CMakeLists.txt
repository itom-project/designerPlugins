SET (target_name itomIsoGLFigurePlugin)
SET (figure_name itomIsoGLWidget)
project(${target_name})

cmake_minimum_required(VERSION 2.8)

#CMAKE Policies
if (POLICY CMP0028)
    cmake_policy(SET CMP0028 NEW) #raise an CMake error if an imported target, containing ::, could not be found
ENDIF (POLICY CMP0028)

message(STATUS "\n--------------- PLUGIN ${target_name} ---------------")

OPTION(BUILD_UNICODE "Build with unicode charset if set to ON, else multibyte charset." ON)
OPTION(BUILD_SHARED_LIBS "Build shared library." ON)
OPTION(BUILD_TARGET64 "Build for 64 bit target if set to ON or 32 bit if set to OFF." OFF)
OPTION(BUILD_ISOGL_3DCONNEXION "If 3DConnexion is installed, support is build into the ISOGL-Widget." OFF)
OPTION(UPDATE_TRANSLATIONS "Update source translation translation/*.ts files (WARNING: make clean will delete the source .ts files! Danger!)")
SET (ITOM_SDK_DIR "" CACHE PATH "base path to itom_sdk")
SET (CMAKE_DEBUG_POSTFIX "d" CACHE STRING "Adds a postfix for debug-built libraries.")
SET (ITOM_LANGUAGES "de" CACHE STRING "semicolon separated list of languages that should be created (en must not be given since it is the default)")
set(CMAKE_INCLUDE_CURRENT_DIR ON)

SET (CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${PROJECT_SOURCE_DIR} ${ITOM_SDK_DIR})

IF(BUILD_SHARED_LIBS)
    SET(LIBRARY_TYPE SHARED)
ELSE(BUILD_SHARED_LIBS)
    SET(LIBRARY_TYPE STATIC)
ENDIF(BUILD_SHARED_LIBS)

find_package(ITOM_SDK COMPONENTS dataobject itomCommonLib itomCommonQtLib itomCommonPlotLib REQUIRED)
include("${ITOM_SDK_DIR}/ItomBuildMacros.cmake")

if (ITOM_SDK_PCL_SUPPORT) #this comes from itom_sdk.cmake, describing the build configuration of itom
    find_package(ITOM_SDK COMPONENTS dataobject itomCommonLib itomCommonQtLib pointcloud itomCommonPlotLib REQUIRED)
endif (ITOM_SDK_PCL_SUPPORT)

FIND_PACKAGE_QT(ON Core Widgets Designer Xml Svg UiTools OpenGL LinguistTools)
find_package(OpenGL)

IF(OPENGL_FOUND)

    find_package(VisualLeakDetector QUIET)

    if (ITOM_SDK_PCL_SUPPORT) #this comes from itom_sdk.cmake, describing the build configuration of itom
        find_package(PCL 1.5.1 REQUIRED COMPONENTS common)
        ADD_DEFINITIONS(-DUSEPCL -D_USEPCL)
    endif (ITOM_SDK_PCL_SUPPORT)

    IF(BUILD_ISOGL_3DCONNEXION)
        find_package(3DCONNEXION)
        message(STATUS "3DConnexion enabled by user")
    ELSE(BUILD_ISOGL_3DCONNEXION)
        set(3DCONNEXION_FOUND false)
        message(STATUS "3DConnexion disabled by user")
    ENDIF(BUILD_ISOGL_3DCONNEXION)

    IF (BUILD_UNICODE)
        ADD_DEFINITIONS(-DUNICODE -D_UNICODE)
    ENDIF (BUILD_UNICODE)
    ADD_DEFINITIONS(-DCMAKE -DITOMSHAREDDESIGNER)

    IF(VISUALLEAKDETECTOR_FOUND AND VISUALLEAKDETECTOR_ENABLED)
        ADD_DEFINITIONS(-DVISUAL_LEAK_DETECTOR_CMAKE)
    ENDIF(VISUALLEAKDETECTOR_FOUND AND VISUALLEAKDETECTOR_ENABLED)

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
        ${3DCONNEXION_INCLUDE_PATH}
        ${VISUALLEAKDETECTOR_INCLUDE_DIR}
        ${PCL_INCLUDE_DIRS}
        ${Boost_INCLUDE_DIR}
    )

    IF(UNIX)
        INCLUDE_DIRECTORIES(
        usr/include #for GL/GL.h and GL/GLU.h
        )
    ENDIF(UNIX)

    LINK_DIRECTORIES(
        ${CMAKE_CURRENT_SOURCE_DIR}/..
        ${3DCONNEXION_LIBRARIES}
    )

    set(plugin_HEADERS
        ${ITOM_SDK_INCLUDE_DIR}/common/apiFunctionsGraphInc.h
        ${ITOM_SDK_INCLUDE_DIR}/common/apiFunctionsInc.h
        ${ITOM_SDK_INCLUDE_DIR}/common/sharedStructures.h
        ${ITOM_SDK_INCLUDE_DIR}/common/sharedStructuresGraphics.h
        ${ITOM_SDK_INCLUDE_DIR}/common/sharedStructuresQt.h
        ${ITOM_SDK_INCLUDE_DIR}/common/typeDefs.h
        ${ITOM_SDK_INCLUDE_DIR}/DataObject/dataObjectFuncs.h
        ${ITOM_SDK_INCLUDE_DIR}/plot/AbstractDObjPCLFigure.h
        ${ITOM_SDK_INCLUDE_DIR}/plot/AbstractFigure.h
        ${ITOM_SDK_INCLUDE_DIR}/plot/AbstractItomDesignerPlugin.h
        ${ITOM_SDK_INCLUDE_DIR}/plot/AbstractNode.h
        ${CMAKE_CURRENT_SOURCE_DIR}/itomIsoGLFigure.h
        ${CMAKE_CURRENT_SOURCE_DIR}/itomIsoGLFigurePlugin.h
        ${CMAKE_CURRENT_SOURCE_DIR}/plotIsoGLWidget.h
        ${CMAKE_CURRENT_SOURCE_DIR}/pluginVersion.h
    )

    set(plugin_UI
    #    ${CMAKE_CURRENT_SOURCE_DIR}/Iso.ui
    )

    set(plugin_RCC
        ${CMAKE_CURRENT_SOURCE_DIR}/../itomDesignerPlugins.qrc
        ${CMAKE_CURRENT_SOURCE_DIR}/itomIsoGLFigurePlugin.qrc
    )

    set(plugin_SOURCES 
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

    IF(3DCONNEXION_FOUND)
        Message(STATUS "3DCONNEXION was found ${3DCONNEXION_SIAPP}.")
        TARGET_LINK_LIBRARIES(${target_name} ${3DCONNEXION_SIAPP} ${3DCONNEXION_SPWMATH} ${QT_LIBRARIES} ${ITOM_SDK_LIBRARIES} ${OPENGL_LIBRARIES} ${QT5_LIBRARIES} ${VISUALLEAKDETECTOR_LIBRARIES})
        
    ELSE(3DCONNEXION_FOUND)
        Message(STATUS "WARNING: 3DCONNEXION was not found or disabled.")
        TARGET_LINK_LIBRARIES(${target_name} ${QT_LIBRARIES} ${ITOM_SDK_LIBRARIES} ${OPENGL_LIBRARIES} ${QT5_LIBRARIES} ${VISUALLEAKDETECTOR_LIBRARIES})
    ENDIF(3DCONNEXION_FOUND)
    
    IF (QT5_FOUND AND CMAKE_VERSION VERSION_LESS 3.0.2)
        qt5_use_modules(${target_name} ${QT_COMPONENTS})
    ENDIF (QT5_FOUND AND CMAKE_VERSION VERSION_LESS 3.0.2)

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
    POST_BUILD_COPY_FILES(${target_name} COPY_SOURCES COPY_DESTINATIONS)

ELSE (OPENGL_FOUND)
    message(WARNING "OpenGL could not be found. ${target_name} will not be build")
ENDIF (OPENGL_FOUND)