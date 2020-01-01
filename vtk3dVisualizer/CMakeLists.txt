SET (target_name vtk3dVisualizer)
SET (figure_name vtk3dVisualizer)
project(${target_name})

cmake_minimum_required(VERSION 2.8)

#CMAKE Policies
if (POLICY CMP0028)
    cmake_policy(SET CMP0028 NEW) #raise an CMake error if an imported target, containing ::, could not be found
ENDIF (POLICY CMP0028)

message(STATUS "\n--------------- PLUGIN ${target_name} ---------------")

OPTION(BUILD_UNICODE "Build with unicode charset if set to ON, else multibyte charset." ON)
OPTION(BUILD_SHARED_LIBS "Build shared library." ON)
OPTION(BUILD_TARGET64 "Build for 64 bit target if set to ON or 32 bit if set to OFF." ON)
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

#this is required to get the state of ITOM_SDK_PCL_SUPPORT
find_package(ITOM_SDK COMPONENTS dataobject itomCommonLib itomCommonQtLib itomCommonPlotLib REQUIRED)
include("${ITOM_SDK_DIR}/ItomBuildMacros.cmake")

IF(ITOM_SDK_PCL_SUPPORT)
    find_package(ITOM_SDK COMPONENTS dataobject itomCommonLib itomCommonQtLib itomCommonPlotLib pointcloud qpropertyeditor REQUIRED)
    
    FIND_PACKAGE_QT(ON Core Widgets Designer OpenGL LinguistTools)

    #SET(VTK_INSTALL_EXPORT_NAME 1) #avoid that vtk directly links to an absolute Qt-library, that is located somewhere on their computer configuration.
    #SET(ANDROID_NDK TRUE) #hack, such that tkCommon vtkRendering and vtkHybrid are not linked in PCLConfig.cmake, since VTK is externally linked

    find_package(VTK)
    IF(EXISTS ${VTK_DIR})
    
        find_package(PCL 1.5.1 REQUIRED COMPONENTS common kdtree search surface filters visualization) # do not include anything with visualization -> this is done manually since Vtk is externally linked. 
        find_package(VisualLeakDetector QUIET)
        
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
        
        #PCL does not have a proper version number. Sometimes, new enumerations or methods are introduced without incrementing
        #the version. Therefore, a search has to be done to check for the availability of some features.
        FIND_FILE(PCL_VISUALIZER_COMMON_H common.h PATHS ${PCL_INCLUDE_DIRS} PATH_SUFFIXES pcl/visualization/common)
        IF (PCL_VISUALIZER_COMMON_H)
            FILE(READ ${PCL_VISUALIZER_COMMON_H} TMPTXT)
            STRING(FIND "${TMPTXT}" "PCL_VISUALIZER_LUT_BLUE2RED" matchres)
            IF(${matchres} EQUAL -1)
                MESSAGE(WARNING "PCL is no final version of 1.8.0 and does not have the enumeration pcl::visualization::LookUpTableRepresentationProperties with the value PCL_VISUALIZER_LUT_BLUE2RED. Not all features are available.")
            ELSE()
                ADD_DEFINITIONS(-DPCL_HASLUT)
            ENDIF()  
        ENDIF()

        INCLUDE_DIRECTORIES(
            ${CMAKE_CURRENT_BINARY_DIR}
            ${CMAKE_CURRENT_SOURCE_DIR}
            ${PCL_INCLUDE_DIRS}
            ${ITOM_SDK_INCLUDE_DIRS}
            ${VISUALLEAKDETECTOR_INCLUDE_DIR}
            ${CMAKE_CURRENT_SOURCE_DIR}/icons
        )

        LINK_DIRECTORIES(
            ${CMAKE_CURRENT_SOURCE_DIR}/..
            ${Boost_LIBRARY_DIR}
            ${BOOST_LIBRARYDIR}
            ${VTK_DIR}
        )

        set(plugin_HEADERS
            ${CMAKE_CURRENT_SOURCE_DIR}/vtk3dVisualizerFactory.h
            ${CMAKE_CURRENT_SOURCE_DIR}/pluginVersion.h
            ${CMAKE_CURRENT_SOURCE_DIR}/vtk3dVisualizer.h
            ${CMAKE_CURRENT_SOURCE_DIR}/coordSysProperty.h
            ${CMAKE_CURRENT_SOURCE_DIR}/CustomTypes.h
            ${CMAKE_CURRENT_SOURCE_DIR}/item.h
            ${CMAKE_CURRENT_SOURCE_DIR}/itemPointCloud.h
            ${CMAKE_CURRENT_SOURCE_DIR}/itemPointCloudNormal.h
            ${CMAKE_CURRENT_SOURCE_DIR}/itemPolygonMesh.h
            ${CMAKE_CURRENT_SOURCE_DIR}/Vec3fProperty.h
            ${CMAKE_CURRENT_SOURCE_DIR}/itemGeometry.h
            ${CMAKE_CURRENT_SOURCE_DIR}/pointCloudHandlerGenericFields.h
            ${CMAKE_CURRENT_SOURCE_DIR}/treeWidgetKeyEater.h
        )

        set(plugin_HEADERS_COPY
            ${CMAKE_CURRENT_SOURCE_DIR}/vtk3dVisualizer.h
        )

        set(plugin_UI
            ${CMAKE_CURRENT_SOURCE_DIR}/vtk3dVisualizer.ui
        )

        set(plugin_RCC
            ${CMAKE_CURRENT_SOURCE_DIR}/../itomDesignerPlugins.qrc
            ${CMAKE_CURRENT_SOURCE_DIR}/vtk3dVisualizer.qrc
        )

        set(plugin_SOURCES 
            ${CMAKE_CURRENT_SOURCE_DIR}/vtk3dVisualizerFactory.cpp
            ${CMAKE_CURRENT_SOURCE_DIR}/vtk3dVisualizer.cpp
            ${CMAKE_CURRENT_SOURCE_DIR}/coordSysProperty.cpp
            ${CMAKE_CURRENT_SOURCE_DIR}/CustomTypes.cpp
            ${CMAKE_CURRENT_SOURCE_DIR}/item.cpp
            ${CMAKE_CURRENT_SOURCE_DIR}/itemPointCloud.cpp
            ${CMAKE_CURRENT_SOURCE_DIR}/itemPointCloudNormal.cpp
            ${CMAKE_CURRENT_SOURCE_DIR}/itemPolygonMesh.cpp
            ${CMAKE_CURRENT_SOURCE_DIR}/Vec3fProperty.cpp
            ${CMAKE_CURRENT_SOURCE_DIR}/itemGeometry.cpp
            ${CMAKE_CURRENT_SOURCE_DIR}/treeWidgetKeyEater.cpp
        )

        #If you want to use automatical metadata for dlls under windows use the following if-case.
        if(MSVC)
            list(APPEND designer_SOURCES ${ITOM_SDK_INCLUDE_DIR}/../designerPluginLibraryVersion.rc)
        endif(MSVC)

        if (QT5_FOUND)
            #if automoc if OFF, you also need to call QT5_WRAP_CPP here
            QT5_WRAP_UI(plugin_ui_MOC ${plugin_UI})
            QT5_ADD_RESOURCES(plugin_rcc_MOC ${plugin_RCC})
        else (QT5_FOUND)
            MESSAGE(SEND_ERROR "Currently only Qt5 is supported")
        endif (QT5_FOUND)
        
        set(PCLVTK_LIBS "")
        
        IF("${VTK_MAJOR_VERSION}" EQUAL 5)
            set(VTK_COMPONENTS QVTK)
            foreach(__comps ${VTK_COMPONENTS})
                set(PCLVTK_LIBS ${PCLVTK_LIBS} debug ${__comps}-gd.lib optimized ${__comps}.lib)
            endforeach(__comps)
        ENDIF()

        file (GLOB EXISTING_TRANSLATION_FILES "translation/*.ts")

        ADD_LIBRARY(${target_name} ${LIBRARY_TYPE} ${plugin_SOURCES} ${plugin_HEADERS} ${plugin_HEADERS_MOC} ${plugin_ui_MOC} ${plugin_rcc_MOC} ${EXISTING_TRANSLATION_FILES})

        TARGET_LINK_LIBRARIES(${target_name} ${QT_LIBRARIES} ${PCL_LIBRARIES} ${PCLVTK_LIBS} ${ITOM_SDK_LIBRARIES} ${QT5_LIBRARIES} ${VISUALLEAKDETECTOR_LIBRARIES})
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
        ADD_DESIGNERHEADER_TO_COPY_LIST(${target_name} plugin_HEADERS_COPY COPY_SOURCES COPY_DESTINATIONS)
        POST_BUILD_COPY_FILES(${target_name} COPY_SOURCES COPY_DESTINATIONS)
        
    ELSE(EXISTS ${VTK_DIR})
        message(WARNING "VTK_DIR does not exist. ${target_name} will not be build.")
    ENDIF(EXISTS ${VTK_DIR})

ELSE(ITOM_SDK_PCL_SUPPORT)
    message(WARNING "${target_name} will not be build since itom has been compiled without PCL support")
ENDIF(ITOM_SDK_PCL_SUPPORT)