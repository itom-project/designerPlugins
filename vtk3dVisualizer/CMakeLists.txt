set(target_name vtk3dVisualizer)
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

#this is required to get the state of ITOM_SDK_PCL_SUPPORT
find_package(ITOM_SDK COMPONENTS dataobject itomCommonLib itomCommonQtLib itomCommonPlotLib REQUIRED)

if(ITOM_SDK_PCL_SUPPORT)
    find_package(ITOM_SDK COMPONENTS dataobject itomCommonLib itomCommonQtLib itomCommonPlotLib pointcloud qpropertyeditor REQUIRED)
    
    FIND_PACKAGE_QT(ON Core Widgets Designer OpenGL LinguistTools)

    #set(VTK_INSTALL_EXPORT_NAME 1) #avoid that vtk directly links to an absolute Qt-library, that is located somewhere on their computer configuration.
    #set(ANDROID_NDK TRUE) #hack, such that tkCommon vtkRendering and vtkHybrid are not linked in PCLConfig.cmake, since VTK is externally linked

    find_package(VTK)
    if(EXISTS ${VTK_DIR})
    
        find_package(PCL 1.5.1 REQUIRED COMPONENTS common kdtree search surface filters visualization) # do not include anything with visualization -> this is done manually since Vtk is externally linked. 
        find_package(VisualLeakDetector QUIET)
        
        if(BUILD_UNICODE)
            add_definitions(-DUNICODE -D_UNICODE)
        endif(BUILD_UNICODE)
        add_definitions(-DCMAKE -DITOMSHAREDDESIGNER)

        if(VISUALLEAKDETECTOR_FOUND AND VISUALLEAKDETECTOR_ENABLED)
            add_definitions(-DVISUAL_LEAK_DETECTOR_CMAKE)
        endif(VISUALLEAKDETECTOR_FOUND AND VISUALLEAKDETECTOR_ENABLED)

        # default build types are None, Debug, Release, RelWithDebInfo and MinRelSize
        if(DEFINED CMAKE_BUILD_TYPE)
            set(CMAKE_BUILD_TYPE ${CMAKE_BUILD_TYPE} CACHE STRING "Choose the type of build, options are: None(CMAKE_CXX_FLAGS or CMAKE_C_FLAGS used) Debug Release RelWithDebInfo MinSizeRel.")
        else()
            set(CMAKE_BUILD_TYPE Debug CACHE STRING "Choose the type of build, options are: None(CMAKE_CXX_FLAGS or CMAKE_C_FLAGS used) Debug Release RelWithDebInfo MinSizeRel.")
        endif()
        
        #PCL does not have a proper version number. Sometimes, new enumerations or methods are introduced without incrementing
        #the version. Therefore, a search has to be done to check for the availability of some features.
        find_file(PCL_VISUALIZER_COMMON_H common.h PATHS ${PCL_INCLUDE_DIRS} PATH_SUFFIXES pcl/visualization/common)
        if(PCL_VISUALIZER_COMMON_H)
            file(READ ${PCL_VISUALIZER_COMMON_H} TMPTXT)
            string(FIND "${TMPTXT}" "PCL_VISUALIZER_LUT_BLUE2RED" matchres)
            if(${matchres} EQUAL -1)
                message(WARNING "PCL is no final version of 1.8.0 and does not have the enumeration pcl::visualization::LookUpTableRepresentationProperties with the value PCL_VISUALIZER_LUT_BLUE2RED. Not all features are available.")
            else()
                add_definitions(-DPCL_HASLUT)
            endif()  
        endif()

        include_directories(
            ${CMAKE_CURRENT_BINARY_DIR}
            ${CMAKE_CURRENT_SOURCE_DIR}
            ${PCL_INCLUDE_DIRS}
            ${ITOM_SDK_INCLUDE_DIRS}
            ${VISUALLEAKDETECTOR_INCLUDE_DIR}
            ${CMAKE_CURRENT_SOURCE_DIR}/icons
        )

        link_directories(
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
        endif()

        if(QT5_FOUND)
            #if automoc if OFF, you also need to call QT5_WRAP_CPP here
            QT5_WRAP_UI(plugin_ui_MOC ${plugin_UI})
            QT5_ADD_RESOURCES(plugin_rcc_MOC ${plugin_RCC})
        else()
            message(SEND_ERROR "Currently only Qt5 is supported")
        endif()
        
        set(PCLVTK_LIBS "")
        
        if("${VTK_MAJOR_VERSION}" EQUAL 5)
            set(VTK_COMPONENTS QVTK)
            foreach(__comps ${VTK_COMPONENTS})
                set(PCLVTK_LIBS ${PCLVTK_LIBS} debug ${__comps}-gd.lib optimized ${__comps}.lib)
            endforeach(__comps)
        endif()

        file(GLOB EXISTING_TRANSLATION_FILES "translation/*.ts")

        add_library(${target_name} ${LIBRARY_TYPE} ${plugin_SOURCES} ${plugin_HEADERS} ${plugin_HEADERS_MOC} ${plugin_ui_MOC} ${plugin_rcc_MOC} ${EXISTING_TRANSLATION_FILES})

        target_link_libraries(${target_name} ${QT_LIBRARIES} ${PCL_LIBRARIES} ${PCLVTK_LIBS} ${ITOM_SDK_LIBRARIES} ${QT5_LIBRARIES} ${VISUALLEAKDETECTOR_LIBRARIES})

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
        ADD_DESIGNERHEADER_TO_COPY_LIST(${target_name} plugin_HEADERS_COPY COPY_SOURCES COPY_DESTINATIONS)
        POST_BUILD_COPY_FILES(${target_name} COPY_SOURCES COPY_DESTINATIONS)
        
    else(EXISTS ${VTK_DIR})
        message(WARNING "VTK_DIR does not exist. ${target_name} will not be build.")
    endif(EXISTS ${VTK_DIR})

else(ITOM_SDK_PCL_SUPPORT)
    message(WARNING "${target_name} will not be build since itom has been compiled without PCL support")
endif(ITOM_SDK_PCL_SUPPORT)