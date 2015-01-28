SET (target_name vtk3dVisualizer)
SET (figure_name vtk3dVisualizer)
project(${target_name})

cmake_minimum_required(VERSION 2.8)

OPTION(BUILD_UNICODE "Build with unicode charset if set to ON, else multibyte charset." ON)
OPTION(BUILD_SHARED_LIBS "Build shared library." ON)
OPTION(BUILD_TARGET64 "Build for 64 bit target if set to ON or 32 bit if set to OFF." OFF)
OPTION(UPDATE_TRANSLATIONS "Update source translation translation/*.ts files (WARNING: make clean will delete the source .ts files! Danger!)")
SET (ITOM_SDK_DIR "" CACHE PATH "base path to itom_sdk")
SET (CMAKE_DEBUG_POSTFIX "d" CACHE STRING "Adds a postfix for debug-built libraries.")
SET (VTK_QT_DIR ${VTK_DIR} CACHE PATH "path to the vtk directory with qt support (the lib folder must lie in this folder")
SET (VTK_VERSION_STR "vtk-5.8" CACHE STRING "Version of vtk, that is the folder name (e.g. vtk-5.8)")
SET (ITOM_LANGUAGES "de" CACHE STRING "semicolon separated list of languages that should be created (en must not be given since it is the default)")

SET (CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${PROJECT_SOURCE_DIR} ${ITOM_SDK_DIR})

IF(BUILD_SHARED_LIBS)
    SET(LIBRARY_TYPE SHARED)
ELSE(BUILD_SHARED_LIBS)
    SET(LIBRARY_TYPE STATIC)
ENDIF(BUILD_SHARED_LIBS)

set(CMAKE_INCLUDE_CURRENT_DIR ON)

find_package(ITOM_SDK COMPONENTS dataobject itomCommonLib itomCommonQtLib pointcloud qpropertyeditor REQUIRED)

IF(ITOM_SDK_PCL_SUPPORT)
    
    include("${ITOM_SDK_DIR}/ItomBuildMacros.cmake")
    FIND_PACKAGE_QT(ON Core Widgets LinguistTools Designer)

    SET(VTK_INSTALL_EXPORT_NAME 1) #avoid that vtk directly links to an absolute Qt-library, that is located somewhere on their computer configuration.
    #SET(ANDROID_NDK TRUE) #hack, such that tkCommon vtkRendering and vtkHybrid are not linked in PCLConfig.cmake, since VTK is externally linked
    find_package(PCL 1.5.1 REQUIRED COMPONENTS common kdtree search surface filters) # do not include anything with visualization -> this is done manually since Vtk is externally linked. 
    find_package(VisualLeakDetector QUIET)
    
    IF(EXISTS ${VTK_QT_DIR})
        message(STATUS ${VTK_LIBRARY_DIRS} - ${VTK_LIBRARIES})

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

        INCLUDE_DIRECTORIES(
            ${CMAKE_CURRENT_BINARY_DIR}
            ${CMAKE_CURRENT_SOURCE_DIR}
            ${PCL_INCLUDE_DIRS}
            ${ITOM_SDK_INCLUDE_DIRS}
            ${VTK_QT_DIR}/include/${VTK_VERSION_STR}
            ${VISUALLEAKDETECTOR_INCLUDE_DIR}
            ${CMAKE_CURRENT_SOURCE_DIR}/icons
        )

        LINK_DIRECTORIES(
            ${CMAKE_CURRENT_SOURCE_DIR}/..
            ${VTK_QT_DIR}/lib/${VTK_VERSION_STR}
            ${VTK_QT_DIR}/lib
        )

        set(plugin_HEADERS
            ${CMAKE_CURRENT_SOURCE_DIR}/vtk3dVisualizerFactory.h
            ${CMAKE_CURRENT_SOURCE_DIR}/pluginVersion.h
            ${CMAKE_CURRENT_SOURCE_DIR}/vtk3dVisualizer.h
            ${ITOM_SDK_INCLUDE_DIR}/DataObject/dataObjectFuncs.h
            ${CMAKE_CURRENT_SOURCE_DIR}/coordSysProperty.h
            ${CMAKE_CURRENT_SOURCE_DIR}/CustomTypes.h
            ${CMAKE_CURRENT_SOURCE_DIR}/item.h
            ${CMAKE_CURRENT_SOURCE_DIR}/itemCanvas.h
            ${CMAKE_CURRENT_SOURCE_DIR}/itemPointCloud.h
            ${CMAKE_CURRENT_SOURCE_DIR}/itemPointCloudNormal.h
            ${CMAKE_CURRENT_SOURCE_DIR}/itemPolygonMesh.h
            ${CMAKE_CURRENT_SOURCE_DIR}/Vec3fProperty.h
            ${CMAKE_CURRENT_SOURCE_DIR}/itemGeometry.h
        )

		set(plugin_HEADERS_COPY
			${CMAKE_CURRENT_SOURCE_DIR}/vtk3dVisualizer.h
		)

        set(plugin_UI
            ${CMAKE_CURRENT_SOURCE_DIR}/vtk3dVisualizer.ui
        )

        set(plugin_RCC
            ${CMAKE_CURRENT_SOURCE_DIR}/../itomDesignerPlugins.qrc
        )

        set(plugin_SOURCES 
            ${CMAKE_CURRENT_SOURCE_DIR}/vtk3dVisualizerFactory.cpp
            ${CMAKE_CURRENT_SOURCE_DIR}/vtk3dVisualizer.cpp
            ${CMAKE_CURRENT_SOURCE_DIR}/coordSysProperty.cpp
            ${CMAKE_CURRENT_SOURCE_DIR}/CustomTypes.cpp
            ${CMAKE_CURRENT_SOURCE_DIR}/item.cpp
            ${CMAKE_CURRENT_SOURCE_DIR}/itemCanvas.cpp
            ${CMAKE_CURRENT_SOURCE_DIR}/itemPointCloud.cpp
            ${CMAKE_CURRENT_SOURCE_DIR}/itemPointCloudNormal.cpp
            ${CMAKE_CURRENT_SOURCE_DIR}/itemPolygonMesh.cpp
            ${CMAKE_CURRENT_SOURCE_DIR}/Vec3fProperty.cpp
            ${CMAKE_CURRENT_SOURCE_DIR}/itemGeometry.cpp
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
            QT4_WRAP_CPP_ITOM(plugin_HEADERS_MOC ${plugin_HEADERS})
            QT4_WRAP_UI_ITOM(plugin_ui_MOC ${plugin_UI})
            QT4_ADD_RESOURCES(plugin_rcc_MOC ${plugin_RCC})
        endif (QT5_FOUND)
        
        string(REPLACE "common" "visualization" PCL_VISUALIZATION_LIBRARY_DEBUG_TEMP ${PCL_COMMON_LIBRARY_DEBUG})
        string(REPLACE "common" "visualization" PCL_VISUALIZATION_LIBRARY_TEMP ${PCL_COMMON_LIBRARY})
        
        set(PCLVTK_LIBS debug ${PCL_VISUALIZATION_LIBRARY_DEBUG_TEMP} optimized ${PCL_VISUALIZATION_LIBRARY_TEMP})
        
        STRING(REGEX MATCH "[0-9]\\.[0-9]$" VTK_VERSION_SUBSTR ${VTK_VERSION_STR})
        
        IF(${VTK_VERSION_STR} MATCHES "(.*)6(.*)")
            set(VTK_COMPONENTS vtkCommonCore vtkCommonMath vtkRenderingCore vtkRenderingLOD vtkRenderingQt vtkGUISupportQt vtkCommonDataModel vtkIOCore vtkViewsCore vtksys vtkzlib vtkfreetype vtklibxml2 vtkexpat vtkpng vtktiff vtkjpeg vtkftgl)
            foreach(__comps ${VTK_COMPONENTS})
                set(PCLVTK_LIBS ${PCLVTK_LIBS} debug ${__comps}-${VTK_VERSION_SUBSTR}-gd.lib optimized ${__comps}-${VTK_VERSION_SUBSTR}.lib)
            endforeach(__comps)
        ELSE()
            set(VTK_COMPONENTS QVTK vtkGraphics vtkIO vtkCommon vtkViews vtkWidgets vtkRendering vtksys vtkFiltering vtkzlib vtkfreetype vtklibxml2 vtkexpat vtkpng vtktiff vtkjpeg vtkftgl)
            foreach(__comps ${VTK_COMPONENTS})
                set(PCLVTK_LIBS ${PCLVTK_LIBS} debug ${__comps}-gd.lib optimized ${__comps}.lib)
            endforeach(__comps)
        ENDIF()

        file (GLOB EXISTING_TRANSLATION_FILES "translation/*.ts")

        ADD_LIBRARY(${target_name} ${LIBRARY_TYPE} ${plugin_SOURCES} ${plugin_HEADERS} ${plugin_HEADERS_MOC} ${plugin_ui_MOC} ${plugin_rcc_MOC} ${EXISTING_TRANSLATION_FILES})

        TARGET_LINK_LIBRARIES(${target_name} ${QT_LIBRARIES} ${PCL_LIBRARIES} ${PCLVTK_LIBS} ${ITOM_SDK_LIBRARIES} ${VISUALLEAKDETECTOR_LIBRARIES} opengl32)
        if (QT5_FOUND)
            qt5_use_modules(${target_name} ${QT_COMPONENTS})
        endif (QT5_FOUND)

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
        
    ELSE(EXISTS ${VTK_QT_DIR})
        message(WARNING "VTK_QT_DIR does not exist. ${target_name} will not be build.")
    ENDIF(EXISTS ${VTK_QT_DIR})

ELSE(ITOM_SDK_PCL_SUPPORT)
    message(FATAL "${target_name} will not be build since itom has been compiled without PCL support")
ENDIF(ITOM_SDK_PCL_SUPPORT)