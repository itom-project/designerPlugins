#********************************************************************
#    vtk3dVisualizer-Plugin for itom
#    URL: http://www.uni-stuttgart.de/ito
#    Copyright (C) 2020, Institut für Technische Optik (ITO),
#    Universität Stuttgart, Germany
#
#    This files is part of the designer-Plugin vtk3dVisualizer for the
#    measurement software itom. All files of this plugin, where not stated
#    as port of the itom sdk, fall under the GNU Library General
#    Public Licence and must behandled accordingly.
#
#    vtk3dVisualizer is free software; you can redistribute it and/or modify it
#    under the terms of the GNU Library General Public Licence as published by
#    the Free Software Foundation; either version 2 of the Licence, or (at
#    your option) any later version.
#
#    This itom-plugin is distributed in the hope that it will be useful,
#    but WITHOUT ANY WARRANTY; without even the implied warranty of
#    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
#    GNU General Public License for more details.
#
#    itom is free software by ITO, Universität Stuttgart published under
#    GNU General Public License as published by the Free Software
#    Foundation. See <https://github.com/itom-project/itom>
#
#    You should have received a copy of the GNU Library General Public License
#    along with itom. If not, see <http://www.gnu.org/licenses/>.
#*********************************************************************** */

cmake_minimum_required(VERSION 3.12...3.29)
set(target_name vtk3dVisualizer)

#this is to automatically detect the SDK subfolder of the itom build directory.
if(NOT EXISTS ${ITOM_SDK_DIR})
    find_path(ITOM_SDK_DIR "cmake/itom_sdk.cmake"
    HINTS "$ENV{ITOM_SDK_ROOT}"
          "${CMAKE_CURRENT_BINARY_DIR}/../itom/SDK"
    DOC "Path of SDK subfolder of itom root (build) directory")
endif(NOT EXISTS ${ITOM_SDK_DIR})

if(NOT EXISTS ${ITOM_SDK_DIR})
    message(FATAL_ERROR "ITOM_SDK_DIR is invalid. Provide itom SDK directory path first")
endif(NOT EXISTS ${ITOM_SDK_DIR})

# ...which contains useful scripts for completing this file
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${ITOM_SDK_DIR}/cmake)
set(CMAKE_INCLUDE_CURRENT_DIR ON)

# ... for example this one, found in the CMAKE_MODULE_PATH
include(ItomBuildMacros)
itom_init_cmake_policy(3.12)
itom_init_designerplugin_library(${target_name})

#this is required to get the state of ITOM_SDK_PCL_SUPPORT
find_package(ITOM_SDK COMPONENTS dataobject itomCommonLib itomCommonQtLib itomCommonPlotLib pointcloud qpropertyeditor REQUIRED)

if(ITOM_SDK_PCL_SUPPORT)
    #find_package(ITOM_SDK COMPONENTS dataobject itomCommonLib itomCommonQtLib itomCommonPlotLib pointcloud qpropertyeditor REQUIRED)

    itom_find_package_qt(ON Core Widgets Designer OpenGL LinguistTools)


    find_package(VTK QUIET COMPONENTS vtkCommonCore QUIET)

    include(Setup_PCL_Plugins)
    if(WIN32)
        find_package(PCL 1.5.1 REQUIRED COMPONENTS common kdtree search surface filters visualization PATHS ${PCL_CMAKE_DIR} NO_DEFAULT_PATH)
    else(WIN32)
        find_package(PCL 1.5.1 REQUIRED COMPONENTS common kdtree search surface filters visualization)
    endif(WIN32)
    add_definitions(-DUSEPCL -D_USEPCL)

    if(PCL_FOUND)


        add_definitions(-DITOMSHAREDDESIGNER)

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
            ${CMAKE_CURRENT_SOURCE_DIR}/icons
        )

        link_directories(
            ${CMAKE_CURRENT_SOURCE_DIR}/..
            ${VTK_DIR}
        )

        set(DESIGNER_HEADERS
            ${CMAKE_CURRENT_SOURCE_DIR}/vtk3dVisualizerFactory.h
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
            ${CMAKE_CURRENT_BINARY_DIR}/pluginVersion.h
            ${CMAKE_CURRENT_BINARY_DIR}/gitVersion.h
        )

        set(DESIGNER_HEADERS_COPY
            ${CMAKE_CURRENT_SOURCE_DIR}/vtk3dVisualizer.h
        )

        set(DESIGNER_UI
        )

        set(DESIGNER_RCC
            ${CMAKE_CURRENT_SOURCE_DIR}/../itomDesignerPlugins.qrc
            ${CMAKE_CURRENT_SOURCE_DIR}/vtk3dVisualizer.qrc
        )

        set(DESIGNER_SOURCES
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

        #Define Version in pluginVersion.h
        configure_file( ${CMAKE_CURRENT_SOURCE_DIR}/pluginVersion.h.in ${CMAKE_CURRENT_BINARY_DIR}/pluginVersion.h)

        #If you want to use automatically metadata for dlls under windows use the following if-case.
        if(MSVC)
            list(APPEND DESIGNER_SOURCES ${ITOM_SDK_INCLUDE_DIR}/../designerPluginLibraryVersion.rc)
        endif()

        add_library(${target_name} SHARED ${DESIGNER_SOURCES} ${DESIGNER_HEADERS} ${DESIGNER_UI} ${DESIGNER_RCC})

        if(WIN32)
            target_link_libraries(${target_name} ${PCL_LIBRARIES} ${VTK_LIBRARIES} ${ITOM_SDK_LIBRARIES} ${QT5_LIBRARIES} ${VISUALLEAKDETECTOR_LIBRARIES})
        else(WIN32)
            target_link_libraries(${target_name} ${VTK_LIBRARIES} ${ITOM_SDK_LIBRARIES} ${QT5_LIBRARIES} ${VISUALLEAKDETECTOR_LIBRARIES})
        endif(WIN32)

        # Qt: enable all automoc, autouic and autorcc. Autouic file will be disabled for all files that are processed manually by itom_qt_wrap_ui.
        set_target_properties(${target_name} PROPERTIES AUTOMOC ON AUTORCC ON AUTOUIC ON)

        # translations
        set(FILES_TO_TRANSLATE ${DESIGNER_SOURCES} ${DESIGNER_HEADERS} ${DESIGNER_UI})
        itom_library_translation(QM_FILES TARGET ${target_name} FILES_TO_TRANSLATE ${FILES_TO_TRANSLATE})

        configure_file(${CMAKE_CURRENT_SOURCE_DIR}/docs/doxygen/doxygen.dox.in ${CMAKE_CURRENT_BINARY_DIR}/docs/doxygen/doxygen.dox )

        # COPY SECTION
        set(COPY_SOURCES "")
        set(COPY_DESTINATIONS "")
        itom_add_designerlibrary_to_copy_list(${target_name} COPY_SOURCES COPY_DESTINATIONS)
        itom_add_designer_qm_files_to_copy_list(QM_FILES COPY_SOURCES COPY_DESTINATIONS)
        itom_add_designerplugin_headers_to_copy_list(${target_name} DESIGNER_HEADERS_COPY COPY_SOURCES COPY_DESTINATIONS)
        itom_post_build_copy_files(${target_name} COPY_SOURCES COPY_DESTINATIONS)

    else(PCL_FOUND)
        message(WARNING "PCL is not found. ${target_name} will not be build.")
    endif(PCL_FOUND)

else(ITOM_SDK_PCL_SUPPORT)
    message(SEND_ERROR "${target_name} will not be build since itom has bfn compiled without PCL support")
endif(ITOM_SDK_PCL_SUPPORT)
