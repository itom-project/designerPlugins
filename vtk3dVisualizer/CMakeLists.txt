set(target_name vtk3dVisualizer)

cmake_minimum_required(VERSION 3.1...3.15)

set(ITOM_SDK_DIR NOTFOUND CACHE PATH "path of SDK subfolder of itom root (build) directory")

#this is to automatically detect the SDK subfolder of the itom build directory.
find_path(ITOM_SDK_DIR "cmake/itom_sdk.cmake"
    HINTS "C:/itom/build/itom/SDK"
          "${CMAKE_CURRENT_BINARY_DIR}/../../itom/SDK"
    DOC "path of SDK subfolder of itom root (build) directory")

if(NOT ITOM_SDK_DIR)
    message(SEND_ERROR "ITOM_SDK_DIR is invalid. Provide itom SDK directory path first")
endif()

# ...which contains useful scripts for completing this file
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${ITOM_SDK_DIR}/cmake)
set(CMAKE_INCLUDE_CURRENT_DIR ON)
option(VTK3DVISUALIZER_FORCE_LEGACY_VTK "Usually with vtk >= 8, the new QVTKOpenGLNativeWidget is used, else the legacy QVTKWidget. If this option is set to ON, the legacy version is also used for VTK8." OFF)

# ... for example this one, found in the CMAKE_MODULE_PATH
include(ItomBuildMacros)
itom_init_cmake_policy(3.12)
itom_init_designerplugin_library(${target_name})

#this is required to get the state of ITOM_SDK_PCL_SUPPORT
find_package(ITOM_SDK COMPONENTS dataobject itomCommonLib itomCommonQtLib itomCommonPlotLib REQUIRED)

if(ITOM_SDK_PCL_SUPPORT)
    find_package(ITOM_SDK COMPONENTS dataobject itomCommonLib itomCommonQtLib itomCommonPlotLib pointcloud qpropertyeditor REQUIRED)
    
    itom_find_package_qt(ON Core Widgets Designer OpenGL LinguistTools)

    #set(VTK_INSTALL_EXPORT_NAME 1) #avoid that vtk directly links to an absolute Qt-library, that is located somewhere on their computer configuration.
    #set(ANDROID_NDK TRUE) #hack, such that tkCommon vtkRendering and vtkHybrid are not linked in PCLConfig.cmake, since VTK is externally linked
    
    # vtk 8.0 is required to support QOpenGLWidget, vtk 6.0 is required to support the deprecated QVTKWidget
    find_package(VTK)
    if(EXISTS ${VTK_DIR})
    
        find_package(PCL 1.5.1 REQUIRED COMPONENTS common kdtree search surface filters visualization) # do not include anything with visualization -> this is done manually since Vtk is externally linked. 
        
        if(${VTK_MAJOR_VERSION} LESS 8)
            add_definitions(-DLEGACY_VTK)
        elseif(${VTK_MAJOR_VERSION} GREATER_EQUAL 9)
            # VTK9 does not support the legacy interface
            if (VTK3DVISUALIZER_FORCE_LEGACY_VTK)
                message(WARNING "For VTK9 or higher, the legacy VTK Qt widget is not supported.")
            endif()
        elseif(VTK3DVISUALIZER_FORCE_LEGACY_VTK)
            add_definitions(-DLEGACY_VTK)
        elseif(Qt5_VERSION VERSION_LESS 5.10.0)
            message(WARNING "For Qt < 5.10 it is recommended to enable VTK3DVISUALIZER_FORCE_LEGACY_VTK")
        endif()

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
            ${Boost_LIBRARY_DIR}
            ${BOOST_LIBRARYDIR}
            ${VTK_DIR}
        )

        set(DESIGNER_HEADERS
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

        #If you want to use automatical metadata for dlls under windows use the following if-case.
        if(MSVC)
            list(APPEND DESIGNER_SOURCES ${ITOM_SDK_INCLUDE_DIR}/../designerPluginLibraryVersion.rc)
        endif()
        
        set(PCLVTK_LIBS "")
        
        if("${VTK_MAJOR_VERSION}" EQUAL 5)
            set(VTK_COMPONENTS QVTK)
            foreach(__comps ${VTK_COMPONENTS})
                set(PCLVTK_LIBS ${PCLVTK_LIBS} debug ${__comps}-gd.lib optimized ${__comps}.lib)
            endforeach(__comps)
        endif()
        
        add_library(${target_name} SHARED ${DESIGNER_SOURCES} ${DESIGNER_HEADERS} ${DESIGNER_UI} ${DESIGNER_RCC})
        
        target_link_libraries(${target_name} ${PCL_LIBRARIES} ${PCLVTK_LIBS} ${ITOM_SDK_LIBRARIES} ${QT5_LIBRARIES} ${VISUALLEAKDETECTOR_LIBRARIES})
        
        
        # Qt: enable all automoc, autouic and autorcc. Autouic file will be disabled for all files that are processed manually by itom_qt_wrap_ui.
set_target_properties(${target_name} PROPERTIES AUTOMOC ON AUTORCC ON AUTOUIC ON)

        if(QT5_FOUND)
            #parse all *.ui files by Qt's uic process and get the parsed source files
            qt5_wrap_ui(DESIGNER_UIC ${DESIGNER_UI})
            #add the output files to the target
            target_sources(${target_name} PRIVATE ${DESIGNER_UIC})
        else()
            message(SEND_ERROR "Currently only Qt5 is supported")
        endif()

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
        
    else(EXISTS ${VTK_DIR})
        message(WARNING "VTK_DIR does not exist. ${target_name} will not be build.")
    endif(EXISTS ${VTK_DIR})

else(ITOM_SDK_PCL_SUPPORT)
    message(WARNING "${target_name} will not be build since itom has been compiled without PCL support")
endif(ITOM_SDK_PCL_SUPPORT)