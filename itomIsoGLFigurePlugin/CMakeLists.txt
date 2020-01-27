set(target_name itomIsoGLFigurePlugin)
project(${target_name})

option(BUILD_TARGET64 "Build for 64 bit target if set to ON or 32 bit if set to OFF." ON)
option(BUILD_ISOGL_3DCONNEXION "If 3DConnexion is installed, support is build into the ISOGL-Widget." OFF)
option(UPDATE_TRANSLATIONS "Update source translation translation/*.ts files (WARNING: make clean will delete the source .ts files! Danger!)")
set(ITOM_SDK_DIR "" CACHE PATH "base path to itom_sdk")
set(CMAKE_DEBUG_POSTFIX "d" CACHE STRING "Adds a postfix for debug-built libraries.")
set(ITOM_LANGUAGES "de" CACHE STRING "semicolon separated list of languages that should be created (en must not be given since it is the default)")

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${PROJECT_SOURCE_DIR} ${ITOM_SDK_DIR}/cmake)
set(CMAKE_INCLUDE_CURRENT_DIR ON)

include("ItomBuildMacros")
itom_init_cmake_policy()
itom_init_designerplugin_library(${target_name})

find_package(ITOM_SDK COMPONENTS dataobject itomCommonLib itomCommonQtLib itomCommonPlotLib REQUIRED)

if(ITOM_SDK_PCL_SUPPORT) #this comes from itom_sdk.cmake, describing the build configuration of itom
    find_package(ITOM_SDK COMPONENTS dataobject itomCommonLib itomCommonQtLib pointcloud itomCommonPlotLib REQUIRED)
endif(ITOM_SDK_PCL_SUPPORT)

itom_find_package_qt(ON Core Widgets Designer Xml Svg UiTools OpenGL LinguistTools)
find_package(OpenGL)

if(OPENGL_FOUND)

    find_package(VisualLeakDetector QUIET)

    if(ITOM_SDK_PCL_SUPPORT) #this comes from itom_sdk.cmake, describing the build configuration of itom
        find_package(PCL 1.5.1 REQUIRED COMPONENTS common)
        add_definitions(-DUSEPCL -D_USEPCL)
    endif(ITOM_SDK_PCL_SUPPORT)

    if(BUILD_ISOGL_3DCONNEXION)
        find_package(3DCONNEXION)
        message(STATUS "3DConnexion enabled by user")
    else(BUILD_ISOGL_3DCONNEXION)
        set(3DCONNEXION_FOUND false)
        message(STATUS "3DConnexion disabled by user")
    endif(BUILD_ISOGL_3DCONNEXION)


    add_definitions(-DITOMSHAREDDESIGNER)

    if(VISUALLEAKDETECTOR_FOUND AND VISUALLEAKDETECTOR_ENABLED)
        add_definitions(-DVISUAL_LEAK_DETECTOR_CMAKE)
    endif(VISUALLEAKDETECTOR_FOUND AND VISUALLEAKDETECTOR_ENABLED)

    # default build types are None, Debug, Release, RelWithDebInfo and MinRelSize
    if(DEFINED CMAKE_BUILD_TYPE)
        set(CMAKE_BUILD_TYPE ${CMAKE_BUILD_TYPE} CACHE STRING "Choose the type of build, options are: None(CMAKE_CXX_FLAGS or CMAKE_C_FLAGS used) Debug Release RelWithDebInfo MinSizeRel.")
    else()
        set(CMAKE_BUILD_TYPE Debug CACHE STRING "Choose the type of build, options are: None(CMAKE_CXX_FLAGS or CMAKE_C_FLAGS used) Debug Release RelWithDebInfo MinSizeRel.")
    endif()

    

    include_directories(
        ${CMAKE_CURRENT_BINARY_DIR}
        ${CMAKE_CURRENT_SOURCE_DIR}
        ${ITOM_SDK_INCLUDE_DIRS}
        ${CMAKE_CURRENT_SOURCE_DIR}/icons
        ${CMAKE_CURRENT_SOURCE_DIR}/../qwt/src
        ${3DCONNEXION_INCLUDE_PATH}
        ${VISUALLEAKDETECTOR_INCLUDE_DIR}
        ${PCL_INCLUDE_DIRS}
    )

    if(UNIX)
        include_directories(
        usr/include #for GL/GL.h and GL/GLU.h
        )
    endif(UNIX)

    link_directories(
        ${CMAKE_CURRENT_SOURCE_DIR}/..
        ${3DCONNEXION_LIBRARIES}
    )

    set(DESIGNER_HEADERS
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

    set(DESIGNER_UI
    #    ${CMAKE_CURRENT_SOURCE_DIR}/Iso.ui
    )

    set(DESIGNER_RCC
        ${CMAKE_CURRENT_SOURCE_DIR}/../itomDesignerPlugins.qrc
        ${CMAKE_CURRENT_SOURCE_DIR}/itomIsoGLFigurePlugin.qrc
    )

    set(DESIGNER_SOURCES 
        ${CMAKE_CURRENT_SOURCE_DIR}/itomIsoGLFigure.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/itomIsoGLFigurePlugin.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/plotIsoGLWidget.cpp
    )

    if(3DCONNEXION_FOUND)
        add_definitions(-DCONNEXION_FOUND)
        #list(APPEND  ${link_directories} ${3DCONNEXION_LIBRARIES})
        #list(APPEND  ${include_directories} ${3DCONNEXION_INCLUDE_PATH})
        #list(APPEND DESIGNER_HEADERS ${3DCONNEXION_LIBRARIES}/designerPluginLibraryVersion.rc)
    endif(3DCONNEXION_FOUND)

    #If you want to use automatical metadata for dlls under windows use the following if-case.
    if(MSVC)
        list(APPEND DESIGNER_SOURCES ${ITOM_SDK_INCLUDE_DIR}/../designerPluginLibraryVersion.rc)
    endif() 

    add_library(${target_name} SHARED ${DESIGNER_SOURCES} ${DESIGNER_HEADERS} ${DESIGNER_UI} ${DESIGNER_RCC})

    if(3DCONNEXION_FOUND)
        Message(STATUS "3DCONNEXION was found ${3DCONNEXION_SIAPP}.")
        target_link_libraries(${target_name} ${3DCONNEXION_SIAPP} ${3DCONNEXION_SPWMATH} ${ITOM_SDK_LIBRARIES} ${OPENGL_LIBRARIES} ${QT5_LIBRARIES} ${VISUALLEAKDETECTOR_LIBRARIES})
        
    else(3DCONNEXION_FOUND)
        Message(STATUS "WARNING: 3DCONNEXION was not found or disabled.")
        target_link_libraries(${target_name} ${ITOM_SDK_LIBRARIES} ${OPENGL_LIBRARIES} ${QT5_LIBRARIES} ${VISUALLEAKDETECTOR_LIBRARIES})
    endif(3DCONNEXION_FOUND)
    
    
    # Qt: enable all automoc, autouic and autorcc. Autouic file will be disabled for all files that are processed manually by itom_qt_wrap_ui.
    set_target_properties(${target_name} PROPERTIES AUTOMOC ON AUTORCC ON AUTOUIC ON)

    # manually parse the ui files to obtain the output files for the translation below
    itom_qt_wrap_ui(DESIGNER_UIC ${target_name} ${DESIGNER_UI})

    # translations
    file(GLOB EXISTING_TRANSLATION_FILES "translation/*.ts")
    set(FILES_TO_TRANSLATE ${DESIGNER_SOURCES} ${DESIGNER_HEADERS} ${DESIGNER_UIC})
    itom_library_translation(QM_FILES ${target_name} ${UPDATE_TRANSLATIONS} "${EXISTING_TRANSLATION_FILES}" ITOM_LANGUAGES "${FILES_TO_TRANSLATE}")

    configure_file(${CMAKE_CURRENT_SOURCE_DIR}/docs/doxygen/doxygen.dox.in ${CMAKE_CURRENT_BINARY_DIR}/docs/doxygen/doxygen.dox )

    # COPY SECTION
    set(COPY_SOURCES "")
    set(COPY_DESTINATIONS "")
    itom_add_designerlibrary_to_copy_list(${target_name} COPY_SOURCES COPY_DESTINATIONS)
    itom_add_designer_qm_files_to_copy_list(QM_FILES COPY_SOURCES COPY_DESTINATIONS)
    itom_post_build_copy_files(${target_name} COPY_SOURCES COPY_DESTINATIONS)

else(OPENGL_FOUND)
    message(WARNING "OpenGL could not be found. ${target_name} will not be build")
endif(OPENGL_FOUND)