cmake_minimum_required(VERSION 3.12...3.29)
set(target_name motorController)

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


set(CMAKE_INCLUDE_CURRENT_DIR ON)

find_package(ITOM_SDK COMPONENTS dataobject itomCommonLib itomCommonQtLib REQUIRED)
itom_find_package_qt(ON Core Widgets UiTools Svg Xml Designer LinguistTools)


if(BUILD_MOTORCTRL_3DCONNEXION)
    find_package(3DCONNEXION)
    message(STATUS "3DConnexion enabled by user")
else(BUILD_MOTORCTRL_3DCONNEXION)
    set(3DCONNEXION_FOUND false)
    message(STATUS "3DConnexion disabled by user")
endif(BUILD_MOTORCTRL_3DCONNEXION)

add_definitions(-DITOMSHAREDDESIGNER)

include_directories(
    ${CMAKE_CURRENT_BINARY_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${ITOM_SDK_INCLUDE_DIRS}
    ${CMAKE_CURRENT_SOURCE_DIR}/icons
    ${3DCONNEXION_INCLUDE_PATH}
)

link_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}/..
    ${3DCONNEXION_LIBRARIES}
)

set(DESIGNER_HEADERS
    ${ITOM_SDK_INCLUDE_DIR}/common/sharedStructures.h
    ${CMAKE_CURRENT_SOURCE_DIR}/motorController.h
    ${CMAKE_CURRENT_SOURCE_DIR}/motorControllerfactory.h
    ${CMAKE_CURRENT_BINARY_DIR}/pluginVersion.h
    ${CMAKE_CURRENT_BINARY_DIR}/gitVersion.h
)

#Define Version in pluginVersion.h
configure_file( ${CMAKE_CURRENT_SOURCE_DIR}/pluginVersion.h.in ${CMAKE_CURRENT_BINARY_DIR}/pluginVersion.h)

set(DESIGNER_UI
#    ${CMAKE_CURRENT_SOURCE_DIR}/dialog2DScale.ui
)

set(DESIGNER_RCC
    ${CMAKE_CURRENT_SOURCE_DIR}/../itomDesignerPlugins.qrc
)

set(DESIGNER_SOURCES

    ${CMAKE_CURRENT_SOURCE_DIR}/motorController.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/motorControllerfactory.cpp
)

if(3DCONNEXION_FOUND)
    add_definitions(-DCONNEXION_FOUND)
    #list(APPEND  ${link_directories} ${3DCONNEXION_LIBRARIES})
    #list(APPEND  ${include_directories} ${3DCONNEXION_INCLUDE_PATH})
    #list(APPEND DESIGNER_HEADERS ${3DCONNEXION_LIBRARIES}/designerPluginLibraryVersion.rc)
endif(3DCONNEXION_FOUND)

#If you want to use automatically metadata for dlls under windows use the following if-case.
if(MSVC)
    list(APPEND DESIGNER_SOURCES ${ITOM_SDK_INCLUDE_DIR}/../designerPluginLibraryVersion.rc)
endif()

add_library(${target_name} SHARED ${DESIGNER_SOURCES} ${DESIGNER_HEADERS} ${DESIGNER_UI} ${DESIGNER_RCC})

if(3DCONNEXION_FOUND)
    message(STATUS "3DCONNEXION was found ${3DCONNEXION_SIAPP}.")
    target_link_libraries(${target_name} ${3DCONNEXION_SIAPP} ${3DCONNEXION_SPWMATH} ${ITOM_SDK_LIBRARIES} ${QT5_LIBRARIES} ${VISUALLEAKDETECTOR_LIBRARIES})
else(3DCONNEXION_FOUND)
    message(STATUS "WARNING: 3DCONNEXION was not found.")
    target_link_libraries(${target_name} ${ITOM_SDK_LIBRARIES} ${QT5_LIBRARIES} ${VISUALLEAKDETECTOR_LIBRARIES})
endif(3DCONNEXION_FOUND)

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
itom_post_build_copy_files(${target_name} COPY_SOURCES COPY_DESTINATIONS)
