cmake_minimum_required(VERSION 3.12...3.24)

set(target_name itomQwtPlotBase)

message(STATUS "\n<--- BASE LIBRARY ${target_name} --->")
message(STATUS "-- Build directory: ${CMAKE_CURRENT_BINARY_DIR}")

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

find_package(ITOM_SDK COMPONENTS
    dataobject
    itomCommonLib
    itomCommonQtLib
    itomCommonPlotLib
    itomWidgets
    itomShapeLib
    REQUIRED)
itom_find_package_qt(ON
    Core
    Widgets
    Xml
    Designer
    PrintSupport
    Svg
    Concurrent
    OpenGL
    LinguistTools)

add_definitions(-DITOMQWTPLOTBASE_DLL)
add_definitions(-DQWT_DLL) #use the shared library version of qwt


include_directories(
    ${CMAKE_CURRENT_BINARY_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/version
    ${ITOM_SDK_INCLUDE_DIRS}
    #${CMAKE_CURRENT_SOURCE_DIR}/../qwt/src
)

link_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}/..
)

set(DESIGNER_HEADERS
    ${CMAKE_CURRENT_SOURCE_DIR}/version/pluginVersion.h
    ${CMAKE_CURRENT_SOURCE_DIR}/dialogExportProperties.h
    ${CMAKE_CURRENT_SOURCE_DIR}/multiPointPickerMachine.h
    ${CMAKE_CURRENT_SOURCE_DIR}/userInteractionPlotPicker.h
    ${CMAKE_CURRENT_SOURCE_DIR}/drawItem.h
    ${CMAKE_CURRENT_SOURCE_DIR}/itomPlotMagnifier.h
    ${CMAKE_CURRENT_SOURCE_DIR}/itomPlotZoomer.h
    ${CMAKE_CURRENT_SOURCE_DIR}/itomLogLogScaleEngine.h
    ${CMAKE_CURRENT_SOURCE_DIR}/itomQwtPlot.h
    ${CMAKE_CURRENT_SOURCE_DIR}/itomQwtDObjFigure.h
    ${CMAKE_CURRENT_SOURCE_DIR}/itomQwtPlotEnums.h
    ${CMAKE_CURRENT_SOURCE_DIR}/itomColorMap.h
    ${CMAKE_CURRENT_SOURCE_DIR}/itomPlotPicker.h
    ${CMAKE_CURRENT_SOURCE_DIR}/itomQwtPlotPanner.h
    ${CMAKE_CURRENT_SOURCE_DIR}/markerModel.h
    ${CMAKE_CURRENT_SOURCE_DIR}/markerWidget.h
)

set(DESIGNER_HEADERS_COPY
    ${CMAKE_CURRENT_SOURCE_DIR}/itomQwtPlotEnums.h
    ${CMAKE_CURRENT_SOURCE_DIR}/itomQwtDObjFigure.h
    ${CMAKE_CURRENT_SOURCE_DIR}/itomQwtPlotBase.h
)

set(DESIGNER_UI
    ${CMAKE_CURRENT_SOURCE_DIR}/dialogExportProperties.ui
)

set(DESIGNER_RCC
    #${CMAKE_CURRENT_SOURCE_DIR}/itom1DQwtPlotPlugin.qrc
    #${CMAKE_CURRENT_SOURCE_DIR}/../itomDesignerPlugins.qrc
)

set(DESIGNER_SOURCES
    ${CMAKE_CURRENT_SOURCE_DIR}/dialogExportProperties.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/multiPointPickerMachine.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/userInteractionPlotPicker.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/drawItem.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/itomPlotMagnifier.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/itomPlotZoomer.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/itomLogLogScaleEngine.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/itomQwtPlot.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/itomQwtDObjFigure.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/itomColorMap.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/itomPlotPicker.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/itomQwtPlotPanner.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/markerModel.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/markerWidget.cpp
)

#If you want to use automatical metadata for dlls under windows use the following if-case.
if(MSVC)
    list(APPEND DESIGNER_SOURCES ${ITOM_SDK_INCLUDE_DIR}/../designerPluginLibraryVersion.rc)
endif()

add_library(${target_name} SHARED
    ${DESIGNER_SOURCES}
    ${DESIGNER_HEADERS}
    ${DESIGNER_UI}
    ${DESIGNER_RCC})

target_link_libraries(${target_name}
    ${ITOM_SDK_LIBRARIES}
    ${QT5_LIBRARIES}
    ${VISUALLEAKDETECTOR_LIBRARIES}
    qwt)

# Qt: enable all automoc, autouic and autorcc. Autouic file will be disabled for all files that are processed manually by itom_qt_wrap_ui.
set_target_properties(${target_name} PROPERTIES
    AUTOMOC ON
    AUTORCC ON
    AUTOUIC ON)

itom_qt_wrap_ui(uic_files ${target_name} ${DESIGNER_UI})

set(additional_include_folders "${CMAKE_CURRENT_SOURCE_DIR}")

foreach(uic_file ${uic_files})
    get_filename_component(dir ${uic_file} DIRECTORY)
    set(additional_include_folders "${additional_include_folders}" "${dir}")
endforeach()

#targets that link to this target have to add the following include directories (automatically done if
#this target is among the target_link_libraries).
set_target_properties(
    ${target_name}
    PROPERTIES
    INTERFACE_INCLUDE_DIRECTORIES "${additional_include_folders}"
)

# translations
set(FILES_TO_TRANSLATE ${DESIGNER_SOURCES} ${DESIGNER_HEADERS} ${DESIGNER_UI})
itom_library_translation(QM_FILES TARGET ${target_name} FILES_TO_TRANSLATE ${FILES_TO_TRANSLATE})

# COPY SECTION
set(COPY_SOURCES "")
set(COPY_DESTINATIONS "")
itom_add_designerlibrary_to_copy_list(${target_name} COPY_SOURCES COPY_DESTINATIONS)
itom_add_designer_qm_files_to_copy_list(QM_FILES COPY_SOURCES COPY_DESTINATIONS)
itom_add_designerplugin_headers_to_copy_list(${target_name} DESIGNER_HEADERS_COPY COPY_SOURCES COPY_DESTINATIONS)
itom_post_build_copy_files(${target_name} COPY_SOURCES COPY_DESTINATIONS)
