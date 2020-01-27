set(target_name itomQwtPlotBase)

project(${target_name})

message(STATUS "\n<--- BASE LIBRARY ${target_name} --->")
message(STATUS "-- Build directory: ${CMAKE_CURRENT_BINARY_DIR}")

option(BUILD_TARGET64 "Build for 64 bit target if set to ON or 32 bit if set to OFF." ON)
option(UPDATE_TRANSLATIONS "Update source translation translation/*.ts files (WARNING: make clean will delete the source .ts files! Danger!)")
set(ITOM_SDK_DIR "" CACHE PATH "base path to itom_sdk")
set(CMAKE_DEBUG_POSTFIX "d" CACHE STRING "Adds a postfix for debug-built libraries.")
set(ITOM_LANGUAGES "de" CACHE STRING "semicolon separated list of languages that should be created (en must not be given since it is the default)")

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${PROJECT_SOURCE_DIR} ${ITOM_SDK_DIR}/cmake)
set(CMAKE_INCLUDE_CURRENT_DIR ON)

include("ItomBuildMacros")
itom_init_cmake_policy()

find_package(ITOM_SDK COMPONENTS dataobject itomCommonLib itomCommonQtLib itomCommonPlotLib itomWidgets itomShapeLib REQUIRED)
itom_find_package_qt(ON Core Widgets Xml Designer PrintSupport Svg Concurrent OpenGL LinguistTools)
find_package(VisualLeakDetector QUIET)

add_definitions(-DUNICODE -D_UNICODE)
add_definitions(-DITOMQWTPLOTBASE_DLL)
add_definitions(-DQWT_DLL) #use the shared library version of qwt

if(VISUALLEAKDETECTOR_FOUND AND VISUALLEAKDETECTOR_ENABLED)
    add_definitions(-DVISUAL_LEAK_DETECTOR_CMAKE)
endif(VISUALLEAKDETECTOR_FOUND AND VISUALLEAKDETECTOR_ENABLED)


include_directories(
    ${CMAKE_CURRENT_BINARY_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/version
    ${ITOM_SDK_INCLUDE_DIRS}
    ${CMAKE_CURRENT_SOURCE_DIR}/../qwt/src
    ${VISUALLEAKDETECTOR_INCLUDE_DIR}
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
)

set(plugin_HEADERS_COPY
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
)

#If you want to use automatical metadata for dlls under windows use the following if-case.
if(MSVC)
    list(APPEND DESIGNER_SOURCES ${ITOM_SDK_INCLUDE_DIR}/../designerPluginLibraryVersion.rc)
endif()

add_library(${target_name} SHARED ${DESIGNER_SOURCES} ${DESIGNER_HEADERS} ${DESIGNER_UI} ${DESIGNER_RCC})

target_link_libraries(${target_name} ${ITOM_SDK_LIBRARIES} ${QT5_LIBRARIES} ${VISUALLEAKDETECTOR_LIBRARIES} qwt)

# Qt: enable all automoc, autouic and autorcc. Autouic file will be disabled for all files that are processed manually by itom_qt_wrap_ui.
set_target_properties(${target_name} PROPERTIES AUTOMOC ON AUTORCC ON AUTOUIC ON)

# manually parse the ui files to obtain the output files for the translation below
itom_qt_wrap_ui(DESIGNER_UIC ${target_name} ${DESIGNER_UI})

# translations
file(GLOB EXISTING_TRANSLATION_FILES "translation/*.ts")
set(FILES_TO_TRANSLATE ${DESIGNER_SOURCES} ${DESIGNER_HEADERS} ${DESIGNER_UIC})
itom_library_translation(QM_FILES ${target_name} ${UPDATE_TRANSLATIONS} "${EXISTING_TRANSLATION_FILES}" ITOM_LANGUAGES "${FILES_TO_TRANSLATE}")

# COPY SECTION
set(COPY_SOURCES "")
set(COPY_DESTINATIONS "")
itom_add_designerlibrary_to_copy_list(${target_name} COPY_SOURCES COPY_DESTINATIONS)
itom_add_designer_qm_files_to_copy_list(QM_FILES COPY_SOURCES COPY_DESTINATIONS)
itom_add_designerplugin_headers_to_copy_list(${target_name} plugin_HEADERS_COPY COPY_SOURCES COPY_DESTINATIONS)
itom_post_build_copy_files(${target_name} COPY_SOURCES COPY_DESTINATIONS)
