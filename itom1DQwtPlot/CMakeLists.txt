set(target_name itom1DQwtPlot)
project(${target_name})

option(BUILD_TARGET64 "Build for 64 bit target if set to ON or 32 bit if set to OFF." ON)
option(UPDATE_TRANSLATIONS "Update source translation translation/*.ts files (WARNING: make clean will delete the source .ts files! Danger!)")
set(ITOM_SDK_DIR "" CACHE PATH "base path to itom_sdk")
set(CMAKE_DEBUG_POSTFIX "d" CACHE STRING "Adds a postfix for debug-built libraries.")
set(ITOM_LANGUAGES "de" CACHE STRING "semicolon separated list of languages that should be created (en must not be given since it is the default)")

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${PROJECT_SOURCE_DIR} ${ITOM_SDK_DIR}/cmake)
set(CMAKE_INCLUDE_CURRENT_DIR ON)

include("ItomBuildMacros")
itom_init_cmake_policy()
itom_init_designerplugin_library(${target_name})

find_package(ITOM_SDK COMPONENTS dataobject itomCommonLib itomCommonQtLib itomCommonPlotLib itomWidgets itomShapeLib REQUIRED)
itom_find_package_qt(ON Core Widgets Xml Designer PrintSupport Svg Concurrent OpenGL LinguistTools)
find_package(VisualLeakDetector QUIET)

add_definitions(-DUNICODE -D_UNICODE)
add_definitions(-DITOMSHAREDDESIGNER)
add_definitions(-DQWT_DLL)

if(VISUALLEAKDETECTOR_FOUND AND VISUALLEAKDETECTOR_ENABLED)
    add_definitions(-DVISUAL_LEAK_DETECTOR_CMAKE)
endif(VISUALLEAKDETECTOR_FOUND AND VISUALLEAKDETECTOR_ENABLED)

include_directories(
    ${CMAKE_CURRENT_BINARY_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${ITOM_SDK_INCLUDE_DIRS}
    ${CMAKE_CURRENT_SOURCE_DIR}/icons
    ${CMAKE_CURRENT_SOURCE_DIR}/../qwt/src
    ${VISUALLEAKDETECTOR_INCLUDE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/../itomQwtPlotBase
    ${CMAKE_CURRENT_BINARY_DIR}/../itomQwtPlotBase
)

link_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}/..
)

set(DESIGNER_HEADERS
    ${CMAKE_CURRENT_SOURCE_DIR}/dataObjectSeriesData.h
    ${CMAKE_CURRENT_SOURCE_DIR}/dataObjectSeriesDataXY.h
    ${CMAKE_CURRENT_SOURCE_DIR}/dialog1DScale.h
    ${CMAKE_CURRENT_SOURCE_DIR}/widgetCurveProperties.h
    ${CMAKE_CURRENT_SOURCE_DIR}/itom1DQwtPlot.h
    ${CMAKE_CURRENT_SOURCE_DIR}/itom1DQwtPlotPlugin.h
    ${CMAKE_CURRENT_SOURCE_DIR}/plot1DWidget.h
    ${CMAKE_CURRENT_SOURCE_DIR}/qwtPlotCurveDataObject.h
    ${CMAKE_CURRENT_SOURCE_DIR}/qwtPlotCurveProperty.h
    ${CMAKE_CURRENT_SOURCE_DIR}/valuepicker1d.h
    ${CMAKE_CURRENT_SOURCE_DIR}/pluginVersion.h
    ${CMAKE_CURRENT_SOURCE_DIR}/itomPlotMarker.h
)

set(plugin_HEADERS_COPY
    ${CMAKE_CURRENT_SOURCE_DIR}/itom1DQwtPlot.h
    ${CMAKE_CURRENT_SOURCE_DIR}/itom1DQwtPlotEnums.h
)

set(DESIGNER_UI
    ${CMAKE_CURRENT_SOURCE_DIR}/dialog1DScale.ui
    ${CMAKE_CURRENT_SOURCE_DIR}/widgetCurveProperties.ui
)

set(DESIGNER_RCC
    ${CMAKE_CURRENT_SOURCE_DIR}/itom1DQwtPlotPlugin.qrc
    ${CMAKE_CURRENT_SOURCE_DIR}/../itomDesignerPlugins.qrc
)

set(DESIGNER_SOURCES 
    ${CMAKE_CURRENT_SOURCE_DIR}/dataObjectSeriesData.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/dataObjectSeriesDataXY.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/dialog1DScale.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/widgetCurveProperties.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/itom1DQwtPlot.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/itom1DQwtPlotPlugin.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/plot1DWidget.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/valuepicker1d.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/qwtPlotCurveDataObject.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/qwtPlotCurveProperty.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/itomPlotMarker.cpp
)

#If you want to use automatical metadata for dlls under windows use the following if-case.
if(MSVC)
    list(APPEND DESIGNER_SOURCES ${ITOM_SDK_INCLUDE_DIR}/../designerPluginLibraryVersion.rc)
endif()

add_library(${target_name} SHARED ${DESIGNER_SOURCES} ${DESIGNER_HEADERS} ${DESIGNER_UI} ${DESIGNER_RCC})

target_link_libraries(${target_name} ${ITOM_SDK_LIBRARIES} ${QT5_LIBRARIES} ${VISUALLEAKDETECTOR_LIBRARIES} itomQwtPlotBase)

# Qt: enable all automoc, autouic and autorcc. Autouic file will be disabled for all files that are processed manually by itom_qt_wrap_ui.
set_target_properties(${target_name} PROPERTIES AUTOMOC ON AUTORCC ON AUTOUIC ON) #AUTOUIC OFF since the resulting files should also be parsed by the translation tool

# translations
file(GLOB EXISTING_TRANSLATION_FILES "translation/*.ts")
set(FILES_TO_TRANSLATE ${DESIGNER_SOURCES} ${DESIGNER_HEADERS} ${designer_ui_MOC})
itom_library_translation(QM_FILES ${target_name} ${UPDATE_TRANSLATIONS} "${EXISTING_TRANSLATION_FILES}" ITOM_LANGUAGES "${FILES_TO_TRANSLATE}")

configure_file(${CMAKE_CURRENT_SOURCE_DIR}/docs/doxygen/doxygen.dox.in ${CMAKE_CURRENT_BINARY_DIR}/docs/doxygen/doxygen.dox )

# COPY SECTION
set(COPY_SOURCES "")
set(COPY_DESTINATIONS "")
itom_add_designerlibrary_to_copy_list(${target_name} COPY_SOURCES COPY_DESTINATIONS)
itom_add_designer_qm_files_to_copy_list(QM_FILES COPY_SOURCES COPY_DESTINATIONS)
itom_add_designerplugin_headers_to_copy_list(${target_name} plugin_HEADERS_COPY COPY_SOURCES COPY_DESTINATIONS)
itom_post_build_copy_files(${target_name} COPY_SOURCES COPY_DESTINATIONS)
