SET (target_name qwtsrc)

project(${target_name})

cmake_minimum_required(VERSION 2.8)

OPTION(BUILD_UNICODE "Build with unicode charset if set to ON, else multibyte charset." ON)
OPTION(BUILD_SHARED "Build shared library." ON)
OPTION(BUILD_TARGET64 "Build for 64 bit target if set to ON or 32 bit if set to OFF." OFF)
SET (ITOM_SDK_DIR "" CACHE PATH "base path to itom_sdk")
SET (CMAKE_DEBUG_POSTFIX "d" CACHE STRING "Adds a postfix for debug-built libraries.")
SET (CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${PROJECT_SOURCE_DIR} ${ITOM_SDK_DIR})
SET (LIBRARY_TYPE STATIC)

#if qwt is static, add -fPIC as compiler flag for linux
IF(UNIX)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fPIC")
ENDIF(UNIX)

find_package(Qt4 REQUIRED)
find_package(ITOM_SDK REQUIRED)

include("${ITOM_SDK_DIR}/ItomBuildMacros.cmake")




ADD_DEFINITIONS(${QT_DEFINITIONS})
IF (BUILD_UNICODE)
	ADD_DEFINITIONS(-DUNICODE -D_UNICODE)
ENDIF (BUILD_UNICODE)
ADD_DEFINITIONS(-DCMAKE)

# enable some qt stuff
#SET (QT_USE_QTOPENGL TRUE)
SET (QT_USE_QTXML TRUE)

# default build types are None, Debug, Release, RelWithDebInfo and MinRelSize
IF (DEFINED CMAKE_BUILD_TYPE)
	SET(CMAKE_BUILD_TYPE ${CMAKE_BUILD_TYPE} CACHE STRING "Choose the type of build, options are: None(CMAKE_CXX_FLAGS or CMAKE_C_FLAGS used) Debug Release RelWithDebInfo MinSizeRel.")
ELSE(CMAKE_BUILD_TYPE)
	SET (CMAKE_BUILD_TYPE Debug CACHE STRING "Choose the type of build, options are: None(CMAKE_CXX_FLAGS or CMAKE_C_FLAGS used) Debug Release RelWithDebInfo MinSizeRel.")
ENDIF (DEFINED CMAKE_BUILD_TYPE)

INCLUDE_DIRECTORIES(
    ${CMAKE_CURRENT_BINARY_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${QT_QTSVG_INCLUDE_DIR}
    ${ITOM_SDK_INCLUDE_DIR}
)

LINK_DIRECTORIES(
	${CMAKE_CURRENT_SOURCE_DIR}/..
)

INCLUDE(${QT_USE_FILE})

message(STATUS ${CMAKE_CURRENT_BINARY_DIR})

set(designer_HEADERS
    ${CMAKE_CURRENT_SOURCE_DIR}/src/qwt.h
    ${CMAKE_CURRENT_SOURCE_DIR}/src/qwt_abstract_scale.h
    ${CMAKE_CURRENT_SOURCE_DIR}/src/qwt_abstract_scale_draw.h
    ${CMAKE_CURRENT_SOURCE_DIR}/src/qwt_abstract_slider.h
    ${CMAKE_CURRENT_SOURCE_DIR}/src/qwt_analog_clock.h
    ${CMAKE_CURRENT_SOURCE_DIR}/src/qwt_arrow_button.h
    ${CMAKE_CURRENT_SOURCE_DIR}/src/qwt_clipper.h
    ${CMAKE_CURRENT_SOURCE_DIR}/src/qwt_color_map.h
    ${CMAKE_CURRENT_SOURCE_DIR}/src/qwt_column_symbol.h
    ${CMAKE_CURRENT_SOURCE_DIR}/src/qwt_compass.h
    ${CMAKE_CURRENT_SOURCE_DIR}/src/qwt_compass_rose.h
    ${CMAKE_CURRENT_SOURCE_DIR}/src/qwt_compat.h
    ${CMAKE_CURRENT_SOURCE_DIR}/src/qwt_counter.h
    ${CMAKE_CURRENT_SOURCE_DIR}/src/qwt_curve_fitter.h
    ${CMAKE_CURRENT_SOURCE_DIR}/src/qwt_dial.h
    ${CMAKE_CURRENT_SOURCE_DIR}/src/qwt_dial_needle.h
    ${CMAKE_CURRENT_SOURCE_DIR}/src/qwt_double_range.h
    ${CMAKE_CURRENT_SOURCE_DIR}/src/qwt_dyngrid_layout.h
    ${CMAKE_CURRENT_SOURCE_DIR}/src/qwt_event_pattern.h
    ${CMAKE_CURRENT_SOURCE_DIR}/src/qwt_global.h
    ${CMAKE_CURRENT_SOURCE_DIR}/src/qwt_interval.h
    ${CMAKE_CURRENT_SOURCE_DIR}/src/qwt_interval_symbol.h
    ${CMAKE_CURRENT_SOURCE_DIR}/src/qwt_knob.h
    ${CMAKE_CURRENT_SOURCE_DIR}/src/qwt_legend.h
    ${CMAKE_CURRENT_SOURCE_DIR}/src/qwt_legend_item.h
    ${CMAKE_CURRENT_SOURCE_DIR}/src/qwt_legend_itemmanager.h
    ${CMAKE_CURRENT_SOURCE_DIR}/src/qwt_magnifier.h
    ${CMAKE_CURRENT_SOURCE_DIR}/src/qwt_math.h
    ${CMAKE_CURRENT_SOURCE_DIR}/src/qwt_matrix_raster_data.h
    ${CMAKE_CURRENT_SOURCE_DIR}/src/qwt_null_paintdevice.h
    ${CMAKE_CURRENT_SOURCE_DIR}/src/qwt_painter.h
    ${CMAKE_CURRENT_SOURCE_DIR}/src/qwt_panner.h
    ${CMAKE_CURRENT_SOURCE_DIR}/src/qwt_picker.h
    ${CMAKE_CURRENT_SOURCE_DIR}/src/qwt_picker_machine.h
    ${CMAKE_CURRENT_SOURCE_DIR}/src/qwt_plot.h
    ${CMAKE_CURRENT_SOURCE_DIR}/src/qwt_plot_canvas.h
    ${CMAKE_CURRENT_SOURCE_DIR}/src/qwt_plot_curve.h
    ${CMAKE_CURRENT_SOURCE_DIR}/src/qwt_plot_dict.h
    ${CMAKE_CURRENT_SOURCE_DIR}/src/qwt_plot_directpainter.h
    ${CMAKE_CURRENT_SOURCE_DIR}/src/qwt_plot_grid.h
    ${CMAKE_CURRENT_SOURCE_DIR}/src/qwt_plot_histogram.h
    ${CMAKE_CURRENT_SOURCE_DIR}/src/qwt_plot_intervalcurve.h
    ${CMAKE_CURRENT_SOURCE_DIR}/src/qwt_plot_item.h
    ${CMAKE_CURRENT_SOURCE_DIR}/src/qwt_plot_layout.h
    ${CMAKE_CURRENT_SOURCE_DIR}/src/qwt_plot_magnifier.h
    ${CMAKE_CURRENT_SOURCE_DIR}/src/qwt_plot_marker.h
    ${CMAKE_CURRENT_SOURCE_DIR}/src/qwt_plot_panner.h
    ${CMAKE_CURRENT_SOURCE_DIR}/src/qwt_plot_picker.h
    ${CMAKE_CURRENT_SOURCE_DIR}/src/qwt_plot_rasteritem.h
    ${CMAKE_CURRENT_SOURCE_DIR}/src/qwt_plot_renderer.h
    ${CMAKE_CURRENT_SOURCE_DIR}/src/qwt_plot_rescaler.h
    ${CMAKE_CURRENT_SOURCE_DIR}/src/qwt_plot_scaleitem.h
    ${CMAKE_CURRENT_SOURCE_DIR}/src/qwt_plot_seriesitem.h
    ${CMAKE_CURRENT_SOURCE_DIR}/src/qwt_plot_spectrocurve.h
    ${CMAKE_CURRENT_SOURCE_DIR}/src/qwt_plot_spectrogram.h
    ${CMAKE_CURRENT_SOURCE_DIR}/src/qwt_plot_svgitem.h
    ${CMAKE_CURRENT_SOURCE_DIR}/src/qwt_plot_zoomer.h
    ${CMAKE_CURRENT_SOURCE_DIR}/src/qwt_point_3d.h
    ${CMAKE_CURRENT_SOURCE_DIR}/src/qwt_point_polar.h
    ${CMAKE_CURRENT_SOURCE_DIR}/src/qwt_raster_data.h
    ${CMAKE_CURRENT_SOURCE_DIR}/src/qwt_round_scale_draw.h
    ${CMAKE_CURRENT_SOURCE_DIR}/src/qwt_sampling_thread.h
    ${CMAKE_CURRENT_SOURCE_DIR}/src/qwt_scale_div.h
    ${CMAKE_CURRENT_SOURCE_DIR}/src/qwt_scale_draw.h
    ${CMAKE_CURRENT_SOURCE_DIR}/src/qwt_scale_engine.h
    ${CMAKE_CURRENT_SOURCE_DIR}/src/qwt_scale_map.h
    ${CMAKE_CURRENT_SOURCE_DIR}/src/qwt_scale_widget.h
    ${CMAKE_CURRENT_SOURCE_DIR}/src/qwt_series_data.h
    ${CMAKE_CURRENT_SOURCE_DIR}/src/qwt_slider.h
    ${CMAKE_CURRENT_SOURCE_DIR}/src/qwt_spline.h
    ${CMAKE_CURRENT_SOURCE_DIR}/src/qwt_symbol.h
    ${CMAKE_CURRENT_SOURCE_DIR}/src/qwt_system_clock.h
    ${CMAKE_CURRENT_SOURCE_DIR}/src/qwt_text.h
    ${CMAKE_CURRENT_SOURCE_DIR}/src/qwt_text_engine.h
    ${CMAKE_CURRENT_SOURCE_DIR}/src/qwt_text_label.h
    ${CMAKE_CURRENT_SOURCE_DIR}/src/qwt_thermo.h
    ${CMAKE_CURRENT_SOURCE_DIR}/src/qwt_wheel.h
)

QT4_WRAP_CPP_ITOM(designer_HEADERS_MOC ${designer_HEADERS})

#set(designer_ui
#    ${CMAKE_CURRENT_SOURCE_DIR}/dialog1DScale.ui
#)

QT4_WRAP_UI_ITOM(designer_ui_MOC ${designer_ui})

#set(designer_rcc
#    ${CMAKE_CURRENT_SOURCE_DIR}/qwt.qrc
#)

set(designer_SOURCES 
    ${CMAKE_CURRENT_SOURCE_DIR}/src/qwt_abstract_scale.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/qwt_abstract_scale_draw.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/qwt_abstract_slider.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/qwt_analog_clock.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/qwt_arrow_button.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/qwt_clipper.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/qwt_color_map.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/qwt_column_symbol.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/qwt_compass.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/qwt_compass_rose.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/qwt_counter.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/qwt_curve_fitter.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/qwt_dial.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/qwt_dial_needle.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/qwt_double_range.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/qwt_dyngrid_layout.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/qwt_event_pattern.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/qwt_interval.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/qwt_interval_symbol.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/qwt_knob.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/qwt_legend.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/qwt_legend_item.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/qwt_magnifier.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/qwt_math.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/qwt_matrix_raster_data.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/qwt_null_paintdevice.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/qwt_painter.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/qwt_panner.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/qwt_picker.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/qwt_picker_machine.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/qwt_plot.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/qwt_plot_axis.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/qwt_plot_canvas.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/qwt_plot_curve.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/qwt_plot_dict.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/qwt_plot_directpainter.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/qwt_plot_grid.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/qwt_plot_histogram.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/qwt_plot_intervalcurve.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/qwt_plot_item.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/qwt_plot_layout.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/qwt_plot_magnifier.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/qwt_plot_marker.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/qwt_plot_panner.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/qwt_plot_picker.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/qwt_plot_rasteritem.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/qwt_plot_renderer.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/qwt_plot_rescaler.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/qwt_plot_scaleitem.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/qwt_plot_seriesitem.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/qwt_plot_spectrocurve.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/qwt_plot_spectrogram.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/qwt_plot_svgitem.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/qwt_plot_xml.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/qwt_plot_zoomer.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/qwt_point_3d.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/qwt_point_polar.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/qwt_raster_data.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/qwt_round_scale_draw.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/qwt_sampling_thread.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/qwt_scale_div.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/qwt_scale_draw.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/qwt_scale_engine.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/qwt_scale_map.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/qwt_scale_widget.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/qwt_series_data.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/qwt_slider.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/qwt_spline.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/qwt_symbol.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/qwt_system_clock.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/qwt_text.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/qwt_text_engine.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/qwt_text_label.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/qwt_thermo.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/qwt_wheel.cpp
)

QT4_ADD_RESOURCES(designer_rcc_MOC ${designer_rcc})
ADD_LIBRARY(${target_name} ${LIBRARY_TYPE} ${designer_SOURCES} ${designer_HEADERS} ${designer_HEADERS_MOC} ${designer_ui_MOC} ${designer_rcc_MOC})

TARGET_LINK_LIBRARIES(${target_name} ${QT_LIBRARIES} ${QT_QTUITOOLS_LIBRARY} ${QT_QTXML_LIBRARY} ${QT_QTDESIGNER_LIBRARY} ${QT_QTOPENGL_LIBRARY} ${QT_QTSVG_LIBRARY})
