cmake_minimum_required(VERSION 3.12...3.24)

set(target_name qwt)

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

set(CMAKE_MODULE_PATH
    ${CMAKE_MODULE_PATH}
    ${PROJECT_SOURCE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${ITOM_SDK_DIR}/cmake
    )
# As moc files are generated in the binary dir, tell CMake
# to always look for includes there:
set(CMAKE_INCLUDE_CURRENT_DIR ON)


find_package(ITOM_SDK COMPONENTS itomCommonLib REQUIRED)
include(ItomBuildMacros)

itom_init_cmake_policy(3.12)
itom_init_designerplugin_library(${target_name})

itom_find_package_qt(ON Core Widgets PrintSupport Xml Designer Widgets Svg OpenGL Concurrent)

add_definitions(-DQWT_DLL)
add_definitions(-DQWT_MAKEDLL)
add_definitions(-DQWT_MOC_INCLUDE)

include_directories(
    ${CMAKE_CURRENT_BINARY_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${ITOM_SDK_INCLUDE_DIRS}
    ${Qt5OpenGL_INCLUDE_DIRS}
)

link_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}/..
)

set(qwt_HEADERS
    ${CMAKE_CURRENT_SOURCE_DIR}/src/qwt_plot_opengl_canvas.h
    ${CMAKE_CURRENT_SOURCE_DIR}/src/qwt_plot_multi_barchart.h
    ${CMAKE_CURRENT_SOURCE_DIR}/src/qwt_plot_marker.h
    ${CMAKE_CURRENT_SOURCE_DIR}/src/qwt_plot_picker.h
    ${CMAKE_CURRENT_SOURCE_DIR}/src/qwt_plot_magnifier.h
    ${CMAKE_CURRENT_SOURCE_DIR}/src/qwt_plot_rasteritem.h
    ${CMAKE_CURRENT_SOURCE_DIR}/src/qwt_plot_legenditem.h
    ${CMAKE_CURRENT_SOURCE_DIR}/src/qwt_plot_renderer.h
    ${CMAKE_CURRENT_SOURCE_DIR}/src/qwt_plot_layout.h
    ${CMAKE_CURRENT_SOURCE_DIR}/src/qwt_plot_rescaler.h
    ${CMAKE_CURRENT_SOURCE_DIR}/src/qwt_plot_item.h
    ${CMAKE_CURRENT_SOURCE_DIR}/src/qwt_plot_scaleitem.h
    ${CMAKE_CURRENT_SOURCE_DIR}/src/qwt_plot_intervalcurve.h
    ${CMAKE_CURRENT_SOURCE_DIR}/src/qwt_plot_seriesitem.h
    ${CMAKE_CURRENT_SOURCE_DIR}/src/qwt_plot_histogram.h
    ${CMAKE_CURRENT_SOURCE_DIR}/src/qwt_plot_shapeitem.h
    ${CMAKE_CURRENT_SOURCE_DIR}/src/qwt_plot_grid.h
    ${CMAKE_CURRENT_SOURCE_DIR}/src/qwt_plot_spectrocurve.h
    ${CMAKE_CURRENT_SOURCE_DIR}/src/qwt_plot_graphicitem.h
    ${CMAKE_CURRENT_SOURCE_DIR}/src/qwt_plot_spectrogram.h
    # ${CMAKE_CURRENT_SOURCE_DIR}/src/qwt_plot_glcanvas.h
    ${CMAKE_CURRENT_SOURCE_DIR}/src/qwt_plot_svgitem.h
    ${CMAKE_CURRENT_SOURCE_DIR}/src/qwt_plot_directpainter.h
    ${CMAKE_CURRENT_SOURCE_DIR}/src/qwt_plot_textlabel.h
    ${CMAKE_CURRENT_SOURCE_DIR}/src/qwt_plot_dict.h
    ${CMAKE_CURRENT_SOURCE_DIR}/src/qwt_plot_tradingcurve.h
    ${CMAKE_CURRENT_SOURCE_DIR}/src/qwt_plot_curve.h
    ${CMAKE_CURRENT_SOURCE_DIR}/src/qwt_plot_vectorfield.h
    ${CMAKE_CURRENT_SOURCE_DIR}/src/qwt_plot_canvas.h
    ${CMAKE_CURRENT_SOURCE_DIR}/src/qwt_plot_zoneitem.h
    ${CMAKE_CURRENT_SOURCE_DIR}/src/qwt_plot_barchart.h
    ${CMAKE_CURRENT_SOURCE_DIR}/src/qwt_plot_zoomer.h
    ${CMAKE_CURRENT_SOURCE_DIR}/src/qwt_plot_abstract_canvas.h
    ${CMAKE_CURRENT_SOURCE_DIR}/src/qwt_point_3d.h
    ${CMAKE_CURRENT_SOURCE_DIR}/src/qwt_plot_abstract_barchart.h
    ${CMAKE_CURRENT_SOURCE_DIR}/src/qwt_point_data.h
    ${CMAKE_CURRENT_SOURCE_DIR}/src/qwt_plot.h
    ${CMAKE_CURRENT_SOURCE_DIR}/src/qwt_point_mapper.h
    ${CMAKE_CURRENT_SOURCE_DIR}/src/qwt_pixel_matrix.h
    ${CMAKE_CURRENT_SOURCE_DIR}/src/qwt_point_polar.h
    ${CMAKE_CURRENT_SOURCE_DIR}/src/qwt_polar.h
    ${CMAKE_CURRENT_SOURCE_DIR}/src/qwt_picker_machine.h
    ${CMAKE_CURRENT_SOURCE_DIR}/src/qwt_polar_canvas.h
    ${CMAKE_CURRENT_SOURCE_DIR}/src/qwt_picker.h
    ${CMAKE_CURRENT_SOURCE_DIR}/src/qwt_polar_curve.h
    ${CMAKE_CURRENT_SOURCE_DIR}/src/qwt_panner.h
    ${CMAKE_CURRENT_SOURCE_DIR}/src/qwt_polar_fitter.h
    ${CMAKE_CURRENT_SOURCE_DIR}/src/qwt_painter_command.h
    ${CMAKE_CURRENT_SOURCE_DIR}/src/qwt_polar_grid.h
    ${CMAKE_CURRENT_SOURCE_DIR}/src/qwt_painter.h
    ${CMAKE_CURRENT_SOURCE_DIR}/src/qwt_polar_item.h
    ${CMAKE_CURRENT_SOURCE_DIR}/src/qwt_null_paintdevice.h
    ${CMAKE_CURRENT_SOURCE_DIR}/src/qwt_polar_itemdict.h
    ${CMAKE_CURRENT_SOURCE_DIR}/src/qwt_matrix_raster_data.h
    ${CMAKE_CURRENT_SOURCE_DIR}/src/qwt_polar_layout.h
    ${CMAKE_CURRENT_SOURCE_DIR}/src/qwt_math.h
    ${CMAKE_CURRENT_SOURCE_DIR}/src/qwt_polar_magnifier.h
    ${CMAKE_CURRENT_SOURCE_DIR}/src/qwt_magnifier.h
    ${CMAKE_CURRENT_SOURCE_DIR}/src/qwt_polar_marker.h
    ${CMAKE_CURRENT_SOURCE_DIR}/src/qwt_legend_label.h
    ${CMAKE_CURRENT_SOURCE_DIR}/src/qwt_polar_panner.h
    ${CMAKE_CURRENT_SOURCE_DIR}/src/qwt_legend_data.h
    ${CMAKE_CURRENT_SOURCE_DIR}/src/qwt_polar_picker.h
    ${CMAKE_CURRENT_SOURCE_DIR}/src/qwt_legend.h
    ${CMAKE_CURRENT_SOURCE_DIR}/src/qwt_polar_plot.h
    ${CMAKE_CURRENT_SOURCE_DIR}/src/qwt_knob.h
    ${CMAKE_CURRENT_SOURCE_DIR}/src/qwt_polar_renderer.h
    ${CMAKE_CURRENT_SOURCE_DIR}/src/qwt_interval_symbol.h
    ${CMAKE_CURRENT_SOURCE_DIR}/src/qwt_polar_spectrogram.h
    ${CMAKE_CURRENT_SOURCE_DIR}/src/qwt_interval.h
    ${CMAKE_CURRENT_SOURCE_DIR}/src/qwt_raster_data.h
    ${CMAKE_CURRENT_SOURCE_DIR}/src/qwt_graphic.h
    ${CMAKE_CURRENT_SOURCE_DIR}/src/qwt_round_scale_draw.h
    ${CMAKE_CURRENT_SOURCE_DIR}/src/qwt_samples.h
    ${CMAKE_CURRENT_SOURCE_DIR}/src/qwt_global.h
    ${CMAKE_CURRENT_SOURCE_DIR}/src/qwt_sampling_thread.h
    ${CMAKE_CURRENT_SOURCE_DIR}/src/qwt_event_pattern.h
    ${CMAKE_CURRENT_SOURCE_DIR}/src/qwt_scale_div.h
    ${CMAKE_CURRENT_SOURCE_DIR}/src/qwt_dyngrid_layout.h
    ${CMAKE_CURRENT_SOURCE_DIR}/src/qwt_scale_draw.h
    ${CMAKE_CURRENT_SOURCE_DIR}/src/qwt_dial_needle.h
    ${CMAKE_CURRENT_SOURCE_DIR}/src/qwt_scale_engine.h
    ${CMAKE_CURRENT_SOURCE_DIR}/src/qwt_dial.h
    ${CMAKE_CURRENT_SOURCE_DIR}/src/qwt_scale_map.h
    ${CMAKE_CURRENT_SOURCE_DIR}/src/qwt_date_scale_engine.h
    ${CMAKE_CURRENT_SOURCE_DIR}/src/qwt_scale_widget.h
    ${CMAKE_CURRENT_SOURCE_DIR}/src/qwt_date_scale_draw.h
    ${CMAKE_CURRENT_SOURCE_DIR}/src/qwt_series_data.h
    ${CMAKE_CURRENT_SOURCE_DIR}/src/qwt_series_store.h
    ${CMAKE_CURRENT_SOURCE_DIR}/src/qwt_date.h
    ${CMAKE_CURRENT_SOURCE_DIR}/src/qwt_slider.h
    ${CMAKE_CURRENT_SOURCE_DIR}/src/qwt_curve_fitter.h
    ${CMAKE_CURRENT_SOURCE_DIR}/src/qwt_spline.h
    ${CMAKE_CURRENT_SOURCE_DIR}/src/qwt_counter.h
    ${CMAKE_CURRENT_SOURCE_DIR}/src/qwt_spline_basis.h
    ${CMAKE_CURRENT_SOURCE_DIR}/src/qwt_compass_rose.h
    ${CMAKE_CURRENT_SOURCE_DIR}/src/qwt_spline_cubic.h
    ${CMAKE_CURRENT_SOURCE_DIR}/src/qwt_compass.h
    ${CMAKE_CURRENT_SOURCE_DIR}/src/qwt_spline_curve_fitter.h
    ${CMAKE_CURRENT_SOURCE_DIR}/src/qwt_column_symbol.h
    ${CMAKE_CURRENT_SOURCE_DIR}/src/qwt_spline_local.h
    ${CMAKE_CURRENT_SOURCE_DIR}/src/qwt_color_map.h
    ${CMAKE_CURRENT_SOURCE_DIR}/src/qwt_spline_parametrization.h
    ${CMAKE_CURRENT_SOURCE_DIR}/src/qwt_clipper.h
    ${CMAKE_CURRENT_SOURCE_DIR}/src/qwt_spline_pleasing.h
    ${CMAKE_CURRENT_SOURCE_DIR}/src/qwt_bezier.h
    ${CMAKE_CURRENT_SOURCE_DIR}/src/qwt_spline_polynomial.h
    ${CMAKE_CURRENT_SOURCE_DIR}/src/qwt_axis_id.h
    ${CMAKE_CURRENT_SOURCE_DIR}/src/qwt_symbol.h
    ${CMAKE_CURRENT_SOURCE_DIR}/src/qwt_axis.h
    ${CMAKE_CURRENT_SOURCE_DIR}/src/qwt_system_clock.h
    ${CMAKE_CURRENT_SOURCE_DIR}/src/qwt_arrow_button.h
    ${CMAKE_CURRENT_SOURCE_DIR}/src/qwt_text.h
    ${CMAKE_CURRENT_SOURCE_DIR}/src/qwt_analog_clock.h
    ${CMAKE_CURRENT_SOURCE_DIR}/src/qwt_text_engine.h
    ${CMAKE_CURRENT_SOURCE_DIR}/src/qwt_abstract_slider.h
    ${CMAKE_CURRENT_SOURCE_DIR}/src/qwt_text_label.h
    ${CMAKE_CURRENT_SOURCE_DIR}/src/qwt_abstract_scale_draw.h
    ${CMAKE_CURRENT_SOURCE_DIR}/src/qwt_thermo.h
    ${CMAKE_CURRENT_SOURCE_DIR}/src/qwt_abstract_scale.h
    ${CMAKE_CURRENT_SOURCE_DIR}/src/qwt_transform.h
    ${CMAKE_CURRENT_SOURCE_DIR}/src/qwt_abstract_legend.h
    ${CMAKE_CURRENT_SOURCE_DIR}/src/qwt_vectorfield_symbol.h
    ${CMAKE_CURRENT_SOURCE_DIR}/src/qwt.h
    ${CMAKE_CURRENT_SOURCE_DIR}/src/qwt_weeding_curve_fitter.h
    ${CMAKE_CURRENT_SOURCE_DIR}/src/qwt_widget_overlay.h
    ${CMAKE_CURRENT_SOURCE_DIR}/src/qwt_wheel.h
    ${CMAKE_CURRENT_SOURCE_DIR}/src/qwt_plot_panner.h
)

set(qwt_SOURCES 
    ${CMAKE_CURRENT_SOURCE_DIR}/src/qwt_wheel.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/qwt.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/qwt_weeding_curve_fitter.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/qwt_abstract_legend.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/qwt_vectorfield_symbol.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/qwt_abstract_scale.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/qwt_transform.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/qwt_abstract_scale_draw.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/qwt_thermo.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/qwt_abstract_slider.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/qwt_text_label.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/qwt_analog_clock.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/qwt_text_engine.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/qwt_arrow_button.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/qwt_text.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/qwt_system_clock.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/qwt_symbol.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/qwt_bezier.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/qwt_spline_polynomial.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/qwt_clipper.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/qwt_spline_pleasing.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/qwt_color_map.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/qwt_spline_parametrization.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/qwt_column_symbol.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/qwt_spline_local.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/qwt_compass.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/qwt_spline_curve_fitter.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/qwt_compass_rose.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/qwt_spline_cubic.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/qwt_counter.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/qwt_spline_basis.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/qwt_curve_fitter.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/qwt_spline.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/qwt_date.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/qwt_slider.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/qwt_date_scale_draw.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/qwt_series_data.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/qwt_date_scale_engine.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/qwt_scale_widget.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/qwt_dial.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/qwt_scale_map.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/qwt_dial_needle.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/qwt_scale_engine.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/qwt_dyngrid_layout.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/qwt_scale_draw.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/qwt_event_pattern.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/qwt_scale_div.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/qwt_sampling_thread.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/qwt_graphic.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/qwt_round_scale_draw.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/qwt_interval.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/qwt_raster_data.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/qwt_interval_symbol.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/qwt_polar_spectrogram.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/qwt_knob.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/qwt_polar_renderer.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/qwt_legend.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/qwt_polar_plot.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/qwt_legend_data.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/qwt_polar_picker.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/qwt_legend_label.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/qwt_polar_panner.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/qwt_magnifier.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/qwt_polar_marker.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/qwt_math.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/qwt_polar_magnifier.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/qwt_matrix_raster_data.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/qwt_polar_layout.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/qwt_null_paintdevice.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/qwt_polar_itemdict.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/qwt_painter.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/qwt_polar_item.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/qwt_painter_command.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/qwt_polar_grid.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/qwt_panner.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/qwt_polar_fitter.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/qwt_picker.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/qwt_polar_curve.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/qwt_picker_machine.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/qwt_polar_canvas.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/qwt_pixel_matrix.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/qwt_point_polar.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/qwt_plot.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/qwt_point_mapper.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/qwt_plot_abstract_barchart.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/qwt_point_data.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/qwt_plot_abstract_canvas.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/qwt_point_3d.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/qwt_plot_axis.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/qwt_plot_barchart.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/qwt_plot_zoomer.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/qwt_plot_canvas.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/qwt_plot_zoneitem.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/qwt_plot_curve.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/qwt_plot_vectorfield.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/qwt_plot_dict.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/qwt_plot_tradingcurve.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/qwt_plot_directpainter.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/qwt_plot_textlabel.cpp
    # ${CMAKE_CURRENT_SOURCE_DIR}/src/qwt_plot_glcanvas.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/qwt_plot_svgitem.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/qwt_plot_graphicitem.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/qwt_plot_spectrogram.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/qwt_plot_grid.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/qwt_plot_spectrocurve.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/qwt_plot_histogram.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/qwt_plot_shapeitem.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/qwt_plot_intervalcurve.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/qwt_plot_seriesitem.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/qwt_plot_item.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/qwt_plot_scaleitem.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/qwt_plot_layout.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/qwt_plot_rescaler.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/qwt_plot_legenditem.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/qwt_plot_renderer.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/qwt_plot_magnifier.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/qwt_plot_rasteritem.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/qwt_plot_marker.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/qwt_plot_picker.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/qwt_plot_multi_barchart.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/qwt_widget_overlay.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/qwt_plot_opengl_canvas.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/qwt_plot_panner.cpp
)

add_library(${target_name} SHARED ${qwt_SOURCES} ${qwt_HEADERS})

target_link_libraries(${target_name} ${QT5_LIBRARIES} ${VISUALLEAKDETECTOR_LIBRARIES})

# Qt: enable all automoc, autouic and autorcc. Autouic file will be disabled for all files that are processed manually by itom_qt_wrap_ui.
set_target_properties(${target_name} PROPERTIES AUTOMOC ON AUTORCC ON AUTOUIC ON)

#targets that link to this target have to add the following include directories (automatically done if
#this target is among the target_link_libraries).
set_target_properties(
    ${target_name}
    PROPERTIES
    INTERFACE_INCLUDE_DIRECTORIES "${CMAKE_CURRENT_SOURCE_DIR}/src"
)

# COPY SECTION
set(COPY_SOURCES "")
set(COPY_DESTINATIONS "")
itom_add_designerlibrary_to_copy_list(${target_name} COPY_SOURCES COPY_DESTINATIONS)
itom_post_build_copy_files(${target_name} COPY_SOURCES COPY_DESTINATIONS)
