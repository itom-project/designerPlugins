
macro(itom_plugin_option PLUGIN_ID)

    set(PLUGINS_LIST    # Legend: X = OFF, D = Default, S = Setup, T = Test
    "+---------------------------------+-----------------------------------+"
    "| **Plugin**                      |  Win  | macOS | Ubu2404 | Rasbian |"
    "+=================================+===================================+"
    "| PLUGIN_evaluateGeometrics       |   D   |   D   |    D    |    D    |"
    "+---------------------------------+-----------------------------------+"
    "| PLUGIN_itom1DQwtPlot            |   D   |   D   |    D    |    D    |"
    "+---------------------------------+-----------------------------------+"
    "| PLUGIN_itom2DQwtPlot            |   D   |   D   |    D    |    D    |"
    "+---------------------------------+-----------------------------------+"
    "| PLUGIN_itomIsoGLFigurePlugin    |   X   |   D   |    D    |    D    |"
    "+---------------------------------+-----------------------------------+"
    "| PLUGIN_matplotlibPlot           |   D   |   D   |    D    |    D    |"
    "+---------------------------------+-----------------------------------+"
    "| PLUGIN_motorController          |   D   |   D   |    D    |    D    |"
    "+---------------------------------+-----------------------------------+"
    "| PLUGIN_plotlyPlot               |   D   |   D   |    D    |    D    |"
    "+---------------------------------+-----------------------------------+"
    "| PLUGIN_slider2D                 |   D   |   D   |    D    |    D    |"
    "+---------------------------------+-----------------------------------+"
    "| PLUGIN_twipOGLPlugin            |   D   |   D   |    D    |    D    |"
    "+---------------------------------+-----------------------------------+"
    "| PLUGIN_vtk3dVisualizer          |   D   |   D   |    D    |    D    |"
    )

    set(PATTERN "${PLUGIN_ID}.*$")

    # get column index
    if(WIN32)
        set(INDEX 1)
    elseif(APPLE)
        set(INDEX 3)
    elseif(UNIX)
        set(INDEX 4)
    endif(WIN32)

    foreach(PLUGIN_ROW ${PLUGINS_LIST})
        # get row
        string(REGEX MATCH ${PATTERN} MATCHSTRING ${PLUGIN_ROW})
        if(MATCHSTRING)

            string(REPLACE "|" ";" SPLIT_LIST "${MATCHSTRING}")
            list(GET SPLIT_LIST ${INDEX} ELEMENT)
            string(STRIP "${ELEMENT}" VALUE)

            # case DEFAULT
            if(PLUGIN_BUILD_OPTION STREQUAL "default" AND VALUE STREQUAL "D")
                option(${PLUGIN_ID} "Build with this plugin." ON)
            # case SETUP
            elseif(PLUGIN_BUILD_OPTION STREQUAL "setup" AND (VALUE STREQUAL "D" OR VALUE STREQUAL "S"))
                option(${PLUGIN_ID} "Build with this plugin." ON)
                SET(${PLUGIN_ID} ON CACHE BOOL "Build with this plugin." FORCE)
            # case TEST
            elseif(PLUGIN_BUILD_OPTION STREQUAL "test" AND (VALUE STREQUAL "D" OR VALUE STREQUAL "S" OR VALUE STREQUAL "T"))
                option(${PLUGIN_ID} "Build with this plugin." ON)
                SET(${PLUGIN_ID} ON CACHE BOOL "Build with this plugin." FORCE)
            else()
                option(${PLUGIN_ID} "Build with this plugin." OFF)
            endif()
        endif(MATCHSTRING)
    endforeach()
endmacro()
