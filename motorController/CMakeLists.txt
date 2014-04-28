SET (target_name motorController)

project(${target_name})

cmake_minimum_required(VERSION 2.8)

OPTION(BUILD_UNICODE "Build with unicode charset if set to ON, else multibyte charset." ON)
OPTION(BUILD_SHARED_LIBS "Build shared library." ON)
OPTION(BUILD_TARGET64 "Build for 64 bit target if set to ON or 32 bit if set to OFF." OFF)
OPTION(BUILD_MOTORCTRL_3DCONNEXION "If 3DConnexion is installed, support is build into the MotorCtrl-Widget." OFF)
OPTION(UPDATE_TRANSLATIONS "Update source translation translation/*.ts files (WARNING: make clean will delete the source .ts files! Danger!)")
SET (ITOM_SDK_DIR "" CACHE PATH "base path to itom_sdk")
SET (CMAKE_DEBUG_POSTFIX "d" CACHE STRING "Adds a postfix for debug-built libraries.")
SET (ITOM_LANGUAGES "de" CACHE STRING "semicolon separated list of languages that should be created (en must not be given since it is the default)")

SET (CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${PROJECT_SOURCE_DIR} ${ITOM_SDK_DIR})

IF(BUILD_SHARED_LIBS)
    SET(LIBRARY_TYPE SHARED)
ELSE(BUILD_SHARED_LIBS)
    SET(LIBRARY_TYPE STATIC)
ENDIF(BUILD_SHARED_LIBS)

set(CMAKE_INCLUDE_CURRENT_DIR ON)

find_package(ITOM_SDK COMPONENTS dataobject itomCommonLib itomCommonQtLib REQUIRED)
include("${ITOM_SDK_DIR}/ItomBuildMacros.cmake")
FIND_PACKAGE_QT(ON Core Widgets UiTools Svg Xml Designer LinguistTools)

find_package(VisualLeakDetector QUIET)

IF(BUILD_MOTORCTRL_3DCONNEXION)
    find_package(3DCONNEXION)
    message(STATUS "3DConnexion enabled by user")
ELSE(BUILD_MOTORCTRL_3DCONNEXION)
    set(3DCONNEXION_FOUND false)
    message(STATUS "3DConnexion disabled by user")
ENDIF(BUILD_MOTORCTRL_3DCONNEXION)

IF (BUILD_UNICODE)
    ADD_DEFINITIONS(-DUNICODE -D_UNICODE)
ENDIF (BUILD_UNICODE)
ADD_DEFINITIONS(-DCMAKE -DITOMSHAREDDESIGNER)

IF(VISUALLEAKDETECTOR_FOUND AND VISUALLEAKDETECTOR_ENABLED)
    ADD_DEFINITIONS(-DVISUAL_LEAK_DETECTOR_CMAKE)
ENDIF(VISUALLEAKDETECTOR_FOUND AND VISUALLEAKDETECTOR_ENABLED)

# default build types are None, Debug, Release, RelWithDebInfo and MinRelSize
IF (DEFINED CMAKE_BUILD_TYPE)
    SET(CMAKE_BUILD_TYPE ${CMAKE_BUILD_TYPE} CACHE STRING "Choose the type of build, options are: None(CMAKE_CXX_FLAGS or CMAKE_C_FLAGS used) Debug Release RelWithDebInfo MinSizeRel.")
ELSE(CMAKE_BUILD_TYPE)
    SET (CMAKE_BUILD_TYPE Debug CACHE STRING "Choose the type of build, options are: None(CMAKE_CXX_FLAGS or CMAKE_C_FLAGS used) Debug Release RelWithDebInfo MinSizeRel.")
ENDIF (DEFINED CMAKE_BUILD_TYPE)

INCLUDE_DIRECTORIES(
    ${CMAKE_CURRENT_BINARY_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${QT_QTCORE_INCLUDE_DIR}
    ${ITOM_SDK_INCLUDE_DIRS}
    ${CMAKE_CURRENT_SOURCE_DIR}/icons
    ${3DCONNEXION_INCLUDE_PATH}
    ${VISUALLEAKDETECTOR_INCLUDE_DIR}
)

LINK_DIRECTORIES(
    ${CMAKE_CURRENT_SOURCE_DIR}/..
    ${3DCONNEXION_LIBRARIES}
)

set(plugin_HEADERS
    ${ITOM_SDK_INCLUDE_DIR}/common/sharedStructures.h
    ${CMAKE_CURRENT_SOURCE_DIR}/motorController.h
    ${CMAKE_CURRENT_SOURCE_DIR}/motorControllerfactory.h
    ${CMAKE_CURRENT_SOURCE_DIR}/pluginVersion.h
)

set(plugin_UI
#    ${CMAKE_CURRENT_SOURCE_DIR}/dialog2DScale.ui
)

set(plugin_RCC
    ${CMAKE_CURRENT_SOURCE_DIR}/../itomDesignerPlugins.qrc
)

set(plugin_SOURCES 

    ${CMAKE_CURRENT_SOURCE_DIR}/motorController.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/motorControllerfactory.cpp
)

IF(3DCONNEXION_FOUND)
    ADD_DEFINITIONS(-DCONNEXION_FOUND)
    #list(APPEND  ${LINK_DIRECTORIES} ${3DCONNEXION_LIBRARIES})
    #list(APPEND  ${INCLUDE_DIRECTORIES} ${3DCONNEXION_INCLUDE_PATH})
    #list(APPEND designer_HEADERS ${3DCONNEXION_LIBRARIES}/designerPluginLibraryVersion.rc)
ENDIF(3DCONNEXION_FOUND)

#If you want to use automatical metadata for dlls under windows use the following if-case.
if(MSVC)
    list(APPEND plugin_SOURCES ${ITOM_SDK_INCLUDE_DIR}/../designerPluginLibraryVersion.rc)
endif(MSVC)

if (QT5_FOUND)
    #if automoc if OFF, you also need to call QT5_WRAP_CPP here
    QT5_WRAP_UI(plugin_ui_MOC ${plugin_UI})
    QT5_ADD_RESOURCES(plugin_rcc_MOC ${plugin_RCC})
else (QT5_FOUND)
    QT4_WRAP_CPP_ITOM(plugin_HEADERS_MOC ${plugin_HEADERS})
    QT4_WRAP_UI_ITOM(plugin_ui_MOC ${plugin_UI})
    QT4_ADD_RESOURCES(plugin_rcc_MOC ${plugin_RCC})
endif (QT5_FOUND)

file (GLOB EXISTING_TRANSLATION_FILES "translation/*.ts")

ADD_LIBRARY(${target_name} ${LIBRARY_TYPE} ${plugin_SOURCES} ${plugin_HEADERS} ${plugin_HEADERS_MOC} ${plugin_ui_MOC} ${plugin_rcc_MOC} ${EXISTING_TRANSLATION_FILES})

IF(3DCONNEXION_FOUND)
    Message(STATUS "3DCONNEXION was found ${3DCONNEXION_SIAPP}.")
#        TARGET_LINK_LIBRARIES(${target_name} ${QT_LIBRARIES} ${OpenCV_LIBS} ${ITOM_SDK_dataobject_LIBRARY} ${3DCONNEXION_SIAPP} ${3DCONNEXION_SPWMATH} ${VISUALLEAKDETECTOR_LIBRARIES} ${ITOM_SDK_itomCommonLib_LIBRARY} ${ITOM_SDK_itomCommonQtLib_LIBRARY})
#        TARGET_LINK_LIBRARIES(${target_name} ${QT_LIBRARIES} ${3DCONNEXION_SIAPP} ${3DCONNEXION_SPWMATH} ${VISUALLEAKDETECTOR_LIBRARIES} ${ITOM_SDK_itomCommonLib_LIBRARY} ${ITOM_SDK_itomCommonQtLib_LIBRARY})
    TARGET_LINK_LIBRARIES(${target_name} ${QT_LIBRARIES} ${3DCONNEXION_SIAPP} ${3DCONNEXION_SPWMATH} ${ITOM_SDK_LIBRARIES} ${VISUALLEAKDETECTOR_LIBRARIES})
    if (QT5_FOUND)
        qt5_use_modules(${target_name} ${QT_COMPONENTS})
    endif (QT5_FOUND)
ELSE(3DCONNEXION_FOUND)
    Message(STATUS "WARNING: 3DCONNEXION was not found.")
#        TARGET_LINK_LIBRARIES(${target_name} ${OpenCV_LIBS} ${ITOM_SDK_dataobject_LIBRARY} ${VISUALLEAKDETECTOR_LIBRARIES} ${ITOM_SDK_itomCommonLib_LIBRARY} ${ITOM_SDK_itomCommonQtLib_LIBRARY})
#        TARGET_LINK_LIBRARIES(${target_name} ${QT_LIBRARIES} ${QT_QTUITOOLS_LIBRARY} ${QT_QTXML_LIBRARY} ${QT_QTDESIGNER_LIBRARY} ${QT_QTSVG_LIBRARY} ${OpenCV_LIBS} ${ITOM_SDK_dataobject_LIBRARY} ${VISUALLEAKDETECTOR_LIBRARIES} ${ITOM_SDK_itomCommonLib_LIBRARY} ${ITOM_SDK_itomCommonQtLib_LIBRARY})
    TARGET_LINK_LIBRARIES(${target_name} ${QT_LIBRARIES} ${ITOM_SDK_LIBRARIES} ${VISUALLEAKDETECTOR_LIBRARIES})
    if (QT5_FOUND)
        qt5_use_modules(${target_name} ${QT_COMPONENTS})
    endif (QT5_FOUND)
ENDIF(3DCONNEXION_FOUND)

#translation
set (FILES_TO_TRANSLATE ${plugin_SOURCES} ${plugin_HEADERS} ${plugin_UI})
PLUGIN_TRANSLATION(QM_FILES ${target_name} ${UPDATE_TRANSLATIONS} "${EXISTING_TRANSLATION_FILES}" ITOM_LANGUAGES "${FILES_TO_TRANSLATE}")

# COPY SECTION
set(COPY_SOURCES "")
set(COPY_DESTINATIONS "")
ADD_DESIGNERLIBRARY_TO_COPY_LIST(${target_name} COPY_SOURCES COPY_DESTINATIONS)
ADD_DESIGNER_QM_FILES_TO_COPY_LIST(QM_FILES COPY_SOURCES COPY_DESTINATIONS)
POST_BUILD_COPY_FILES(${target_name} COPY_SOURCES COPY_DESTINATIONS)
