==============================
twip OpenGL Figure for itom
==============================

=============== ========================================================================================================
**Summary**:    :plotsummary:`twipOGLFigure`
**Type**:       :plottype:`twipOGLFigure`
**Input**:       :plotinputtype:`twipOGLFigure`
**Formats**:       :plotdataformats:`twipOGLFigure`
**Features**:       :plotfeatures:`twipOGLFigure`
**License**:    :plotlicense:`twipOGLFigure`
**Platforms**:  Windows, Linux
**Author**:     :plotauthor:`twipOGLFigure`
=============== ========================================================================================================

.. plotsummeryextended::
    :plugin: twipOGLFigure


Overview
--------------

"twipOGLFigure" is a plot for pseudo 3D visualization of images like dataObjects. In case itom was compiles using the PointCloudLibrary, this widgets allows the plotting of pointClounds. polygonMeshes are not supported yet. The plot is based on openGL and renders plot objects
either using triangles or using points. For pointClouds only points are supported.

Basicly 3 different display modes are supported. In normal display mode, each point or triangle is positioned accoring to its x,y,z-Coordinate and the color of the element is given by the current color palette. In illumination mode, the color of each element is a combination of the z-position dependent false color plus the intensity modified by the directed illumination. In overlayMode the false color is a selectable combination between the applied overlay and the z-dependent false color.

If dataObjects are plotted, all DataTypes except "rgba32" are accepted. To plot complex objects, it is possible to select between the following modes: "absolut", "phase", "real" and "imaginary". An additional overlay (usually an intensity image) can be added. The overlay can be of everytype except complexTypes.

If pointClouds are plotted, the plot features differ slightly. E.g. the overlay can not be added seperatly, but the intensity from pointClouds of types including RGB- or I-values are interpreted as the overlay directly.

Plotting pointClouds of types with normals defined, allow the visualisation of the curvature as a false color map for the points. Some itomfilter code imformation within the curvature. The plot type also allows illumination and intensity overlay.

The "TwipOGLFigure" does neither support graphic element / marker plotting nor line or pixel picking at the moment.

Examples
----------------

Properties
------------------------------------------

.. plotproperties::
    :plugin: twipOGLFigure

Slots
------------------------------------------

The following slots are public and either callable by python or c++. See the c++ Reference for details.

.. plotslots::
    :plugin: twipOGLFigure

Signals
------------------------------------------

The following signals are public and either catchable by python or c++. See the c++ Reference for details.

.. plotsignals::
    :plugin: twipOGLFigure

C++ Reference
------------------------------------------

.. doxygenclass:: TwipOGLFigure
    :project: designerPlugin
    :members: public*,protected*,signal
