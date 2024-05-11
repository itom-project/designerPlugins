/* ********************************************************************
   itom measurement system
   URL: http://www.uni-stuttgart.de/ito
   Copyright (C) 2018, Institut für Technische Optik (ITO),
   Universität Stuttgart, Germany

   This file is part of the designer widget 'vtk3dVisualizer' for itom.

   itom is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   itom is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with itom. If not, see <http://www.gnu.org/licenses/>.
*********************************************************************** */

#define ITOM_IMPORT_API
#define ITOM_IMPORT_PLOTAPI
#include "vtk3dVisualizer.h"

#include <QtCore/QtPlugin>
#include "vtk3dVisualizerFactory.h"
#include "pluginVersion.h"
#include "gitVersion.h"

//---------------------------------------------------------------------------------------------------------------
Vtk3DVisualizerFactory::Vtk3DVisualizerFactory(QObject *parent)
    : AbstractItomDesignerPlugin(parent)
{
    m_plotDataFormats = ito::Format_Float32 | ito::Format_RGB32 | ito::Format_ARGB32;
    m_plotDataTypes = ito::PointCloud | ito::PolygonMesh;
    m_plotFeatures = ito::Static | ito::Plot3D | ito::PlotISO | ito::OpenGl | ito::Cartesian;


    m_description = QObject::tr("3D visualization of point clouds, polygon meshes and geometries using PCL and VTK.");
    m_detaildescription = QObject::tr("You can display any point cloud (with and without normals), polygon meshes or geometries, group them in a tree structure, change their display properties and connect to click events.");

    m_author = PLUGIN_AUTHOR;
    m_version = PLUGIN_VERSION;
    m_minItomVer = PLUGIN_MIN_ITOM_VERSION;
    m_maxItomVer = PLUGIN_MAX_ITOM_VERSION;
    m_license = QObject::tr(PLUGIN_LICENCE);
    m_aboutThis = QObject::tr(GITVERSION);

    initialized = false;
}

//---------------------------------------------------------------------------------------------------------------
void Vtk3DVisualizerFactory::initialize(QDesignerFormEditorInterface * /*core*/)
{
    if (initialized)
        return;

    initialized = true;
}

//---------------------------------------------------------------------------------------------------------------
bool Vtk3DVisualizerFactory::isInitialized() const
{
    return initialized;
}

//---------------------------------------------------------------------------------------------------------------
QWidget *Vtk3DVisualizerFactory::createWidget(QWidget *parent)
{
    return new Vtk3dVisualizer(m_itomSettingsFile, ito::AbstractFigure::ModeStandaloneInUi, parent);
}
//---------------------------------------------------------------------------------------------------------------
QWidget *Vtk3DVisualizerFactory::createWidgetWithMode(ito::AbstractFigure::WindowMode winMode, QWidget * parent)
{
    return new Vtk3dVisualizer(m_itomSettingsFile, winMode, parent);
}

//---------------------------------------------------------------------------------------------------------------
QString Vtk3DVisualizerFactory::name() const
{
    return "Vtk3dVisualizer";
}

//---------------------------------------------------------------------------------------------------------------
QString Vtk3DVisualizerFactory::group() const
{
    return "itom Plugins";
}

//---------------------------------------------------------------------------------------------------------------
QIcon Vtk3DVisualizerFactory::icon() const
{
    return QIcon(":/itomDesignerPlugins/itom/icons/q_itoM32.png");
}

//---------------------------------------------------------------------------------------------------------------
QString Vtk3DVisualizerFactory::toolTip() const
{
    return QString();
}

//---------------------------------------------------------------------------------------------------------------
QString Vtk3DVisualizerFactory::whatsThis() const
{
    return QObject::tr("itom widget to show point clouds, polygon meshes, geometries...");
}

//---------------------------------------------------------------------------------------------------------------
bool Vtk3DVisualizerFactory::isContainer() const
{
    return false;
}

//---------------------------------------------------------------------------------------------------------------
QString Vtk3DVisualizerFactory::domXml() const
{
    return "<widget class=\"Vtk3dVisualizer\" name=\"vtk3dVisualizer\">\n"
           " <property name=\"geometry\">\n"
        "  <rect>\n"
        "   <x>0</x>\n"
        "   <y>0</y>\n"
        "   <width>650</width>\n"
        "   <height>400</height>\n"
        "  </rect>\n"
        " </property>\n"
        "</widget>\n";
}

//---------------------------------------------------------------------------------------------------------------
QString Vtk3DVisualizerFactory::includeFile() const
{
    return "win3dVisualizer.h";
}
