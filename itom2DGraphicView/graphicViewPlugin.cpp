/* ********************************************************************
   itom measurement system
   URL: http://www.uni-stuttgart.de/ito
   Copyright (C) 2018, Institut fuer Technische Optik (ITO), 
   Universitaet Stuttgart, Germany 
 
   This file is part of itom.

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
#include "graphicViewPlugin.h"
#include "pluginVersion.h"

#include <QtCore/QtPlugin>
#include "graphicViewPlot.h"

GraphicViewPlugin::GraphicViewPlugin(QObject *parent)
    : ito::AbstractItomDesignerPlugin(parent)
{
    m_plotDataFormats = ito::Format_Gray8 | ito::Format_Gray16 | ito::Format_Gray32 | ito::Format_Float32 | ito::Format_ARGB32 | ito::Format_RGB32 | ito::Format_Float64 | ito::Format_Complex;
    m_plotDataTypes = ito::DataObjPlane;
    m_plotFeatures = ito::Live | ito::Static | ito::PlotImage | ito::Cartesian;

    m_description = QObject::tr("ITOM widget for 2D DataObjects based on qGraphicView.");
    m_detaildescription = QObject::tr("");
    m_author = "Wolfram Lyda, twip optical solutions GmbH";
    m_version = (PLUGIN_VERSION_MAJOR << 16) + (PLUGIN_VERSION_MINOR << 8) + PLUGIN_VERSION_PATCH;
    m_license = QObject::tr("LGPL");

    initialized = false;
}

void GraphicViewPlugin::initialize(QDesignerFormEditorInterface * /*core*/)
{
    if (initialized)
        return;

    initialized = true;
}

bool GraphicViewPlugin::isInitialized() const
{
    return initialized;
}

QWidget *GraphicViewPlugin::createWidget(QWidget *parent)
{
    return new GraphicViewPlot(m_itomSettingsFile, ito::AbstractFigure::ModeStandaloneInUi, parent);
}

QWidget *GraphicViewPlugin::createWidgetWithMode(ito::AbstractFigure::WindowMode winMode, QWidget * parent)
{
    return new GraphicViewPlot(m_itomSettingsFile, winMode, parent);
}

QString GraphicViewPlugin::name() const
{
    return "GraphicViewPlot";
}

QString GraphicViewPlugin::group() const
{
    return "itom Plugins";
}

QIcon GraphicViewPlugin::icon() const
{
    return QIcon(":/itomDesignerPlugins/itom/icons/q_itoM32.png");
}

QString GraphicViewPlugin::toolTip() const
{
    return QString("Use this widget to plot 2D images into your UI dialog.");
}

QString GraphicViewPlugin::whatsThis() const
{
    return m_description;
}

bool GraphicViewPlugin::isContainer() const
{
    return false;
}

QString GraphicViewPlugin::domXml() const
{
    return "<widget class=\"GraphicViewPlot\" name=\"graphicViewPlot\">\n"
        " <property name=\"geometry\">\n"
        "  <rect>\n"
        "   <x>0</x>\n"
        "   <y>0</y>\n"
        "   <width>100</width>\n"
        "   <height>100</height>\n"
        "  </rect>\n"
        " </property>\n"
        "</widget>\n";
}

QString GraphicViewPlugin::includeFile() const
{
    return "graphicViewPlot.h";
}

#if (QT_VERSION < 0x050000)
    Q_EXPORT_PLUGIN2(graphicViewPlot, GraphicViewPlugin)
#endif
