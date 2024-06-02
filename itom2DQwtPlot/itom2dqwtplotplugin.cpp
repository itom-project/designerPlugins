/* ********************************************************************
   itom measurement system
   URL: http://www.uni-stuttgart.de/ito
   Copyright (C) 2018, Institut für Technische Optik (ITO),
   Universität Stuttgart, Germany

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
#include "itom2dqwtplot.h"

#include <QtCore/QtPlugin>
#include "itom2dqwtplotplugin.h"
#include "pluginVersion.h"
#include "gitVersion.h"

Itom2dQwtPlotPlugin::Itom2dQwtPlotPlugin(QObject *parent)
    : ito::AbstractItomDesignerPlugin(parent)
{
    m_plotDataFormats = ito::Format_Gray8 | ito::Format_Gray16 | ito::Format_Gray32 | ito::Format_Float32 | ito::Format_Float64 | ito::Format_Complex | ito::Format_ARGB32 | ito::Format_RGB32;
    m_plotDataTypes = ito::DataObjPlane | ito::DataObjLine;
    m_plotFeatures = ito::Static | ito::Live | ito::PlotImage | ito::PlotLine | ito::Cartesian;

    m_description = QObject::tr("itom widget for 2D-visualisation of 2D/3D dataObjects based on Qwt.");
    m_detaildescription = QObject::tr("This designer plugin is an itom widget for image-like visualisation of dataObjects and live images. This widget is based on the Qwt framework (http://qwt.sf.net).");

    m_author = PLUGIN_AUTHOR;
    m_version = PLUGIN_VERSION;
    m_minItomVer = PLUGIN_MIN_ITOM_VERSION;
    m_maxItomVer = PLUGIN_MAX_ITOM_VERSION;
    m_license = QObject::tr(PLUGIN_LICENCE);
    m_aboutThis = QObject::tr(GITVERSION);

    initialized = false;
}

void Itom2dQwtPlotPlugin::initialize(QDesignerFormEditorInterface * /*core*/)
{
    if (initialized)
        return;

    initialized = true;
}

bool Itom2dQwtPlotPlugin::isInitialized() const
{
    return initialized;
}

QWidget *Itom2dQwtPlotPlugin::createWidget(QWidget *parent)
{
    return new Itom2dQwtPlot(m_itomSettingsFile, ito::AbstractFigure::ModeStandaloneInUi, parent);
}

QWidget *Itom2dQwtPlotPlugin::createWidgetWithMode(ito::AbstractFigure::WindowMode winMode, QWidget * parent)
{
    return new Itom2dQwtPlot(m_itomSettingsFile, winMode, parent);
}

QString Itom2dQwtPlotPlugin::name() const
{
    return "Itom2dQwtPlot";
}

QString Itom2dQwtPlotPlugin::group() const
{
    return "itom Plugins";
}

QIcon Itom2dQwtPlotPlugin::icon() const
{
    return QIcon(":/itomDesignerPlugins/itom/icons/q_itoM32.png");
}

QString Itom2dQwtPlotPlugin::toolTip() const
{
    return QString("Use this widget in your UI for 2D plots in this widget's canvas.");
}

QString Itom2dQwtPlotPlugin::whatsThis() const
{
    return QString("itom widget for 2D DataObjects.");
}

bool Itom2dQwtPlotPlugin::isContainer() const
{
    return false;
}

QString Itom2dQwtPlotPlugin::domXml() const
{
    return "<widget class=\"Itom2dQwtPlot\" name=\"itom2dQwtPlot\">\n"
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

QString Itom2dQwtPlotPlugin::includeFile() const
{
    return "itom2dqwtplot.h";
}
