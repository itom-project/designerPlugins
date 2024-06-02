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
#include "itom1DQwtPlotPlugin.h"
#include "pluginVersion.h"

#include <QtCore/QtPlugin>
#include "itom1DQwtPlot.h"
#include "gitVersion.h"


Itom1DQwtPlotPlugin::Itom1DQwtPlotPlugin(QObject *parent)
    : AbstractItomDesignerPlugin(parent)
{
    m_plotDataFormats = ito::Format_Gray8 | ito::Format_Gray16 | ito::Format_Gray32 | ito::Format_Float32 | ito::Format_Float64 | ito::Format_Complex | ito::Format_ARGB32 | ito::Format_RGB32;
    m_plotDataTypes = ito::DataObjLine | ito::DataObjPlane;
    m_plotFeatures = ito::Live | ito::Static |  ito::PlotImage | ito::PlotLine | ito::Cartesian;

    m_description = QObject::tr("itom widget for 1D dataObjects based on Qwt.");
    m_detaildescription = QObject::tr("This designer plugin is an itom widget for linewise / graph-based visualisation of dataObjects and live-outputs of line cameras. This widget is based on the Qwt framework (http://qwt.sf.net).");

    m_author = PLUGIN_AUTHOR;
    m_version = PLUGIN_VERSION;
    m_minItomVer = PLUGIN_MIN_ITOM_VERSION;
    m_maxItomVer = PLUGIN_MAX_ITOM_VERSION;
    m_license = QObject::tr(PLUGIN_LICENCE);
    m_aboutThis = QObject::tr(GITVERSION);

    initialized = false;
}

void Itom1DQwtPlotPlugin::initialize(QDesignerFormEditorInterface * /*core*/)
{
    if (initialized)
        return;

    initialized = true;
}

bool Itom1DQwtPlotPlugin::isInitialized() const
{
    return initialized;
}

QWidget *Itom1DQwtPlotPlugin::createWidget(QWidget *parent)
{
    return new Itom1DQwtPlot(m_itomSettingsFile, ito::AbstractFigure::ModeStandaloneInUi, parent);
}

QWidget *Itom1DQwtPlotPlugin::createWidgetWithMode(ito::AbstractFigure::WindowMode winMode, QWidget * parent)
{
    return new Itom1DQwtPlot(m_itomSettingsFile, winMode, parent);
}

QString Itom1DQwtPlotPlugin::name() const
{
    return "Itom1DQwtPlot";
}

QString Itom1DQwtPlotPlugin::group() const
{
    return "itom Plugins";
}

QIcon Itom1DQwtPlotPlugin::icon() const
{
    return QIcon(":/itomDesignerPlugins/itom/icons/q_itoM32.png");
}

QString Itom1DQwtPlotPlugin::toolTip() const
{
    return QString("Use this widget in your UI for 1D plots in this widget's canvas.");
}

QString Itom1DQwtPlotPlugin::whatsThis() const
{
    return m_description;
}

bool Itom1DQwtPlotPlugin::isContainer() const
{
    return false;
}

QString Itom1DQwtPlotPlugin::domXml() const
{
    return "<widget class=\"Itom1DQwtPlot\" name=\"itom1DQwtPlot\">\n"
        " <property name=\"geometry\">\n"
        "  <rect>\n"
        "   <x>0</x>\n"
        "   <y>0</y>\n"
        "   <width>250</width>\n"
        "   <height>150</height>\n"
        "  </rect>\n"
        " </property>\n"
        "</widget>\n";
}

QString Itom1DQwtPlotPlugin::includeFile() const
{
    return "itom1DQwtPlot.h";
}
