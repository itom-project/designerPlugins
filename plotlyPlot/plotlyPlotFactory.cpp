/* ********************************************************************
   itom measurement system
   URL: http://www.uni-stuttgart.de/ito
   Copyright (C) 2021, Institut fuer Technische Optik (ITO),
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
#include "plotlyPlotFactory.h"
#include "pluginVersion.h"

#include <QtCore/QtPlugin>
#include "plotlyPlot.h"


//-------------------------------------------------------------------------------------
PlotlyPlotFactory::PlotlyPlotFactory(QObject *parent)
    : ito::AbstractItomDesignerPlugin(parent)
{
    m_plotDataFormats = ito::PlotDataFormat();
    m_plotDataTypes = ito::PlotDataTypes();
    m_plotFeatures = ito::PlotFeatures();

    m_description = QObject::tr("itom widget for plotly figures.");
    m_detaildescription = QObject::tr("");
    m_author = "Marc Gronle, ITO";
    m_version = (PLUGIN_VERSION_MAJOR << 16) + (PLUGIN_VERSION_MINOR << 8) + PLUGIN_VERSION_PATCH;
    m_license = QObject::tr("LGPL");
    
    initialized = false;
}

//-------------------------------------------------------------------------------------
void PlotlyPlotFactory::initialize(QDesignerFormEditorInterface * /*core*/)
{
    if (initialized)
    {
        return;
    }

    initialized = true;
}

//-------------------------------------------------------------------------------------
bool PlotlyPlotFactory::isInitialized() const
{
    return initialized;
}

//-------------------------------------------------------------------------------------
QWidget *PlotlyPlotFactory::createWidget(QWidget *parent)
{
    return new PlotlyPlot(m_itomSettingsFile, ito::AbstractFigure::ModeStandaloneInUi, parent);
}

//-------------------------------------------------------------------------------------
QWidget *PlotlyPlotFactory::createWidgetWithMode(ito::AbstractFigure::WindowMode winMode, QWidget * parent)
{
    return new PlotlyPlot(m_itomSettingsFile, winMode, parent);
}

//-------------------------------------------------------------------------------------
QString PlotlyPlotFactory::name() const
{
    return "PlotlyPlot";
}

//-------------------------------------------------------------------------------------
QString PlotlyPlotFactory::group() const
{
    return "itom Plugins";
}

//-------------------------------------------------------------------------------------
QIcon PlotlyPlotFactory::icon() const
{
    return QIcon(":/itomDesignerPlugins/itom/icons/q_itoM32.png");
}

//-------------------------------------------------------------------------------------
QString PlotlyPlotFactory::toolTip() const
{
    return QString("Use this widget in your UI to plot plotly figures in this widget's canvas.");
}

//-------------------------------------------------------------------------------------
QString PlotlyPlotFactory::whatsThis() const
{
    return m_description;
}

//-------------------------------------------------------------------------------------
bool PlotlyPlotFactory::isContainer() const
{
    return false;
}

//-------------------------------------------------------------------------------------
QString PlotlyPlotFactory::domXml() const
{
    return "<widget class=\"PlotlyPlot\" name=\"plotlyPlot\">\n"
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

//-------------------------------------------------------------------------------------
QString PlotlyPlotFactory::includeFile() const
{
    return "plotlyPlot.h";
}
