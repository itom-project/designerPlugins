/* ********************************************************************
   itom measurement system
   URL: http://www.uni-stuttgart.de/ito
   Copyright (C) 2020, Institut für Technische Optik (ITO),
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
#include "itomIsoGLFigure.h"

#include <QtCore/QtPlugin>
#include "itomIsoGLFigurePlugin.h"
#include "pluginVersion.h"
#include "gitVersion.h"

ItomIsoGLWidgetPlugin::ItomIsoGLWidgetPlugin(QObject *parent)
    : ito::AbstractItomDesignerPlugin(parent)
{

    m_plotDataFormats = ito::Format_Gray8 | ito::Format_Gray16 | ito::Format_Gray32 | ito::Format_Float32 | ito::Format_Float64 | ito::Format_Complex;
    m_plotDataTypes = ito::DataObjPlane | ito::PointCloud | ito::PolygonMesh;
    m_plotFeatures = ito::Static | ito::PlotImage | ito::Cartesian | ito::Plot3D | ito::PlotISO;

    m_description = QObject::tr("ITOM widget for isometric visualisation of 2D DataObjects.");
    m_detaildescription = QObject::tr("");

    m_author = PLUGIN_AUTHOR;
    m_version = PLUGIN_VERSION;
    m_minItomVer = PLUGIN_MIN_ITOM_VERSION;
    m_maxItomVer = PLUGIN_MAX_ITOM_VERSION;
    m_license = QObject::tr(PLUGIN_LICENCE);
    m_aboutThis = QObject::tr(GITVERSION);

    initialized = false;
}

void ItomIsoGLWidgetPlugin::initialize(QDesignerFormEditorInterface * /*core*/)
{
    if (initialized)
        return;

    initialized = true;
}

bool ItomIsoGLWidgetPlugin::isInitialized() const
{
    return initialized;
}

QWidget *ItomIsoGLWidgetPlugin::createWidget(QWidget *parent)
{
    return new ItomIsoGLWidget(m_itomSettingsFile, ito::ParamBase::DObjPtr, ito::AbstractFigure::ModeStandaloneInUi, parent);
}

QWidget *ItomIsoGLWidgetPlugin::createWidgetWithMode(ito::AbstractFigure::WindowMode winMode, QWidget * parent)
{
    return new ItomIsoGLWidget(m_itomSettingsFile, ito::ParamBase::DObjPtr, winMode, parent);
}

QString ItomIsoGLWidgetPlugin::name() const
{
    return "ItomIsoGLWidget";
}

QString ItomIsoGLWidgetPlugin::group() const
{
    return "itom Plugins";
}

QIcon ItomIsoGLWidgetPlugin::icon() const
{
    return QIcon(":/itomDesignerPlugins/itom/icons/q_itoM32.png");
}

QString ItomIsoGLWidgetPlugin::toolTip() const
{
    return QString("Use this widget in your UI for 2D plots in this widget's canvas.");
}

QString ItomIsoGLWidgetPlugin::whatsThis() const
{
    //return m_description;
    return QString("itom widget for 2D DataObjects.");
}

bool ItomIsoGLWidgetPlugin::isContainer() const
{
    return false;
}

QString ItomIsoGLWidgetPlugin::domXml() const
{
    return "<widget class=\"ItomIsoGLWidget\" name=\"ItomIsoGLWidget\">\n"
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

QString ItomIsoGLWidgetPlugin::includeFile() const
{
    return "ItomIsoGLWidget.h";
}
