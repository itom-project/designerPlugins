/* ********************************************************************
#    twipOGLFigure-Plugin for itom
#    URL: http://www.twip-os.com
#    Copyright (C) 2014, twip optical solutions GmbH,
#    Stuttgart, Germany
#
#    This files is part of the designer-Plugin twipOGLFigure for the
#    measurement software itom. All files of this plugin, where not stated
#    as port of the itom sdk, fall under the GNU Library General
#    Public Licence and must behandled accordingly.
#
#    twipOGLFigure is free software; you can redistribute it and/or modify it
#    under the terms of the GNU Library General Public Licence as published by
#    the Free Software Foundation; either version 2 of the Licence, or (at
#    your option) any later version.
#
#    Information under www.twip-os.com or mail to info@twip-os.com.
#
#    This itom-plugin is distributed in the hope that it will be useful,
#    but WITHOUT ANY WARRANTY; without even the implied warranty of
#    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
#    GNU General Public License for more details.
#
#    itom is free software by ITO, University Stuttgart published under
#    GNU General Public License as published by the Free Software
#    Foundation. See <https://github.com/itom-project/itom>
#
#    You should have received a copy of the GNU Library General Public License
#    along with itom. If not, see <http://www.gnu.org/licenses/>.
*********************************************************************** */

#define ITOM_IMPORT_API
#define ITOM_IMPORT_PLOTAPI
#include "twipOGLPlugin.h"

#include <QtCore/QtPlugin>
#include "twipOGLFigure.h"
#include "pluginVersion.h"
TwipOGLPlugin::TwipOGLPlugin(QObject *parent)
    : ito::AbstractItomDesignerPlugin(parent)
{
    m_plotDataFormats = ito::Format_Gray8 | ito::Format_Gray16 | ito::Format_Gray32 | ito::Format_Float32 | ito::Format_Float64 | ito::Format_Complex;
    m_plotDataTypes = ito::DataObjPlane | ito::PointCloud | ito::PolygonMesh;
    m_plotFeatures = ito::Static | ito::PlotImage | ito::Cartesian | ito::Plot3D | ito::PlotISO;

    m_description = QObject::tr("twip optical solutions GmbH widget for visualisation of 2D / 3D DataObjects and PointClouds.");
    m_detaildescription = QObject::tr("");
    m_author = "Christian Kohler, twip optical solutions GmbH";
    m_version = (PLUGIN_VERSION_MAJOR << 16) + (PLUGIN_VERSION_MINOR << 8) + PLUGIN_VERSION_PATCH;
    m_license = QObject::tr("Published under LGPL v2.0 by twip optical solutions GmbH");

    initialized = false;
}

void TwipOGLPlugin::initialize(QDesignerFormEditorInterface * /*core*/)
{
    if (initialized)
        return;

    initialized = true;
}

bool TwipOGLPlugin::isInitialized() const
{
    return initialized;
}

QWidget *TwipOGLPlugin::createWidget(QWidget *parent)
{
    return new TwipOGLFigure(m_itomSettingsFile, ito::ParamBase::DObjPtr, ito::AbstractFigure::ModeStandaloneInUi, parent);
}

QWidget *TwipOGLPlugin::createWidgetWithMode(ito::AbstractFigure::WindowMode winMode, QWidget * parent)
{
    return new TwipOGLFigure(m_itomSettingsFile, ito::ParamBase::DObjPtr, winMode, parent);
}

QString TwipOGLPlugin::name() const
{
    return "TwipOGLFigure";
}

QString TwipOGLPlugin::group() const
{
    return "twip Plugins";
}

QIcon TwipOGLPlugin::icon() const
{
    return QIcon(":/twipDesignerPlugins/general/icons/topoTwip.png");
}

QString TwipOGLPlugin::toolTip() const
{
    return QString("Use this widget in your UI for 2D / 3D plots in this widget's canvas.");
}

QString TwipOGLPlugin::whatsThis() const
{
    return QString("twip widget for 2D / 3D DataObjects and PointClouds.");
}

bool TwipOGLPlugin::isContainer() const
{
    return false;
}

QString TwipOGLPlugin::domXml() const
{
    return "<widget class=\"TwipOGLFigure\" name=\"twipOGLFigure\">\n"
        " <property name=\"geometry\">\n"
        "  <rect>\n"
        "   <x>0</x>\n"
        "   <y>0</y>\n"
        "   <width>400</width>\n"
        "   <height>400</height>\n"
        "  </rect>\n"
        " </property>\n"
        "</widget>\n";
}

QString TwipOGLPlugin::includeFile() const
{
    return "twipOGLFigure.h";
}

#if QT_VERSION <  QT_VERSION_CHECK(5,0,0)
    Q_EXPORT_PLUGIN2(TwipOGLPlugin, TwipOGLPlugin)
#endif
