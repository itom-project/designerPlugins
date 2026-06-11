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
#include "evaluateGeometricsPlugin.h"
#include "pluginVersion.h"

#include <QtCore/QtPlugin>
#include "evaluateGeometrics.h"
#include "gitVersion.h"

/*!
 * \file evaluateGeometricsPlugin.cpp
 * \brief This file contains the factory definition for the evaluateGeometrics-Widget.
 */

//---------------------------------------------------------------------------------------------------------------
EvaluateGeometricsPlugin::EvaluateGeometricsPlugin(QObject *parent)
    : AbstractItomDesignerPlugin(parent)
{
    m_plotDataFormats = ito::Format_Float32;
    m_plotDataTypes = ito::PlotDataTypes();
    m_plotFeatures = ito::PlotFeatures();

    m_description = QObject::tr("itom measurement widget");
    m_detaildescription = QObject::tr("The evaluate geometrics plugin can be used to evaluate geometric parameters of shapes drawn in itom plots.");
    m_author = PLUGIN_AUTHOR;
    m_version = PLUGIN_VERSION;
    m_minItomVer = PLUGIN_MIN_ITOM_VERSION;
    m_maxItomVer = PLUGIN_MAX_ITOM_VERSION;
    m_license = QObject::tr(PLUGIN_LICENCE);
    m_aboutThis = QObject::tr(GITVERSION);



    initialized = false;
}
//---------------------------------------------------------------------------------------------------------------
void EvaluateGeometricsPlugin::initialize(QDesignerFormEditorInterface * /*core*/)
{
    if (initialized)
        return;

    initialized = true;
}
//---------------------------------------------------------------------------------------------------------------
bool EvaluateGeometricsPlugin::isInitialized() const
{
    return initialized;
}
//---------------------------------------------------------------------------------------------------------------
QWidget *EvaluateGeometricsPlugin::createWidget(QWidget *parent)
{
    return new EvaluateGeometricsFigure(m_itomSettingsFile, ito::AbstractFigure::ModeStandaloneInUi, parent);
}
//---------------------------------------------------------------------------------------------------------------
QWidget *EvaluateGeometricsPlugin::createWidgetWithMode(ito::AbstractFigure::WindowMode winMode, QWidget * parent)
{
    return new EvaluateGeometricsFigure(m_itomSettingsFile, winMode, parent);
}
//---------------------------------------------------------------------------------------------------------------
QString EvaluateGeometricsPlugin::name() const
{
    return "EvaluateGeometricsFigure";
}
//---------------------------------------------------------------------------------------------------------------
QString EvaluateGeometricsPlugin::group() const
{
    return "itom Plugins";
}
//---------------------------------------------------------------------------------------------------------------
QIcon EvaluateGeometricsPlugin::icon() const
{
    return QIcon(":/itomDesignerPlugins/itom/icons/q_itoM32.png");
}
//---------------------------------------------------------------------------------------------------------------
QString EvaluateGeometricsPlugin::toolTip() const
{
    return QString("Evaluated geometric elements and mathematic relations between geometric elements.");
}
//---------------------------------------------------------------------------------------------------------------
QString EvaluateGeometricsPlugin::whatsThis() const
{
    return m_description;
}
//---------------------------------------------------------------------------------------------------------------
bool EvaluateGeometricsPlugin::isContainer() const
{
    return false;
}
//---------------------------------------------------------------------------------------------------------------
QString EvaluateGeometricsPlugin::domXml() const
{
    return "<widget class=\"EvaluateGeometricsFigure\" name=\"EvaluateGeometricsFigure\">\n"
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
//---------------------------------------------------------------------------------------------------------------
QString EvaluateGeometricsPlugin::includeFile() const
{
    return "evaluateGeometrics.h";
}
