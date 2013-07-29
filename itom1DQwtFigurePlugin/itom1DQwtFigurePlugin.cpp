#include "itom1DQwtFigurePlugin.h"
#include "pluginVersion.h"

#include <QtCore/QtPlugin>
#include "itom1DQwtFigure.h"


Itom1DQwtFigurePlugin::Itom1DQwtFigurePlugin(QObject *parent)
    : AbstractItomDesignerPlugin(parent)
{
    m_plotDataFormats = ito::Format_Gray8 | ito::Format_Gray16 | ito::Format_Gray32 | ito::Format_Float32 | ito::Format_Float64 | ito::Format_Complex;
    m_plotDataTypes = ito::DataObjLine;
    m_plotFeatures = ito::Live | ito::Static | ito::PlotLine | ito::Cartesian;

    m_description = QObject::tr("ITOM widget for 1D DataObjects based on QWT.");
    m_detaildescription = QObject::tr("");
    m_author = "Marc Gronle, ITO";
    m_version = (PLUGIN_VERSION_MAJOR << 16) + (PLUGIN_VERSION_MINOR << 8) + PLUGIN_VERSION_PATCH;
    m_license = QObject::tr("LGPL, for Qwt see Qwt License");

    initialized = false;
}

void Itom1DQwtFigurePlugin::initialize(QDesignerFormEditorInterface * /*core*/)
{
    if (initialized)
        return;

    initialized = true;
}

bool Itom1DQwtFigurePlugin::isInitialized() const
{
    return initialized;
}

QWidget *Itom1DQwtFigurePlugin::createWidget(QWidget *parent)
{
    return new Itom1DQwtFigure(m_itomSettingsFile, ito::AbstractFigure::ModeStandaloneInUi, parent);
}

QWidget *Itom1DQwtFigurePlugin::createWidgetWithMode(ito::AbstractFigure::WindowMode winMode, QWidget * parent)
{
    return new Itom1DQwtFigure(m_itomSettingsFile, winMode, parent);
}

QString Itom1DQwtFigurePlugin::name() const
{
    return "Itom1DQwtFigure";
}

QString Itom1DQwtFigurePlugin::group() const
{
    return "itom Plugins";
}

QIcon Itom1DQwtFigurePlugin::icon() const
{
    return QIcon(":/itomDesignerPlugins/itom/icons/q_itoM32.png");
}

QString Itom1DQwtFigurePlugin::toolTip() const
{
    return QString("Use this widget in your UI for 1D plots in this widget's canvas.");
}

QString Itom1DQwtFigurePlugin::whatsThis() const
{
    return m_description;
}

bool Itom1DQwtFigurePlugin::isContainer() const
{
    return false;
}

QString Itom1DQwtFigurePlugin::domXml() const
{
    return "<widget class=\"Itom1DQwtFigure\" name=\"Itom1DQwtFigure\">\n"
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

QString Itom1DQwtFigurePlugin::includeFile() const
{
    return "itom1DQwtFigure.h";
}

Q_EXPORT_PLUGIN2(Itom1DQwtFigure, Itom1DQwtFigurePlugin)