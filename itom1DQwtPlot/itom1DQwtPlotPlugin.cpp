#include "itom1DQwtPlotPlugin.h"
#include "pluginVersion.h"

#include <QtCore/QtPlugin>
#include "itom1DQwtPlot.h"


Itom1DQwtPlotPlugin::Itom1DQwtPlotPlugin(QObject *parent)
    : AbstractItomDesignerPlugin(parent)
{
    m_plotDataFormats = ito::Format_Gray8 | ito::Format_Gray16 | ito::Format_Gray32 | ito::Format_Float32 | ito::Format_Float64 | ito::Format_Complex;
    m_plotDataTypes = ito::DataObjLine | ito::DataObjPlane;
    m_plotFeatures = ito::Live | ito::Static |  ito::PlotImage | ito::PlotLine | ito::Cartesian;

    m_description = QObject::tr("itom widget for 1D DataObjects based on QWT.");
    m_detaildescription = QObject::tr("");
    m_author = "Marc Gronle, ITO";
    m_version = (PLUGIN_VERSION_MAJOR << 16) + (PLUGIN_VERSION_MINOR << 8) + PLUGIN_VERSION_PATCH;
    m_license = QObject::tr("LGPL, for Qwt see Qwt License");

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

Q_EXPORT_PLUGIN2(Itom1DQwtPlot, Itom1DQwtPlotPlugin)