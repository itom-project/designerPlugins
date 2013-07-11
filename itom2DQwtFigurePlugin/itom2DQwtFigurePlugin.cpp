#include "itom2DQwtFigurePlugin.h"
#include "pluginVersion.h"

#include "plot/AbstractFigure.h"

#include <QtCore/QtPlugin>
#include "itom2DQwtFigure.h"


itom2DQwtFigurePlugin::itom2DQwtFigurePlugin(QObject *parent)
    : ito::AbstractItomDesignerPlugin(parent)
{
    m_plotDataFormats = ito::Format_Gray8 | ito::Format_Gray16 | ito::Format_Gray32 | ito::Format_Float32 | ito::Format_Float64 | ito::Format_Complex;
    m_plotDataTypes = ito::DataObjPlane;
    m_plotFeatures = ito::Static | ito::Live | ito::PlotImage | ito::Cartesian;

    m_description = QObject::tr("ITOM widget for 2D-visualisation of 2D/3D DataObjects based on QWT.");
    m_detaildescription = QObject::tr("");
    m_author = "Marc Gronle, ITO";
    m_version = (PLUGIN_VERSION_MAJOR << 16) + (PLUGIN_VERSION_MINOR << 8) + PLUGIN_VERSION_PATCH;
    m_license = QObject::tr("LGPL with ITO itom-exception");

    initialized = false;
}

void itom2DQwtFigurePlugin::initialize(QDesignerFormEditorInterface * /*core*/)
{
    if (initialized)
        return;

    initialized = true;
}

bool itom2DQwtFigurePlugin::isInitialized() const
{
    return initialized;
}

QWidget *itom2DQwtFigurePlugin::createWidget(QWidget *parent)
{
    return new itom2DQwtFigure(m_itomSettingsFile, ito::AbstractFigure::ModeStandaloneInUi, parent);
}

QWidget *itom2DQwtFigurePlugin::createWidgetWithMode(ito::AbstractFigure::WindowMode winMode, QWidget * parent)
{
    return new itom2DQwtFigure(m_itomSettingsFile, winMode, parent);
}

QString itom2DQwtFigurePlugin::name() const
{
    return "itom2DQwtFigure";
}

QString itom2DQwtFigurePlugin::group() const
{
    return "ITOM Plugins";
}

QIcon itom2DQwtFigurePlugin::icon() const
{
    return QIcon(":/itom/icons/q_itoM32.png");
}

QString itom2DQwtFigurePlugin::toolTip() const
{
    return QString("Use this widget in your UI for 2D plots in this widget's canvas.");
}

QString itom2DQwtFigurePlugin::whatsThis() const
{
    return m_description;
}

bool itom2DQwtFigurePlugin::isContainer() const
{
    return false;
}

QString itom2DQwtFigurePlugin::domXml() const
{
    return "<widget class=\"itom2DQwtFigure\" name=\"itom2DQwtFigure\">\n"
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

QString itom2DQwtFigurePlugin::includeFile() const
{
    return "itom2DQwtFigure.h";
}

Q_EXPORT_PLUGIN2(itom2DQwtFigure, itom2DQwtFigurePlugin)
