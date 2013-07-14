#include "itomIsoGLFigurePlugin.h"
#include "pluginVersion.h"

#include <QtCore/QtPlugin>
#include "itomIsoGLFigure.h"


ItomIsoGLWidgetPlugin::ItomIsoGLWidgetPlugin(QObject *parent)
    : ito::AbstractItomDesignerPlugin(parent)
{

    m_plotDataFormats = ito::Format_Gray8 | ito::Format_Gray16 | ito::Format_Gray32 | ito::Format_Float32 | ito::Format_Float64 | ito::Format_Complex;
    m_plotDataTypes = ito::DataObjPlane;
    m_plotFeatures = ito::Static | ito::PlotImage | ito::Cartesian;

    m_description = QObject::tr("ITOM widget for isometric visualisation of 2D DataObjects.");
    m_detaildescription = QObject::tr("");
    m_author = "Christian Kohler, ITO";
    m_version = (PLUGIN_VERSION_MAJOR << 16) + (PLUGIN_VERSION_MINOR << 8) + PLUGIN_VERSION_PATCH;
    m_license = QObject::tr("LGPL with ITO itom-exception");

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
    return new ItomIsoGLWidget(m_itomSettingsFile, ito::AbstractFigure::ModeStandaloneInUi, parent);
}

QWidget *ItomIsoGLWidgetPlugin::createWidgetWithMode(ito::AbstractFigure::WindowMode winMode, QWidget * parent)
{
    return new ItomIsoGLWidget(m_itomSettingsFile, winMode, parent);
}

QString ItomIsoGLWidgetPlugin::name() const
{
    return "ItomIsoGLWidget";
}

QString ItomIsoGLWidgetPlugin::group() const
{
    return "ITOM Plugins";
}

QIcon ItomIsoGLWidgetPlugin::icon() const
{
    return QIcon(":/itom/icons/q_itoM32.png");
}

QString ItomIsoGLWidgetPlugin::toolTip() const
{
    return QString("Use this widget in your UI for 2D plots in this widget's canvas.");
}

QString ItomIsoGLWidgetPlugin::whatsThis() const
{
    return m_description;
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

Q_EXPORT_PLUGIN2(ItomIsoGLWidgetPlugin, ItomIsoGLWidgetPlugin)
