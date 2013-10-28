#include "evaluateGeometricsPlugin.h"
#include "pluginVersion.h"

#include <QtCore/QtPlugin>
#include "evaluateGeometrics.h"


EvaluateGeometricsPlugin::EvaluateGeometricsPlugin(QObject *parent)
    : AbstractItomDesignerPlugin(parent)
{
    m_plotDataFormats = ito::Format_Float32;
    m_plotDataTypes = ito::DataObjPlane;
    m_plotFeatures = ito::Live | ito::Static | ito::Cartesian;

    m_description = QObject::tr("itom widget");
    m_detaildescription = QObject::tr("");
    m_author = "Christian Kohler, twip OS";
    m_version = (PLUGIN_VERSION_MAJOR << 16) + (PLUGIN_VERSION_MINOR << 8) + PLUGIN_VERSION_PATCH;
    m_license = QObject::tr("LGPL 2.0");

    initialized = false;
}

void EvaluateGeometricsPlugin::initialize(QDesignerFormEditorInterface * /*core*/)
{
    if (initialized)
        return;

    initialized = true;
}

bool EvaluateGeometricsPlugin::isInitialized() const
{
    return initialized;
}

QWidget *EvaluateGeometricsPlugin::createWidget(QWidget *parent)
{
    return new EvaluateGeometricsFigure(m_itomSettingsFile, ito::AbstractFigure::ModeStandaloneInUi, parent);
}

QWidget *EvaluateGeometricsPlugin::createWidgetWithMode(ito::AbstractFigure::WindowMode winMode, QWidget * parent)
{
    return new EvaluateGeometricsFigure(m_itomSettingsFile, winMode, parent);
}

QString EvaluateGeometricsPlugin::name() const
{
    return "EvaluateGeometricsFigure";
}

QString EvaluateGeometricsPlugin::group() const
{
    return "itom Plugins";
}

QIcon EvaluateGeometricsPlugin::icon() const
{
    return QIcon(":/itomDesignerPlugins/itom/icons/q_itoM32.png");
}

QString EvaluateGeometricsPlugin::toolTip() const
{
    return QString(".");
}

QString EvaluateGeometricsPlugin::whatsThis() const
{
    return m_description;
}

bool EvaluateGeometricsPlugin::isContainer() const
{
    return false;
}

QString EvaluateGeometricsPlugin::domXml() const
{
    return "<widget class=\"EvaluateGeometrics\" name=\"EvaluateGeometrics\">\n"
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

QString EvaluateGeometricsPlugin::includeFile() const
{
    return "EvaluateGeometrics.h";
}

Q_EXPORT_PLUGIN2(EvaluateGeometrics, EvaluateGeometricsPlugin)