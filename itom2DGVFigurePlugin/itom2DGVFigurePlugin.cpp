#include "itom2DGVFigurePlugin.h"

#include <QtCore/QtPlugin>
#include "itom2DGVFigure.h"


itom2DGVFigurePlugin::itom2DGVFigurePlugin(QObject *parent)
    : ito::AbstractItomDesignerPlugin(parent)
{
    m_plotDataFormats = ito::Format_Gray8 | ito::Format_Gray16 | ito::Format_Gray32 | ito::Format_Float32 | ito::Format_ARGB32 | ito::Format_RGB32 | ito::Format_Float64 | ito::Format_Complex;
    m_plotDataTypes = ito::DataObjPlane;
    m_plotFeatures = ito::Live | ito::Static | ito::PlotImage | ito::Cartesian;

    m_description = QObject::tr("ITOM widget for 2D DataObjects.");
    m_detaildescription = QObject::tr("");
    m_author = "Marc Gronle, ITO";
    m_version = 0;
    m_license = QObject::tr("LGPL with ITO itom-exception");

    initialized = false;
}

void itom2DGVFigurePlugin::initialize(QDesignerFormEditorInterface * /*core*/)
{
    if (initialized)
        return;

    initialized = true;
}

bool itom2DGVFigurePlugin::isInitialized() const
{
    return initialized;
}

QWidget *itom2DGVFigurePlugin::createWidget(QWidget *parent)
{
    return new itom2DGVFigure(m_itomSettingsFile, ito::AbstractFigure::ModeStandaloneInUi, parent);
}

QWidget *itom2DGVFigurePlugin::createWidgetWithMode(ito::AbstractFigure::WindowMode winMode, QWidget * parent)
{
    return new itom2DGVFigure(m_itomSettingsFile, winMode, parent);
}

QString itom2DGVFigurePlugin::name() const
{
    return "itom2DGVFigure";
}

QString itom2DGVFigurePlugin::group() const
{
    return "ITOM Plugins";
}

QIcon itom2DGVFigurePlugin::icon() const
{
    return QIcon(":/itom/icons/q_itoM32.png");
}

QString itom2DGVFigurePlugin::toolTip() const
{
    return QString("Use this widget in your UI for 2D plots in this widget's canvas.");
}

QString itom2DGVFigurePlugin::whatsThis() const
{
    return m_description;
}

bool itom2DGVFigurePlugin::isContainer() const
{
    return false;
}

QString itom2DGVFigurePlugin::domXml() const
{
    return "<widget class=\"itom2DGVFigure\" name=\"itom2DGVFigure\">\n"
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

QString itom2DGVFigurePlugin::includeFile() const
{
    return "itom2DGVFigure.h";
}

Q_EXPORT_PLUGIN2(itom2DGVFigurePlugin, itom2DGVFigurePlugin)
