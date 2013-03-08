#include "itom1DQwtFigurePlugin.h"

#include <QtCore/QtPlugin>
#include "itom1DQwtFigure.h"


itom1DQwtFigurePlugin::itom1DQwtFigurePlugin(QObject *parent)
    : AbstractItomDesignerPlugin(parent)
{
    m_plotDataFormats = ito::Format_Gray8 | ito::Format_Gray16 | ito::Format_Gray32 | ito::Format_Float32 | ito::Format_Float64 | ito::Format_Complex;
    m_plotDataTypes = ito::DataObjLine;
    m_plotFeatures = ito::Live | ito::Static | ito::PlotLine | ito::Cartesian;

    m_description = QObject::tr("ITOM widget for 1D DataObjects based on QWT.");
    m_detaildescription = QObject::tr("");
    m_author = "Marc Gronle, ITO";
    m_version = 0;
    m_license = QObject::tr("LGPL with ITO itom-exception");

    initialized = false;
}

void itom1DQwtFigurePlugin::initialize(QDesignerFormEditorInterface * /*core*/)
{
    if (initialized)
        return;

    initialized = true;
}

bool itom1DQwtFigurePlugin::isInitialized() const
{
    return initialized;
}

QWidget *itom1DQwtFigurePlugin::createWidget(QWidget *parent)
{
    return new itom1DQwtFigure(m_itomSettingsFile, parent);
}

QString itom1DQwtFigurePlugin::name() const
{
    return "itom1DQwtFigure";
}

QString itom1DQwtFigurePlugin::group() const
{
    return "ITOM Plugins";
}

QIcon itom1DQwtFigurePlugin::icon() const
{
    return QIcon(":/itom/icons/q_itoM32.png");
}

QString itom1DQwtFigurePlugin::toolTip() const
{
    return QString("Use this widget in your UI for 1D plots in this widget's canvas.");
}

QString itom1DQwtFigurePlugin::whatsThis() const
{
    return m_description;
}

bool itom1DQwtFigurePlugin::isContainer() const
{
    return false;
}

QString itom1DQwtFigurePlugin::domXml() const
{
    return "<widget class=\"itom1DQwtFigure\" name=\"itom1DQwtFigure\">\n"
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

QString itom1DQwtFigurePlugin::includeFile() const
{
    return "itom1DQwtFigure.h";
}

Q_EXPORT_PLUGIN2(itom1DQwtFigure, itom1DQwtFigurePlugin)