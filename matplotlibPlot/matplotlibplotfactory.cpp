#include "matplotlibplot.h"
#include "pluginVersion.h"

#include <QtCore/QtPlugin>
#include "matplotlibplotfactory.h"


//----------------------------------------------------------------------------------------------------------------------------------
MatplotlibPlotFactory::MatplotlibPlotFactory(QObject *parent)
    : AbstractItomDesignerPlugin(parent)
{

    m_description = QObject::tr("itom widget for matplotlib plots.");
    m_detaildescription = QObject::tr("");
    m_author = "Marc Gronle, ITO";
    m_version = (PLUGIN_VERSION_MAJOR << 16) + (PLUGIN_VERSION_MINOR << 8) + PLUGIN_VERSION_PATCH;
    m_license = QObject::tr("LGPL");
    
    initialized = false;
}

//----------------------------------------------------------------------------------------------------------------------------------
void MatplotlibPlotFactory::initialize(QDesignerFormEditorInterface * /*core*/)
{
    if (initialized)
    {
        return;
    }

    initialized = true;
}

//----------------------------------------------------------------------------------------------------------------------------------
bool MatplotlibPlotFactory::isInitialized() const
{
    return initialized;
}

//----------------------------------------------------------------------------------------------------------------------------------
QWidget *MatplotlibPlotFactory::createWidget(QWidget *parent)
{
    return new MatplotlibPlot(parent);
}

//----------------------------------------------------------------------------------------------------------------------------------
QWidget *MatplotlibPlotFactory::createWidgetWithMode(ito::AbstractFigure::WindowMode winMode, QWidget * parent)
{
    return new MatplotlibPlot(parent);
}

//----------------------------------------------------------------------------------------------------------------------------------
QString MatplotlibPlotFactory::name() const
{
    return "MatplotlibPlot";
}

//----------------------------------------------------------------------------------------------------------------------------------
QString MatplotlibPlotFactory::group() const
{
    return "itom Plugins";
}

//----------------------------------------------------------------------------------------------------------------------------------
QIcon MatplotlibPlotFactory::icon() const
{
    return QIcon(":/itomDesignerPlugins/itom/icons/q_itoM32.png");
}

//----------------------------------------------------------------------------------------------------------------------------------
QString MatplotlibPlotFactory::toolTip() const
{
    return QString("Use this widget in your UI to plot matplotlib-figures in this widget's canvas.");
}

//----------------------------------------------------------------------------------------------------------------------------------
QString MatplotlibPlotFactory::whatsThis() const
{
    return m_description;
}

//----------------------------------------------------------------------------------------------------------------------------------
bool MatplotlibPlotFactory::isContainer() const
{
    return false;
}

//----------------------------------------------------------------------------------------------------------------------------------
QString MatplotlibPlotFactory::domXml() const
{
    return "<widget class=\"MatplotlibPlot\" name=\"matplotlibPlot\">\n"
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

//----------------------------------------------------------------------------------------------------------------------------------
QString MatplotlibPlotFactory::includeFile() const
{
    return "matplotlibplot.h";
}

Q_EXPORT_PLUGIN2(matplotlibplot, MatplotlibPlotFactory)
