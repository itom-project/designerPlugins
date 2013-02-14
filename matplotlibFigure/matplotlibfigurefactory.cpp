#include "matplotlibfigure.h"

#include <QtCore/QtPlugin>
#include "matplotlibfigurefactory.h"


MatplotlibFigureFactory::MatplotlibFigureFactory(QObject *parent)
    : AbstractItomDesignerPlugin(parent)
{
    initialized = false;
}

void MatplotlibFigureFactory::initialize(QDesignerFormEditorInterface * /*core*/)
{
    if (initialized)
        return;

    initialized = true;
}

bool MatplotlibFigureFactory::isInitialized() const
{
    return initialized;
}

QWidget *MatplotlibFigureFactory::createWidget(QWidget *parent)
{
    return new MatplotlibFigure(parent);
}

QString MatplotlibFigureFactory::name() const
{
    return "MatplotlibFigure";
}

QString MatplotlibFigureFactory::group() const
{
    return "ITOM Plugins";
}

QIcon MatplotlibFigureFactory::icon() const
{
    return QIcon(":/itom/icons/q_itoM32.png");
}

QString MatplotlibFigureFactory::toolTip() const
{
    return QString("Use this widget in your UI to plot matplotlib-figures in this widget's canvas.");
}

QString MatplotlibFigureFactory::whatsThis() const
{
    return QString("ITOM widget for matplotlib figures.");
}

bool MatplotlibFigureFactory::isContainer() const
{
    return false;
}

QString MatplotlibFigureFactory::domXml() const
{
    return "<widget class=\"MatplotlibFigure\" name=\"matplotlibFigure\">\n"
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

QString MatplotlibFigureFactory::includeFile() const
{
    return "matplotlibfigure.h";
}

Q_EXPORT_PLUGIN2(matplotlibfigure, MatplotlibFigureFactory)
