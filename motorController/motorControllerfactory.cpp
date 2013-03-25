#include "motorController.h"

#include <QtCore/QtPlugin>
#include "motorControllerfactory.h"


MotorControllerFactory::MotorControllerFactory(QObject *parent)
    : QObject(parent)
{
    initialized = false;
}

void MotorControllerFactory::initialize(QDesignerFormEditorInterface * /*core*/)
{
    if (initialized)
        return;

    initialized = true;
}

bool MotorControllerFactory::isInitialized() const
{
    return initialized;
}

QWidget *MotorControllerFactory::createWidget(QWidget *parent)
{
    return new MotorController(parent);
}

QString MotorControllerFactory::name() const
{
    return "MotorController";
}

QString MotorControllerFactory::group() const
{
    return "ITOM Plugins";
}

QIcon MotorControllerFactory::icon() const
{
    return QIcon(":/itom/icons/q_itoM32.png");
}

QString MotorControllerFactory::toolTip() const
{
    return QString();
}

QString MotorControllerFactory::whatsThis() const
{
    return QObject::tr("ITOM widget to show current axis positions of a motor.");
}

bool MotorControllerFactory::isContainer() const
{
    return false;
}

QString MotorControllerFactory::domXml() const
{
    return "<widget class=\"MotorController\" name=\"MotorController\">\n"
        " <property name=\"geometry\">\n"
        "  <rect>\n"
        "   <x>0</x>\n"
        "   <y>0</y>\n"
        "   <width>470</width>\n"
        "   <height>20</height>\n"
        "  </rect>\n"
        " </property>\n"
        "</widget>\n";
}

QString MotorControllerFactory::includeFile() const
{
    return "motorController.h";
}

Q_EXPORT_PLUGIN2(MotorController, MotorControllerFactory)