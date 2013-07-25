#include "dataObjectTable.h"

#include <QtCore/QtPlugin>
#include "dataobjecttablefactory.h"


DataObjectTableFactory::DataObjectTableFactory(QObject *parent)
    : QObject(parent)
{
    initialized = false;
}

void DataObjectTableFactory::initialize(QDesignerFormEditorInterface * /*core*/)
{
    if (initialized)
        return;

    initialized = true;
}

bool DataObjectTableFactory::isInitialized() const
{
    return initialized;
}

QWidget *DataObjectTableFactory::createWidget(QWidget *parent)
{
    return new DataObjectTable(parent);
}

QString DataObjectTableFactory::name() const
{
    return "DataObjectTable";
}

QString DataObjectTableFactory::group() const
{
    return "itom Plugins";
}

QIcon DataObjectTableFactory::icon() const
{
    return QIcon(":/itomDesignerPlugins/itom/icons/q_itoM32.png");
}

QString DataObjectTableFactory::toolTip() const
{
    return QString();
}

QString DataObjectTableFactory::whatsThis() const
{
    return QObject::tr("itom widget to interprete a dataObject as a table.");
}

bool DataObjectTableFactory::isContainer() const
{
    return false;
}

QString DataObjectTableFactory::domXml() const
{
    return "<widget class=\"DataObjectTable\" name=\"dataObjectTable\">\n"
        " <attribute name=\"verticalHeaderDefaultSectionSize\">\n \
            <number>20</number>\n \
          </attribute>\n"
        " <attribute name=\"horizontalHeaderDefaultSectionSize\">\n \
            <number>100</number>\n \
          </attribute>\n"
        "<property name=\"rowCount\">\n \
            <number>3</number>\n \
           </property>\n \
         <property name=\"columnCount\">\n \
            <number>3</number>\n \
           </property>\n \
        </widget>\n";
}

QString DataObjectTableFactory::includeFile() const
{
    return "dataObjectTable.h";
}

Q_EXPORT_PLUGIN2(dataObjectTable, DataObjectTableFactory)