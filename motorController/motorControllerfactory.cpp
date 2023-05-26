/* ********************************************************************
   itom measurement system
   URL: http://www.uni-stuttgart.de/ito
   Copyright (C) 2018, Institut fuer Technische Optik (ITO),
   Universitaet Stuttgart, Germany

   This file is part of itom.

   itom is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   itom is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with itom. If not, see <http://www.gnu.org/licenses/>.
*********************************************************************** */

/**
* \file motorControllerFactory.cpp
* \brief In this file the functions of the factory for the MotorController-Widget are defined
*
*    The MotorController-Class defines a widget for generic motor monitoring and controll. The following files are
*   needed: MotorController.cpp, MotorController.h, MotorControllerFactory.h, MotorControllerFactory.cpp
*
*\sa MotorController, MotorControllerFactory, MotorController.h
*\author ITO
*\date    2013
*/

#include "motorController.h"

#include <QtCore/QtPlugin>
#include "motorControllerfactory.h"

//-----------------------------------------------------------------------------------------------
MotorControllerFactory::MotorControllerFactory(QObject *parent)
    : QObject(parent)
{
    initialized = false;
}
//-----------------------------------------------------------------------------------------------
void MotorControllerFactory::initialize(QDesignerFormEditorInterface * /*core*/)
{
    if (initialized)
        return;

    initialized = true;
}
//-----------------------------------------------------------------------------------------------
bool MotorControllerFactory::isInitialized() const
{
    return initialized;
}
//-----------------------------------------------------------------------------------------------
QWidget *MotorControllerFactory::createWidget(QWidget *parent)
{
    return new MotorController(parent);
}
//-----------------------------------------------------------------------------------------------
QString MotorControllerFactory::name() const
{
    return "MotorController";
}
//-----------------------------------------------------------------------------------------------
QString MotorControllerFactory::group() const
{
    return "itom Plugins";
}
//-----------------------------------------------------------------------------------------------
QIcon MotorControllerFactory::icon() const
{
    return QIcon(":/itomDesignerPlugins/itom/icons/q_itoM32.png");
}
//-----------------------------------------------------------------------------------------------
QString MotorControllerFactory::toolTip() const
{
    return QString();
}
//-----------------------------------------------------------------------------------------------
QString MotorControllerFactory::whatsThis() const
{
    return QObject::tr("ITOM widget to show current axis positions of a motor.");
}
//-----------------------------------------------------------------------------------------------
bool MotorControllerFactory::isContainer() const
{
    return false;
}
//-----------------------------------------------------------------------------------------------
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
//-----------------------------------------------------------------------------------------------
QString MotorControllerFactory::includeFile() const
{
    return "motorController.h";
}
