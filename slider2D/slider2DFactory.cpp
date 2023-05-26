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

#include "slider2D.h"

#include <QtCore/QtPlugin>
#include "slider2DFactory.h"


Slider2DFactory::Slider2DFactory(QObject *parent)
    : QObject(parent)
{
    initialized = false;
}

void Slider2DFactory::initialize(QDesignerFormEditorInterface * /*core*/)
{
    if (initialized)
        return;

    initialized = true;
}

bool Slider2DFactory::isInitialized() const
{
    return initialized;
}

QWidget *Slider2DFactory::createWidget(QWidget *parent)
{
    return new Slider2D(parent);
}

QString Slider2DFactory::name() const
{
    return "Slider2D";
}

QString Slider2DFactory::group() const
{
    return "itom Plugins";
}

QIcon Slider2DFactory::icon() const
{
    return QIcon(":/itomDesignerPlugins/itom/icons/q_itoM32.png");
}

QString Slider2DFactory::toolTip() const
{
    return QString();
}

QString Slider2DFactory::whatsThis() const
{
    return QObject::tr("itom widget : color wheel.");
}

bool Slider2DFactory::isContainer() const
{
    return false;
}

QString Slider2DFactory::domXml() const
{
    return "<widget class=\"Slider2D\" name=\"Slider2D\">\n"
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

QString Slider2DFactory::includeFile() const
{
    return "slider2D.h";
}
