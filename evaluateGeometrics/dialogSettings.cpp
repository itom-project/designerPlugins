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

/*!
 * \file dialogSettings.cpp
 * \brief This file contains the definitions for the dialog "settings" for the evaluateGeometrics-Widget.
 */

#include "dialogSettings.h"

//-----------------------------------------------------------------------------------------------
DialogSettings::DialogSettings(const InternalInfo &data, const int &geometicElements, QWidget *parent) :
    QDialog(parent)
{
    ui.setupUi(this);

    ui.spinBoxGeometrics->setValue(geometicElements);
    ui.spinBoxRelations->setValue(data.m_relationsList.size());
    ui.lineEditValueUnit->setText(data.m_valueUnit);
    ui.spinBoxNumberDigits->setValue(data.m_numberOfDigits);

    QString relationString = data.m_relationNames[0];

    for(int i = 1; i < data.m_relationNames.size(); i++)
    {
        relationString.append("\n");
        relationString.append(data.m_relationNames[i]);
    }

    ui.plainTextEditRelationNames->setPlainText(relationString);
}

//-----------------------------------------------------------------------------------------------
void DialogSettings::getData(InternalInfo &data)
{
    data.m_valueUnit = ui.lineEditValueUnit->text();
    data.m_numberOfDigits = ui.spinBoxNumberDigits->value();
}

//-----------------------------------------------------------------------------------------------
