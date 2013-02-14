/* ********************************************************************
   itom measurement system
   URL: http://www.uni-stuttgart.de/ito
   Copyright (C) 2012, Institut für Technische Optik (ITO), 
   Universität Stuttgart, Germany 
 
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

#include "dialog1DScale.h"

Dialog1DScale::Dialog1DScale(const double minX, const double maxX, const double minRangeX, const double maxRangeX,  const double minY, const double maxY, const double minRangeY, const double maxRangeY, const bool autoCalcX, const bool autoCalcY)
{
    ui.setupUi(this);
    ui.radioManualX->setChecked(!autoCalcX);
    ui.radioManualY->setChecked(!autoCalcY);
    
    ui.doubleSpinMinX->setMinimum(minRangeX);
    ui.doubleSpinMinX->setMaximum(maxRangeX);
    ui.doubleSpinMaxX->setMinimum(minRangeX);
    ui.doubleSpinMaxX->setMaximum(maxRangeX);

    ui.doubleSpinMinX->setValue(minX);
    ui.doubleSpinMaxX->setValue(maxX);

    
    ui.doubleSpinMinY->setMinimum(minRangeY);
    ui.doubleSpinMinY->setMaximum(maxRangeY);

    ui.doubleSpinMaxY->setMinimum(minRangeY);
    ui.doubleSpinMaxY->setMaximum(maxRangeY);

    ui.doubleSpinMinY->setValue(minY);
    ui.doubleSpinMaxY->setValue(maxY);

    ui.groupPlane->setVisible(false);
}

void Dialog1DScale::getData(double &minX, double &maxX, double &minY, double &maxY, bool &autoCalcX, bool &autoCalcOnceX, bool &autoCalcY, bool &autoCalcOnceY)
{
    autoCalcOnceX = false;
    autoCalcOnceY = false;
    autoCalcX = ui.radioAutoCalcX->isChecked();
    autoCalcY = ui.radioAutoCalcY->isChecked();
    minX = ui.doubleSpinMinX->value();
    maxX = ui.doubleSpinMaxX->value();
    minY = ui.doubleSpinMinY->value();
    maxY = ui.doubleSpinMaxY->value();
}