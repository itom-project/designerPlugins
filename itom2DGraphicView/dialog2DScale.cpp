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

#include "dialog2DScale.h"

Dialog2DScale::Dialog2DScale(const double minX, const double maxX, const double minRangeX, const double maxRangeX, const double minY, const double maxY, const double minRangeY, const double maxRangeY, const double minZ, const double maxZ, const int numDims, const int numPlanes, const int curPlane)
{
    ui.setupUi(this);
    ui.radioManualX->setChecked(true);
    ui.radioManualY->setChecked(true);
    ui.radioManualZ->setChecked(true);
    ui.spinMinX->setMinimum(minRangeX);
    ui.spinMinX->setMaximum(maxRangeX);
    ui.spinMinX->setValue(minX);

    ui.spinMaxX->setMinimum(minRangeX);
    ui.spinMaxX->setMaximum(maxRangeX);
    ui.spinMaxX->setValue(maxX);

    ui.spinMinY->setMinimum(minRangeY);
    ui.spinMinY->setMaximum(maxRangeY);
    ui.spinMinY->setValue(minY);

    ui.spinMaxY->setMinimum(minRangeY);
    ui.spinMaxY->setMaximum(maxRangeY);
    ui.spinMaxY->setValue(maxY);
    ui.spinMinZ->setValue(minZ);
    ui.spinMaxZ->setValue(maxZ);

    if(numDims < 3)
    {
        ui.groupPlane->setVisible(false);
    }
    ui.spinBox_curplane->setValue(curPlane);
    ui.spinBox_curplane->setMaximum(numPlanes-1);
    ui.spinBox_totalplane->setValue(numPlanes);
}

void Dialog2DScale::getData(double &minX, double &maxX, double &minY, double &maxY, double &minZ, double &maxZ, int & /*newPlane*/, bool &autoCalcX, bool &autoCalcY, bool &autoCalcZ)
{
    autoCalcX = ui.radioAutoCalcX->isChecked();
    autoCalcY = ui.radioAutoCalcY->isChecked();
    autoCalcZ = ui.radioAutoCalcZ->isChecked();
    minX = ui.spinMinX->value();
    maxX = ui.spinMaxX->value();
    minY = ui.spinMinY->value();
    maxY = ui.spinMaxY->value();
    minZ = ui.spinMinZ->value();
    maxZ = ui.spinMaxZ->value();
}
