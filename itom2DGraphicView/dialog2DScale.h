/* ********************************************************************
   itom measurement system
   URL: http://www.uni-stuttgart.de/ito
   Copyright (C) 2012, Institut fuer Technische Optik (ITO), 
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

#ifndef DIALOG2DSCALE
#define DIALOG2DSCALE

#include <QtGui>
#include <qdialog.h>

#include "ui_dialog2DScale.h"

class Dialog2DScale : public QDialog 
{
public:
    Dialog2DScale(const double minX,const  double maxX, const double minRangeX, const double maxRangeX,const  double minY,const  double maxY, const double minRangeY, const double maxRangeY,const  double minZ,const  double maxZ, const int numDims, const int numPlanes, const int curPlane);
    ~Dialog2DScale() {};

    void getData(double &minX, double &maxX, double &minY, double &maxY, double &minZ, double &maxZ, int &newPlane, bool &autoCalcX, bool &autoCalcY, bool &autoCalcZ);

private:
    Ui::Dialog2DScale ui;

private slots:

};

#endif