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

#ifndef DIALOG1DSCALE
#define DIALOG1DSCALE

#include <QtGui>
#include <qdialog.h>

#include "ui_dialog1DScale.h"

class Dialog1DScale : public QDialog 
{
public:
    Dialog1DScale(const double minX, const double maxX, const double minRangeX, const double maxRangeX,  const double minY, const double maxY, const double minRangeY, const double maxRangeY, const bool autoCalcX, const bool autoCalcY);
    ~Dialog1DScale() {};

    void getData(double &minX, double &maxX, double &minY, double &maxY, bool &autoCalcX, bool &autoCalcOnceX, bool &autoCalcY, bool &autoCalcOnceY);

private:
    Ui::Dialog1DScale ui;

private slots:

};

#endif