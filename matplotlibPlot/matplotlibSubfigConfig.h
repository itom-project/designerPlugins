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

#ifndef MATPLOTLIBSUBFIGCONFIG_H
#define MATPLOTLIBSUBFIGCONFIG_H

#include <QtGui>
#include <qdialog.h>

#include "ui_matplotlibSubfigConfig.h"

class MatplotlibSubfigConfig : public QDialog 
{
    Q_OBJECT
public:
    MatplotlibSubfigConfig(float valLeft, float valTop, float valRight, float valBottom, float valWSpace, float valHSpace, QWidget *parent = 0) :
        QDialog(parent)
    {
        ui.setupUi(this);

        ui.dblRangeLeftRight->setValues(valLeft / 10.0, valRight / 10.0);
        ui.dblRangeBottomTop->setValues(valBottom / 10.0, valTop / 10.0);
        ui.sliderWSpace->setValue(valWSpace / 10.0);
        ui.sliderHSpace->setValue(valHSpace / 10.0);
    }

    ~MatplotlibSubfigConfig() {};

    DoubleRangeWidget *sliderLeftRight()  { return ui.dblRangeLeftRight; }
    DoubleRangeWidget *sliderBottomTop() { return ui.dblRangeBottomTop; }
    SliderWidget *sliderHSpace() { return ui.sliderHSpace; }
    SliderWidget *sliderWSpace() { return ui.sliderWSpace; }

private:
    Ui::frmMatplotlibSubfigConfig ui;

private slots:

};

#endif
