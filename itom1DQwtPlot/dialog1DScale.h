/* ********************************************************************
   itom measurement system
   URL: http://www.uni-stuttgart.de/ito
   Copyright (C) 2012, Institut f�r Technische Optik (ITO), 
   Universit�t Stuttgart, Germany 
 
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

#include "plot1DWidget.h"

#include "ui_dialog1DScale.h"

class Dialog1DScale : public QDialog 
{
public:
    Dialog1DScale(const InternalData &data, QWidget *parent = NULL);
    ~Dialog1DScale() {};

    void getData(InternalData &data);

private:

    void getDataTypeRange(ito::tDataType type, double &min, double &max);

    Ui::Dialog1DScale ui;

private slots:

};

#endif