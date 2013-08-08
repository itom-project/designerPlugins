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

#ifndef DIALOG2DSCALE
#define DIALOG2DSCALE

#include <QtGui>
#include <qdialog.h>

#include "plotCanvas.h"

#include "ui_dialog2DScale.h"

class Dialog2DScale : public QDialog 
{
public:
    Dialog2DScale(const InternalData &data, QWidget *parent = NULL);
    ~Dialog2DScale() {};

    void getData(InternalData &data);

private:

    void getDataTypeRange(ito::tDataType type, double &min, double &max);

    Ui::Dialog2DScale ui;

private slots:

};

#endif