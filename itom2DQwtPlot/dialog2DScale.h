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

#ifndef DIALOG2DSCALE
#define DIALOG2DSCALE

#include <QtGui>
#include <qdialog.h>
#include <qlocale.h>

#include "plotCanvas.h"

#include "ui_dialog2DScale.h"

class Dialog2DScale : public QDialog 
{
	Q_OBJECT

public:
    Dialog2DScale(const PlotCanvas::InternalData &data, QWidget *parent = NULL);
    ~Dialog2DScale() {};

    void getData(PlotCanvas::InternalData &data);

private:
    void getDataTypeRange(ito::tDataType type, double &min, double &max);
	bool checkValue(QLineEdit *lineEdit, const double &min, const double &max, const QString &name);

    Ui::Dialog2DScale ui;

	double m_minX;
	double m_maxX;
	double m_minY;
	double m_maxY;
	double m_minValue;
	double m_maxValue;

	QLocale m_locale;

private slots:
	void on_buttonBox_accepted();
};

#endif
