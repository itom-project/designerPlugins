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

#ifndef DIALOGEXPORT
#define DIALOGEXPORT

#include "itomQwtPlotBase.h"

#include <QtGui>
#include <qdialog.h>

#include "ui_dialogExportProperties.h"

class ITOMQWTPLOTBASE_EXPORT DialogExportProperties : public QDialog
{
    Q_OBJECT

public:


    DialogExportProperties(const QSizeF &currentSizePx, QWidget *parent = NULL);
    ~DialogExportProperties() {};

    void getData(QSizeF &exportSizePx, QSizeF &exportSizeMm, int &resolution);

protected:
    void updateOutput(void);

    double pxToMm(const int &px);
    int mmToPx(const double &mm);

private:
    Ui::DialogExport2File ui;

    double m_aspect; /*!< canvas width / canvas height */

    bool m_inEditing;
    bool m_keepAspectRatio;

private slots:
    void on_dSB_destHeight_valueChanged(double mm);
    void on_dSB_destWidth_valueChanged(double mm);

    void on_sB_destHeight_valueChanged(int pixel);
    void on_sB_destWidth_valueChanged(int pixel);

    void on_cB_ExpType_currentIndexChanged(int index);
    void on_sB_destResolution_valueChanged(int value);

};

#endif
