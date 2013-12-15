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

#ifndef DIALOGEXPORT
#define DIALOGEXPORT

#include <QtGui>
#include <qdialog.h>

#include "ui_dialogExportProperties.h"

class DialogExportProperties : public QDialog 
{
    Q_OBJECT

public:
    

    DialogExportProperties(const QString &type, const QSizeF curSize, QWidget *parent = NULL);
    ~DialogExportProperties() {};

    void getData(QSizeF &exportSize, int &resolution);

private:
    QSizeF m_startSize;
    Ui::DialogExport2File ui;
    int m_currentMode;
    QStringList m_items;

    bool m_skipMetricX;
    bool m_skipMetricY;
    bool m_skipPixelX;
    bool m_skipPixelY;

    double m_aspect;

private slots:
    void updateOutPut(void);

    void on_dSB_destHeight_valueChanged(double input);
    void on_dSB_destWidth_valueChanged(double input);

    void on_sB_destHeight_valueChanged(int input);
    void on_sB_destWidth_valueChanged(int input);

    void on_cB_ExpType_currentIndexChanged(int index);
    void on_sB_destRolution_valueChanged(int input);

};

#endif