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

#include "dialogExportProperties.h"

//-----------------------------------------------------------------------------------------------
DialogExportProperties::DialogExportProperties(const QString &type, const QSizeF curSize, QWidget *parent) :
    QDialog(parent)
{
    ui.setupUi(this);

    m_currentMode = 0;

    m_skipMetricX = false;
    m_skipMetricY = false;
    m_skipPixelX = false;
    m_skipPixelY = false;

    m_startSize = curSize;
    m_aspect = curSize.width() / curSize.height();

    ui.sB_orHeight->setValue(curSize.height());
    ui.sB_orWidth->setValue(curSize.width());

    ui.sB_destRolution->setValue(150);

    ui.dSB_destWidth->setValue(curSize.width() / ui.sB_destRolution->value() * 25.4 );
    ui.dSB_destHeight->setValue( curSize.height() / ui.sB_destRolution->value() * 25.4);

    m_items.clear();

    m_items << tr("user defined");
    m_items << tr("user defined (keep aspect)");

    m_items << tr("A4 landscape");
    m_items << tr("A4 portrait");
    m_items << tr("A4 landscape (fitting)");
    m_items << tr("A4 portrait (fitting)");

    m_items << tr("A5 landscape");
    m_items << tr("A5 portrait");
    m_items << tr("A5 landscape (fitting)");
    m_items << tr("A5 portrait (fitting)");

    ui.cB_ExpType->addItems(m_items);

    updateOutPut();

    return;
}

//-----------------------------------------------------------------------------------------------
void DialogExportProperties::getData(QSizeF &exportSize, int &resolution)
{
    exportSize.setHeight(ui.dSB_destHeight->value());
    exportSize.setWidth(ui.dSB_destWidth->value());
    resolution = ui.sB_destRolution->value();

    return;
}
//-----------------------------------------------------------------------------------------------
void DialogExportProperties::updateOutPut()
{
    m_skipMetricX = true;
    m_skipMetricY = true;
    m_skipPixelX = true;
    m_skipPixelY = true;

    int index = ui.cB_ExpType->currentIndex();

    bool disable = index < 2;
    ui.sB_destHeight->setEnabled(disable);
    ui.dSB_destHeight->setEnabled(disable);


    disable = index < 1;
    ui.sB_destWidth->setEnabled(disable);
    ui.dSB_destWidth->setEnabled(disable);

    switch(index)
    {
        default:
        case 0: // user defined
            break;
        case 1: // user defined (keep aspect)
            ui.dSB_destWidth->setValue( ui.dSB_destHeight->value() * m_aspect);
            break;

        case 2: // A4 landscape
            ui.dSB_destWidth->setValue( 290.0 );
            ui.dSB_destHeight->setValue( 210.0);
            break;

        case 3: // A4 portrait
            ui.dSB_destWidth->setValue(210.0);
            ui.dSB_destHeight->setValue(290.0);
            break;

        case 4: // A4 landscape (fitting)

            if(m_aspect < 1.380)
            {
                ui.dSB_destWidth->setValue( 210.0 * m_aspect);
                ui.dSB_destHeight->setValue( 210.0 );
            }
            else
            {
                ui.dSB_destWidth->setValue(290.0);
                ui.dSB_destHeight->setValue( 290.0 / m_aspect);
            }

            break;

        case 5: // A4 portrait (fitting)

            if(m_aspect < 0.724)
            {
                ui.dSB_destWidth->setValue( 290.0 * m_aspect);
                ui.dSB_destHeight->setValue( 290.0 );
            }
            else
            {
                ui.dSB_destWidth->setValue(210.0);
                ui.dSB_destHeight->setValue( 210.0 / m_aspect);
            }

            break;

        case 6: // A5 landscape 210 x 148
            ui.dSB_destWidth->setValue( 210.0 );
            ui.dSB_destHeight->setValue( 148.0);

            break;

        case 7: // A5 portrait 148 x 210
            ui.dSB_destWidth->setValue( 148.0 );
            ui.dSB_destHeight->setValue( 210.0);

            break;

        case 8: // A5 landscape (fitting)

            if(m_aspect  < 1.414)
            {
                ui.dSB_destWidth->setValue( 148.0 * m_aspect);
                ui.dSB_destHeight->setValue(148.0);
            }
            else
            {
                ui.dSB_destWidth->setValue(210.0);    
                ui.dSB_destHeight->setValue( 210.0 / m_aspect);
            }

            break;

        case 9: // A5 portrait (fitting)

            if(m_aspect  < 0.707)
            {
                ui.dSB_destWidth->setValue( 210.0 * m_aspect);
                ui.dSB_destHeight->setValue(210.0);
            }
            else
            {
                ui.dSB_destWidth->setValue(148.0);   
                ui.dSB_destHeight->setValue( 148.0 / m_aspect);
            }

            break;
        
    }

    ui.sB_destWidth->setValue(ui.dSB_destWidth->value() / 25.4 * ui.sB_destRolution->value());
    ui.sB_destHeight->setValue(ui.dSB_destHeight->value() / 25.4 * ui.sB_destRolution->value());

}
//-----------------------------------------------------------------------------------------------
void DialogExportProperties::on_dSB_destHeight_valueChanged(double input)
{
    if(m_skipMetricX)
    {
        m_skipMetricX = false;
        return;
    }
    m_skipPixelX = true;
    ui.sB_destHeight->setValue(input * ui.sB_destRolution->value() / 25.4);

    if(ui.cB_ExpType->currentIndex() == 1)
    {
        ui.dSB_destWidth->setValue( ui.dSB_destHeight->value() * m_aspect);
    }
    
    return;
}
//-----------------------------------------------------------------------------------------------
void DialogExportProperties::on_dSB_destWidth_valueChanged(double input)
{
    if(m_skipMetricY)
    {
        m_skipMetricY = false;
        return;
    }
    m_skipPixelY = true;  
    ui.sB_destWidth->setValue(input * ui.sB_destRolution->value() / 25.4);
    
    return;
}
//-----------------------------------------------------------------------------------------------
void DialogExportProperties::on_sB_destHeight_valueChanged(int input)
{
    if(m_skipPixelX)
    {
        m_skipPixelX = false;
        return;
    }
    m_skipMetricX = true;
    ui.dSB_destHeight->setValue((double)input * 25.4 /  ui.sB_destRolution->value());
    if(ui.cB_ExpType->currentIndex() == 1)
    {
        ui.dSB_destWidth->setValue( ui.dSB_destHeight->value() * m_aspect);
    }

    return;
}
//-----------------------------------------------------------------------------------------------
void DialogExportProperties::on_sB_destWidth_valueChanged(int input)
{
    if(m_skipPixelY)
    {
        m_skipPixelY = false;
        return;
    }
    m_skipMetricY = true;
    ui.dSB_destWidth->setValue((double)input * 25.4 / ui.sB_destRolution->value());

    return;
}
//-----------------------------------------------------------------------------------------------
void DialogExportProperties::on_cB_ExpType_currentIndexChanged(int index)
{
    updateOutPut();
}
//-----------------------------------------------------------------------------------------------
void DialogExportProperties::on_sB_destRolution_valueChanged(int input)
{
    updateOutPut();
}
//-----------------------------------------------------------------------------------------------