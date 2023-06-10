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

#include "dialogExportProperties.h"

//-----------------------------------------------------------------------------------------------
DialogExportProperties::DialogExportProperties(const QSizeF &currentSizePx, QWidget *parent) :
    QDialog(parent),
    m_inEditing(false),
    m_keepAspectRatio(false)
{
    ui.setupUi(this);

    ui.lblCanvasWidth->setText(QString("%1 px").arg(currentSizePx.width()));
    ui.lblCanvasHeight->setText(QString("%1 px").arg(currentSizePx.height()));

    QStringList items;
    items << tr("user defined");
    items << tr("user defined (keep aspect ratio)");

    items << tr("A4 landscape");
    items << tr("A4 portrait");
    items << tr("A4 landscape (fitting)");
    items << tr("A4 portrait (fitting)");

    items << tr("A5 landscape");
    items << tr("A5 portrait");
    items << tr("A5 landscape (fitting)");
    items << tr("A5 portrait (fitting)");
    ui.cB_ExpType->addItems(items);

    //set initialize values
    m_inEditing = true;

    m_aspect = currentSizePx.width() / currentSizePx.height();

    ui.cB_ExpType->setCurrentIndex(0); //user defined
    ui.sB_destResolution->setValue(150);

    ui.dSB_destWidth->setValue(25.4 * currentSizePx.width() / 92.0); //todo: consider dpi of real screen (92.0 is only a default value)
    ui.dSB_destHeight->setValue(25.4 * currentSizePx.height() / 92.0); //todo: consider dpi of real screen (92.0 is only a default value)

    ui.sB_destWidth->setValue(currentSizePx.width());
    ui.sB_destHeight->setValue( currentSizePx.height());

    m_inEditing = false;
}

//-----------------------------------------------------------------------------------------------
double DialogExportProperties::pxToMm(const int &px)
{
    return 25.4 * (double)px / (double)(ui.sB_destResolution->value());
}

//-----------------------------------------------------------------------------------------------
int DialogExportProperties::mmToPx(const double &mm)
{
    return qRound((double)(ui.sB_destResolution->value()) * mm / 25.4);
}

//-----------------------------------------------------------------------------------------------
void DialogExportProperties::getData(QSizeF &exportSizePx, QSizeF &exportSizeMm, int &resolution)
{
    exportSizePx.setHeight(ui.sB_destHeight->value());
    exportSizePx.setWidth(ui.sB_destWidth->value());
    exportSizeMm.setHeight(ui.dSB_destHeight->value());
    exportSizeMm.setWidth(ui.dSB_destWidth->value());
    resolution = ui.sB_destResolution->value();
}
//-----------------------------------------------------------------------------------------------
void DialogExportProperties::updateOutput()
{
    int index = ui.cB_ExpType->currentIndex();

    bool enable = index < 2;
    ui.sB_destHeight->setEnabled(enable);
    ui.dSB_destHeight->setEnabled(enable);
    ui.sB_destWidth->setEnabled(enable);
    ui.dSB_destWidth->setEnabled(enable);

    m_keepAspectRatio = (index > 0);

    switch(index)
    {
        default:
        case 0: // user defined
            break;
        case 1: // user defined (keep aspect ratio)
            ui.dSB_destWidth->setValue( ui.dSB_destHeight->value() * m_aspect);
            break;

        case 2: // A4 landscape
            ui.dSB_destWidth->setValue( 297.0 );
            ui.dSB_destHeight->setValue( 210.0);
            break;

        case 3: // A4 portrait
            ui.dSB_destWidth->setValue(210.0);
            ui.dSB_destHeight->setValue(297.0);
            break;

        case 4: // A4 landscape (fitting)

            if(m_aspect < 1.414)
            {
                ui.dSB_destWidth->setValue( 210.0 * m_aspect);
                ui.dSB_destHeight->setValue( 210.0 );
            }
            else
            {
                ui.dSB_destWidth->setValue(297.0);
                ui.dSB_destHeight->setValue( 297.0 / m_aspect);
            }

            break;

        case 5: // A4 portrait (fitting)

            if(m_aspect < 0.707)
            {
                ui.dSB_destWidth->setValue( 297.0 * m_aspect);
                ui.dSB_destHeight->setValue( 297.0 );
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

    ui.sB_destWidth->setValue(mmToPx(ui.dSB_destWidth->value()));
    ui.sB_destHeight->setValue(mmToPx(ui.dSB_destHeight->value()));

}
//-----------------------------------------------------------------------------------------------
void DialogExportProperties::on_dSB_destHeight_valueChanged(double mm)
{
    if (!m_inEditing)
    {
        m_inEditing = true;

        ui.sB_destHeight->setValue(mmToPx(mm));

        if (m_keepAspectRatio)
        {
            ui.dSB_destWidth->setValue(mm * m_aspect);
            ui.sB_destWidth->setValue(mmToPx(mm) * m_aspect);
        }

        m_inEditing = false;
    }
}
//-----------------------------------------------------------------------------------------------
void DialogExportProperties::on_dSB_destWidth_valueChanged(double mm)
{
    if (!m_inEditing)
    {
        m_inEditing = true;

        ui.sB_destWidth->setValue(mmToPx(mm));

        if (m_keepAspectRatio)
        {
            ui.dSB_destHeight->setValue(mm/m_aspect);
            ui.sB_destHeight->setValue(mmToPx(mm)/m_aspect);
        }

        m_inEditing = false;
    }
}
//-----------------------------------------------------------------------------------------------
void DialogExportProperties::on_sB_destHeight_valueChanged(int pixel)
{
    if (!m_inEditing)
    {
        m_inEditing = true;

        ui.dSB_destHeight->setValue(pxToMm(pixel));

        if (m_keepAspectRatio)
        {
            ui.dSB_destWidth->setValue(pxToMm(pixel) * m_aspect);
            ui.sB_destWidth->setValue(pixel * m_aspect);
        }

        m_inEditing = false;
    }
}
//-----------------------------------------------------------------------------------------------
void DialogExportProperties::on_sB_destWidth_valueChanged(int pixel)
{
    if (!m_inEditing)
    {
        m_inEditing = true;

        ui.dSB_destWidth->setValue(pxToMm(pixel));

        if (m_keepAspectRatio)
        {
            ui.dSB_destHeight->setValue(pxToMm(pixel)/m_aspect);
            ui.sB_destHeight->setValue(pixel/m_aspect);
        }

        m_inEditing = false;
    }
}
//-----------------------------------------------------------------------------------------------
void DialogExportProperties::on_cB_ExpType_currentIndexChanged(int index)
{
    if (!m_inEditing)
    {
        m_inEditing = true;

        updateOutput();

        m_inEditing = false;
    }
}
//-----------------------------------------------------------------------------------------------
void DialogExportProperties::on_sB_destResolution_valueChanged(int value)
{
    if (!m_inEditing)
    {
        m_inEditing = true;

        updateOutput();

        m_inEditing = false;
    }
}
//-----------------------------------------------------------------------------------------------
