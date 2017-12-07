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

#include "valuePicker2d.h"

#include <qpainter.h>
#include <qbrush.h>
#include <qwt_plot_canvas.h>

#include "common/numeric.h"

//----------------------------------------------------------------------------------------------------------------------------------
ValuePicker2D::ValuePicker2D(int xAxis, int yAxis, QWidget* parent, const DataObjRasterData* valueData, const DataObjRasterData* overlayData) : 
    QwtPlotPicker(xAxis, yAxis, parent),
    m_valueData(valueData),
    m_overlayData(overlayData),
    m_showOverlayInfo(false)
{
}

//----------------------------------------------------------------------------------------------------------------------------------
ValuePicker2D::~ValuePicker2D()
{
}

//----------------------------------------------------------------------------------------------------------------------------------
QwtText ValuePicker2D::trackerTextF( const QPointF &pos ) const
{
    QString text;
    QString coordinates;
    if (m_valueData)
    {
        double sx, sy, ox, oy;
        m_valueData->getPlaneScaleAndOffset(sy, sx, oy, ox);

        if (sx == 0.0 || sy == 0.0 || (ito::areEqual<double>(oy, 0.0) && ito::areEqual<double>(ox, 0.0) && \
            ito::areEqual<double>(sy, 1.0) && ito::areEqual<double>(sx, 1.0)))
        {
            coordinates = QString("[x: %1, y: %2]").arg(qRound(pos.x())).arg(qRound(pos.y()));
        }
        else
        {
            int x = qRound((pos.x() / sx) + ox);
            int y = qRound((pos.y() / sy) + oy);
            double temp = std::min(std::abs(sy), std::abs(sx));
            int prec = (temp >= 0.1) ? 1 : (temp >= 0.01 ? 2 : ((temp >= 0.001) ? 3 : 4));
            if (prec > 1)
            {
                coordinates = QString("Phys.: [x: %3, y: %4]\nPx: [x: %1, y: %2]").arg(x).arg(y).arg(pos.x(), 0, 'e', prec).arg(pos.y(), 0, 'e', prec);
            }
            else
            {
                coordinates = QString("Phys.: [x: %3, y: %4]\nPx: [x: %1, y: %2]").arg(x).arg(y).arg(pos.x(), 0, 'g', 5).arg(pos.y(), 0, 'g', 5);
            }
        }

        if (m_valueData->getTypeFlag() == DataObjRasterData::tRGB)
        {
            QRgb value = m_valueData->value_rgb(pos.x(), pos.y());
            if(m_showOverlayInfo && m_overlayData)
            {
                switch (m_overlayData->getTypeFlag())
                {
                    case DataObjRasterData::tRGB:
                    {
                        QRgb value2 = m_overlayData->value_rgb(pos.x(), pos.y());
                        text.sprintf("\nL1:rgb %i,%i,%i alpha %i\nL2:rgb %i,%i,%i alpha %i", qRed(value), qGreen(value), qBlue(value), qAlpha(value), qRed(value2), qGreen(value2), qBlue(value2), qAlpha(value2));
                    }
                    break;
                    case DataObjRasterData::tFloatOrComplex:
                    {
                        double value2 = m_overlayData->value(pos.x(), pos.y());
                        text.sprintf("\nL1:rgb %i,%i,%i alpha %i\nL2:%.4f", qRed(value), qGreen(value), qBlue(value), qAlpha(value), value2);
                    }
                    break;
                    case DataObjRasterData::tInteger:
                    {
                        double value2 = m_overlayData->value(pos.x(), pos.y());
                        text.sprintf("\nL1:rgb %i,%i,%i alpha %i\nL2:%d", qRed(value), qGreen(value), qBlue(value), qAlpha(value), qRound(value2));
                    }
                    break;
                }
            }
            else
            {
                text.sprintf("\nrgb %i,%i,%i alpha %i", qRed(value), qGreen(value), qBlue(value), qAlpha(value));
            }
        }
        else if (m_valueData->getTypeFlag() == DataObjRasterData::tFloatOrComplex)
        {
            double value = m_valueData->value(pos.x(), pos.y());
            if(m_showOverlayInfo && m_overlayData)
            {
                switch (m_overlayData->getTypeFlag())
                {
                    case DataObjRasterData::tRGB:
                    {
                        QRgb value2 = m_overlayData->value_rgb(pos.x(), pos.y());
                        text.sprintf("\nL1:%.4f\nL2:rgb %i,%i,%i alpha %i", value, qRed(value2), qGreen(value2), qBlue(value2), qAlpha(value2));
                    }
                    break;
                    case DataObjRasterData::tFloatOrComplex:
                    {
                        double value2 = m_overlayData->value(pos.x(), pos.y());
                        text.append(QString("\nL1:%1\nL2:%2").arg(value, 0, 'g', 4).arg(value2, 0, 'g', 4));
                    }
                    break;
                    case DataObjRasterData::tInteger:
                    {
                        double value2 = m_overlayData->value(pos.x(), pos.y());
                        text.append(QString("\nL1:%1\nL2:%2").arg(value, 0, 'g', 4).arg(qRound(value2)));
                    }
                    break;
                }
            }
            else
            {
                text.append(QString("\n%1").arg(value, 0, 'g', 4));
            }
        }
        else if (m_valueData->getTypeFlag() == DataObjRasterData::tInteger)
        {
            int value = qRound(m_valueData->value(pos.x(), pos.y()));
            if (m_showOverlayInfo && m_overlayData)
            {
                switch (m_overlayData->getTypeFlag())
                {
                case DataObjRasterData::tRGB:
                {
                    QRgb value2 = m_overlayData->value_rgb(pos.x(), pos.y());
                    text.sprintf("\nL1:%d\nL2:rgb %i,%i,%i alpha %i", value, qRed(value2), qGreen(value2), qBlue(value2), qAlpha(value2));
                }
                break;
                case DataObjRasterData::tFloatOrComplex:
                {
                    double value2 = m_overlayData->value(pos.x(), pos.y());
                    text.append(QString("\nL1:%1\nL2:%2").arg(value).arg(value2, 0, 'g', 4));
                }
                break;
                case DataObjRasterData::tInteger:
                {
                    double value2 = m_overlayData->value(pos.x(), pos.y());
                    text.append(QString("\nL1:%1\nL2:%2").arg(value).arg(qRound(value2)));
                }
                break;
                }
            }
            else
            {
                text.sprintf("\n%d", value);
            }
        }
    }
    else
    {
        text = "";
    }

    return coordinates + text;
}

//----------------------------------------------------------------------------------------------------------------------------------
void ValuePicker2D::drawTracker( QPainter *painter ) const
{
    const QRect textRect = trackerRect( painter->font() );
    if ( !textRect.isEmpty() )
    {
        const QwtText label = trackerText( trackerPosition() );
        if ( !label.isEmpty() )
        {
            painter->fillRect(textRect, m_rectFillBrush);
            label.draw( painter, textRect );
        }
    }
}

//----------------------------------------------------------------------------------------------------------------------------------
void ValuePicker2D::setBackgroundFillBrush( const QBrush &brush )
{
    if(brush != this->m_rectFillBrush)
    {
        m_rectFillBrush = brush;
        updateDisplay();
    }
}

//----------------------------------------------------------------------------------------------------------------------------------
