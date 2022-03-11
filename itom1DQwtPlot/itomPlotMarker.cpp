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

#include "itomPlotMarker.h"

#include <qpainter.h>
#include <qwt_symbol.h>
#include <qwt_text.h>

//----------------------------------------------------------------------------------------------------------------------------------
ItomPlotMarker::ItomPlotMarker(bool labelState, ItomQwtPlotEnums::PlotPickerType type, Qt::Alignment align, Qt::Orientation orient) : 
    QwtPlotMarker(), 
    m_plotType(type), 
    m_labelState(labelState)
{ 
    setLabelAlignment(align);
    setLabelOrientation(orient);
    setLabelEnabled(m_labelState);
    setPlotType(type);
}
//----------------------------------------------------------------------------------------------------------------------------------
void ItomPlotMarker::setPlotType(const ItomQwtPlotEnums::PlotPickerType value)
{
    m_plotType = value;
    switch(value)
    {
        case ItomQwtPlotEnums::DefaultMarker:
            setLineStyle(QwtPlotMarker::NoLine);
            break;
        case ItomQwtPlotEnums::RangeMarker:
        case ItomQwtPlotEnums::AxisRangeMarker:
            setLineStyle(QwtPlotMarker::VLine);
            setLinePen( Qt::gray, 1.0, Qt::DashLine );
            break;
        case ItomQwtPlotEnums::ValueRangeMarker:
            setLineStyle(QwtPlotMarker::HLine);
            setLinePen( Qt::gray, 1.0, Qt::DashLine );
            break;
    }
}
//----------------------------------------------------------------------------------------------------------------------------------
void ItomPlotMarker::updateLabelValue()
{
    if(m_labelState)
    {
        QwtText tmp(QString("%1\n%2").arg(this->xValue(),0,'g',4).arg(this->yValue(),0,'g',4 )  );
        tmp.setBackgroundBrush(QBrush(QColor(255, 255, 255, 180), Qt::SolidPattern));
        setLabel(tmp);
    }
    else
    {
        QwtText tmp;
        tmp.setBackgroundBrush(QBrush(QColor(255, 255, 255, 0), Qt::SolidPattern));
        setLabel(tmp);
    }
}
//----------------------------------------------------------------------------------------------------------------------------------
// taken form QWT: qwt_plot_marker.h
/*!
  Align and draw the text label of the marker

  \param painter Painter
  \param canvasRect Contents rectangle of the canvas in painter coordinates
  \param pos Position of the marker, translated into widget coordinates

  \sa drawLabel(), QwtSymbol::drawSymbol()
*/
void ItomPlotMarker::drawLabel( QPainter *painter,
    const QRectF &canvasRect, const QPointF &pos ) const
{
    if ( label().isEmpty() )
        return;

    Qt::Alignment align = labelAlignment();
    QPointF alignPos = pos;

    QSizeF symbolOff( 0, 0 );

    switch ( lineStyle() )
    {
        case QwtPlotMarker::VLine:
        {
            // In VLine-style the y-position is pointless and
            // the alignment flags are relative to the canvas

            if ( align & Qt::AlignTop )
            {
                alignPos.setY( canvasRect.top() );
                align &= ~Qt::AlignTop;
                align |= Qt::AlignBottom;
            }
            else if ( align & Qt::AlignBottom )
            {
                // In HLine-style the x-position is pointless and
                // the alignment flags are relative to the canvas

                alignPos.setY( canvasRect.bottom() - 1 );
                align &= ~Qt::AlignBottom;
                align |= Qt::AlignTop;
            }
            else
            {
                alignPos.setY( canvasRect.center().y() );
            }
            break;
        }
        case QwtPlotMarker::HLine:
        {
            if ( align & Qt::AlignLeft )
            {
                alignPos.setX( canvasRect.left() );
                align &= ~Qt::AlignLeft;
                align |= Qt::AlignRight;
            }
            else if ( align & Qt::AlignRight )
            {
                alignPos.setX( canvasRect.right() - 1 );
                align &= ~Qt::AlignRight;
                align |= Qt::AlignLeft;
            }
            else
            {
                alignPos.setX( canvasRect.center().x() );
            }
            break;
        }
        default:
        {
            if ( symbol() &&
                ( symbol()->style() != QwtSymbol::NoSymbol ) )
            {
                symbolOff = symbol()->size() + QSizeF( 1, 1 );
                symbolOff /= 2;
            }
        }
    }

    qreal pw2 = linePen().widthF() / 2.0;
    if ( pw2 == 0.0 )
        pw2 = 0.5;

    const int spacing = this->spacing();

    const qreal xOff = qMax( pw2, symbolOff.width() );
    const qreal yOff = qMax( pw2, symbolOff.height() );

    const QSizeF textSize = label().textSize( painter->font() );

    bool retry = true;
    int cnt = 0;
    QPointF alignPosTmp;

    while(retry && cnt < 6)
    {
        alignPosTmp = alignPos;
        retry = false;
        if ( align & Qt::AlignLeft )
        {
            alignPosTmp.rx() -= xOff + spacing;
            if ( labelOrientation() == Qt::Vertical )
                alignPosTmp.rx() -= textSize.height();
            else
                alignPosTmp.rx() -= textSize.width();
        }
        else if ( align & Qt::AlignRight )
        {
            alignPosTmp.rx() += xOff + spacing;
        }
        else
        {
            if ( labelOrientation() == Qt::Vertical )
                alignPosTmp.rx() -= textSize.height() / 2;
            else
                alignPosTmp.rx() -= textSize.width() / 2;
        }

        if ( align & Qt::AlignTop )
        {
            alignPosTmp.ry() -= yOff + spacing;
            if ( labelOrientation() != Qt::Vertical )
                alignPosTmp.ry() -= textSize.height();
        }
        else if ( align & Qt::AlignBottom )
        {
            alignPosTmp.ry() += yOff + spacing;
            if ( labelOrientation() == Qt::Vertical )
                alignPosTmp.ry() += textSize.width();
        }
        else
        {
            if ( labelOrientation() == Qt::Vertical )
                alignPosTmp.ry() += textSize.width() / 2;
            else
                alignPosTmp.ry() -= textSize.height() / 2;
        }

        float x0 = 0.0;
        float y0 = 0.0;
        float x1 = 0.0;
        float y1 = 0.0;

        if ( labelOrientation() == Qt::Vertical )
        {
            x0 = alignPosTmp.ry();
            x1 = alignPosTmp.ry() + textSize.width();
            y0 = alignPosTmp.rx();
            y1 = alignPosTmp.rx() + textSize.height();
        }
        else
        {
            x0 = alignPosTmp.rx();
            x1 = alignPosTmp.rx() + textSize.width();
            y0 = alignPosTmp.ry();
            y1 = alignPosTmp.ry() + textSize.height();
        }

        if(cnt < 3 && (y0 < canvasRect.top() || y1 > canvasRect.bottom()))
        {
            if(align& Qt::AlignBottom)
            {
                align &= ~Qt::AlignBottom;
                align |= Qt::AlignVCenter;
            }
            else if(align & Qt::AlignVCenter)
            {
                align &= ~Qt::AlignVCenter;
                align |= Qt::AlignTop;
            }
            else
            {
                align &= ~Qt::AlignTop;
                align |= Qt::AlignBottom;
            }
            retry = true;
        }

        if(!retry && (x0 < canvasRect.left() || x1 > canvasRect.right()))
        {
            if(align & Qt::AlignLeft)
            {
                align &= ~Qt::AlignLeft;
                align |= Qt::AlignVCenter;
            }
            else if(align & Qt::AlignHCenter)
            {
                align &= ~Qt::AlignHCenter;
                align |= Qt::AlignRight;
            }
            else
            {
                align &= ~Qt::AlignRight;
                align |= Qt::AlignLeft;
            }
            retry = true;
        }
        cnt++;
    }
    

    alignPos = alignPosTmp;

    painter->translate( alignPos.x(), alignPos.y() );
    if ( labelOrientation() == Qt::Vertical )
        painter->rotate( -90.0 );

    const QRectF textRect( 0, 0, textSize.width(), textSize.height() );
    label().draw( painter, textRect );
}