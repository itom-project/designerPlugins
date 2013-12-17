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

#include "qwtPlotCurveDataObject.h"

#include "common/sharedStructures.h"

#include "DataObject/dataobj.h"

#include "dataObjectSeriesData.h"
#include <qwt_painter.h>
#include <qwt_clipper.h>
#include <qpainter.h>
#include <qwt_curve_fitter.h>
#include <qnumeric.h>
#include <qwt_symbol.h>

static int verifyRange( int size, int &i1, int &i2 )
{
    if ( size < 1 )
        return 0;

    i1 = qBound( 0, i1, size - 1 );
    i2 = qBound( 0, i2, size - 1 );

    if ( i1 > i2 )
        qSwap( i1, i2 );

    return ( i2 - i1 + 1 );
}

void QwtPlotCurveDataObject::draw( QPainter *painter, const QwtScaleMap &xMap, const QwtScaleMap &yMap, const QRectF &canvasRect ) const
{
    DataObjectSeriesData *myData = NULL;
    myData = (DataObjectSeriesData *)data();

    if (myData && myData->isDobjInit())
    {
        myData->beginSampling(xMap, yMap, canvasRect);
        drawSeries( painter, xMap, yMap, canvasRect, 0, -1 );
        myData->endSampling();
    }
}

void QwtPlotCurveDataObject::drawCurve( QPainter *painter, int style, const QwtScaleMap &xMap, const QwtScaleMap &yMap, const QRectF &canvasRect, int from, int to ) const
{
    switch ( style )
    {
        case Lines:
            if ( testCurveAttribute( Fitted ) )
            {
                // we always need the complete
                // curve for fitting
                from = 0;
                to = (int)dataSize() - 1;
            }
            drawLines( painter, xMap, yMap, canvasRect, from, to );
            break;
        case Sticks:
            drawSticks( painter, xMap, yMap, canvasRect, from, to );
            break;
        case Steps:
            drawSteps( painter, xMap, yMap, canvasRect, from, to );
            break;
        case Dots:
            drawDots( painter, xMap, yMap, canvasRect, from, to );
            break;
        case NoCurve:
        default:
            break;
    }
}

void QwtPlotCurveDataObject::drawSeries( QPainter *painter, const QwtScaleMap &xMap, const QwtScaleMap &yMap, const QRectF &canvasRect, int from, int to ) const
{
    if ( !painter || dataSize() <= 0 )
        return;

    if ( to < 0 )
        to = (int)dataSize() - 1;

    if ( verifyRange( (int)dataSize(), from, to ) > 0 )
    {
        painter->save();
        painter->setPen( pen() );

        /*
          Qt 4.0.0 is slow when drawing lines, but it's even
          slower when the painter has a brush. So we don't
          set the brush before we really need it.
         */

        drawCurve( painter, style(), xMap, yMap, canvasRect, from, to );
        painter->restore();

        if ( symbol() &&
            ( symbol()->style() != QwtSymbol::NoSymbol ) )
        {
            painter->save();
            drawSymbols( painter, *symbol(),
                xMap, yMap, canvasRect, from, to );
            painter->restore();
        }
    }
}

void QwtPlotCurveDataObject::drawLines( QPainter *painter, const QwtScaleMap &xMap, const QwtScaleMap &yMap, const QRectF &canvasRect, int from, int to ) const
{
    int size = to - from + 1;
    if ( size <= 0 )
        return;

    const bool doAlign = QwtPainter::roundingAlignment( painter );

    const DataObjectSeriesData *d_objseries = static_cast<const DataObjectSeriesData*>( data() );

    if(d_objseries->floatingPointValues())
    {
        QPolygonF polyline;
        polyline.reserve(size);
        QPointF sample;

        for ( int i = from; i <= to; i++ )
        {
            sample = d_objseries->sample( i );

            double x = xMap.transform( sample.x() );
            double y = yMap.transform( sample.y() );
            if ( doAlign )
            {
                x = qRound( x );
                if(qIsFinite(y))
                {
                    y = qRound( y );
                    sample.rx() = x;
                    sample.ry() = y;
                    polyline << sample;
                }
                else
                {
                    drawPolyline(painter,polyline,xMap,yMap,canvasRect);
                    polyline.clear();
                }
            }
            else
            {
                if(qIsFinite(y))
                {
                    sample.rx() = x;
                    sample.ry() = y;
                    polyline << sample;
                }
                else
                {
                    drawPolyline(painter,polyline,xMap,yMap,canvasRect);
                    polyline.clear();
                }
            }
        }

        if(polyline.size() > 0)
        {
            drawPolyline(painter,polyline,xMap,yMap,canvasRect);
            polyline.clear();
        }
    }
    else //no NaN may appear
    {
        QPolygonF polyline( size );

        QPointF *points = polyline.data();
        for ( int i = from; i <= to; i++ )
        {
            const QPointF sample = d_objseries->sample( i );

            double x = xMap.transform( sample.x() );
            double y = yMap.transform( sample.y() );
            if ( doAlign )
            {
                x = qRound( x );
                //if(qIsFinite(y))
                //{
                y = qRound( y );
                //}
            }

            points[i - from].rx() = x;
            points[i - from].ry() = y;
        }

        drawPolyline(painter, polyline, xMap, yMap, canvasRect);
    }
}

void QwtPlotCurveDataObject::drawPolyline(QPainter *painter, QPolygonF &polyline, const QwtScaleMap &xMap, const QwtScaleMap &yMap, const QRectF &canvasRect) const
{
    if ( testCurveAttribute(Fitted) && m_privCurveFitter ) //( d_data->attributes & Fitted ) && d_data->curveFitter )
            polyline = m_privCurveFitter->fitCurve( polyline );

    if ( testPaintAttribute(ClipPolygons) ) //d_data->paintAttributes & ClipPolygons )
    {
        qreal pw = qMax( qreal( 1.0 ), painter->pen().widthF());
        const QPolygonF clipped = QwtClipper::clipPolygonF(canvasRect.adjusted(-pw, -pw, pw, pw), polyline, false );

        QwtPainter::drawPolyline( painter, clipped );
    }
    else
    {
        QwtPainter::drawPolyline( painter, polyline );
    }

    if ( m_privBrush.style() != Qt::NoBrush )
        fillCurve( painter, xMap, yMap, canvasRect, polyline );
}