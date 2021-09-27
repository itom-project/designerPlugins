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

/* **********************************************************************

   The code form this file is based on / partially copied from the code
   of qwt_plot_curve.cpp / qwt_plot_curve.h in the QWT-Framework 

     * Qwt Widget Library
     * Copyright (C) 1997   Josef Wilgen
     * Copyright (C) 2002   Uwe Rathmann
     *
     * This library is free software; you can redistribute it and/or
     * modify it under the terms of the Qwt License, Version 1.0
************************************************************************ */

#include "qwtPlotCurveDataObject.h"

#include "common/sharedStructures.h"

#include "DataObject/dataobj.h"

#include "dataObjectSeriesData.h"
#include "dataObjectSeriesDataXY.h"
#include <qwt_painter.h>
#include <qwt_clipper.h>
#include <qpainter.h>
#include <qwt_curve_fitter.h>
#include <qnumeric.h>
#include <qwt_symbol.h>
#include "qwt_point_mapper.h"
//----------------------------------------------------------------------------------------------------------------------------------
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

//----------------------------------------------------------------------------------------------------------------------------------
QwtPlotCurveDataObject::QwtPlotCurveDataObject(const QString &title /*= QString::null*/) : 
    QwtPlotCurve(title), 
    m_privCurveFitter(NULL), 
    m_curveFillState(ItomQwtPlotEnums::NoCurveFill)
{
}

//----------------------------------------------------------------------------------------------------------------------------------
QwtPlotCurveDataObject::QwtPlotCurveDataObject(const QwtText &title) :
    QwtPlotCurve(title), 
    m_privCurveFitter(NULL)
{
}

//----------------------------------------------------------------------------------------------------------------------------------
void QwtPlotCurveDataObject::draw( QPainter *painter, const QwtScaleMap &xMap, const QwtScaleMap &yMap, const QRectF &canvasRect ) const
{
    DataObjectSeriesData *myData = NULL;
    myData = (DataObjectSeriesData *)data();

    if (myData && myData->isDobjInit())
    {
        myData->beginSampling(xMap, yMap, canvasRect);

        if ( testPaintAttribute(ClipPolygons) && !myData->hasAxisObj()) // xy plot loses line between points inside and outside of the displayed area
        {
            QPoint indices = myData->indexRange(xMap, false);
            if (indices.ry() >= indices.rx())
            {
                drawSeries( painter, xMap, yMap, canvasRect, indices.rx(), indices.ry() );
            }
            else
            {
                drawSeries( painter, xMap, yMap, canvasRect, 0, -1 );
            }
        }
        else
        {
            drawSeries( painter, xMap, yMap, canvasRect, 0, -1 );
        }

        myData->endSampling();
    }
}
//----------------------------------------------------------------------------------------------------------------------------------
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
        case UserCurve:
            drawCenteredSteps( painter, xMap, yMap, canvasRect, from, to );
            break;
        case Dots:
            drawDots( painter, xMap, yMap, canvasRect, from, to );
            break;
        case NoCurve:
        default:
            break;
    }
}
//----------------------------------------------------------------------------------------------------------------------------------
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

//----------------------------------------------------------------------------------------------------------------------------------
/*
this method can reduce the number of points in a polyline, if the plot area in pixels is much smaller than the number of points.
If the reduction should be applied (doAlign must be true), only every n.th point is selected.
Therefore, the reduction cannot be applied for X/Y-data, only for equidistant data!
*/
QPolygonF QwtPlotCurveDataObject::reducePoints(const QPolygonF &polyline, const QwtScaleMap &xMap, bool doAlign) const
{
    int canvas_pixel_size = qRound(xMap.pDist());
    int polyline_size = polyline.size() - 2; //start and end of polyline is always included

    if (polyline_size > 2)
    {
        qreal start = qMax(xMap.p1(), polyline.first().x());
        qreal end = qMin(xMap.p2(), polyline.last().x());
        if (qIsFinite(start) && qIsFinite(end))
        {
            canvas_pixel_size = qAbs(end - start);
        }
    }

    canvas_pixel_size = std::max(canvas_pixel_size, 100);

    //xMap.pDist() is the width (in pixel) of the area, where polyline should be displayed.
    /*If polyline contains less than 10 times this width, nothing is filtered.
    Else: It is desired to place about two points per pixel, one point contains
    the minimum value in each range around this point and one the local maximum.*/
    if (!doAlign || polyline_size < 10 * canvas_pixel_size)
    {
        return polyline;
    }

    
    int chunkSize = polyline_size / canvas_pixel_size;
    int halfChunk = chunkSize / 2;
    canvas_pixel_size *= 2;
    if (qRound(xMap.pDist()) % chunkSize != 0)
    {
        canvas_pixel_size += 2;
    }

    QPolygonF polyline2;
    polyline2.reserve(canvas_pixel_size);
    const QPointF *data = polyline.constData();

    //add startpoint
    polyline2 << data[0];
    ++data;

    QPointF pmin, pmax;
    int i = 0;
    int j;
    int chunk = chunkSize;

    while (i < polyline_size)
    {
        if (chunkSize >= polyline_size - i)
        {
            chunk = polyline_size - i;
            halfChunk = chunk / 2;
        }

        pmin = data[0];
        pmax = data[halfChunk];

        for (j = 0; j < chunk; ++j)
        {
            if (data[j].y() < pmin.ry())
            {
                pmin = data[j];
            }

            if (data[j].y() > pmax.ry())
            {
                pmax = data[j];
            }
        }

        i += chunk;
        data += chunk;
        
        if (pmin.rx() < pmax.rx())
        {
            polyline2 << pmin << pmax;
        }
        else
        {
            polyline2 << pmax << pmin;
        }
    }

    //add last point
    polyline2 << polyline.last();

    return polyline2;
}

//----------------------------------------------------------------------------------------------------------------------------------
void QwtPlotCurveDataObject::drawLines( QPainter *painter, const QwtScaleMap &xMap, const QwtScaleMap &yMap, const QRectF &canvasRect, int from, int to ) const
{
    int size = to - from + 1;
    if ( size <= 0 )
        return;

    const bool doAlign = QwtPainter::roundingAlignment( painter );

    const DataObjectSeriesData *d_objseries = dynamic_cast<const DataObjectSeriesData*>( data() );
    const DataObjectSeriesDataXY *d_objseriesXY = dynamic_cast<const DataObjectSeriesDataXY*>(d_objseries);

    if (d_objseries && 
        (d_objseries->floatingPointValues() ||
        (d_objseriesXY && d_objseriesXY->floatingPointXValues())))
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
                if(qIsFinite(y))
                {
                    //qRound(NaN) will return 0! Be careful about this (probably undefined) behaviour!
                    sample.rx() = qRound(x);
                    sample.ry() = qRound(y);
                    polyline << sample;
                }
                else
                {
                    if (!d_objseriesXY)
                    {
                        polyline = reducePoints(polyline, xMap, doAlign);
                    }
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
                    if (!d_objseriesXY)
                    {
                        polyline = reducePoints(polyline, xMap, doAlign);
                    }
                    drawPolyline(painter,polyline,xMap,yMap,canvasRect);
                    polyline.clear();
                }
            }
        }

        if(polyline.size() > 0)
        {
            if (!d_objseriesXY)
            {
                polyline = reducePoints(polyline, xMap, doAlign);
            }
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
                y = qRound( y );
            }

            points[i - from].rx() = x;
            points[i - from].ry() = y;
        }

        if (!d_objseriesXY)
        {
            polyline = reducePoints(polyline, xMap, doAlign);
        }
        drawPolyline(painter, polyline, xMap, yMap, canvasRect);
    }
}
//----------------------------------------------------------------------------------------------------------------------------------
void QwtPlotCurveDataObject::drawPolyline(QPainter *painter, QPolygonF &polyline, const QwtScaleMap &xMap, const QwtScaleMap &yMap, const QRectF &canvasRect) const
{
    if ( testCurveAttribute(Fitted) && m_privCurveFitter ) //( d_data->attributes & Fitted ) && d_data->curveFitter )
            polyline = m_privCurveFitter->fitCurve( polyline );

    if ( testPaintAttribute(ClipPolygons) ) //d_data->paintAttributes & ClipPolygons )
    {
        qreal pw = qMax( qreal( 1.0 ), painter->pen().widthF());
        QwtClipper::clipPolygonF(canvasRect.adjusted(-pw, -pw, pw, pw), polyline, false );

        QwtPainter::drawPolyline( painter, polyline );
    }
    else
    {
        QwtPainter::drawPolyline( painter, polyline );
    }

    if ( m_privBrush.style() != Qt::NoBrush )
        fillCurve( painter, xMap, yMap, canvasRect, polyline );
}
//----------------------------------------------------------------------------------------------------------------------------------
void QwtPlotCurveDataObject::drawSymbols( QPainter *painter, const QwtSymbol &symbol, const QwtScaleMap &xMap, const QwtScaleMap &yMap, const QRectF &canvasRect, int from, int to ) const
{
    int size = to - from + 1;
    if ( size <= 0 )
        return;

    const bool doAlign = QwtPainter::roundingAlignment( painter );

    const DataObjectSeriesData *d_objseries = dynamic_cast<const DataObjectSeriesData*>( data() );
    const DataObjectSeriesDataXY *d_objseriesXY = dynamic_cast<const DataObjectSeriesDataXY*>(d_objseries);

    if (d_objseries &&
        (d_objseries->floatingPointValues() ||
        (d_objseriesXY && d_objseriesXY->floatingPointXValues())))
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
            }
            else
            {
                if(qIsFinite(y))
                {
                    sample.rx() = x;
                    sample.ry() = y;
                    polyline << sample;
                }
            }
        }

        if(polyline.size() > 0)
        {
            if (!d_objseriesXY)
            {
                polyline = reducePoints(polyline, xMap, doAlign);
            }
            symbol.drawSymbols( painter, polyline );
            polyline.clear();
        }
    }
    else
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
                y = qRound( y );
            }

            points[i - from].rx() = x;
            points[i - from].ry() = y;
        }

        if (!d_objseriesXY)
        {
            polyline = reducePoints(polyline, xMap, doAlign);
        }

        symbol.drawSymbols( painter, polyline );
    }
}
//----------------------------------------------------------------------------------------------------------------------------------
void QwtPlotCurveDataObject::drawSticks( QPainter *painter,  const QwtScaleMap &xMap, const QwtScaleMap &yMap, const QRectF &canvasRect, int from, int to ) const
{
    painter->save();
    painter->setRenderHint( QPainter::Antialiasing, false );

    const bool doAlign = QwtPainter::roundingAlignment( painter );

    double x0 = xMap.transform( baseline());
    double y0 = yMap.transform( baseline());
    if ( doAlign )
    {
        x0 = qRound( x0 );
        y0 = qRound( y0 );
    }

    const Qt::Orientation o = orientation();

    const DataObjectSeriesData *d_objseries = static_cast<const DataObjectSeriesData*>( data() );

    if(d_objseries->floatingPointValues())
    {
        for ( int i = from; i <= to; i++ )
        {
            const QPointF sample = d_objseries->sample( i );
            double xi = xMap.transform( sample.x() );

            double yi = sample.y();
            if(!qIsFinite(yi))
            {
                continue;
            }

            yi = yMap.transform( yi );

            if ( doAlign )
            {
                xi = qRound( xi );
                yi = qRound( yi );
            }

            if ( o == Qt::Horizontal )
                QwtPainter::drawLine( painter, x0, yi, xi, yi );
            else
                QwtPainter::drawLine( painter, xi, y0, xi, yi );
        }
    }
    else
    {
        for ( int i = from; i <= to; i++ )
        {
            const QPointF sample = d_objseries->sample( i );
            double xi = xMap.transform( sample.x() );
            double yi = yMap.transform( sample.y() );
            if ( doAlign )
            {
                xi = qRound( xi );
                yi = qRound( yi );
            }

            if ( o == Qt::Horizontal )
                QwtPainter::drawLine( painter, x0, yi, xi, yi );
            else
                QwtPainter::drawLine( painter, xi, y0, xi, yi );
        }
    }

    painter->restore();
}
//----------------------------------------------------------------------------------------------------------------------------------
void QwtPlotCurveDataObject::drawDots( QPainter *painter, const QwtScaleMap &xMap, const QwtScaleMap &yMap, const QRectF &canvasRect, int from, int to ) const
{
    int size = to - from + 1;
    if ( size <= 0 )
        return;

    const QColor color = painter->pen().color();

    if ( painter->pen().style() == Qt::NoPen || color.alpha() == 0 )
    {
        return;
    }

    const bool doFill =  (m_privBrush.style() != Qt::NoBrush ) && ( m_privBrush.color().alpha() > 0 );
    const bool doAlign = QwtPainter::roundingAlignment( painter );

    const DataObjectSeriesData *d_objseries = static_cast<const DataObjectSeriesData*>( data() );
/*
    if ( testPaintAttribute(FilterPoints) )
    {
        if ( ( color.alpha() == 255 )
            && !( painter->renderHints() & QPainter::Antialiasing ) )
        {
            mapper.setFlag( QwtPointMapper::WeedOutPoints, true );
        }
    }
*/

    if ( testPaintAttribute(ImageBuffer) )
    {
        QwtPointMapper mapper;
        mapper.setBoundingRect( canvasRect );
        mapper.setFlag( QwtPointMapper::RoundPoints, doAlign );

        const QImage image = mapper.toImage( xMap, yMap,
            data(), from, to, this->pen(), 
            painter->testRenderHint( QPainter::Antialiasing ),
            renderThreadCount() );

        painter->drawImage( canvasRect.toAlignedRect(), image );
    }
    else if ( testPaintAttribute( MinimizeMemory ) )
    {
        const QwtSeriesData<QPointF> *series = data();

        for ( int i = from; i <= to; i++ )
        {
            const QPointF sample = d_objseries->sample( i );

            double xi = xMap.transform( sample.x() );
            double yi = sample.y();

            if(!qIsFinite(yi))
            {
                continue;
            }

            yi = yMap.transform( yi );
            
            if ( doAlign )
            {
                xi = qRound( xi );
                yi = qRound( yi );
            }

            QwtPainter::drawPoint( painter, QPointF( xi, yi ) );
        }
    }
    else
    {
        if(d_objseries->floatingPointValues())
        {        
            QPolygonF points;
            points.reserve(size);
            QPointF sample;
            bool newPol = false;
            for ( int i = from; i <= to; i++ )
            {
                QPointF sample = d_objseries->sample( i );

                double xi = xMap.transform( sample.x() );
                double yi = sample.y();

                if(!qIsFinite(yi))
                {
                    newPol = true;
                }

                yi = yMap.transform( yi );
            
                if ( doAlign )
                {
                    xi = qRound( xi );
                    yi = qRound( yi );
                }

                sample.rx() = xi;
                sample.ry() = yi;

                if(newPol)
                {
                    QwtPainter::drawPoints( painter, points );
                    if ( doFill )
                        fillCurve( painter, xMap, yMap, canvasRect, points );
                    points.clear();
                    newPol = false;
                }
                else
                {
                    points << sample;
                }
            }

            if(points.size() > 0)
            {
                QwtPainter::drawPoints( painter, points );
                if ( doFill )
                    fillCurve( painter, xMap, yMap, canvasRect, points );  
            }
        }
        else
        {
            QPolygonF points(size);
            QPointF sample;

            for ( int i = from; i <= to; i++ )
            {
                const QPointF sample = d_objseries->sample( i );

                double xi = xMap.transform( sample.x() );
                double yi = sample.y();

                if(!qIsFinite(yi))
                {
                    continue;
                }

                yi = yMap.transform( yi );
            
                if ( doAlign )
                {
                    xi = qRound( xi );
                    yi = qRound( yi );
                }
                points[i - from].rx() = xi;
                points[i - from].ry() = yi;
            }
            QwtPainter::drawPoints( painter, points );
            fillCurve( painter, xMap, yMap, canvasRect, points );
        }
    }
}
//----------------------------------------------------------------------------------------------------------------------------------
void QwtPlotCurveDataObject::drawSteps( QPainter *painter, const QwtScaleMap &xMap, const QwtScaleMap &yMap, const QRectF &canvasRect, int from, int to ) const
{
    const bool doAlign = QwtPainter::roundingAlignment( painter );
    int size = to - from + 1;
    if ( size <= 1 )
        return;

    bool inverted = orientation() == Qt::Vertical;
    if ( testCurveAttribute(Inverted) )
        inverted = !inverted;

    const DataObjectSeriesData *d_objseries = static_cast<const DataObjectSeriesData*>( data() );

    if(d_objseries->floatingPointValues())
    {
        QPolygonF polyline;
        polyline.reserve(2 * size - 1);
        QPointF sample;
        QPointF sample2;

        int i;
        for ( i = from; i <= to; i++)
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
                    

                    if ( polyline.size() > 0 )
                    {
                        const QPointF &p0 = polyline.last();

                        if ( inverted )
                        {
                            sample2.rx() = p0.x();
                            sample2.ry() = y;
                        }
                        else
                        {
                            sample2.rx() = x;
                            sample2.ry() = p0.y();
                        }
                        polyline << sample2;
                    }
                    polyline << sample;

                }
                else if(polyline.size() > 0)
                {
                    if ( this->testPaintAttribute(ClipPolygons) )
                    {
                        QwtClipper::clipPolygonF( canvasRect, polyline, false );

                        QwtPainter::drawPolyline( painter, polyline );
                    }
                    else
                    {
                        QwtPainter::drawPolyline( painter, polyline );
                    }

                    if ( m_privBrush.style() != Qt::NoBrush )
                        fillCurve( painter, xMap, yMap, canvasRect, polyline );

                    polyline.clear();
                }
            }
            else
            {
                if(qIsFinite(y))
                {
                    sample.rx() = x;
                    sample.ry() = y;

                    if ( polyline.size() > 0 )
                    {
                        const QPointF &p0 = polyline.last();

                        if ( inverted )
                        {
                            sample2.rx() = p0.x();
                            sample2.ry() = y;
                        }
                        else
                        {
                            sample2.rx() = x;
                            sample2.ry() = p0.y();
                        }
                        polyline << sample2;
                    }

                    polyline << sample;
                }
                else if(polyline.size() > 0)
                {
                    if ( this->testPaintAttribute(ClipPolygons) )
                    {
                        QwtClipper::clipPolygonF( canvasRect, polyline, false );

                        QwtPainter::drawPolyline( painter, polyline );
                    }
                    else
                    {
                        QwtPainter::drawPolyline( painter, polyline );
                    }

                    if ( m_privBrush.style() != Qt::NoBrush )
                        fillCurve( painter, xMap, yMap, canvasRect, polyline );

                    polyline.clear();
                }
            }
        }

        if(polyline.size() > 0)
        {


            if ( this->testPaintAttribute(ClipPolygons) )
            {
                QwtClipper::clipPolygonF( canvasRect, polyline, false );

                QwtPainter::drawPolyline( painter, polyline );
            }
            else
            {
                QwtPainter::drawPolyline( painter, polyline );
            }

            if ( m_privBrush.style() != Qt::NoBrush )
                fillCurve( painter, xMap, yMap, canvasRect, polyline );

            polyline.clear();
        }
    }
    else
    {
        QPolygonF polyline( 2 * size - 1 );

        QPointF *points = polyline.data();
        int i, ip;
        for ( i = from, ip = 0; i <= to; i++, ip += 2 )
        {
            const QPointF sample = d_objseries->sample( i );

            double x = xMap.transform( sample.x() );
            double y = yMap.transform( sample.y() );
            if ( doAlign )
            {
                x = qRound( x );
                y = qRound( y );
            }

            if ( ip > 0 )
            {
                const QPointF &p0 = points[ip - 2];
                QPointF &p = points[ip - 1];

                if ( inverted )
                {
                    p.rx() = p0.x();
                    p.ry() = y;
                }
                else
                {
                    p.rx() = x;
                    p.ry() = p0.y();
                }
            }

            points[ip].rx() = x;
            points[ip].ry() = y;
        }

        if ( this->testPaintAttribute(ClipPolygons) )
        {
            QwtClipper::clipPolygonF( canvasRect, polyline, false );
            QwtPainter::drawPolyline( painter, polyline );
        }
        else
        {
            QwtPainter::drawPolyline( painter, polyline );
        }

        if ( m_privBrush.style() != Qt::NoBrush )
        {
            fillCurve( painter, xMap, yMap, canvasRect, polyline );
        }
    }
}
//----------------------------------------------------------------------------------------------------------------------------------
void QwtPlotCurveDataObject::drawCenteredSteps( QPainter *painter, const QwtScaleMap &xMap, const QwtScaleMap &yMap, const QRectF &canvasRect, int from, int to ) const
{
    const bool doAlign = QwtPainter::roundingAlignment( painter );
    int size = to - from + 1;
    if ( size <= 1 )
        return;

    const DataObjectSeriesData *d_objseries = static_cast<const DataObjectSeriesData*>( data() );

    if(d_objseries->floatingPointValues())
    {
        QPolygonF polyline;
        polyline.reserve(2 * size );
        QPointF sample;
        QPointF sample2;
        QPointF sample3;
        int i;
        for ( i = from; i <= to; i++)
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
                    

                    if ( polyline.size() > 0 )
                    {
                        const QPointF &p0 = polyline.last();

                        sample2.rx() = p0.x();
                        sample2.ry() = y;

                        polyline << sample2;
                    }
                    else
                    {
                        polyline << sample;
                    }

                    if(i < to)
                    {
                        sample3 = d_objseries->sample( i + 1 );

                        if(qIsFinite(sample3.y()))
                        {
                            x = xMap.transform( sample3.x() );
                            x = qRound( x );
                            sample.rx() = (sample.rx() + x) / 2;
                        }
                    }
                    polyline << sample;

                }
                else if(polyline.size() > 0)
                {
                    if ( this->testPaintAttribute(ClipPolygons) )
                    {
                        QwtClipper::clipPolygonF( canvasRect, polyline, false );

                        QwtPainter::drawPolyline( painter, polyline );
                    }
                    else
                    {
                        QwtPainter::drawPolyline( painter, polyline );
                    }

                    if ( m_privBrush.style() != Qt::NoBrush )
                        fillCurve( painter, xMap, yMap, canvasRect, polyline );

                    polyline.clear();
                }
            }
            else
            {
                if(qIsFinite(y))
                {
                    sample.rx() = x;
                    sample.ry() = y;

                    if ( polyline.size() > 0 )
                    {
                        const QPointF &p0 = polyline.last();

                        sample2.rx() = p0.x();
                        sample2.ry() = y;

                        polyline << sample2;
                    }
                    else
                    {
                        polyline << sample;
                    }

                    if(i < to)
                    {
                        sample3 = d_objseries->sample( i + 1 );

                        if(qIsFinite(sample3.y()))
                        {
                            x = xMap.transform( sample3.x() );
                            sample.rx() = (sample.rx() + x) / 2;
                        }
                    }

                    polyline << sample;
                }
                else if(polyline.size() > 0)
                {
                    if ( this->testPaintAttribute(ClipPolygons) )
                    {
                        QwtClipper::clipPolygonF( canvasRect, polyline, false );

                        QwtPainter::drawPolyline( painter, polyline );
                    }
                    else
                    {
                        QwtPainter::drawPolyline( painter, polyline );
                    }

                    if ( m_privBrush.style() != Qt::NoBrush )
                        fillCurve( painter, xMap, yMap, canvasRect, polyline );

                    polyline.clear();
                }
            }
        }

        if(polyline.size() > 0)
        {


            if ( this->testPaintAttribute(ClipPolygons) )
            {
                QwtClipper::clipPolygonF( canvasRect, polyline, false );

                QwtPainter::drawPolyline( painter, polyline );
            }
            else
            {
                QwtPainter::drawPolyline( painter, polyline );
            }

            if ( m_privBrush.style() != Qt::NoBrush )
                fillCurve( painter, xMap, yMap, canvasRect, polyline );

            polyline.clear();
        }
    }
    else
    {
        QPolygonF polyline( 2 * size);

        QPointF *points = polyline.data();
        int i, ip;
        for ( i = from, ip = 1; i <= to; i++, ip += 2 )
        {
            const QPointF sample = d_objseries->sample( i );

            double x = xMap.transform( sample.x() );
            double y = yMap.transform( sample.y() );
            if ( doAlign )
            {
                y = qRound( y );
            }

            if ( ip > 1 )
            {
                const QPointF &p0 = points[ip - 2];
                QPointF &p = points[ip - 1];

                p.rx() = p0.x();
                p.ry() = y;
            }
            else
            {
                QPointF &p = points[ip - 1];
                p.rx() = doAlign ? qRound( x ) : x;
                p.ry() = y;
            }

            if(i < to)
            {
                const QPointF sample2 = d_objseries->sample( i + 1 );
                x += xMap.transform( sample2.x() );;
                x /= 2.0;
                x = doAlign ? qRound( x ) : x;
            }
            else
            {
                x = doAlign ? qRound( x ) : x;
            }

            points[ip].rx() = x;
            points[ip].ry() = y;
        }

        if ( this->testPaintAttribute(ClipPolygons) )
        {
            QwtClipper::clipPolygonF( canvasRect, polyline, false );
            QwtPainter::drawPolyline( painter, polyline );
        }
        else
        {
            QwtPainter::drawPolyline( painter, polyline );
        }

        if ( m_privBrush.style() != Qt::NoBrush )
        {
            fillCurve( painter, xMap, yMap, canvasRect, polyline );
        }
    }
}
//----------------------------------------------------------------------------------------------------------------------------------
/*!
  Fill the area between the curve and the baseline with
  the curve brush

  \param painter Painter
  \param xMap x map
  \param yMap y map
  \param canvasRect Contents rectangle of the canvas
  \param polygon Polygon - will be modified !

  \sa setBrush(), setBaseline(), setStyle()
*/
void QwtPlotCurveDataObject::fillCurve( QPainter *painter,
    const QwtScaleMap &xMap, const QwtScaleMap &yMap,
    const QRectF &canvasRect, QPolygonF &polygon ) const
{
    if ( m_privBrush == Qt::NoBrush )
        return;

    closePolyline( painter, xMap, yMap, polygon );
    if ( polygon.count() <= 2 ) // a line can't be filled
        return;

    QBrush brush = m_privBrush;
    if ( !brush.color().isValid() )
        brush.setColor( pen().color() );

    if ( testPaintAttribute(ClipPolygons) )
        QwtClipper::clipPolygonF( canvasRect, polygon, true );

    painter->save();

    painter->setPen( Qt::NoPen );
    painter->setBrush( brush );

    QwtPainter::drawPolygon( painter, polygon );

    painter->restore();
}
//----------------------------------------------------------------------------------------------------------------------------------
/*!
  \brief Complete a polygon to be a closed polygon including the 
         area between the original polygon and the baseline.

  \param painter Painter
  \param xMap X map
  \param yMap Y map
  \param polygon Polygon to be completed
*/
void QwtPlotCurveDataObject::closePolyline( QPainter *painter, const QwtScaleMap &xMap, const QwtScaleMap &yMap, QPolygonF &polygon ) const
{
    if ( polygon.size() < 2 )
        return;

    const bool doAlign = QwtPainter::roundingAlignment( painter );
    const DataObjectSeriesData *d_objseries = static_cast<const DataObjectSeriesData*>( data() );

    double baseline = this->baseline();
    
    if(ItomQwtPlotEnums::FillFromTop == m_curveFillState)
    {
        if ( orientation() == Qt::Vertical ) baseline = yMap.invTransform(yMap.p2());
        else baseline = xMap.invTransform(xMap.p2());
    }
    else if (ItomQwtPlotEnums::FillFromBottom == m_curveFillState)
    {
        if ( orientation() == Qt::Vertical ) baseline = yMap.invTransform(yMap.p1());
        else baseline = xMap.invTransform(xMap.p1());
    }

    if ( orientation() == Qt::Vertical )
    {
        if ( yMap.transformation() )
            baseline = yMap.transformation()->bounded( baseline );

        double refY = yMap.transform( baseline );
        if ( doAlign )
            refY = qRound( refY );

        polygon += QPointF( polygon.last().x(), refY );
        polygon += QPointF( polygon.first().x(), refY );
    }
    else
    {
        if ( xMap.transformation() )
            baseline = xMap.transformation()->bounded( baseline );

        double refX = xMap.transform( baseline );
        if ( doAlign )
            refX = qRound( refX );

        polygon += QPointF( refX, polygon.last().y() );
        polygon += QPointF( refX, polygon.first().y() );
    }
}
//----------------------------------------------------------------------------------------------------------------------------------