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

#include "itomPlotMagnifier.h"

#include <qevent.h>
#include <qwt_plot.h>
#include <qwt_scale_div.h>
#include <QtCore/qmath.h>
#include "itomPlotZoomer.h"

/*!
   Constructor
   \param parent Widget to be magnified
*/
ItomPlotMagnifier::ItomPlotMagnifier( QWidget *parent, ItomPlotZoomer *zoomer /*= NULL*/):
    QwtPlotMagnifier( parent )
{
    m_zoomer = QPointer<ItomPlotZoomer>(zoomer);
}

//! Destructor
ItomPlotMagnifier::~ItomPlotMagnifier()
{
}

//-------------------------------------------------------------------------------------------------
void ItomPlotMagnifier::widgetMouseMoveEvent( QMouseEvent *mouseEvent )
{
    m_mousePos = mouseEvent->pos();

    QwtPlotMagnifier::widgetMouseMoveEvent(mouseEvent);
}


//-------------------------------------------------------------------------------------------------
void ItomPlotMagnifier::rescale(double factor)
{
    QwtPlot* plt = plot();
    if ( plt == NULL )
        return;

    factor = qAbs( factor );
    if ( factor == 1.0 || factor == 0.0 )
        return;

    bool doReplot = false;

    const bool autoReplot = plt->autoReplot();
    plt->setAutoReplot( false );
    
    int xAxis = -1;
    int yAxis = -1;
    QRectF zoomRect;

    //if a zoomer is given, and both axes covered by the zoomer are part of this magnifier
    //then the magnifier uses the zoomer to execute the zoom. Else there might be a 'fight'
    //between zoomer and magnifier in case of a fixed aspect ratio. This is since the
    //size of the canvas might change while magnifying due to varying sizes of the axes
    //descriptions and ticks. Then the resizeEvent of the zoomer is called, that finally
    //fights against this magnifier.
    if (m_zoomer.data())
    {
        xAxis = m_zoomer->xAxis();
        yAxis = m_zoomer->yAxis();

        if ((xAxis >= 0 && !isAxisEnabled(xAxis)) || (yAxis >= 0 && !isAxisEnabled(yAxis)))
        {
            xAxis = -1;
            yAxis = -1;
        }
    }


    for ( int axisId = 0; axisId < QwtPlot::axisCnt; axisId++ )
    {
        const QwtScaleDiv &scaleDiv = plt->axisScaleDiv( axisId );
        if ( isAxisEnabled( axisId ) )
        {
            const double center = scaleDiv.lowerBound() + scaleDiv.range() / 2;
            const double width_2 = scaleDiv.range() / 2 * factor;

            if (xAxis == axisId)
            {
                zoomRect.setLeft(center - width_2);
                zoomRect.setRight(center + width_2);
            }
            else if (yAxis == axisId)
            {
                zoomRect.setTop(center - width_2);
                zoomRect.setBottom(center + width_2);
            }
            else
            {
                plt->setAxisScale( axisId, center - width_2, center + width_2 );
                doReplot = true;
            }
        }
    }

    if (m_zoomer.data())
    {
        m_zoomer->zoom(zoomRect);
    }

    plt->setAutoReplot( autoReplot );

    if ( doReplot )
        plt->replot();
}

//-------------------------------------------------------------------------------------------------
void ItomPlotMagnifier::rescale(double factor, const QPointF &mouseCoords)
{
    QwtPlot* plt = plot();
    if ( plt == NULL )
        return;

    factor = qAbs( factor );
    if ( factor == 1.0 || factor == 0.0 )
        return;

    bool doReplot = false;

    const bool autoReplot = plt->autoReplot();
    plt->setAutoReplot( false );

    //if a zoomer is given, and both axes covered by the zoomer are part of this magnifier
    //then the magnifier uses the zoomer to execute the zoom. Else there might be a 'fight'
    //between zoomer and magnifier in case of a fixed aspect ratio. This is since the
    //size of the canvas might change while magnifying due to varying sizes of the axes
    //descriptions and ticks. Then the resizeEvent of the zoomer is called, that finally
    //fights against this magnifier.
    int xAxis = -1;
    int yAxis = -1;
    QRectF zoomRect;

    if (m_zoomer.data())
    {
        xAxis = m_zoomer->xAxis();
        yAxis = m_zoomer->yAxis();

        if ((xAxis >= 0 && !isAxisEnabled(xAxis)) || (yAxis >= 0 && !isAxisEnabled(yAxis)))
        {
            xAxis = -1;
            yAxis = -1;
        }
    }

    for ( int axisId = 0; axisId < QwtPlot::axisCnt; axisId++ )
    {
        const QwtScaleDiv &scaleDiv = plt->axisScaleDiv( axisId );
        if ( isAxisEnabled( axisId ) )
        {
            if (axisId == QwtPlot::xBottom || axisId == QwtPlot::yLeft)
            {
                const double width = scaleDiv.range() * factor;
                const double scaleXorY = plt->invTransform(axisId, (axisId == QwtPlot::xBottom) ? mouseCoords.x() : mouseCoords.y());

                if (scaleDiv.contains(scaleXorY))
                {
                    const double factor = (scaleXorY - scaleDiv.lowerBound()) / scaleDiv.range();

                    if (xAxis == axisId)
                    {
                        zoomRect.setLeft(scaleXorY - width * factor);
                        zoomRect.setRight(scaleXorY + width * (1-factor));
                    }
                    else if (yAxis == axisId)
                    {
                        zoomRect.setTop(scaleXorY - width * factor);
                        zoomRect.setBottom(scaleXorY + width * (1-factor));
                    }
                    else
                    {
                        plt->setAxisScale( axisId, scaleXorY - width * factor, scaleXorY + width * (1-factor) );
                        doReplot = true;
                    }
                }
                else
                {
                    const double center = scaleDiv.lowerBound() + scaleDiv.range() / 2;
                    
                    if (xAxis == axisId)
                    {
                        zoomRect.setLeft(center - width / 2);
                        zoomRect.setRight(center + width / 2);
                    }
                    else if (yAxis == axisId)
                    {
                        zoomRect.setTop(center - width / 2);
                        zoomRect.setBottom(center + width / 2);
                    }
                    else
                    {
                        plt->setAxisScale( axisId, center - width / 2, center + width / 2 );
                        doReplot = true;
                    }
                }
            }
            else
            {
                const double width_2 = scaleDiv.range() * factor / 2;
                const double center = scaleDiv.lowerBound() + scaleDiv.range() / 2;

                if (xAxis == axisId)
                {
                    zoomRect.setLeft(center - width_2);
                    zoomRect.setRight(center + width_2);
                }
                else if (yAxis == axisId)
                {
                    zoomRect.setTop(center - width_2);
                    zoomRect.setBottom(center + width_2);
                }
                else
                {
                    plt->setAxisScale( axisId, center - width_2, center + width_2 );
                    doReplot = true;
                }
            }
        }
    }

    if (m_zoomer.data())
    {
        m_zoomer->zoom(zoomRect);
    }

    plt->setAutoReplot( autoReplot );

    if ( doReplot )
        plt->replot();
}

//-------------------------------------------------------------------------------------------------
void ItomPlotMagnifier::widgetWheelEvent( QWheelEvent *wheelEvent )
{
    if ( wheelEvent->modifiers() != wheelModifiers() )
    {
        return;
    }

    if ( wheelFactor() != 0.0 )
    {
        /*
            A positive delta indicates that the wheel was
            rotated forwards away from the user; a negative
            value indicates that the wheel was rotated
            backwards toward the user.
            Most mouse types work in steps of 15 degrees,
            in which case the delta value is a multiple
            of 120 (== 15 * 8).
         */
        double f = qPow( wheelFactor(), 
            qAbs( wheelEvent->delta() / 120.0 ) );

        if ( wheelEvent->delta() > 0 )
            f = 1 / f;

        if (m_mousePos.isNull())
        {
            rescale( f );
        }
        else
        {
            rescale(f, m_mousePos);
        }
    }
}