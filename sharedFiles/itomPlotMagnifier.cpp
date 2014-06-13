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

#include "itomPlotMagnifier.h"

#include <qevent.h>
#include <qwt_plot.h>
#include <qwt_scale_div.h>
#include <QtCore/qmath.h>

/*!
   Constructor
   \param parent Widget to be magnified
*/
ItomPlotMagnifier::ItomPlotMagnifier( QWidget *parent ):
    QwtPlotMagnifier( parent )
{
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

    for ( int axisId = 0; axisId < QwtPlot::axisCnt; axisId++ )
    {
        const QwtScaleDiv &scaleDiv = plt->axisScaleDiv( axisId );
        if ( isAxisEnabled( axisId ) )
        {
            const double center = scaleDiv.lowerBound() + scaleDiv.range() / 2;
            const double width_2 = scaleDiv.range() / 2 * factor;

            plt->setAxisScale( axisId, center - width_2, center + width_2 );
            doReplot = true;
        }
    }

    plt->setAutoReplot( autoReplot );

    if ( doReplot )
        plt->replot();
}

//-------------------------------------------------------------------------------------------------
void ItomPlotMagnifier::rescale(double factor, QPointF mouseCoords)
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

    for ( int axisId = 0; axisId < QwtPlot::axisCnt; axisId++ )
    {
        const QwtScaleDiv &scaleDiv = plt->axisScaleDiv( axisId );
        if ( isAxisEnabled( axisId ) )
        {
            if (axisId == QwtPlot::xBottom || axisId == QwtPlot::yLeft)
            {
                const double width = scaleDiv.range() * factor;
                const double scaleXorY = plt->invTransform(axisId, (axisId == QwtPlot::xBottom) ? mouseCoords.rx() : mouseCoords.ry());

                if (scaleDiv.contains(scaleXorY))
                {
                    const double factor = (scaleXorY - scaleDiv.lowerBound()) / scaleDiv.range();
                    plt->setAxisScale( axisId, scaleXorY - width * factor, scaleXorY + width * (1-factor) );
                }
                else
                {
                    const double center = scaleDiv.lowerBound() + scaleDiv.range() / 2;
                    plt->setAxisScale( axisId, center - width / 2, center + width / 2 );
                }

                doReplot = true;
            }
            else
            {
                const double width_2 = scaleDiv.range() * factor / 2;
                const double center = scaleDiv.lowerBound() + scaleDiv.range() / 2;

                plt->setAxisScale( axisId, center - width_2, center + width_2 );
                doReplot = true;
            }
        }
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