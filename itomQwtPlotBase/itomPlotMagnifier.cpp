/* ********************************************************************
   itom measurement system
   URL: http://www.uni-stuttgart.de/ito
   Copyright (C) 2022, Institut fuer Technische Optik (ITO),
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
#include <qwt_scale_engine.h>
#include <qwt_transform.h>
#include <QtCore/qmath.h>
#include "itomPlotZoomer.h"

#include <qdebug.h>

/*!
   Constructor
   \param parent Widget to be magnified
*/
ItomPlotMagnifier::ItomPlotMagnifier( QWidget *parent, ItomPlotZoomer *zoomer /*= NULL*/):
    QwtPlotMagnifier( parent ),
    m_pActiveDisabledAxesSet( NULL )
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
void ItomPlotMagnifier::setAxesDisabledOnAdditionalModifier(const QList<int> &axes, Qt::KeyboardModifiers modifiers /*= Qt::NoModifier*/)
{
    if (m_disabledAxesOnSpecialModifiers.contains(wheelModifiers() | modifiers) && axes.size() == 0)
    {
        m_disabledAxesOnSpecialModifiers.remove(wheelModifiers() | modifiers);
    }
    else
    {
        m_disabledAxesOnSpecialModifiers[wheelModifiers() | modifiers] = axes;
    }
}

//-------------------------------------------------------------------------------------------------
bool ItomPlotMagnifier::isAxisEnabledSpecial(int axis)
{
    if (!m_pActiveDisabledAxesSet)
    {
        return isAxisEnabled(axis);
    }
    else
    {
        //if fixed aspect ratio: no single axis magnification is possible
        return isAxisEnabled(axis) && (!m_pActiveDisabledAxesSet->contains(axis) || (m_zoomer.data() && m_zoomer->fixedAspectRatio()));
    }
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

    int xAxisZoomer = -1;
    int yAxisZoomer = -1;
    QRectF zoomRect;

    //if a zoomer is given, and both axes covered by the zoomer are part of this magnifier
    //then the magnifier uses the zoomer to execute the zoom. Else there might be a 'fight'
    //between zoomer and magnifier in case of a fixed aspect ratio. This is since the
    //size of the canvas might change while magnifying due to varying sizes of the axes
    //descriptions and ticks. Then the resizeEvent of the zoomer is called, that finally
    //fights against this magnifier.
    if (m_zoomer.data())
    {
        xAxisZoomer = m_zoomer->xAxis();
        yAxisZoomer = m_zoomer->yAxis();

        //zoomer and magnifier control different axes...
        if ((xAxisZoomer >= 0 && !isAxisEnabled(xAxisZoomer)) && (yAxisZoomer >= 0 && !isAxisEnabled(yAxisZoomer)))
        {
            xAxisZoomer = -1;
            yAxisZoomer = -1;
        }
    }


    for ( int axisId = 0; axisId < QwtPlot::axisCnt; axisId++ )
    {
        const QwtTransform *trafo = plt->axisScaleEngine( axisId )->transformation();
        const QwtScaleDiv &scaleDiv = plt->axisScaleDiv( axisId );
        double leftTop;
        double rightBottom;

        if (!trafo)
        {
            leftTop = scaleDiv.lowerBound() + (1 - factor) * scaleDiv.range() / 2.0;
            rightBottom = scaleDiv.lowerBound() + (1 + factor) * scaleDiv.range() / 2.0;
        }
        else
        {
            double bord1 = trafo->transform(scaleDiv.lowerBound());
            double bord2 = trafo->transform(scaleDiv.upperBound());
            double center = 0.5 * (bord1 + bord2);
            double width_2 = (bord2 - bord1) / 2.0;
            bord1 = center - factor * width_2;
            bord2 = center + factor * width_2;
            leftTop = trafo->invTransform(bord1);
            rightBottom = trafo->invTransform(bord2);
        }

        if (xAxisZoomer == axisId)
        {
            if (isAxisEnabledSpecial(axisId))
            {
                zoomRect.setLeft(leftTop);
                zoomRect.setRight(rightBottom);
            }
            else
            {
                //let this axis unchanged
                zoomRect.setLeft(scaleDiv.lowerBound());
                zoomRect.setRight(scaleDiv.upperBound());
            }
        }
        else if (yAxisZoomer == axisId)
        {
            if (isAxisEnabledSpecial(axisId))
            {
                zoomRect.setTop(leftTop);
                zoomRect.setBottom(rightBottom);
            }
            else
            {
                //let this axis unchanged
                zoomRect.setTop(scaleDiv.lowerBound());
                zoomRect.setBottom(scaleDiv.upperBound());
            }
        }
        else if (isAxisEnabledSpecial(axisId))
        {
            plt->setAxisScale(axisId, leftTop, rightBottom);
            doReplot = true;
        }
    }

    if (m_zoomer.data() && zoomRect.isValid())
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
    int xAxisZoomer = -1;
    int yAxisZoomer = -1;
    QRectF zoomRect;

    if (m_zoomer.data())
    {
        xAxisZoomer = m_zoomer->xAxis();
        yAxisZoomer = m_zoomer->yAxis();

        //zoomer and magnifier control different axes...
        if ((xAxisZoomer >= 0 && !isAxisEnabled(xAxisZoomer)) && (yAxisZoomer >= 0 && !isAxisEnabled(yAxisZoomer)))
        {
            xAxisZoomer = -1;
            yAxisZoomer = -1;
        }
    }

    for ( int axisId = 0; axisId < QwtPlot::axisCnt; axisId++ )
    {
        const QwtTransform *trafo = plt->axisScaleEngine( axisId )->transformation();
        const QwtScaleDiv &scaleDiv = plt->axisScaleDiv( axisId );

        if (axisId == QwtPlot::xBottom || axisId == QwtPlot::yLeft)
        {
            const double scaleXorY = plt->invTransform(axisId, (axisId == QwtPlot::xBottom) ? mouseCoords.x() : mouseCoords.y());

            double border1;
            double border2;

            if (!trafo)
            {
                border1 = scaleXorY * (1 - factor) + factor * scaleDiv.lowerBound();
                border2 = scaleXorY * (1 - factor) + factor * scaleDiv.upperBound();
            }
            else
            {
                border1 = trafo->invTransform(trafo->transform(scaleXorY) * (1 - factor) + factor * trafo->transform(scaleDiv.lowerBound()));
                border2 = trafo->invTransform(trafo->transform(scaleXorY) * (1 - factor) + factor * trafo->transform(scaleDiv.upperBound()));
            }

            if (scaleDiv.contains(scaleXorY))
            {
                if (xAxisZoomer == axisId)
                {
                    if (isAxisEnabledSpecial(axisId))
                    {
                        zoomRect.setLeft(border1);
                        zoomRect.setRight(border2);
                    }
                    else
                    {
                        //let this axis unchanged
                        zoomRect.setLeft(scaleDiv.lowerBound());
                        zoomRect.setRight(scaleDiv.upperBound());
                    }
                }
                else if (yAxisZoomer == axisId)
                {
                    if (isAxisEnabledSpecial(axisId))
                    {
                        zoomRect.setTop(border1);
                        zoomRect.setBottom(border2);
                    }
                    else
                    {
                        //let this axis unchanged
                        zoomRect.setTop(scaleDiv.lowerBound());
                        zoomRect.setBottom(scaleDiv.upperBound());
                    }
                }
                else if (isAxisEnabledSpecial(axisId))
                {
                    plt->setAxisScale( axisId, border1, border2);
                    doReplot = true;
                }
            }
            else
            {
                double border1, border2;
                if (!trafo)
                {
                    border1 = scaleDiv.lowerBound() + (1 - factor) * (scaleDiv.upperBound() - scaleDiv.lowerBound()) / 2;
                    border2 = scaleDiv.lowerBound() + (1 + factor) * (scaleDiv.upperBound() - scaleDiv.lowerBound()) / 2;
                }
                else
                {
                    border1 = trafo->invTransform(trafo->transform(scaleDiv.lowerBound()) + \
                        (1 - factor) * (trafo->transform(scaleDiv.upperBound()) - trafo->transform(scaleDiv.lowerBound())) / 2);
                    border2 = trafo->invTransform(trafo->transform(scaleDiv.lowerBound()) + \
                        (1 + factor) * (trafo->transform(scaleDiv.upperBound()) - trafo->transform(scaleDiv.lowerBound())) / 2);
                }

                if (xAxisZoomer == axisId)
                {
                    if (isAxisEnabledSpecial(axisId))
                    {
                        zoomRect.setLeft(border1);
                        zoomRect.setRight(border2);
                    }
                    else
                    {
                        //let this axis unchanged
                        zoomRect.setLeft(scaleDiv.lowerBound());
                        zoomRect.setRight(scaleDiv.upperBound());
                    }
                }
                else if (yAxisZoomer == axisId)
                {
                    if (isAxisEnabledSpecial(axisId))
                    {
                        zoomRect.setTop(border1);
                        zoomRect.setBottom(border2);
                    }
                    else
                    {
                        //let this axis unchanged
                        zoomRect.setTop(scaleDiv.lowerBound());
                        zoomRect.setBottom(scaleDiv.upperBound());
                    }
                }
                else if (isAxisEnabledSpecial(axisId))
                {
                    plt->setAxisScale( axisId, border1, border2 );
                    doReplot = true;
                }
            }
        }
        else
        {
            double border1, border2;
            if (!trafo)
            {
                border1 = scaleDiv.lowerBound() + (1 - factor) * (scaleDiv.upperBound() - scaleDiv.lowerBound()) / 2;
                border2 = scaleDiv.lowerBound() + (1 + factor) * (scaleDiv.upperBound() - scaleDiv.lowerBound()) / 2;
            }
            else
            {
                border1 = trafo->invTransform(trafo->transform(scaleDiv.lowerBound()) + \
                    (1 - factor) * (trafo->transform(scaleDiv.upperBound()) - trafo->transform(scaleDiv.lowerBound())) / 2);
                border2 = trafo->invTransform(trafo->transform(scaleDiv.lowerBound()) + \
                    (1 + factor) * (trafo->transform(scaleDiv.upperBound()) - trafo->transform(scaleDiv.lowerBound())) / 2);
            }

            if (xAxisZoomer == axisId)
            {
                if (isAxisEnabledSpecial(axisId))
                {
                    zoomRect.setLeft(border1);
                    zoomRect.setRight(border2);
                }
                else
                {
                    //let this axis unchanged
                    zoomRect.setLeft(scaleDiv.lowerBound());
                    zoomRect.setRight(scaleDiv.upperBound());
                }
            }
            else if (yAxisZoomer == axisId)
            {
                if (isAxisEnabledSpecial(axisId))
                {
                    zoomRect.setTop(border1);
                    zoomRect.setBottom(border2);
                }
                else
                {
                    //let this axis unchanged
                    zoomRect.setTop(scaleDiv.lowerBound());
                    zoomRect.setBottom(scaleDiv.upperBound());
                }
            }
            else if (isAxisEnabledSpecial(axisId))
            {
                plt->setAxisScale( axisId, border1, border2);
                doReplot = true;
            }
        }
    }

    zoomRect = zoomRect.normalized();
    if (m_zoomer.data() && zoomRect.isValid())
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
    const Qt::KeyboardModifiers &modifiers = wheelEvent->modifiers();
    if ( modifiers != wheelModifiers() && !m_disabledAxesOnSpecialModifiers.contains(modifiers) )
    {
        return;
    }

    if ( wheelFactor() != 0.0 )
    {
        if (modifiers != wheelModifiers())
        {
            m_pActiveDisabledAxesSet = &(m_disabledAxesOnSpecialModifiers[modifiers]);
        }

        /*
            A positive delta indicates that the wheel was
            rotated forwards away from the user; a negative
            value indicates that the wheel was rotated
            backwards toward the user.
            Most mouse types work in steps of 15 degrees,
            in which case the delta value is a multiple
            of 120 (== 15 * 8).
         */
        QPoint angleDeltaPt = wheelEvent->angleDelta();

        double angleDelta;

        if (std::abs(angleDeltaPt.x()) > std::abs(angleDeltaPt.y()))
        {
            angleDelta = angleDeltaPt.x();
        }
        else
        {
            angleDelta = angleDeltaPt.y();
        }

        if (wheelEvent->inverted())
        {
            angleDelta *= -1.0;
        }

        double f = qPow(wheelFactor(),
            qAbs(angleDelta / 120.0 ) );

        if (angleDelta < 0)
        {
            f = 1 / f;
        }

        if (m_mousePos.isNull())
        {
            rescale( f );
        }
        else
        {
            rescale(f, m_mousePos);
        }

        //reset temporarily disabled axes
        m_pActiveDisabledAxesSet = NULL;
    }
}
