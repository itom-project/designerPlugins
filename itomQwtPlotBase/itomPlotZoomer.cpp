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

#include "itomPlotZoomer.h"

#include <qevent.h>
#include <qdebug.h>
#include <qstack.h>
#include <qrect.h>
#include <qwt_plot.h>
#include <qwt_scale_div.h>
#include <qwt_scale_draw.h>
#include <qwt_scale_engine.h>
#include <QtCore/qmath.h>

#define MAXRESCALESTACKDEPTH 20 //rescale might sometimes be called in a recursive call. This might crash the application. Therefore the recursion is limited to this number!

//---------------------------------------------------------------------------
ItomPlotZoomer::ItomPlotZoomer( QWidget *parent, bool doReplot /*= true*/ ) :
    QwtPlotZoomer(parent, doReplot),
    m_fixedAspectRatio(false),
    m_aspectRatioChanged(false),
    m_invertedAxes(-1),
    m_nrOfRescaleCalls(0)
{
}

//---------------------------------------------------------------------------
ItomPlotZoomer::ItomPlotZoomer( int xAxis, int yAxis,
                        QWidget *parent, bool doReplot /*= true*/ ) :
    QwtPlotZoomer(xAxis, yAxis, parent, doReplot),
    m_fixedAspectRatio(false),
    m_aspectRatioChanged(false),
    m_invertedAxes(-1),
    m_nrOfRescaleCalls(0)
{
}

//---------------------------------------------------------------------------
ItomPlotZoomer::~ItomPlotZoomer()
{
}

//---------------------------------------------------------------------------
void ItomPlotZoomer::setFixedAspectRatio(bool fixed)
{
    if (m_fixedAspectRatio != fixed)
    {
        m_aspectRatioChanged = true;

        QWidget *w = parentWidget();
        if ( w && !isEnabled())
        {
            if (fixed)
            {
                w->installEventFilter( this );
            }
            else
            {
                w->removeEventFilter( this );
            }
        }

        m_fixedAspectRatio = fixed;
        rescale();
    }
    else
    {
        m_fixedAspectRatio = fixed;
    }
}

//---------------------------------------------------------------------------
bool ItomPlotZoomer::accept( QPolygon &pa ) const
{
    //pa is in screen coordinates (pixels)
    if (!m_fixedAspectRatio)
    {
        return QwtPlotZoomer::accept(pa);
    }
    else
    {
        if ( pa.count() < 2 )
            return false;

        QRect rect = QRect( pa[0], pa[int( pa.count() ) - 1] );
        rect = rect.normalized();

        const int minSize = 2;
        if ( rect.width() < minSize && rect.height() < minSize )
            return false;

        //try to make rect a square (the longer side length is taken)
        const int minZoomSize = qMax(11, qMax(rect.width(), rect.height()));

        const QPoint center = rect.center();

        rect.setSize( rect.size().expandedTo( QSize( minZoomSize, minZoomSize ) ) );
        rect.moveCenter( center );

        pa.resize( 2 );
        pa[0] = rect.topLeft();
        pa[1] = rect.bottomRight();

        return true;
    }
}

//---------------------------------------------------------------------------
void ItomPlotZoomer::rescale(bool resizeEvent)
{
    if (m_nrOfRescaleCalls == MAXRESCALESTACKDEPTH)
    {
        qDebug() << "ItomPlotZoomer::rescale: maximum number of recursive calls reached.";
        return;
    }

    m_nrOfRescaleCalls++;

    QwtPlot *plt = plot();
    if ( !plt )
        return;

    int xAxisId = xAxis();
    int yAxisId = yAxis();

    if (!m_fixedAspectRatio && !m_aspectRatioChanged)
    {
        int invertedAxes = plt->axisScaleEngine(xAxisId)->testAttribute(QwtScaleEngine::Inverted) ? 1 : 0;
        invertedAxes += plt->axisScaleEngine(yAxisId)->testAttribute(QwtScaleEngine::Inverted) ? 2 : 0;

        const QRectF &rect = zoomRect();
        if ( rect != scaleRect() || (invertedAxes != m_invertedAxes))
        {
            m_invertedAxes = invertedAxes;

            const bool doReplot = plt->autoReplot();
            plt->setAutoReplot( false );

            double x1 = rect.left();
            double x2 = rect.right();
            double y1 = rect.top();
            double y2 = rect.bottom();

            if (invertedAxes & 1)
            {
                if (x1 < x2) qSwap(x1,x2);
            }
            else
            {
                if (x2 < x1) qSwap(x1,x2);
            }

            if (invertedAxes & 2)
            {
                if (y1 < y2) qSwap(y1,y2);
            }
            else
            {
                if (y2 < y1) qSwap(y1,y2);
            }

            plt->setAxisScale( xAxisId, x1, x2 );
            plt->setAxisScale( yAxisId, y1, y2 );

            plt->setAutoReplot( doReplot );
        }

        plt->replot();
    }
    else
    {
        double x1, x2, y1, y2;
        bool rescale = false;

        const bool doReplot = plt->autoReplot();
        plt->setAutoReplot( false );

        if (resizeEvent) //the currently visible part of the contents rect should be still visible (if possible)
        {
            QRectF visibleContentsRect = zoomRect().intersected(scaleRect());

            //rect is in scale coordinates, the square however should be guaranteed in screen coordinates (pixels)
            x1 = visibleContentsRect.left();
            x2 = visibleContentsRect.right();
            y1 = visibleContentsRect.top();
            y2 = visibleContentsRect.bottom();

            if (m_fixedAspectRatio)
            {

                //get effective area of the current plot (without margins, axes,...)
                //int left, top, right, bottom;
                //plt->canvas()->getContentsMargins( &left, &top, &right, &bottom );
                //plt->canvas()->setStyleSheet("background-color: #ff00cc");
                ////qDebug() << plt->canvas()->contentsMargins() << plt->contentsMargins();
                //const QSize size = plt->canvas()->contentsRect().size(); // - QSize(left+right,top+bottom);

                //qDebug() << plt->size() << plt->canvas()->size();

                //more exact: take real pixel lenghts of axisScaleDraws are real area (only if axis is available, else take size of canvas)
                const int canvas_width = plt->axisEnabled(xAxisId) ? plt->axisScaleDraw(xAxisId)->length() : plt->canvas()->width();
                const int canvas_height = plt->axisEnabled(yAxisId) ? plt->axisScaleDraw(yAxisId)->length() : plt->canvas()->height();

                //make square
                double lx = qAbs(x2 - x1);
                double ly = qAbs(y2 - y1);
                double sx = lx / canvas_width;
                double sy = ly / canvas_height;

                if (sy > sx)
                {
                    double factor = sy / sx;
                    double center = (x1+x2)/2.0;
                    //increase x1,x2
                    x1 = center - (center - x1)*factor;
                    x2 = center + (x2 - center)*factor;
                }
                else if (sy < sx)
                {
                    double factor = sx / sy;
                    double center = (y1+y2)/2.0;
                    //increase y1,y2
                    y1 = center - (center - y1)*factor;
                    y2 = center + (y2 - center)*factor;
                }
            }

            rescale = true;
        }
        else
        {
            //if the zoomer is disabled and this method is called, this might come frome an explicit call to zoom... or due to a
            //resize event with fixed aspect ratio. In the latter case, the zoom should not be done with respect to the current zoomRect
            //but to the scaleRect, that is the currently visible region.
            if ( !isEnabled() || zoomRect() != scaleRect() || m_aspectRatioChanged )
            {
                //rect is in scale coordinates, the square however should be guaranteed in screen coordinates (pixels)
                x1 = zoomRect().left();
                x2 = zoomRect().right();
                y1 = zoomRect().top();
                y2 = zoomRect().bottom();

                if (m_fixedAspectRatio)
                {

                    //get effective area of the current plot (without margins, axes,...)
                    //int left, top, right, bottom;
                    //plt->canvas()->getContentsMargins( &left, &top, &right, &bottom );
                    //plt->canvas()->setStyleSheet("background-color: #ff00cc");
                    ////qDebug() << plt->canvas()->contentsMargins() << plt->contentsMargins();
                    //const QSize size = plt->canvas()->contentsRect().size(); // - QSize(left+right,top+bottom);

                    //qDebug() << plt->size() << plt->canvas()->size();

                    //more exact: take real pixel lenghts of axisScaleDraws are real area (only if axis is available, else take size of canvas)
                    const int canvas_width = plt->axisEnabled(xAxisId) ? plt->axisScaleDraw(xAxisId)->length() : plt->canvas()->width();
                    const int canvas_height = plt->axisEnabled(yAxisId) ? plt->axisScaleDraw(yAxisId)->length() : plt->canvas()->height();

                    //make square
                    double lx = qAbs(x2 - x1);
                    double ly = qAbs(y2 - y1);
                    double sx = lx / canvas_width;
                    double sy = ly / canvas_height;

                    if (sy > sx)
                    {
                        double factor = sy / sx;
                        double center = (x1+x2)/2.0;
                        //increase x1,x2
                        x1 = center - (center - x1)*factor;
                        x2 = center + (x2 - center)*factor;
                    }
                    else if (sy < sx)
                    {
                        double factor = sx / sy;
                        double center = (y1+y2)/2.0;
                        //increase y1,y2
                        y1 = center - (center - y1)*factor;
                        y2 = center + (y2 - center)*factor;
                    }
                }

                rescale = true;
            }
        }

        if (rescale)
        {
            if (plt->axisScaleEngine(xAxisId)->testAttribute(QwtScaleEngine::Inverted))
            {
                if (x1 < x2) qSwap(x1,x2);
            }
            else
            {
                if (x2 < x1) qSwap(x1,x2);
            }

            if (plt->axisScaleEngine(yAxisId)->testAttribute(QwtScaleEngine::Inverted))
            {
                if (y1 < y2) qSwap(y1,y2);
            }
            else
            {
                if (y2 < y1) qSwap(y1,y2);
            }

            plt->setAxisScale( yAxisId, y1, y2 );
            plt->setAxisScale( xAxisId, x1, x2 );

            m_aspectRatioChanged = false;
        }

        plt->setAutoReplot( doReplot );

        plt->replot();
    }

    m_nrOfRescaleCalls--;
}

//--------------------------------------------------------------------------------------
//!  Event filter for the plot canvas
bool ItomPlotZoomer::eventFilter( QObject *object, QEvent *event )
{
    if ( object && object == parentWidget() )
    {
        switch ( event->type() )
        {
            case QEvent::Resize:
            {
                rescale(true);
                break;
            }
            case QEvent::PolishRequest:
            {
                rescale();
                break;
            }
            default:;
        }

        if (isEnabled())
        {
            return QwtPlotZoomer::eventFilter(object,event);
        }
    }

    return false;
}

//--------------------------------------------------------------------------------------
void ItomPlotZoomer::setEnabled(bool enabled)
{
    QwtPicker::setEnabled(enabled);

    if (!enabled && m_fixedAspectRatio == true)
    {
        QWidget *w = parentWidget();
        if ( w )
        {
            w->installEventFilter( this ); //re-install the event filter that has been removed by setEnabled of QwtPicker (if fixed aspect ratio is on)
        }
    }

}

//--------------------------------------------------------------------------------------
void ItomPlotZoomer::canvasPanned(int /*dx*/, int /*dy*/) //connect this to panned signal of panner to synchronize both
{
    QStack<QRectF> currentStack = zoomStack();

    if (maxStackDepth() < 0 || currentStack.count() < maxStackDepth())
    {
        currentStack.append(scaleRect());
        setZoomStack(currentStack);
    }
    else if (currentStack.count() >= 1)
    {
        //remove 2nd element (1st is base - do not change this) from stack and append this one to the end
        //since the max stack depth is reached.
        currentStack.remove(1);
        currentStack.append(scaleRect());
        setZoomStack(currentStack);
    }
}
