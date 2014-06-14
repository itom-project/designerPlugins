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

#include "itomPlotZoomer.h"

#include <qevent.h>
#include <qwt_plot.h>
#include <qwt_scale_div.h>
#include <qwt_scale_draw.h>
#include <QtCore/qmath.h>

//---------------------------------------------------------------------------
ItomPlotZoomer::ItomPlotZoomer( QWidget *parent, bool doReplot /*= true*/ ) :
    QwtPlotZoomer(parent, doReplot),
    m_fixedAspectRatio(false),
    m_aspectRatioChanged(false)
{
}

//---------------------------------------------------------------------------
ItomPlotZoomer::ItomPlotZoomer( int xAxis, int yAxis,
                        QWidget *parent, bool doReplot /*= true*/ ) :
    QwtPlotZoomer(xAxis, yAxis, parent, doReplot),
    m_fixedAspectRatio(false),
    m_aspectRatioChanged(false)
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

        if (plot())
        {
            QWidget *w = plot()->canvas();
            if ( w )
            {
                if ( fixed )
                    w->installEventFilter( this );
                else
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
    if (!m_fixedAspectRatio && !m_aspectRatioChanged)
    {
        return QwtPlotZoomer::rescale();
    }
    else
    {
        QwtPlot *plt = plot();
        if ( !plt )
            return;

        //if the zoomer is disabled and this method is called, this might come frome an explicit call to zoom... or due to a
        //resize event with fixed aspect ratio. In the latter case, the zoom should not be done with respect to the current zoomRect
        //but to the scaleRect, that is the currently visible region.
        const QRectF &rect = (isEnabled() || !resizeEvent) ? zoomRect() : scaleRect();
        if ( !isEnabled() || rect != scaleRect() || m_aspectRatioChanged )
        {
            const bool doReplot = plt->autoReplot();
            plt->setAutoReplot( false );

            //rect is in scale coordinates, the square however should be guaranteed in screen coordinates (pixels)
            double x1 = rect.left();
            double x2 = rect.right();
            double y1 = rect.top();
            double y2 = rect.bottom();

            if (m_fixedAspectRatio)
            {

                //get effective area of the current plot (without margins, axes,...)
                //int left, top, right, bottom;
                //plt->canvas()->getContentsMargins( &left, &top, &right, &bottom );
                //plt->canvas()->setStyleSheet("background-color: #ff00cc");
                ////qDebug() << plt->canvas()->contentsMargins() << plt->contentsMargins();
                //const QSize size = plt->canvas()->contentsRect().size(); // - QSize(left+right,top+bottom);

                //qDebug() << plt->size() << plt->canvas()->size();

                //more exact: take real pixel lenghts of axisScaleDraws are real area
                const QSize size(plt->axisScaleDraw(xAxis())->length(), plt->axisScaleDraw(yAxis())->length());

                //make square
                double lx = qAbs(x2 - x1);
                double ly = qAbs(y2 - y1);
                double sx = lx / size.width();
                double sy = ly / size.height();

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

            if ( !plt->axisScaleDiv( xAxis() ).isIncreasing() )
                qSwap( x1, x2 );

            plt->setAxisScale( xAxis(), x1, x2 );

            
            if ( !plt->axisScaleDiv( yAxis() ).isIncreasing() )
                qSwap( y1, y2 );

            plt->setAxisScale( yAxis(), y1, y2 );

            plt->setAutoReplot( doReplot );

            plt->replot();

            /*QRectF pixels(plt->transform(xAxis(), x1), plt->transform(yAxis(), y1), 0 , 0);
            pixels.setRight(plt->transform(xAxis(), x2));
            pixels.setBottom(plt->transform(yAxis(), y2));
            pixels = pixels.normalized();
            double lx = qAbs(x2 - x1);
            double ly = qAbs(y2 - y1);
            qDebug() << (lx/pixels.width()) << (ly/pixels.height());*/

            m_aspectRatioChanged = false;
        }
    }
}


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