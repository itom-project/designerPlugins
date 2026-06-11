/* ********************************************************************
   itom measurement system
   URL: http://www.uni-stuttgart.de/ito
   Copyright (C) 2018, Institut für Technische Optik (ITO),
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

#include "userInteractionPlotPicker.h"

#include <qpainterpath.h>

#include <qwt_picker_machine.h>
#include <qwt_painter.h>
#include <qwt_text.h>

//----------------------------------------------------------------------------------------------------------------------------------
UserInteractionPlotPicker::UserInteractionPlotPicker(QWidget *canvas) : QwtPlotPicker(canvas), m_keepAspectRatio(false)
{
    init();
}

//----------------------------------------------------------------------------------------------------------------------------------
UserInteractionPlotPicker::~UserInteractionPlotPicker()
{
}

//----------------------------------------------------------------------------------------------------------------------------------
UserInteractionPlotPicker::UserInteractionPlotPicker(int xAxis, int yAxis, QWidget *widget) : QwtPlotPicker(xAxis,yAxis,widget), m_keepAspectRatio(false)
{
    init();
}

//----------------------------------------------------------------------------------------------------------------------------------
UserInteractionPlotPicker::UserInteractionPlotPicker(int xAxis, int yAxis,
    RubberBand rubberBand, DisplayMode trackerMode, QWidget *widget) :
    QwtPlotPicker(xAxis,yAxis,rubberBand,trackerMode,widget),
    m_keepAspectRatio(false)
{
    init();
}

//----------------------------------------------------------------------------------------------------------------------------------
void UserInteractionPlotPicker::reset()
{
    if (isEnabled())
    {
        // if the Abort-Key is pressed, we have to clear the selection
        m_selection.clear();

        if (!isActive())
        {
            //at the beginning no point is clicked, nevertheless the Abort-Key should abort the selection and
            // send activated(false) such that itom is able to continue
            emit activated(false);
        }
    }

    QwtPlotPicker::reset();
}

//----------------------------------------------------------------------------------------------------------------------------------
void UserInteractionPlotPicker::init()
{
    connect(this, SIGNAL(activated(bool)), this, SLOT(selectionActivated(bool)));
    connect(this, SIGNAL(appended(QPointF)), this, SLOT(selectionAppended(QPointF)));
    connect(this, SIGNAL(moved(QPointF)), this, SLOT(selectionMoved(QPointF)));
}

//----------------------------------------------------------------------------------------------------------------------------------
void UserInteractionPlotPicker::setBackgroundFillBrush(const QBrush &brush)
{
    if (brush != this->m_rectFillBrush)
    {
        m_rectFillBrush = brush;
        updateDisplay();
    }
}

//----------------------------------------------------------------------------------------------------------------------------------
void UserInteractionPlotPicker::drawTracker(QPainter *painter) const
{
    const QRect textRect = trackerRect(painter->font());
    if (!textRect.isEmpty())
    {
        const QwtText label = trackerText(trackerPosition());
        if (!label.isEmpty())
        {
            painter->fillRect(textRect, m_rectFillBrush);
            label.draw(painter, textRect);
        }
    }
}

//----------------------------------------------------------------------------------------------------------------------------------
void UserInteractionPlotPicker::drawRubberBand(QPainter *painter) const
{
    if (!isActive() || rubberBand() == NoRubberBand ||
        rubberBandPen().style() == Qt::NoPen)
    {
        return;
    }

    const QPolygon pa = adjustedPoints(pickedPoints());

    QwtPickerMachine::SelectionType selectionType =
        QwtPickerMachine::NoSelection;

    if (stateMachine())
        selectionType = stateMachine()->selectionType();

    switch (selectionType)
    {
        case QwtPickerMachine::NoSelection:
        case QwtPickerMachine::PointSelection:
        {
            if (pa.count() < 1)
                return;

            const QPoint pos = pa[0];

            const QRect pRect = pickArea().boundingRect().toRect();
            switch (rubberBand())
            {
                case VLineRubberBand:
                {
                    QwtPainter::drawLine(painter, pos.x(),
                        pRect.top(), pos.x(), pRect.bottom());
                    break;
                }
                case HLineRubberBand:
                {
                    QwtPainter::drawLine(painter, pRect.left(),
                        pos.y(), pRect.right(), pos.y());
                    break;
                }
                case CrossRubberBand:
                {
                    QwtPainter::drawLine(painter, pos.x(),
                        pRect.top(), pos.x(), pRect.bottom());
                    QwtPainter::drawLine(painter, pRect.left(),
                        pos.y(), pRect.right(), pos.y());
                    break;
                }
                default:
                    break;
            }
            break;
        }
        case QwtPickerMachine::RectSelection:
        {
            if (pa.count() < 2)
                return;

            const QRect rect = QRect(pa.first(), pa.last()).normalized();
            switch (rubberBand())
            {
                case EllipseRubberBand:
                {
                    QwtPainter::drawEllipse(painter, rect);
                    break;
                }
                case RectRubberBand:
                {
                    QwtPainter::drawRect(painter, rect);
                    break;
                }
                default:
                    break;
            }
            break;
        }
        case QwtPickerMachine::PolygonSelection:
        {
            if (rubberBand() == PolygonRubberBand)
            {
                painter->drawPolyline(pa);
            }
            else if (rubberBand() == QwtPicker::UserRubberBand)
            {
                if (pa.size() > 0)
                {
                    const QPoint pos = pa.last();

                    const QRect pRect = pickArea().boundingRect().toRect();

                    QwtPainter::drawLine(painter, pos.x(),
                            pRect.top(), pos.x(), pRect.bottom());
                    QwtPainter::drawLine(painter, pRect.left(),
                        pos.y(), pRect.right(), pos.y());
                }
                break;
            }
            break;
        }
        default:
            break;
    }
}

//----------------------------------------------------------------------------------------------------------------------------------
void UserInteractionPlotPicker::selectionAppended(const QPointF &pos)
{
    if (!m_keepAspectRatio || m_selection.size() != 1)
    {
        m_selection.append(pos);
    }
    else
    {
        //the second point can be the end point of a square or circle since the aspect ratio is fixed. Correct this point!
        QPointF start = m_selection[0];
        QPointF diff = pos - start;
        const qreal size = 0.5 * (qAbs(diff.x()) + qAbs(diff.y()));
        QPointF dest = start;
        dest.rx() += size * (diff.x() >= 0 ? 1.0 : -1.0);
        dest.ry() += size * (diff.y() >= 0 ? 1.0 : -1.0);
        m_selection.append(dest);
    }
}

//----------------------------------------------------------------------------------------------------------------------------------
void UserInteractionPlotPicker::selectionMoved(const QPointF &pos)
{
    if (m_selection.size() == 2 && m_keepAspectRatio)
    {
        //the second point can be the end point of a square or circle since the aspect ratio is fixed. Correct this point!
        QPointF start = m_selection[0];
        QPointF diff = pos - start;
        const qreal size = 0.5 * (qAbs(diff.x()) + qAbs(diff.y()));
        QPointF dest = start;
        dest.rx() += size * (diff.x() >= 0 ? 1.0 : -1.0);
        dest.ry() += size * (diff.y() >= 0 ? 1.0 : -1.0);
        m_selection.last() = dest;
    }
    else if (m_selection.size() > 0)
    {
        m_selection.last() = pos;
    }
}

//----------------------------------------------------------------------------------------------------------------------------------
void UserInteractionPlotPicker::selectionActivated(bool on)
{
    if (on)
    {
        m_selection.clear();
    }
}

//----------------------------------------------------------------------------------------------------------------------------------
QPolygon UserInteractionPlotPicker::adjustedPoints(const QPolygon &points) const
{
    if (!m_keepAspectRatio)
    {
        return points;
    }
    else
    {
        if ( points.size() == 2 )
        {
            QPointF start = invTransform(points[0]);
            QPointF end = invTransform(points[1]);
            QPolygon adjusted;
            QPointF diff = end - start;
            const qreal size = 0.5 * (qAbs(diff.x()) + qAbs(diff.y()));
            QPointF dest = start;
            dest.rx() += size * (diff.x() >= 0 ? 1.0 : -1.0);
            dest.ry() += size * (diff.y() >= 0 ? 1.0 : -1.0);

            adjusted += points[0];
            adjusted += transform(dest);
            return adjusted;
        }
        else
        {
            return points;
        }

    }
}
