/* ********************************************************************
 i tom measurement sys*tem
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

#include "drawItem.h"
#include "qwt_scale_map.h"
#include <qwt_symbol.h>

#include <qrect.h>

QVector<int> DrawItem::idxVec;

class DrawItemPrivate
{
public:
    DrawItemPrivate() : m_pparent(NULL) {}
    ~DrawItemPrivate() {};

    ito::Shape m_shape;
    QPen m_markerPen;
    QBrush m_markerBrush;
    QPen m_linePen;
    QwtPlot *m_pparent;

    bool m_selected;

    QVector<QwtPlotMarker *> m_marker;
    char m_active;
    char m_type;    

    bool m_autoColor;
    bool m_labelVisible;

    QColor m_markerColor;
    QColor m_lineColor;

    QPointF m_point1;
    QPointF m_point2;
};

//----------------------------------------------------------------------------------------------------------------------------------
DrawItem::DrawItem(const ito::Shape &shape, QwtPlot *parent, ito::RetVal *retVal /*=NULL*/) : QwtPlotShapeItem(shape.name()), d(NULL)
{
    d = new DrawItemPrivate();
    if (retVal)
    {
        *retVal += setShape(shape);
    }
    else
    {
        setShape(shape);
    }
    d->m_pparent = parent;
    d->m_active = 0;
    d->m_autoColor = true;
    d->m_selected = false;
    d->m_labelVisible = false;

    if (shape.index() < 0)
    {
        int idxCtr = 0;
        do 
            idxCtr++;
        while (idxVec.contains(idxCtr));
        d->m_shape.setIndex(idxCtr);
        idxVec.append(idxCtr);
    }
    else 
    {
        idxVec.append(shape.index());
    }

    setRenderHint( QwtPlotItem::RenderAntialiased, true); //set AntiAliasing of geometric objects to true in order to plot them smoother
}

//----------------------------------------------------------------------------------------------------------------------------------
DrawItem::~DrawItem()
{
    detach();
    for (int n = 0; n < d->m_marker.size(); n++)
    {
        d->m_marker[n]->detach();
//        m_marker.remove(n);
        delete d->m_marker[n];
    }
    d->m_marker.clear();
    idxVec.remove(idxVec.indexOf(d->m_shape.index()));

    delete d;
    d = NULL;
    
}
//----------------------------------------------------------------------------------------------------------------------------------
void DrawItem::setSelected(const bool selected)
{
    d->m_selected = selected;

    if (d->m_shape.type() == ito::Shape::Point && d->m_marker.size() > 0)
    {
        QColor markerColor = d->m_marker[0]->linePen().color();
        d->m_marker[0]->setSymbol(new QwtSymbol(selected ? QwtSymbol::Rect : QwtSymbol::Triangle, QBrush(markerColor),
            QPen(QBrush(markerColor), 1), selected ? QSize(9,9) : QSize(7,7) ));
    }
    setPen(pen().color(), d->m_selected ? 3 : 1);
    return;
}

//----------------------------------------------------------------------------------------------------------------------------------
bool DrawItem::getSelected() const
{
    return d->m_selected;
}

//----------------------------------------------------------------------------------------------------------------------------------
void DrawItem::setLabel(const QString &label)
{
    d->m_shape.setName(label);
    QwtPlotShapeItem::setTitle(label);
}

//----------------------------------------------------------------------------------------------------------------------------------
QString DrawItem::getLabel() const
{
    return d->m_shape.name();
}

//----------------------------------------------------------------------------------------------------------------------------------
void DrawItem::setLabelVisible(const bool labelVisible)
{
    d->m_labelVisible = labelVisible;
}

//----------------------------------------------------------------------------------------------------------------------------------
bool DrawItem::getLabelVisible() const
{
    return d->m_labelVisible;
}

//----------------------------------------------------------------------------------------------------------------------------------
int DrawItem::getIndex() const
{
    return d->m_shape.index();
}

//----------------------------------------------------------------------------------------------------------------------------------
bool DrawItem::getAutoColor() const
{
    return d->m_autoColor;
}

//----------------------------------------------------------------------------------------------------------------------------------
const ito::Shape &DrawItem::getShape() const
{
    return d->m_shape;
}

//----------------------------------------------------------------------------------------------------------------------------------
QPointF DrawItem::getPoint1() const
{
    return d->m_point1;
}

//----------------------------------------------------------------------------------------------------------------------------------
QPointF DrawItem::getPoint2() const
{
    return d->m_point2;
}

//----------------------------------------------------------------------------------------------------------------------------------
char DrawItem::getActive() const
{
    return d->m_active;
}

//----------------------------------------------------------------------------------------------------------------------------------
void DrawItem::setActive(char active)
{
    QColor markerColor = Qt::green;
    if (d->m_marker.size()) markerColor = d->m_marker[0]->linePen().color();

    if (d->m_shape.type() == ito::Shape::Point)
    {
        for (int n = 0; n < d->m_marker.size(); n++)
        {
            d->m_marker[n]->setLinePen(QPen(markerColor));
            d->m_marker[n]->setSymbol(new QwtSymbol(QwtSymbol::Triangle, QBrush(markerColor),
                QPen(QBrush(markerColor), 1),  QSize(7,7) ));
        }
    }
    else
    {
        for (int n = 0; n < d->m_marker.size(); n++)
        {
            d->m_marker[n]->setSymbol(new QwtSymbol(QwtSymbol::Diamond, QBrush(markerColor),
                QPen(QBrush(markerColor), 1),  QSize(7,7) ));
        }
    }

    if (active == 1)
    {
        if (d->m_marker.size() >= 1)
            d->m_marker[0]->setSymbol(new QwtSymbol(QwtSymbol::Rect, QBrush(markerColor),
                QPen(QBrush(markerColor), 1),  QSize(9,9) ));
    }
    else if (active == 2)
    {
        if (d->m_marker.size() >= 2)
            d->m_marker[1]->setSymbol(new QwtSymbol(QwtSymbol::Rect, QBrush(markerColor),
                QPen(QBrush(markerColor), 1),  QSize(9,9) ));
    }

    d->m_active = active;
}
//----------------------------------------------------------------------------------------------------------------------------------
void DrawItem::setColor(const QColor &markerColor, const QColor &lineColor)
{
    if(d->m_shape.type() == ito::Shape::Point)
    {
        for (int n = 0; n < d->m_marker.size(); n++)
        {
            d->m_marker[n]->setLinePen(QPen(markerColor));
            d->m_marker[n]->setSymbol(new QwtSymbol(QwtSymbol::Triangle, QBrush(markerColor),
                QPen(QBrush(markerColor), 1),  QSize(7,7) ));
        }
    }
    else
    {
        for (int n = 0; n < d->m_marker.size(); n++)
        {
            d->m_marker[n]->setSymbol(new QwtSymbol(QwtSymbol::Diamond, QBrush(markerColor),
                QPen(QBrush(markerColor), 1),  QSize(7,7) ));
        }
    }
    setPen(QColor(lineColor), d->m_selected ? 3 : 1);

    d->m_markerColor = markerColor;
    d->m_lineColor = lineColor;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal DrawItem::setShape(const ito::Shape &shape)
{
    ito::RetVal retVal;

    //create QPainterPath from shape
    QPainterPath path; //this must be inside of for loop, else it is not cleared for every new shape
    d->m_point1 = QPointF();
    d->m_point2 = QPointF();

    switch (shape.type())
    {
    case ito::Shape::Point:
    {
        d->m_point1 = shape.transform().map(shape.basePoints()[0]);
        path.moveTo(d->m_point1);
        path.lineTo(d->m_point1);
    }
    break;

    case ito::Shape::Line:
    {
        d->m_point1 = shape.transform().map(shape.basePoints()[0]);
        d->m_point2 = shape.transform().map(shape.basePoints()[1]);
        path.moveTo(d->m_point1);
        path.lineTo(d->m_point2);
    }
    break;

    case ito::Shape::Rectangle:
    case ito::Shape::Square:
        if (!shape.transform().isRotating())
        {
            d->m_point1 = shape.transform().map(shape.basePoints()[0]);
            d->m_point2 = shape.transform().map(shape.basePoints()[1]);
            QRectF rect(d->m_point1, d->m_point2);
            path.addRect(rect);
        }
        else
        {
            retVal += ito::RetVal(ito::retError, 0, QObject::tr("rotated shapes are currently not supported.").toLatin1().data());
        }
        
        break;

    case ito::Shape::Ellipse:
    case ito::Shape::Circle:
        if (!shape.transform().isRotating())
        {
            d->m_point1 = shape.transform().map(shape.basePoints()[0]);
            d->m_point2 = shape.transform().map(shape.basePoints()[1]);
            QRectF rect(d->m_point1, d->m_point2);
            path.addEllipse(rect);
        }
        else
        {
            retVal += ito::RetVal(ito::retError, 0, QObject::tr("rotated shapes are currently not supported.").toLatin1().data());
        }

        break;

    default:
        retVal += ito::RetVal(ito::retError, 0, QObject::tr("invalid geometric shape type").toLatin1().data());
        break;
    }

    if (!retVal.containsError() && path.elementCount() > 0)
    {
        d->m_shape = shape;
        QwtPlotMarker *marker = NULL;
        int numOfElements = path.elementCount();

        QwtPlotShapeItem::setShape(path);
        QwtPlotShapeItem::setTitle(d->m_shape.name());

        if (d->m_marker.size() > 0)
        {

            for (int n = 0; n < d->m_marker.size(); n++)
            {
                d->m_marker[n]->detach();
                delete d->m_marker[n];
            }
            d->m_marker.clear();
        }

        if (!d->m_point1.isNull())
        {
            marker = new QwtPlotMarker();

            if (shape.type() == ito::Shape::Point)
            {
                marker->setLinePen(QPen(d->m_markerColor));
                marker->setSymbol(new QwtSymbol(QwtSymbol::Triangle, QBrush(d->m_markerColor),
                    QPen(QBrush(d->m_markerColor), 1), QSize(7, 7)));
            }
            else
            {
                marker->setLinePen(QPen(d->m_markerColor));
                marker->setSymbol(new QwtSymbol(QwtSymbol::Diamond, QBrush(d->m_markerColor),
                    QPen(QBrush(d->m_markerColor), 1), QSize(7, 7)));
            }

            marker->setXValue(d->m_point1.x());
            marker->setYValue(d->m_point1.y());
            marker->setVisible(true);
            marker->attach(d->m_pparent);
            d->m_marker.append(marker);
        }

        if (!d->m_point2.isNull())
        {
            marker = new QwtPlotMarker();
            marker->setLinePen(QPen(d->m_lineColor));
            marker->setSymbol(new QwtSymbol(QwtSymbol::Diamond, QBrush(d->m_lineColor),
                QPen(QBrush(d->m_lineColor), 1), QSize(7, 7)));

            marker->setXValue(d->m_point2.x());
            marker->setYValue(d->m_point2.y());
            marker->setVisible(true);
            marker->attach(d->m_pparent);
            d->m_marker.append(marker);
        }
    }

    return retVal;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal DrawItem::setShape(const ito::Shape &shape, const QColor &firstColor, const QColor &secondColor)
{
    setColor(firstColor, secondColor);
    return setShape(shape);
}


//----------------------------------------------------------------------------------------------------------------------------------
void DrawItem::draw( QPainter *painter, 
            const QwtScaleMap &xMap, const QwtScaleMap &yMap,
            const QRectF &canvasRect ) const
{
    QwtPlotShapeItem::draw(painter, xMap, yMap, canvasRect);

    //const QRectF cRect = QwtScaleMap::invTransform(xMap, yMap, canvasRect.toRect() );
    if(!d->m_shape.name().isEmpty() && d->m_labelVisible)
    {
        QRectF myRect(0, 0, 0, 0);
    
        const QwtText label(d->m_shape.name());

        const QSizeF textSize = label.textSize( painter->font() );

        const QPointF textSizeScales = QPointF(textSize.width() * fabs(xMap.sDist()/xMap.pDist()), textSize.height() * fabs(yMap.sDist()/yMap.pDist()));

        if (d->m_marker.size() == 1 && d->m_marker[0] != NULL)
        {
            myRect = QRectF(d->m_marker[0]->xValue(), d->m_marker[0]->yValue(), textSizeScales.x(), textSizeScales.y());
        }
        else if (d->m_marker.size() > 1)
        {
            myRect = QRectF(d->m_marker[0]->xValue(), d->m_marker[0]->yValue(), textSizeScales.x(), textSizeScales.y());
        }
        const QRectF cRect = QwtScaleMap::transform(xMap, yMap, myRect );

        if ( !cRect.isEmpty() )
        {
            if ( !label.isEmpty() )
            {
                QBrush myBrush = QBrush(QColor(255, 255, 255, 170),Qt::SolidPattern);
                painter->fillRect(cRect, myBrush);
                label.draw( painter, cRect );
            }
        }
    }

    return;
}
//----------------------------------------------------------------------------------------------------------------------------------
