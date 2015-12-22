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

    QVector<QwtPlotMarker*> m_marker;
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
    
    d->m_pparent = parent;
    d->m_autoColor = true;
    d->m_selected = false;
    d->m_labelVisible = false;

    if (retVal)
    {
        *retVal += setShape(shape);
    }
    else
    {
        setShape(shape);
    }

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
    for (int n = 0; n < d->m_marker.size(); n++)
    {
        d->m_marker[n]->detach();
        delete d->m_marker[n];
    }
    d->m_marker.clear();

    detach();

    idxVec.remove(idxVec.indexOf(d->m_shape.index()));

    delete d;
    d = NULL;
    
}
//----------------------------------------------------------------------------------------------------------------------------------
void DrawItem::setSelected(const bool selected)
{
    d->m_selected = selected;
    int flags = d->m_shape.flags();
    bool moveable = !(flags & ito::Shape::MoveLock);
    bool resizeable = !(flags & ito::Shape::ResizeLock);
    QwtPlotMarker *marker = NULL;

    switch (d->m_shape.type())
    {
    case ito::Shape::Invalid:
        break;
    case ito::Shape::Point:
    case ito::Shape::MultiPointPick:
    
        {
            if (selected && (moveable | resizeable))
            {
                foreach(QwtPlotMarker* marker, d->m_marker)
                {
                    marker->setSymbol(new QwtSymbol(selected ? QwtSymbol::Rect : QwtSymbol::Triangle, QBrush(d->m_markerColor),
                        QPen(QBrush(d->m_markerColor), 1), selected ? QSize(9, 9) : QSize(7, 7)));
                }
            }
            else
            {
                foreach(QwtPlotMarker* marker, d->m_marker)
                {
                    marker->setSymbol(new QwtSymbol(QwtSymbol::Triangle, QBrush(d->m_markerColor),
                        QPen(QBrush(d->m_markerColor), 1), QSize(7, 7)));
                }
            }
        }
        break;
    case ito::Shape::Line:
    case ito::Shape::Polygon:
        {
            QTransform &trafo = d->m_shape.rtransform();
            QPointF pt;

            if (selected && resizeable)
            {
                while (d->m_marker.size() < d->m_shape.rbasePoints().size())
                {
                    marker = new QwtPlotMarker();
                    marker->setLinePen(QPen(d->m_markerColor));
                    marker->setSymbol(new QwtSymbol(QwtSymbol::Rect, QBrush(d->m_markerColor), QPen(QBrush(d->m_markerColor), 1), QSize(7, 7)));
                    marker->setVisible(true);
                    marker->attach(d->m_pparent);
                    d->m_marker.append(marker);
                }

                for (int i = 0; i < d->m_shape.rbasePoints().size(); ++i)
                {
                    pt = trafo.map(d->m_shape.rbasePoints()[i]);
                    d->m_marker[i]->setXValue(pt.x());
                    d->m_marker[i]->setYValue(pt.y());
                }
            }
            else
            {
                //delete markers, since deselected
                for (int n = 0; n < d->m_marker.size(); n++)
                {
                    d->m_marker[n]->detach();
                    delete d->m_marker[n];
                }
                d->m_marker.clear();
            }
        }
        break;
    case ito::Shape::Rectangle:
    case ito::Shape::Square:
    case ito::Shape::Ellipse:
    case ito::Shape::Circle:
    {
        QRectF box(d->m_shape.rbasePoints()[0], d->m_shape.rbasePoints()[1]);
        bool keepAspect = ((d->m_shape.type() == ito::Shape::Square) || (d->m_shape.type() == ito::Shape::Circle));
        int numMarkers = keepAspect ? 4 : 8;
        QTransform &trafo = d->m_shape.rtransform();
        QPointF pt;

        if (selected && resizeable)
        {
            while (d->m_marker.size() < numMarkers)
            {
                marker = new QwtPlotMarker();
                marker->setLinePen(QPen(d->m_markerColor));
                marker->setSymbol(new QwtSymbol(QwtSymbol::Rect, QBrush(d->m_markerColor), QPen(QBrush(d->m_markerColor), 1), QSize(7, 7)));
                marker->setVisible(true);
                marker->attach(d->m_pparent);
                d->m_marker.append(marker);
            }

            if (keepAspect)
            {
                //1:
                pt = trafo.map(box.topLeft());
                d->m_marker[0]->setXValue(pt.x());
                d->m_marker[0]->setYValue(pt.y());

                //2:
                pt = trafo.map(box.topRight());
                d->m_marker[1]->setXValue(pt.x());
                d->m_marker[1]->setYValue(pt.y());

                //3:
                pt = trafo.map(box.bottomRight());
                d->m_marker[2]->setXValue(pt.x());
                d->m_marker[2]->setYValue(pt.y());

                //4:
                pt = trafo.map(box.bottomLeft());
                d->m_marker[3]->setXValue(pt.x());
                d->m_marker[3]->setYValue(pt.y());
            }
            else
            {
                //1:
                pt = trafo.map(box.topLeft());
                d->m_marker[0]->setXValue(pt.x());
                d->m_marker[0]->setYValue(pt.y());

                //2:
                pt = trafo.map(0.5 * (box.topRight() + box.topLeft()));
                d->m_marker[1]->setXValue(pt.x());
                d->m_marker[1]->setYValue(pt.y());

                //3:
                pt = trafo.map(box.topRight());
                d->m_marker[2]->setXValue(pt.x());
                d->m_marker[2]->setYValue(pt.y());

                //4:
                pt = trafo.map(0.5 * (box.bottomRight() + box.topRight()));
                d->m_marker[3]->setXValue(pt.x());
                d->m_marker[3]->setYValue(pt.y());

                //5:
                pt = trafo.map(box.bottomRight());
                d->m_marker[4]->setXValue(pt.x());
                d->m_marker[4]->setYValue(pt.y());

                //6:
                pt = trafo.map(0.5 * (box.bottomLeft() + box.bottomRight()));
                d->m_marker[5]->setXValue(pt.x());
                d->m_marker[5]->setYValue(pt.y());

                //7:
                pt = trafo.map(box.bottomLeft());
                d->m_marker[6]->setXValue(pt.x());
                d->m_marker[6]->setYValue(pt.y());

                //8:
                pt = trafo.map(0.5 * (box.bottomLeft() + box.topLeft()));
                d->m_marker[7]->setXValue(pt.x());
                d->m_marker[7]->setYValue(pt.y());
            }
      
        }
        else
        {
            //delete markers, since deselected
            for (int n = 0; n < d->m_marker.size(); n++)
            {
                d->m_marker[n]->detach();
                delete d->m_marker[n];
            }
            d->m_marker.clear();
        }
    }
    break;
    default:
        break;
    }
    
    //line width of painter path also depends on selected state
    setPen(d->m_lineColor, d->m_selected ? 3 : 1);
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
QPointF DrawItem::getMarkerPosScale(int index) const
{
    if (index < 0 || index >= d->m_marker.size())
    {
        return QPointF();
    }
    return QPointF(d->m_marker[index]->xValue(), d->m_marker[index]->yValue());
}

//----------------------------------------------------------------------------------------------------------------------------------
bool DrawItem::shapeMoveTo(const QPointF &marker1ScaleCoordinate)
{
    bool moveable = !(d->m_shape.flags() & ito::Shape::MoveLock);
    if (moveable)
    {
        d->m_shape.point1MoveTo(marker1ScaleCoordinate);
        setShape(d->m_shape);
        return true;
    }
    return false;
}

//----------------------------------------------------------------------------------------------------------------------------------
bool DrawItem::shapeResize(int markerIdx, const QPointF &markerScaleCoordinate, const Qt::KeyboardModifiers &modifiers /*= Qt::NoModifier*/)
{
    bool resizable = !(d->m_shape.flags() & ito::Shape::ResizeLock);
    if (resizable)
    {
        const QTransform &trafo = d->m_shape.rtransform();
        QTransform invTrafo = trafo.inverted();
        QPolygonF &basePoints = d->m_shape.rbasePoints();
        bool success = false;

        switch (d->m_shape.type())
        {
        case ito::Shape::Line:
        {
            QPointF newPoint = invTrafo.map(markerScaleCoordinate);
            if (modifiers & Qt::ControlModifier)
            {
                if (markerIdx == 1)
                {
                    QLineF line(basePoints[2], newPoint);
                    double angle = 45.0 * qRound(line.angle() / 45.0);
                    line.setAngle(angle);
                    basePoints[1] = line.p2();
                    success = true;
                }
                else if (markerIdx == 2)
                {
                    QLineF line(newPoint, basePoints[1]);
                    double angle = 45.0 * qRound(line.angle() / 45.0);
                    line.setAngle(angle);
                    basePoints[2] = line.p2();
                    success = true;
                }
            }
            else if (markerIdx == 1 || markerIdx == 2)
            {
                basePoints[markerIdx] = newPoint;
                success = true;
            }
        }
        case ito::Shape::Point:
        case ito::Shape::MultiPointPick:
        case ito::Shape::Polygon:
        {
            if (markerIdx >= 0 && markerIdx < basePoints.size())
            {
                basePoints[markerIdx] = invTrafo.map(markerScaleCoordinate);
                success = true;
            }
            break;
        }
        case ito::Shape::Rectangle:
        case ito::Shape::Ellipse:
        {
            QPointF newPoint = invTrafo.map(markerScaleCoordinate);
            QRectF baseRect(basePoints[0], basePoints[1]);
            success = true;

            switch (markerIdx)
            {
            case 1: //top, left
                baseRect.setTopLeft(newPoint);
                break;
            case 2: //top, center
                baseRect.setTop(newPoint.y());
                break;
            case 3: //top, right
                baseRect.setTopRight(newPoint);
                break;
            case 4: //right, center
                baseRect.setRight(newPoint.x());
                break;
            case 5: //bottom, right
                baseRect.setBottomRight(newPoint);
                break;
            case 6: //bottom, center
                baseRect.setBottom(newPoint.y());
                break;
            case 7: //bottom, left
                baseRect.setBottomLeft(newPoint);
                break;
            case 8: //left, center
                baseRect.setLeft(newPoint.x());
                break;
            default:
                success = false;
            }

            if (success)
            {
                basePoints[0] = baseRect.topLeft();
                basePoints[1] = baseRect.bottomRight();
            }
            break;
        }
        case ito::Shape::Square:
        case ito::Shape::Circle:
        {
            QPointF newPoint = invTrafo.map(markerScaleCoordinate);
            QRectF baseRect(basePoints[0], basePoints[1]);
            QPointF diff = newPoint - baseRect.center();
            qreal equal_diff = 0.5*(diff.x() + diff.y());
            newPoint = baseRect.center() + QPointF(equal_diff, equal_diff);
            success = true;

            switch (markerIdx)
            {
            case 1: //top, left
                baseRect.setBottomLeft(newPoint);
                break;
            case 2: //top, right
                baseRect.setBottomRight(newPoint);
                break;
            case 3: //bottom, right
                baseRect.setTopRight(newPoint);
                break;
            case 4: //bottom, left
                baseRect.setTopLeft(newPoint);
                break;
            default:
                success = false;
            }

            if (success)
            {
                basePoints[0] = baseRect.topLeft();
                basePoints[1] = baseRect.bottomRight();
            }
            break;
        }
        }
        setShape(d->m_shape);
        return success;
    }
    return false;
}

//----------------------------------------------------------------------------------------------------------------------------------
void DrawItem::setColor(const QColor &markerColor, const QColor &lineColor)
{
    if (d->m_selected == false)
    {
        if (d->m_shape.type() == ito::Shape::Point)
        {
            for (int n = 0; n < d->m_marker.size(); n++)
            {
                d->m_marker[n]->setLinePen(QPen(markerColor));
                d->m_marker[n]->setSymbol(new QwtSymbol(QwtSymbol::Triangle, QBrush(markerColor),
                    QPen(QBrush(markerColor), 1), QSize(7, 7)));
            }
        }
        else
        {
            for (int n = 0; n < d->m_marker.size(); n++)
            {
                d->m_marker[n]->setSymbol(new QwtSymbol(QwtSymbol::Diamond, QBrush(markerColor),
                    QPen(QBrush(markerColor), 1), QSize(7, 7)));
            }
        }
    }
    else if (d->m_selected)
    {
        if (d->m_marker.size() >= 1)
            d->m_marker[0]->setSymbol(new QwtSymbol(QwtSymbol::Rect, QBrush(markerColor),
            QPen(QBrush(markerColor), 1), QSize(9, 9)));
    }

    setPen(lineColor, d->m_selected ? 3 : 1);

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
        QwtPlotShapeItem::setShape(path);
        QwtPlotShapeItem::setTitle(d->m_shape.name());

        setSelected(d->m_selected); //to possibly adjust the position of markers
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
bool DrawItem::hitLine(const QPointF &point_transformed, const QLineF &line, double tol_x, double tol_y) const
{
    

    QLineF normal = line.normalVector();
    normal.translate(-line.p1() + point_transformed);

    QPointF intersection;
    line.intersect(normal, &intersection);

    if (std::abs(QLineF(point_transformed, intersection).dx()) <= tol_x && std::abs(QLineF(point_transformed, intersection).dy()) <= tol_y)
    {
        //check that intersection point lies on line.
        double dist1 = QLineF(line.p1(), intersection).length();
        double dist2 = QLineF(line.p2(), intersection).length();
        double lineLength = line.length();
        return (dist1 <= lineLength && dist2 <= lineLength);
    }

    return false;
}

//----------------------------------------------------------------------------------------------------------------------------------
//returns -1 if not hit, 0: if contour hit for moving or 1..8 if markers hit for resizing (only if moving or resizing is allowed)
char DrawItem::hitEdge(const QPointF &point, double tol_x, double tol_y) const
{
    ito::RetVal retVal;
    const ito::Shape &shape = d->m_shape;
    QPointF point_trafo = shape.transform().inverted().map(point); //transform point to base coordinate system of shape

    bool hitEdge = false;

    switch (d->m_shape.type())
    {
    case ito::Shape::Point:
    {
        QLineF line(point_trafo, shape.basePoints()[0]);
        hitEdge = std::abs(line.dx() <= tol_x) && std::abs(line.dy() <= tol_y);
    }
    break;

    case ito::Shape::Line:
    {
        QLineF line(shape.basePoints()[0], shape.basePoints()[1]);
        hitEdge = hitLine(point_trafo, line, tol_x, tol_y);
    }
    break;

    case ito::Shape::Rectangle:
    case ito::Shape::Square:
    {
        QPolygonF contour = shape.contour();
        hitEdge = (hitLine(point, QLineF(contour[0], contour[1]), tol_x, tol_y) || \
            hitLine(point, QLineF(contour[1], contour[2]), tol_x, tol_y) || \
            hitLine(point, QLineF(contour[2], contour[3]), tol_x, tol_y) || \
            hitLine(point, QLineF(contour[3], contour[0]), tol_x, tol_y));
    }
    break;

    case ito::Shape::Ellipse:
    case ito::Shape::Circle:
    {
        //inverse transform point
        QPointF size = shape.basePoints()[1] - shape.basePoints()[0];
        double a = size.x()/2.0 + tol_x;
        double b = size.y()/2.0 + tol_y;
        QPointF center = (shape.basePoints()[0] + shape.basePoints()[1]) / 2.0;
        double x = point_trafo.x() - center.x();
        double y = point_trafo.y() - center.y();

        if ((((x*x) / (a*a)) + ((y*y) / (b*b))) <= 1.0)
        {
            a = size.x() / 2.0 - tol_x;
            b = size.y() / 2.0 - tol_y;
            hitEdge =((((x*x) / (a*a)) + ((y*y) / (b*b))) > 1.0);
        }
    }
    break;

    case ito::Shape::MultiPointPick:
    case ito::Shape::Polygon:
    {
        QLineF line;
        foreach(const QPointF &pt, shape.basePoints())
        {
            line = QLineF(point_trafo, pt);
            if (std::abs(line.dx() <= tol_x) && std::abs(line.dy() <= tol_y))
            {
                hitEdge = true;
                break;
            }
        }
    }
    break;
    default:
        break;
    }

    if (hitEdge)
    {
        int flags = d->m_shape.flags();
        bool moveable = !(flags & ito::Shape::MoveLock);
        bool resizeable = !(flags & ito::Shape::ResizeLock);

        if (resizeable)
        {
            for (int i = 0; i < d->m_marker.size(); ++i)
            {
                if (std::abs(d->m_marker[i]->xValue() - point.x()) <= tol_x && \
                    std::abs(d->m_marker[i]->yValue() - point.y()) <= tol_y)
                {
                    return (i + 1);
                }
            }
        }

        if (moveable)
        {
            return 0;
        }
    }

    return -1;
}
