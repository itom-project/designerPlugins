/* ********************************************************************
 i tom measurement sys*tem
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

#include "drawItem.h"
#include "qwt_scale_map.h"
#include <qwt_symbol.h>
#include <qwt_plot.h>
#include <qwt_text.h>
#include "common/apiFunctionsGraphInc.h"
#include <qrect.h>
#include <qmath.h>
#include <qvariant.h>
#include <qpainter.h>


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

        bool m_autoColor; //!< if true, the given shape brings it own valid color, which is used as line color. Else, the line color depends on the invalid color of the current color palette
        bool m_labelVisible;

        QColor m_markerColor;
        QColor m_markerColor2; //!> second marker color, used when a single point (currently only for polygones) is selected for manipulation
        QColor m_lineColor;
        QColor m_labelTextColor;
        QBrush m_fillBrush;
        QBrush m_fillBrushSelected;
        QBrush m_labelBrush;
        QFont m_labelFont;
        QPen m_elementPen;

        QPointF m_point1;
        QPointF m_point2;

        ItomQwtPlotEnums::ModificationModes m_modificationModes;

        int m_currentMarker;

        int m_rotationMarkerIndex;
        QPointF m_rotationMarkerBasePoint; //this is the point that lies in the middle between the two corner points of the bottom line
        QPointF m_rotationMarkerPoint; //this is the real point of the rotation marker, however the distance to the base point might change, depending on the current axis scales
};

//----------------------------------------------------------------------------------------------------------------------------------
DrawItem::DrawItem(const ito::Shape &shape, ItomQwtPlotEnums::ModificationModes &modificationModes, QwtPlot *parent, ito::RetVal *retVal /*=NULL*/, bool labelVisible /*= false*/) :
    QwtPlotShapeItem(shape.name()),
    d(NULL)
{
    d = new DrawItemPrivate();

    d->m_pparent = parent;
    d->m_autoColor = true;
    d->m_selected = false;
    d->m_labelVisible = labelVisible;
    d->m_modificationModes = modificationModes;
    d->m_fillBrush = QBrush();
    d->m_fillBrushSelected = QBrush();
    d->m_labelBrush = QBrush(Qt::white, Qt::SolidPattern);
    d->m_labelFont = QFont("Verdana", 10);
    //this pen is only used to set the style and the width of the elements. The color is set to the inverse color of the current palette
    d->m_elementPen = QPen(QBrush(Qt::red), 1, Qt::SolidLine);

    if (shape.color().isValid())
    {
        d->m_lineColor = shape.color();
        d->m_autoColor = false; //use the line color from the given shape, not from any inverse colors...
    }

    d->m_labelTextColor = QColor(Qt::red);

    d->m_rotationMarkerIndex = -1;
    d->m_rotationMarkerBasePoint = QPointF();
    d->m_rotationMarkerPoint = QPointF();
    d->m_currentMarker = -1;

    if (ito::ITOM_API_FUNCS_GRAPH)
    {
        if (d->m_pparent->parent())
        {
            d->m_labelBrush = apiGetFigureSetting((QObject*)(d->m_pparent->parent()), "shapeLabelBackground", d->m_labelBrush, NULL).value<QBrush>();
            d->m_labelFont = apiGetFigureSetting((QObject*)(d->m_pparent->parent()), "shapeLabelFont", d->m_labelFont, NULL).value<QFont>();
            d->m_elementPen = apiGetFigureSetting((QObject*)(d->m_pparent->parent()), "geometricShapePen", d->m_elementPen, NULL).value<QPen>();
            d->m_labelTextColor = apiGetFigureSetting((QObject*)(d->m_pparent->parent()), "shapeLabelTextColor", d->m_labelTextColor, NULL).value<QColor>();
        }
    }
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
        while (idxVec.contains(idxCtr))
        {
            idxCtr++;
        }
        d->m_shape.setIndex(idxCtr);
        idxVec.append(idxCtr);
    }
    else
    {
        idxVec.append(shape.index());
    }

	setZ(25.0); //display over lines / images (20.0), grid (10.0), but below markers (30.0) and labels (150.0)
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
int DrawItem::getSelectedMarker() const
{
    return d->m_currentMarker;
}

//----------------------------------------------------------------------------------------------------------------------------------
void DrawItem::setFillOpacity(int opacity, int opacitySelected)
{
    if (opacity <= 0)
    {
        d->m_fillBrush = QBrush();
    }
    else
    {
        QColor fillColor = d->m_lineColor;
        fillColor.setAlpha(opacity);
        d->m_fillBrush = QBrush(fillColor, Qt::SolidPattern);
    }

    if (opacitySelected <= 0)
    {
        d->m_fillBrushSelected = QBrush();
    }
    else
    {
        QColor fillColor = d->m_lineColor;
        fillColor.setAlpha(opacitySelected);
        d->m_fillBrushSelected = QBrush(fillColor, Qt::SolidPattern);
    }

    setSelected(d->m_selected);
}

//----------------------------------------------------------------------------------------------------------------------------------
void DrawItem::setModificationModes(const ItomQwtPlotEnums::ModificationModes &modes)
{
    d->m_modificationModes = modes;
    setSelected(d->m_selected);
}

//----------------------------------------------------------------------------------------------------------------------------------
void DrawItem::setSelected(const bool selected, int nMarker /*= -1*/)
{
    d->m_selected = selected;
    int flags = d->m_shape.flags();
    bool moveable = !(flags & ito::Shape::MoveLock) && d->m_modificationModes.testFlag(ItomQwtPlotEnums::Move);
    bool resizeable = !(flags & ito::Shape::ResizeLock) && d->m_modificationModes.testFlag(ItomQwtPlotEnums::Resize);
    bool rotatable = !(flags & ito::Shape::RotateLock) && d->m_modificationModes.testFlag(ItomQwtPlotEnums::Rotate);
    QwtPlotMarker *marker = NULL;

    setRenderHint(QwtPlotItem::RenderAntialiased, false);
    d->m_currentMarker = -1;
    d->m_rotationMarkerIndex = -1;

    switch (d->m_shape.type())
    {
        case ito::Shape::Invalid:
        break;

        case ito::Shape::Point:
        case ito::Shape::MultiPointPick:
            {
                // the color of point markers must be the line color,
                // since this is the color that can be defined by shape.color (if desired).
                if (selected && (moveable | resizeable))
                {
                    foreach(QwtPlotMarker* marker, d->m_marker)
                    {
                        marker->setSymbol(new QwtSymbol(selected ? QwtSymbol::Star1 : QwtSymbol::XCross, QBrush(d->m_lineColor),
                            QPen(QBrush(d->m_lineColor), selected ? 2 : 1), selected ? QSize(15, 15) : QSize(11, 11)));
                    }
                }
                else
                {
                    foreach(QwtPlotMarker* marker, d->m_marker)
                    {
                        marker->setSymbol(new QwtSymbol(QwtSymbol::XCross, QBrush(d->m_lineColor),
                            QPen(QBrush(d->m_lineColor), 1), QSize(11, 11)));
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
                    d->m_currentMarker = nMarker;
                    if (d->m_marker.size() > d->m_shape.rbasePoints().size())
                    {
                        // some marker has been deleted, rebuild markers
                        for (int n = 0; n < d->m_marker.size(); n++)
                        {
                            d->m_marker[n]->detach();
                            delete d->m_marker[n];
                        }
                        d->m_marker.clear();
                    }

                    // here we always adjust the markers so only the currently selected gets highlighted
                    for (int nm = 0; nm < d->m_marker.size(); nm++)
                    {
                        if (d->m_shape.type() == ito::Shape::Polygon && nm == nMarker)
                            d->m_marker[nm]->setSymbol(new QwtSymbol(QwtSymbol::Rect, QBrush(d->m_markerColor2), QPen(d->m_markerColor, 1), QSize(9, 9)));
                        else
                            d->m_marker[nm]->setSymbol(new QwtSymbol(QwtSymbol::Rect, QBrush(d->m_markerColor), QPen(d->m_markerColor, 1), QSize(9, 9)));
                    }

                    // This run only once, creating markers
                    int npt = 0;
                    while (d->m_marker.size() < d->m_shape.rbasePoints().size())
                    {
                        marker = new QwtPlotMarker();
                        marker->setLinePen(QPen());
                        if (d->m_shape.type() == ito::Shape::Polygon && npt == nMarker)
                            marker->setSymbol(new QwtSymbol(QwtSymbol::Rect, QBrush(d->m_markerColor2), QPen(d->m_markerColor, 1), QSize(9, 9)));
                        else
                            marker->setSymbol(new QwtSymbol(QwtSymbol::Rect, QBrush(d->m_markerColor), QPen(d->m_markerColor, 1), QSize(9, 9)));
                        marker->setVisible(true);
                        marker->attach(d->m_pparent);
                        d->m_marker.append(marker);
                        npt++;
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
            setRenderHint(QwtPlotItem::RenderAntialiased, true); //set AntiAliasing of geometric objects to true in order to plot them smoother
        break;

        case ito::Shape::Ellipse:
        case ito::Shape::Circle:
        case ito::Shape::Rectangle:
        case ito::Shape::Square:
        {
            //set AntiAliasing of geometric objects to true in order to plot them smoother
            setRenderHint(QwtPlotItem::RenderAntialiased, true);

            QRectF box(d->m_shape.rbasePoints()[0], d->m_shape.rbasePoints()[1]);
            bool keepAspect = ((d->m_shape.type() == ito::Shape::Square) || (d->m_shape.type() == ito::Shape::Circle));
            int numMarkers = resizeable ? (keepAspect ? 4 : 8) : 0;
            QTransform &trafo = d->m_shape.rtransform();
            QPointF pt;

            if (selected)
            {
                if (resizeable)
                {
                    while (d->m_marker.size() < numMarkers)
                    {
                        marker = new QwtPlotMarker();
                        marker->setLinePen(QPen(d->m_markerColor));
                        marker->setSymbol(new QwtSymbol(QwtSymbol::Rect, QBrush(d->m_markerColor) /*)*/, QPen(QBrush(d->m_markerColor), 1), QSize(9, 9)));
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

                if (rotatable)
                {
                    if (d->m_marker.size() < numMarkers + 1)
                    {
                        marker = new QwtPlotMarker();
                        marker->setLinePen(QPen(d->m_markerColor));
                        marker->setSymbol(new QwtSymbol(QwtSymbol::Ellipse, QBrush(d->m_markerColor) /*)*/, QPen(QBrush(d->m_markerColor), 1), QSize(9, 9)));
                        marker->setVisible(true);
                        marker->attach(d->m_pparent);
                        d->m_marker.append(marker);
                    }

                    if (box.bottom() < box.top())
                    {
                        d->m_rotationMarkerBasePoint = trafo.map(0.5 * (box.bottomLeft() + box.bottomRight()));
                        d->m_rotationMarkerPoint = trafo.map(QPointF(0.5 * (box.bottomLeft().x() + box.bottomRight().x()), 0.5 * (box.bottomLeft().y() + box.bottomRight().y()) - 5));

                        pt = d->m_rotationMarkerPoint;
                    }
                    else
                    {
                        d->m_rotationMarkerBasePoint = trafo.map(0.5 * (box.topLeft() + box.topRight()));
                        d->m_rotationMarkerPoint = trafo.map(QPointF(0.5 * (box.topLeft().x() + box.topRight().x()), 0.5 * (box.topLeft().y() + box.topRight().y()) - 5));

                        pt = d->m_rotationMarkerPoint;
                    }

                    d->m_rotationMarkerIndex = d->m_marker.size() - 1;
                    d->m_marker[d->m_rotationMarkerIndex]->setXValue(pt.x());
                    d->m_marker[d->m_rotationMarkerIndex]->setYValue(pt.y());

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
    setPen(d->m_lineColor, d->m_selected ? d->m_elementPen.width() + 2 : d->m_elementPen.width(), d->m_elementPen.style());

    if (selected)
    {
        setBrush(d->m_fillBrushSelected);
    }
    else
    {
        setBrush(d->m_fillBrush);
    }
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
    if (index >= 0 && index < d->m_marker.size())
    {
        return QPointF(d->m_marker[index]->xValue(), d->m_marker[index]->yValue());
    }

    //marker not available (e.g. no resize mode allowed, there we need to calculate the positions by hand...)

    switch (d->m_shape.type())
    {
        case ito::Shape::Invalid:
            return QPointF();

        case ito::Shape::Point:
        case ito::Shape::Line:
        case ito::Shape::Polygon:
        case ito::Shape::MultiPointPick:
            if (index >= -1 && index < d->m_shape.rbasePoints().size())
            {
                return d->m_shape.rtransform().map(d->m_shape.rbasePoints()[0]);
            }
            else
            {
                return QPointF();
            }

        case ito::Shape::Circle:
        case ito::Shape::Ellipse: {
            QRectF box(d->m_shape.rbasePoints()[0], d->m_shape.rbasePoints()[1]);
            bool keepAspect = d->m_shape.type() == ito::Shape::Circle;
            QTransform& trafo = d->m_shape.rtransform();

            if (index == -1)
            {
                // label position
                QPointF pt = box.center();
                qreal a = box.width() / 2;
                qreal b = box.height() / 2;
                pt.rx() += a * qCos(M_PI * 225.0 / 180.0);
                pt.ry() += b * qCos(M_PI * 225.0 / 180.0);
                return trafo.map(pt);
            }

            if (keepAspect)
            {
                switch (index)
                {
                case 0:
                    return trafo.map(box.topLeft());
                case 1:
                    return trafo.map(box.topRight());
                case 2:
                    return trafo.map(box.bottomRight());
                case 3:
                    return trafo.map(box.bottomLeft());
                case 4: // rotation marker
                    return trafo.map(QPointF(
                        0.5 * (box.topLeft().x() + box.topRight().x()),
                        0.5 * (box.topLeft().y() + box.topRight().y()) - 5));
                }
            }
            else
            {
                switch (index)
                {
                case 0:
                    return trafo.map(box.topLeft());
                case 1:
                    return trafo.map(0.5 * (box.topRight() + box.topLeft()));
                case 2:
                    return trafo.map(box.topRight());
                case 3:
                    return trafo.map(0.5 * (box.bottomRight() + box.topRight()));
                case 4:
                    return trafo.map(box.bottomRight());
                case 5:
                    return trafo.map(0.5 * (box.bottomLeft() + box.bottomRight()));
                case 6:
                    return trafo.map(box.bottomLeft());
                case 7:
                    return trafo.map(0.5 * (box.bottomLeft() + box.topLeft()));
                case 8: // rotation marker
                    trafo.map(QPointF(
                        0.5 * (box.topLeft().x() + box.topRight().x()),
                        0.5 * (box.topLeft().y() + box.topRight().y()) - 5));
                }
            }
        }
        case ito::Shape::Rectangle:
        case ito::Shape::Square: {
            QRectF box(d->m_shape.rbasePoints()[0], d->m_shape.rbasePoints()[1]);
            bool keepAspect = d->m_shape.type() == ito::Shape::Circle;
            QTransform &trafo = d->m_shape.rtransform();

            if (keepAspect)
            {
                switch (index)
                {
                    case -1: // label position
                    case 0:
                        return trafo.map(box.topLeft());
                    case 1:
                        return trafo.map(box.topRight());
                    case 2:
                        return trafo.map(box.bottomRight());
                    case 3:
                        return trafo.map(box.bottomLeft());
                    case 4: //rotation marker
                        return trafo.map(QPointF(0.5 * (box.topLeft().x() + box.topRight().x()), 0.5 * (box.topLeft().y() + box.topRight().y()) - 5));
                }
            }
            else
            {
                switch (index)
                {
                    case -1: // label position
                    case 0:
                        return trafo.map(box.topLeft());
                    case 1:
                        return trafo.map(0.5 * (box.topRight() + box.topLeft()));
                    case 2:
                        return trafo.map(box.topRight());
                    case 3:
                        return trafo.map(0.5 * (box.bottomRight() + box.topRight()));
                    case 4:
                        return trafo.map(box.bottomRight());
                    case 5:
                        return trafo.map(0.5 * (box.bottomLeft() + box.bottomRight()));
                    case 6:
                        return trafo.map(box.bottomLeft());
                    case 7:
                        return trafo.map(0.5 * (box.bottomLeft() + box.topLeft()));
                    case 8: //rotation marker
                        trafo.map(QPointF(0.5 * (box.topLeft().x() + box.topRight().x()), 0.5 * (box.topLeft().y() + box.topRight().y()) - 5));
                }
            }
        }
    }

    return QPointF();
}

//----------------------------------------------------------------------------------------------------------------------------------
bool DrawItem::shapeMoveTo(const QPointF &marker1ScaleCoordinate)
{
    bool moveable = !(d->m_shape.flags() & ito::Shape::MoveLock) && d->m_modificationModes.testFlag(ItomQwtPlotEnums::Move);
    if (moveable)
    {
        d->m_shape.point1MoveTo(marker1ScaleCoordinate);
        setShape(d->m_shape);
        return true;
    }
    return false;
}

//----------------------------------------------------------------------------------------------------------------------------------
bool DrawItem::shapeResizeOrRotate(int hitTypeAndMarkerIndex, const QPointF &markerScaleCoordinate, const Qt::KeyboardModifiers &modifiers /*= Qt::NoModifier*/)
{
    bool resizeable = !(d->m_shape.flags() & ito::Shape::ResizeLock) && d->m_modificationModes.testFlag(ItomQwtPlotEnums::Resize);
    bool rotatable = !(d->m_shape.flags() & ito::Shape::RotateLock) && d->m_modificationModes.testFlag(ItomQwtPlotEnums::Rotate);

    int hitType = hitNone;
    int markerIndex = -1;
    const QTransform &trafo = d->m_shape.rtransform();
    QTransform invTrafo = trafo.inverted();
    QPolygonF &basePoints = d->m_shape.rbasePoints();
    bool success = false;

    if (hitTypeAndMarkerIndex >= hitResize)
    {
        hitType = hitResize;
        markerIndex = hitTypeAndMarkerIndex - hitResize;
    }
    else if (hitTypeAndMarkerIndex <= hitRotation)
    {
        hitType = hitRotation;
        markerIndex = hitRotation - hitTypeAndMarkerIndex;
    }

    if (resizeable && hitType == hitResize)
    {
        switch (d->m_shape.type())
        {
            case ito::Shape::Line:
            {
                QPointF newPoint = invTrafo.map(markerScaleCoordinate);
                if (modifiers & Qt::ControlModifier)
                {
                    if (markerIndex == 0)
                    {
                        QLineF line(basePoints[1], newPoint);
                        double angle = 45.0 * qRound(line.angle() / 45.0);
                        line.setAngle(angle);
                        basePoints[0] = line.p2();
                        success = true;
                    }
                    else if (markerIndex == 1)
                    {
                        QLineF line(basePoints[0], newPoint);
                        double angle = 45.0 * qRound(line.angle() / 45.0);
                        line.setAngle(angle);
                        basePoints[1] = line.p2();
                        success = true;
                    }
                }
                else if (markerIndex == 0 || markerIndex == 1)
                {
                    basePoints[markerIndex] = newPoint;
                    success = true;
                }
            }
            break;

            case ito::Shape::Point:
            case ito::Shape::MultiPointPick:
            {
                if (markerIndex >= 0 && markerIndex < basePoints.size())
                {
                    basePoints[markerIndex - 1] = invTrafo.map(markerScaleCoordinate);
                    success = true;
                }
            }
            break;
            case ito::Shape::Polygon:
            {
                if (markerIndex >= 0 && markerIndex <= basePoints.size())
                {
                    basePoints[markerIndex % basePoints.size()] = invTrafo.map(markerScaleCoordinate);
                    success = true;
                }
            }
            break;

            case ito::Shape::Rectangle:
            case ito::Shape::Ellipse:
            {
                QPointF newPoint = invTrafo.map(markerScaleCoordinate);
                QRectF baseRect(basePoints[0], basePoints[1]);
                success = true;

                switch (markerIndex)
                {
                    case 0: //top, left
                        baseRect.setTopLeft(newPoint);
                    break;

                    case 1: //top, center
                        baseRect.setTop(newPoint.y());
                    break;

                    case 2: //top, right
                        baseRect.setTopRight(newPoint);
                    break;

                    case 3: //right, center
                        baseRect.setRight(newPoint.x());
                    break;

                    case 4: //bottom, right
                        baseRect.setBottomRight(newPoint);
                    break;

                    case 5: //bottom, center
                        baseRect.setBottom(newPoint.y());
                    break;

                    case 6: //bottom, left
                        baseRect.setBottomLeft(newPoint);
                    break;

                    case 7: //left, center
                        baseRect.setLeft(newPoint.x());
                    break;

                    default:
                        success = false;
                    break;
                }

                if (success)
                {
                    basePoints[0] = baseRect.topLeft();
                    basePoints[1] = baseRect.bottomRight();
                }
            }
            break;

            case ito::Shape::Square:
            case ito::Shape::Circle:
            {
                QPointF newPoint = invTrafo.map(markerScaleCoordinate);
                QRectF baseRect(basePoints[0], basePoints[1]);
                QPointF diff = newPoint - baseRect.center();
                qreal sign_x = (diff.x() >= 0 ? 1.0 : -1.0);
                qreal sign_y = (diff.y() >= 0 ? 1.0 : -1.0);
                qreal equal_diff = 0.5*(std::abs(diff.x()) + std::abs(diff.y()));
                newPoint = baseRect.center() + QPointF(sign_x * equal_diff, sign_y * equal_diff);
                success = true;

                switch (markerIndex)
                {
                    case 0: //top, left
                        baseRect.setTopLeft(newPoint);
                    break;

                    case 1: //top, right
                        baseRect.setTopRight(newPoint);
                    break;

                    case 2: //bottom, right
                        baseRect.setBottomRight(newPoint);
                    break;

                    case 3: //bottom, left
                        baseRect.setBottomLeft(newPoint);
                    break;

                    default:
                        success = false;
                    break;
                }

                if (success)
                {
                    basePoints[0] = baseRect.topLeft();
                    basePoints[1] = baseRect.bottomRight();
                }
            }
            break;
        }
    }
    else if (rotatable && hitType == hitRotation)
    {
        switch (d->m_shape.type())
        {
            case ito::Shape::Rectangle:
            case ito::Shape::Ellipse:
            {
                if ((markerIndex == 8 && resizeable) || markerIndex == 0)
                {
                    // rotation
                    QPointF diffScreen;
                    QPointF centerScale = d->m_shape.centerPoint();

                    QwtPlot *canvas = plot();
                    if (canvas)
                    {

                        diffScreen = QPointF(canvas->transform(QwtPlot::xBottom, markerScaleCoordinate.x()), canvas->transform(QwtPlot::yLeft, markerScaleCoordinate.y())) - \
                            QPointF(canvas->transform(QwtPlot::xBottom, centerScale.x()), canvas->transform(QwtPlot::yLeft, centerScale.y()));
                    }
                    else
                    {
                        diffScreen = markerScaleCoordinate - centerScale;
                    }

					float angle = - (+atan2(diffScreen.y(), diffScreen.x()) - (M_PI / 2)) - d->m_shape.rotationAngleRad();

                    //qDebug() << d->m_shape.rotationAngleRad() << (+atan2(diffScreen.y(), diffScreen.x()) + (M_PI / 2)) << angle;


                    if (modifiers & Qt::ShiftModifier)
                    {
                        //round to 22.5deg steps
						angle += d->m_shape.rotationAngleRad();
                        float stepSize = (22.5 * (float)M_PI / 180.0);
                        angle = stepSize * (qRound(angle / stepSize));
						angle -= d->m_shape.rotationAngleRad();
                    }

                    d->m_shape.rotateByCenterRad(angle);

                    success = true;
                }
            }
            break;

            case ito::Shape::Square:
            case ito::Shape::Circle:
            {
                if ((markerIndex == 4 && resizeable) || markerIndex == 0)
                {
                    // rotation
					QPointF diffScreen;
					QPointF centerScale = d->m_shape.centerPoint();

					QwtPlot *canvas = plot();
					if (canvas)
					{

						diffScreen = QPointF(canvas->transform(QwtPlot::xBottom, markerScaleCoordinate.x()), canvas->transform(QwtPlot::yLeft, markerScaleCoordinate.y())) - \
							QPointF(canvas->transform(QwtPlot::xBottom, centerScale.x()), canvas->transform(QwtPlot::yLeft, centerScale.y()));
					}
					else
					{
						diffScreen = markerScaleCoordinate - centerScale;
					}

					float angle = -(+atan2(diffScreen.y(), diffScreen.x()) - (M_PI / 2)) - d->m_shape.rotationAngleRad();

					//qDebug() << d->m_shape.rotationAngleRad() << (+atan2(diffScreen.y(), diffScreen.x()) + (M_PI / 2)) << angle;


					if (modifiers & Qt::ShiftModifier)
					{
						//round to 22.5deg steps
						angle += d->m_shape.rotationAngleRad();
						float stepSize = (22.5 * (float)M_PI / 180.0);
						angle = stepSize * (qRound(angle / stepSize));
						angle -= d->m_shape.rotationAngleRad();
					}

					d->m_shape.rotateByCenterRad(angle);

					success = true;
                }
            }
            break;
        }
    }

    if (success)
    {
        setShape(d->m_shape);
    }

    return success;
}

//----------------------------------------------------------------------------------------------------------------------------------
void DrawItem::setColor(const QColor &markerColor, const QColor &lineColor, const QColor &markerColor2 /*= QColor() */)
{
    if (d->m_markerColor != markerColor || d->m_lineColor != lineColor)
    {
        d->m_markerColor = markerColor;

        if (d->m_autoColor)
        {
            d->m_lineColor = lineColor;
        }

        if (markerColor2 != QColor())
        {
            d->m_markerColor2 = markerColor2;
        }
        else
        {
            d->m_markerColor2 = markerColor;
        }

        if (d->m_fillBrush.style() != Qt::NoBrush)
        {
            int alpha = d->m_fillBrush.color().alpha();
            QColor fillColor = d->m_lineColor;
            fillColor.setAlpha(alpha);
            d->m_fillBrush.setColor(fillColor);
        }

        if (d->m_fillBrushSelected.style() != Qt::NoBrush)
        {
            int alpha = d->m_fillBrushSelected.color().alpha();
            QColor fillColor = d->m_lineColor;
            fillColor.setAlpha(alpha);
            d->m_fillBrushSelected.setColor(fillColor);
        }

        if (d->m_shape.type() != ito::Shape::Point && \
            d->m_shape.type() != ito::Shape::MultiPointPick)
        {
            //delete markers, since their color might be changed
            for (int n = 0; n < d->m_marker.size(); n++)
            {
                d->m_marker[n]->detach();
                delete d->m_marker[n];
            }
            d->m_marker.clear();
        }
    }

    setSelected(d->m_selected);
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
        case ito::Shape::MultiPointPick:
        {
            int num = shape.basePoints().size();
            d->m_point1 = shape.transform().map(shape.rbasePoints()[0]);
            //path.moveTo(d->m_point1);
            //path.lineTo(d->m_point1);

            //delete markers, since deselected
            for (int n = num; n < d->m_marker.size(); n++)
            {
                d->m_marker[n]->detach();
                delete d->m_marker[n];
                d->m_marker.pop_back();
            }

            QwtPlotMarker *marker;

            while (d->m_marker.size() < num)
            {
                marker = new QwtPlotMarker();
                marker->setLinePen(QPen(d->m_markerColor));
                marker->setSymbol(new QwtSymbol(QwtSymbol::Triangle, QBrush(d->m_markerColor), QPen(QBrush(d->m_markerColor), 1), QSize(9, 9)));
                marker->attach(d->m_pparent);
                d->m_marker.append(marker);
            }

            for (int i = 0; i < num; ++i)
            {
                d->m_marker[i]->setXValue(shape.transform().map(shape.rbasePoints()[i]).x());
                d->m_marker[i]->setYValue(shape.transform().map(shape.rbasePoints()[i]).y());
                d->m_marker[i]->setVisible(true);
            }
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
                QRectF rect(shape.basePoints()[0], shape.basePoints()[1]);
                QPainterPath tmpPath;
                tmpPath.addRect(rect);

                path.addPath(shape.transform().map(tmpPath));
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
                QRectF rect(shape.basePoints()[0], shape.basePoints()[1]);
                QPainterPath tmpPath;
                tmpPath.addEllipse(rect);

                path.addPath(shape.transform().map(tmpPath));
            }
        break;

        case ito::Shape::Polygon:
        {
            QPolygonF basePoints = shape.basePoints();
            if (!shape.unclosed() && basePoints.size() > 1)
            {
                basePoints.append(basePoints[0]);
            }

            QPolygonF transformedPoints = shape.transform().map(basePoints);

            if (transformedPoints.size() > 0)
            {
                path.moveTo(transformedPoints[0]);
                for (int nl = 1; nl < transformedPoints.size(); ++nl)
                {
                    path.lineTo(transformedPoints[nl]);
                }
            }
        }
        break;

        default:
            retVal += ito::RetVal(ito::retError, 0, QObject::tr("invalid geometric shape type").toLatin1().data());
        break;
    }

    if (!retVal.containsError())
    {
        d->m_shape = shape;
        QwtPlotShapeItem::setShape(path);
        QwtPlotShapeItem::setTitle(d->m_shape.name());

        d->m_autoColor = true;
        if (shape.color().isValid())
        {
            d->m_autoColor = false;
            d->m_lineColor = shape.color();
        }

        setSelected(d->m_selected, d->m_currentMarker); //to possibly adjust the position of markers
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
    if (d->m_rotationMarkerIndex >= 0)
    {
        QPointF diffScale = d->m_rotationMarkerPoint - d->m_rotationMarkerBasePoint;
        QPointF diffPixel = QPointF(xMap.transform(diffScale.x()) - xMap.transform(0.0), yMap.transform(diffScale.y()) - yMap.transform(0.0));
        qreal lenPx = qSqrt(diffPixel.x() * diffPixel.x() + diffPixel.y() * diffPixel.y());
        diffPixel = diffPixel / lenPx * 25.0; //25.0 is the distance of the rotation marker from the shape in px
        diffScale = QPointF(xMap.invTransform(diffPixel.x()) - xMap.invTransform(0.0), yMap.invTransform(diffPixel.y()) - yMap.invTransform(0.0));
        d->m_marker[d->m_rotationMarkerIndex]->setXValue(d->m_rotationMarkerBasePoint.x() + diffScale.x());
        d->m_marker[d->m_rotationMarkerIndex]->setYValue(d->m_rotationMarkerBasePoint.y() + diffScale.y());
    }

    QwtPlotShapeItem::draw(painter, xMap, yMap, canvasRect);

	//label will only be printed, if the property 'geometricShapesLabelsVisible' is true, if the label is empty (then a default name is chosen) or
	//if the trimmed label name is not empty. Hence, if you choose a label name consisting of spaces, the label is not printed!
    if(d->m_labelVisible)
    {
		QString name = d->m_shape.name();

		if (name.isEmpty())
		{
			name = QString("(%1)").arg(d->m_shape.index());
		}

		if (!name.isEmpty() && !name.trimmed().isEmpty())
		{
			QRectF myRect(0, 0, 0, 0);

			QwtText label(d->m_shape.name());
			label.setFont(d->m_labelFont);
			label.setColor(d->m_labelTextColor); //remove this to set the color equal to the shape color

			const QSizeF textSize = label.textSize(painter->font());
			const QPointF textSizeScales = QPointF(textSize.width() * fabs(xMap.sDist() / xMap.pDist()), textSize.height() * fabs(yMap.sDist() / yMap.pDist()));
			QPointF labelPosition = getMarkerPosScale(-1);

			if (labelPosition.isNull() == false)
			{
                myRect = QRectF(labelPosition - textSizeScales, labelPosition);

				QRectF cRect = QwtScaleMap::transform(xMap, yMap, myRect);
				//cRect.translate(QPoint(-10, -10));
				cRect.adjust(-12, -8, -6, -4);

				if (!label.isEmpty())
				{
					painter->fillRect(cRect, d->m_labelBrush);
					label.draw(painter, cRect);
				}
			}
		}
    }

    return;
}

//----------------------------------------------------------------------------------------------------------------------------------
bool DrawItem::hitLine(const QPointF &point_transformed, const QLineF &line, double tol_x, double tol_y) const
{
    //check small circles around end points
    if (std::abs(point_transformed.x() - line.p1().x()) <= tol_x && std::abs(point_transformed.y() - line.p1().y()) <= tol_y)
    {
        return true;
    }
    else if (std::abs(point_transformed.x() - line.p2().x()) <= tol_x && std::abs(point_transformed.y() - line.p2().y()) <= tol_y)
    {
        return true;
    }

    //check line between end points
    QLineF normal = line.normalVector();
    normal.translate(-line.p1() + point_transformed);

    QPointF intersection;
#if (QT_VERSION >= QT_VERSION_CHECK(5, 14, 0))
    line.intersects(normal, &intersection);
#else
    line.intersect(normal, &intersection);
#endif

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
//returns -1 if not hit, 0: if contour hit for moving (only if moving is allowed) or 1..8 if markers hit for resizing (only if resizing is allowed), -2 if rotation marker hit (only if rotating is allowed)
int DrawItem::hitEdge(const QPointF &point, double tol_x, double tol_y) const
{
    ito::RetVal retVal;
    const ito::Shape &shape = d->m_shape;
    QPointF point_trafo = shape.transform().inverted().map(point); //transform point to base coordinate system of shape

    unsigned int flags = d->m_shape.flags();
    bool moveable = !(flags & ito::Shape::MoveLock) && d->m_modificationModes.testFlag(ItomQwtPlotEnums::Move);
    bool resizeable = !(flags & ito::Shape::ResizeLock) && d->m_modificationModes.testFlag(ItomQwtPlotEnums::Resize);
    bool rotatable = !(flags & ito::Shape::RotateLock) && d->m_modificationModes.testFlag(ItomQwtPlotEnums::Rotate);

    int result = hitNone; //default: nothing hit

    switch (d->m_shape.type())
    {
        case ito::Shape::Point:
        {
            if (moveable)
            {
                QLineF line(point_trafo, shape.basePoints()[0]);
                if ((std::abs(line.dx()) <= tol_x) && (std::abs(line.dy()) <= tol_y))
                {
                    result = hitMove;
                }
            }
        }
        break;

        case ito::Shape::Line:
        {
            QLineF line(shape.basePoints()[0], shape.basePoints()[1]);
            if (hitLine(point_trafo, line, tol_x, tol_y))
            {
                if (resizeable)
                {
                    qreal dist_x, dist_y;

                    for (int i = 0; i < d->m_marker.size(); ++i)
                    {
                        dist_x = std::abs(d->m_marker[i]->xValue() - point.x());
                        dist_y = std::abs(d->m_marker[i]->yValue() - point.y());
                        if ((dist_x <= tol_x) && (dist_y <= tol_y))
                        {
                            result = (i + hitResize);
                            break;
                        }
                    }

                    //if not yet selected, but not moveable. The user clicked any part of the
                    //contour but not the corner squares, nevertheless, the item should be
                    //selected for any resize operation.
                    if (result == hitNone && !d->m_selected && !moveable)
                    {
                        result = hitMove;
                    }
                }

                if (result == hitNone && moveable)
                {
                    result = hitMove;
                }
            }
        }
        break;

        case ito::Shape::Rectangle:
        case ito::Shape::Square:
        {
            QPolygonF contour = shape.contour(/*applyTrafo = */ false);

            // rotation marker must be checked separately
            if (rotatable && d->m_selected && d->m_marker.size() > 0)
            {
                if (std::abs(d->m_marker[d->m_marker.size() - 1]->xValue() - point.x()) < tol_x
                    && std::abs(d->m_marker[d->m_marker.size() - 1]->yValue() - point.y()) < tol_y)
                {
                    result = hitRotation - (d->m_marker.size() - 1); //rotation marker clicked
                }
            }

            if (result == hitNone && \
                (hitLine(point_trafo, QLineF(contour[0], contour[1]), tol_x, tol_y) || \
                hitLine(point_trafo, QLineF(contour[1], contour[2]), tol_x, tol_y) || \
                hitLine(point_trafo, QLineF(contour[2], contour[3]), tol_x, tol_y) || \
                hitLine(point_trafo, QLineF(contour[3], contour[0]), tol_x, tol_y)))
            {
                if (resizeable)
                {
                    qreal dist_x, dist_y;

                    for (int i = 0; i < d->m_marker.size(); ++i)
                    {
                        dist_x = std::abs(d->m_marker[i]->xValue() - point.x());
                        dist_y = std::abs(d->m_marker[i]->yValue() - point.y());
                        if ((dist_x <= tol_x) && (dist_y <= tol_y))
                        {
                            result = (i + hitResize);
                            break;
                        }
                    }

                    //if not yet selected, but not moveable. The user clicked any part of the
                    //contour but not the corner squares, nevertheless, the item should be
                    //selected for any resize operation.
                    if (result == hitNone && !d->m_selected && !moveable)
                    {
                        result = hitMove;
                    }
                }

                if (result == hitNone && moveable)
                {
                    result = hitMove;
                }
                else if (result == hitNone && rotatable)
                {
                    result = hitRotation;
                }
            }
        }
        break;

        case ito::Shape::Ellipse:
        case ito::Shape::Circle:
        {
            bool hit_edge = false;

			// rotation marker must be checked separately
			if (rotatable &&  d->m_selected && d->m_marker.size() > 0)
			{
				if (std::abs(d->m_marker[d->m_marker.size() - 1]->xValue() - point.x()) < tol_x
					&& std::abs(d->m_marker[d->m_marker.size() - 1]->yValue() - point.y()) < tol_y)
				{
					result = hitRotation - (d->m_marker.size() - 1); //rotation marker clicked
				}
			}

			//if selected, some markers are out of the circle, therefore their area must be checked separately
			if (result == hitNone && resizeable && d->m_selected)
			{
				for (int i = 0; i < d->m_marker.size() - 1; ++i)
				{
					if (std::abs(d->m_marker[i]->xValue() - point.x()) < tol_x && std::abs(d->m_marker[i]->yValue() - point.y()) < tol_y)
					{
						result = (i + hitResize);
						break;
					}
				}
			}

			if (result == hitNone)
			{
				//inverse transform point
				QPointF size = shape.basePoints()[1] - shape.basePoints()[0];
				double a = std::abs(size.x()) / 2.0 + tol_x;
				double b = std::abs(size.y()) / 2.0 + tol_y;
				QPointF center = shape.baseCenterPoint();
				double x = point_trafo.x() - center.x();
				double y = point_trafo.y() - center.y();

				if ((((x*x) / (a*a)) + ((y*y) / (b*b))) <= 1.0)
				{
					a -= 2 * tol_x;
					b -= 2 * tol_y;
					if ((((x*x) / (a*a)) + ((y*y) / (b*b))) > 1.0)
					{
                        if (moveable)
                        {
						    result = hitMove;
                        }
                        else if (rotatable)
                        {
                            result = hitRotation;
                        }
					}
				}
			}
        }
        break;

        case ito::Shape::MultiPointPick:
        case ito::Shape::Polygon:
        {
            if (moveable || resizeable)
            {
                QLineF line;
                bool hit_edge = false;
                const QPolygonF &basePoints = shape.basePoints();

                for (int i = 0; i < basePoints.size(); ++i)
                {
                    line = QLineF(point_trafo, basePoints[i]);
                    if ((std::abs(line.dx()) <= tol_x) && (std::abs(line.dy()) <= tol_y))
                    {
                        hit_edge = true;
                        break;
                    }
                    int closed = shape.unclosed() ? 1 : 0;
                    if (i < shape.basePoints().size() - closed)
                    {
                        if (hit_edge = hitLine(point_trafo, QLineF(basePoints[i], basePoints[(i + 1) % basePoints.size()]), tol_x, tol_y))
                            break;
                    }
                }

                if (hit_edge)
                {
                    if (resizeable)
                    {
                        qreal dist_x, dist_y;

                        for (int i = 0; i < d->m_marker.size(); ++i)
                        {
                            dist_x = std::abs(d->m_marker[i]->xValue() - point.x());
                            dist_y = std::abs(d->m_marker[i]->yValue() - point.y());
                            if ((dist_x <= tol_x) && (dist_y <= tol_y))
                            {
                                result = (i + hitResize);
                                break;
                            }
                        }

                        //if not yet selected, but not moveable. The user clicked any part of the
                        //contour but not the corner squares, nevertheless, the item should be
                        //selected for any resize operation.
                        if (result == hitNone && !d->m_selected && !moveable)
                        {
                            result = hitMove;
                        }
                    }

                    if (result == hitNone && moveable)
                    {
                        result = hitMove;
                    }
                }
            }
        }
        break;

        default:
        break;
    }


    return result;
}

//----------------------------------------------------------------------------------------------------------------------------------
