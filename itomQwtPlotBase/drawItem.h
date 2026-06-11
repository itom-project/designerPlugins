/* ********************************************************************
 *   itom measurement system
 *   URL: http://www.uni-stuttgart.de/ito
 *   Copyright (C) 2018, Institut für Technische Optik (ITO),
 *   Universität Stuttgart, Germany
 *
 *   This file is part of itom.
 *
 *   itom is free software: you can redistribute it and/or modify
 *   it under the terms of the GNU General Public License as published by
 *   the Free Software Foundation, either version 3 of the License, or
 *   (at your option) any later version.
 *
 *   itom is distributed in the hope that it will be useful,
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 *   GNU General Public License for more details.
 *
 *   You should have received a copy of the GNU General Public License
 *   along with itom. If not, see <http://www.gnu.org/licenses/>.
 *********************************************************************** */

#ifndef DRAWITEM_H
#define DRAWITEM_H

#include "itomQwtPlotBase.h"

#include <qwt_plot_canvas.h>
#include <qwt_plot_shapeitem.h>
#include <qwt_plot_marker.h>

#include <qpoint.h>
#include <qpainterpath.h>
#include <qpoint.h>

#include "common/shape.h"
#include "common/retVal.h"
#include "itomQwtPlotEnums.h"

class DrawItemPrivate; //forward declaration

class ITOMQWTPLOTBASE_EXPORT DrawItem : public QwtPlotShapeItem
{
    public:
        enum HitType
        {
            hitRotation = -2, /*rotation are all values <= hitRotation, the marker index, that was clicked to be rotated, is index = hitRotation - type*/
            hitNone = -1,
            hitMove = 0,
            hitResize = 1, /*resize are all values >= hitResize, the marker index, that was clicked to be resized, is index = type - hitResize*/
        };

        explicit DrawItem(const ito::Shape &shape, ItomQwtPlotEnums::ModificationModes &modificationModes, QwtPlot *parent, ito::RetVal *retVal = NULL, bool labelVisible = false);
        virtual ~DrawItem();
        ito::RetVal setShape(const ito::Shape &shape);
        ito::RetVal setShape(const ito::Shape &shape, const QColor &markerColor, const QColor &lineColor);

        void setColor(const QColor &markerColor, const QColor &lineColor, const QColor &markerColor2 = QColor());

        bool getSelected() const;
        void setSelected(const bool selected, const int nMarker = -1);

        QString getLabel() const;
        void setLabel(const QString &label);

        const ito::Shape &getShape() const;
        int getIndex() const;
        bool getAutoColor() const;

        void setModificationModes(const ItomQwtPlotEnums::ModificationModes &modes);

        QPointF getPoint1() const;
        QPointF getPoint2() const;
        QPointF getMarkerPosScale(int index) const;

        bool shapeMoveTo(const QPointF &marker1ScaleCoordinate);
        bool shapeResizeOrRotate(int hitTypeAndMarkerIndex, const QPointF &markerScaleCoordinate, const Qt::KeyboardModifiers &modifiers = Qt::NoModifier);

        //returns -1 if not hit, 0: if contour hit for moving (only if moving is allowed) or 1..8 if markers hit for resizing (only if resizing is allowed), -2 if rotation marker hit (only if rotating is allowed)
        //the return values are values from the enumeration HitType (all hits to resize corner points are hitResize + X)
        int hitEdge(const QPointF &point, double tol_x, double tol_y) const;

        bool hitLine(const QPointF &point_transformed, const QLineF &line, double tol_x, double tol_y) const;

        void setLabelVisible(const bool labelVisible);
        bool getLabelVisible() const;

        void setFillOpacity(int opacity, int opacitySelected);

        static QVector<int> idxVec;

        virtual void draw(QPainter *painter,
            const QwtScaleMap &xMap, const QwtScaleMap &yMap,
            const QRectF &canvasRect ) const;

        int getSelectedMarker() const;

    private:
        DrawItemPrivate *d;


};

#endif //DRAWITEM_H
