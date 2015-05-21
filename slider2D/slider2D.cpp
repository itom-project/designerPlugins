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

/**
@author Mattia Basaglia
@section License
    Copyright (C) 2013-2015 Mattia Basaglia
    This software is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.
    This software is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.
    You should have received a copy of the GNU General Public License
    along with Color Widgets.  If not, see <http://www.gnu.org/licenses/>.
*/

#include "slider2D.h"

#include <QMouseEvent>
#include <QPainter>
#include <QSlider>
#include <QHBoxLayout>
#include <QLineF>
#include <qmath.h>
#include <cmath>

enum Mouse_Status
{
    nothing,
    dragSquare,
    dragX,
    dragY
};


//static const double selector_radius = 6;


class Slider2D::Private
{
private:
    Slider2D * const w;

public:

    qreal xVal, yVal;
    QPoint xRange;
    QPoint yRange;
    Mouse_Status mouseStatus;
    QImage innerSelector;
    QImage leftSliderBackgr;
    QImage rightSliderBackgr;



    Private(Slider2D *widget)
        : w(widget), xVal(0), yVal(0),
        xRange(0,1), yRange(0,1), mouseStatus(nothing)
    { }


    /// Calculate the edge length of the inner square

    qreal squareSize() const
    {
        return qMin(w->geometry().width()*0.78, w->geometry().height()*0.78);
    }

    /// return line from center to given point
    QLineF lineToPoint(const QPoint &p) const
    {
        return QLineF (w->geometry().width()/2, w->geometry().height()/2, p.x(), p.y());
    }

    void renderBackground()
    {
        int sz = squareSize();
        innerSelector = QImage(sz, sz, QImage::Format_RGB32);
        innerSelector.fill(qRgb(200, 200, 200));

        leftSliderBackgr = QImage(0.15*sz, sz, QImage::Format_RGB32);
        leftSliderBackgr.fill(qRgb(220, 220, 220));

        rightSliderBackgr = QImage(sz, 0.15*sz, QImage::Format_RGB32);
        rightSliderBackgr.fill(qRgb(220, 220, 220));
    }



    /// Offset of the selector image
    QPointF selectorImageOffset()
    {
        return QPointF(1.1*(-squareSize()/2),1.1*(-squareSize()/2));
    }



    /// Rotation of the selector image
    qreal selectorImageAngle()
    {
        return 0;

    }


};

Slider2D::Slider2D(QWidget *parent) :
    QWidget(parent), p(new Private(this))
{

}

Slider2D::~Slider2D()
{
    delete p;
}


//QSize Slider2D::sizeHint() const
//{
//    return QSize(p->wheel_width*5, p->wheel_width*5);
//}


qreal Slider2D::xVal() const
{
    return p->xVal;
}

qreal Slider2D::yVal() const
{
    return p->yVal; //color().valueF();
}

QPoint Slider2D::xRange() const
{
    return p->xRange;
}

QPoint Slider2D::yRange() const
{
    return p->yRange;
}

void Slider2D::setXRange(QPoint xr)
{
    p->xRange = xr;
    update();
}

void Slider2D::setYRange(QPoint yr)
{
    p->yRange = yr;
    update();
}

void Slider2D::paintEvent(QPaintEvent * )
{
    //QSlider *slider = new QSlider(Qt::Horizontal, this);
    qreal xValRanged = (p->xVal - p->xRange.x())/(p->xRange.y() - p->xRange.x());        // transforms coordinate of range (xmin, xmax) to range (0,1)
    qreal yValRanged = (p->yVal - p->yRange.y())/(p->yRange.x() - p->yRange.y());


    QPainter painter(this);

    painter.setRenderHint(QPainter::Antialiasing);
    painter.translate(geometry().width()/2,geometry().height()/2);

    

    if(p->innerSelector.isNull())
        p->renderBackground();

    painter.translate(p->selectorImageOffset());

    QPointF selector2DPosition;
    qreal side = p->squareSize();
    selector2DPosition = QPointF(xValRanged*side, yValRanged*side);

    QRectF text_rect_1 = QRectF(side*1.03, yValRanged*side - 0.075*side, 0.15*side, 0.15*side);
    QRectF text_rect_2 = QRectF(xValRanged*side - 0.075*side, side*1.03 , 0.15*side, 0.15*side);

    QPainterPath path_tri_1;
    path_tri_1.moveTo (side, yValRanged*side);
    path_tri_1.lineTo (side*1.03, yValRanged*side - 0.075*side);
    path_tri_1.lineTo (side*1.03,   yValRanged*side + 0.075*side);
    path_tri_1.lineTo (side, yValRanged*side);

    QPainterPath path_tri_2;
    path_tri_2.moveTo (xValRanged*side, side);
    path_tri_2.lineTo (xValRanged*side - 0.075*side, side*1.03);
    path_tri_2.lineTo (xValRanged*side + 0.075*side, side*1.03);
    path_tri_2.lineTo (xValRanged*side, side);


    painter.drawImage(0,0,p->innerSelector);
    painter.drawImage(side*1.03,0,p->leftSliderBackgr);

    painter.drawText(text_rect_1, Qt::AlignCenter | Qt::TextDontClip, QString::number(p->yVal,'g',3));
    painter.drawRect(text_rect_1);

    painter.drawImage(0,side*1.03,p->rightSliderBackgr);
    painter.drawText(text_rect_2, Qt::AlignCenter | Qt::TextDontClip, QString::number(p->xVal,'g',3));
    painter.drawRect(text_rect_2);

    painter.setClipping(false);

    painter.setPen(QPen(Qt::black, 2)); 
    painter.setBrush(Qt::NoBrush);
    painter.drawEllipse(selector2DPosition, 0.03*side, 0.03*side);

    painter.setPen(QPen(Qt::black, 2));  
    painter.drawLine(0, yValRanged*side, side, yValRanged*side);
    painter.drawLine(xValRanged*side, 0, xValRanged*side, side);
    painter.fillPath (path_tri_1, QBrush (QColor ("black")));
    painter.fillPath (path_tri_2, QBrush (QColor ("black")));

}

void Slider2D::mouseMoveEvent(QMouseEvent *ev)
{
    if(p->mouseStatus == dragSquare)
    {
        QLineF globMouseLn = p->lineToPoint(ev->pos());
        QLineF centerMouseLn ( QPointF(0,0),
                                 globMouseLn.p2() - globMouseLn.p1() );

     //   centerMouseLn.setAngle(centerMouseLn.angle());
        centerMouseLn.setP2(centerMouseLn.p2()-p->selectorImageOffset());

        qreal xval_old = qBound(0.0, centerMouseLn.x2()/p->squareSize(), 1.0);
        qreal yval_old = qBound(0.0, centerMouseLn.y2()/p->squareSize(), 1.0);



        p->xVal =   xval_old*(p->xRange.y() - p->xRange.x()) + p->xRange.x();  // changes coordinate from range (0, 1) to range (xmin, xmax)
        p->yVal =   yval_old*(p->yRange.x() - p->yRange.y()) + p->yRange.y();


        emit xValChanged(xVal());
        emit yValChanged(yVal());
        update();
    }

    if(p->mouseStatus == dragY)
    {
        QLineF globMouseLn = p->lineToPoint(ev->pos());
        QLineF centerMouseLn ( QPointF(0,0),
                                 globMouseLn.p2() - globMouseLn.p1() );

       // centerMouseLn.setAngle(centerMouseLn.angle());
        centerMouseLn.setP2(centerMouseLn.p2()-p->selectorImageOffset());

        qreal yval_old = qBound(0.0, centerMouseLn.y2()/p->squareSize(), 1.0);  
        
        p->yVal =   yval_old*(p->yRange.x() - p->yRange.y()) + p->yRange.y();

        emit yValChanged(yVal());
        update();
    }

    if(p->mouseStatus == dragX)
    {
        QLineF globMouseLn = p->lineToPoint(ev->pos());
        QLineF centerMouseLn ( QPointF(0,0),
                                 globMouseLn.p2() - globMouseLn.p1() );

       // centerMouseLn.setAngle(centerMouseLn.angle());
        centerMouseLn.setP2(centerMouseLn.p2()-p->selectorImageOffset());


        qreal xval_old = qBound(0.0, centerMouseLn.x2()/p->squareSize(), 1.0);
        p->xVal =   xval_old*(p->xRange.y() - p->xRange.x()) + p->xRange.x();

        emit xValChanged(xVal());
        update();
    }
}

void Slider2D::mousePressEvent(QMouseEvent *ev)
{
    
    if ( ev->buttons() & Qt::LeftButton )
    {
        qreal side = p->squareSize();
        QPointF offset = p->selectorImageOffset();
        QLineF ray = p->lineToPoint(ev->pos());
        if ( abs(ray.dx()) <= (p->squareSize()/2)*1.1 && abs(ray.dy()) <= (p->squareSize()/2)*1.1)
            QLineF ray_ = p->lineToPoint(ev->pos());
            if (ev->x() > side*1.15)  //((abs(ev->x() - side*0.13 - p->xVal*side)> 0.1*side) && (abs(ev->y() - side*0.13 - p->yVal*side) < 0.06*side)) || 
                p->mouseStatus = dragY;
            else if (ev->y() > side*1.12) //(((abs(ev->y() - side*0.2 - p->yVal*side)> 0.1*side) && (abs(ev->x() - side*0.2 - p->xVal*side) < 0.06*side)) || 
                p->mouseStatus = dragX;
            else
                p->mouseStatus = dragSquare;
    }
}

void Slider2D::mouseReleaseEvent(QMouseEvent *ev)
{
    mouseMoveEvent(ev);
    p->mouseStatus = nothing;
}

void Slider2D::resizeEvent(QResizeEvent *)
{
    p->renderBackground();
}



void Slider2D::setX(qreal s)
{
    qreal low = p->xRange.x();
    qreal high = p->xRange.y();
    p->xVal = qBound(low, s, high);
    update();
}

void Slider2D::setY(qreal v)
{    
    qreal low = p->yRange.x();
    qreal high = p->yRange.y();
    p->yVal = qBound(low, v, high);
    update();
}

