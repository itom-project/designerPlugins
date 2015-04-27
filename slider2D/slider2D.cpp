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
    Nothing,
    Drag_Square,
	Drag_X,
	Drag_Y
};


static const double selector_radius = 6;


class Slider2D::Private
{
private:
    Slider2D * const w;

public:

    qreal x_val, y_val;
    QPoint x_range;
	QPoint y_range;
    Mouse_Status mouse_status;
    QImage inner_selector;
	QImage left_slider_backgr;
	QImage right_slider_backgr;



    Private(Slider2D *widget)
        : w(widget), x_val(0), y_val(0),
        x_range(0,1), y_range(0,1), mouse_status(Nothing)
    { }


    /// Calculate the edge length of the inner square

    qreal square_size() const
    {
        return qMin(w->geometry().width()*0.7, w->geometry().height()*0.7);
    }

    /// return line from center to given point
    QLineF line_to_point(const QPoint &p) const
    {
        return QLineF (w->geometry().width()/2, w->geometry().height()/2, p.x(), p.y());
    }

    void render_background()
    {
        int sz = square_size();
        inner_selector = QImage(sz, sz, QImage::Format_RGB32);
		inner_selector.fill(qRgb(200, 200, 200));

		left_slider_backgr = QImage(0.15*sz, sz, QImage::Format_RGB32);
		left_slider_backgr.fill(qRgb(220, 220, 220));

		right_slider_backgr = QImage(sz, 0.15*sz, QImage::Format_RGB32);
		right_slider_backgr.fill(qRgb(220, 220, 220));
    }



    /// Offset of the selector image
    QPointF selector_image_offset()
    {
        return QPointF(1.2*(-square_size()/2),1.2*(-square_size()/2));
    }



    /// Rotation of the selector image
    qreal selector_image_angle()
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


qreal Slider2D::x_val() const
{
    return p->x_val;
}

qreal Slider2D::y_val() const
{
	return p->y_val; //color().valueF();
}

QPoint Slider2D::x_range() const
{
    return p->x_range;
}

QPoint Slider2D::y_range() const
{
    return p->y_range;
}

void Slider2D::set_xRange(QPoint xr)
{
	p->x_range = xr;
    update();
}

void Slider2D::set_yRange(QPoint yr)
{
	p->y_range = yr;
    update();
}

void Slider2D::paintEvent(QPaintEvent * )
{
	//QSlider *slider = new QSlider(Qt::Horizontal, this);
	qreal xval_ranged = (p->x_val - p->x_range.x())/(p->x_range.y() - p->x_range.x());		// transforms coordinate of range (xmin, xmax) to range (0,1)
	qreal yval_ranged = (p->y_val - p->y_range.y())/(p->y_range.x() - p->y_range.y());


    QPainter painter(this);

    painter.setRenderHint(QPainter::Antialiasing);
    painter.translate(geometry().width()/2,geometry().height()/2);

	

    if(p->inner_selector.isNull())
        p->render_background();

    painter.translate(p->selector_image_offset());

    QPointF selector2D_position;
	qreal side = p->square_size();
    selector2D_position = QPointF(xval_ranged*side, yval_ranged*side);

	QRectF text_rect_1 = QRectF(side*1.03, yval_ranged*side - 0.075*side, 0.15*side, 0.15*side);
	QRectF text_rect_2 = QRectF(xval_ranged*side - 0.075*side, side*1.03 , 0.15*side, 0.15*side);

	QPainterPath path_tri_1;
	path_tri_1.moveTo (side, yval_ranged*side);
	path_tri_1.lineTo (side*1.03, yval_ranged*side - 0.075*side);
	path_tri_1.lineTo (side*1.03,   yval_ranged*side + 0.075*side);
	path_tri_1.lineTo (side, yval_ranged*side);

	QPainterPath path_tri_2;
	path_tri_2.moveTo (xval_ranged*side, side);
	path_tri_2.lineTo (xval_ranged*side - 0.075*side, side*1.03);
	path_tri_2.lineTo (xval_ranged*side + 0.075*side, side*1.03);
	path_tri_2.lineTo (xval_ranged*side, side);


    painter.drawImage(0,0,p->inner_selector);
	painter.drawImage(side*1.03,0,p->left_slider_backgr);

	painter.drawText(text_rect_1, Qt::AlignCenter | Qt::TextDontClip, QString::number(p->y_val,'g',3));
	painter.drawRect(text_rect_1);

	painter.drawImage(0,side*1.03,p->right_slider_backgr);
	painter.drawText(text_rect_2, Qt::AlignCenter | Qt::TextDontClip, QString::number(p->x_val,'g',3));
	painter.drawRect(text_rect_2);

    painter.setClipping(false);

    painter.setPen(QPen(Qt::black, 2)); 
    painter.setBrush(Qt::NoBrush);
	painter.drawEllipse(selector2D_position, 0.03*side, 0.03*side);

	painter.setPen(QPen(Qt::black, 2));  
	painter.drawLine(0, yval_ranged*side, side, yval_ranged*side);
	painter.drawLine(xval_ranged*side, 0, xval_ranged*side, side);
	painter.fillPath (path_tri_1, QBrush (QColor ("black")));
	painter.fillPath (path_tri_2, QBrush (QColor ("black")));

}

void Slider2D::mouseMoveEvent(QMouseEvent *ev)
{
    if(p->mouse_status == Drag_Square)
    {
        QLineF glob_mouse_ln = p->line_to_point(ev->pos());
        QLineF center_mouse_ln ( QPointF(0,0),
                                 glob_mouse_ln.p2() - glob_mouse_ln.p1() );

     //   center_mouse_ln.setAngle(center_mouse_ln.angle());
        center_mouse_ln.setP2(center_mouse_ln.p2()-p->selector_image_offset());

		qreal xval_old = qBound(0.0, center_mouse_ln.x2()/p->square_size(), 1.0);
		qreal yval_old = qBound(0.0, center_mouse_ln.y2()/p->square_size(), 1.0);



        p->x_val =   xval_old*(p->x_range.y() - p->x_range.x()) + p->x_range.x();  // changes coordinate from range (0, 1) to range (xmin, xmax)
        p->y_val =   yval_old*(p->y_range.x() - p->y_range.y()) + p->y_range.y();


		emit x_valChanged(x_val());
		emit y_valChanged(y_val());
        update();
    }

	if(p->mouse_status == Drag_Y)
    {
        QLineF glob_mouse_ln = p->line_to_point(ev->pos());
        QLineF center_mouse_ln ( QPointF(0,0),
                                 glob_mouse_ln.p2() - glob_mouse_ln.p1() );

       // center_mouse_ln.setAngle(center_mouse_ln.angle());
        center_mouse_ln.setP2(center_mouse_ln.p2()-p->selector_image_offset());

		qreal yval_old = qBound(0.0, center_mouse_ln.y2()/p->square_size(), 1.0);  
        
		p->y_val =   yval_old*(p->y_range.x() - p->y_range.y()) + p->y_range.y();

		emit y_valChanged(y_val());
        update();
    }

	if(p->mouse_status == Drag_X)
    {
        QLineF glob_mouse_ln = p->line_to_point(ev->pos());
        QLineF center_mouse_ln ( QPointF(0,0),
                                 glob_mouse_ln.p2() - glob_mouse_ln.p1() );

       // center_mouse_ln.setAngle(center_mouse_ln.angle());
        center_mouse_ln.setP2(center_mouse_ln.p2()-p->selector_image_offset());


		qreal xval_old = qBound(0.0, center_mouse_ln.x2()/p->square_size(), 1.0);
        p->x_val =   xval_old*(p->x_range.y() - p->x_range.x()) + p->x_range.x();

		emit x_valChanged(x_val());
        update();
    }
}

void Slider2D::mousePressEvent(QMouseEvent *ev)
{
	
    if ( ev->buttons() & Qt::LeftButton )
    {
		qreal side = p->square_size();
		QPointF offset = p->selector_image_offset();
        QLineF ray = p->line_to_point(ev->pos());
        if ( abs(ray.dx()) <= (p->square_size()/2)*1.1 && abs(ray.dy()) <= (p->square_size()/2)*1.1)
			QLineF ray_ = p->line_to_point(ev->pos());
			if (ev->x() > side*1.15)  //((abs(ev->x() - side*0.13 - p->x_val*side)> 0.1*side) && (abs(ev->y() - side*0.13 - p->y_val*side) < 0.06*side)) || 
				p->mouse_status = Drag_Y;
			else if (ev->y() > side*1.12) //(((abs(ev->y() - side*0.2 - p->y_val*side)> 0.1*side) && (abs(ev->x() - side*0.2 - p->x_val*side) < 0.06*side)) || 
				p->mouse_status = Drag_X;
			else
				p->mouse_status = Drag_Square;
    }
}

void Slider2D::mouseReleaseEvent(QMouseEvent *ev)
{
    mouseMoveEvent(ev);
    p->mouse_status = Nothing;
}

void Slider2D::resizeEvent(QResizeEvent *)
{
    p->render_background();
}



void Slider2D::setX(qreal s)
{
	qreal low = p->x_range.x();
	qreal high = p->x_range.y();
    p->x_val = qBound(low, s, high);
    update();
}

void Slider2D::setY(qreal v)
{	
	qreal low = p->y_range.x();
	qreal high = p->y_range.y();
    p->y_val = qBound(low, v, high);
    update();
}

