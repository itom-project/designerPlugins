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
#ifndef Slider2D_HPP
#define Slider2D_HPP

#include <QWidget>
#include <qglobal.h>

/**
 * \brief Display an analog widget that allows the selection of a HSV color
 *
 * It has an outer wheel to select the Hue and an intenal square to select
 * Saturation and Lightness.
 */
class Slider2D : public QWidget
{
    Q_OBJECT

    Q_PROPERTY(qreal xVal READ xVal WRITE setX NOTIFY xValChanged DESIGNABLE true )
    Q_PROPERTY(qreal yVal READ yVal WRITE setY NOTIFY yValChanged DESIGNABLE true )
    Q_PROPERTY(QPoint xRange READ xRange WRITE setXRange DESIGNABLE true )
	Q_PROPERTY(QPoint yRange READ yRange WRITE setYRange DESIGNABLE true )

public:

    explicit Slider2D(QWidget *parent = 0);
    ~Slider2D();


   // QSize sizeHint() const;


    /// Get current xVal in the range [0-1]
    qreal xVal() const;

    /// Get current yVal in the range [0-1]
    qreal yVal() const;

    /// Get the width in pixels of the outer wheel
    QPoint xRange() const;
	QPoint yRange() const;

    /// Set the width in pixels of the outer wheel
    void setXRange(QPoint xRange);
	void setYRange(QPoint yRange);


public slots:

    void setX(qreal s);
    void setY(qreal v);


signals:

	void xValChanged(qreal);
	void yValChanged(qreal);


protected:
    void paintEvent(QPaintEvent *);
    void mouseMoveEvent(QMouseEvent *);
    void mousePressEvent(QMouseEvent *);
    void mouseReleaseEvent(QMouseEvent *);
    void resizeEvent(QResizeEvent *);

private:
    class Private;
    Private * const p;
};


#endif // Slider2D_HPP