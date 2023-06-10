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

    //keep the order, the initialization from an ui file is processed in this order
    Q_PROPERTY(QPointF xRange READ xRange WRITE setXRange DESIGNABLE true )
    Q_PROPERTY(QPointF yRange READ yRange WRITE setYRange DESIGNABLE true )
    Q_PROPERTY(qreal xStepSize READ getXStepSize WRITE setXStepSize DESIGNABLE true)
    Q_PROPERTY(qreal yStepSize READ getYStepSize WRITE setYStepSize DESIGNABLE true)
    Q_PROPERTY(qreal xVal READ xVal WRITE setX NOTIFY xValChanged DESIGNABLE true)
    Q_PROPERTY(qreal yVal READ yVal WRITE setY NOTIFY yValChanged DESIGNABLE true)
    Q_PROPERTY(int decimals READ getDecimals WRITE setDecimals DESIGNABLE true)

    Q_CLASSINFO("prop://xRange", "get/set the range of the x-value");
    Q_CLASSINFO("prop://yRange", "get/set the range of the y-value");
    Q_CLASSINFO("prop://xStepSize", "get/set the step size of the x-value (default: 0.0, no step size contraints)");
    Q_CLASSINFO("prop://yStepSize", "get/set the step size of the y-value (default: 0.0, no step size contraints)");
    Q_CLASSINFO("prop://xVal", "get/set current horizontal value (x-value) of the slider");
    Q_CLASSINFO("prop://yVal", "get/set current vertical value (y-value) of the slider");
    Q_CLASSINFO("prop://decimals", "get/set the number of decimals");

    Q_CLASSINFO("signal://xValChanged", "signal is emitted if the x-value of the slider changed.")
    Q_CLASSINFO("signal://yValChanged", "signal is emitted if the y-value of the slider changed.")
    Q_CLASSINFO("signal://valuesChanged", "signal is emitted if the hoirzontal and/or vertical value of the slider changed.")


public:

    explicit Slider2D(QWidget *parent = 0);
    ~Slider2D();


   // QSize sizeHint() const;


    /// Get current xVal in the range [0-1]
    qreal xVal() const;

    /// Get current yVal in the range [0-1]
    qreal yVal() const;

    /// Get the width in pixels of the outer wheel
    QPointF xRange() const;
    QPointF yRange() const;

    /// Set the width in pixels of the outer wheel
    void setXRange(QPointF xRange);
    void setYRange(QPointF yRange);

    qreal getXStepSize() const;
    void setXStepSize(qreal xStepSize);

    qreal getYStepSize() const;
    void setYStepSize(qreal yStepSize);

    int getDecimals() const;
    void setDecimals(int decimals);


public slots:

    void setX(qreal x);
    void setY(qreal y);
    void setValues(qreal x, qreal y);


signals:

    void xValChanged(qreal x);
    void yValChanged(qreal y);
    void valuesChanged(qreal x, qreal y);


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
