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

#ifndef PLOT1DWIDGET_H
#define PLOT1DWIDGET_H

#include "common/sharedStructures.h"
#include "DataObject/dataobj.h"

#include <qwidget.h>
#include <qstring.h>
#include <qevent.h>
#include <qpainter.h>
#include <qpoint.h>
#include <qtimer.h>
#include <qcoreapplication.h>
#include <qapplication.h>
#include <qgraphicsview.h>
#include <qgraphicsscene.h>
#include <qgraphicsitem.h>
#include <qqueue.h>
#include <qmenu.h>

#include <qwt_plot.h>
#include <qwt_plot_zoomer.h>
#include <qwt_plot_panner.h>
#include <qwt_plot_curve.h>
#include <qwt_plot_marker.h>

#include "valuepicker1d.h"

class itom1DQwtFigure;

class Plot1DWidget : public QwtPlot
{
    Q_OBJECT
    public:
        Plot1DWidget(QMenu *contextMenu, QWidget * parent = 0);
        ~Plot1DWidget();

        bool m_showContextMenu;
        void refreshPlot(QSharedPointer<ito::DataObject> dataObj, QVector<QPointF> pts);

        friend class itom1DQwtFigure;
        ito::RetVal setInterval(const Qt::Axis axis, const bool autoCalcLimits, const double minValue, const double maxValue);

        void setZoomerEnable(const bool checked);
        void setPickerEnable(const bool checked);

    protected:
        void keyPressEvent ( QKeyEvent * event );
        void mouseReleaseEvent ( QMouseEvent * event );
        void contextMenuEvent(QContextMenuEvent * event);

    private:

        QMenu *m_contextMenu;
        QwtPlotCurve **m_pContent; //content-element, added to canvas when first valid data object becomes available

        bool m_startScaledY;
        bool m_startScaledX;
        bool m_xDirect;
        bool m_yDirect;
        bool m_pCurserEnable;
        bool m_cmplxState;

        int m_numElements;
        int m_multiLine;
        char m_autoLineColIndex;
        long m_lineCol;
        int m_lineStyle;
        int m_linePlotID;

        int m_Curser[2];
        bool m_curserFirstActive;

        QStringList m_colorList;
        QPointF m_startRangeY;
        QPointF m_startRangeX;

        QWidget *m_pParent;
        QwtPlotZoomer *m_pZoomer;
        QwtPlotPanner *m_pPanner;

        ValuePicker1D *m_pValuePicker;

        QwtPlotMarker * m_pCurser1;
        QwtPlotMarker * m_pCurser2;

		QMenu *m_pCmplxMenu;

    signals:
        void spawnNewChild(QVector<QPointF>);
        void updateChildren(QVector<QPointF>);

    public slots:
        void replot();

};


#endif
