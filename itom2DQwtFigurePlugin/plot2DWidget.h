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

#ifndef PLOT2DWIDGET_H
#define PLOT2DWIDGET_H

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

#include "itoPlotSpectrogram.h"
#include "valuepicker2d.h"


class itom2DQwtFigure;

class Plot2DWidget : public QwtPlot
{
    Q_OBJECT
    public:
        Plot2DWidget(QMenu *contextMenu, QWidget * parent = 0);
        ~Plot2DWidget();

        bool m_showContextMenu;
        void refreshPlot(ito::ParamBase *dataObj);

        friend class itom2DQwtFigure;
        ito::RetVal setInterval(const Qt::Axis axis, const bool autoCalcLimits, const double minValue, const double maxValue);

        inline bool getStackStatus(){return m_stackState;};
        inline bool getCmplxStatus(){return m_cmplxState;};

    protected:
        void contextMenuEvent(QContextMenuEvent * event);
        void keyPressEvent ( QKeyEvent * event );
        void keyReleaseEvent ( QKeyEvent * event );
    private:

        ito::uint32 m_lineplotUID;
        QMenu *m_contextMenu;

        ItoPlotSpectrogram *m_pContent; //content-element, added to canvas when first valid data object becomes available

        bool m_startScaledZ;
        bool m_startScaledY;
        bool m_startScaledX;

        QPointF m_startRangeZ;
        QPointF m_startRangeY;
        QPointF m_startRangeX;

		QRectF m_orgImageSize;

        int m_paletteNum;
        //int m_linePlotID;

        QWidget *m_pParent;
        QwtPlotZoomer *m_pZoomer;
        QwtPlotPanner *m_pPanner;
        QwtPicker *m_pLinePicker;
        QwtPicker *m_pAScanPicker;
        QwtPlotMarker *m_pAScanMarker;
        ValuePicker2D *m_pValuePicker;
		QMenu *m_pCmplxMenu;
        QwtPlotCurve m_lineCut;

        bool m_cmplxState;
        bool m_stackState;
        bool m_stateMoveAligned;

    signals:

        void spawnNewChild(QVector<QPointF>);
        void updateChildren(QVector<QPointF>);

    public slots:

        void trackerAScanMoved(const QPoint &pt);
        void trackerAScanAppended(const QPoint &pt);
        void trackerMoved(const QPoint &pt);
        void trackerAppended(const QPoint &pt);

        void refreshColorMap(QString colormap = QString());

        void setCursors(int cursorId)
        {
            QApplication::restoreOverrideCursor();
            switch(cursorId)
            {
            case Qt::ArrowCursor:
                QApplication::setOverrideCursor( QCursor(Qt::ArrowCursor) );
                break;
            case Qt::CrossCursor:
                QApplication::setOverrideCursor( QCursor(Qt::CrossCursor) );
                break;
            case Qt::SizeAllCursor:
                QApplication::setOverrideCursor( QCursor(Qt::SizeAllCursor) );
                break;
            case Qt::PointingHandCursor:
                QApplication::setOverrideCursor( QCursor(Qt::PointingHandCursor) );
                break;
            }
        };
};


#endif
