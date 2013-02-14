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

#include <QtGui/QGraphicsView>

#include <qgraphicsitem.h>
#include <QGraphicsLineItem>

#include <qwidget.h>
#include <qstring.h>
#include <qevent.h>
#include <qpainter.h>
#include <qpoint.h>
#include <qtimer.h>
#include <qcoreapplication.h>
#include <qapplication.h>
#include <qqueue.h>
#include <qmenu.h>

#include "rasterToQImage.h"

#include "common/sharedStructures.h"

//#include "valuepicker2d.h"

class itom2DGVFigure;

class plot2DWidget :  public QGraphicsView
{
    Q_OBJECT
    public:
        plot2DWidget(QMenu *contextMenu, QWidget * parent = 0);
        ~plot2DWidget();

        bool m_showContextMenu;
        void refreshPlot(ito::ParamBase *dataObj);

        friend class itom2DGVFigure;
        ito::RetVal setInterval(const Qt::Axis axis, const bool autoCalcLimits, const double minValue, const double maxValue);
        ito::RetVal setCanvasZoom(const int zoolLevel);

        inline bool getStackStatus(){return m_stackState;}
        inline bool getCmplxStatus(){return m_cmplxState;}
        void enableLinePointer(const bool enabled);
        void enableMarker(const bool enabled);

        QPixmap m_pixMap;

    protected:
        void resizeEvent (QResizeEvent * event);
        void keyPressEvent ( QKeyEvent * event );
        void keyReleaseEvent ( QKeyEvent * event );
        void leaveEvent ( QEvent * event );
        void enterEvent ( QEvent * event );
        void wheelEvent( QWheelEvent * event );
        void mouseDoubleClickEvent ( QMouseEvent * event );
        void mouseMoveEvent ( QMouseEvent * event );
        void mousePressEvent ( QMouseEvent * event );
        void mouseReleaseEvent ( QMouseEvent * event );
        void contextMenuEvent(QContextMenuEvent * event);

    private:

        void trackerMoved(const QPointF &pt);
        void trackerAppended(const QPointF &pt);

        struct PendingEvent
        {
            public:
                PendingEvent() : m_valid(false) {}
                PendingEvent(int x, int y, int button) : m_type(typeMouseMove), m_x(x), m_y(y), m_button(button), m_valid(true) {}
                PendingEvent(const PendingEvent &cpy) : m_type(cpy.m_type), m_x(cpy.m_x), m_y(cpy.m_y), m_button(cpy.m_button), m_h(cpy.m_h), m_w(cpy.m_w), m_valid(cpy.m_valid) {}
                bool isValid() { return m_valid; }
                void clear() { m_valid = false; }
                enum tPendingEventType { typeResize, typeMouseMove };

                tPendingEventType m_type;
                int m_x;
                int m_y;
                int m_button;
                int m_h;
                int m_w;
                bool m_valid;
        };
        void handleMouseEvent( int type, QMouseEvent *event);

        QTimer m_timer;
        ito::uint32 m_lineplotUID;
        PendingEvent m_pendingEvent;
        QMenu *m_contextMenu;

        RasterToQImageObj *m_ObjectContainer;

        QGraphicsScene *m_pContent;
        QGraphicsPixmapItem *m_pItem;

        QGraphicsLineItem *m_pLineCut;
        QGraphicsTextItem *m_pointTracker;
        QGraphicsEllipseItem *m_pointMarker;

//        ItoPlotSpectrogram *m_pContent; //content-element, added to canvas when first valid data object becomes available

        enum tZoomRatio{
            RatioOff    = 0x00,
            Ratio1_1    = 0x01,
            Ratio4_1    = 0x02,
            Ratio2_1    = 0x04,
            Ratio1_2    = 0x08,
            Ratio1_4    = 0x10,
        };

        bool m_startScaledZ;
        bool m_startScaledY;
        bool m_startScaledX;
        bool m_fixedZoom;
        bool m_lineIsSampling;
        bool m_trackerIsSampling;

        bool m_stateMoveAligned;

        unsigned char m_showColored;

        QPointF m_startRangeZ;
        QPointF m_startRangeY;
        QPointF m_startRangeX;

        int m_paletteNum;
        //int m_linePlotID;

        QWidget *m_pParent;
//        ValuePicker2D *m_pValuePicker;
        QMenu *m_pCmplxMenu;
/*
        GVPlotZoomer *m_pZoomer;
        GVPlotPanner *m_pPanner;
        GVPicker *m_pLinePicker;
        GVPicker *m_pAScanPicker;
        GVPlotMarker *m_pAScanMarker;
        GVPlotCurve m_lineCut;
*/

        bool m_cmplxState;
        bool m_stackState;

    signals:
        void eventLeaveEnter(bool enter);
        void eventMouse(int type, int x, int y, int button);
        void eventWheel(int x, int y, int delta, int orientation);
        void eventKey(int type, int keyId, QString keyString, bool autoRepeat);
        void eventPaintRequest();
        void eventIdle();

        void spawnNewChild(QVector<QPointF>);
        void updateChildren(QVector<QPointF>);

    public slots:
        void paintTimeout();
        void stopTimer()
        {
            m_timer.stop();
        }

        void updatePointTracker();

        void trackerAScanMoved(const QPoint &pt);
        void trackerAScanAppended(const QPoint &pt);
#if linux
        void refreshColorMap(QString colormap = QString());
#else
        void refreshColorMap(QString colormap = QString::QString());
#endif
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
        }
};


#endif
