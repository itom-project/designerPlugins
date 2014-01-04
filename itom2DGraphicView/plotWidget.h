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

#ifndef PlotWidget_H
#define PlotWidget_H

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

#include "dObjToQImage.h"

#include "common/sharedStructures.h"
#include "common/sharedStructuresGraphics.h"
#include "common/apiFunctionsGraphInc.h"
//#include "valuepicker2d.h"

class QGraphicsViewValuePicker : public QGraphicsItem
{
    public:
        QGraphicsViewValuePicker(const QString text, QGraphicsScene* scene, QGraphicsItem* parent = NULL)
            :QGraphicsItem(NULL, scene)
        {
            m_pointTracker = new QGraphicsTextItem(text, NULL, scene);
            m_pointTracker->setPos(0.0, 0.0);
            m_pointTracker->setVisible(false);

            m_pointMarker  = new QGraphicsEllipseItem(-1.0, -1.0, 2.0, 2.0, NULL, scene);
            m_pointMarker->setVisible(false);
        }
        ~QGraphicsViewValuePicker()
        {
            /*
            if(m_pointTracker)
            {
                delete m_pointTracker;
                m_pointTracker = NULL;
            }
            if(m_pointMarker)
            {
                delete m_pointMarker;
                m_pointMarker = NULL;
            }
            */
        }

         QRectF boundingRect() const
         {
             return QRectF(0.0, 0.0, 1.0, 1.0);
         }

         void paint(QPainter* /*painter*/, const QStyleOptionGraphicsItem* /*option*/, QWidget* /*widget*/)
         {
         }

        void setVisible(const bool visibility)
        {
            m_pointTracker->setVisible(visibility);
            m_pointMarker->setVisible(visibility);
        }

        void setText(const char *txt)
        {
            m_pointTracker->setPlainText(txt);
        }

        bool isVisible() const {return m_pointMarker->isVisible() || m_pointTracker->isVisible();}

        void setColor(const QColor newColor)
        {
            m_pointMarker->setPen(QPen(newColor));
            m_pointTracker->setDefaultTextColor(newColor);        
        }

        /* Set center of the marker Ellipse */
        void setPos(const QPointF pos)
        {
            m_outDated = false;
            m_pointMarker->setPos(pos - QPointF(1.0, 1.0));
            m_pointTracker->setPos(pos);
        }

        /* Center of the marker Ellipse */
        QPointF pos() const {return m_pointMarker->pos() + QPointF(1.0, 1.0);}
        double x() const {return m_pointMarker->x() + 1.0;}
        double y() const {return m_pointMarker->y() + 1.0;}

    private:
        bool m_outDated;
        QGraphicsTextItem *m_pointTracker;
        QGraphicsEllipseItem *m_pointMarker;
        
};

class GraphicViewPlot;
struct InternalData;

class PlotWidget :  public QGraphicsView
{
    Q_OBJECT
    public:
        PlotWidget(InternalData* pData, QMenu *contextMenu, QWidget * parent = 0);
        ~PlotWidget();

        bool m_showContextMenu;
        void refreshPlot(ito::ParamBase *dataObj);

        friend class GraphicViewPlot;

        ito::RetVal setCanvasZoom(const int zoolLevel);

        void enableLinePointer(const bool enabled);
        void enableMarker(const bool enabled);

        QPixmap m_pixMap;

        enum tZoomRatio{
            RatioOff    = 0x00,
            Ratio1_1    = 0x01,
            Ratio4_1    = 0x02,
            Ratio2_1    = 0x04,
            Ratio1_2    = 0x08,
            Ratio1_4    = 0x10,
        };

        enum tState 
        { 
            tIdle = 0, 
            tZoom = 1, 
            tValuePicker = 2,
            tPan = 3,
            tLineCut = 4, 
            tStackCut = 5, 
        };

        enum ComplexType 
        { 
            tAbsolute = 0, 
            tImag = 1, 
            tReal = 2, 
            tPhase = 3 
        }; //definition like in dataObject: 0:abs-Value, 1:imaginary-Value, 2:real-Value, 3: argument-Value

        ito::RetVal init();

    protected:
        void resizeEvent (QResizeEvent * event);
        void keyPressEvent ( QKeyEvent * event );
        void keyReleaseEvent ( QKeyEvent * event );
        void mouseMoveEvent ( QMouseEvent * event );
        void mousePressEvent ( QMouseEvent * event );
        void mouseReleaseEvent ( QMouseEvent * event );
        void contextMenuEvent(QContextMenuEvent * event);

        bool setColorMap(QString colormap = "__next__");
        void refreshStyles();
        void updateLabels();

        void enableAxis(const int axis, const bool value);
        QPointF calcInterval(const int axis) const;
        void setState( tState state);

    private:
        

        InternalData* m_pData;

        void trackerMoved(const QPointF &pt);
        void trackerAppended(const QPointF &pt);

        void handleMouseEvent( int type, QMouseEvent *event);

        ito::uint32 m_lineplotUID;
        QMenu *m_contextMenu;

        RasterToQImageObj *m_ObjectContainer;

        QGraphicsScene *m_pContent;
        QGraphicsPixmapItem *m_pItem;

        QGraphicsLineItem *m_pLineCut;
        QGraphicsViewValuePicker *m_pValuePicker;

//        ItoPlotSpectrogram *m_pContent; //content-element, added to canvas when first valid data object becomes available

        bool m_lineIsSampling;
        bool m_trackerIsSampling;

        bool m_stateMoveAligned;

        QWidget *m_pParent;
        QMenu *m_pCmplxMenu;


    signals:
        void spawnNewChild(QVector<QPointF>);
        void updateChildren(QVector<QPointF>);

        void statusBarClear();
        void statusBarMessage(const QString &message, int timeout = 0);

    public slots:

        void updatePointTracker();

        void trackerAScanMoved(const QPoint &pt);
        void trackerAScanAppended(const QPoint &pt);

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

struct InternalData
{
    InternalData()
    {
        m_title = "";
        m_yaxisLabel = "";
        m_xaxisLabel = "";
        m_valueLabel = "";
        m_titleDObj = "";
        m_xaxisLabelDObj = "";
        m_yaxisLabelDObj = "";
        m_valueLabelDObj = "";
        m_autoTitle = 1;
        m_autoxAxisLabel = 1;
        m_autoyAxisLabel = 1;
        m_autoValueLabel = 1;

        m_valueScaleAuto = 1;
        m_valueMin = 0;
        m_valueMax = 0;

        m_xaxisScaleAuto = 1;
        m_xaxisMin = 0;
        m_xaxisMax = 0;
        m_xaxisVisible = 1;

        m_yaxisScaleAuto = 1;
        m_yaxisMin = 0;
        m_yaxisMax = 0;
        m_yaxisFlipped = 0;
        m_yaxisVisible = 1;

        m_colorBarVisible = 0;
        m_paletteNum = 0;
        m_numBits = 8;

        m_cmplxType = PlotWidget::tAbsolute;
        m_state = PlotWidget::tIdle;
        m_zoomLevel = PlotWidget::RatioOff;

        m_colorMode = RasterToQImageObj::ColorIndex8Scaled;

        m_colorTable.clear();
        m_colorTable.resize(256);
        for(unsigned int i = 0; i < 256; i++)
        {
            m_colorTable[i] = qRgb(i, i, i);
        }   
        m_inverseColor0 = Qt::red;
        m_inverseColor1 = Qt::green;


    }
    ~InternalData()
    {


    }
    ito::tDataType m_dataType;

    QString m_title;
    QString m_yaxisLabel;
    QString m_xaxisLabel;
    QString m_valueLabel;

    QString m_titleDObj;
    QString m_xaxisLabelDObj;
    QString m_yaxisLabelDObj;
    QString m_valueLabelDObj;

    bool m_autoTitle;
    bool m_autoxAxisLabel;
    bool m_autoyAxisLabel;
    bool m_autoValueLabel;

    bool m_valueScaleAuto;
    double m_valueMin;
    double m_valueMax;

    bool m_xaxisScaleAuto;
    double m_xaxisMin;
    double m_xaxisMax;
    bool m_xaxisVisible;

    bool m_yaxisScaleAuto;
    double m_yaxisMin;
    double m_yaxisMax;
    bool m_yaxisFlipped;
    bool m_yaxisVisible;

    bool m_colorBarVisible;

    QColor m_inverseColor0;
    QColor m_inverseColor1;

    QVector<ito::uint32> m_colorTable;
    int m_paletteNum;
    unsigned char m_numBits;

    PlotWidget::ComplexType m_cmplxType;
    PlotWidget::tState m_state;
    PlotWidget::tZoomRatio m_zoomLevel;
    RasterToQImageObj::tValueType m_colorMode;
};

#endif
