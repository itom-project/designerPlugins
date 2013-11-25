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

#ifndef PLOTCANVAS_H
#define PLOTCANVAS_H

#include "common/sharedStructures.h"
#include "DataObject/dataobj.h"

#include "dataObjItem.h"
#include "userInteractionPlotPicker.h"
#include "drawItem.h"


#include <qwidget.h>
#include <qstring.h>
#include <qevent.h>
#include <qpainter.h>
#include <qpoint.h>
#include <qtimer.h>
#include <qcoreapplication.h>
#include <qapplication.h>
#include <qspinbox.h>
#include <qhash.h>

#include <qwt_plot.h>
#include <qwt_plot_zoomer.h>
#include <qwt_plot_panner.h>
#include <qwt_plot_curve.h>
#include <qwt_plot_marker.h>
#include <qwt_plot_magnifier.h>
#include <qwt_plot_shapeitem.h>
#include <qcolor.h>

#include <qwt_plot_rescaler.h>

class Itom2dQwtPlot; //forward declaration
class ValuePicker2D;
struct InternalData;
class DataObjRasterData;
//class UserInteractionPlotPicker;


class PlotCanvas : public QwtPlot
{
    Q_OBJECT
    public:
        enum tState { tIdle, tZoom, tValuePicker, tPan, tLineCut, tStackCut, tMultiPointPick, tPoint, tLine, tRect, tEllipse, tCircle, tSquare };
        enum ComplexType { Real = 2, Imag = 1, Abs = 0, Phase = 3 }; //definition like in dataObject: 0:abs-Value, 1:imaginary-Value, 2:real-Value, 3: argument-Value

        PlotCanvas(InternalData *m_pData, QWidget * parent = NULL);
        ~PlotCanvas();

        ito::RetVal init();
        void refreshPlot(const ito::DataObject *dObj, int plane = -1);

        void changePlane(int plane);
        void internalDataUpdated();

        void setState( tState state);
        void childFigureDestroyed(QObject* obj, ito::uint32 UID);

        QPointF getInterval(Qt::Axis axis) const;
        void setInterval(Qt::Axis axis, const QPointF &interval);

        ito::RetVal plotMarkers(const ito::DataObject *coords, QString style, QString id, int plane);
        ito::RetVal deleteMarkers(const QString &id);
        ito::RetVal deleteMarkers(const int id);

/*        
        ito::RetVal addDrawItems(const ito::DataObject *items, QString style);
        ito::RetVal delDrawItems(const ito::DataObject *items, QString style);
*/        
        friend class Itom2dQwtPlot;
        friend class DrawItem;

    protected:
        void contextMenuEvent(QContextMenuEvent * event);
        void keyPressEvent ( QKeyEvent * event );
        void keyReleaseEvent ( QKeyEvent * event );

        void setLabels(const QString &title, const QString &valueLabel, const QString &xAxisLabel, const QString &yAxisLabel);
        void updateLabels();
        void updateScaleValues();
        void setColorBarVisible(bool visible);
        bool setColorMap(QString colormap = "__next__");
        inline QString colorMapName() const { return m_colorMapName; }

        void refreshStyles();

        ito::RetVal userInteractionStart(int type, bool start, int maxNrOfPoints);

        void mousePressEvent ( QMouseEvent * event );
        void mouseMoveEvent ( QMouseEvent * event );
        void mouseReleaseEvent ( QMouseEvent * event );

        void configRescaler(void);

    private:
        QwtPlotRescaler* m_pRescaler;

        QwtPlotZoomer *m_pZoomer;
        QwtPlotPanner *m_pPanner;
        QwtPlotMagnifier *m_pMagnifier;

        QwtPlotPicker *m_pLineCutPicker;
        QwtPlotCurve *m_pLineCutLine;
        ValuePicker2D *m_pValuePicker;
        QwtPlotPicker *m_pStackPicker;
        QwtPlotMarker *m_pStackCutMarker;
        UserInteractionPlotPicker *m_pMultiPointPicker;

        QString m_colorMapName;
        QMultiHash<QString, QPair<int, QwtPlotMarker*> > m_plotMarkers;

        int m_curColorMapIndex;
        DataObjItem *m_dObjItem;
        DataObjRasterData *m_rasterData;

        ito::uint32 m_zstackCutUID;
        ito::uint32 m_lineCutUID;

        InternalData *m_pData;
        const ito::DataObject *m_dObjPtr; //pointer to the current source (original) data object

        Qt::KeyboardModifiers m_activeModifiers;

        QColor m_inverseColor0, m_inverseColor1;
        int m_activeDrawItem;


    signals:
        void spawnNewChild(QVector<QPointF>);
        void updateChildren(QVector<QPointF>);

        void statusBarClear();
        void statusBarMessage(const QString &message, int timeout = 0);

    private slots:
        void zStackCutTrackerMoved(const QPoint &pt);
        void zStackCutTrackerAppended(const QPoint &pt);
        void lineCutMoved(const QPoint &pt);
        void lineCutAppended(const QPoint &pt);

        void multiPointActivated (bool on);
        //void multiPointSelected (const QPolygon &polygon);
        //void multiPointAppended (const QPoint &pos);
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
        m_cmplxType = PlotCanvas::Real;
        m_state = PlotCanvas::tIdle;
        m_pConstOutput = NULL;

        m_elementsToPick = 0;

        m_pDrawItems.clear();
        m_keepAspect = false; 
    }
    ~InternalData()
    {
        QList<int> keys = m_pDrawItems.keys();
        for (int i = 0; i < keys.size(); i++)
        {
            if(m_pDrawItems[keys[i]] != NULL)
            {
                DrawItem *delItem = m_pDrawItems[keys[i]];
                delItem->detach();
                //delete delItem;
                //m_pDrawItems[keys[i]] = NULL;
                m_pDrawItems.remove(keys[i]);
                // ToDo: Check for memoryleak!!!
                
            }
            //it.value()->detach();
            //delete it.value();
        }
/*        
        for (int n = 0; n < m_pDrawItems.size(); n++)
        {
            m_pDrawItems[n]->detach();
            delete m_pDrawItems[n];
        }
*/        
        m_pDrawItems.clear();
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

    bool m_keepAspect;
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

    int m_elementsToPick;

    PlotCanvas::ComplexType m_cmplxType;

    PlotCanvas::tState m_state;

    const QHash<QString, ito::Param*> *m_pConstOutput;
//    QVector<DrawItem *> m_pDrawItems;
    QHash<int, DrawItem *> m_pDrawItems;
};


#endif
