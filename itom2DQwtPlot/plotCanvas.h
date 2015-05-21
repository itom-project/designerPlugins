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
#include "common/sharedStructuresPrimitives.h"
#include "common/interval.h"

#include "DataObject/dataobj.h"

#include "dataObjItem.h"
#include "../sharedFiles/userInteractionPlotPicker.h"
#include "../sharedFiles/drawItem.h"
#include "../sharedFiles/itomPlotZoomer.h"
#include "../sharedFiles/itomPlotMagnifier.h"
#include "../sharedFiles/itomPlotPicker.h"


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
#include <qwt_plot_panner.h>
#include <qwt_plot_curve.h>
#include <qwt_plot_marker.h>
#include <qwt_plot_shapeitem.h>
#include <qcolor.h>
#include <qmenu.h>

#include "itom2dqwtplotenums.h"

class Itom2dQwtPlot; //forward declaration
class ValuePicker2D;
struct InternalData;
class DataObjRasterData;
//class UserInteractionPlotPicker;


class PlotCanvas : public QwtPlot
{
    Q_OBJECT
    public:
        enum tState 
        { 
            tIdle = 0, 
            tZoom = 1, 
            tValuePicker = 2, 
            tPan = 3, 
            tLineCut = 4, 
            tStackCut = 5, 
            tMultiPointPick = ito::PrimitiveContainer::tMultiPointPick, 
            tPoint = ito::PrimitiveContainer::tPoint, 
            tLine = ito::PrimitiveContainer::tLine, 
            tRect = ito::PrimitiveContainer::tRectangle, 
            tSquare = ito::PrimitiveContainer::tSquare,
            tEllipse = ito::PrimitiveContainer::tEllipse, 
            tCircle = ito::PrimitiveContainer::tCircle, 
            tPolygon = ito::PrimitiveContainer::tPolygon
        };

        enum changeFlag {
            changeNo = 0,
            changeAppearance = 1,
            changeData = 2
        };

        PlotCanvas(QMenu *contextMenu, InternalData *m_pData, QWidget * parent = NULL);
        ~PlotCanvas();

        bool m_showContextMenu;

        ito::RetVal init();
        void refreshPlot(const ito::DataObject *dObj, int plane = -1);

        void changePlane(int plane);
        int getCurrentPlane();

        void internalDataUpdated();

        void setState( tState state);
        void childFigureDestroyed(QObject* obj, ito::uint32 UID);

        ito::AutoInterval getInterval(Qt::Axis axis) const;
        void setInterval(Qt::Axis axis, const ito::AutoInterval &interval);

        ito::AutoInterval getOverlayInterval(Qt::Axis axis) const;
        void setOverlayInterval(Qt::Axis axis, const ito::AutoInterval &interval);

        ito::RetVal plotMarkers(const ito::DataObject *coords, QString style, QString id, int plane);
        ito::RetVal deleteMarkers(const QString &id);
        ito::RetVal deleteMarkers(const int id);

        ito::int32 getCurrentPlane() const;
        QSharedPointer<ito::DataObject> getDisplayed(void);
        QSharedPointer<ito::DataObject> getOverlayObject(void);
        QSharedPointer<ito::DataObject> getDisplayedOverlayObject(void);

        void setVisible(bool visible);
/*        
        ito::RetVal addDrawItems(const ito::DataObject *items, QString style);
        ito::RetVal delDrawItems(const ito::DataObject *items, QString style);
*/        
        friend class Itom2dQwtPlot;
        friend class DrawItem;

    protected:
        void getMinMaxLoc(double &min, ito::uint32 *minLoc, double &max, ito::uint32 *maxLoc);
        void getMinMaxPhysLoc(double &min, double *minPhysLoc, double &max, double *maxPhysLoc);
        void contextMenuEvent(QContextMenuEvent * event);
        void keyPressEvent ( QKeyEvent * event );
        void keyReleaseEvent ( QKeyEvent * event );

        void setLabels(const QString &title, const QString &valueLabel, const QString &xAxisLabel, const QString &yAxisLabel);
        void updateLabels();
        void synchronizeScaleValues();
        void updateScaleValues(bool doReplot = true, bool doZoomBase = true);
        void setColorBarVisible(bool visible);
        bool setColorMap(QString colormap = "__next__");
        inline QString colorMapName() const { return m_colorMapName; }

        bool setOverlayColorMap(QString colormap = "__next__");
        inline QString colorOverlayMapName() const { return m_colorOverlayMapName; }

        void refreshStyles();

        ito::RetVal userInteractionStart(int type, bool start, int maxNrOfPoints);

        void mousePressEvent ( QMouseEvent * event );
        void mouseMoveEvent ( QMouseEvent * event );
        void mouseReleaseEvent ( QMouseEvent * event );

        void configRescaler(void);
        
        void setOverlayObject(ito::DataObject* newOverlay);
        void alphaChanged();
        void updateColors();
        void updateLabelVisibility();
    private:

        // a1 is line1 start, a2 is line1 end, b1 is line2 start, b2 is line2 end
        bool lineIntersection(const QPointF &a1, const QPointF &a2, const QPointF &b1, const QPointF &b2, QPointF &intersection);

        ito::DataObject randImg;

        ItomPlotZoomer *m_pZoomer;
        QwtPlotPanner *m_pPanner;
        ItomPlotMagnifier *m_pMagnifier;

        QwtPlotPicker *m_pLineCutPicker;
        QwtPlotCurve *m_pLineCutLine;
        ValuePicker2D *m_pValuePicker;
        ItomPlotPicker *m_pStackPicker;
        QwtPlotMarker *m_pStackCutMarker;
        QwtPlotMarker *m_pCenterMarker;

        UserInteractionPlotPicker *m_pMultiPointPicker;

        QString m_colorOverlayMapName;
        QString m_colorMapName;
        QMultiHash<QString, QPair<int, QwtPlotMarker*> > m_plotMarkers;

        int m_curColorMapIndex;
        int m_curOverlayColorMapIndex;

        DataObjItem *m_dObjItem;
        DataObjItem *m_dOverlayItem;

        DataObjRasterData *m_rasterData;
        DataObjRasterData *m_rasterOverlayData;
        
        ito::uint32 m_zstackCutUID;
        ito::uint32 m_lineCutUID;
        bool m_lineCutValidStart; //true if the first point of the line cut is a valid point inside of the data object

        InternalData *m_pData;
        const ito::DataObject *m_dObjPtr; //pointer to the current source (original) data object

        Qt::KeyboardModifiers m_activeModifiers;

        QColor m_inverseColor0, m_inverseColor1;
        int m_activeDrawItem;

        QMenu *m_contextMenu;

        QVector<ito::uint16> m_drawedIemsIndexes;
        bool m_ignoreNextMouseEvent;

        QPoint m_initialMousePosition;
        QPointF m_initialMarkerPosition;

        bool m_isRefreshingPlot; //true if the refreshPlot method is currently executed (in order to avoid interative, stacked calls to refreshPlot)
        bool m_firstTimeVisible; //true if this plot becomes visible for the first time
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
        m_autoTitle = true;
        m_autoxAxisLabel = true;
        m_autoyAxisLabel = true;
        m_autoValueLabel = true;

        m_valueScaleAuto = true;
        m_valueMin = 0;
        m_valueMax = 0;

        m_xaxisScaleAuto = true;
        m_xaxisMin = 0;
        m_xaxisMax = 0;
        m_xaxisVisible = true;

        m_yaxisScaleAuto = true;
        m_yaxisMin = 0;
        m_yaxisMax = 0;
        m_yaxisFlipped = false;
        m_yaxisVisible = true;

        m_colorBarVisible = false;
        m_cmplxType = Itom2DQwt::Real;
        m_state = PlotCanvas::tIdle;
        m_modState = Itom2DQwt::tMoveGeometricElements;
        m_pConstOutput = NULL;

        m_elementsToPick = 0;

        m_pDrawItems.clear();
        m_keepAspect = false;
        m_enablePlotting = true;
        m_showCenterMarker = false;
        m_alpha = 0;

        m_overlayScaleAuto = true;
        m_overlayMin = 0;
        m_overlayMax = 0;

        m_axisColor = Qt::black;
        m_textColor = Qt::black;
        m_backgnd = Qt::white;

        m_markerLabelVisible = false;
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
                m_pDrawItems.remove(keys[i]);
                delete delItem;
                
            }
        }
   
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

    bool m_overlayScaleAuto;
    double m_overlayMin;
    double m_overlayMax;

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
    bool m_markerLabelVisible;
    int m_elementsToPick;
    unsigned char m_alpha;

    QColor m_backgnd;           //!> plot background color
    QColor m_axisColor;         //!> color of axis
    QColor m_textColor;         //!> text color

    bool m_enablePlotting;
    bool m_showCenterMarker;

    Itom2DQwt::tComplexType m_cmplxType;

    PlotCanvas::tState m_state;
    Itom2DQwt::tModificationState m_modState;
    const QHash<QString, ito::Param*> *m_pConstOutput;
//    QVector<DrawItem *> m_pDrawItems;
    QHash<int, DrawItem *> m_pDrawItems;
};


#endif
