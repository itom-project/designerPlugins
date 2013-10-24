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
#include <qwt_plot_grid.h>

#include "valuepicker1d.h"

class itom1DQwtFigure;

struct InternalData
{
    InternalData() : m_title(""), m_axisLabel(""), m_valueLabel(""), m_titleDObj(""),
        m_axisLabelDObj(""), m_valueLabelDObj(""), m_autoTitle(1), m_autoAxisLabel(1), m_autoValueLabel(1),
        m_valueScaleAuto(1), m_valueMin(0), m_valueMax(0), m_axisScaleAuto(1), m_axisMin(0), m_axisMax(0), m_forceValueParsing(1) {}

    ito::tDataType m_dataType;

    QString m_title;
    QString m_axisLabel;
    QString m_valueLabel;

    QString m_titleDObj;
    QString m_axisLabelDObj;
    QString m_valueLabelDObj;

    bool m_autoTitle;
    bool m_autoAxisLabel;
    bool m_autoValueLabel;

    bool m_valueScaleAuto;
    double m_valueMin;
    double m_valueMax;

    bool m_axisScaleAuto;
    double m_axisMin;
    double m_axisMax;

    //true for one replot if setSource-Property has been set 
    //(even if the same data object is given one more time, 
    //the hash might be the same, but we want to recalcuate 
    //boundaries if values of dataObject changed.
    bool m_forceValueParsing; 
};

class Plot1DWidget : public QwtPlot
{
    Q_OBJECT
    public:
        enum MultiLineMode { FirstRow, FirstCol, MultiRows, MultiCols };
        enum State { stateIdle, statePanner, stateZoomer, statePicker };

        Plot1DWidget(QMenu *contextMenu, InternalData *data, QWidget * parent = 0);
        ~Plot1DWidget();

        ito::RetVal init();

        bool m_showContextMenu;
        void refreshPlot(const ito::DataObject* dataObj, QVector<QPointF> bounds = QVector<QPointF>() );

        friend class Itom1DQwtPlot;
        ito::RetVal setInterval(const Qt::Axis axis, const bool autoCalcLimits, const double minValue, const double maxValue);

        void setZoomerEnable(const bool checked);
        void setPickerEnable(const bool checked);
        void setPannerEnable(const bool checked);

        void setMainMarkersToIndex(int idx1, int idx2, int curveIdx);

    protected:
        void keyPressEvent ( QKeyEvent * event );
        void mousePressEvent ( QMouseEvent * event );
        void mouseMoveEvent ( QMouseEvent * event );
        void mouseReleaseEvent ( QMouseEvent * event );
        void contextMenuEvent(QContextMenuEvent * event);

        void setLabels(const QString &title, const QString &valueLabel, const QString &axisLabel);
        void updateLabels();
        void updateScaleValues(bool recalculateBoundaries = false);

        

    private:

        struct Marker
        {
            Marker() : item(NULL), active(0), curveIdx(0) {}
            QwtPlotMarker *item;
            bool active;
            int curveIdx;
        };

        void stickMarkerToXPx(Marker *m, double xScaleStart, int dir);
        void stickMarkerToSampleIdx(Marker *m, int idx, int curveIdx, int dir);
        void updateMarkerPosition(bool updatePositions, bool clear = false);


        QMenu *m_contextMenu;

        QList<QwtPlotCurve*> m_plotCurveItems;

        QwtPlotGrid *m_pPlotGrid;

        QByteArray m_hash; //hash of recently loaded dataObject

        //QwtPlotCurve **m_pContent; //content-element, added to canvas when first valid data object becomes available
        InternalData *m_pData;

        //bool m_startScaledY;
        //bool m_startScaledX;
        bool m_xDirect;
        bool m_yDirect;
        bool m_cmplxState;

        MultiLineMode m_multiLine;
        unsigned char m_autoLineColIndex;
        long m_lineCol;
        int m_lineStyle;
        int m_linePlotID;

        int m_Curser[2];
        int m_actPickerIdx;

        QStringList m_colorList;

        QWidget *m_pParent;
        QwtPlotZoomer *m_pZoomer;
        QwtPlotPanner *m_pPanner;

        ValuePicker1D *m_pValuePicker;

        QList<Marker> m_markers;

		QMenu *m_pCmplxMenu;

        State m_state;

    signals:
        void spawnNewChild(QVector<QPointF>);
        void updateChildren(QVector<QPointF>);

        void setMarkerText(const QString &coords, const QString &offsets);

    public slots:
        //void replot();

};


#endif
