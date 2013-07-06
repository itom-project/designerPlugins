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

#include "dataObjItem.h"
#include "dataObjRasterData.h"

#include <qwidget.h>
#include <qstring.h>
#include <qevent.h>
#include <qpainter.h>
#include <qpoint.h>
#include <qtimer.h>
#include <qcoreapplication.h>
#include <qapplication.h>
#include <qspinbox.h>

#include <qwt_plot.h>
#include <qwt_plot_zoomer.h>
#include <qwt_plot_panner.h>
#include <qwt_plot_curve.h>
#include <qwt_plot_marker.h>

class Itom2dQwtPlot; //forward declaration

struct InternalData
{
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

    bool m_yaxisScaleAuto;
    double m_yaxisMin;
    double m_yaxisMax;

    bool m_colorBarVisible;
};

class PlotCanvas : public QwtPlot
{
    Q_OBJECT
    public:
        enum tState { tIdle, tZoom, tValuePicker, tPan, tLineCut, tStackCut };

        PlotCanvas(InternalData *m_pData, QWidget * parent = NULL);
        ~PlotCanvas();

        void refreshPlot(ito::ParamBase *dataObj);

        void setState( tState state);

        friend class Itom2dQwtPlot;

    protected:
        void contextMenuEvent(QContextMenuEvent * event);
        void keyPressEvent ( QKeyEvent * event );
        void keyReleaseEvent ( QKeyEvent * event );

        void setLabels(const QString &title, const QString &valueLabel, const QString &xAxisLabel, const QString &yAxisLabel);
        void updateLabels();
        void updateScaleValues();
        void setColorBarVisible(bool visible);
        void setColorMap(QString colormap = "__next__");
    
	private:
        QwtPlotZoomer *m_pZoomer;
        QwtPlotPanner *m_pPanner;
        QwtPicker *m_pLineCutPicker;
        QwtPicker *m_pStackCutPicker;
        QwtPlotMarker *m_pStackCutMarker;

		int m_curColorMapIndex;
		QRectF m_orgImageSize;
		DataObjItem *m_dObjItem;
        DataObjRasterData *m_rasterData;

        tState m_state;

		InternalData *m_pData;

    signals:
        void spawnNewChild(QVector<QPointF>);
        void updateChildren(QVector<QPointF>);

    public slots:
        void trackerAScanMoved(const QPoint &pt);
        void trackerAScanAppended(const QPoint &pt);
        void trackerMoved(const QPoint &pt);
        void trackerAppended(const QPoint &pt);
};


#endif
