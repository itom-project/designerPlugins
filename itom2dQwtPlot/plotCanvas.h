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

#include "../../common/sharedStructures.h"

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

struct Itom2dQwtPlotActions
{
public:
	QAction *m_actSave;
	QAction *m_actHome;
	QAction *m_actPan;
	QAction *m_actZoom;
	QAction *m_actTracker;
	QAction *m_actScaleSettings;
	QAction *m_actColorPalette;
	QAction *m_actToggleColorBar;
	QAction *m_actLineCut;
	QAction *m_actStackCut;
	QAction *m_actPlaneSelector;
	QSpinBox *m_planeSelector;
};

class PlotCanvas : public QwtPlot
{
    Q_OBJECT
    public:
        PlotCanvas(Itom2dQwtPlotActions *actions, QWidget * parent = NULL);
        ~PlotCanvas();

        void refreshPlot(ito::ParamBase *dataObj);

    protected:
        void contextMenuEvent(QContextMenuEvent * event);
        void keyPressEvent ( QKeyEvent * event );
        void keyReleaseEvent ( QKeyEvent * event );
    
	private:
        QwtPlotZoomer *m_pZoomer;
        QwtPlotPanner *m_pPanner;
        QwtPicker *m_pLinePicker;
        QwtPicker *m_pAScanPicker;
        QwtPlotMarker *m_pAScanMarker;

		QSpinBox *m_planeSelector; //borrowed reference, taken by corresponding action in m_actions
		int m_curColorPaletteIndex;
		QRectF m_orgImageSize;
		DataObjItem *m_dObjItem;
        DataObjRasterData *m_data;

		Itom2dQwtPlotActions *m_pActions;

    signals:
        void spawnNewChild(QVector<QPointF>);
        void updateChildren(QVector<QPointF>);

    public slots:
        void trackerAScanMoved(const QPoint &pt);
        void trackerAScanAppended(const QPoint &pt);
        void trackerMoved(const QPoint &pt);
        void trackerAppended(const QPoint &pt);

        void refreshColorMap(QString colormap = QString());

		void mnuSwitchColorPalette();
		void mnuToggleColorBar(bool checked);
};


#endif
