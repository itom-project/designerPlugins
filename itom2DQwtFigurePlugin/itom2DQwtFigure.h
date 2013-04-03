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

#ifndef ITOM2D_QWT_FIGURE_H
#define ITOM2D_QWT_FIGURE_H

#include "plot/AbstractDObjFigure.h"

#include "plot2DWidget.h"

#include "valuepicker2d.h"

#include <qwt_plot.h>
#include <qgridlayout.h>
#include <qstyle.h>
#include <qstyleoption.h>
#include <qpainter.h>
#include <qwt_plot_zoomer.h>
#include <qwt_plot_panner.h>
#include <qwt_plot_curve.h>
#include <qwt_plot_marker.h>

#include <qaction.h>

#include <qsharedpointer.h>
#include <qwidget.h>

Q_DECLARE_METATYPE(QSharedPointer<ito::DataObject>)

class itom2DQwtFigure : public ito::AbstractDObjFigure
{
    Q_OBJECT

    public:
        itom2DQwtFigure(const QString &itomSettingsFile, AbstractFigure::WindowMode windowMode, QWidget *parent = 0);
        ~itom2DQwtFigure();

        //properties
        ito::RetVal displayLineCut(QVector<QPointF> bounds, ito::uint32 &uniqueID);
        void setShowContextMenu(bool show); 
        bool showContextMenu() const;
        ito::RetVal applyUpdate();  //!> does the real update work
        QSharedPointer<ito::DataObject> getSource(void);
        QSharedPointer<ito::DataObject> getDisplayed(void);

        virtual inline void setOutpBounds(QVector<QPointF> bounds) 
        { 
            double *pointArr = new double[2 * bounds.size()];
            for (int np = 0; np < bounds.size(); np++)
            {
                pointArr[np * 2] = bounds[np].x();
                pointArr[np * 2 + 1] = bounds[np].y();
            }
            m_pOutput["bounds"]->setVal(pointArr, 2 * bounds.size());
            delete pointArr;
        }

        void enableComplexGUI(const bool checked);
        void enableZStackGUI(const bool checked);
        void setLinePlotCoordinates(const QVector<QPointF> pts);

        QPointF getZAxisInterval(void);
        void setZAxisInterval(QPointF);

    protected:
        Plot2DWidget *m_pContent;

    private:
        QAction* m_actScaleSetting;
       
        QAction  *m_actPan;
        QAction  *m_actZoomToRect;
        QAction  *m_actMarker;
        QAction  *m_actLineCut;

		QAction  *m_actHome;
		QAction  *m_actSave;

        QAction  *m_actPalette;
        QAction  *m_actToggleColorBar;

        QAction  *m_actAScan;
        QAction  *m_actForward;
        QAction  *m_actBack;
	    QAction  *m_actCmplxSwitch;
	    QMenu    *m_mnuCmplxSwitch;

		QLabel *m_lblCoordinates;

    signals:
    
    private slots:

    public slots:
        void mnuPanner(bool checked);
        void mnuPalette();
        void mnuValuePicker(bool checked);
        void mnuAScanPicker(bool checked);
        void mnuLinePicker(bool checked);
        void mnuScaleSetting();
        void mnuColorBar(bool checked);
		void mnuCmplxSwitch(QAction *action);
        void mnuZoomer(bool checked);
        void mnuExport();

	private slots:
		void mnuHome();
        

        

        QString getColorPalette(void);
        void setColorPalette(QString);
};

//----------------------------------------------------------------------------------------------------------------------------------
/*
class Plot2DEFilter : public QObject
{
    Q_OBJECT

    public:
        Plot2DEFilter(Plot2DImage *plotObj, AbstractNode *plotNode, ItoPlotSpectrogram *plot2D)
            : m_plotObj(plotObj), m_plotNode(plotNode), m_plot2D(plot2D) { }
        ~Plot2DEFilter() {}
        virtual bool eventFilter(QObject *, QEvent *);
        virtual bool event(QEvent *);
        Plot2DImage *m_plotObj;
        AbstractNode *m_plotNode;
        ItoPlotSpectrogram *m_plot2D;

    private:

    signals:

    public slots:

    private slots:

};
*/
//----------------------------------------------------------------------------------------------------------------------------------

#endif // ITOM2D_QWT_FIGURE_H
