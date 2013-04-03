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

#ifndef ITOMFIGURE_H
#define ITOMFIGURE_H

#include "plot/AbstractDObjFigure.h"

#include "plot1DWidget.h"

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


class itom1DQwtFigure : public ito::AbstractDObjFigure
{
    Q_OBJECT
    Q_PROPERTY(QVector<QPointF> bounds READ getBounds WRITE setBounds DESIGNABLE false)

    public:
        itom1DQwtFigure(const QString &itomSettingsFile, AbstractFigure::WindowMode windowMode, QWidget *parent = 0);
        ~itom1DQwtFigure();

        //properties
        void setShowContextMenu(bool show); 
        bool showContextMenu() const;
        ito::RetVal applyUpdate();                              //!< propagates updated data through the subtree
        void setSource(QSharedPointer<ito::DataObject> source);
        QSharedPointer<ito::DataObject> getSource(void);
        QSharedPointer<ito::DataObject> getDisplayed(void);

        virtual inline void setBounds(QVector<QPointF> bounds) 
        { 
            double *pointArr = new double[2 * bounds.size()];
            for (int np = 0; np < bounds.size(); np++)
            {
                pointArr[np * 2] = bounds[np].x();
                pointArr[np * 2 + 1] = bounds[np].y();
            }
            m_pInput["bounds"]->setVal(pointArr, 2 * bounds.size());
            delete pointArr;
        }

        virtual inline QVector<QPointF> getBounds(void) 
        { 
            QVector<QPointF> boundsVec;
            double *ptsDblVec = m_pInput["bounds"]->getVal<double*>();
            int numPts = m_pInput["bounds"]->getLen();

            boundsVec.reserve(numPts / 2);
            for (int n = 0; n < numPts / 2; n++)
            {
                boundsVec.append(QPointF(ptsDblVec[n * 2], ptsDblVec[n * 2 + 1]));
            }
            return boundsVec;
        }

        void setMarkerCoordinates(const QVector<QPointF> pts);
        void enableComplexGUI(const bool checked);
    
    protected:
        Plot1DWidget *m_pContent;

    private:

        QAction* m_actScaleSetting;
        QAction* m_rescaleParent;

        QAction  *m_actPan;
        QAction  *m_actZoomToRect;
        QAction  *m_actMarker;

		QAction *m_actSave;
		QAction *m_actHome;

        QMenu    *m_mnuSetMarker;
        QAction  *m_actSetMarker;

        QAction *m_actForward;
        QAction *m_actBack;
        
	    QAction* m_actCmplxSwitch;
	    QMenu *m_mnuCmplxSwitch;

        QLabel *m_CurCoordDelta;
		QLabel *m_lblCoordinates;

    signals:
    
    private slots:

    public slots:
        void mnuMarkerClick(bool checked);
        void mnuPanner(bool checked);
        void mnuScaleSetting();
        void mnuParentScaleSetting();
		void mnuCmplxSwitch(QAction *action);
        void mnuSetMarker(QAction *action);
        void mnuZoomer(bool checked);
        void mnuExport();

        QPointF getYAxisInterval(void);
        void setYAxisInterval(QPointF);
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

#endif // ITOMFIGURE_H
