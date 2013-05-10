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


class Itom1DQwtFigure : public ito::AbstractDObjFigure
{
    Q_OBJECT
    Q_PROPERTY(QVector<QPointF> bounds READ getBounds WRITE setBounds DESIGNABLE false)
    Q_PROPERTY(QString title READ getTitle WRITE setTitle RESET resetTitle)
    Q_PROPERTY(QString axisLabel READ getAxisLabel WRITE setAxisLabel RESET resetAxisLabel)
    Q_PROPERTY(QString valueLabel READ getValueLabel WRITE setValueLabel RESET resetValueLabel)

    public:
        Itom1DQwtFigure(const QString &itomSettingsFile, AbstractFigure::WindowMode windowMode, QWidget *parent = 0);
        virtual ~Itom1DQwtFigure();

        ito::RetVal applyUpdate();                              //!< propagates updated data through the subtree

        //properties
        bool showContextMenu() const;
        void setShowContextMenu(bool show); 

        QVector<QPointF> getBounds(void);
        void setBounds(QVector<QPointF> bounds);

        
        void enableComplexGUI(const bool checked);

        QString getTitle();
        void setTitle(const QString &title);
        void resetTitle();

        QString getAxisLabel();
        void setAxisLabel(const QString &label);
        void resetAxisLabel();

        QString getValueLabel();
        void setValueLabel(const QString &label);
        void resetValueLabel();
    
    protected:
        ito::RetVal init() { return m_pContent->init(); }; //called when api-pointers are transmitted, directly after construction

        Plot1DWidget *m_pContent;
        InternalData m_data;

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

        QLabel *m_lblMarkerOffsets;
		QLabel *m_lblMarkerCoords;

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

    private slots:
        void mnuHome();
        void setMarkerText(const QString &coords, const QString &offsets);
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
