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

#ifndef ITOMPLOT_H
#define ITOMPLOT_H

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


class Itom1DQwtPlot : public ito::AbstractDObjFigure
{
    Q_OBJECT
    Q_PROPERTY(QVector<QPointF> bounds READ getBounds WRITE setBounds DESIGNABLE false)
    Q_PROPERTY(QString title READ getTitle WRITE setTitle RESET resetTitle)
    Q_PROPERTY(QString axisLabel READ getAxisLabel WRITE setAxisLabel RESET resetAxisLabel)
    Q_PROPERTY(QString valueLabel READ getValueLabel WRITE setValueLabel RESET resetValueLabel)
    Q_PROPERTY(QFont titleFont READ getTitleFont WRITE setTitleFont)
    Q_PROPERTY(QFont labelFont READ getLabelFont WRITE setLabelFont)
    Q_PROPERTY(QFont axisFont READ getAxisFont WRITE setAxisFont)

    Q_CLASSINFO("prop://title", "Title of the plot or '<auto>' if the title of the data object should be used.")
    Q_CLASSINFO("prop://axisLabel", "Label of the direction (x/y) axis or '<auto>' if the descriptions from the data object should be used.")
    Q_CLASSINFO("prop://valueLabel", "Label of the value axis (y-axis) or '<auto>' if the description should be used from data object.")
    Q_CLASSINFO("prop://titleFont", "Font for title.")
    Q_CLASSINFO("prop://labelFont", "Font for axes descriptions.")
    Q_CLASSINFO("prop://axisFont", "Font for axes tick values.")

    public:
        Itom1DQwtPlot(const QString &itomSettingsFile, AbstractFigure::WindowMode windowMode, QWidget *parent = 0);
        virtual ~Itom1DQwtPlot();

        ito::RetVal applyUpdate();                              //!< propagates updated data through the subtree

        //properties
        bool getContextMenuEnabled() const;
        void setContextMenuEnabled(bool show); 

        QVector<QPointF> getBounds(void);
        void setBounds(QVector<QPointF> bounds);

        void enableComplexGUI(const bool checked);

        QString getTitle() const;
        void setTitle(const QString &title);
        void resetTitle();

        QString getAxisLabel() const;
        void setAxisLabel(const QString &label);
        void resetAxisLabel();

        QString getValueLabel() const;
        void setValueLabel(const QString &label);
        void resetValueLabel();

        QPointF getYAxisInterval(void) const;
        void setYAxisInterval(QPointF);

        QFont getTitleFont(void) const;
        void setTitleFont(const QFont &font);

        QFont getLabelFont(void) const;
        void setLabelFont(const QFont &font);

        QFont getAxisFont(void) const;
        void setAxisFont(const QFont &font);

        void setSource(QSharedPointer<ito::DataObject> source);
    
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

        QSharedPointer<ito::DataObject> getDisplayed(void);
        
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

#endif // ITOMPLOT_H
