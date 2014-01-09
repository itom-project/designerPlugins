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

#ifndef ITOM1DPLOT_H
#define ITOM1DPLOT_H

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
    Q_PROPERTY(QString title READ getTitle WRITE setTitle RESET resetTitle USER true)
    Q_PROPERTY(QString axisLabel READ getAxisLabel WRITE setAxisLabel RESET resetAxisLabel USER true)
    Q_PROPERTY(QString valueLabel READ getValueLabel WRITE setValueLabel RESET resetValueLabel USER true)
    Q_PROPERTY(QFont titleFont READ getTitleFont WRITE setTitleFont USER true)
    Q_PROPERTY(QFont labelFont READ getLabelFont WRITE setLabelFont USER true)
    Q_PROPERTY(QFont axisFont READ getAxisFont WRITE setAxisFont USER true)

    // Properties related with geometric elements
    Q_PROPERTY(QSharedPointer< ito::DataObject > geometricElements READ getGeometricElements WRITE setGeometricElements DESIGNABLE false)
    Q_PROPERTY(int geometricElementsCount READ getGeometricElementsCount DESIGNABLE false)
    Q_PROPERTY(bool keepAspectRatio READ getkeepAspectRatio WRITE setkeepAspectRatio USER true)
    Q_PROPERTY(bool enablePlotting READ getEnabledPlotting WRITE setEnabledPlotting USER true)
    Q_PROPERTY(int selectedGeometry READ getSelectedElement WRITE setSelectedElement DESIGNABLE false)

    Q_CLASSINFO("prop://title", "Title of the plot or '<auto>' if the title of the data object should be used.")
    Q_CLASSINFO("prop://axisLabel", "Label of the direction (x/y) axis or '<auto>' if the descriptions from the data object should be used.")
    Q_CLASSINFO("prop://valueLabel", "Label of the value axis (y-axis) or '<auto>' if the description should be used from data object.")
    Q_CLASSINFO("prop://titleFont", "Font for title.")
    Q_CLASSINFO("prop://labelFont", "Font for axes descriptions.")
    Q_CLASSINFO("prop://axisFont", "Font for axes tick values.")

    Q_CLASSINFO("prop://geometricElements", "Geometric elements defined by a float32[11] array for each element.")
    Q_CLASSINFO("prop://geometricElementsCount", "Number of currently existing geometric elements.")
    Q_CLASSINFO("prop://keepAspectRatio", "Enable and disable a fixed 1:1 aspect ratio between x and y axis.")
    Q_CLASSINFO("prop://enablePlotting", "Enable and disable internal plotting functions and GUI-elements for geometric elements.")


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
    
        int getGeometricElementsCount() const { return m_data.m_pDrawItems.size();}
        void setGeometricElementsCount(const int value){ return;}

        bool getkeepAspectRatio(void) const {return this->m_data.m_keepAspect;}
        void setkeepAspectRatio(const bool &keepAspectEnable);

        QSharedPointer< ito::DataObject > getGeometricElements();
        void setGeometricElements(QSharedPointer< ito::DataObject > geometricElements);

        bool getEnabledPlotting(void) const {return m_data.m_enablePlotting;}
        void setEnabledPlotting(const bool &enabled);

        int getSelectedElement(void) const;
        void setSelectedElement(const int idx);

        friend class Plot1DWidget;

    protected:
        void createActions();
        ito::RetVal init() { return m_pContent->init(); }; //called when api-pointers are transmitted, directly after construction

        Plot1DWidget *m_pContent;
        InternalData m_data;

    private:

        QAction* m_pActScaleSetting;
        QAction* m_pRescaleParent;

        QAction  *m_pActPan;
        QAction  *m_pActZoomToRect;
        QAction  *m_pActMarker;

        QAction *m_pActAspectRatio;

        QAction *m_pActSave;
        QAction *m_pActHome;

        QMenu    *m_pMnuSetMarker;
        QAction  *m_pActSetMarker;

        QAction *m_pActForward;
        QAction *m_pActBack;
        
        QAction* m_pActCmplxSwitch;
        QMenu *m_pMnuCmplxSwitch;

        QLabel *m_pLblMarkerOffsets;
        QLabel *m_pLblMarkerCoords;

        QActionGroup *m_pDrawModeActGroup;
        QAction *m_pActClearDrawings;
        QAction* m_pActDrawMode;
        QMenu *m_pMnuDrawMode;

        QAction *m_pActProperties;

        ito::RetVal qvector2DataObject(const ito::DataObject *dstObject);

    public slots:

        ito::RetVal plotMarkers(const ito::DataObject &coords, QString style, QString id = QString::Null(), int plane = -1);
        ito::RetVal deleteMarkers(int id);

        void userInteractionStart(int type, bool start, int maxNrOfPoints = -1);
        ito::RetVal clearGeometricElements(void);

        QSharedPointer<ito::DataObject> getDisplayed(void);
        
    private slots:
        void resizeEvent ( QResizeEvent * event );

        void mnuMarkerClick(bool checked);
        void mnuPanner(bool checked);
        void mnuScaleSetting();
        void mnuParentScaleSetting();
        void mnuCmplxSwitch(QAction *action);
        void mnuSetMarker(QAction *action);
        void mnuZoomer(bool checked);
        void mnuExport();

        void mnuActRatio(bool checked);
        void mnuDrawMode(QAction *action);
        void mnuDrawMode(bool checked);

        void mnuHome();
        void setMarkerText(const QString &coords, const QString &offsets);

    signals:
        void userInteractionDone(int type, bool aborted, QPolygonF points);
        void plotItemChanged(ito::int32 idx, ito::int32 flags, QVector<ito::float32> values);
        void plotItemDeleted(ito::int32 idx);
        void plotItemsDeleted();
        void plotItemsFinished(int type, bool aborted);
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
