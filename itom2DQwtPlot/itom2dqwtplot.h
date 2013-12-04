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

#ifndef ITOM2DQWTPLOT_H
#define ITOM2DQWTPLOT_H

#include "plot/AbstractDObjFigure.h"
#include "plot/AbstractNode.h"

#include "plotCanvas.h"

#include <qaction.h>
#include <qwidgetaction.h>
#include <qspinbox.h>
#include <qwt_plot_shapeitem.h>

Q_DECLARE_METATYPE(QSharedPointer<ito::DataObject>)

class Itom2dQwtPlot : public ito::AbstractDObjFigure
{
    Q_OBJECT

    Q_PROPERTY(QString title READ getTitle WRITE setTitle RESET resetTitle)
    Q_PROPERTY(QString xAxisLabel READ getxAxisLabel WRITE setxAxisLabel RESET resetxAxisLabel)
    Q_PROPERTY(bool xAxisVisible READ getxAxisVisible WRITE setxAxisVisible)
    Q_PROPERTY(QString yAxisLabel READ getyAxisLabel WRITE setyAxisLabel RESET resetyAxisLabel)
    Q_PROPERTY(bool yAxisVisible READ getyAxisVisible WRITE setyAxisVisible)
    Q_PROPERTY(bool yAxisFlipped READ getyAxisFlipped WRITE setyAxisFlipped)
    Q_PROPERTY(QString valueLabel READ getValueLabel WRITE setValueLabel RESET resetValueLabel)
    Q_PROPERTY(bool colorBarVisible READ colorBarVisible WRITE setColorBarVisible DESIGNABLE true)
    Q_PROPERTY(QString colorBar READ getColorMap WRITE setColorMap DESIGNABLE true)
    Q_PROPERTY(QFont titleFont READ getTitleFont WRITE setTitleFont)
    Q_PROPERTY(QFont labelFont READ getLabelFont WRITE setLabelFont)
    Q_PROPERTY(QFont axisFont READ getAxisFont WRITE setAxisFont)
    Q_PROPERTY(QSharedPointer< ito::DataObject > geometricElements READ getGeometricElements WRITE setGeometricElements DESIGNABLE false)
    Q_PROPERTY(int geometricElementsCount READ getGeometricElementsCount WRITE setGeometricElementsCount DESIGNABLE false)
    Q_PROPERTY(bool keepAspectRatio READ getkeepAspectRatio WRITE setkeepAspectRatio)
    Q_PROPERTY(bool enablePlotting READ getEnabledPlotting WRITE setEnabledPlotting)
    Q_PROPERTY(bool showCenterMarker READ getEnabledCenterMarker WRITE setEnabledCenterMarker)

    Q_CLASSINFO("prop://title", "Title of the plot or '<auto>' if the title of the data object should be used.")
    Q_CLASSINFO("prop://xAxisLabel", "Label of the x-axis or '<auto>' if the description from the data object should be used.")
    Q_CLASSINFO("prop://xAxisVisible", "Sets visibility of the x-axis.")
    Q_CLASSINFO("prop://yAxisLabel", "Label of the y-axis or '<auto>' if the description from the data object should be used.")
    Q_CLASSINFO("prop://yAxisVisible", "Sets visibility of the y-axis.")
    Q_CLASSINFO("prop://yAxisFlipped", "Sets whether y-axis should be flipped (default: false, zero is at the bottom).")
    Q_CLASSINFO("prop://valueLabel", "Label of the value axis or '<auto>' if the description should be used from data object.")
    Q_CLASSINFO("prop://colorBarVisible", "Defines whether the color bar should be visible.")
    Q_CLASSINFO("prop://colorBar", "Defines which color bar should be used [e.g. grayMarked, hotIron].")
    Q_CLASSINFO("prop://titleFont", "Font for title.")
    Q_CLASSINFO("prop://labelFont", "Font for axes descriptions.")
    Q_CLASSINFO("prop://axisFont", "Font for axes tick values.")
    Q_CLASSINFO("prop://geometricElements", "Geometric elements defined by a float32[11] array for each element.")
    Q_CLASSINFO("prop://geometricElementsCount", "Number of currently existing geometric elements.")
    Q_CLASSINFO("prop://keepAspectRatio", "Enable and disable a fixed 1:1 aspect ratio between x and y axis.")
    Q_CLASSINFO("prop://enablePlotting", "Enable and disable internal plotting functions and GUI-elements for geometric elements.")
    Q_CLASSINFO("prop://showCenterMarker", "Enable a marker for the center of a data object.")

public:
    Itom2dQwtPlot(const QString &itomSettingsFile, AbstractFigure::WindowMode windowMode, QWidget *parent = 0);
    ~Itom2dQwtPlot();

    ito::RetVal displayCut(QVector<QPointF> bounds, ito::uint32 &uniqueID, bool zStack = false);

    ito::RetVal applyUpdate();  //!> does the real update work

	//properties (setter/getter)
    void setContextMenuEnabled(bool show) {}
    bool getContextMenuEnabled() const { return false; }

    bool colorBarVisible() const;
    void setColorBarVisible(bool value);

    QString getTitle() const;
    void setTitle(const QString &title);
    void resetTitle();

    QString getxAxisLabel() const;
    void setxAxisLabel(const QString &label);
    void resetxAxisLabel();

    QString getyAxisLabel() const;
    void setyAxisLabel(const QString &label);
    void resetyAxisLabel();

    QString getValueLabel() const;
    void setValueLabel(const QString &label);
    void resetValueLabel();

    int getGeometricElementsCount() const { return m_data.m_pDrawItems.size();}
    void setGeometricElementsCount(const int value){ return;}

    bool getyAxisFlipped() const;
    void setyAxisFlipped(const bool &value);

    bool getxAxisVisible() const;
    void setxAxisVisible(const bool &value);
    
    bool getyAxisVisible() const;
    void setyAxisVisible(const bool &value);

    QString getColorMap() const;
    void setColorMap(const QString &name);

    void setPlaneRange(int min, int max);
    void setCmplxSwitch(PlotCanvas::ComplexType type, bool visible);

    virtual QPointF getXAxisInterval(void) const;
    virtual void setXAxisInterval(QPointF point);
        
    virtual QPointF getYAxisInterval(void) const;
    virtual void setYAxisInterval(QPointF point);
        
    virtual QPointF getZAxisInterval(void) const;
    virtual void setZAxisInterval(QPointF point);

    QFont getTitleFont(void) const;
    void setTitleFont(const QFont &font);

    QFont getLabelFont(void) const;
    void setLabelFont(const QFont &font);

    QFont getAxisFont(void) const;
    void setAxisFont(const QFont &font);

    bool getkeepAspectRatio(void) const {return this->m_data.m_keepAspect;}
    void setkeepAspectRatio(const bool &keepAspectEnable);

    QSharedPointer< ito::DataObject > getGeometricElements();
    void setGeometricElements(QSharedPointer< ito::DataObject > geometricElements);

    bool getEnabledPlotting(void) const {return m_data.m_enablePlotting;}
    void setEnabledPlotting(const bool &enabled);

    bool getEnabledCenterMarker(void) const {return m_data.m_showCenterMarker;}
    void setEnabledCenterMarker(const bool &enabled);

    friend class PlotCanvas;

protected:
    ito::RetVal init() { return m_pContent->init(); } //called when api-pointers are transmitted, directly after construction

    void createActions();

    //void setLinePlotCoordinates(const QVector<QPointF> pts);

private:

    PlotCanvas *m_pContent;	
    InternalData m_data;

    QAction *m_pActSave;
    QAction *m_pActHome;
    QAction *m_pActPan;
    QAction *m_pActZoom;
    QAction *m_pActAspectRatio;
    QAction *m_pActScaleSettings;
    QAction *m_pActColorPalette;
    QAction *m_pActToggleColorBar;
    QAction *m_pActValuePicker;
    QAction *m_pActLineCut;
    QAction *m_pActStackCut;
    QWidgetAction *m_pActPlaneSelector;
    QActionGroup *m_pDrawModeActGroup;
    QAction *m_pActClearDrawings;

    QLabel *m_pCoordinates;
    QWidgetAction *m_pActCoordinates;

    QAction* m_pActCmplxSwitch;
    QMenu *m_mnuCmplxSwitch;

    QAction* m_pActDrawMode;
    QMenu *m_pMnuDrawMode;

    QAction* m_pActCntrMarker;

    QHash<QObject*,ito::uint32> m_childFigures;

    ito::RetVal qvector2DataObject(const ito::DataObject *dstObject);

private slots:
    void mnuActSave();
    void mnuActHome();
    void mnuActPan(bool checked);
    void mnuActZoom(bool checked);
    void mnuActRatio(bool checked);
    void mnuActScaleSettings();
    void mnuActColorPalette();
    void mnuActToggleColorBar(bool checked);
    void mnuActValuePicker(bool checked);
    void mnuActLineCut(bool checked);
    void mnuActStackCut(bool checked);
    void mnuActPlaneSelector(int plane);
    void mnuDrawMode(QAction *action);
    void mnuDrawMode(bool checked);

    void mnuActCenterMarker(bool checked);

    void mnuCmplxSwitch(QAction *action);
    void childFigureDestroyed(QObject *obj);

private slots:
    void resizeEvent ( QResizeEvent * event );

    void setCoordinates(const QVector<QPointF> &pts, bool visible = true);

public slots:
    ito::RetVal plotMarkers(const ito::DataObject &coords, QString style, QString id = QString::Null(), int plane = -1);
    ito::RetVal deleteMarkers(QString id);
    ito::RetVal deleteMarkers(int id);

    void userInteractionStart(int type, bool start, int maxNrOfPoints = -1);
    ito::RetVal clearGeometricElements(void);
//    void userInteractionEndRect(const QRectF &rect);
//    void userInteractionEndEllipse(const QRectF &rect);    
//    void userInteractionEndPt(const QVector<QPointF> &points);
//    void userInteractionEndLine(const QVector<QPointF> &points);

    QSharedPointer<ito::DataObject> getDisplayed(void);

signals:
    void userInteractionDone(int type, bool aborted, QPolygonF points);
    void plotItemChanged(ito::int32 idx, ito::int32 flags, QVector<ito::float32> values);
    void plotItemDeleted(ito::int32 idx);
    void plotItemsDeleted();
    //void plotItemChanged(ito::int32 idx);
    void plotItemsFinished(int type, bool aborted);

};

#endif // ITOM2DQWTPLOT_H
