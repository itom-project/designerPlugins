/* ********************************************************************
   itom measurement system
   URL: http://www.uni-stuttgart.de/ito
   Copyright (C) 2012, Institut fuer Technische Optik (ITO), 
   Universitaet Stuttgart, Germany 
 
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

class PlotCanvas;

#if defined(ITOMSHAREDDESIGNER)
    #define ITOM2DPLOT_EXPORT Q_DECL_EXPORT
#else
    #define ITOM2DPLOT_EXPORT Q_DECL_IMPORT
#endif

#include "itomQwtDObjFigure.h"
#include "plot/AbstractNode.h"

#include <qaction.h>
#include <qwidgetaction.h>
#include <qspinbox.h>
#include <qslider.h>
#include <qlabel.h>

class InternalData;

class ITOM2DPLOT_EXPORT Itom2dQwtPlot : public ItomQwtDObjFigure
{
    Q_OBJECT

    Q_PROPERTY(QString title READ getTitle WRITE setTitle RESET resetTitle USER true)
    Q_PROPERTY(QString xAxisLabel READ getxAxisLabel WRITE setxAxisLabel RESET resetxAxisLabel USER true)
    Q_PROPERTY(bool xAxisVisible READ getxAxisVisible WRITE setxAxisVisible USER true)
    Q_PROPERTY(QString yAxisLabel READ getyAxisLabel WRITE setyAxisLabel RESET resetyAxisLabel USER true)
    Q_PROPERTY(bool yAxisVisible READ getyAxisVisible WRITE setyAxisVisible USER true)
    Q_PROPERTY(bool yAxisFlipped READ getyAxisFlipped WRITE setyAxisFlipped USER true)
    Q_PROPERTY(QString valueLabel READ getValueLabel WRITE setValueLabel RESET resetValueLabel USER true)
    Q_PROPERTY(bool colorBarVisible READ colorBarVisible WRITE setColorBarVisible DESIGNABLE true USER true)
    Q_PROPERTY(QString colorMap READ getColorMap WRITE setColorMap DESIGNABLE true USER true)
    Q_PROPERTY(QFont titleFont READ getTitleFont WRITE setTitleFont USER true)
    Q_PROPERTY(QFont labelFont READ getLabelFont WRITE setLabelFont USER true)
    Q_PROPERTY(QFont axisFont READ getAxisFont WRITE setAxisFont USER true)
    Q_PROPERTY(bool showCenterMarker READ getEnabledCenterMarker WRITE setEnabledCenterMarker USER true)

    Q_PROPERTY(QSharedPointer< ito::DataObject > overlayImage READ getOverlayImage WRITE setOverlayImage RESET resetOverlayImage DESIGNABLE false)
    Q_PROPERTY(int overlayAlpha READ getOverlayAlpha WRITE setOverlayAlpha RESET resetOverlayAlpha USER true)
    Q_PROPERTY(ito::AutoInterval overlayInterval READ getOverlayInterval WRITE setOverlayInterval DESIGNABLE true USER true)
    Q_PROPERTY(QString overlayColorMap READ getOverlayColorMap WRITE setOverlayColorMap DESIGNABLE true USER true)

    Q_PROPERTY(QSharedPointer< ito::DataObject > lineCutData READ getDisplayedLineCut DESIGNABLE false)

    Q_PROPERTY(QColor backgroundColor READ getBackgroundColor WRITE setBackgroundColor USER true)
    Q_PROPERTY(QColor axisColor READ getAxisColor WRITE setAxisColor USER true)
    Q_PROPERTY(QColor textColor READ getTextColor WRITE setTextColor USER true)
    
    Q_PROPERTY(int planeIndex READ getPlaneIndex WRITE setPlaneIndex USER true)

    
    Q_PROPERTY(ito::ItomPlotHandle lineCutPlotItem READ getLineCutPlotItem WRITE setLineCutPlotItem DESIGNABLE false USER true)
    Q_PROPERTY(ito::ItomPlotHandle zSlicePlotItem READ getZSlicePlotItem WRITE setZSlicePlotItem DESIGNABLE false USER true)
    
    Q_CLASSINFO("prop://title", "Title of the plot or '<auto>' if the title of the data object should be used.")
    Q_CLASSINFO("prop://xAxisLabel", "Label of the x-axis or '<auto>' if the description from the data object should be used.")
    Q_CLASSINFO("prop://xAxisVisible", "Sets visibility of the x-axis.")
    Q_CLASSINFO("prop://yAxisLabel", "Label of the y-axis or '<auto>' if the description from the data object should be used.")
    Q_CLASSINFO("prop://yAxisVisible", "Sets visibility of the y-axis.")
    Q_CLASSINFO("prop://yAxisFlipped", "Sets whether y-axis should be flipped (default: false, zero is at the bottom).")
    Q_CLASSINFO("prop://valueLabel", "Label of the value axis or '<auto>' if the description should be used from data object.")
    Q_CLASSINFO("prop://colorBarVisible", "Defines whether the color bar should be visible.")
    Q_CLASSINFO("prop://colorMap", "Defines which color map should be used [e.g. grayMarked, hotIron].")
    Q_CLASSINFO("prop://titleFont", "Font for title.")
    Q_CLASSINFO("prop://labelFont", "Font for axes descriptions.")
    Q_CLASSINFO("prop://axisFont", "Font for axes tick values.")
    Q_CLASSINFO("prop://showCenterMarker", "Enable a marker for the center of a data object.")

    Q_CLASSINFO("prop://overlayImage", "Set an overlay which is shown as a black&white image.")
    Q_CLASSINFO("prop://overlayAlpha", "Changes the value of the overlay channel")        
    Q_CLASSINFO("prop://overlayInterval", "Range of the overlayInterval to scale the values")    

    Q_CLASSINFO("prop://overlayColorMap", "Defines which color map should be used for the overlay channel [e.g. grayMarked, hotIron].")

    Q_CLASSINFO("prop://lineCutData", "Get the currently displayed slices from the child lineplot")    

    Q_CLASSINFO("prop://backgroundColor", "Set the background / canvas color.")
    Q_CLASSINFO("prop://axisColor", "Set the color of the axis.")
    Q_CLASSINFO("prop://textColor", "Set the color of text and tick-numbers.")

    Q_CLASSINFO("prop://planeIndex", "Plane index of currently visible plane.")

    Q_CLASSINFO("prop://lineCutPlotItem", "Set/Get the ui-Handle of the current line plot respective the destination line plot for lateral slicing.")
    Q_CLASSINFO("prop://zSlicePlotItem", "Set/Get the ui-Handle of the current line plot respective the destination line plot for z slicing.")
      
    //Q_CLASSINFO("slot://deleteMarkers", "Delete a specific marker")
    Q_CLASSINFO("slot://getDisplayed", "")  
    Q_CLASSINFO("slot://getDisplayedLineCut", "")
    Q_CLASSINFO("slot://setLinePlot", "")
    Q_CLASSINFO("slot://removeOverlayImage", "")

public:
    Itom2dQwtPlot(QWidget *parent = 0);
    Itom2dQwtPlot(const QString &itomSettingsFile, AbstractFigure::WindowMode windowMode, QWidget *parent = 0);
    ~Itom2dQwtPlot();

    ito::RetVal displayCut(QVector<QPointF> bounds, ito::uint32 &uniqueID, bool zStack = false);

    ito::RetVal applyUpdate();  //!> does the real update work

    //properties (setter/getter)
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

    bool getyAxisFlipped() const;
    void setyAxisFlipped(const bool &value);

    bool getxAxisVisible() const;
    void setxAxisVisible(const bool &value);
    
    bool getyAxisVisible() const;
    void setyAxisVisible(const bool &value);

    QString getColorMap() const;
    void setColorMap(const QString &name);

    QString getOverlayColorMap() const;
    void setOverlayColorMap(const QString &name);

    int getPlaneIndex() const;
    void setPlaneIndex(const int &index);
    
    void setPlaneRange(int min, int max);

    virtual ito::AutoInterval getXAxisInterval(void) const;
    virtual void setXAxisInterval(ito::AutoInterval interval);
        
    virtual ito::AutoInterval getYAxisInterval(void) const;
    virtual void setYAxisInterval(ito::AutoInterval interval);
        
    virtual ito::AutoInterval getZAxisInterval(void) const;
    virtual void setZAxisInterval(ito::AutoInterval interval);

    ito::AutoInterval getOverlayInterval(void) const;
    void setOverlayInterval(ito::AutoInterval interval);

    QFont getTitleFont(void) const;
    void setTitleFont(const QFont &font);

    QFont getLabelFont(void) const;
    void setLabelFont(const QFont &font);

    QFont getAxisFont(void) const;
    void setAxisFont(const QFont &font);

    bool getEnabledCenterMarker(void) const;
    void setEnabledCenterMarker(const bool &enabled);

    int getSelectedElement(void) const;
    void setSelectedElement(const int idx);

    int getOverlayAlpha () const;
    void setOverlayAlpha (const int alpha);

    void resetOverlayAlpha(void)
    {
        setOverlayAlpha(0);
    }

    void setUnitLabelStyle(const ito::AbstractFigure::UnitLabelStyle &style);

    QSharedPointer< ito::DataObject > getOverlayImage() const;
    void setOverlayImage(QSharedPointer< ito::DataObject > newOverlayObj);
    void resetOverlayImage(void);

    void enableOverlaySlider(bool enabled);

    //!> set new background color
    void setBackgroundColor(const QColor newVal);

    //!> get current background color
    QColor getBackgroundColor(void) const;

    /** set color of axis
    *   @param [in] newVal  new axis color
    */
    void setAxisColor(const QColor newVal);

    //!> return axis color
    QColor getAxisColor(void) const;

    /** set text color
    *   @param [in] newVal  new text color
    */
    void setTextColor(const QColor newVal);

    //!> return text color
    QColor getTextColor(void) const;

    //!> set a line cut id for lateral slices
    ito::ItomPlotHandle getLineCutPlotItem() const;

    //!> return the current line cut id for lateral slices
    void setLineCutPlotItem(const ito::ItomPlotHandle idx);

    //!> set a line cut id for zSlices
    ito::ItomPlotHandle getZSlicePlotItem() const;

    //!> return the current line cut id for zSlices
    void setZSlicePlotItem(const ito::ItomPlotHandle idx);


    friend class PlotCanvas;

protected:
    ito::RetVal init(); //called when api-pointers are transmitted, directly after construction

private:
    void constructor();

    PlotCanvas *m_pContent;    
    InternalData *m_pData;

    QHash<QObject*,ito::uint32> m_childFigures;

private slots:
    
    
    
    void childFigureDestroyed(QObject *obj);

    

public slots:
    

    QSharedPointer<ito::DataObject> getDisplayed(void);

    QSharedPointer<ito::DataObject> getDisplayedLineCut(void);

    //this can be invoked by python to trigger a lineplot
    ito::RetVal setLinePlot(const double x0, const double y0, const double x1, const double y1, const int destID = -1);

    void removeOverlayImage(void) { return resetOverlayImage();}



};

#endif // ITOM2DQWTPLOT_H
