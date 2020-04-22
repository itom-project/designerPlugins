/* ********************************************************************
   itom measurement system
   URL: http://www.uni-stuttgart.de/ito
   Copyright (C) 2019, Institut fuer Technische Optik (ITO), 
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

class Itom2dQwtPlotPrivate;

class ITOM2DPLOT_EXPORT Itom2dQwtPlot : public ItomQwtDObjFigure
{
    Q_OBJECT

    //DESIGNABLE (default: true): property is visible in QtDesigner property editor
    //USER (default: false): property is visible in property editor of plot

    Q_PROPERTY(QString title READ getTitle WRITE setTitle RESET resetTitle USER true)
    Q_PROPERTY(QString xAxisLabel READ getxAxisLabel WRITE setxAxisLabel RESET resetxAxisLabel USER true)
    Q_PROPERTY(bool xAxisVisible READ getxAxisVisible WRITE setxAxisVisible USER true)
    Q_PROPERTY(QString yAxisLabel READ getyAxisLabel WRITE setyAxisLabel RESET resetyAxisLabel USER true)
    Q_PROPERTY(bool yAxisVisible READ getyAxisVisible WRITE setyAxisVisible USER true)
    Q_PROPERTY(bool yAxisFlipped READ getyAxisFlipped WRITE setyAxisFlipped USER true)
    Q_PROPERTY(QString valueLabel READ getValueLabel WRITE setValueLabel RESET resetValueLabel USER true)
	Q_PROPERTY(ItomQwtPlotEnums::ScaleEngine valueScale READ getValueScale WRITE setValueScale USER true);
    Q_PROPERTY(bool colorBarVisible READ colorBarVisible WRITE setColorBarVisible DESIGNABLE true USER true)
    Q_PROPERTY(QString colorMap READ getColorMap WRITE setColorMap DESIGNABLE true USER true)
    Q_PROPERTY(QFont titleFont READ getTitleFont WRITE setTitleFont USER true)
    Q_PROPERTY(QFont labelFont READ getLabelFont WRITE setLabelFont USER true)
    Q_PROPERTY(QFont axisFont READ getAxisFont WRITE setAxisFont USER true)
    Q_PROPERTY(bool showCenterMarker READ getEnabledCenterMarker WRITE setEnabledCenterMarker USER true)

    Q_PROPERTY(GridStyle grid READ getGrid WRITE setGrid USER true)

    Q_PROPERTY(QSharedPointer< ito::DataObject > overlayImage READ getOverlayImage WRITE setOverlayImage RESET resetOverlayImage DESIGNABLE false)
    Q_PROPERTY(int overlayAlpha READ getOverlayAlpha WRITE setOverlayAlpha RESET resetOverlayAlpha USER true)
    Q_PROPERTY(ito::AutoInterval overlayInterval READ getOverlayInterval WRITE setOverlayInterval DESIGNABLE false USER true)
    Q_PROPERTY(QString overlayColorMap READ getOverlayColorMap WRITE setOverlayColorMap DESIGNABLE false USER true)

    Q_PROPERTY(QSharedPointer <ito::DataObject> contourLevels READ getContourLevels WRITE setContourLevels RESET resetContourLevels DESIGNABLE false)
    Q_PROPERTY(QString contourColorMap READ getContourColorMap WRITE setContourColorMap DESIGNABLE true USER true)
    Q_PROPERTY(float contourLineWidth READ getContourLineWidth WRITE setContourLineWidth DESIGNABLE true USER true);

    Q_PROPERTY(QSharedPointer< ito::DataObject > lineCutData READ getDisplayedLineCut DESIGNABLE false)
    Q_PROPERTY(QVector<QPointF> bounds READ getBounds WRITE setBounds DESIGNABLE false)
    
    Q_PROPERTY(int planeIndex READ getPlaneIndex WRITE setPlaneIndex USER true)
    Q_PROPERTY(ItomQwtPlotEnums::DataChannel dataChannel READ getDataChannel WRITE setDataChannel  RESET resetDataChannel DESIGNABLE true USER true);

    
    Q_PROPERTY(ito::ItomPlotHandle lineCutPlotItem READ getLineCutPlotItem WRITE setLineCutPlotItem DESIGNABLE false)
    Q_PROPERTY(ito::ItomPlotHandle zSlicePlotItem READ getZSlicePlotItem WRITE setZSlicePlotItem DESIGNABLE false)
    Q_PROPERTY(ito::ItomPlotHandle volumeCutPlotItem READ getVolumeCutPlotItem WRITE setVolumeCutPlotItem DESIGNABLE false)

#if QT_VERSION < 0x050500
    //for >= Qt 5.5.0 see Q_ENUM definition below
    Q_ENUMS(GridStyle);
#endif

    Q_CLASSINFO("prop://title", "Title of the plot or '<auto>' if the title of the data object should be used.")
    Q_CLASSINFO("prop://xAxisLabel", "Label of the x-axis or '<auto>' if the description from the data object should be used.")
    Q_CLASSINFO("prop://xAxisVisible", "Sets visibility of the x-axis.")
    Q_CLASSINFO("prop://yAxisLabel", "Label of the y-axis or '<auto>' if the description from the data object should be used.")
    Q_CLASSINFO("prop://yAxisVisible", "Sets visibility of the y-axis.")
    Q_CLASSINFO("prop://yAxisFlipped", "Sets whether y-axis should be flipped (default: false, zero is at the bottom).")
    Q_CLASSINFO("prop://valueLabel", "Label of the value axis or '<auto>' if the description should be used from data object.")
	Q_CLASSINFO("prop://valueScale", "linear or logarithmic scale (various bases) can be chosen for the value axis (color bar). Please consider, that a logarithmic scale can only display values > 1e-100 while the lower limit for the double-logarithmic scale is 1+1e-100.")
    Q_CLASSINFO("prop://colorBarVisible", "Defines whether the color bar should be visible.")
    Q_CLASSINFO("prop://colorMap", "Defines which color map should be used [e.g. grayMarked, hotIron].")
    Q_CLASSINFO("prop://titleFont", "Font for title.")
    Q_CLASSINFO("prop://labelFont", "Font for axes descriptions.")
    Q_CLASSINFO("prop://axisFont", "Font for axes tick values.")
    Q_CLASSINFO("prop://showCenterMarker", "Shows or hides a marker for the center of a data object.")
    Q_CLASSINFO("prop://grid", "Style of the grid ('GridNo', 'GridMajorXY', 'GridMajorX', 'GridMajorY', 'GridMinorXY', 'GridMinorX', 'GridMinorY').")

    Q_CLASSINFO("prop://contourLevels", "Defines which contour levels should be plotted. Each value inside the given dataObject results in one contour level. Possible types are uint8, int8, uint16, int16, int32, float32 and float64.")
    Q_CLASSINFO("prop://contourLineWidth", "Defines the line width of the contour lines")
    Q_CLASSINFO("prop://contourColorMap", "Defines which color map should be used for the contour lines [e.g. gray, grayMarked, falseColor, falseColorIR, hotIron, red, blue, green, viridis].")

    Q_CLASSINFO("prop://overlayImage", "Set an overlay dataObject which is shown above the main dataObject and whose opacity (see 'overlayAlpha') can be controlled by a slider in the toolbar. Assign None to remove the overlay object.")
    Q_CLASSINFO("prop://overlayAlpha", "Changes the value of the overlay channel")        
    Q_CLASSINFO("prop://overlayInterval", "Range of the overlayInterval to scale the values")    
    Q_CLASSINFO("prop://overlayColorMap", "Defines which color map should be used for the overlay channel [e.g. gray, grayMarked, falseColor, falseColorIR, hotIron, red, blue, green, viridis].")

    Q_CLASSINFO("prop://lineCutData", "Get the currently displayed slices from the child lineplot")    

    Q_CLASSINFO("prop://planeIndex", "Plane index of currently visible plane.")
    Q_CLASSINFO("prop://dataChannel", "Type of visualized dataChannel. This is only considered for rgba32 dataObjects, in all other cases this property is ignored.")

    Q_CLASSINFO("prop://lineCutPlotItem", "Set/get the uiItem of the current line plot respective the destination line plot for lateral slicing. The 'uiItem' can be savely cast to 'plotItem'.")
    Q_CLASSINFO("prop://zSlicePlotItem", "Set/get the uiItem of the current line plot respective the destination line plot for z slicing. The 'uiItem' can be savely cast to 'plotItem'.")
    Q_CLASSINFO("prop://volumeCutPlotItem", "Set/get the uiItem of the current line plot respective the destination line plot for the volume cut. The 'uiItem' can be savely cast to 'plotItem'.")
    Q_CLASSINFO("slot://getDisplayed", "returns the currently displayed dataObject.")
    Q_CLASSINFO("slot://getDisplayedLineCut", "returns the currently displayed line cut dataObject")

    Q_CLASSINFO("slot://setLinePlot", "displays a line cut plot with the given bounds.\n"
    "\n"
    "Parameters\n"
    "-------------\n"
    "x0 : {int}\n"
    "    x-coordinate (physical units) of the first end point of the line cut.\n"
    "y0 : {int}\n"
    "    y-coordinate (physical units) of the first end point of the line cut.\n"
    "x1 : {int}\n"
    "    x-coordinate (physical units) of the first end point of the line cut.\n"
    "y1 : {int}\n"
    "    y-coordinate (physical units) of the second end point of the line cut.\n"
    "destID : {int}\n"
    "    optional and unused")

    Q_CLASSINFO("slot://removeOverlayImage", "removes an overlay image. This is the same than assigning 'None' to the property 'overlayImage'")

    Q_CLASSINFO("signal://planeIndexChanged", "This signal is emitted whenever the displayed plane in a 3D dataObject is changed\n"
    "\n"
    "Parameters\n"
    "-------------\n"
    "plane : {int}\n"
    "    index of the displayed plane in the dataObject")

public:
    Itom2dQwtPlot(QWidget *parent = 0);
    Itom2dQwtPlot(const QString &itomSettingsFile, AbstractFigure::WindowMode windowMode, QWidget *parent = 0);
    ~Itom2dQwtPlot();

    enum GridStyle { GridNo = 0, GridMajorXY = 1, GridMajorX = 2, GridMajorY = 3, GridMinorXY = 4, GridMinorX = 5, GridMinorY = 6 };


#if QT_VERSION >= 0x050500
    //Q_ENUM exposes a meta object to the enumeration types, such that the key names for the enumeration
    //values are always accessible.
    Q_ENUM(GridStyle);
#endif

    ito::RetVal displayVolumeCut(const QVector<QPointF> &bounds, ito::uint32 *childFigureUID = NULL);
    ito::RetVal displayZStackCut(const QVector<QPointF> &bounds, ito::uint32 *childFigureUID = NULL);
    ito::RetVal displayLineCut(const QVector<QPointF> &bounds, ito::uint32 *childFigureUID = NULL);

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

    GridStyle getGrid(void) const;
    void setGrid(const GridStyle &gridStyle);

    QSharedPointer< ito::DataObject > getContourLevels() const;
    void setContourLevels(QSharedPointer< ito::DataObject > newContourLevels);
    void resetContourLevels(void);

    void setContourColorMap(const QString &name);
    QString getContourColorMap() const;

    void setContourLineWidth(const float& width);
    float getContourLineWidth() const;

    int getPlaneIndex() const;
    void setPlaneIndex(const int &index);

    ItomQwtPlotEnums::DataChannel getDataChannel() const;
    void setDataChannel(const ItomQwtPlotEnums::DataChannel &dataChannel);
    void resetDataChannel();

	ItomQwtPlotEnums::ScaleEngine getValueScale() const;
	void setValueScale(const ItomQwtPlotEnums::ScaleEngine &scale);
    
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

    //!> set a line cut id for lateral slices
    ito::ItomPlotHandle getLineCutPlotItem() const;

    //!> return the current line cut id for lateral slices
    void setLineCutPlotItem(const ito::ItomPlotHandle &plotHandle);

    //!> set a line cut id for zSlices
    ito::ItomPlotHandle getZSlicePlotItem() const;

    //!> return the current line cut id for zSlices
    void setZSlicePlotItem(const ito::ItomPlotHandle &plotHandle);
    
    //!> set a line cut id for volume cut
    ito::ItomPlotHandle getVolumeCutPlotItem() const;

    //!> return the current line cut id for volume cut
    void setVolumeCutPlotItem(const ito::ItomPlotHandle &plotHandle);

    //!> set the bounds for volume cut
    void setBounds(QVector<QPointF> bounds);

    //!> get the bounds for volume cut
    QVector<QPointF> getBounds() const; 

	ItomQwtPlotEnums::ComplexType getComplexStyle() const;
	void setComplexStyle(const ItomQwtPlotEnums::ComplexType &type);


    friend class PlotCanvas;

protected:
    ito::RetVal init(); //called when api-pointers are transmitted, directly after construction

    ito::RetVal moveChildPlotCloseToThis(QWidget *child); //tries to place a line cut, volume cut ... a little bit to the right / bottom of this plot, however still in the same screen

private:
    void constructor();

    PlotCanvas *m_pContent;   

    //avoid to add private members but put them in the Itom2dQwtPlotPrivate container
    //since this file is part of the itom SDK and can be included in other plugin's source code.
    //The container is defined in the cpp file only, therefore members can be changed there, without
    //breaking the binary compatibility.
    Itom2dQwtPlotPrivate *d;

private slots:
    void childFigureDestroyed(QObject *obj);
    void activatePlot();

public slots:
    QSharedPointer<ito::DataObject> getDisplayed();

    QSharedPointer<ito::DataObject> getDisplayedLineCut(void);

    //this can be invoked by python to trigger a lineplot
    ito::RetVal setLinePlot(const double x0, const double y0, const double x1, const double y1, const int destID = -1);

    void removeOverlayImage(void) { return resetOverlayImage();}

signals:
    void planeIndexChanged(int planeIndex);
};

#endif // ITOM2DQWTPLOT_H
