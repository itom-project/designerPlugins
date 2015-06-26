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

class PlotCanvas;

#if defined(ITOMSHAREDDESIGNER)
    #define ITOM2DPLOT_EXPORT Q_DECL_EXPORT
#else
    #define ITOM2DPLOT_EXPORT Q_DECL_IMPORT
#endif

#include "plot/AbstractDObjFigure.h"
#include "plot/AbstractNode.h"
#include "itom2dqwtplotenums.h"
//#include "plotCanvas.h"
//#include <qwt_plot_shapeitem.h>

#include "common/itomPlotHandle.h"

#include <qaction.h>
#include <qwidgetaction.h>
#include <qspinbox.h>
#include <qslider.h>
#if QT_VERSION >= 0x050000
#include <QtWidgets/qlabel.h>
#endif

#ifndef DECLAREMETADATAOBJECT
    Q_DECLARE_METATYPE(QSharedPointer<ito::DataObject>)
    #define DECLAREMETADATAOBJECT
#endif


#ifndef DECLAREMETAPLOTHANDLE
    Q_DECLARE_METATYPE(ito::ItomPlotHandle)
    #define DECLAREMETAPLOTHANDLE
#endif

class ITOM2DPLOT_EXPORT Itom2dQwtPlot : public ito::AbstractDObjFigure
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
    Q_PROPERTY(QSharedPointer< ito::DataObject > geometricElements READ getGeometricElements WRITE setGeometricElements DESIGNABLE false)
    Q_PROPERTY(int geometricElementsCount READ getGeometricElementsCount DESIGNABLE false)
    Q_PROPERTY(bool keepAspectRatio READ getkeepAspectRatio WRITE setkeepAspectRatio USER true)
    Q_PROPERTY(bool enablePlotting READ getEnabledPlotting WRITE setEnabledPlotting USER true)
    Q_PROPERTY(bool showCenterMarker READ getEnabledCenterMarker WRITE setEnabledCenterMarker USER true)
    Q_PROPERTY(int selectedGeometry READ getSelectedElement WRITE setSelectedElement DESIGNABLE false)
    Q_PROPERTY(bool markerLabelsVisible READ getMarkerLablesVisible WRITE setMarkerLablesVisible DESIGNABLE true)

    Q_PROPERTY(QSharedPointer< ito::DataObject > overlayImage READ getOverlayImage WRITE setOverlayImage RESET resetOverlayImage DESIGNABLE false)
    Q_PROPERTY(int overlayAlpha READ getOverlayAlpha WRITE setOverlayAlpha RESET resetOverlayAlpha USER true)
    Q_PROPERTY(ito::AutoInterval overlayInterval READ getOverlayInterval WRITE setOverlayInterval DESIGNABLE true USER true)
    Q_PROPERTY(QString overlayColorMap READ getOverlayColorMap WRITE setOverlayColorMap DESIGNABLE true USER true)

    Q_PROPERTY(QSharedPointer< ito::DataObject > lineCutData READ getDisplayedLineCut DESIGNABLE false)

    Q_PROPERTY(QColor backgroundColor READ getBackgroundColor WRITE setBackgroundColor USER true)
    Q_PROPERTY(QColor axisColor READ getAxisColor WRITE setAxisColor USER true)
    Q_PROPERTY(QColor textColor READ getTextColor WRITE setTextColor USER true)
    
    Q_PROPERTY(int planeIndex READ getPlaneIndex WRITE setPlaneIndex USER true)

    Q_PROPERTY(Itom2DQwt::tModificationState geometryModMode READ getModState WRITE setModState DESIGNABLE true)
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
    Q_CLASSINFO("prop://geometricElements", "Geometric elements defined by a float32[11] array for each element.")
    Q_CLASSINFO("prop://geometricElementsCount", "Number of currently existing geometric elements.")
    Q_CLASSINFO("prop://keepAspectRatio", "Enable and disable a fixed 1:1 aspect ratio between x and y axis.")
    Q_CLASSINFO("prop://enablePlotting", "Enable and disable internal plotting functions and GUI-elements for geometric elements.")
    Q_CLASSINFO("prop://showCenterMarker", "Enable a marker for the center of a data object.")
    Q_CLASSINFO("prop://selectedGeometry", "Get or set the currently highlighted geometric element. After manipulation the last element stays selected.")
    Q_CLASSINFO("prop://markerLabelsVisible", "Toggle visibility of marker labels.")

    Q_CLASSINFO("prop://overlayImage", "Set an overlay which is shown as a black&white image.")
    Q_CLASSINFO("prop://overlayAlpha", "Changes the value of the overlay channel")        
    Q_CLASSINFO("prop://overlayInterval", "Range of the overlayInterval to scale the values")    

    Q_CLASSINFO("prop://overlayColorMap", "Defines which color map should be used for the overlay channel [e.g. grayMarked, hotIron].")

    Q_CLASSINFO("prop://lineCutData", "Get the currently displayed slices from the child lineplot")    

    Q_CLASSINFO("prop://backgroundColor", "Set the background / canvas color.")
    Q_CLASSINFO("prop://axisColor", "Set the color of the axis.")
    Q_CLASSINFO("prop://textColor", "Set the color of text and tick-numbers.")

    Q_CLASSINFO("prop://planeIndex", "Plane index of currently visible plane.")

    Q_CLASSINFO("prop://geometryModMode", "Change the geometry modification mode (move, resize, rotate).")
    Q_CLASSINFO("prop://lineCutPlotItem", "Set/Get the ui-Handle of the current line plot respective the destination line plot for lateral slicing.")
    Q_CLASSINFO("prop://zSlicePlotItem", "Set/Get the ui-Handle of the current line plot respective the destination line plot for z slicing.")

    Q_CLASSINFO("slot://plotMarkers", "")
    Q_CLASSINFO("slot://deleteMarkers", "Delete a specific marker")  
    //Q_CLASSINFO("slot://deleteMarkers", "Delete a specific marker")
    Q_CLASSINFO("slot://userInteractionStart", "")  
    Q_CLASSINFO("slot://clearGeometricElements", "")
    Q_CLASSINFO("slot://getDisplayed", "")  
    Q_CLASSINFO("slot://getDisplayedLineCut", "")
    Q_CLASSINFO("slot://setLinePlot", "")
    Q_CLASSINFO("slot://removeOverlayImage", "")
    Q_CLASSINFO("slot://copyToClipBoard", "")

    Q_CLASSINFO("slot://setGeometricElementLabel", "Set the label of geometric element with the index id")
    Q_CLASSINFO("slot://setGeometricElementLabelVisible", "Set the visibility of the label of geometric element with the index id")


    Q_CLASSINFO("signal://plotItemsFinished", "Signal emitted when geometrical plotting was finished.") 
    Q_CLASSINFO("signal://userInteractionDone", "")
    Q_CLASSINFO("signal://plotItemChanged", "")
    Q_CLASSINFO("signal://plotItemDeleted", "")
    Q_CLASSINFO("signal://plotItemsDeleted", "")

    DESIGNER_PLUGIN_ITOM_API
    public:
    Itom2dQwtPlot(QWidget *parent = 0);
    Itom2dQwtPlot(const QString &itomSettingsFile, AbstractFigure::WindowMode windowMode, QWidget *parent = 0);
    ~Itom2dQwtPlot();

    enum GeometryModificationMode
    {
        tNextElementMode = 0,
        tMoveGeometricElements = 1,
        tRotateGeometricElements = 2,
        tResizeGeometricElements = 3,
        tModifyPoints   = 4
    }; //!> This enum must be equal to the enum PlotCanvas::tModificationState, todo: find a better solution

    ito::RetVal displayCut(QVector<QPointF> bounds, ito::uint32 &uniqueID, bool zStack = false);

    ito::RetVal applyUpdate();  //!> does the real update work

    //properties (setter/getter)
    void setContextMenuEnabled(bool show);
    bool getContextMenuEnabled() const;

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

    int getGeometricElementsCount() const;

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
    void setCmplxSwitch(/*PlotCanvas::ComplexType*/ int type, bool visible);

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

    bool getkeepAspectRatio(void) const;
    void setkeepAspectRatio(const bool &keepAspectEnable);

    QSharedPointer< ito::DataObject > getGeometricElements();
    void setGeometricElements(QSharedPointer< ito::DataObject > geometricElements);

    bool getEnabledPlotting(void) const;
    void setEnabledPlotting(const bool &enabled);

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

    QSharedPointer< ito::DataObject > getOverlayImage() const;
    void setOverlayImage(QSharedPointer< ito::DataObject > newOverlayObj);
    void resetOverlayImage(void);

    void enableOverlaySlider(bool enabled) {m_pActOverlaySlider->setVisible(enabled);}

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
    
    bool getMarkerLablesVisible(void) const;
    void setMarkerLablesVisible(const bool val);

    Itom2DQwt::tModificationState getModState() const;
    void setModState(const Itom2DQwt::tModificationState val);

    QPixmap renderToPixMap(const int xsize, const int ysize, const int resolution);



    friend class PlotCanvas;

protected:
    ito::RetVal init(); //called when api-pointers are transmitted, directly after construction

    void createActions();

    //void setLinePlotCoordinates(const QVector<QPointF> pts);
    void setColorDataTypeRepresentation(bool colorOn);

private:
    void constructor();

    PlotCanvas *m_pContent;    
//  InternalData m_data;
    void *m_pVData;

    QAction *m_pActSave;
    QAction *m_pActCopyClipboard;
    QAction *m_pActHome;
    QAction *m_pActPan;
    QAction *m_pActZoom;
    QAction *m_pActSendCurrentToWorkspace;
    QAction *m_pActAspectRatio;
    QAction *m_pActScaleSettings;
    QAction *m_pActColorPalette;
    QAction *m_pActToggleColorBar;
    QAction *m_pActValuePicker;
    QAction *m_pActLineCut;
    QMenu *m_pActLineCutMode;
    QActionGroup *m_pActLineCutGroup;

    QAction *m_pActStackCut;
    QWidgetAction *m_pActPlaneSelector;
    QActionGroup *m_pDrawModeActGroup;
    QActionGroup *m_pDrawModifyModeActGroup;
    QAction *m_pActClearDrawings;
    QAction *m_pActProperties;

    QLabel *m_pCoordinates;
    QWidgetAction *m_pActCoordinates;

    QAction* m_pActCmplxSwitch;
    QMenu *m_mnuCmplxSwitch;

    QAction* m_pActDrawMode;
    QMenu *m_pMnuDrawMode;

    QAction* m_pActDrawModifyMode;
    QMenu *m_pMnuDrawModifyMode;

    QAction* m_pActCntrMarker;

    QSlider* m_pOverlaySlider;
    QWidgetAction *m_pActOverlaySlider;

    QHash<QObject*,ito::uint32> m_childFigures;

    ito::RetVal qvector2DataObject(const ito::DataObject *dstObject);
    ito::RetVal exportCanvas(const bool copyToClipboardNotFile, const QString &fileName, QSizeF curSize = QSizeF(0.0,0.0), const int resolution = 300);

private slots:
    void mnuActSave();
    void mnuActSendCurrentToWorkspace();

    void mnuActHome();
    void mnuActPan(bool checked);
    void mnuActZoom(bool checked);
    void mnuActRatio(bool checked);
    void mnuActScaleSettings();
    void mnuActColorPalette();
    void mnuActToggleColorBar(bool checked);
    void mnuActValuePicker(bool checked);
    void mnuActLineCut(bool checked);
    void mnuLineCutMode(QAction *action);
    void mnuActStackCut(bool checked);
    
    
    void mnuActPlaneSelector(int plane);
    void mnuDrawModifyMode(QAction *action);
    void mnuDrawMode(QAction *action);
    void mnuDrawMode(bool checked);
    void mnuOverlaySliderChanged(int value);

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

    QSharedPointer<ito::DataObject> getDisplayedLineCut(void);

    //this can be invoked by python to trigger a lineplot
    ito::RetVal setLinePlot(const double x0, const double y0, const double x1, const double y1, const int destID = -1);

    ito::RetVal setGeometricElementLabel(int id, QString label);
    ito::RetVal setGeometricElementLabelVisible(int id, bool setVisible);

    void removeOverlayImage(void) { return resetOverlayImage();}
    ito::RetVal copyToClipBoard();

signals:
    void userInteractionDone(int type, bool aborted, QPolygonF points);
    void plotItemChanged(int idx, int flags, QVector<float> values);
    void plotItemDeleted(int idx);
    void plotItemsDeleted();
    //void plotItemChanged(ito::int32 idx);
    void plotItemsFinished(int type, bool aborted);

};

#endif // ITOM2DQWTPLOT_H
