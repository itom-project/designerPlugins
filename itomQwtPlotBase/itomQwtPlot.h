/* ********************************************************************
   itom measurement system
   URL: http://www.uni-stuttgart.de/ito
   Copyright (C) 2018, Institut fuer Technische Optik (ITO), 
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

#ifndef ITOMQWTPLOT_H
#define ITOMQWTPLOT_H

#include "itomQwtPlotBase.h"

#include <qwt_plot.h>

#include "plot/AbstractFigure.h"
#include "itomQwtPlotEnums.h"

#include <qhash.h>
#include <qcolor.h>
#include <qvector.h>
#include <qsharedpointer.h>
#include <qsize.h>
#include <qstring.h>
#include <qprinter.h>

#include "DataObject/dataobj.h"
#include "common/shape.h"

class QMenu;
class ItomPlotZoomer;
class ItomPlotMagnifier;
class ItomQwtPlotPanner;
class QwtPlotMarker;
class QContextMenuEvent;
class QResizeEvent;
class DrawItem;
class QKeyEvent;
class QMouseEvent;
class QAction;
class QActionGroup;
class QToolBar;
class UserInteractionPlotPicker;
class ItomQwtDObjFigure;

class ITOMQWTPLOTBASE_EXPORT ItomQwtPlot : public QwtPlot
{
    Q_OBJECT

public:
    enum State
    {
        stateIdle = 0,
        stateZoom = 1,
        statePan = 2,
        stateValuePicker = 3,
        stateLineCut = 4,
        stateStackCut = 5,
        stateDrawShape = 6,
        stateMarkerPicker = 7,
        stateVolumeCut = 8,
        stateUser = 100
    };

    explicit ItomQwtPlot(ItomQwtDObjFigure *parent = NULL);
    virtual ~ItomQwtPlot();

    bool showContextMenu() const { return m_showContextMenu; }
    void setShowContextMenu(bool value) { m_showContextMenu = value; }

    bool keepAspectRatio() const { return m_keepAspectRatio; }
    void setKeepAspectRatio(bool keep);

    bool markerLabelVisible() const { return m_markerLabelVisible; }
    void setMarkerLabelVisible(bool visible);

    bool shapesLabelVisible() const { return m_shapesLabelVisible; }
    void setShapesLabelVisible(bool visible);

    virtual void setButtonStyle(int style); /*!< can be overwritten, however call this base implementation in the overloaded method, too.*/
    int buttonStyle() const { return m_buttonStyle; }

    void setBoxFrame(bool boxFrame);
    int boxFrame() const { return m_boxFrame; }

    virtual void setPlottingEnabled(bool enabled); /*!< can be overwritten, however call this base implementation in the overloaded method, too.*/
    bool plottingEnabled() const { return m_plottingEnabled; }

    ito::AbstractFigure::UnitLabelStyle unitLabelStyle() const { return m_unitLabelStyle; }
    virtual void setUnitLabelStyle(ito::AbstractFigure::UnitLabelStyle style) 
	{ 
		m_unitLabelStyle = style; 
	};

    ItomQwtPlotEnums::ModificationModes shapeModificationModes() const { return m_shapeModificationModes; }
    void setShapeModificationModes(const ItomQwtPlotEnums::ModificationModes &modes);

    ItomQwtPlotEnums::ShapeTypes allowedGeometricShapes() const { return m_allowedShapeTypes; }
    void setAllowedGeometricShapes(const ItomQwtPlotEnums::ShapeTypes &allowedTypes);

    QColor backgroundColor() const { return m_backgroundColor; }
    void setBackgroundColor(const QColor &color);

    QColor canvasColor() const { return m_canvasColor; }
    void setCanvasColor(const QColor &color);

    QColor axisColor() const { return m_axisColor; }
    void setAxisColor(const QColor &color);

    QColor textColor() const { return m_textColor; }
    void setTextColor(const QColor &color);

    int getGeometricShapesFillOpacity() const { return m_geometricShapeOpacity; }
    void setGeometricShapesFillOpacity(const int &opacity);

    int getGeometricShapesFillOpacitySelected() const { return m_geometricShapeOpacitySelected; }
    void setGeometricShapesFillOpacitySelected(const int &opacity);

    void setVisible(bool visible);
    bool m_copyDisplayedAsComplex;

    int countGeometricShapes() const { return m_pShapes.count(); }
    int getSelectedGeometricShapeIdx() const;
    void setSelectedGeometricShapeIdx(int idx);
    ito::RetVal setGeometricShapes(const QVector<ito::Shape> &geometricShapes);
    ito::RetVal addGeometricShape(const ito::Shape &geometricShape, int *newIndex = NULL); //add the new shape (only if its index does not already exist)
    ito::RetVal updateGeometricShape(const ito::Shape &geometricShape, int *newIndex = NULL); //modifies the shape with the same index or add the given shape if its index does not already exist.
    ito::RetVal deleteGeometricShape(const int idx);
    QVector<ito::Shape> getGeometricShapes();
    ito::RetVal setGeometricShapeLabelVisible(int idx, bool setVisible);
    ito::RetVal setGeometricShapeLabel(int idx, const QString &label);

    ito::RetVal userInteractionStart(int type, bool start, int maxNrOfPoints);

    virtual QList<QToolBar*> getToolbars() { return m_toolbars; }
    virtual QList<QMenu*> getMenus() { return m_menus; }

    ito::RetVal exportCanvas(const bool copyToClipboardNotFile, const QString &fileName, const QSizeF &curSizeMm = QSizeF(0.0, 0.0), const int resolution = 200);
    ito::RetVal printCanvas();

    ito::RetVal plotMarkers(const QSharedPointer<ito::DataObject> coordinates, const QString &style, const QString &id, int plane);
    ito::RetVal deleteMarkers(const QString &id);

protected:

    void** getItomApiFuncsPtr() const; 
    void** getItomApiFuncsPtrGraph() const;
    
    void loadStyles(bool overwriteDesignableProperties);
    ItomPlotZoomer *zoomer() const;
    ItomQwtPlotPanner *panner() const;
    void configRescaler();

    ito::RetVal changeVisibleMarkers(int currentPlane);

    /*
    @param doReplot forces a replot of the content
    @param doZoomBase if true, the x/y-zoom is reverted to the full x-y-area of the manually set ranges (the same holds for the value range)
    */
    virtual void updateScaleValues(bool doReplot = true, bool doZoomBase = true, bool clearStack = false) = 0; //to be overwritten in every plot.

    virtual void contextMenuEvent(QContextMenuEvent * event);
    virtual void resizeEvent(QResizeEvent * event);
    virtual void stateChanged(int state) {}; /*!< implement this function to get informed about changes in the state*/

    ito::RetVal startOrStopDrawGeometricShape(bool start);

    void setState(int state);
    int state() const { return m_state; }

    void setInverseColors(const QColor &color0, const QColor &color1);
    QColor inverseColor0() const { return m_inverseColor0; }
    QColor inverseColor1() const { return m_inverseColor1; }

    virtual void home() = 0;
    
    void keyPressEvent(QKeyEvent * event);
    void mousePressEvent(QMouseEvent * event);
    void mouseMoveEvent(QMouseEvent * event);
    void mouseReleaseEvent(QMouseEvent * event);
    bool event(QEvent * event);


    QAction *m_pActSave;        /*!< action to save the plot */
    QAction *m_pActPrint;        /*!< action to print the dialog with a print-preview dialog */
    QAction *m_pActHome;          /*!< action for homing */
    QAction *m_pActPan;         /*!< action for panner */
    QAction *m_pActZoom;        /*!< action for zoomer */
    QAction *m_pActAspectRatio; /*!< action to keep aspect ratio */
    QAction *m_pActSendCurrentToWorkspace; /*!< action to send current view to workspace */
    QAction *m_pActCopyClipboard; /*!< action to copy plot to clipboard */

    QMenu   *m_pMenuShapeType;
    QAction *m_pActShapeType;

    QAction *m_pActClearShapes;

    QAction *m_pActProperties;
    QAction *m_pActCamParameters;

    QList<QToolBar*> m_toolbars;
    QList<QMenu*> m_menus;
    QMenu *m_pContextMenu;
    bool m_styledBackground;
    bool m_firstTimeVisible; /*!< true if this plot becomes visible for the first time */
    

private:
    void createBaseActions();
    bool m_showContextMenu;
    void closePolygon(bool aborted);

    ItomPlotZoomer *m_pZoomer;
    ItomPlotMagnifier *m_pMagnifier;
    ItomQwtPlotPanner *m_pPanner;
    UserInteractionPlotPicker *m_pMultiPointPicker;

    //geometric shapes
    QMap<int, DrawItem *> m_pShapes; /*!< map table with all geometric shapes (besides marker sets) currently displayed (lookup is sorted by the integer index) */
    DrawItem *m_selectedShape;
    char m_selectedShapeHitType; /*!< 0: edge only, 1..8 marker number, 9 rotation marker */
    QPointF m_startMouseScale; /*!< position of mouse click on mousePressEvent based on scale coordinate system */
    QPoint m_startMousePx; /*!< position of mouse click on mousePressEvent based on screen coordinate system */
    QPointF m_startMouseScaleDiff; /*!< difference from startMouseScale to marker1..marker8 (edge is marker1, too)*/
    int m_mouseDragReplotCounter; /*!< a replot during mouse movement is time consuming, therefore only replot it every 2nd or 3rd movement */

    QVector<int> m_currentShapeIndices; /*!< indices of all shapes that have already been drawn in the current collection, e.g. if 4 rectangles are requested, the four indices are stored until the last rectangle has been given.*/
    int m_numShapesToPick; /*!< number of shapes that should be picked in the current session. */
    bool m_isUserInteraction;
    ito::Shape::ShapeType m_currentShapeType;
    ItomQwtPlotEnums::ShapeTypes m_allowedShapeTypes;
    ItomQwtPlotEnums::ModificationModes m_shapeModificationModes;
    bool m_shapesLabelVisible;
    int m_geometricShapeOpacity;
    int m_geometricShapeOpacitySelected;

    //markers
    QMultiHash<QString, QPair<int, QwtPlotMarker*> > m_plotMarkers;
    bool m_markerLabelVisible;

    bool m_ignoreNextMouseEvent; //todo: what is this?
    bool m_shapeModifiedByMouseMove;
    QColor m_inverseColor0;
    QColor m_inverseColor1;

    bool m_keepAspectRatio;
    bool m_plottingEnabled;

    int m_buttonStyle; /*!< 0: dark buttons for bright theme, 1: bright buttons for dark theme */
    bool m_boxFrame;
	int m_mouseCatchTolerancePx; /*!< number of pixels used as tolerance for clicking a certain marker or object with the mouse (default: 8px).*/
    
    int m_state; /*!< current state (value of enum State or stateUser + X) */
    bool m_stateIsChanging;
    int m_currentPlane;

    QColor m_backgroundColor;       //!> plot background color
    QColor m_axisColor;         //!> color of axis
    QColor m_textColor;         //!> text color
    QColor m_canvasColor;       //!> canvas color

    ito::AbstractFigure::UnitLabelStyle m_unitLabelStyle;

    QPrinter *m_pPrinter;
    
    void attachAndSelectNewShape(const ito::Shape &shape);
    void linePickingFinished();
    void squarePickingFinished();
    void rectanglePickingFinished();
    void circlePickingFinished();
    void ellipsePickingFinished();
    void pointPickingFinished();
    void polygonSinglePointPickingFinished();

    // during an interactive shape session, this method collect all current shapes and returns its vector. 
    // additionally, the shape toolbox is updated and (if desired) the geometricShapeChanged signal
    // is emitted for the last added shape in the m_currentShapeIndices vector.
    QVector<ito::Shape> getAndUpdateAllCurrentShapes(bool emitUpdateForLastModifiedShape);
    
public slots:
    void clearAllGeometricShapes();

private slots:
    void multiPointActivated(bool on);

    void mnuActSave();
    void mnuActPrint();
    void mnuActHome();
    void mnuActPan(bool checked);
    void mnuActZoom(bool checked);
    void mnuActRatio(bool checked);
    void mnuGroupShapeTypes(QAction *action);
    void mnuShapeType(bool checked);
    void mnuCopyToClipboard();
    void mnuSendCurrentToWorkspace();

    void updateColors(void);

    void printPreviewRequested(QPrinter* printer);

signals:
    void statusBarClear();
    void statusBarMessage(const QString &message, int timeout = 0);
};

#endif //ITOMQWTPLOT_H