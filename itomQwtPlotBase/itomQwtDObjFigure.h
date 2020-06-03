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

#ifndef ITOMQWTDOBJFIGURE_H
#define ITOMQWTDOBJFIGURE_H

#include "itomQwtPlotBase.h"

#include "plot/AbstractDObjFigure.h"
#include "common/retVal.h"
#include "common/itomPlotHandle.h"
#include "DataObject/dataobj.h"
#include "itomQwtPlotEnums.h"
#include "common/shape.h"

#include "itomWidgets/plotInfoShapes.h"
#include "itomWidgets/plotInfoMarker.h"
#include "itomWidgets/plotInfoPicker.h"
#include "itomWidgets/plotInfoDObject.h"

#include <qsize.h>
#include <qstring.h>
#include <qpixmap.h>
#include <qsharedpointer.h>

#ifndef DECLAREMETADATAOBJECT
    Q_DECLARE_METATYPE(QSharedPointer<ito::DataObject>)
    #define DECLAREMETADATAOBJECT
#endif


#ifndef DECLAREMETAPLOTHANDLE
    Q_DECLARE_METATYPE(ito::ItomPlotHandle)
    #define DECLAREMETAPLOTHANDLE
#endif

class ItomQwtPlot;
class ItomQwtDObjFigurePrivate;


class ITOMQWTPLOTBASE_EXPORT ItomQwtDObjFigure : public ito::AbstractDObjFigure
{
    Q_OBJECT

#if QT_VERSION < 0x050500
    //for >= Qt 5.5.0 see Q_ENUM definition below
    Q_ENUMS(ButtonStyle)
#endif

    //DESIGNABLE (default: true): property is visible in QtDesigner property editor
    //USER (default: false): property is visible in property editor of plot

    Q_PROPERTY(bool enableBoxFrame READ getBoxFrame WRITE setBoxFrame DESIGNABLE true USER true)
    Q_PROPERTY(ButtonStyle buttonSet READ getButtonSet WRITE setButtonSet DESIGNABLE true USER true)
    Q_PROPERTY(QColor axisColor READ getAxisColor WRITE setAxisColor USER true)
    Q_PROPERTY(QColor textColor READ getTextColor WRITE setTextColor USER true)
    Q_PROPERTY(QColor canvasColor READ getCanvasColor WRITE setCanvasColor USER true)
    Q_PROPERTY(QColor backgroundColor READ getBackgroundColor WRITE setBackgroundColor USER true)

    Q_PROPERTY(bool keepAspectRatio READ getKeepAspectRatio WRITE setKeepAspectRatio USER true)

    // Properties related with geometric elements
    Q_PROPERTY(QVector<ito::Shape> geometricShapes READ getGeometricShapes WRITE setGeometricShapes DESIGNABLE false USER false)
    Q_PROPERTY(int geometricShapesCount READ getGeometricShapesCount DESIGNABLE false USER false)
    Q_PROPERTY(int selectedGeometricShape READ getSelectedGeometricShape WRITE setSelectedGeometricShape DESIGNABLE false USER true)
    Q_PROPERTY(bool geometricShapesDrawingEnabled READ getEnabledPlotting WRITE setEnabledPlotting USER true)
    Q_PROPERTY(ItomQwtPlotEnums::ModificationModes geometryModificationModes READ getModificationModes WRITE setModificationModes DESIGNABLE true USER true);
    Q_PROPERTY(ItomQwtPlotEnums::ShapeTypes allowedGeometricShapes READ getAllowedGeometricShapes WRITE setAllowedGeometricShapes DESIGNABLE true USER true);
    Q_PROPERTY(int geometricShapesFillOpacity READ getGeometricShapesFillOpacity WRITE setGeometricShapesFillOpacity DESIGNABLE true USER true);
    Q_PROPERTY(int geometricShapesFillOpacitySelected READ getGeometricShapesFillOpacitySelected WRITE setGeometricShapesFillOpacitySelected DESIGNABLE true USER true);
    
    Q_PROPERTY(bool geometricShapesLabelsVisible READ getShapesLabelsVisible WRITE setShapesLabelsVisible DESIGNABLE true USER true)

    Q_PROPERTY(ito::AbstractFigure::UnitLabelStyle unitLabelStyle READ getUnitLabelStyle WRITE setUnitLabelStyle USER true);
    
    Q_PROPERTY(bool markerLabelsVisible READ getMarkerLabelsVisible WRITE setMarkerLabelsVisible DESIGNABLE true USER true)
	Q_PROPERTY(ItomQwtPlotEnums::ComplexType complexStyle READ getComplexStyle WRITE setComplexStyle DESIGNABLE true USER true);
    
    Q_CLASSINFO("prop://enableBoxFrame", "If true, a 1px solid border is drawn as a boxed rectangle around the canvas, else no margin is visible on the upper and right side.")
    Q_CLASSINFO("prop://buttonSet", "Get/set the button set used (normal or light color for dark themes).")
    Q_CLASSINFO("prop://axisColor", "Get/set the color of the axis.")
    Q_CLASSINFO("prop://textColor", "Get/set the color of text and tick-numbers.")
    Q_CLASSINFO("prop://canvasColor", "Get/set the color of the canvas.")
    Q_CLASSINFO("prop://backgroundColor", "Get/set the background color.")

    Q_CLASSINFO("prop://keepAspectRatio", "Enable or disable a fixed 1:1 aspect ratio between x and y axis.")
    Q_CLASSINFO("prop://geometricShapes", "Get or set the geometric shapes on the canvas, they are set as a sequence of itom.shape for each shape.")
    Q_CLASSINFO("prop://geometricShapesCount", "Number of currently existing geometric shapes.")
    Q_CLASSINFO("prop://selectedGeometricShape", "Get or set the currently highlighted geometric shape. After manipulation the last element stays selected.")
    Q_CLASSINFO("prop://geometricShapesDrawingEnabled", "Enable and disable internal plotting functions and GUI-elements for geometric elements.")
    Q_CLASSINFO("prop://geometryModificationModes", "Bitmask to globally change how geometric shapes can be modified. The possible modes of a shape are both restricted by the shape's flags and the allowed modes of the plot (move: 0x01, rotate: 0x02, resize: 0x04)")
    Q_CLASSINFO("prop://allowedGeometricShapes", "Combination of values of enumeration ShapeType to decide which types of geometric shapes are allowed (default: all shape types are allowed)")
    Q_CLASSINFO("prop://geometricShapesLabelsVisible", "Toggle visibility of shape labels, the label is the name of the shape.")
    Q_CLASSINFO("prop://geometricShapesFillOpacity", "Opacity for geometric shapes with an area > 0. This value ranges from 0 (not filled) to 255 (opaque).")
    Q_CLASSINFO("prop://geometricShapesFillOpacitySelected", "Opacity for the selected geometric shapes with an area > 0. This value ranges from 0 (not filled) to 255 (opaque).")

    Q_CLASSINFO("prop://unitLabelStyle", "style of the axes label (slash: 'name / unit', keyword-in: 'name in unit', square brackets: 'name [unit]'")
    Q_CLASSINFO("prop://markerLabelsVisible", "Toggle visibility of marker labels, the label is the set name of the marker.")
	Q_CLASSINFO("prop://complexStyle", "Defines whether the real, imaginary, phase or absolute of a complex number is shown. Possible options are CmplxAbs(0), CmplxImag (1), CmplxReal (2) and CmplxArg (3).")

    Q_CLASSINFO("slot://copyToClipBoard", "copies the entire plot to the clipboard as bitmap data (uses the default export resolution).")

    Q_CLASSINFO("slot://savePlot", "saves the plot as image, pdf or svg file (the supported file formats are listed in the save dialog of the plot)\n"
    "\n"
    "Parameters\n"
    "------------\n"
    "filename : {str}\n"
    "    absolute or relative filename whose suffix defines the file format\n"
    "xsize : {float}\n"
    "    x-size of the canvas in mm. If 0.0 [default], the size of the canvas is determined by the current size of the figure\n"
    "ysize : {float}\n"
    "    y-size of the canvas in mm. If 0.0 [default], the size of the canvas is determined by the current size of the figure\n"
    "resolution : {int}\n"
    "    resolution of image components in the plot in dpi (default: 300dpi)")
    
    Q_CLASSINFO("slot://renderToPixMap", "returns a QPixmap with the content of the plot\n"
    "\n"
    "Parameters\n"
    "------------\n"
    "xsize : {int}\n"
    "    width of the pixmap\n"
    "ysize : {int}\n"
    "    height of the pixmap\n"
    "resolution : {int}\n"
    "    resolution of the pixmap in dpi")

    Q_CLASSINFO("slot://userInteractionStart", "starts or aborts the process to let the user add a certain number of geometric shapes to the canvas.\n"
    "\n"
    "Parameters\n"
    "-----------\n"
    "type : {int}\n"
    "    type of the geometric shape the user should add (e.g. shape.Line, shape.Point, shape.Rectangle, shape.Square...\n"
    "start : {bool}\n"
    "    True if the interaction should be started, False if a running interaction process should be aborted\n"
    "maxNrOfPoints : {int}\n"
    "    number of shapes that should be added, the user can quit earlier by pressing Esc (optional, default: -1 -> infinite number of shapes)")

    Q_CLASSINFO("slot://clearGeometricShapes", "removes all geometric shapes from the canvas.")

    Q_CLASSINFO("slot://deleteGeometricShape", "deletes the geometric shape with the given index.\n"
    "\n"
    "Parameters\n"
    "------------\n"
    "idx : {int}\n"
    "    idx is the index of the shape to be removed. This is the index of the shape instance itself and must not always correspond to the index-position of the shape within the tuple of all shapes")

    Q_CLASSINFO("slot://setGeometricShapes", "This slot is the same than assigning a sequence of shape to the property 'geometricShapes'. It replaces all existing shapes by the new set of shapes.\n"
    "\n"
    "Parameters\n"
    "------------\n"
    "geometricShapes : {seq. of shapes}\n"
    "    Sequence (e.g tuple or list) of shapes that replace all existing shapes by this new set.")

    Q_CLASSINFO("slot://addGeometricShape", "Add a new geometric shape to the canvas if no shape with the same index already exists. \n"
    "\n"
    "If the index of the new shape is -1 (default), the next free auto-incremented index will be set for this shape. (C++ only: this new index ist\n"
    "stored in the optional 'newIndex' parameter).\n"
    "\n"
    "Parameters\n"
    "------------\n"
    "geometricShape : {shape}\n"
    "    new geometric shape\n"
    "\n"
    "Raises\n"
    "------------\n"
    "RuntimeError\n"
    "    if the index of the shape is != -1 and does already exist")

    Q_CLASSINFO("slot://updateGeometricShape", "Updates an existing geometric shape by the new shape if the index of the shape already exists, else add the new shape to the canvas (similar to 'addGeometricShape'. \n"
    "\n"
    "If the index of the new shape is -1 (default), the next free auto-incremented index will be set for this shape. (C++ only: this new index ist\n"
    "stored in the optional 'newIndex' parameter).\n"
    "\n"
    "Parameters\n"
    "------------\n"
    "geometricShape : {shape}\n"
    "    new geometric shape")
    
    Q_CLASSINFO("slot://setGeometricShapeLabel", "Set the label of geometric shape with the index idx.\n"
    "\n"
    "Parameters\n"
    "------------\n"
    "idx : {int}\n"
    "    index of the shape\n"
    "label : {str}\n"
    "    new label of the shape")

    Q_CLASSINFO("slot://setGeometricShapeLabelVisible", "Set the visibility of the label of a geometric shape with the given index.\n"
    "\n"
    "Parameters\n"
    "------------\n"
    "idx : {int}\n"
    "    index of the shape\n"
    "visible : {bool}\n"
    "    True if the label should be displayed close to the shape, else False")
    
    Q_CLASSINFO("slot://plotMarkers", "Draws sub-pixel wise markers to the canvas of the plot\n"
    "\n"
    "Parameters\n"
    "------------\n"
    "coordinates : {dataObject}\n"
    "    2xN data object with the 2D coordinates of the markers (first row: X, second row: Y coordinates in axis coordinates of the plot)\n"
    "style : {str}\n"
    "    Style string for the set of markers (e.g. 'r+20' for red crosses with a size of 20px)\n"
    "id : {str}\n"
    "    Name of the set of added markers (optional, default='')\n"
    "plane : {int}\n"
    "    If the dataObject has more than 2 dimensions, it is possible to add the markers to a specific plane only (optional, default=-1 -> all planes)")
    
    Q_CLASSINFO("slot://deleteMarkers", "Delete all sets of markers with the given id or all markers if no or an empty id is passed.\n"
    "\n"
    "Parameters\n"
    "------------\n"
    "id : {str} \n"
    "    name of the marker set that should be removed (optional)")
    
    Q_CLASSINFO("slot://replot", "Force a replot which is for instance necessary if values of the displayed data object changed and you want to update the plot, too.")

    Q_CLASSINFO("signal://geometricShapeStartUserInput", "This signal is emitted whenever the plot enters a mode where the user can add a new geometric shape using the mouse\n"
    "\n"
    "Parameters\n"
    "-----------\n"
    "type : {int}\n"
    "    Type of the shape that could be added by the user, this is one of the constants shape.Circle, shape.Ellipse, shape.Line...\n"
    "userInteractionReason : {bool}\n"
    "    True if the process to add a new shape has been initialized by a script-base call, False if it has been started by a button in the toolbar or menu of the plot")

    Q_CLASSINFO("signal://userInteractionDone", "This signal is emitted if the user finished adding the requested number of shapes or aborted the process by pressing the Esc key\n"
    "\n"
    "This signal is only emitted if the user interaction has been started by the slot *userInteractionStart* or by plotItem.drawAndPickElements.\n"
    "\n"
    "Parameters\n"
    "-------------\n"
    "type : {int}\n"
    "    type of the shapes that have been recently added (e.g. shape.Line, shape.Point, shape.Rectangle, ...)\n"
    "aborted : {bool}\n"
    "    True if the user aborted the process by pressing the Esc key before having added the total number of requested shapes"
    "shapes : {list of shape}\n"
    "    list of shapes that have been added.")

    Q_CLASSINFO("signal://geometricShapeAdded", "This signal is emitted whenever a geometric shape has been added\n"
    "\n"
    "Parameters\n"
    "-------------\n"
    "idx : {int}\n"
    "    index of the new shape (this is the index of the second parameter 'shape')\n"
    "shape : {shape}\n"
    "    new shape")

    Q_CLASSINFO("signal://geometricShapeChanged", "This signal is emitted whenever a geometric shape has been changed (e.g. its position or form has been changed)\n"
    "\n"
    "Parameters\n"
    "-------------\n"
    "idx : {int}\n"
    "    index of the changed shape (this is the index of the second parameter 'shape')\n"
    "shape : {shape}\n"
    "    shape that has been changed")
    
    Q_CLASSINFO("signal://geometricShapeDeleted", "This signal is emitted whenever a geometric shape has been deleted\n"
    "\n"
    "Parameters\n"
    "-------------\n"
    "idx : {int}\n"
    "    index of the deleted shape")
    
    Q_CLASSINFO("signal://geometricShapesDeleted", "This signal is emitted when the last geometric shape has been deleted or removed.")
    
    Q_CLASSINFO("signal://geometricShapeFinished", "This signal is emitted whenever one or multiple geometric shapes have been added, removed or modified\n"
    "\n"
    "Parameters\n"
    "-------------\n"
    "shapes : {tuple of shape}\n"
    "    A tuple containing all shapes that have been modified\n"
    "aborted : {bool}\n"
    "    True if the modification process has been aborted, else False")
    
    Q_CLASSINFO("signal://geometricShapeCurrentChanged", "This signal is emitted whenever the currently selected geometric has been changed\n"
    "\n"
    "Parameters\n"
    "-------------\n"
    "currentShape : {shape}\n"
    "    new current shape or an invalid shape if the current shape has been deleted and no other shape is selected now")

    DESIGNER_PLUGIN_ITOM_API

public:
    enum ButtonStyle
    {
        StyleBright = 0,
        StyleDark = 1
    };

#if QT_VERSION >= 0x050500
    //Q_ENUM exposes a meta object to the enumeration types, such that the key names for the enumeration
    //values are always accessible.
    Q_ENUM(ButtonStyle)
#endif

    explicit ItomQwtDObjFigure(QWidget *parent = NULL);
    explicit ItomQwtDObjFigure(const QString &itomSettingsFile, AbstractFigure::WindowMode windowMode, QWidget *parent = NULL);
    virtual ~ItomQwtDObjFigure();

    //!> set new button set
    void setButtonSet(const ButtonStyle buttonSet);

    //!> get current button set
    ButtonStyle getButtonSet(void) const;

    void setBoxFrame(const bool boxFrame);
    bool getBoxFrame(void) const;

    int getGeometricShapesCount() const;

    bool getKeepAspectRatio(void) const;
    void setKeepAspectRatio(const bool &keepAspectEnable);

    QVector< ito::Shape > getGeometricShapes();
    //setter is a public slot

    bool getEnabledPlotting(void) const;
    void setEnabledPlotting(const bool &enabled);

    int getSelectedGeometricShape(void) const;
    void setSelectedGeometricShape(const int idx);

    bool getMarkerLabelsVisible(void) const;
    void setMarkerLabelsVisible(const bool &visible);

    bool getShapesLabelsVisible(void) const;
    void setShapesLabelsVisible(const bool &visible);

    int getGeometricShapesFillOpacity() const;
    void setGeometricShapesFillOpacity(const int &opacity);

    int getGeometricShapesFillOpacitySelected() const;
    void setGeometricShapesFillOpacitySelected(const int &opacity);

    void setContextMenuEnabled(bool show); /*!< overloaded from AbstractFigure */
    bool getContextMenuEnabled() const;    /*!< overloaded from AbstractFigure */

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

    /** set canvas color
    *   @param [in] newVal  new text color
    */
    void setCanvasColor(const QColor newVal);

    //!> return canvas color
    QColor getCanvasColor(void) const;

    ito::AbstractFigure::UnitLabelStyle getUnitLabelStyle() const;
    virtual void setUnitLabelStyle(const ito::AbstractFigure::UnitLabelStyle &style) = 0;

    ItomQwtPlotEnums::ModificationModes getModificationModes() const;
    void setModificationModes(const ItomQwtPlotEnums::ModificationModes modes);
    
    ItomQwtPlotEnums::ShapeTypes getAllowedGeometricShapes() const;
    void setAllowedGeometricShapes(const ItomQwtPlotEnums::ShapeTypes &allowedTypes);

	virtual void setComplexStyle(const ItomQwtPlotEnums::ComplexType &type) = 0;
	virtual ItomQwtPlotEnums::ComplexType getComplexStyle() const = 0;

    QDockWidget *markerDockWidget() const;
    QDockWidget *pickerDockWidget() const;
    QDockWidget *shapesDockWidget() const;
    QDockWidget *dObjectDockWidget() const;

    friend ItomQwtPlot;

    
public Q_SLOTS:
    ito::RetVal copyToClipBoard();
    ito::RetVal savePlot(const QString &filename, float xsize = 0, float ysize = 0, int resolution = 300);
    QPixmap renderToPixMap(const int xsize, const int ysize, const int resolution);

    void userInteractionStart(int type, bool start, int maxNrOfPoints = -1);
    ito::RetVal clearGeometricShapes(void);
    ito::RetVal deleteGeometricShape(int idx);
    ito::RetVal setGeometricShapes(QVector<ito::Shape> geometricShapes);
    ito::RetVal addGeometricShape(const ito::Shape &geometricShape, int *newIndex = NULL); //add the new shape (only if its index does not already exist)
    ito::RetVal updateGeometricShape(const ito::Shape &geometricShape, int *newIndex = NULL); //updates the shape with the same index or add the given shape if its index does not already exist.

    ito::RetVal setGeometricShapeLabel(int idx, QString label);
    ito::RetVal setGeometricShapeLabelVisible(int idx, bool visible);

    ito::RetVal plotMarkers(QSharedPointer<ito::DataObject> coordinates, QString style, QString id = QString::Null(), int plane = -1);
    ito::RetVal deleteMarkers(QString id = "");

    void replot();
    

protected:
    inline PlotInfoMarker* markerWidget(void) const { return m_pMarkerInfo; }
    inline PlotInfoPicker* pickerWidget(void) const { return m_pPickerInfo; }
    inline PlotInfoShapes* shapesWidget(void) const { return m_pShapesInfo; }
    inline PlotInfoDObject* dObjectWidget(void) const { return m_pObjectInfo; }

    void addToolbarsAndMenus();
    
    ItomQwtPlot *m_pBaseContent;

private:
	void construct();

    PlotInfoMarker  *m_pMarkerInfo;
	PlotInfoPicker  *m_pPickerInfo;
	PlotInfoShapes  *m_pShapesInfo;
	PlotInfoDObject *m_pObjectInfo;
    
    //avoid to add private members but put them in the ItomQwtDObjFigurePrivate container
    //since this file is part of the itom SDK and can be included in other plugin's source code.
    //The container is defined in the cpp file only, therefore members can be changed there, without
    //breaking the binary compatibility.
    ItomQwtDObjFigurePrivate *d;

signals :
    void geometricShapeStartUserInput(int type, bool userInteractionReason); //userInteractionReason is true, if creating new shapes has been started by slot 'userInteractionStart', else false (e.g. by button in toolbar)
    void userInteractionDone(int type, bool aborted, QVector<ito::Shape> shapes);
    void geometricShapeAdded(int idx, ito::Shape shape);
    void geometricShapeChanged(int idx, ito::Shape shape);
    void geometricShapeDeleted(int idx);
    void geometricShapesDeleted();
    void geometricShapeFinished(QVector<ito::Shape> shapes, bool aborted);
    void geometricShapeCurrentChanged(ito::Shape currentShape); /*invalid shape if nothing is selected*/
};

#endif //ITOMQWTDOBJFIGURE_H