/* ********************************************************************
   itom measurement system
   URL: http://www.uni-stuttgart.de/ito
   Copyright (C) 2015, Institut fuer Technische Optik (ITO), 
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

#if defined(ITOMSHAREDDESIGNER)
    #define ITOMQWTDOBJFIGURE_EXPORT Q_DECL_EXPORT
#else
    #define ITOMQWTDOBJFIGURE_EXPORT Q_DECL_IMPORT
#endif

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


class ITOMQWTDOBJFIGURE_EXPORT ItomQwtDObjFigure : public ito::AbstractDObjFigure
{
    Q_OBJECT
    Q_ENUMS(ButtonStyle)

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
    
    Q_PROPERTY(bool geometricShapesLabelsVisible READ getShapesLabelsVisible WRITE setShapesLabelsVisible DESIGNABLE true USER true)

    Q_PROPERTY(ito::AbstractFigure::UnitLabelStyle unitLabelStyle READ getUnitLabelStyle WRITE setUnitLabelStyle USER true);
    
    Q_PROPERTY(bool markerLabelsVisible READ getMarkerLabelsVisible WRITE setMarkerLabelsVisible DESIGNABLE true USER true)
    
    Q_CLASSINFO("prop://enableBoxFrame", "If true, a 1px solid border is drawn as a boxed rectangle around the canvas, else no margin is visible on the upper and right side.")
    Q_CLASSINFO("prop://buttonSet", "Set the button set used (normal or light color for dark themes).")
    Q_CLASSINFO("prop://backgroundColor", "Set the background color.")
    Q_CLASSINFO("prop://axisColor", "Set the color of the axis.")
    Q_CLASSINFO("prop://textColor", "Set the color of text and tick-numbers.")
    Q_CLASSINFO("prop://canvasColor", "Set the color of the canvas.")

    Q_CLASSINFO("prop://keepAspectRatio", "Enable and disable a fixed 1:1 aspect ratio between x and y axis.")
    Q_CLASSINFO("prop://geometricShapes", "Geometric shapes defined by a vector of itom.shape for each element.")
    Q_CLASSINFO("prop://geometricShapesCount", "Number of currently existing geometric shapes.")
    Q_CLASSINFO("prop://geometricShapesLabelsVisible", "Toggle visibility of shape labels, the label is the name of the shape.")

    Q_CLASSINFO("prop://geometricShapesDrawingEnabled", "Enable and disable internal plotting functions and GUI-elements for geometric elements.")

    Q_CLASSINFO("prop://geometryModificationModes", "Bitmask to globally change how geometric shapes can be modified. The possible modes of a shape are both restricted by the shape's flags and the allowed modes of the plot (move: 0x01, rotate: 0x02, resize: 0x04)")
    Q_CLASSINFO("prop://allowedGeometricShapes", "Combination of values of enumeration ShapeType to decide which types of geometric shapes are allowed (default: all shape types are allowed)")


    Q_CLASSINFO("prop://selectedGeometricShape", "Get or set the currently highlighted geometric shape. After manipulation the last element stays selected.")
    Q_CLASSINFO("prop://markerLabelsVisible", "Toggle visibility of marker labels, the label is the set name of the marker.")

    Q_CLASSINFO("prop://unitLabelStyle", "style of the axes label (slash: 'name / unit', keyword-in: 'name in unit', square brackets: 'name [unit]'")
    
    Q_CLASSINFO("slot://copyToClipBoard", "")
    Q_CLASSINFO("slot://userInteractionStart", "")
    Q_CLASSINFO("slot://clearGeometricShapes", "removes all geometric shapes from the canvas.")
    Q_CLASSINFO("slot://deleteGeometricShape", "deletes the geometric shape with the given index. The index is thereby part of the shape object and must not always corresponds to the position in the vector of the geometricShapes property.")
    Q_CLASSINFO("slot://setGeometricShapes", "This slot is also called by the setter function of the property 'geometricShapes'. Pass a vector of geometric shapes that replace any existing shape and will be displayed on the canvas.")
    Q_CLASSINFO("slot://addGeometricShape", "Add another geometric shape if its index is not already available. If the index is -1 (default), the next free value >= 0 will be assigned as index to the new shape object.")
    Q_CLASSINFO("slot://updateGeometricShape", "Updates an existing geometric shape, if the index already exists or add the given shape as new shape to the list of existing shapes.")
    Q_CLASSINFO("slot://setGeometricElementLabel", "Set the label of geometric element with the index idx.")
    Q_CLASSINFO("slot://setGeometricElementLabelVisible", "Set the visibility of the label of geometric element with the index idx.")
    Q_CLASSINFO("slot://plotMarkers", "Put markers all all coordinates given by the first dataObject argument (2xN, first row: x-coordinates, second row: y-coordinates). The style of the markers is given as second argument, while an optional set name can be given as third argument. If the markers should only be displayed in one plane, give the plane index as last argument.")
    Q_CLASSINFO("slot://deleteMarkers", "Delete all sets of markers with given name or all markers if no or an empty name is passed.")
    Q_CLASSINFO("slot://replot", "Force a replot which is for instance necessary if values of the displayed data object changed and you want to update the plot, too.")

    Q_CLASSINFO("signal://plotItemsFinished", "Signal emitted when geometrical plotting was finished.")
    Q_CLASSINFO("signal://userInteractionDone", "")
    Q_CLASSINFO("signal://plotItemChanged", "")
    Q_CLASSINFO("signal://plotItemDeleted", "")
    Q_CLASSINFO("signal://plotItemsDeleted", "")

    DESIGNER_PLUGIN_ITOM_API

public:
    enum ButtonStyle
    {
        StyleBright = 0,
        StyleDark = 1
    };

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

    friend ItomQwtPlot;

    
public Q_SLOTS:
    ito::RetVal copyToClipBoard();
    QPixmap renderToPixMap(const int xsize, const int ysize, const int resolution);

    void userInteractionStart(int type, bool start, int maxNrOfPoints = -1);
    ito::RetVal clearGeometricShapes(void);
    ito::RetVal deleteGeometricShape(int idx);
    ito::RetVal setGeometricShapes(QVector< ito::Shape > geometricShapes);
    ito::RetVal addGeometricShape(const ito::Shape &geometricShape, int *newIndex = NULL); //add the new shape (only if its index does not already exist)
    ito::RetVal updateGeometricShape(const ito::Shape &geometricShape, int *newIndex = NULL); //updates the shape with the same index or add the given shape if its index does not already exist.

    ito::RetVal setGeometricShapeLabel(int idx, QString label);
    ito::RetVal setGeometricShapeLabelVisible(int idx, bool setVisible);

    ito::RetVal plotMarkers(QSharedPointer< ito::DataObject > coords, QString style, QString id = QString::Null(), int plane = -1);
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
    void userInteractionDone(int type, bool aborted, QVector<ito::Shape> shapes);
    void geometricShapeChanged(int idx, ito::Shape shape);
    void geometricShapeDeleted(int idx);
    void geometricShapesDeleted();
    void geometricShapeFinished(int type, bool aborted);
    
};

#endif //ITOMQWTDOBJFIGURE_H