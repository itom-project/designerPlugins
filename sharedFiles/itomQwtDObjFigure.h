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


class ITOMQWTDOBJFIGURE_EXPORT ItomQwtDObjFigure : public ito::AbstractDObjFigure
{
    Q_OBJECT
    Q_ENUMS(ButtonStyle)

    Q_PROPERTY(ButtonStyle buttonSet READ getButtonSet WRITE setButtonSet DESIGNABLE true USER true)
    Q_PROPERTY(bool keepAspectRatio READ getKeepAspectRatio WRITE setKeepAspectRatio USER true)

    // Properties related with geometric elements
    Q_PROPERTY(QVector<ito::Shape> geometricShapes READ getGeometricShapes WRITE setGeometricShapes DESIGNABLE false USER false)
    Q_PROPERTY(int geometricShapesCount READ getGeometricShapesCount DESIGNABLE false USER false)
    Q_PROPERTY(bool enablePlotting READ getEnabledPlotting WRITE setEnabledPlotting USER true)
    Q_PROPERTY(int selectedGeometricShape READ getSelectedGeometricShape WRITE setSelectedGeometricShape DESIGNABLE false USER true)
    
    Q_PROPERTY(bool markerLabelsVisible READ getMarkerLabelsVisible WRITE setMarkerLabelsVisible DESIGNABLE true USER true)
    Q_PROPERTY(bool geometricShapesLabelsVisible READ getShapesLabelsVisible WRITE setShapesLabelsVisible DESIGNABLE true USER true)

    Q_PROPERTY(ito::AbstractFigure::UnitLabelStyle unitLabelStyle READ getUnitLabelStyle WRITE setUnitLabelStyle USER true);
    Q_PROPERTY(ItomQwtPlotEnums::ModificationModes geometryModificationModes READ getModificationModes WRITE setModificationModes DESIGNABLE true USER true);

    Q_CLASSINFO("prop://buttonSet", "Set the button set used (normal or light color for dark themes).")
    Q_CLASSINFO("prop://keepAspectRatio", "Enable and disable a fixed 1:1 aspect ratio between x and y axis.")
    Q_CLASSINFO("prop://geometricShapes", "Geometric shapes defined by a vector of itom.shape for each element.")
    Q_CLASSINFO("prop://geometricShapesCount", "Number of currently existing geometric shapes.")
    Q_CLASSINFO("prop://geometricShapesLabelsVisible", "Toggle visibility of shape labels, the label is the name of the shape.")

    Q_CLASSINFO("prop://enablePlotting", "Enable and disable internal plotting functions and GUI-elements for geometric elements.")

    Q_CLASSINFO("prop://selectedGeometricShape", "Get or set the currently highlighted geometric shape. After manipulation the last element stays selected.")
    Q_CLASSINFO("prop://markerLabelsVisible", "Toggle visibility of marker labels, the label is the set name of the marker.")

    Q_CLASSINFO("prop://unitLabelStyle", "style of the axes label (slash: 'name / unit', keyword-in: 'name in unit', square brackets: 'name [unit]'")
    Q_CLASSINFO("prop://geometryModificationModes", "Bitmask to globally change how geometric shapes can be modified. The possible modes of a shape are both restricted by the shape's flags and the allowed modes of the plot (move: 0x01, rotate: 0x02, resize: 0x04)")

    Q_CLASSINFO("slot://copyToClipBoard", "")
    Q_CLASSINFO("slot://userInteractionStart", "")
    Q_CLASSINFO("slot://clearGeometricElements", "")
    Q_CLASSINFO("slot://setGeometricElementLabel", "Set the label of geometric element with the index id")
    Q_CLASSINFO("slot://setGeometricElementLabelVisible", "Set the visibility of the label of geometric element with the index id")
    Q_CLASSINFO("slot://plotMarkers", "")
    Q_CLASSINFO("slot://deleteMarkers", "Delete all sets of markers with given name or all markers if no or an empty name is passed.")

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
    void setButtonSet(const ButtonStyle newVal);

    //!> get current button set
    ButtonStyle getButtonSet(void) const;

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

    ito::AbstractFigure::UnitLabelStyle getUnitLabelStyle() const;
    virtual void setUnitLabelStyle(const ito::AbstractFigure::UnitLabelStyle &style) = 0;

    ItomQwtPlotEnums::ModificationModes getModificationModes() const;
    void setModificationModes(const ItomQwtPlotEnums::ModificationModes modes);

    friend ItomQwtPlot;

    
public Q_SLOTS:
    ito::RetVal copyToClipBoard();
    QPixmap renderToPixMap(const int xsize, const int ysize, const int resolution);

    void userInteractionStart(int type, bool start, int maxNrOfPoints = -1);
    ito::RetVal clearGeometricShapes(void);
    ito::RetVal deleteGeometricShape(int id);
    ito::RetVal setGeometricShapes(QVector< ito::Shape > geometricShapes);

    ito::RetVal setGeometricShapeLabel(int id, QString label);
    ito::RetVal setGeometricShapeLabelVisible(int id, bool setVisible);

    ito::RetVal plotMarkers(QSharedPointer< ito::DataObject > coords, QString style, QString id = QString::Null(), int plane = -1);
    ito::RetVal deleteMarkers(QString id = "");
    

protected:

	inline QObject* MarkerWidget(void) const { return m_pMarkerInfo; }
	inline QObject* PickerWidget(void) const { return m_pPickerInfo; }
	inline QObject* ShapesWidget(void) const { return m_pShapesInfo; }
	inline QObject* DObjectWidget(void) const { return m_pObjectInfo; }

    void addToolbarsAndMenus();
    
    ItomQwtPlot *m_pBaseContent;

private:
    
	QDockWidget *m_pMarkerDock;
	QDockWidget *m_pPickerDock;
	QDockWidget *m_pShapesDock;
	QDockWidget *m_pObjectInfoDock;

	PlotInfoMarker  *m_pMarkerInfo;
	PlotInfoPicker  *m_pPickerInfo;
	PlotInfoShapes  *m_pShapesInfo;
	PlotInfoDObject *m_pObjectInfo;

signals :
    void userInteractionDone(int type, bool aborted, QVector<ito::Shape> shapes);
    void geometricShapeChanged(int idx, ito::Shape shape);
    void geometricShapeDeleted(int idx);
    void geometricShapesDeleted();
    void geometricShapeFinished(int type, bool aborted);
    
};

#endif //ITOMQWTDOBJFIGURE_H