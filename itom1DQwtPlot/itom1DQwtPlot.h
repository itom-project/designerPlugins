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

#ifndef ITOM1DPLOT_H
#define ITOM1DPLOT_H

#if defined(ITOMSHAREDDESIGNER)
    #define ITOM1DPLOT_EXPORT Q_DECL_EXPORT
#else
    #define ITOM1DPLOT_EXPORT Q_DECL_IMPORT
#endif

#include "itomQwtDObjFigure.h"
#include "itom1DQwtPlotEnums.h"

#include <qsharedpointer.h>
#include <qvector.h>
#include <qpoint.h>
#include <qstring.h>
#include <qfont.h>


class Itom1DQwtPlotPrivate;
class Plot1DWidget;

class ITOM1DPLOT_EXPORT Itom1DQwtPlot : public ItomQwtDObjFigure
{
    Q_OBJECT

    //DESIGNABLE (default: true): property is visible in QtDesigner property editor
    //USER (default: false): property is visible in property editor of plot

    Q_PROPERTY(QVector<QPointF> bounds READ getBounds WRITE setBounds RESET resetBounds DESIGNABLE false)

    Q_PROPERTY(QString title READ getTitle WRITE setTitle RESET resetTitle USER true)
    Q_PROPERTY(QString axisLabel READ getAxisLabel WRITE setAxisLabel RESET resetAxisLabel USER true)
    Q_PROPERTY(QString valueLabel READ getValueLabel WRITE setValueLabel RESET resetValueLabel USER true)
    Q_PROPERTY(QFont titleFont READ getTitleFont WRITE setTitleFont USER true)
    Q_PROPERTY(QFont axisFont READ getAxisFont WRITE setAxisFont USER true)
    Q_PROPERTY(QFont labelFont READ getLabelFont WRITE setLabelFont USER true)
    Q_PROPERTY(QFont legendFont READ getLegendFont WRITE setLegendFont USER true)
    Q_PROPERTY(double axisLabelRotation READ getAxisLabelRotation WRITE setAxisLabelRotation USER true)
    Q_PROPERTY(Qt::Alignment axisLabelAlignment READ getAxisLabelAlignment WRITE setAxisLabelAlignment USER true)
    
    Q_PROPERTY(LegendPos legendPosition READ getLegendPosition WRITE setLegendPosition USER true);
    Q_PROPERTY(QStringList legendTitles READ getLegendTitles WRITE setLegendTitles USER true);

    Q_PROPERTY(qreal lineWidth READ getLineWidth WRITE setLineWidth USER true)
    Q_PROPERTY(Qt::PenStyle lineStyle READ getLineStyle WRITE setLineStyle USER true)
    Q_PROPERTY(ItomQwtPlotEnums::CurveStyle curveStyle READ getCurveStyle WRITE setCurveStyle USER true);
    Q_PROPERTY(ItomQwtPlotEnums::FillCurveStyle fillCurve READ getCurveFilled WRITE setCurveFilled USER true)
    Q_PROPERTY(QColor curveFillColor READ getCurveFillColor WRITE setCurveFillColor RESET resetCurveFillColor USER true)
    Q_PROPERTY(int curveFillAlpha READ getCurveFillAlpha WRITE setCurveFillAlpha USER true)
    Q_PROPERTY(qreal baseLine READ getBaseLine WRITE setBaseLine USER true)

    Q_PROPERTY(Symbol lineSymbol READ getLineSymbol WRITE setLineSymbol USER true);
    Q_PROPERTY(int lineSymbolSize READ getLineSymbolSize WRITE setLineSymbolSize USER true);

    Q_PROPERTY(GridStyle grid READ getGrid WRITE setGrid USER true)
    
    Q_PROPERTY(ItomQwtPlotEnums::MultiLineMode columnInterpretation READ getRowPresentation WRITE setRowPresentation RESET resetRowPresentation DESIGNABLE true USER true)
    
    Q_PROPERTY(ItomQwtPlotEnums::PlotPickerType pickerType READ getPickerType WRITE setPickerType USER true);
    Q_PROPERTY(int pickerLimit READ getPickerLimit WRITE setPickerLimit RESET resetPickerLimit DESIGNABLE true USER true)
    Q_PROPERTY(int pickerCount READ getPickerCount DESIGNABLE false USER true)
    Q_PROPERTY(int currentPickerIndex READ getCurrentPickerIndex WRITE setCurrentPickerIndex DESIGNABLE false USER true)
    Q_PROPERTY(QSharedPointer< ito::DataObject > picker READ getPicker DESIGNABLE false)
    
    Q_PROPERTY(bool pickerLabelVisible READ getPickerLabelVisible WRITE setPickerLabelVisible USER true);
    Q_PROPERTY(Qt::Orientation pickerLabelOrientation READ getPickerLabelOrientation WRITE setPickerLabelOrientation USER true);
    Q_PROPERTY(Qt::Alignment pickerLabelAlignment READ getPickerLabelAlignment WRITE setPickerLabelAlignment USER true);

    Q_PROPERTY(ItomQwtPlotEnums::ScaleEngine valueScale READ getValueScale WRITE setValueScale USER true);
    Q_PROPERTY(ItomQwtPlotEnums::ScaleEngine axisScale READ getAxisScale WRITE setAxisScale USER true);

    Q_PROPERTY(QSharedPointer<ito::DataObject> xData READ getXData WRITE setXData DESIGNABLE false USER false);

    Q_PROPERTY(bool antiAliased READ getAntiAliased WRITE setAntiAliased DESIGNABLE true USER true);
	Q_PROPERTY(int legendLabelWidth READ getLegendLabelWidth WRITE setLegendLabelWidth USER true);
    
    Q_CLASSINFO("prop://title", "Title of the plot or '<auto>' if the title of the data object should be used.")
    Q_CLASSINFO("prop://axisLabel", "Label of the direction (x/y) axis or '<auto>' if the descriptions from the data object should be used.")
    Q_CLASSINFO("prop://valueLabel", "Label of the value axis (y-axis) or '<auto>' if the description should be used from data object.")
    Q_CLASSINFO("prop://titleFont", "Font for title.")
    Q_CLASSINFO("prop://labelFont", "Font for axes descriptions.")
    Q_CLASSINFO("prop://axisFont", "Font for axes tick values.")
    Q_CLASSINFO("prop://legendFont", "Font for legend entries")

    Q_CLASSINFO("prop://axisLabelRotation", "The rotation angle in degree of the labels on the bottom x-axis. If changed, the alignment should also be adapted.")
    Q_CLASSINFO("prop://axisLabelAlignment", "The label alignment for the x-axis. This value has to be adjusted if the rotation is changed.")

    Q_CLASSINFO("prop://grid", "Style of the grid ('GridNo', 'GridMajorXY', 'GridMajorX', 'GridMajorY', 'GridMinorXY', 'GridMinorX', 'GridMinorY').")

    Q_CLASSINFO("prop://lineWidth", "width of all curves in pixel.")
    Q_CLASSINFO("prop://lineStyle", "style of all lines.")
    Q_CLASSINFO("prop://curveStyle", "set the style of the qwt-plot according to curve styles.")

    Q_CLASSINFO("prop://fillCurve", "fill curve below / above or according to baseline.")
    Q_CLASSINFO("prop://curveFillColor", "the fill color for the curve, invalid color leads to line color selection.")
    Q_CLASSINFO("prop://curveFillAlpha", "set the alpha value for the curve fill color seperatly.")

    Q_CLASSINFO("prop://lineSymbol", "Get / Set the current line symbol type")
    Q_CLASSINFO("prop://lineSymbolSize", "Get / Set the current line symbol size")

    Q_CLASSINFO("prop://baseLine", "If curveStyle is set to 'Sticks', 'SticksVertical' or 'SticksHorizontal', the baseline indicates the start point of each line either in vertical or horizontal direction. For all other curve types, the baseline is considered if fillCurve is set to 'FillBaseLine'.")

    Q_CLASSINFO("prop://columnInterpretation", "Define the interpretation of M x N objects as Auto, FirstRow, FirstCol, MultiRows, MultiCols.")

    Q_CLASSINFO("prop://pickerLimit", "Define the maximal number of picker for this plot.")
    Q_CLASSINFO("prop://pickerCount", "Number of picker within the plot.")
    Q_CLASSINFO("prop://picker", "Get picker defined by a Mx4 float32 data object. Each row represents one picker and contains the following information: [pixelIndex, physIndex, value, curveIndex]. PixelIndex and physIndex are equal if axisScale = 1 and axisOffset = 0 for the corresponding dataObject.")
    Q_CLASSINFO("prop://currentPickerIndex", "Get / set currently active picker.")

    Q_CLASSINFO("prop://legendPosition", "Position of the legend (Off, Left, Top, Right, Bottom)")
    Q_CLASSINFO("prop://legendTitles", "Seq. of strings with the legend titles for all curves. If no legends are given, the dataObject is checked for tags named 'legendTitle0', 'legendTitle1'... If these tags are not given, the default titles 'curve 0', 'curve 1'... are taken.")
	Q_CLASSINFO("prop://legendLabelWidth", "Defines the width of a legend label. This can be used to create a longer line in legend entries. The minimal size is 10")

    Q_CLASSINFO("prop://pickerLabelVisible", "Enable and disable the labels next to each picker.")
    Q_CLASSINFO("prop://pickerLabelOrientation", "Get / set the label orientation for the picker labels.")
    Q_CLASSINFO("prop://pickerLabelAlignment", "Get / set label alignment for the picker labels.")
    Q_CLASSINFO("prop://pickerType", "Get / set the current picker type ('DefaultMarker', 'RangeMarker', 'ValueRangeMarker', 'AxisRangeMarker')")

    Q_CLASSINFO("prop://valueScale", "linear or logarithmic scale (various bases) can be chosen for the vertical axis (y-axis). Please consider, that a logarithmic scale can only display values > 1e-100 while the lower limit for the double-logarithmic scale is 1+1e-100.")
    Q_CLASSINFO("prop://axisScale", "linear or logarithmic scale (various bases) can be chosen for the horizontal axis (x-axis). Please consider, that a logarithmic scale can only display values > 1e-100.")

    Q_CLASSINFO("prop://antiAliased", "True, if all curves should be plot with an anti-aliased render mode (slower) or False if not (faster).")
    Q_CLASSINFO("prop://xData", "DataObject representing the xData of the plot. Expect a two dimensional dataObject with a (n x m) or (1 x m) shape for an (n x m) source object with n < m. For n > m a shape of (n x 1) or (n x m) is exspected.")

    Q_CLASSINFO("slot://setPicker", "Set plot pickers to a specific curve either in physical (axis) or in pixel coordinates.\n"
    "\n"
    "The pixel coordinates are the pixels of the currently displayed dataObject. The coordinates are the axis positions only, \n"
    "the values are chosen from the curve values. Existing pickers are deleted at first.\n"
    "\n"
    "Parameters\n"
    "-------------\n"
    "coordinates : {seq. of float}\n"
    "    x-coordinates of each picker, the y-coordinate is automatically chosen from the shape of the curve. If the size of the sequence exceeds the 'pickerLimit', a RuntimeError is thrown."
    "curveIndex : {int}\n"
    "    index of the curve where the pickers should be attached to (optional, default: 0 - first curve)\n"
    "physicalCoordinates : {bool}\n"
    "    optional, if True (default), 'coordinates' are given in axis coordinates of the plot (hence, physical coordinates of the dataObject; False: 'coordinates' are given in pixel coordinates of the dataObject")

    Q_CLASSINFO("slot://appendPicker", "Append plot pickers to a specific curve either in physical (axis) or in pixel coordinates.\n"
    "\n"
    "The pixel coordinates are the pixels of the currently displayed dataObject. The coordinates are the axis positions only, \n"
    "the values are chosen from the curve values. Existing pickers are not removed before this operation.\n"
    "\n"
    "Parameters\n"
    "-------------\n"
    "coordinates : {seq. of float}\n"
    "    x-coordinates of each picker, the y-coordinate is automatically chosen from the shape of the curve. If the size of the sequence plus the number of existing pickers exceed the 'pickerLimit', a RuntimeError is thrown."
    "curveIndex : {int}\n"
    "    index of the curve where the pickers should be attached to (optional, default: 0 - first curve)\n"
    "physicalCoordinates : {bool}\n"
    "    optional, if True (default), 'coordinates' are given in axis coordinates of the plot (hence, physical coordinates of the dataObject; False: 'coordinates' are given in pixel coordinates of the dataObject")

    Q_CLASSINFO("slot://deletePicker", "Delete the i-th picker (id >= 0) or all pickers (id = -1)\n"
    "\n"
    "Parameters\n"
    "-------------\n"
    "id : {int}\n"
    "    zero-based index of the picker to be deleted, or -1 if all pickers should be deleted (default). This parameter is optional.")

    Q_CLASSINFO("slot://setCurveProperty", "Set a property of a specific curve\n"
    "\n"
    "Some curve properties can be changed globally for all curves using the global properties. However, it is also possible to\n"
    "set a property to different values for each curve.\n"
    "\n"
    "Parameters\n"
    "-------------\n"
    "index : {int}\n"
    "    zero-based index of the curve whose property should be changed.\n"
    "property : {str}\n"
    "    name of the property to be changed\n"
    "value : {various}\n"
    "    value of the property")

    Q_CLASSINFO("slot://getCurveProperty", "Get a property of a specific curve\n"
    "\n"
    "Get the value of a property of a specific curve (see slot 'setCurveProperty').\n"
    "\n"
    "Parameters\n"
    "-------------\n"
    "index : {int}\n"
    "    zero-based index of the curve whose property should be changed.\n"
    "property : {str}\n"
    "    name of the property to be changed\n"
    "\n"
    "Returns\n"
    "-------------\n"
    "value : {variant}\n"
    "    value of the requested property")
    
    Q_CLASSINFO("slot://getDisplayed", "returns the currently displayed dataObject.")

    Q_CLASSINFO("signal://pickerChanged", "This signal is emitted whenever the current picker changed its position\n"
    "\n"
    "Parameters\n"
    "-------------\n"
    "pickerIndex : {int}\n"
    "    index of the changed picker\n"
    "positionX : {double}\n"
    "    horizontal position of currently changed picker\n"
    "positionY : {double}\n"
    "    vertical position of the currently changed picker\n"
    "curveIndex : {int}\n"
    "    index of the curve the picker is attached to")

    public:
        Itom1DQwtPlot(QWidget *parent = 0);
        Itom1DQwtPlot(const QString &itomSettingsFile, AbstractFigure::WindowMode windowMode, QWidget *parent = 0);
        virtual ~Itom1DQwtPlot();

        enum LegendPos { Off = 0, Left = 1, Top = 2, Right = 3, Bottom = 4 };
        enum Symbol
        {
            NoSymbol = 0, Ellipse, Rect, Diamond, Triangle, DTriangle, UTriangle, LTriangle, RTriangle, Cross, XCross, HLine, VLine, Star1, Star2, Hexagon
        }; //this enum corresponds to the first entries of QwtSymbol::Style, however all values are shifted by 1, such that NoSymbol (0) corresponds to QwtSymbol::NoSymbol (-1) -> enum value -1 is invalid for Qt meta enumeration system.
        enum ColorHandling { AutoColor, Gray, RGB, RGBA, RGBGray };
        enum GridStyle { GridNo = 0, GridMajorXY = 1, GridMajorX = 2, GridMajorY = 3, GridMinorXY = 4, GridMinorX = 5, GridMinorY = 6 };

        //Q_ENUM exposes a meta object to the enumeration types, such that the key names for the enumeration
        //values are always accessible.
        Q_ENUM(LegendPos);
        Q_ENUM(Symbol);
        Q_ENUM(GridStyle);

        ito::RetVal applyUpdate();                              //!< propagates updated data through the subtree
        
        //properties
        QVector<QPointF> getBounds(void) const;
        void setBounds(QVector<QPointF> bounds);
        void resetBounds();

        void enableObjectGUIElements(const int mode);

        QString getTitle() const;
        void setTitle(const QString &title);
        void resetTitle();

        QString getAxisLabel() const;
        void setAxisLabel(const QString &label);
        void resetAxisLabel();

        QString getValueLabel() const;
        void setValueLabel(const QString &label);
        void resetValueLabel();

        ito::AutoInterval getXAxisInterval(void) const;
        void setXAxisInterval(ito::AutoInterval);

        ito::AutoInterval getYAxisInterval(void) const;
        void setYAxisInterval(ito::AutoInterval);

        ItomQwtPlotEnums::ScaleEngine getValueScale() const;
        void setValueScale(const ItomQwtPlotEnums::ScaleEngine &scale);

        ItomQwtPlotEnums::ScaleEngine getAxisScale() const;
        void setAxisScale(const ItomQwtPlotEnums::ScaleEngine &scale);

        QFont getTitleFont(void) const;
        void setTitleFont(const QFont &font);

        QFont getLabelFont(void) const;
        void setLabelFont(const QFont &font);

        QFont getAxisFont(void) const;
        void setAxisFont(const QFont &font);

        QFont getLegendFont() const;
        void setLegendFont(const QFont &font);

        double getAxisLabelRotation() const;
        void setAxisLabelRotation(const double &rotation);

        Qt::Alignment getAxisLabelAlignment() const;
        void setAxisLabelAlignment(const Qt::Alignment &alignment);

        GridStyle getGrid(void) const;
        void setGrid(const GridStyle &gridStyle);

        qreal getLineWidth(void) const;
        void setLineWidth(const qreal &width);

        Qt::PenStyle getLineStyle() const;
        void setLineStyle(const Qt::PenStyle &style);

        ItomQwtPlotEnums::CurveStyle getCurveStyle() const;
        void setCurveStyle(const ItomQwtPlotEnums::CurveStyle state);

        LegendPos getLegendPosition() const;
        void setLegendPosition(LegendPos legendPosition);

        QStringList getLegendTitles() const;

        bool getAntiAliased() const;
        void setAntiAliased(bool &antiAliased);

        ito::RetVal setSource(QSharedPointer<ito::DataObject> source);

        void setUnitLabelStyle(const ito::AbstractFigure::UnitLabelStyle &style);
    
        ItomQwtPlotEnums::MultiLineMode getRowPresentation(void) const;
        void setRowPresentation(const ItomQwtPlotEnums::MultiLineMode idx);
        void resetRowPresentation(); 

        int getRGBPresentation() const;
        void setRGBPresentation(const ItomQwtPlotEnums::ColorHandling idx);
        void resetRGBPresentation(); 

        int getPickerLimit() const;
        void setPickerLimit(const int idx);
        void resetPickerLimit(); 

        int getPickerCount() const;
        QSharedPointer< ito::DataObject > getPicker() const;

        QVector<int> getPickerPixel() const;
        QVector<float> getPickerPhys() const;

        void setPickerLabelVisible(const bool state);
        bool getPickerLabelVisible() const;

        void setPickerLabelOrientation(const Qt::Orientation val);
        Qt::Orientation getPickerLabelOrientation() const;

        void setPickerLabelAlignment(const Qt::Alignment val);
        Qt::Alignment getPickerLabelAlignment()const ;

        ItomQwtPlotEnums::PlotPickerType getPickerType() const;
        void setPickerType(const ItomQwtPlotEnums::PlotPickerType val);

        int getCurrentPickerIndex() const;
        void setCurrentPickerIndex(const int index);

        Symbol getLineSymbol() const;
        void setLineSymbol(const Symbol symbol);
        void resetLineSymbol();

        int getLineSymbolSize() const;
        void setLineSymbolSize(int size);
        void resetLineSymbolSize();

        void setBaseLine(const qreal val);
        qreal getBaseLine() const;

        QColor getCurveFillColor() const;
        void setCurveFillColor(const QColor val);
        void resetCurveFillColor();

        ItomQwtPlotEnums::FillCurveStyle getCurveFilled() const;
        void setCurveFilled(const ItomQwtPlotEnums::FillCurveStyle state);

        void setCurveFillAlpha(const int val);
        int getCurveFillAlpha() const;

		void setComplexStyle(const ItomQwtPlotEnums::ComplexType &type);
		ItomQwtPlotEnums::ComplexType getComplexStyle() const;

		QWidget* getWidgetCurveProperties();
        QAction* getCurvePropertiesToggleViewAction();

		int getLegendLabelWidth() const;
		void setLegendLabelWidth(const int& length);

        ito::RetVal setXData(QSharedPointer<ito::DataObject> data);
        QSharedPointer<ito::DataObject> getXData() const;


		
        friend Plot1DWidget;

    protected:
        ito::RetVal init(); // { return m_pContent->init(); }; //called when api-pointers are transmitted, directly after construction

    private:
        
        void constructor();

        Plot1DWidget *m_pContent;

        //avoid to add private members but put them in the Itom1DQwtPlotPrivate container
        //since this file is part of the itom SDK and can be included in other plugin's source code.
        //The container is defined in the cpp file only, therefore members can be changed there, without
        //breaking the binary compatibility.
        Itom1DQwtPlotPrivate *d;

    public slots:
        void setLegendTitles(const QStringList &legends);
        ito::RetVal setPicker(const QVector<double> &coordinates, int curveIndex = 0, bool physicalCoordinates = true);
        ito::RetVal appendPicker(const QVector<double> &coordinates, int curveIndex = 0, bool physicalCoordinates = true);
        ito::RetVal deletePicker(int id = -1);

        ito::RetVal setCurveProperty(int index, const QByteArray &property, const QVariant &value);
        QVariant getCurveProperty(int index, const QByteArray &property);

		void showCurveProperties();

        QSharedPointer<ito::DataObject> getDisplayed();

    private slots:
		void updatePropertiesDock();

    signals:
        void pickerChanged(int pickerIndex, double positionX, double positionY, int curveIndex);        
};
//----------------------------------------------------------------------------------------------------------------------------------

#endif // ITOMPLOT_H
