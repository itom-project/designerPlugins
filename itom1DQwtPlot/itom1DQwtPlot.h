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

class Plot1DWidget;
class ItomPlotMarker;
struct InternalData;


class ITOM1DPLOT_EXPORT Itom1DQwtPlot : public ItomQwtDObjFigure
{
    Q_OBJECT
    Q_PROPERTY(QVector<QPointF> bounds READ getBounds WRITE setBounds DESIGNABLE false)

    Q_PROPERTY(QString title READ getTitle WRITE setTitle RESET resetTitle USER true)
    Q_PROPERTY(QString axisLabel READ getAxisLabel WRITE setAxisLabel RESET resetAxisLabel USER true)
    Q_PROPERTY(QString valueLabel READ getValueLabel WRITE setValueLabel RESET resetValueLabel USER true)
    Q_PROPERTY(QFont titleFont READ getTitleFont WRITE setTitleFont USER true)
    Q_PROPERTY(QFont axisFont READ getAxisFont WRITE setAxisFont USER true)
    Q_PROPERTY(QFont labelFont READ getLabelFont WRITE setLabelFont USER true)
    Q_PROPERTY(QColor axisColor READ getAxisColor WRITE setAxisColor USER true)
    Q_PROPERTY(QColor textColor READ getTextColor WRITE setTextColor USER true)

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

    Q_PROPERTY(bool grid READ getGrid WRITE setGrid USER true)
    Q_PROPERTY(QColor backgroundColor READ getBackgroundColor WRITE setBackgroundColor USER true)

    Q_PROPERTY(ItomQwtPlotEnums::MultiLineMode columnInterpretation READ getRowPresentation WRITE setRowPresentation RESET resetRowPresentation DESIGNABLE true USER true)
    
    Q_PROPERTY(ItomQwtPlotEnums::PlotPickerType pickerType READ getPickerType WRITE setPickerType USER true);
    Q_PROPERTY(int pickerLimit READ getPickerLimit WRITE setPickerLimit RESET resetPickerLimit DESIGNABLE true USER true)
    Q_PROPERTY(int pickerCount READ getPickerCount DESIGNABLE false USER true)
    Q_PROPERTY(QSharedPointer< ito::DataObject > picker READ getPicker DESIGNABLE false)
    
    Q_PROPERTY(bool pickerLabelVisible READ getPickerLabelVisible WRITE setPickerLabelVisible USER true);
    Q_PROPERTY(Qt::Orientation pickerLabelOrientation READ getPickerLabelOrientation WRITE setPickerLabelOrientation USER true);
    Q_PROPERTY(Qt::Alignment pickerLabelAlignment READ getPickerLabelAlignment WRITE setPickerLabelAlignment USER true);

    Q_PROPERTY(ItomQwtPlotEnums::ScaleEngine valueScale READ getValueScale WRITE setValueScale USER true);
    Q_PROPERTY(ItomQwtPlotEnums::ScaleEngine axisScale READ getAxisScale WRITE setAxisScale USER true);

    Q_ENUMS(LegendPos);
    Q_ENUMS(Symbol);
    
    Q_CLASSINFO("prop://title", "Title of the plot or '<auto>' if the title of the data object should be used.")
    Q_CLASSINFO("prop://axisLabel", "Label of the direction (x/y) axis or '<auto>' if the descriptions from the data object should be used.")
    Q_CLASSINFO("prop://valueLabel", "Label of the value axis (y-axis) or '<auto>' if the description should be used from data object.")
    Q_CLASSINFO("prop://titleFont", "Font for title.")
    Q_CLASSINFO("prop://labelFont", "Font for axes descriptions.")
    Q_CLASSINFO("prop://axisFont", "Font for axes tick values.")
    Q_CLASSINFO("prop://grid", "enables/disables a grid.")

    Q_CLASSINFO("prop://lineWidth", "width of lines in pixel.")
    Q_CLASSINFO("prop://lineStyle", "style of lines.")
    Q_CLASSINFO("prop://curveStyle", "set the style of the qwt-plot according to curve styles.")

    Q_CLASSINFO("prop://fillCurve", "fill curve below / above or according to baseline.")
    Q_CLASSINFO("prop://curveFillColor", "the fill color for the curve, invalid color leads to line color selection.")
    Q_CLASSINFO("prop://curveFillAlpha", "set the alpha value for the curve fill color seperatly.")

    Q_CLASSINFO("prop://lineSymbol", "Get / Set the current line symbol type")
    Q_CLASSINFO("prop://lineSymbolSize", "Get / Set the current line symbol size")

    Q_CLASSINFO("prop://baseLine", "the baseline/reference for the curveStyle::sticks mode.")
    //Q_CLASSINFO("prop://stickOrientation", "the orientation for the curveStyle::sticks mode.")

    

    Q_CLASSINFO("prop://columnInterpretation", "Define the interpretation of M x N objects as Auto, FirstRow, FirstCol, MultiRows, MultiCols.")
    
    Q_CLASSINFO("prop://pickerLimit", "Define the maximal number of picker for this plot.")
    Q_CLASSINFO("prop://pickerCount", "Number of picker within the plot.")
    Q_CLASSINFO("prop://picker", "Get picker defined by a Mx4 float32 data object. Each row represents one picker and contains the following information: [pixelIndex, physIndex, value, curveIndex]. PixelIndex and physIndex are equal if axisScale = 1 and axisOffset = 0 for the corresponding dataObject.")

    Q_CLASSINFO("prop://backgroundColor", "Set the background / canvas color.")
    
    Q_CLASSINFO("prop://axisColor", "Set the color of the axis.")
    Q_CLASSINFO("prop://textColor", "Set the color of text and tick-numbers")

    Q_CLASSINFO("prop://legendPosition", "Position of the legend (Off, Left, Top, Right, Bottom)")
    Q_CLASSINFO("prop://legendTitles", "Stringlist with the legend titles for all curves. If the list has less entries than curves, the last curves don't have any title. If no legends are given, the data object is checked for tags named 'legendTitle0', 'legendTitle1'... If these tags are not given, the default titles 'curve 0', 'curve 1'... are taken.")

    Q_CLASSINFO("prop://pickerLabelVisible", "Enable and disable the picker label.")
    Q_CLASSINFO("prop://pickerLabelOrientation", "Get / Set label orintation for the picker-label.")
    Q_CLASSINFO("prop://pickerLabelAlignment", "Get / Set label alignment for the picker-label.")
    Q_CLASSINFO("prop://pickerType", "Get / Set the current picker type")

    Q_CLASSINFO("prop://valueScale", "linear or logarithmic scale (various bases) can be chosen for the vertical axis (y-axis). Please consider, that a logarithmic scale can only display values > 1e-100.")
    Q_CLASSINFO("prop://axisScale", "linear or logarithmic scale (various bases) can be chosen for the horizontal axis (x-axis). Please consider, that a logarithmic scale can only display values > 1e-100.")

    

    Q_CLASSINFO("slot://setPicker", "Set plot pickers to a specific curve either in physical or in pixel coordinates. The coordinates are the axis positions only, the values are chosen from the curve values. Existing pickers are deleted at first.")
    Q_CLASSINFO("slot://appendPicker", "Append plot pickers to a specific curve either in physical or in pixel coordinates. The coordinates are the axis positions only, the values are chosen from the curve values.")
    Q_CLASSINFO("slot://plotMarkers", "Plot markers.")
    Q_CLASSINFO("slot://deleteMarkers", "Delete a specific marker")  
    Q_CLASSINFO("slot://deletePicker", "Delete a specific (id >= 0) or all pickers (id = -1)")
    
    Q_CLASSINFO("slot://getDisplayed", "")

    

    public:
        Itom1DQwtPlot(QWidget *parent = 0);
        Itom1DQwtPlot(const QString &itomSettingsFile, AbstractFigure::WindowMode windowMode, QWidget *parent = 0);
        virtual ~Itom1DQwtPlot();

        enum LegendPos { Off = 0, Left = 1, Top = 2, Right = 3, Bottom = 4 };
        enum Symbol
        {
            NoSymbol = -1, Ellipse, Rect, Diamond, Triangle, DTriangle, UTriangle, LTriangle, RTriangle, Cross, XCross, HLine, VLine, Star1, Star2, Hexagon
        };
        enum ColorHandling { AutoColor, Gray, RGB, RGBA, RGBGray };

        ito::RetVal applyUpdate();                              //!< propagates updated data through the subtree
        
        //properties
        QVector<QPointF> getBounds(void) const;
        void setBounds(QVector<QPointF> bounds);

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

        bool getGrid(void) const;
        void setGrid(const bool &enabled);

        qreal getLineWidth(void) const;
        void setLineWidth(const qreal &width);

        Qt::PenStyle getLineStyle() const;
        void setLineStyle(const Qt::PenStyle &style);

        ItomQwtPlotEnums::CurveStyle getCurveStyle() const;
        void setCurveStyle(const ItomQwtPlotEnums::CurveStyle state);

        LegendPos getLegendPosition() const;
        void setLegendPosition(LegendPos legendPosition);

        QStringList getLegendTitles() const;
        void setLegendTitles(const QStringList &legends);

        void setSource(QSharedPointer<ito::DataObject> source);

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

        void setPickerLabelVisible(const bool state);
        bool getPickerLabelVisible() const;

        void setPickerLabelOrientation(const Qt::Orientation val);
        Qt::Orientation getPickerLabelOrientation() const;

        void setPickerLabelAlignment(const Qt::Alignment val);
        Qt::Alignment getPickerLabelAlignment()const ;

        ItomQwtPlotEnums::PlotPickerType getPickerType() const;
        void setPickerType(const ItomQwtPlotEnums::PlotPickerType val);

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

        friend Plot1DWidget;

    protected:
        ito::RetVal init(); // { return m_pContent->init(); }; //called when api-pointers are transmitted, directly after construction

        Plot1DWidget *m_pContent;
        InternalData *m_pData;

    private:
        

        void constructor();

    public slots:
        ito::RetVal setPicker(const QVector<double> &coords, int curveIndex = 0, bool physicalCoordinates = true);
        ito::RetVal appendPicker(const QVector<double> &coords, int curveIndex = 0, bool physicalCoordinates = true);
        ito::RetVal deletePicker(int id = -1);

        ito::RetVal plotMarkers(const ito::DataObject &coords, QString style, QString id = QString::Null(), int plane = -1);
        ito::RetVal deleteMarkers(int id);
        

        QSharedPointer<ito::DataObject> getDisplayed(void);

    private slots:

};
//----------------------------------------------------------------------------------------------------------------------------------

#endif // ITOMPLOT_H
