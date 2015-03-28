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

#ifndef ITOM1DPLOT_H
#define ITOM1DPLOT_H

#if defined(ITOMSHAREDDESIGNER)
    #define ITOM1DPLOT_EXPORT Q_DECL_EXPORT
#else
    #define ITOM1DPLOT_EXPORT Q_DECL_IMPORT
#endif


#include "plot/AbstractDObjFigure.h"
#include "itom1DQwtPlotEnums.h"
//#include "plot1DWidget.h"

#include <qaction.h>
#include <qsharedpointer.h>
#include <qwidget.h>
#include <qstyle.h>
#include <qstyleoption.h>
#include <qpainter.h>
#if QT_VERSION >= 0x050000
#include <QtWidgets/qlabel.h>
#endif

#ifndef DECLAREMETADATAOBJECT
    Q_DECLARE_METATYPE(QSharedPointer<ito::DataObject>)
    #define DECLAREMETADATAOBJECT
#endif

class Plot1DWidget;
class ItomPlotMarker;
struct InternalData;


class ITOM1DPLOT_EXPORT Itom1DQwtPlot : public ito::AbstractDObjFigure
{
    Q_OBJECT
    Q_PROPERTY(QVector<QPointF> bounds READ getBounds WRITE setBounds DESIGNABLE false)
    Q_PROPERTY(QString title READ getTitle WRITE setTitle RESET resetTitle USER true)
    Q_PROPERTY(QString axisLabel READ getAxisLabel WRITE setAxisLabel RESET resetAxisLabel USER true)
    Q_PROPERTY(QString valueLabel READ getValueLabel WRITE setValueLabel RESET resetValueLabel USER true)
    Q_PROPERTY(QFont titleFont READ getTitleFont WRITE setTitleFont USER true)
    Q_PROPERTY(QFont labelFont READ getLabelFont WRITE setLabelFont USER true)
    Q_PROPERTY(QFont axisFont READ getAxisFont WRITE setAxisFont USER true)
    Q_PROPERTY(bool grid READ getGrid WRITE setGrid USER true)

    Q_PROPERTY(qreal lineWidth READ getLineWidth WRITE setLineWidth USER true)
    Q_PROPERTY(Qt::PenStyle lineStyle READ getLineStyle WRITE setLineStyle USER true)
    Q_PROPERTY(Itom1DQwt::tCurveStyle curveStyle READ getCurveStyle WRITE setCurveStyle USER true);
    Q_PROPERTY(bool fillCurve READ getCurveFilled WRITE setCurveFilled USER true)
    Q_PROPERTY(QColor curveFillColor READ getCurveFillColor WRITE setCurveFillColor RESET resetCurveFillColor USER true)

    Q_PROPERTY(qreal baseLine READ getBaseLine WRITE setBaseLine USER true)

    Q_PROPERTY(tSymbol lineSymbol READ getLineSymbol WRITE setLineSymbol USER true);
    Q_PROPERTY(int lineSymbolSize READ getLineSymbolSize WRITE setLineSymbolSize USER true);

    // Properties related with geometric elements
    Q_PROPERTY(QSharedPointer< ito::DataObject > geometricElements READ getGeometricElements WRITE setGeometricElements DESIGNABLE false)
    Q_PROPERTY(int geometricElementsCount READ getGeometricElementsCount DESIGNABLE false)
    Q_PROPERTY(bool keepAspectRatio READ getkeepAspectRatio WRITE setkeepAspectRatio USER true)
    Q_PROPERTY(bool enablePlotting READ getEnabledPlotting WRITE setEnabledPlotting USER true)
    Q_PROPERTY(int selectedGeometry READ getSelectedElement WRITE setSelectedElement DESIGNABLE false )

    Q_PROPERTY(Itom1DQwt::tMultiLineMode columnInterpretation READ getRowPresentation WRITE setRowPresentation RESET resetRowPresentation DESIGNABLE true USER true)
    
    Q_PROPERTY(int pickerLimit READ getPickerLimit WRITE setPickerLimit RESET resetPickerLimit DESIGNABLE true USER true)
    Q_PROPERTY(int pickerCount READ getPickerCount DESIGNABLE false USER true)
    Q_PROPERTY(QSharedPointer< ito::DataObject > picker READ getPicker DESIGNABLE false)
    
    Q_PROPERTY(QColor backgroundColor READ getBackgroundColor WRITE setBackgroundColor USER true)
    Q_PROPERTY(QColor axisColor READ getAxisColor WRITE setAxisColor USER true)
    Q_PROPERTY(QColor textColor READ getTextColor WRITE setTextColor USER true)

    Q_PROPERTY(LegendPos legendPosition READ getLegendPosition WRITE setLegendPosition USER true);
    Q_PROPERTY(QStringList legendTitles READ getLegendTitles WRITE setLegendTitles USER true);

    Q_PROPERTY(bool pickerLabelVisible READ getPickerLabelVisible WRITE setPickerLabelVisible USER true);
    Q_PROPERTY(Qt::Orientation pickerLabelOrientation READ getPickerLabelOrientation WRITE setPickerLabelOrientation USER true);
    Q_PROPERTY(Qt::Alignment pickerLabelAlignment READ getPickerLabelAlignment WRITE setPickerLabelAlignment USER true);
    Q_PROPERTY(Itom1DQwt::tPlotPickerType pickerType READ getPickerType WRITE setPickerType USER true);


    Q_ENUMS(LegendPos);
    Q_ENUMS(tSymbol);
    
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

    Q_CLASSINFO("prop://lineSymbol", "Get / Set the current line symbol type")
    Q_CLASSINFO("prop://lineSymbolSize", "Get / Set the current line symbol size")

    Q_CLASSINFO("prop://baseLine", "the baseline/reference for the curveStyle::sticks mode.")
    //Q_CLASSINFO("prop://stickOrientation", "the orientation for the curveStyle::sticks mode.")

    Q_CLASSINFO("prop://geometricElements", "Geometric elements defined by a float32[11] array for each element.")
    Q_CLASSINFO("prop://geometricElementsCount", "Number of currently existing geometric elements.")
    Q_CLASSINFO("prop://keepAspectRatio", "Enable and disable a fixed 1:1 aspect ratio between x and y axis.")
    Q_CLASSINFO("prop://enablePlotting", "Enable and disable internal plotting functions and GUI-elements for geometric elements.")
    Q_CLASSINFO("prop://selectedGeometry", "Get or set the currently highlighted geometric element. After manipulation the last element stays selected.")

    Q_CLASSINFO("prop://columnInterpretation", "Define the interpretation of M x N objects as Auto, FirstRow, FirstCol, MultiRows, MultiCols.")
    
    Q_CLASSINFO("prop://pickerLimit", "Define the maximal number of picker for this plot.")
    Q_CLASSINFO("prop://pickerCount", "Number of picker within the plot.")
    Q_CLASSINFO("prop://picker", "Get picker defined by a float32[3] array for each element containing [pixelIndex, physIndex, value].")

    Q_CLASSINFO("prop://backgroundColor", "Set the background / canvas color.")
    Q_CLASSINFO("prop://axisColor", "Set the color of the axis.")
    Q_CLASSINFO("prop://textColor", "Set the color of text and tick-numbers")

    Q_CLASSINFO("prop://legendPosition", "Position of the legend (Off, Left, Top, Right, Bottom)")
    Q_CLASSINFO("prop://legendTitles", "Stringlist with the legend titles for all curves. If the list has less entries than curves, the last curves don't have any title. If no legends are given, the default titles 'curve 0', 'curve 1'... are taken.")

    Q_CLASSINFO("prop://pickerLabelVisible", "Enable and disable the picker label.")
    Q_CLASSINFO("prop://pickerLabelOrientation", "Get / Set label orintation for the picker-label.")
    Q_CLASSINFO("prop://pickerLabelAlignment", "Get / Set label alignment for the picker-label.")
    Q_CLASSINFO("prop://pickerType", "Get / Set the current picker type")

    Q_CLASSINFO("slot://setPicker", "Set the position of a plot picker either in physical or in pixel coordinates")
    //Q_CLASSINFO("slot://setPicker", "Set the position of a plot picker in pixel coordinates")  
    Q_CLASSINFO("slot://plotMarkers", "Delete a specific marker")
    Q_CLASSINFO("slot://deleteMarkers", "Delete a specific marker")  
    Q_CLASSINFO("slot://copyToClipBoard", "")
    Q_CLASSINFO("slot://userInteractionStart", "")  
    Q_CLASSINFO("slot://clearGeometricElements", "")
    Q_CLASSINFO("slot://getDisplayed", "")

    Q_CLASSINFO("slot://setGeometricElementLabel", "Set the label of geometric element with the index id")
    Q_CLASSINFO("slot://setGeometricElementLabelVisible", "Set the visibility of the label of geometric element with the index id")

    Q_CLASSINFO("signal://plotItemsFinished", "Signal emitted when geometrical plotting was finished.") 
    Q_CLASSINFO("signal://userInteractionDone", "")
    Q_CLASSINFO("signal://plotItemChanged", "")
    Q_CLASSINFO("signal://plotItemDeleted", "")
    Q_CLASSINFO("signal://plotItemsDeleted", "")

    DESIGNER_PLUGIN_ITOM_API

    public:
        Itom1DQwtPlot(QWidget *parent = 0);
        Itom1DQwtPlot(const QString &itomSettingsFile, AbstractFigure::WindowMode windowMode, QWidget *parent = 0);
        virtual ~Itom1DQwtPlot();

        enum LegendPos { Off = 0, Left = 1, Top = 2, Right = 3, Bottom = 4 };
        enum tSymbol
        {
            NoSymbol = -1, Ellipse, Rect, Diamond, Triangle, DTriangle, UTriangle, LTriangle, RTriangle, Cross, XCross, HLine, VLine, Star1, Star2, Hexagon
        };
        ito::RetVal applyUpdate();                              //!< propagates updated data through the subtree
        
        //properties
        bool getContextMenuEnabled() const;
        void setContextMenuEnabled(bool show); 

        QVector<QPointF> getBounds(void);
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

        Qt::PenStyle getLineStyle(void) const;
        void setLineStyle(const Qt::PenStyle &style);

        Itom1DQwt::tCurveStyle getCurveStyle(void) const;
        void setCurveStyle(const Itom1DQwt::tCurveStyle state);

        LegendPos getLegendPosition() const;
        void setLegendPosition(LegendPos legendPosition);

        QStringList getLegendTitles() const;
        void setLegendTitles(const QStringList &legends);

        void setSource(QSharedPointer<ito::DataObject> source);
    
        int getGeometricElementsCount() const;
        void setGeometricElementsCount(const int value){ return;}

        bool getkeepAspectRatio(void) const;
        void setkeepAspectRatio(const bool &keepAspectEnable);

        QSharedPointer< ito::DataObject > getGeometricElements();
        void setGeometricElements(QSharedPointer< ito::DataObject > geometricElements);

        bool getEnabledPlotting(void) const;
        void setEnabledPlotting(const bool &enabled);

        int getSelectedElement(void) const;
        void setSelectedElement(const int idx);

        Itom1DQwt::tMultiLineMode getRowPresentation(void) const;
        void setRowPresentation(const Itom1DQwt::tMultiLineMode idx);
        void resetRowPresentation(); 

        int getRGBPresentation(void) const;
        void setRGBPresentation(const int idx);
        void resetRGBPresentation(); 

        int getPickerLimit(void) const;
        void setPickerLimit(const int idx);
        void resetPickerLimit(); 

        int getPickerCount(void) const;
        QSharedPointer< ito::DataObject > getPicker() const;

        QPixmap renderToPixMap(const int xsize, const int ysize, const int resolution);

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

        Itom1DQwt::tPlotPickerType getPickerType() const;
        void setPickerType(const Itom1DQwt::tPlotPickerType val);

        tSymbol getLineSymbol() const;
        void setLineSymbol(const tSymbol symbol);
        void resetLineSymbol();

        int getLineSymbolSize() const;
        void setLineSymbolSize(int size);
        void resetLineSymbolSize();

        void setBaseLine(const qreal val);
        qreal getBaseLine() const;

        QColor getCurveFillColor() const;
        void setCurveFillColor(const QColor val);
        void resetCurveFillColor();

        bool getCurveFilled() const;
        void setCurveFilled(const bool state);


        friend class Plot1DWidget;

    protected:
        void createActions();
        ito::RetVal init(); // { return m_pContent->init(); }; //called when api-pointers are transmitted, directly after construction

        Plot1DWidget *m_pContent;
        InternalData *m_data;

    private:
        QAction* m_pActScaleSetting;
        QAction* m_pRescaleParent;
        QAction  *m_pActPan;
        QAction  *m_pActZoomToRect;
        QAction  *m_pActMarker;
        QAction *m_pActAspectRatio;
        QAction *m_pActSave;
        QAction *m_pActCopyClipboard;
        QAction *m_pActHome;
        QMenu    *m_pMnuSetMarker;
        QAction  *m_pActSetMarker;
        QAction *m_pActForward;
        QAction *m_pActBack;
        QAction* m_pActCmplxSwitch;
        QMenu *m_pMnuCmplxSwitch;
        QAction *m_pActRGBSwitch;
        QMenu *m_pMnuRGBSwitch;
        QLabel *m_pLblMarkerOffsets;
        QLabel *m_pLblMarkerCoords;
        QActionGroup *m_pDrawModeActGroup;
        QAction *m_pActClearDrawings;
        QAction* m_pActDrawMode;
        QMenu *m_pMnuDrawMode;
        QAction *m_pActProperties;
        QAction *m_pActGrid;
        QAction *m_pActGridSettings;

        QAction* m_pActMultiRowSwitch;
        QMenu *m_pMnuMultiRowSwitch;

        void constructor();

        ito::RetVal qvector2DataObject(const ito::DataObject *dstObject);
        ito::RetVal exportCanvas(const bool copyToClipboardNotFile, const QString &fileName, QSizeF curSize = QSizeF(0.0,0.0), const int resolution = 300);

    public slots:
        //ito::RetVal setPicker(const QVector<ito::int32> &pxCords);
        //ito::RetVal setPicker(const QVector<ito::float32> &physCords);
        ito::RetVal setPicker(const QVector<int> &pxCords);
        ito::RetVal setPicker(const QVector<float> &physCords);

        ito::RetVal plotMarkers(const ito::DataObject &coords, QString style, QString id = QString::Null(), int plane = -1);
        ito::RetVal deleteMarkers(int id);
        ito::RetVal copyToClipBoard();

        void userInteractionStart(int type, bool start, int maxNrOfPoints = -1);
        ito::RetVal clearGeometricElements(void);

        QSharedPointer<ito::DataObject> getDisplayed(void);
        
        ito::RetVal setGeometricElementLabel(int id, QString label);
        ito::RetVal setGeometricElementLabelVisible(int id, bool setVisible);

    private slots:
        void resizeEvent ( QResizeEvent * event );

        void mnuMarkerClick(bool checked);
        void mnuPanner(bool checked);
        void mnuScaleSetting();
        void mnuParentScaleSetting();
        void mnuCmplxSwitch(QAction *action);
        void mnuRGBSwitch(QAction *action);
        void mnuMultiRowSwitch(QAction *action);
        void mnuSetMarker(QAction *action);
        void mnuZoomer(bool checked);
        void mnuExport();

        void mnuActRatio(bool checked);
        void mnuDrawMode(QAction *action);
        void mnuDrawMode(bool checked);

        void mnuGridEnabled(bool checked);
        void mnuHome();
        void setPickerText(const QString &coords, const QString &offsets);

    signals:
        void userInteractionDone(int type, bool aborted, QPolygonF points);
        void plotItemChanged(int idx, int flags, QVector<float> values);
        void plotItemDeleted(int idx);
        void plotItemsDeleted();
        void plotItemsFinished(int type, bool aborted);
};
//----------------------------------------------------------------------------------------------------------------------------------

#endif // ITOMPLOT_H
