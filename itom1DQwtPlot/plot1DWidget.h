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

#ifndef PLOT1DWIDGET_H
#define PLOT1DWIDGET_H

#include "common/sharedStructures.h"
#include "DataObject/dataobj.h"
#include "plot/AbstractFigure.h"

#include <qwidget.h>
#include <qstring.h>
#include <qevent.h>
#include <qpainter.h>
#include <qpoint.h>
#include <qtimer.h>
#include <qcoreapplication.h>
#include <qqueue.h>
#include <qmenu.h>

#include <qwt_plot_curve.h>
#include <qwt_plot_marker.h>
#include <qwt_plot_grid.h>
#include <qwt_symbol.h>

#include "valuepicker1d.h"

#include "../sharedFiles/itomQwtPlot.h"
#include "itom1DQwtPlot.h"
#include "itomPlotMarker.h"

class Itom1DQwtPlot;
class ItomQwtDObjFigure;
class QwtLegend;
struct InternalData;
class QwtPlotCurveProperty;


class Plot1DWidget : public ItomQwtPlot
{
    Q_OBJECT
    public:

        Plot1DWidget(InternalData *data, ItomQwtDObjFigure *parent = 0);
        ~Plot1DWidget();

        ito::RetVal init(bool overwriteDesignableProperties);

        void refreshPlot(const ito::DataObject* dataObj, QVector<QPointF> bounds = QVector<QPointF>(), const ito::DataObject* xVec = NULL);

        ito::RetVal setInterval(const Qt::Axis axis, const bool autoCalcLimits, const double minValue, const double maxValue);

        void setZoomerEnable(const bool checked);
        void setPickerEnable(const bool checked);
        void setPannerEnable(const bool checked);

        void setGridStyle(const Itom1DQwtPlot::GridStyle gridStyle);

        void setMainPickersToIndex(int idx1, int idx2, int curveIdx);

        ito::RetVal setPicker(const QVector<double> &coords, int curveIndex = 0, bool physNotPix = true, bool append = false);
        ito::RetVal clearPicker(int id = -1, bool doReplot = true);

        void setLegendPosition(LegendPosition position, bool visible);
		void setLegendLabelWidth(const int &width);
        void setLegendTitles(const QStringList &legends, const ito::DataObject *object);
		void toggleLegendLabel(QwtPlotCurve* curve, const bool state);
        void setLegendFont(const QFont &font);
        void applyLegendFont();
        const QFont getLegendFont() const;

        QVector<int> getPickerPixel() const;
        QVector<float> getPickerPhys() const;

        QSharedPointer<ito::DataObject> getDisplayed(bool copyDisplayedAsComplex);

        void setSymbolStyle(const QwtSymbol::Style style, int size);

        virtual void setButtonStyle(int style);

        void synchronizeCurrentScaleValues();
        void updateScaleValues(bool doReplot = true, bool doZoomBase = true);
        
        void enableObjectGUIElements(const int mode);

        int getPickerCount() const { return m_pickers.size(); }
        void setPickerLimit(int limit);
        QSharedPointer< ito::DataObject > getPlotPicker() const;

        bool getAntiAliased() const { return m_antiAliased; }
        void setAntiAliased(bool antiAliased);

        void setRowPresentation(const ItomQwtPlotEnums::MultiLineMode idx);
        void setRGBPresentation(const ItomQwtPlotEnums::ColorHandling idx);

        ito::RetVal setCurveProperty(int index, const QByteArray &property, const QVariant &value);
        QVariant getCurveProperty(int index, const QByteArray &property);

		QList<QwtPlotCurve*> getplotCurveItems();
		QList<QwtPlotCurveProperty*> getPlotCurveProperty();

		void setComplexStyle(const ItomQwtPlotEnums::ComplexType &type);
		ItomQwtPlotEnums::ComplexType getComplexStyle() const;


        friend Itom1DQwtPlot;

    protected:
        void keyPressEvent ( QKeyEvent * event );
        void mousePressEvent ( QMouseEvent * event );
        void mouseMoveEvent ( QMouseEvent * event );
        void mouseReleaseEvent ( QMouseEvent * event );

        void setLabels(const QString &title, const QString &valueLabel, const QString &axisLabel);
        void updateLabels();
        void setPickerText(const QString &coords, const QString &offsets);
		

        void updatePickerStyle(void);

        void setLineWidth(const qreal &width);
        void setLineStyle(const Qt::PenStyle &style);
        void setQwtLineStyle(const ItomQwtPlotEnums::CurveStyle &style);
        void setBaseLine(const qreal &line);
        void setCurveFilled();
        //void setStickOrientation(const qreal &line);
        void setDefaultValueScaleEngine(const ItomQwtPlotEnums::ScaleEngine &scaleEngine);
        void setDefaultAxisScaleEngine(const ItomQwtPlotEnums::ScaleEngine &scaleEngine);


        void home();

        void stateChanged(int state);

    private:

        struct Picker
        {
            Picker() : item(NULL), active(0), curveIdx(0) {}
            ItomPlotMarker *item;
            bool active;
            int curveIdx;
        };

        void stickPickerToXPx(Picker *m, double xScaleStart, int dir);
        void stickPickerToSampleIdx(Picker *m, int idx, int dir);
        void updatePickerPosition(bool updatePositions, bool clear = false);
        void createActions();

        QList<QwtPlotCurve*> m_plotCurveItems;
        QList<QwtPlotCurveProperty*> m_plotCurvePropertyItems; //sychrone with m_plotCurveItems. Every item is derived from QObject and therefore propagate a Q_PROPERTY based set of properties for each curve!
        QwtPlotGrid *m_pPlotGrid;
        QwtLegend *m_pLegend;
        QStringList m_legendTitles;
        QByteArray m_hash; //hash of recently loaded dataObject

        InternalData *m_pData;

        bool m_xDirect;
        bool m_yDirect;
        bool m_cmplxState;
        bool m_colorState;
        bool m_layerState;
        Itom1DQwtPlot::GridStyle m_gridStyle;

        //unsigned char m_autoLineColIndex;
        long m_lineCol;
        qreal m_lineWidth;
        Qt::PenStyle m_lineStyle;
        ito::AbstractFigure::UnitLabelStyle m_unitLabelStyle;
        bool m_antiAliased;

        int m_Curser[2];
//        int m_actPickerIdx;

        QList<QColor> m_colorList;

        ValuePicker1D *m_pValuePicker;

        QList<Picker> m_pickers;
        
        LegendPosition m_legendPosition;
        bool m_legendVisible;
		int m_pLegendLabelWidth;

        ItomQwtPlotEnums::CurveStyle m_qwtCurveStyle;

        qreal m_baseLine;
        bool m_hasParentForRescale;
        int m_guiModeCache;

        QColor m_filledColor;
        ItomQwtPlotEnums::FillCurveStyle m_curveFilled;
        ito::uint8 m_fillCurveAlpa;

        ItomQwtPlotEnums::ScaleEngine m_valueScale;
        ItomQwtPlotEnums::ScaleEngine m_axisScale;

        QVector<QPointF> m_currentBounds;
        QFont m_legendFont;

        QAction* m_pActScaleSettings;
        QAction* m_pRescaleParent;
        QAction  *m_pActPicker;
        QMenu    *m_pMnuSetPicker;
        QAction  *m_pActSetPicker;
        QAction *m_pActForward;
        QAction *m_pActBack;
        QAction *m_pActCmplxSwitch;
        QMenu *m_pMnuCmplxSwitch;
        QAction *m_pActRGBSwitch;
        QMenu *m_pMnuRGBSwitch;
        QLabel *m_pLblMarkerOffsets;
        QLabel *m_pLblMarkerCoords;
        QAction *m_pActGrid;
        QAction *m_pActGridSettings;
        QAction *m_pActMultiRowSwitch;
        QMenu *m_pMnuMultiRowSwitch;
        QAction *m_pActLegendSwitch;
        QMenu *m_pMnuLegendSwitch;

        QAction* m_pActXVAuto;
        QAction* m_pActXVFR;
        QAction* m_pActXVFC;
        QAction* m_pActXVMR;
        QAction* m_pActXVMC;
        QAction* m_pActXVML;
        QAction* m_pActRGBA;
        QAction* m_pActGray;
        QAction* m_pActRGBL;
        QAction* m_pActRGBAL;
        QAction* m_pActRGBG;
		ItomQwtPlotEnums::ComplexType m_pComplexStyle;

    signals:

        

        void spawnNewChild(QVector<QPointF>);
        void updateChildren(QVector<QPointF>);
		void curveChanged();
		void legendModified();

    public slots:
        //void replot();


    private slots:
        void legendItemChecked(const QVariant &itemInfo, bool on);
        void mnuCmplxSwitch(QAction*);
        void mnuLegendSwitch(QAction*);
        void mnuMultiRowSwitch(QAction*);
        void mnuRGBSwitch(QAction*);
        void mnuSetPicker(QAction *action);
        void mnuParentScaleSetting();
        void mnuGridEnabled(bool checked);
        void mnuScaleSettings();
        void mnuPickerClick(bool checked);
        

};

struct InternalData
{
    InternalData() :
        m_title(""),
        m_axisLabel(""),
        m_valueLabel(""),
        m_titleDObj(""),
        m_axisLabelDObj(""),
        m_valueLabelDObj(""),
        m_autoTitle(1),
        m_autoAxisLabel(1),
        m_autoValueLabel(1),
        m_valueScaleAuto(1),
        m_valueMin(0),
        m_valueMax(0),
        m_axisScaleAuto(1),
        m_axisMin(0),
        m_axisMax(0),
        m_forceValueParsing(1),
        m_lineSymboleSize(8),
        m_multiLine(ItomQwtPlotEnums::AutoRowCol),
        m_colorLine(ItomQwtPlotEnums::AutoColor),
        m_pickerLimit(2),
        m_pickerLabelVisible(false),
        m_pickerLabelOrientation(Qt::Horizontal),
        m_pickerLabelAlignment(Qt::AlignRight),
        m_lineSymbole(QwtSymbol::NoSymbol)
    {
    }

    ~InternalData()
    { 
    }

    ito::tDataType m_dataType;
     
    int m_pickerLimit;

    QString m_title;
    QString m_axisLabel;
    QString m_valueLabel;

    QString m_titleDObj;
    QString m_axisLabelDObj;
    QString m_valueLabelDObj;

    bool m_autoTitle;
    bool m_autoAxisLabel;
    bool m_autoValueLabel;

    bool m_valueScaleAuto;
    double m_valueMin;
    double m_valueMax;

    bool m_axisScaleAuto;
    double m_axisMin;
    double m_axisMax;

    bool m_pickerLabelVisible;
    ItomQwtPlotEnums::PlotPickerType m_pickerType;
    Qt::Orientation m_pickerLabelOrientation;
    Qt::Alignment m_pickerLabelAlignment;

    

    //true for one replot if setSource-Property has been set 
    //(even if the same data object is given one more time, 
    //the hash might be the same, but we want to recalcuate 
    //boundaries if values of dataObject changed.
    bool m_forceValueParsing; 

    ItomQwtPlotEnums::MultiLineMode m_multiLine;
    ItomQwtPlotEnums::ColorHandling m_colorLine;
    
    QwtSymbol::Style m_lineSymbole;
    int m_lineSymboleSize;
};

#endif
