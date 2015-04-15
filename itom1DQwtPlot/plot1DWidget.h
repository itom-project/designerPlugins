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

#ifndef PLOT1DWIDGET_H
#define PLOT1DWIDGET_H

#include "common/sharedStructures.h"
#include "DataObject/dataobj.h"
#include "common/sharedStructuresPrimitives.h"

#include <qwidget.h>
#include <qstring.h>
#include <qevent.h>
#include <qpainter.h>
#include <qpoint.h>
#include <qtimer.h>
#include <qcoreapplication.h>
#include <qapplication.h>
#include <qqueue.h>
#include <qmenu.h>

#include <qwt_plot_rescaler.h>
#include <qwt_plot.h>
#include <qwt_plot_zoomer.h>
#include <qwt_plot_panner.h>
#include <qwt_plot_curve.h>
#include <qwt_plot_marker.h>
#include <qwt_plot_grid.h>
#include <qwt_plot_magnifier.h>
#include <qwt_symbol.h>

#include "valuepicker1d.h"

#include "../sharedFiles/userInteractionPlotPicker.h"
#include "../sharedFiles/drawItem.h"
#include "../sharedFiles/itomPlotMagnifier.h"
#include "../sharedFiles/itomPlotZoomer.h"
#include "itomPlotMarker.h"
//#include "itom1DQwtPlotEnums.h"

class Itom1DQwtPlot;
class QwtLegend;
struct InternalData;

class Plot1DWidget : public QwtPlot
{
    Q_OBJECT
    public:

        enum tState
        { 
            stateIdle   = 0, 
            statePanner = 1, 
            stateZoomer = 2, 
            statePicker = 3,             
            tPoint = ito::PrimitiveContainer::tPoint, 
            tLine = ito::PrimitiveContainer::tLine, 
            tRect = ito::PrimitiveContainer::tRectangle, 
//            tSquare = ito::PrimitiveContainer::tSquare,
            tEllipse = ito::PrimitiveContainer::tEllipse, 
//            tCircle = ito::PrimitiveContainer::tCircle, 
            tPolygon = ito::PrimitiveContainer::tPolygon
        };

        Plot1DWidget(QMenu *contextMenu, InternalData *data, QWidget * parent = 0);
        ~Plot1DWidget();

        ito::RetVal init();

        bool m_showContextMenu;
        void refreshPlot(const ito::DataObject* dataObj, QVector<QPointF> bounds = QVector<QPointF>() );

        ito::RetVal setInterval(const Qt::Axis axis, const bool autoCalcLimits, const double minValue, const double maxValue);

        void setZoomerEnable(const bool checked);
        void setPickerEnable(const bool checked);
        void setPannerEnable(const bool checked);

        void setGridEnabled(const bool enabled);

        void setMainPickersToIndex(int idx1, int idx2, int curveIdx);

        ito::RetVal plotMarkers(const ito::DataObject *coords, QString style, QString id, int plane);
        ito::RetVal deleteMarkers(const int id);

        ito::RetVal setPicker(const QVector<int> &pxCords);
        ito::RetVal setPicker(const QVector<float> &physCords);

        void setLegendPosition(LegendPosition position, bool visible);
        void setLegendTitles(const QStringList &legends);

        QVector<int> getPickerPixel() const;
        QVector<float> getPickerPhys() const;

        void setSymbolStyle(const QwtSymbol::Style style, int size);

        friend class Itom1DQwtPlot;
        friend class DrawItem;      

    protected:
        void keyPressEvent ( QKeyEvent * event );
        void mousePressEvent ( QMouseEvent * event );
        void mouseMoveEvent ( QMouseEvent * event );
        void mouseReleaseEvent ( QMouseEvent * event );
        void contextMenuEvent(QContextMenuEvent * event);

        void setLabels(const QString &title, const QString &valueLabel, const QString &axisLabel);
        void updateLabels();
        void synchronizeCurrentScaleValues();
        void updateScaleValues(bool recalculateBoundaries = false);

        void configRescaler();

        ito::RetVal userInteractionStart(int type, bool start, int maxNrOfPoints);

        void setState( tState state);
        void updateColors(void);
        void updatePickerStyle(void);

        void setLineWidth(const qreal &width);
        void setLineStyle(const Qt::PenStyle &style);
        void setQwtLineStyle(const Itom1DQwt::tCurveStyle &style);
        void setBaseLine(const qreal &line);
        void setCurveFilled();
        //void setStickOrientation(const qreal &line);
        void setDefaultValueScaleEngine(const Itom1DQwt::ScaleEngine &scaleEngine);
        void setDefaultAxisScaleEngine(const Itom1DQwt::ScaleEngine &scaleEngine);

    private:
        QwtPlotRescaler* m_pRescaler;

        struct Picker
        {
            Picker() : item(NULL), active(0), curveIdx(0) {}
            ItomPlotMarker *item;
            bool active;
            int curveIdx;
        };

        void stickPickerToXPx(Picker *m, double xScaleStart, int dir);
        void stickPickerToSampleIdx(Picker *m, int idx, int curveIdx, int dir);
        void updatePickerPosition(bool updatePositions, bool clear = false);

        int getPickerCount() const {return m_pickers.size();}
        QSharedPointer< ito::DataObject > getPlotPicker() const;

        void home();


        QMenu *m_contextMenu;

        QList<QwtPlotCurve*> m_plotCurveItems;

        QwtPlotGrid *m_pPlotGrid;

        QwtLegend *m_pLegend;
        QStringList m_legendTitles;

        QVector<ito::uint16> m_drawedIemsIndexes;

        QByteArray m_hash; //hash of recently loaded dataObject

        //QwtPlotCurve **m_pContent; //content-element, added to canvas when first valid data object becomes available
        InternalData *m_pData;

        //bool m_startScaledY;
        //bool m_startScaledX;
        bool m_xDirect;
        bool m_yDirect;
        bool m_cmplxState;
        bool m_colorState;
        bool m_layerState;
        bool m_gridEnabled;

        //unsigned char m_autoLineColIndex;
        long m_lineCol;
        qreal m_lineWidth;
        Qt::PenStyle m_lineStyle;
        int m_linePlotID;

        int m_Curser[2];
        int m_actPickerIdx;

        QStringList m_colorList;

        QWidget *m_pParent;
        ItomPlotZoomer *m_pZoomer;
        QwtPlotPanner *m_pPanner;
        ItomPlotMagnifier *m_pMagnifier;

        ValuePicker1D *m_pValuePicker;

        QList<Picker> m_pickers;

        QMenu *m_pCmplxMenu;

        QColor m_inverseColor0, m_inverseColor1;
        int m_activeDrawItem;

        UserInteractionPlotPicker *m_pMultiPointPicker;
        bool m_ignoreNextMouseEvent;

        LegendPosition m_legendPosition;
        bool m_legendVisible;

        Itom1DQwt::tCurveStyle m_qwtCurveStyle;

        qreal m_baseLine;

        QColor m_filledColor;
        Itom1DQwt::tFillCurveStyle m_curveFilled;
        ito::uint8 m_fillCurveAlpa;

        Itom1DQwt::ScaleEngine m_valueScale;
        Itom1DQwt::ScaleEngine m_axisScale;

    signals:

        void statusBarClear();
        void statusBarMessage(const QString &message, int timeout = 0);

        void spawnNewChild(QVector<QPointF>);
        void updateChildren(QVector<QPointF>);

        void setPickerText(const QString &coords, const QString &offsets);

    public slots:
        //void replot();


    private slots:
        void multiPointActivated (bool on);
        void legendItemChecked(const QVariant &itemInfo, bool on);
        

};

struct InternalData
{
    InternalData() : m_title(""), m_axisLabel(""), m_valueLabel(""), m_titleDObj(""),
        m_axisLabelDObj(""), m_valueLabelDObj(""), m_autoTitle(1), m_autoAxisLabel(1), m_autoValueLabel(1),
        m_valueScaleAuto(1), m_valueMin(0), m_valueMax(0), m_elementsToPick(0), m_axisScaleAuto(1), m_axisMin(0), m_axisMax(0), m_forceValueParsing(1),
        m_enablePlotting(true), m_keepAspect(false), m_lineSymboleSize(1)
    {
        m_pDrawItems.clear();
        m_state = Plot1DWidget::stateIdle;
        m_multiLine = Itom1DQwt::AutoRowCol;
        m_colorLine = Itom1DQwt::AutoColor;
        m_pickerLimit = 2;

        m_pickerLabelVisible = false;
        m_pickerType = Itom1DQwt::DefaultMarker;
        Qt::Orientation m_pickerLabelOrientation = Qt::Horizontal;
        m_pickerLabelAlignment = Qt::AlignRight;

        m_axisColor = Qt::black;
        m_textColor = Qt::black;
        m_backgnd = Qt::white;
        m_lineSymbole = QwtSymbol::NoSymbol;
    }

    ~InternalData()
    {
        QList<int> keys = m_pDrawItems.keys();
        for (int i = 0; i < keys.size(); i++)
        {
            if(m_pDrawItems[keys[i]] != NULL)
            {
                DrawItem *delItem = m_pDrawItems[keys[i]];
                delItem->detach();
                m_pDrawItems.remove(keys[i]);
                delete delItem;          
            }
        }

        m_pDrawItems.clear();    
    }
    ito::tDataType m_dataType;
     
//    Plot1DWidget::tState m_state;
    int m_state;
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

    int m_elementsToPick;
    bool m_enablePlotting;
    bool m_keepAspect;

    bool m_pickerLabelVisible;
    Itom1DQwt::tPlotPickerType m_pickerType;
    Qt::Orientation m_pickerLabelOrientation;
    Qt::Alignment m_pickerLabelAlignment;

    QColor m_backgnd;           //!> plot background color
    QColor m_axisColor;         //!> color of axis
    QColor m_textColor;         //!> text color

    //true for one replot if setSource-Property has been set 
    //(even if the same data object is given one more time, 
    //the hash might be the same, but we want to recalcuate 
    //boundaries if values of dataObject changed.
    bool m_forceValueParsing; 

    Itom1DQwt::tMultiLineMode m_multiLine;
    Itom1DQwt::tColorHandling m_colorLine;
    QHash<int, DrawItem *> m_pDrawItems;
    QwtSymbol::Style m_lineSymbole;
    int m_lineSymboleSize;
};

#endif
