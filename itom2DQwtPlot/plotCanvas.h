/* ********************************************************************
   itom measurement system
   URL: http://www.uni-stuttgart.de/ito
   Copyright (C) 2019, Institut fuer Technische Optik (ITO),
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

#ifndef PLOTCANVAS_H
#define PLOTCANVAS_H

#include "common/sharedStructures.h"
#include "common/interval.h"
#include "plot/AbstractFigure.h"

#include "DataObject/dataobj.h"

#include "dataObjItem.h"

#include "itomQwtPlot.h"
#include "itom2dqwtplot.h"
#include "userInteractionPlotPicker.h"
#include "drawItem.h"
#include "itomPlotPicker.h"


#include <qwidget.h>
#include <qstring.h>
#include <qevent.h>
#include <qpainter.h>
#include <qpoint.h>
#include <qtimer.h>
#include <qcoreapplication.h>
#include <qapplication.h>
#include <qspinbox.h>
#include <qhash.h>

#include <qwt_plot_curve.h>
#include <qwt_plot_marker.h>
#include <qwt_plot_shapeitem.h>
#include <qwt_plot_grid.h>
#include <qcolor.h>

class ValuePicker2D;
class DataObjRasterData;
class QWidgetAction;
class QMenu;
class QSlider;

//class UserInteractionPlotPicker;


class PlotCanvas : public ItomQwtPlot
{
    Q_OBJECT
    public:
        enum changeFlag {
            changeNo = 0,
            changeAppearance = 1,
            changeData = 2
        };
        //moved forward declaration here. Can be accessed using class' namespace
        struct InternalData;
        PlotCanvas(PlotCanvas::InternalData *m_pData, ItomQwtDObjFigure * parent = NULL);
        ~PlotCanvas();

        ito::RetVal init(bool overwriteDesignableProperties);
        void refreshPlot(const ito::DataObject *dObj, int plane = -1, const QVector<QPointF> bounds = QVector<QPointF>());

        void changePlane(int plane);
        int getCurrentPlane();

        void internalDataUpdated();

        virtual void setButtonStyle(int style);

        void removeChildPlotIndicators(bool lineChildPlot, bool zStackChildPlot, bool volumeChildPlot, bool resetState = false);

        ito::AutoInterval getInterval(Qt::Axis axis) const;
        void setInterval(Qt::Axis axis, const ito::AutoInterval &interval);

        bool showCenterMarker() const { return m_showCenterMarker; }
        void setShowCenterMarker(bool show);

        Itom2dQwtPlot::GridStyle gridStyle() const { return m_gridStyle; }
        void setGridStyle(Itom2dQwtPlot::GridStyle gridStyle);

        ito::AutoInterval getOverlayInterval(Qt::Axis axis) const;
        void setOverlayInterval(Qt::Axis axis, const ito::AutoInterval &interval);

        ito::int32 getCurrentPlane() const;
        QSharedPointer<ito::DataObject> getDisplayed();
        QSharedPointer<ito::DataObject> getOverlayObject(void);
        QSharedPointer<ito::DataObject> getDisplayedOverlayObject(void);

        void setContourLevels(QSharedPointer<ito::DataObject> contourLevels);
        QSharedPointer<ito::DataObject> getContourLevels() const;
        bool setContourColorMap(const QString & name = "__next__");
        inline QString getContourColorMap() const { return m_colorContourMapName; }
        float getContourLineWidth() const;
        void setContourLineWidth(const float &width);

        ito::RetVal setLinePlot(const double x0, const double y0, const double x1, const double y1);
        
		void setValueAxisScaleEngine(const ItomQwtPlotEnums::ScaleEngine &scaleEngine);

		ItomQwtPlotEnums::ComplexType getComplexStyle() const;
		void setComplexStyle(const ItomQwtPlotEnums::ComplexType &type);


        friend class Itom2dQwtPlot;

    protected:
        virtual void stateChanged(int state);

        void getMinMaxLoc(double &min, ito::uint32 *minLoc, double &max, ito::uint32 *maxLoc);
        void getMinMaxPhysLoc(double &min, double *minPhysLoc, double &max, double *maxPhysLoc);
        void keyPressEvent ( QKeyEvent * event );
        void keyReleaseEvent ( QKeyEvent * event );

        void updateLabels();
        void synchronizeScaleValues();
        void updateScaleValues(bool doReplot = true, bool doZoomBase = true, bool clearStack = false);
        void setColorBarVisible(bool visible);
        bool setColorMap(const QString &colormap = "__next__");
        inline QString colorMapName() const { return m_colorMapName; }

        bool setOverlayColorMap(const QString &colormap = "__next__");
        inline QString colorOverlayMapName() const { return m_colorOverlayMapName; }

        void refreshStyles(bool overwriteDesignableProperties);
        
        void setOverlayObject(ito::DataObject* newOverlay);
        void alphaChanged();

        void home();
        void adjustColorDataTypeRepresentation();

        void lineCutMovedPhys(const QPointF &pt); //pt is in axes coordinates (physical coordinates)
        void lineCutAppendedPhys(const QPointF &pt); //pt is in axes coordinates (physical coordinates)

        void volumeCutAppendedPhys(const QPointF &pt); //pt is in axes coordinates (physical coordinates)
        void volumeCutMovedPhys(const QPointF &pt); //pt is in axes coordinates (physical coordinates)


    private:
        enum Direction { dirX = 0, dirY = 1, dirZ = 2, dirXY = 3, inPlane = 4 };
        void createActions();

        // a1 is line1 start, a2 is line1 end, b1 is line2 start, b2 is line2 end
        bool lineIntersection(const QPointF &a1, const QPointF &a2, const QPointF &b1, const QPointF &b2, QPointF &intersection);
        void setCoordinates(const QVector<QPointF> &pts, bool visible = true);
        ito::RetVal cutVolume(const ito::DataObject* dataObj, const QVector<QPointF> bounds);
        template <typename _Tp> void parseVolumeCutObj( const ito::DataObject* srcObj, const unsigned int& offsetByte, const QVector<int>& stepByte);
        inline void saturation(int &value, int min, int max) { value = (value < min ? min : (value > max ? max : value)); }

        ito::DataObject randImg;

        QwtPlotPicker *m_pLineCutPicker;
        QwtPlotCurve *m_pLineCutLine;
        QwtPlotPicker *m_pVolumeCutPicker;
        QwtPlotCurve *m_pVolumeCutLine;
        ValuePicker2D *m_pValuePicker;
        ItomPlotPicker *m_pStackPicker;
        QwtPlotMarker *m_pStackCutMarker;
        QwtPlotMarker *m_pCenterMarker;
        QwtPlotGrid *m_pPlotGrid;

        QString m_colorContourMapName;
        QString m_colorOverlayMapName;
        QString m_colorMapName;
        

        int m_curColorMapIndex;
        int m_curOverlayColorMapIndex;
        int m_curContourColorMapIndex;
        ItomQwtPlotEnums::ScaleEngine m_valueScale;

        bool m_unitLabelChanged;

        DataObjItem *m_dObjItem;
        DataObjItem *m_dOverlayItem;

        QSharedPointer<ito::DataObject> m_pContourObj;

        DataObjRasterData *m_rasterData;
        DataObjRasterData *m_rasterOverlayData;
        
        Direction m_dir;
        bool m_lineCutValidStart; //true if the first point of the line cut is a valid point inside of the data object
        bool m_volumeCutValidStart; ////true if the first point of the volume cut is a valid point inside of the data object

        InternalData *m_pData;
        const ito::DataObject *m_dObjPtr; //pointer to the current source (original) data object
        ito::DataObject m_dObjVolumeCut; //data object holding the object used for volume cut

        Itom2dQwtPlot::GridStyle m_gridStyle;

        int m_currentDataType;

        Qt::KeyboardModifiers m_activeModifiers;

        bool m_isRefreshingPlot; //true if the refreshPlot method is currently executed (in order to avoid interative, stacked calls to refreshPlot)
        bool m_showCenterMarker;
		bool m_pPaletteIsChanging; //true if the color palette is currently changing in order to avoid that the fonts are set to default

        QAction *m_pActScaleSettings; //
        QAction *m_pActColorPalette;  //
		QMenu *m_pMenuColorPalette; //sub-menu for all available color palettes, will be created in init, when apis are available
        QAction *m_pActToggleColorBar;
        QAction *m_pActValuePicker;
        QAction *m_pActLineCut;
        QAction *m_pActVolumeCut;
        QMenu *m_pMnuLineCutMode;
        QAction *m_pActStackCut; //
        QWidgetAction *m_pActPlaneSelector; //
        QLabel *m_pCoordinates; //
        QWidgetAction *m_pActCoordinates; //
        QAction *m_pActCmplxSwitch; //
        QMenu *m_pMnuCmplxSwitch; //
        QAction *m_pActGrid;
        QMenu *m_pMnuGrid;
        QAction *m_pActDataChannel;
        QMenu *m_pMnuDataChannel;
        QAction* m_pActCntrMarker; //
        QSlider* m_pOverlaySlider;
        QWidgetAction *m_pActOverlaySlider;



    signals:
        void spawnNewChild(QVector<QPointF>);
        void updateChildren(QVector<QPointF>);

    private slots:
        void zStackCutTrackerMoved(const QPoint &pt);
        void zStackCutTrackerAppended(const QPoint &pt);
        void lineCutMovedPx(const QPoint &pt); //pt is in canvas coordinates (pixel)
        void lineCutAppendedPx(const QPoint &pt); //pt is in canvas coordinates (pixel)
        void volumeCutMovedPx(const  QPoint &pt); //pt is in canvas coordinates (pixel)
        void volumeCutAppendedPx(const QPoint &pt); //pt is in canvas coordinates (pixel)

        void mnuScaleSettings();
        void mnuCmplxSwitch(QAction*);
        void mnuDataChannel(QAction*);
        void mnuColorPalette();
		void mnuGroupColorPalette(QAction*);
        void mnuToggleColorBar(bool checked);
        void mnuValuePicker(bool checked);
        void mnuLineCut(bool checked);
        void mnuLineCutMode(QAction *action);
        void mnuVolumeCut(bool checked);
        void mnuStackCut(bool checked);
        void mnuPlaneSelector(int plane);
        void mnuOverlaySliderChanged(int value);
        void mnuCenterMarker(bool checked);
        void mnuSetGrid(QAction *action);

};


struct PlotCanvas::InternalData
{
    InternalData() :
        m_dataType(ito::tFloat64),
        m_title(""),
        m_yaxisLabel(""),
        m_xaxisLabel(""),
        m_valueLabel(""),
        m_titleDObj(""),
        m_xaxisLabelDObj(""),
        m_yaxisLabelDObj(""),
        m_valueLabelDObj(""),
        m_autoTitle(true),
        m_autoxAxisLabel(true),
        m_autoyAxisLabel(true),
        m_autoValueLabel(true),
        m_valueScaleAuto(true),
        m_valueMin(0),
        m_valueMax(0),
        m_xaxisScaleAuto(true),
        m_xaxisMin(0),
        m_xaxisMax(0),
        m_xaxisVisible(true),
        m_yaxisScaleAuto(true),
        m_yaxisMin(0),
        m_yaxisMax(0),
        m_yaxisFlipped(false),
        m_yaxisVisible(true),
        m_colorBarVisible(false),
        m_cmplxType(ItomQwtPlotEnums::CmplxReal),
        m_dataChannel(ItomQwtPlotEnums::ChannelAuto),
        m_alpha(0),
        m_overlayScaleAuto(true),
        m_overlayMin(0),
        m_overlayMax(0)
    {
    }
    ~InternalData(){}


    ito::tDataType m_dataType;

    QString m_title;
    QString m_yaxisLabel;
    QString m_xaxisLabel;
    QString m_valueLabel;

    QString m_titleDObj;
    QString m_xaxisLabelDObj;
    QString m_yaxisLabelDObj;
    QString m_valueLabelDObj;

    bool m_autoTitle;
    bool m_autoxAxisLabel;
    bool m_autoyAxisLabel;
    bool m_autoValueLabel;

    bool m_valueScaleAuto;
    double m_valueMin;
    double m_valueMax;

    bool m_overlayScaleAuto;
    double m_overlayMin;
    double m_overlayMax;

    bool m_xaxisScaleAuto;
    double m_xaxisMin;
    double m_xaxisMax;
    bool m_xaxisVisible;

    bool m_yaxisScaleAuto;
    double m_yaxisMin;
    double m_yaxisMax;
    bool m_yaxisFlipped;
    bool m_yaxisVisible;

    bool m_colorBarVisible;
    unsigned char m_alpha;

    ItomQwtPlotEnums::ComplexType m_cmplxType;
    ItomQwtPlotEnums::DataChannel m_dataChannel;

    QHash<QString, ito::Param*> m_selectedOutputParameters;
};

#endif
