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

#ifndef DATAOBJECTRASTERDATA_H
#define DATAOBJECTRASTERDATA_H

#include "common/sharedStructures.h"
#include "plot/AbstractFigure.h"
#include "DataObject/dataobj.h"
#include "itomQwtPlotEnums.h"

#include <qwt_series_data.h>
#include <qlist.h>
#include <qrect.h>
#include <qsharedpointer.h>
#include <qwt_scale_map.h>

using namespace ito;

//----------------------------------------------------------------------------------------------------------------------------------
class DataObjectSeriesData : public QwtSeriesData<QPointF>
{
    public:

        enum ColorType { grayColor = 4, blueColor = 0, greenColor = 1, redColor = 2, alphaColor = 3 };
        enum Direction { dirX = 0, dirY = 1, dirZ = 2, dirXY = 3 };

        struct PtsWeighted {
            PtsWeighted()
            {
                xPx[0] = 0; xPx[1] = 0;
                yPx[0] = 0; yPx[1] = 0;
                weights[0] = 0; weights[1] = 0; weights[2] = 0; weights[3] = 0;
            }
            int xPx[2];
            int yPx[2];
            float weights[4]; // weights are for (xPx[0],yPx[0]),(xPx[1],yPx[0]),(xPx[0],yPx[1]),(xPx[1],yPx[1])
        };

        struct LineData {
            LineData() : dir(DataObjectSeriesData::dirX), nrPoints(0), startPhys(0.0), stepSizePhys(0.0), matOffset(0), matStepSize(1), valid(0), plane(0) {}
            Direction dir;
            int nrPoints;
            int plane;
            float startPhys;
            float stepSizePhys; //xRight = xLeft + xStepSize * (m_numPts - 1)
            QPoint startPx;
            QSize stepSizePx;
            int matOffset; //in bytes
            int matStepSize; //in bytes
            QVector<PtsWeighted> points; //only for dirXY, else size=0
            QVector<int> matSteps; //only for dirXY, else size=0 (fast mode)
            bool valid;
        };

        /*struct PtsIdxAndWeights {
            int32 rangeX[2];
            int32 rangeY[2];
            float weights[4];

            PtsIdxAndWeights() { memset(rangeX, 0, 2*sizeof(int32)); memset(rangeY, 0, 2*sizeof(int32)); memset(weights, 0, 4*sizeof(float)); }
        };*/



        explicit DataObjectSeriesData(const int fastmode);
        ~DataObjectSeriesData();

        void beginSampling(const QwtScaleMap &xMap, const QwtScaleMap &yMap, const QRectF &canvasRect);
        void endSampling();

        QPoint indexRange(const QwtScaleMap &xMap, bool clipMargin) const; //!< returns first and last sample index, that is contained within xMap (physical values). If clipMargin is false, first and last are the closest values to xMap which are not contained in xMap (e.g. for polygonal drawings to see the line going out of the window)

        QPointF sample(size_t n) const;

        //returns the bounding box in axis coordinates for this line
        QRectF boundingRect() const;

        RetVal getMinMaxLoc(double &min, double &max, int &minSampleIdx, int &maxSampleIdx) const;
        RetVal getMinMaxLocCropped(const QwtInterval &xInterval, const QwtInterval &yInterval, double &min, double &max, int &minSampleIdx, int &maxSampleIdx) const; //!< returns retWarning if no valid points are within xInterval and yInterval

        size_t size() const;

        inline int isDobjInit() { return m_pDataObj != NULL; }

        bool floatingPointValues() const;

        virtual RetVal updateDataObject(const ito::DataObject* dataObj, QVector<QPointF> bounds, const ito::DataObject* xVec = NULL, const QVector<QPointF>& boundsX = QVector<QPointF>());
        const ito::DataObject* getDataObject() const { return m_pDataObj; }

        inline void setCmplxState(ItomQwtPlotEnums::ComplexType state) { m_cmplxState = state; }
        inline ItomQwtPlotEnums::ComplexType getCmplxState() const { return m_cmplxState; }

        virtual int getPosToPix(const double physx, const double physy = -1, const int indexHint = -1) const;
        
        QString getDObjValueLabel(const AbstractFigure::UnitLabelStyle &unitLabelStyle) const;
        QString getDObjAxisLabel(const AbstractFigure::UnitLabelStyle &unitLabelStyle)  const;
        void getDObjValueDescriptionAndUnit(std::string &description, std::string &unit) const;
        void getDObjAxisDescriptionAndUnit(std::string &description, std::string &unit) const;

        void setIntervalRange(Qt::Axis axis, bool autoCalcLimits, double minValue, double maxValue);

        inline QByteArray getHash() const { return m_hash; }
        
        void calcHash();

        void setColorState(int newVal) {m_colorState = (ColorType)newVal;}

        template <typename _Tp> _Tp sampleComplex(const size_t& n) const; //only supports ito::complex64 and ito::complex128
        inline bool hasAxisObj() const { return m_hasXObj; } //!< returns true if there is a axisObject for the x-axis
        inline bool getXCoordsWholeNumber() const { return m_xCoordsWholeNumber;  }
        QString getAxisUnit() const { return m_dObjAxisUnit; }
        QString getValueUnit() const { return m_dObjValueUnit; }

    protected:

        inline void saturation(int &value, int min, int max) { value = (value < min ? min : (value > max ? max : value)); }
        inline QString fromStdLatin1String(const std::string &str) { return QString::fromLatin1(str.data()); }

        bool m_hasXObj;
        bool m_xCoordsWholeNumber; //true, if the x-position of every sample is a whole number (integer), else false. If not decidable, set it to false
        QString m_dObjAxisDescription;
        QString m_dObjAxisUnit;

    private:
        
        LineData m_d;
        const ito::DataObject* m_pDataObj;               /*!< borrowed reference, do not delete here */
        ColorType m_colorState;
        bool inSamplingMode;
        int m_fast;
        QByteArray m_hash;
        
        QString m_dObjValueDescription;
        QString m_dObjValueUnit;

        bool m_autoScaleY;
        double m_minY;
        double m_maxY;
        bool m_autoScaleX;
        double m_minX;
        double m_maxX;
        ItomQwtPlotEnums::ComplexType m_cmplxState;
};

//----------------------------------------------------------------------------------------------------------------------------------

#endif
