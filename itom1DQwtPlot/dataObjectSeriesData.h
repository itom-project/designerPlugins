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

#ifndef DATAOBJECTRASTERDATA_H
#define DATAOBJECTRASTERDATA_H

#include "common/sharedStructures.h"
#include "DataObject/dataobj.h"

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

        enum ComplexType { cmplxAbs = 0, cmplxReal = 1, cmplxImag = 2, cmplxArg = 3 };
        enum Direction { dirX = 0, dirY = 1, dirZ = 2, dirXY = 3 };

        struct PtsWeighted {
            PtsWeighted()
            {
                xPx[0] = 0; xPx[1] = 0;
                yPx[0] = 0; yPx[1] = 0;
                weights[0] = 0; weights[1] = 0; weights[2] = 0; weights[3] = 0;
            }
            size_t xPx[2];
            size_t yPx[2];
            float weights[4]; // weights are for (xPx[0],yPx[0]),(xPx[1],yPx[0]),(xPx[0],yPx[1]),(xPx[1],yPx[1])
        };

        struct LineData {
            LineData() : nrPoints(0), startPhys(0.0), stepSizePhys(0.0), matOffset(0), matStepSize(1), valid(0) {}
            Direction dir;
            size_t nrPoints;
            float startPhys;
            float stepSizePhys; //xRight = xLeft + xStepSize * (m_numPts - 1)
            QPoint startPx;
            QSize stepSizePx;
            size_t matOffset; //in bytes
            size_t matStepSize; //in bytes
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

        QPointF sample(size_t n) const;
        QRectF boundingRect() const;
        RetVal getMinMaxLoc(double &min, double &max, int &minSampleIdx, int &maxSampleIdx) const;
        //QRectF boundingRectMax() const;

        size_t size() const;

        inline int isDobjInit() { return m_pDataObj != NULL; }

        bool floatingPointValues() const;

        RetVal updateDataObject(const ito::DataObject* dataObj, QVector<QPointF> bounds);

        inline void setCmplxState(ComplexType state) { m_cmplxState = state; }
        inline ComplexType getCmplxState() const {return m_cmplxState; }

        ito::DataObject getResampledDataObject();

        int getPosToPix(const double phys);
        
        QString getDObjValueLabel() const { return m_dObjValueLabel; }
        QString getDObjAxisLabel()  const { return m_dObjAxisLabel;  }

        void setIntervalRange(Qt::Axis axis, bool autoCalcLimits, double minValue, double maxValue);

        inline QByteArray getHash() const { return m_hash; }
        void calcHash();

    protected:

        inline void saturation(int &value, int min, int max) { value = ( value < min ? min : ( value > max ? max : value) ); }

    private:
        const ito::DataObject* m_pDataObj;               /*!< borrowed reference, do not delete here */

        bool inSamplingMode;

//        bool *m_pReplotPending;  /*!< pointer to m_refreshPending of plot2DImage (1 => update requested, 0 => repaint done or no update requested)*/

        int m_fast;

        QByteArray m_hash;

        //QVector<PtsIdxAndWeights> m_pointsXY;

        LineData m_d;

        QString m_dObjValueLabel;
        QString m_dObjAxisLabel;


        /*double m_startPos;
        double m_physLength;
        double m_Scaling;*/
        bool m_autoScaleY;
        double m_minY;
        double m_maxY;
        bool m_autoScaleX;
        double m_minX;
        double m_maxX;
		ComplexType m_cmplxState;

        /*double m_startPhys;
        double m_scale;*/
};

//----------------------------------------------------------------------------------------------------------------------------------

#endif
