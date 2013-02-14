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

using namespace ito;

//----------------------------------------------------------------------------------------------------------------------------------
class DataObjectSeriesData : public QwtSeriesData<QPointF>
{
    public:
        struct ptsAndWeights {
            int32 rangeX[2];
            int32 rangeY[2];
            QPointF pt1;
            double weights[4];

            ptsAndWeights() { memset(rangeX, 0, 2*sizeof(int32)); memset(rangeY, 0, 2*sizeof(int32)); memset(weights, 0, 4*sizeof(double)); }
        };

        explicit DataObjectSeriesData(QSharedPointer<ito::DataObject> dataObj, int startPoint, unsigned int plotDim, unsigned int size, const int fastmode);
        explicit DataObjectSeriesData(QSharedPointer<ito::DataObject> dataObj, QVector<QPointF> pts, const int fastmode);
        ~DataObjectSeriesData();

        QPointF sample(size_t n) const;
        QRectF boundingRect() const;
        QRectF boundingRectMax() const;

        size_t size() const;

        inline int isDobjInit() { return m_dataObj != NULL; }
        void updateDataObject(QSharedPointer<ito::DataObject> dataObj);
        void updateDataObject(QVector<QPointF> pts);
        void updateDataObject(QSharedPointer<ito::DataObject> dataObj, QVector<QPointF> pts);
        void updateDataObject(QSharedPointer<ito::DataObject> dataObj, int startPoint, unsigned int plotDim, unsigned int size);

        inline void setCmplxState(const int state) { m_cmplxState = state; };
        inline int getCmplxState(){return m_cmplxState;};

        inline QSharedPointer<ito::DataObject> getDataObj(void) { return m_dataObj; }

        ito::DataObject getResampledDataObject();

        int getPosToPix(const double phys);
        

        inline void setRasterObj()
        {
            if (m_dataObj)
            {
                m_dataObjWhileRastering = m_dataObj;
                m_dataObjWhileRastering->lockRead();
            }
        }
        inline void releaseRasterObj()
        {
            if (m_dataObjWhileRastering)
            {
                m_dataObjWhileRastering->unlock();
                m_dataObjWhileRastering.clear();
//                if (m_pReplotPending)
//                    *m_pReplotPending = false;
            }
        }
        
        void setIntervalRange(Qt::Axis axis, bool autoCalcLimits, double minValue, double maxValue);

    protected:

    private:
        QSharedPointer<ito::DataObject> m_dataObj;               /*!< borrowed reference, do not delete here */
        QSharedPointer<ito::DataObject> m_dataObjWhileRastering;

//        bool *m_pReplotPending;  /*!< pointer to m_refreshPending of plot2DImage (1 => update requested, 0 => repaint done or no update requested)*/

        int m_startPoint;
        bool m_zDirect;
        unsigned int m_size;
        int m_fast;
        QVector<QPointF> m_points;
        QVector<ptsAndWeights> m_plotPts;
        double m_startPos;
        double m_physLength;
        double m_Scaling;
        bool m_autoScaleY;
        double m_minY;
        double m_maxY;
        bool m_autoScaleX;
        double m_minX;
        double m_maxX;
		int m_cmplxState;
};

//----------------------------------------------------------------------------------------------------------------------------------

#endif
