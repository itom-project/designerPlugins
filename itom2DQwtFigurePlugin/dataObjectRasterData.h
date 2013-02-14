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

//#include "../../Qitom/global.h"
#include "common/sharedStructures.h"
#include "DataObject/dataobj.h"

#include <qwt_raster_data.h>
#include <qlist.h>
#include <qrect.h>
#include <qsharedpointer.h>

//using namespace ito;

//----------------------------------------------------------------------------------------------------------------------------------
//template <class T> double bilinearInterpolation(ito::DataObject* data, double x, double y,  int width, int height, double xScalingFactor, double yScalingFactor);
template <class T> double bilinearInterpolation(cv::Mat* data, double x, double y, int width, int height, double xScalingFactor, double xOffset, double yScalingFactor, double yOffset, int cmplxState = -1);

//----------------------------------------------------------------------------------------------------------------------------------
class DataObjectRasterData : public QwtRasterData
{
    public:
        explicit DataObjectRasterData(QSharedPointer<ito::DataObject> dataObj, QList<unsigned int>startPoint, unsigned int wDimIndex, unsigned int width, unsigned int hDimIndex, unsigned int height, bool replotPending);
        ~DataObjectRasterData();

        double value(double x, double y) const;
        void initRaster( const QRectF &, const QSize& raster );
        void discardRaster();

        void updateDataObject(QSharedPointer<ito::DataObject> dataObj);
        void updateDataObject(QSharedPointer<ito::DataObject> dataObj, QList<unsigned int>startPoint, unsigned int wDimIndex, unsigned int width, unsigned int hDimIndex, unsigned int height);
        inline QSharedPointer<ito::DataObject> getDataObject(void) { return m_dataObj; }

        void setIntervalRange(Qt::Axis axis, bool autoCalcLimits, double minValue, double maxValue);
        inline int getDataObjWidth() { return m_DataObjectWidth; }
        inline int getDataObjHeight() { return m_DataObjectHeight; }
        inline void setCmplxState(const int state) { m_cmplxState = state; }
       
    protected:

    private:
        QSharedPointer<ito::DataObject> m_dataObj;               /*!< borrowed reference, do not delete here */
        QSharedPointer<ito::DataObject> m_dataObjWhileRastering;

        //bool *m_pReplotPending;  /*!< pointer to m_refreshPending of plot2DImage (1 => update requested, 0 => repaint done or no update requested)*/
        bool m_replotPending;

        QList<unsigned int>m_startPoint;
        unsigned int m_wDimIndex;
        unsigned int m_hDimIndex;
        unsigned int m_width;
        unsigned int m_height;
        int m_DataObjectWidth;
        int m_DataObjectHeight;
        double m_xScalingFactor;
        double m_yScalingFactor;
        double m_xOffset;
        double m_yOffset;
		int m_cmplxState;
};

//----------------------------------------------------------------------------------------------------------------------------------

#endif
