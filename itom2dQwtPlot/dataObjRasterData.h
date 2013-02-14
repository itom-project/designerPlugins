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

#ifndef DATAOBJRASTERDATA_H
#define DATAOBJRASTERDATA_H

//#include "../../Qitom/global.h"
#include "../../common/sharedStructures.h"
#include "../../DataObject/dataobj.h"

#include <qwt_raster_data.h>
#include <qlist.h>
#include <qrect.h>
#include <qsharedpointer.h>
#include <qcryptographichash.h>

//----------------------------------------------------------------------------------------------------------------------------------
class DataObjRasterData : public QwtRasterData
{
    public:
        explicit DataObjRasterData();
        explicit DataObjRasterData(QSharedPointer<ito::DataObject> dataObj, QList<unsigned int>startPoint, unsigned int wDimIndex, unsigned int width, unsigned int hDimIndex, unsigned int height, bool replotPending);
        ~DataObjRasterData();

        double value2(int m, int n) const;
        double value(double x, double y) const;
        void initRaster( const QRectF& area, const QSize& raster );
        void discardRaster();

        inline QSize getSize() const { return QSize(m_D.m_xSize, m_D.m_ySize); }


        void updateDataObject(ito::DataObject *dataObj, size_t planeIdx = 0);

        /*void updateDataObject(QSharedPointer<ito::DataObject> dataObj);
        void updateDataObject(QSharedPointer<ito::DataObject> dataObj, QList<unsigned int>startPoint, unsigned int wDimIndex, unsigned int width, unsigned int hDimIndex, unsigned int height);
        inline QSharedPointer<ito::DataObject> getDataObject(void) { return m_dataObj; }

        void setIntervalRange(Qt::Axis axis, bool autoCalcLimits, double minValue, double maxValue);
        inline int getDataObjWidth() { return m_DataObjectWidth; }
        inline int getDataObjHeight() { return m_DataObjectHeight; }
        inline void setCmplxState(const int state) { m_cmplxState = state; }*/
       
    protected:
        //Definition: Scale-Coordinate of dataObject =  ( px-Coordinate - Offset)* Scale
        inline double pxToScaleCoords(double px, double offset, double scaling) { return ((double)px - offset) * scaling; }
        inline double scaleToPxCoords(double coord, double offset, double scaling) { return (coord / scaling) + offset; }

    private:

        void deleteCache();

        ito::DataObject *m_dataObj; //is pointer only

        bool m_validData;
        QByteArray m_dataHash;
        QCryptographicHash m_hashGenerator;
        QRectF m_lastRasteredArea;
        QSize m_lastRasteredRaster;

        struct DataParam {
            int** m_dataPtr; //only for comparison
            size_t m_planeIdx;
            double m_yScaling;
            double m_xScaling;
            double m_yOffset;
            double m_xOffset;
            int m_ySize;
            int m_xSize;
        };

        DataParam m_D;

        cv::Mat *m_plane;
        uchar **m_rasteredLinePtr;
        int *m_xIndizes;

        /*QList<unsigned int>m_startPoint;
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
		int m_cmplxState;*/
};

//----------------------------------------------------------------------------------------------------------------------------------

#endif
