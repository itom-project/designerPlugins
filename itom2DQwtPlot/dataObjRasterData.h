/* ********************************************************************
   itom measurement system
   URL: http://www.uni-stuttgart.de/ito
   Copyright (C) 2012, Institut f�r Technische Optik (ITO), 
   Universit�t Stuttgart, Germany 
 
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

#include "common/sharedStructures.h"
#include "DataObject/dataobj.h"
#include "plotCanvas.h"

#include <qwt_raster_data.h>
#include <qlist.h>
#include <qrect.h>
#include <qsharedpointer.h>
#include <qcryptographichash.h>

struct InternalData;

//----------------------------------------------------------------------------------------------------------------------------------
class DataObjRasterData : public QwtRasterData
{
    public:
        explicit DataObjRasterData(const InternalData *m_internalData, const bool overlay = true);
        explicit DataObjRasterData(QSharedPointer<ito::DataObject> dataObj, QList<unsigned int>startPoint, unsigned int wDimIndex, unsigned int width, unsigned int hDimIndex, unsigned int height, bool replotPending, const bool overlay = true);
        ~DataObjRasterData();

        double value2(int m, int n) const;
        double value2_yinv(int m, int n) const;
        QRgb value2_rgb(int m, int n) const;
        QRgb value2_yinv_rgb(int m, int n) const;
        double value(double x, double y) const;
        QRgb value_rgb(double x, double y) const;
        void initRaster( const QRectF& area, const QSize& raster );
        void discardRaster();

        inline QSize getSize() const { return QSize(m_D.m_xSize, m_D.m_ySize); }

        void calcHash(const ito::DataObject *dObj, QByteArray &dataHash, QByteArray &appearanceHash);

        ito::uint8 updateDataObject(const ito::DataObject *dataObj, int planeIdx = -1);

        bool pointValid(const QPointF &point) const;
       
        ito::int32 getCurrentPlane() const {return (ito::int32)m_D.m_planeIdx;}
        QSharedPointer<ito::DataObject> rasterToObject(void);
        QSharedPointer<ito::DataObject> rasterToObject(const QwtInterval &xInterval, const QwtInterval &yInterval);

        bool isInit() const {return m_dataObj.getDims() > 0 && m_dataObjPlane != NULL;}

        char getTypeFlag() const 
        { 
            int type = m_dataObj.getType();
            if(type == ito::tRGBA32) return tRGB;
            else if (type == ito::tFloat32 || type == ito::tFloat64 || type == ito::tComplex64 || type == ito::tComplex128) return tFloating;
            return tIntegerOrFloat; 
        }

        void getMinMaxLoc(double &min, ito::uint32 *minLoc, double &max, ito::uint32 *maxLoc);

        enum tTypeFlag
        {
            tIntegerOrFloat  = 0,  // Object is an integer or floating value
            tFloating        = 1,  // Object is floating point or complex value
            tRGB             = 2   // Object is true color type
        };

    protected:
        //Definition: Scale-Coordinate of dataObject =  ( px-Coordinate - Offset)* Scale
        inline double pxToScaleCoords(double px, double offset, double scaling) { return ((double)px - offset) * scaling; }
        inline double scaleToPxCoords(double coord, double offset, double scaling) { return (coord / scaling) + offset; }

    private:
        static double quietNaN;
        static QRgb transparentColor;

        void deleteCache();

        QByteArray m_dataHash;
        QByteArray m_appearanceHash;

        ito::DataObject m_dataObj; //the source data object (unchanged)
        ito::DataObject *m_dataObjPlane; //pointer to the source data object (<=2D) or a shallow copy to the depicted plane (>=3D)

        bool m_validData;
        
        QCryptographicHash m_hashGenerator;
        QRectF m_lastRasteredArea;
        QSize m_lastRasteredRaster;

        QRectF m_xyBounds;
        QPointF m_zBounds;

        struct DataParam {
            DataParam() : m_dataPtr(NULL), m_planeIdx(0), m_yScaling(1), m_xScaling(1),
                m_yOffset(0), m_xOffset(0), m_ySize(0), m_xSize(0), m_yaxisFlipped(0) {}

            int** m_dataPtr; //only for comparison
            size_t m_planeIdx;
            double m_yScaling;
            double m_xScaling;
            double m_yOffset;
            double m_xOffset;
            int m_ySize;
            int m_xSize;
            bool m_yaxisFlipped;
        };

        DataParam m_D;

        cv::Mat *m_plane;
        uchar **m_rasteredLinePtr;
        int m_rasteredLines;
        int *m_xIndizes;

        QElapsedTimer timer1, timer2;
        unsigned int nrOfRendering;

        const InternalData *m_pInternalData;

        bool m_notOverlay;
};

//----------------------------------------------------------------------------------------------------------------------------------

#endif
