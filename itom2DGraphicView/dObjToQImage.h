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

#ifndef RASTERTOQIMAGE_H
#define RASTERTOQIMAGE_H

//#include "../../Qitom/global.h"
#include "common/sharedStructures.h"
#include "DataObject/dataobj.h"

#include <qimage.h>
#include <qlist.h>
#include <qrect.h>
#include <qsharedpointer.h>
#include <qelapsedtimer.h>
#include <qcryptographichash.h>

//using namespace ito;

struct InternalData;

//----------------------------------------------------------------------------------------------------------------------------------
class RasterToQImageObj : QObject
{
    public:
        explicit RasterToQImageObj(InternalData *m_internalData);
        ~RasterToQImageObj();

        QImage convert2QImage();

        bool updateDataObject(const ito::DataObject *dataObj, int planeIdx = -1);

        inline ito::DataObject* getDataObject(void) { return m_dataObjPlane; };

        inline int getDataObjWidth() { return m_D.m_xSize; };
        inline int getDataObjHeight() { return m_D.m_ySize; };
        
        ito::float64 getPixel(const QPointF &coords, bool &isInt, const int &cmplxState);
        bool getPixelARGB(const QPointF &coords, unsigned char &AValue, unsigned char &RValue, unsigned char &GValue, unsigned char &BValue);
        
        enum ComplexType 
        { 
            tAbsolute = 0, 
            tImag = 1, 
            tReal = 2, 
            tPhase = 3 
        }; //definition like in dataObject: 0:abs-Value, 1:imaginary-Value, 2:real-Value, 3: argument-Value

        enum tValueType{
            ColorAutoSelect     = 0,
            ColorIndex8Scaled   = 1,
            ColorIndex8Bitshift = 2,
            ColorRGB24          = 3,
            ColorRGB32          = 4
        };

    protected:

    private:

        void deleteCache(void);
        QByteArray calcHash(const ito::DataObject *dObj);

        QByteArray m_hash;

        ito::DataObject m_dataObj; //the source data object (unchanged)
        ito::DataObject *m_dataObjPlane; //pointer to the source data object (<=2D) or a shallow copy to the depicted plane (>=3D)

        bool m_validData;
        QByteArray m_dataHash;
        QCryptographicHash m_hashGenerator;

        struct DataParam {
            DataParam() : m_dataPtr(NULL), m_planeIdx(0), m_yScaling(1), m_xScaling(1),
                m_yOffset(0), m_xOffset(0), m_ySize(0), m_xSize(0), m_yaxisFlipped(0), m_hasROI(false) {}

            int** m_dataPtr; //only for comparison
            size_t m_planeIdx;
            double m_yScaling;
            double m_xScaling;
            double m_yOffset;
            double m_xOffset;
            int m_ySize;
            int m_xSize;
            bool m_yaxisFlipped;
            bool m_hasROI;
        };

        DataParam m_D;
        cv::Mat *m_plane;
        InternalData *m_pInternalData;

        template<typename _Type> inline QImage rescaleByScale(const QVector<ito::uint32> &colorTable, const ito::float64 zMin, const ito::float64 zMax, const int cmplxState)
        {
            QImage retImage(m_D.m_xSize, m_D.m_ySize, QImage::Format_Indexed8);
            retImage.setColorTable(colorTable);

            unsigned char *dstPtr = retImage.bits();

            double scaling = 1.0;


            if(ito::dObjHelper::isNotZero(zMax-zMin))
            {
                scaling = 255.0 / (zMax - zMin);
            }

            _Type *srcPtr = m_plane->ptr<_Type>();
            size_t nextRow = (size_t)((m_plane->step[0] - m_D.m_xSize * m_plane->step[1]) / sizeof(_Type));
            
            size_t nextDstRow = m_D.m_xSize % 4;
            if(nextDstRow > 0) nextDstRow = 4 - nextDstRow;

            for(int y = 0; y < m_D.m_ySize; y++)
            {
                for(int x = 0; x < m_D.m_xSize; x++)
                {
                    *dstPtr = cv::saturate_cast<ito::uint8>((*srcPtr - zMin)*(scaling));
                    dstPtr++;
                    srcPtr++;
                }
                srcPtr += nextRow;
                dstPtr += nextDstRow;
            }


            return retImage;        
        }

        template<> inline QImage rescaleByScale<ito::complex64>(const QVector<ito::uint32> &colorTable, const ito::float64 zMin, const ito::float64 zMax, const int cmplxState)
        {
            QImage retImage(m_D.m_xSize, m_D.m_ySize, QImage::Format_Indexed8);
            retImage.setColorTable(colorTable);

            ito::complex64 *srcPtr = m_plane->ptr<ito::complex64>();
            unsigned char *dstPtr = retImage.bits();

            size_t nextRow = (size_t)((m_plane->step[0] - m_D.m_xSize * m_plane->step[1]) / sizeof(ito::complex64));
            
            size_t nextDstRow = m_D.m_xSize % 4;
            if(nextDstRow > 0) nextDstRow = 4 - nextDstRow;

            double scaling = 255.0 / (zMax - zMin);
            switch (cmplxState)
            {
                default:

                case tAbsolute:
                    for(int y = 0; y < m_D.m_ySize; y++)
                    {
                        for(int x = 0; x < m_D.m_xSize; x++)
                        {
                            *dstPtr = cv::saturate_cast<ito::uint8>((abs(*srcPtr) - zMin)*(scaling));
                            dstPtr++;
                            srcPtr++;
                        }
                        srcPtr += nextRow;
                        dstPtr += nextDstRow;
                    }
                break;
                case tReal:
                    for(int y = 0; y < m_D.m_ySize; y++)
                    {
                        for(int x = 0; x < m_D.m_xSize; x++)
                        {
                            *dstPtr = cv::saturate_cast<ito::uint8>((real(*srcPtr) - zMin)*(scaling));
                            dstPtr++;
                            srcPtr++;
                        }
                        srcPtr += nextRow;
                        dstPtr += nextDstRow;
                    }
                break;
                case tImag:
                    for(int y = 0; y < m_D.m_ySize; y++)
                    {
                        for(int x = 0; x < m_D.m_xSize; x++)
                        {
                            *dstPtr = cv::saturate_cast<ito::uint8>((imag(*srcPtr) - zMin)*(scaling));
                            dstPtr++;
                            srcPtr++;
                        }
                        srcPtr += nextRow;
                        dstPtr += nextDstRow;
                    }
                break;
                case tPhase:
                    for(int y = 0; y < m_D.m_ySize; y++)
                    {
                        for(int x = 0; x < m_D.m_xSize; x++)
                        {
                            *dstPtr = cv::saturate_cast<ito::uint8>((arg(*srcPtr) - zMin)*(scaling));
                            dstPtr++;
                            srcPtr++;
                        }
                        srcPtr += nextRow;
                        dstPtr += nextDstRow;
                    }
                break;
            }

            return retImage;        
        }
        
        template<> inline QImage rescaleByScale<ito::complex128>(const QVector<ito::uint32> &colorTable, const ito::float64 zMin, const ito::float64 zMax, const int cmplxState)
        {
            QImage retImage(m_D.m_xSize, m_D.m_ySize, QImage::Format_Indexed8);
            retImage.setColorTable(colorTable);

            ito::complex128 *srcPtr = m_plane->ptr<ito::complex128>();
            unsigned char *dstPtr = retImage.bits();

            size_t nextRow = (size_t)((m_plane->step[0] - m_D.m_xSize * m_plane->step[1]) / sizeof(ito::complex128));
            size_t nextDstRow = m_D.m_xSize % 4;
            if(nextDstRow > 0) nextDstRow = 4 - nextDstRow;

            double scaling = 255.0 / (zMax - zMin);
            switch (cmplxState)
            {
                default:
                case tAbsolute:
                    for(int y = 0; y < m_D.m_ySize; y++)
                    {
                        for(int x = 0; x < m_D.m_xSize; x++)
                        {
                            *dstPtr = cv::saturate_cast<ito::uint8>((abs(*srcPtr) - zMin)*(scaling));
                            dstPtr++;
                            srcPtr++;
                        }
                        srcPtr += nextRow;
                        dstPtr += nextDstRow;
                    }
                break;
                case tReal:
                    for(int y = 0; y < m_D.m_ySize; y++)
                    {
                        for(int x = 0; x < m_D.m_xSize; x++)
                        {
                            *dstPtr = cv::saturate_cast<ito::uint8>((real(*srcPtr) - zMin)*(scaling));
                            dstPtr++;
                            srcPtr++;
                        }
                        srcPtr += nextRow;
                        dstPtr += nextDstRow;
                    }
                break;
                case tImag:
                    for(int y = 0; y < m_D.m_ySize; y++)
                    {
                        for(int x = 0; x < m_D.m_xSize; x++)
                        {
                            *dstPtr = cv::saturate_cast<ito::uint8>((imag(*srcPtr) - zMin)*(scaling));
                            dstPtr++;
                            srcPtr++;
                        }
                        srcPtr += nextRow;
                        dstPtr += nextDstRow;
                    }
                break;
                case tPhase:
                    for(int y = 0; y < m_D.m_ySize; y++)
                    {
                        for(int x = 0; x < m_D.m_xSize; x++)
                        {
                            *dstPtr = cv::saturate_cast<ito::uint8>((arg(*srcPtr) - zMin)*(scaling));
                            dstPtr++;
                            srcPtr++;
                        }
                        srcPtr += nextRow;
                        dstPtr += nextDstRow;
                    }
                break;
            }
            return retImage;      
        }
};

//----------------------------------------------------------------------------------------------------------------------------------

#endif
