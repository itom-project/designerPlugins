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

//using namespace ito;

struct InternalData;

//----------------------------------------------------------------------------------------------------------------------------------
class RasterToQImageObj : QObject
{
    public:
        explicit RasterToQImageObj(QSharedPointer<ito::DataObject> dataObj, bool replotPending);
        ~RasterToQImageObj();

        QImage convert2QImage(const InternalData *pData);

        void updateDataObject(QSharedPointer<ito::DataObject> dataObj);

		inline QSharedPointer<ito::DataObject> getDataObject(void) { return m_dataObj; };

		inline int getDataObjWidth() { return m_DataObjectWidth; };
		inline int getDataObjHeight() { return m_DataObjectHeight; };
        
        ito::float64 getPixel(const QPointF &coords, bool &isInt, const int &cmplxState);
        bool getPixelARGB(const QPointF &coords, unsigned char &AValue, unsigned char &RValue, unsigned char &GValue, unsigned char &BValue);
        
        enum tValueType{
            ColorAutoSelect     = 0,
            ColorIndex8Scaled   = 1,
            ColorIndex8Bitshift = 2,
            ColorRGB24          = 3,
            ColorRGB32          = 4
        };

    protected:

    private:

        QSharedPointer<ito::DataObject> m_dataObj;               /*!< borrowed reference, do not delete here */
        QSharedPointer<ito::DataObject> m_dataObjWhileRastering;

        bool m_replotPending;

        int m_DataObjectWidth;
        int m_DataObjectHeight;

        template<typename _Type> inline QImage rescaleByScale(const int &ySize, const int &xSize, const QVector<ito::uint32> &colorTable, const ito::float64 zMin, const ito::float64 zMax, const int cmplxState)
        {
            QImage retImage(xSize, ySize, QImage::Format_Indexed8);
            retImage.setColorTable(colorTable);

            int pixelCnt = xSize * ySize;
            _Type *srcPtr = ((cv::Mat*)m_dataObjWhileRastering->get_mdata()[0])->ptr<_Type>();
            unsigned char *dstPtr = retImage.bits();

            double scaling = 1.0;

            if(ito::dObjHelper::isNotZero(zMax-zMin))
            {
                scaling = 255.0 / (zMax - zMin);
            }
            for(int i = 0; i < pixelCnt; i++)
            {
                *dstPtr = cv::saturate_cast<ito::uint8>((*srcPtr - zMin)*(scaling));
                dstPtr++;
                srcPtr++;
            }

            return retImage;        
        }

        template<> inline QImage rescaleByScale<ito::complex64>(const int &ySize, const int &xSize, const QVector<ito::uint32> &colorTable, const ito::float64 zMin, const ito::float64 zMax, const int cmplxState)
        {
            QImage retImage(xSize, ySize, QImage::Format_Indexed8);
            retImage.setColorTable(colorTable);

            int pixelCnt = xSize * ySize;
            ito::complex64 *srcPtr = ((cv::Mat*)m_dataObjWhileRastering->get_mdata()[0])->ptr<ito::complex64>();
            unsigned char *dstPtr = retImage.bits();

            double scaling = 255.0 / (zMax - zMin);
			switch (cmplxState)
			{
				default:

				case 0:
                    for(int i = 0; i < pixelCnt; i++)
                    {
                        *dstPtr = cv::saturate_cast<ito::uint8>((abs(*srcPtr) - zMin)*(scaling));
                        dstPtr++;
                        srcPtr++;
                    }
				break;
				case 2:
                    for(int i = 0; i < pixelCnt; i++)
                    {
                        *dstPtr = cv::saturate_cast<ito::uint8>(((*srcPtr).real() - zMin)*(scaling));
                        dstPtr++;
                        srcPtr++;
                    }
				break;
				case 1:
                    for(int i = 0; i < pixelCnt; i++)
                    {
                        *dstPtr = cv::saturate_cast<ito::uint8>(((*srcPtr).imag() - zMin)*(scaling));
                        dstPtr++;
                        srcPtr++;
                    }
				break;
				case 3:
                    for(int i = 0; i < pixelCnt; i++)
                    {
                        *dstPtr = cv::saturate_cast<ito::uint8>((arg(*srcPtr) - zMin)*(scaling));
                        dstPtr++;
                        srcPtr++;
                    }
				break;
			}

            return retImage;        
        }
        
        template<> inline QImage rescaleByScale<ito::complex128>(const int &ySize, const int &xSize, const QVector<ito::uint32> &colorTable, const ito::float64 zMin, const ito::float64 zMax, const int cmplxState)
        {
            QImage retImage(xSize, ySize, QImage::Format_Indexed8);
            retImage.setColorTable(colorTable);

            int pixelCnt = xSize * ySize;
            ito::complex128 *srcPtr = ((cv::Mat*)m_dataObjWhileRastering->get_mdata()[0])->ptr<ito::complex128>();
            unsigned char *dstPtr = retImage.bits();

            double scaling = 255.0 / (zMax - zMin);
			switch (cmplxState)
			{
				default:
				case 0:
                    for(int i = 0; i < pixelCnt; i++)
                    {
                        *dstPtr = cv::saturate_cast<ito::uint8>((abs(*srcPtr) - zMin)*(scaling));
                        dstPtr++;
                        srcPtr++;
                    }
				break;
				case 2:
                    for(int i = 0; i < pixelCnt; i++)
                    {
                        *dstPtr = cv::saturate_cast<ito::uint8>(((*srcPtr).real() - zMin)*(scaling));
                        dstPtr++;
                        srcPtr++;
                    }
				break;
				case 1:
                    for(int i = 0; i < pixelCnt; i++)
                    {
                        *dstPtr = cv::saturate_cast<ito::uint8>(((*srcPtr).imag() - zMin)*(scaling));
                        dstPtr++;
                        srcPtr++;
                    }
				break;
				case 3:
                    for(int i = 0; i < pixelCnt; i++)
                    {
                        *dstPtr = cv::saturate_cast<ito::uint8>((arg(*srcPtr) - zMin)*(scaling));
                        dstPtr++;
                        srcPtr++;
                    }
				break;
			}

            return retImage;        
        }
};

//----------------------------------------------------------------------------------------------------------------------------------

#endif
