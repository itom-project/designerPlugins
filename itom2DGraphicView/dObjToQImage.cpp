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

#include "dObjToQImage.h"
#include "plotWidget.h"

#include "common/typeDefs.h"

#include "DataObject/dataObjectFuncs.h"

#include <qdebug.h>

#define FAST_VALUES // Warning other interpolationmode does not tested with offsets at the moment

using namespace ito;

//----------------------------------------------------------------------------------------------------------------------------------
RasterToQImageObj::RasterToQImageObj(QSharedPointer<ito::DataObject> dataObj, bool replotPending) :
m_DataObjectWidth(0), 
m_DataObjectHeight(0), 
m_replotPending(replotPending)
{
    updateDataObject(dataObj);
}

//----------------------------------------------------------------------------------------------------------------------------------
RasterToQImageObj::~RasterToQImageObj()
{
    m_dataObjWhileRastering.clear();
    m_dataObj.clear();
}

//----------------------------------------------------------------------------------------------------------------------------------
void RasterToQImageObj::updateDataObject(QSharedPointer<ito::DataObject> dataObj)
{
    m_dataObj = dataObj;

    if(m_dataObj)
    {
        m_dataObj->lockRead();

        int dimX = dataObj->getDims() - 1;
        int dimY = dataObj->getDims() - 2;

        m_DataObjectWidth = dataObj->getSize(dimX); 
        m_DataObjectHeight = dataObj->getSize(dimY);

        m_dataObj->unlock();

    }
    else
    {
        m_DataObjectWidth = 0;
        m_DataObjectHeight = 0;
    }
}

QImage RasterToQImageObj::convert2QImage(const InternalData *pData)
{
    if(!m_dataObj)
    {
        QImage temp(100, 100, QImage::Format_ARGB32);
        temp.setText("", "No Image Data");
        return temp;    
    }

    int type = m_dataObj->getType();

    m_dataObjWhileRastering = m_dataObj;

    int ySize = m_dataObjWhileRastering->getSize(m_dataObjWhileRastering->getDims() - 2);
    int xSize = m_dataObjWhileRastering->getSize(m_dataObjWhileRastering->getDims() - 1);

    if(pData->m_colorMode == ColorIndex8Bitshift)
    {
        switch(type)
        {
            case ito::tUInt8:
            {
                // this is a 8bit image which shall be scaled to 8bit -> just copy the image
                QImage retImage(((cv::Mat*)m_dataObjWhileRastering->get_mdata()[0])->ptr<ito::uint8>(), xSize, ySize, xSize, QImage::Format_Indexed8);
                retImage.setColorTable(pData->m_colorTable);
                return retImage;
            }
            case ito::tUInt16:
            {
                QImage retImage(xSize, ySize, QImage::Format_Indexed8);
                retImage.setColorTable(pData->m_colorTable);

                int pixelCnt = xSize * ySize;
                ito::uint16 *srcPtr = ((cv::Mat*)m_dataObjWhileRastering->get_mdata()[0])->ptr<ito::uint16>();
                ito::uint8 *dstPtr = retImage.bits();

                unsigned char bitShift = pData->m_numBits > 8 ? (pData->m_numBits - 8) : 0;
                bitShift = bitShift > 8 ? 8 : bitShift;
            
                for(int i = 0; i < pixelCnt; i++)
                {
                    *dstPtr = cv::saturate_cast<ito::uint8>(*srcPtr >> bitShift);
                    dstPtr++;
                    srcPtr++;    
                }
            
                return retImage;
            }
            default:
            {
                QImage temp(100, 100, QImage::Format_ARGB32);
                temp.setText("", "Wrong image type");
                return temp;         
            }

        }    
    }
    else if(pData->m_colorMode == ColorRGB24)
    {
        if (type != ito::tUInt32 && type != ito::tInt32 && type != ito::tRGBA32)
        {
            QImage temp(100, 100, QImage::Format_ARGB32);
            temp.setText("", "Wrong image type");
            return temp;      
        }
        return QImage(((cv::Mat*)m_dataObjWhileRastering->get_mdata()[0])->ptr(), m_dataObjWhileRastering->getSize(m_dataObjWhileRastering->getDims() - 1), m_dataObjWhileRastering->getSize(m_dataObjWhileRastering->getDims() - 2), m_dataObjWhileRastering->getSize(m_dataObjWhileRastering->getDims() - 1)*4 , QImage::Format_RGB32);
    }
    else if(pData->m_colorMode == ColorRGB32)
    {
        if (type != ito::tUInt32 && type != ito::tInt32 && type != ito::tRGBA32)
        {
            QImage temp(100, 100, QImage::Format_ARGB32);
            temp.setText("", "Wrong image type");
            return temp;      
        }
        return QImage(((cv::Mat*)m_dataObjWhileRastering->get_mdata()[0])->ptr(), m_dataObjWhileRastering->getSize(m_dataObjWhileRastering->getDims() - 1), m_dataObjWhileRastering->getSize(m_dataObjWhileRastering->getDims() - 2), m_dataObjWhileRastering->getSize(m_dataObjWhileRastering->getDims() - 1)*4 , QImage::Format_ARGB32);
    }
    else
    {
        ito::float64 zMin = pData->m_valueMin;
        ito::float64 zMax = pData->m_valueMax;
        ito::uint32 locMin[3], locMax[3];
        if(pData->m_valueScaleAuto)
        {
            ito::dObjHelper::minMaxValue(m_dataObjWhileRastering.data(), zMin, locMin, zMax, locMax, true, pData->m_cmplxType);
        }

        switch(m_dataObjWhileRastering->getType())
        {
            case ito::tUInt8:
                return rescaleByScale<ito::uint8>(ySize, xSize, pData->m_colorTable, zMin, zMax, 0);
            case ito::tInt8:
                return rescaleByScale<ito::int8>(ySize, xSize, pData->m_colorTable, zMin, zMax, 0);
            case ito::tUInt16:
                return rescaleByScale<ito::uint16>(ySize, xSize, pData->m_colorTable, zMin, zMax, 0);
            case ito::tInt16:
                return rescaleByScale<ito::int16>(ySize, xSize, pData->m_colorTable, zMin, zMax, 0);
            case ito::tUInt32:
                return rescaleByScale<ito::uint32>(ySize, xSize, pData->m_colorTable, zMin, zMax, 0);
            case ito::tInt32:
                return rescaleByScale<ito::int32>(ySize, xSize, pData->m_colorTable, zMin, zMax, 0);
            case ito::tFloat32:
                return rescaleByScale<ito::float32>(ySize, xSize, pData->m_colorTable, zMin, zMax, 0);
            case ito::tFloat64:
                return rescaleByScale<ito::float64>(ySize, xSize, pData->m_colorTable, zMin, zMax, 0);
            case ito::tComplex64:
                return rescaleByScale<ito::complex64>(ySize, xSize, pData->m_colorTable, zMin, zMax, 0);
            case ito::tComplex128:
                return rescaleByScale<ito::complex128>(ySize, xSize, pData->m_colorTable, zMin, zMax, 0);
            default:
            {
                QImage temp(100, 100, QImage::Format_ARGB32);
                temp.setText("", "Wrong image type");
                return temp;         
            }

        }
    }
    return QImage();
}
//----------------------------------------------------------------------------------------------------------------------------------------------
ito::float64 RasterToQImageObj::getPixel(const QPointF &coords, bool &isInt, const int &cmplxState)
{
    if(!m_dataObj || m_dataObj->getDims() < 2)
        return -666.0;

    ito::float64 ret = -666.0;

    m_dataObj->lockWrite();

    unsigned int *index = new unsigned int[m_dataObj->getDims()];

    memset(index, 0, sizeof(unsigned int)*m_dataObj->getDims());
    
    index[m_dataObj->getDims()-1] = (int)coords.x();
    index[m_dataObj->getDims()-2] = (int)coords.y();

    switch(m_dataObj->getType())
    {
        case ito::tInt8:
            ret = (ito::float64)m_dataObj->at<ito::int8>(index);
            isInt = true;
            break;
        case ito::tInt16:
            ret = (ito::float64)m_dataObj->at<ito::int16>(index);
            isInt = true;
            break;
        case ito::tInt32:
            ret = (ito::float64)m_dataObj->at<ito::int32>(index);
            isInt = true;
            break;
        case ito::tUInt8:
            ret = (ito::float64)m_dataObj->at<ito::uint8>(index);
            isInt = true;
            break;
        case ito::tUInt16:
            ret = (ito::float64)m_dataObj->at<ito::uint16>(index);
            isInt = true;
            break;
        case ito::tUInt32:
            ret = (ito::float64)m_dataObj->at<ito::uint32>(index);
            isInt = true;
            break;
        case ito::tFloat32:
            ret = (ito::float64)m_dataObj->at<ito::float32>(index);
            isInt = false;
            break;
        case ito::tFloat64:
            ret = (ito::float64)m_dataObj->at<ito::float64>(index);
            isInt = false;
            break;
        case ito::tComplex64:
        {
            ito::complex64 retTemp = (ito::complex64)m_dataObj->at<ito::complex64>(index);
			switch (cmplxState)
			{
				default:
                case PlotWidget::tAbsolute:
					ret = abs(retTemp);
				break;
				case PlotWidget::tReal:
					ret = retTemp.real();
				break;
				case PlotWidget::tImag:
					ret = retTemp.imag();
				break;
				case PlotWidget::tPhase:
					ret = arg(retTemp);
				break;
			}
            isInt = false;
            break;
        }
        case ito::tComplex128:
        {
            ito::complex128 retTemp = (ito::complex128)m_dataObj->at<ito::complex128>(index);
			switch (cmplxState)
			{
				default:
                case PlotWidget::tAbsolute:
					ret = abs(retTemp);
				break;
				case PlotWidget::tReal:
					ret = retTemp.real();
				break;
				case PlotWidget::tImag:
					ret = retTemp.imag();
				break;
				case PlotWidget::tPhase:
					ret = arg(retTemp);
				break;
			}
            isInt = false;
            break;
        }
    };

    delete index;

    m_dataObj->unlock();

    return ret; 
}
//----------------------------------------------------------------------------------------------------------------------------------------------
bool RasterToQImageObj::getPixelARGB(const QPointF &coords, unsigned char &AValue, unsigned char &RValue, unsigned char &GValue, unsigned char &BValue)
{
    if(!m_dataObj || m_dataObj->getDims() < 2)
        return false;

    m_dataObj->lockWrite();

    bool ret = false;

    unsigned int *index = new unsigned int[m_dataObj->getDims()];
    memset(index, 0, sizeof(unsigned int)*m_dataObj->getDims());
    
    index[m_dataObj->getDims()-1] = (int)coords.x();
    index[m_dataObj->getDims()-2] = (int)coords.y();

    ito::uint32 val = 0;

    switch(m_dataObj->getType())
    {
        //case ito::tInt16:
        //    ret = (ito::float64)m_dataObj->at<ito::int16>(index);
        //    break;
        case ito::tInt32:
            val = (ito::uint32)m_dataObj->at<ito::int32>(index);
            ret = true;
            break;
        //case ito::tUInt16:
        //    ret = (ito::float64)m_dataObj->at<ito::uint16>(index);
        //    break;
        case ito::tUInt32:
            val = (ito::uint32)m_dataObj->at<ito::uint32>(index);
            ret = true;
            break;
        case ito::tRGBA32:
            val = m_dataObj->at<ito::Rgba32>(index).argb();
            ret = true;
            break;
    };

    AValue = (unsigned char)((val & 0xFF000000) >> 24); 
    RValue = (unsigned char)((val & 0x00FF0000) >> 16);
    GValue = (unsigned char)((val & 0x0000FF00) >> 8);
    BValue = (unsigned char)(val & 0x000000FF);

    delete index;

    m_dataObj->unlock();

    return ret; 
}