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

#include "rasterToQImage.h"
#include "common/typeDefs.h"

#include "DataObject/dataObjectFuncs.h"

#include <qdebug.h>

#define FAST_VALUES // Warning other interpolationmode does not tested with offsets at the moment

using namespace ito;

//----------------------------------------------------------------------------------------------------------------------------------
RasterToQImageObj::RasterToQImageObj(QSharedPointer<ito::DataObject> dataObj, const QRect ROI, /*const unsigned int XDimIndex, const unsigned int YDimIndex,*/ bool replotPending) :
//m_wDimIndex(XDimIndex), m_hDimIndex(YDimIndex),
m_DataObjectWidth(0), m_DataObjectHeight(0), 
m_cmplxState(0), 
m_colorMode(0),
m_numBits(0),
m_replotPending(replotPending)
{

    m_colorTable.clear();
    m_colorTable.resize(256);
    for(unsigned int i = 0; i < 256; i++)
    {
        m_colorTable[i] = qRgb(i, i, i);
    }

    updateDataObject(dataObj, ROI /*, XDimIndex, YDimIndex*/);

    if(dataObj.data() != NULL)
    {
        setIntervalRange(Qt::ZAxis, true, 0,0);
    }
    else
    {
        setIntervalRange(Qt::ZAxis, false, 0,255);
    }
}

//----------------------------------------------------------------------------------------------------------------------------------
RasterToQImageObj::~RasterToQImageObj()
{
    m_dataObjWhileRastering.clear();
    m_dataObj.clear();
}

//----------------------------------------------------------------------------------------------------------------------------------
//void DataObjectRasterData::initRaster( const QRectF &, const QSize& /*raster*/ )
//{
//    if(m_dataObj)
//    {
//        //qDebug("raster data: init");
//        m_dataObjWhileRastering = m_dataObj;
//        m_dataObjWhileRastering->lockRead();
//    }
//}

//----------------------------------------------------------------------------------------------------------------------------------
//void DataObjectRasterData::discardRaster()
//{
//    if(m_dataObjWhileRastering)
//    {
//        m_dataObjWhileRastering->unlock();
//        m_dataObjWhileRastering.clear();
//        //qDebug("raster data: end");
//        //if(m_pReplotPending) *m_pReplotPending = false;
//    }
//}

void RasterToQImageObj::setColorMode(const unsigned char colorMode)
{
    if(m_dataObj)
    {
        switch(m_dataObj->getType())
        {
            case ito::tInt8:
            case ito::tUInt8:
            case ito::tInt16:
            case ito::tUInt16:
                if(colorMode == ColorIndex8Bitshift || colorMode == ColorIndex8Scaled)
                {
                    m_colorMode = colorMode;
                }
                else
                {
                    m_colorMode = ColorIndex8Scaled;
                }
                break;
            case ito::tUInt32:
            case ito::tInt32:
                m_colorMode = colorMode;
                break;
            default:
                m_colorMode = ColorIndex8Scaled;
                break;
        }
    }
    return;
}

//----------------------------------------------------------------------------------------------------------------------------------
void RasterToQImageObj::setIntervalRange(Qt::Axis axis, bool autoCalcLimits, double minValue, double maxValue)
{
    if(axis == Qt::ZAxis)
    {
        if(autoCalcLimits)
        {
            if(m_dataObj)
            {
                m_dataObj->lockRead();
                uint32 minPos[3] = {0, 0, 0};
                uint32 maxPos[3] = {0, 0, 0};
                ito::dObjHelper::minMaxValue(m_dataObj.data(), m_minZValue, minPos, m_maxZValue, maxPos, true, m_cmplxState);
				//m_dataObj->getMinMaxValue(minValue, maxValue, true, m_cmplxState);
                m_dataObj->unlock();
            }
        }
        else
        {
            if(minValue < maxValue)
            {
                m_minZValue = minValue;
                m_maxZValue = maxValue;
            }
            else
            {
                m_minZValue = maxValue;
                m_maxZValue = minValue;            
            }

        }
        ito::uint32 maxInt = cv::saturate_cast<ito::uint32>(m_maxZValue - m_minZValue);
        m_numBits = 0;
        do
        {
            m_numBits++;
        } while((maxInt = maxInt >> 1) && (m_numBits < 32));
    }
    else if(axis == Qt::XAxis)
    {
        if(autoCalcLimits)
        {
            if(m_dataObj)
            {
                if(m_hDimIndex >= 0)
                {
                    m_dataObj->lockRead();
                    m_ROI.setLeft(0);
                    m_ROI.setWidth(m_dataObj->getSize(m_hDimIndex));
                    m_dataObj->unlock();
                }
            }
        }
        else
        {
            if(minValue > maxValue)
            {
                if(maxValue < 0.0)
                {
                    m_ROI.setLeft(0);
                }
                else
                {
                    m_ROI.setLeft(maxValue + 0.5);
                }  
                if(minValue < 0.0)
                {
                    m_ROI.setRight(0);
                }
                else
                {
                    m_ROI.setRight(minValue + 0.5);
                }  
            }
            else
            {
                if(minValue < 0.0)
                {
                    m_ROI.setLeft(0);
                }
                else
                {
                    m_ROI.setLeft(minValue + 0.5);
                }  
                if(maxValue < 0.0)
                {
                    m_ROI.setRight(0);
                }
                else
                {
                    m_ROI.setRight(maxValue + 0.5);
                }             
            }
        }
    }
    else if(axis == Qt::YAxis)
    {
        if(autoCalcLimits)
        {
            if(m_dataObj)
            {
                /*
                if(m_wDimIndex >= 0)
                {
                    m_dataObj->lockRead();
                    m_ROI.setBottom(0);
                    m_ROI.setHeight(m_dataObj->getSize(m_wDimIndex, true));
                    m_dataObj->unlock();
                }
                */
            }
        }
        else
        {
            if(minValue > maxValue)
            {
                if(maxValue < 0.0)
                {
                    m_ROI.setBottom(0);
                }
                else
                {
                    m_ROI.setBottom(maxValue + 0.5);
                }  
                if(minValue < 0.0)
                {
                    m_ROI.setHeight(0);
                }
                else
                {
                    m_ROI.setHeight(minValue + 0.5);
                }  
            }
            else
            {
                if(minValue < 0.0)
                {
                    m_ROI.setBottom(0);
                }
                else
                {
                    m_ROI.setBottom(minValue + 0.5);
                }  
                if(maxValue < 0.0)
                {
                    m_ROI.setHeight(0);
                }
                else
                {
                    m_ROI.setHeight(maxValue + 0.5);
                }             
            }
        }
    }
}

//----------------------------------------------------------------------------------------------------------------------------------
void RasterToQImageObj::setColorTable(const QVector<ito::uint32> newColorTable)
{
    if(newColorTable.size() > 255) 
    {
        for(int i = 0; i < 256; i++)
            m_colorTable[i] = newColorTable[i] + 0xFF000000;
    }
}
//----------------------------------------------------------------------------------------------------------------------------------
void RasterToQImageObj::updateDataObject(QSharedPointer<ito::DataObject> dataObj)
{
    m_dataObj = dataObj;

    if(m_dataObj)
    {
        m_dataObj->lockRead();

        bool test = true;

        int dimX = dataObj->getDims() - 1;
        int dimY = dataObj->getDims() - 2;

        /*
        if(m_wDimIndex < dataObj->getDims() && m_wDimIndex != m_hDimIndex)
        {
            dimX = m_wDimIndex;
        }
        else
        {
            m_wDimIndex = dimX;
        }

        if(m_hDimIndex < dataObj->getDims() && m_wDimIndex != m_hDimIndex)
        {
            dimY = m_hDimIndex;
        }
        else
        {
            if(m_wDimIndex == dimY)
            {
                m_hDimIndex = dimX;
            }
            else
            {
                m_hDimIndex = dimY;
            }
        }
        */
        m_DataObjectWidth = dataObj->getSize(dimX); //ly hier bugged was
        m_DataObjectHeight = dataObj->getSize(dimY);//ly hier bugged was
        
        m_ROI.setCoords(0,0, m_DataObjectWidth-1, m_DataObjectHeight-1);
        m_physROI.setCoords(0.0, 1.0, 0.0, 1.0);

        m_dataObj->unlock();

    }
    else
    {
        m_DataObjectWidth = 0;
        m_DataObjectHeight = 0;
    }
}

//----------------------------------------------------------------------------------------------------------------------------------
void RasterToQImageObj::updateDataObject(QSharedPointer<ito::DataObject> dataObj, const QRect ROI /*, const unsigned int XDimIndex, const unsigned int YDimIndex*/)
{
    //m_wDimIndex = XDimIndex;
    //m_hDimIndex = YDimIndex;

    updateDataObject(dataObj);

    m_ROI = ROI;
}

//----------------------------------------------------------------------------------------------------------------------------------
QImage RasterToQImageObj::getRastersImage()
{
    //qDebug() << "framerate: " << (1000.0) / ((float)timer2.elapsed());
    //timer2.restart();

    if(m_dataObj)
    {
        //qDebug("raster data: init");
        m_dataObjWhileRastering = m_dataObj;
        //m_dataObjWhileRastering->lockRead();
    }
    else
    {
        QImage temp(100, 100, QImage::Format_ARGB32);
        temp.setText("", "No Image Data");
        return temp;
    }


    switch(m_dataObjWhileRastering->getType())
    {
        case ito::tUInt8:
        {
            // check if it is scale according to bitdepth
            if(m_colorMode == ColorIndex8Scaled)
            {
                QImage retImage(m_dataObjWhileRastering->getSize(m_dataObjWhileRastering->getDims() - 1), m_dataObjWhileRastering->getSize(m_dataObjWhileRastering->getDims() - 2), QImage::Format_Indexed8);
                retImage.setColorTable(m_colorTable);

                int pixelCnt = m_dataObjWhileRastering->getSize(m_dataObjWhileRastering->getDims() - 1) * m_dataObjWhileRastering->getSize(m_dataObjWhileRastering->getDims() - 2);
                unsigned char *srcPtr = ((cv::Mat*)m_dataObjWhileRastering->get_mdata()[0])->ptr();
                unsigned char *dstPtr = retImage.bits();
                
                unsigned char scaling = cv::saturate_cast<ito::uint8>(255.0 / (m_maxZValue - m_minZValue));
                unsigned char minVal = cv::saturate_cast<ito::uint8>(m_minZValue);
                for(int i = 0; i < pixelCnt; i++)
                {
                    *dstPtr = (*srcPtr - minVal)*(scaling);
                    dstPtr++;
                    srcPtr++;
                }
                return retImage;
            }
            else
            {
                // this is a 8bit image which shall be scaled to 8bit -> just copy the image
                QImage retImage(((cv::Mat*)m_dataObjWhileRastering->get_mdata()[0])->ptr(), m_dataObjWhileRastering->getSize(m_dataObjWhileRastering->getDims() - 1), m_dataObjWhileRastering->getSize(m_dataObjWhileRastering->getDims() - 2), m_dataObjWhileRastering->getSize(m_dataObjWhileRastering->getDims() - 1), QImage::Format_Indexed8);
                retImage.setColorTable(m_colorTable);
                return retImage;
            }
            return QImage();
        }
        case ito::tInt8:
        {
            // check if it is scale according to bitdepth

            QImage retImage(m_dataObjWhileRastering->getSize(m_dataObjWhileRastering->getDims() - 1), m_dataObjWhileRastering->getSize(m_dataObjWhileRastering->getDims() - 2), QImage::Format_Indexed8);
            retImage.setColorTable(m_colorTable);
            
            int pixelCnt = m_dataObjWhileRastering->getSize(m_dataObjWhileRastering->getDims() - 1) * m_dataObjWhileRastering->getSize(m_dataObjWhileRastering->getDims() - 2);
            ito::int8 *srcPtr = (ito::int8*)((cv::Mat*)m_dataObjWhileRastering->get_mdata()[0])->ptr();
            unsigned char *dstPtr = retImage.bits();

            if(m_colorMode == ColorIndex8Scaled)
            {
                double scaling = 255.0 / (m_maxZValue - m_minZValue);
                for(int i = 0; i < pixelCnt; i++)
                {
                    *dstPtr = cv::saturate_cast<ito::uint8>((*srcPtr - m_minZValue)*(scaling));
                    dstPtr++;
                    srcPtr++;
                }
            }
            else
            {
                for(int i = 0; i < pixelCnt; i++)
                {
                    *dstPtr = (ito::uint8)(*srcPtr) + 128;
                    dstPtr++;
                    srcPtr++;
                }                
            }
            return retImage;
        }
        
        case ito::tUInt16:
        {
            QImage retImage(m_dataObjWhileRastering->getSize(m_dataObjWhileRastering->getDims() - 1), m_dataObjWhileRastering->getSize(m_dataObjWhileRastering->getDims() - 2), QImage::Format_Indexed8);
            retImage.setColorTable(m_colorTable);

            int pixelCnt = m_dataObjWhileRastering->getSize(m_dataObjWhileRastering->getDims() - 1) * m_dataObjWhileRastering->getSize(m_dataObjWhileRastering->getDims() - 2);
            unsigned short *srcPtr = (unsigned short*)((cv::Mat*)m_dataObjWhileRastering->get_mdata()[0])->ptr();
            unsigned char *dstPtr = retImage.bits();

            unsigned char bitShift = m_numBits > 8 ? (m_numBits - 8) : 0;
            bitShift = bitShift > 8 ? 8 : bitShift;
            

            if(m_colorMode == ColorIndex8Scaled)
            {
                double scaling = 255.0 / (m_maxZValue - m_minZValue);
                for(int i = 0; i < pixelCnt; i++)
                {
                    *dstPtr = cv::saturate_cast<ito::uint8>((*srcPtr - m_minZValue)*(scaling));
                    dstPtr++;
                    srcPtr++;
                }
            }
            else
            {
                for(int i = 0; i < pixelCnt; i++)
                {
                    *dstPtr = cv::saturate_cast<ito::uint8>(*srcPtr >> bitShift);
                    dstPtr++;
                    srcPtr++;    
                }
            }
            return retImage;
        }
        case ito::tInt16:
        {
            // check if it is scale according to bitdepth

            QImage retImage(m_dataObjWhileRastering->getSize(m_dataObjWhileRastering->getDims() - 1), m_dataObjWhileRastering->getSize(m_dataObjWhileRastering->getDims() - 2), QImage::Format_Indexed8);
            retImage.setColorTable(m_colorTable);

            int pixelCnt = m_dataObjWhileRastering->getSize(m_dataObjWhileRastering->getDims() - 1) * m_dataObjWhileRastering->getSize(m_dataObjWhileRastering->getDims() - 2);
            signed short *srcPtr = (signed short*)((cv::Mat*)m_dataObjWhileRastering->get_mdata()[0])->ptr();
            unsigned char *dstPtr = retImage.bits();

            unsigned char bitShift = m_numBits > 8 ? (m_numBits - 8) : 0;
            bitShift = bitShift > 8 ? 8 : bitShift;

            if(m_colorMode == ColorIndex8Scaled)
            {
                double scaling = 255.0 / (m_maxZValue - m_minZValue);
                for(int i = 0; i < pixelCnt; i++)
                {
                    *dstPtr = cv::saturate_cast<ito::uint8>((*srcPtr - m_minZValue)*(scaling));
                    dstPtr++;
                    srcPtr++;
                }
            }
            else
            {
                if(bitShift < 8)
                {
                    for(int i = 0; i < pixelCnt; i++)
                    {
                        *dstPtr = cv::saturate_cast<ito::uint8>(*srcPtr >> bitShift);
                        dstPtr++;
                        srcPtr++;    
                    }
                }
                else
                {
                    for(int i = 0; i < pixelCnt; i++)
                    {
                        *dstPtr = (ito::uint8)((*srcPtr >> bitShift) + 128);
                        dstPtr++;
                        srcPtr++;    
                    }
                }
            }
            return retImage;

        }
        case ito::tUInt32:
        {
            // check if it is scale according to bitdepth
            if(m_colorMode == ColorRGB24)
            {
                return QImage(((cv::Mat*)m_dataObjWhileRastering->get_mdata()[0])->ptr(), m_dataObjWhileRastering->getSize(m_dataObjWhileRastering->getDims() - 1), m_dataObjWhileRastering->getSize(m_dataObjWhileRastering->getDims() - 2), m_dataObjWhileRastering->getSize(m_dataObjWhileRastering->getDims() - 1)*4, QImage::Format_RGB32);
            }
            else if(m_colorMode == ColorRGB32)
            {
                return QImage(((cv::Mat*)m_dataObjWhileRastering->get_mdata()[0])->ptr(), m_dataObjWhileRastering->getSize(m_dataObjWhileRastering->getDims() - 1), m_dataObjWhileRastering->getSize(m_dataObjWhileRastering->getDims() - 2), m_dataObjWhileRastering->getSize(m_dataObjWhileRastering->getDims() - 1)*4, QImage::Format_ARGB32);          
            }
            else
            {
                QImage retImage(m_dataObjWhileRastering->getSize(m_dataObjWhileRastering->getDims() - 1), m_dataObjWhileRastering->getSize(m_dataObjWhileRastering->getDims() - 2), QImage::Format_Indexed8);
                retImage.setColorTable(m_colorTable);

                int pixelCnt = m_dataObjWhileRastering->getSize(m_dataObjWhileRastering->getDims() - 1) * m_dataObjWhileRastering->getSize(m_dataObjWhileRastering->getDims() - 2);
                ito::uint32 *srcPtr = (ito::uint32*)((cv::Mat*)m_dataObjWhileRastering->get_mdata()[0])->ptr();
                unsigned char *dstPtr = retImage.bits();

                unsigned char bitShift = m_numBits > 8 ? (m_numBits - 8) : 0;

                if(m_colorMode == ColorIndex8Scaled)
                { 
                    double scaling = 255.0 / (m_maxZValue - m_minZValue);
                    for(int i = 0; i < pixelCnt; i++)
                    {
                        *dstPtr = cv::saturate_cast<ito::uint8>((*srcPtr - m_minZValue)*(scaling));
                        dstPtr++;
                        srcPtr++;
                    }
                }
                else
                {
                    for(int i = 0; i < pixelCnt; i++)
                    {
                        *dstPtr = cv::saturate_cast<ito::uint8>(*srcPtr >> bitShift);
                        dstPtr++;
                        srcPtr++;    
                    }
                }
                return retImage;

            }
            return QImage();
        }
        break;
        case ito::tInt32:
        {
            // check if it is scale according to bitdepth
            if(m_colorMode == ColorRGB24)
            {
                return QImage(((cv::Mat*)m_dataObjWhileRastering->get_mdata()[0])->ptr(), m_dataObjWhileRastering->getSize(m_dataObjWhileRastering->getDims() - 1), m_dataObjWhileRastering->getSize(m_dataObjWhileRastering->getDims() - 2), m_dataObjWhileRastering->getSize(m_dataObjWhileRastering->getDims() - 1)*4 , QImage::Format_RGB32);
            }
            else if(m_colorMode == ColorRGB32)
            {
                return QImage(((cv::Mat*)m_dataObjWhileRastering->get_mdata()[0])->ptr(), m_dataObjWhileRastering->getSize(m_dataObjWhileRastering->getDims() - 1), m_dataObjWhileRastering->getSize(m_dataObjWhileRastering->getDims() - 2), m_dataObjWhileRastering->getSize(m_dataObjWhileRastering->getDims() - 1)*4 , QImage::Format_RGB32);
            }
            else
            {
                QImage retImage(m_dataObjWhileRastering->getSize(m_dataObjWhileRastering->getDims() - 1), m_dataObjWhileRastering->getSize(m_dataObjWhileRastering->getDims() - 2), QImage::Format_Indexed8);
                retImage.setColorTable(m_colorTable);

                int pixelCnt = m_dataObjWhileRastering->getSize(m_dataObjWhileRastering->getDims() - 1) * m_dataObjWhileRastering->getSize(m_dataObjWhileRastering->getDims() - 2);
                ito::int32 *srcPtr = (ito::int32*)((cv::Mat*)m_dataObjWhileRastering->get_mdata()[0])->ptr();
                unsigned char *dstPtr = retImage.bits();

                unsigned char bitShift = m_numBits > 8 ? (m_numBits - 8) : 0;

                if(m_colorMode == ColorIndex8Scaled)
                { 
                    double scaling = 255.0 / (m_maxZValue - m_minZValue);
                    for(int i = 0; i < pixelCnt; i++)
                    {
                        *dstPtr = cv::saturate_cast<ito::uint8>((*srcPtr - m_minZValue)*(scaling));
                        dstPtr++;
                        srcPtr++;
                    }
                }
                else
                {
                    if(bitShift < 24)
                    {
                        for(int i = 0; i < pixelCnt; i++)
                        {
                            *dstPtr = cv::saturate_cast<ito::uint8>(*srcPtr >> bitShift);
                            dstPtr++;
                            srcPtr++;    
                        }
                    }
                    else
                    {
                        for(int i = 0; i < pixelCnt; i++)
                        {
                            *dstPtr = ((ito::uint8)(*srcPtr >> bitShift)) + 128;
                            dstPtr++;
                            srcPtr++;    
                        }                    
                    }
                }
                return retImage;
            }

        }
        break;
        case ito::tFloat32:
        {
            // check if it is scale according to bitdepth
            QImage retImage(m_dataObjWhileRastering->getSize(m_dataObjWhileRastering->getDims() - 1), m_dataObjWhileRastering->getSize(m_dataObjWhileRastering->getDims() - 2), QImage::Format_Indexed8);
            retImage.setColorTable(m_colorTable);
            
            int pixelCnt = m_dataObjWhileRastering->getSize(m_dataObjWhileRastering->getDims() - 1) * m_dataObjWhileRastering->getSize(m_dataObjWhileRastering->getDims() - 2);
            ito::float32 *srcPtr = (ito::float32*)((cv::Mat*)m_dataObjWhileRastering->get_mdata()[0])->ptr();
            unsigned char *dstPtr = retImage.bits();
            double scaling = 255.0 / (m_maxZValue - m_minZValue);
            for(int i = 0; i < pixelCnt; i++)
            {
                *dstPtr = cv::saturate_cast<ito::uint8>((*srcPtr - m_minZValue)*(scaling));
                dstPtr++;
                srcPtr++;
            }
            return retImage;
        }
        break;
        case ito::tFloat64:
        {
            // check if it is scale according to bitdepth
            QImage retImage(m_dataObjWhileRastering->getSize(m_dataObjWhileRastering->getDims() - 1), m_dataObjWhileRastering->getSize(m_dataObjWhileRastering->getDims() - 2), QImage::Format_Indexed8);
            retImage.setColorTable(m_colorTable);
            
            int pixelCnt = m_dataObjWhileRastering->getSize(m_dataObjWhileRastering->getDims() - 1) * m_dataObjWhileRastering->getSize(m_dataObjWhileRastering->getDims() - 2);
            ito::float64 *srcPtr = (ito::float64*)((cv::Mat*)m_dataObjWhileRastering->get_mdata()[0])->ptr();
            unsigned char *dstPtr = retImage.bits();
            double scaling = 255.0 / (m_maxZValue - m_minZValue);
            for(int i = 0; i < pixelCnt; i++)
            {
                *dstPtr = cv::saturate_cast<ito::uint8>((*srcPtr - m_minZValue)*(scaling));
                dstPtr++;
                srcPtr++;
            }
            return retImage;
        }
        break;
        case ito::tComplex64:
        {
            // check if it is scale according to bitdepth
            QImage retImage(m_dataObjWhileRastering->getSize(m_dataObjWhileRastering->getDims() - 1), m_dataObjWhileRastering->getSize(m_dataObjWhileRastering->getDims() - 2), QImage::Format_Indexed8);
            retImage.setColorTable(m_colorTable);
            
            int pixelCnt = m_dataObjWhileRastering->getSize(m_dataObjWhileRastering->getDims() - 1) * m_dataObjWhileRastering->getSize(m_dataObjWhileRastering->getDims() - 2);
            ito::complex64 *srcPtr = (ito::complex64*)((cv::Mat*)m_dataObjWhileRastering->get_mdata()[0])->ptr();
            unsigned char *dstPtr = retImage.bits();
            double scaling = 255.0 / (m_maxZValue - m_minZValue);

			switch (m_cmplxState)
			{
				default:
				case 0:
                    for(int i = 0; i < pixelCnt; i++)
                    {
                        *dstPtr = cv::saturate_cast<ito::uint8>((abs(*srcPtr) - m_minZValue)*(scaling));
                        dstPtr++;
                        srcPtr++;
                    }
				break;
				case 1:
                    for(int i = 0; i < pixelCnt; i++)
                    {
                        *dstPtr = cv::saturate_cast<ito::uint8>(((*srcPtr).real() - m_minZValue)*(scaling));
                        dstPtr++;
                        srcPtr++;
                    }
				break;
				case 2:
                    for(int i = 0; i < pixelCnt; i++)
                    {
                        *dstPtr = cv::saturate_cast<ito::uint8>(((*srcPtr).imag() - m_minZValue)*(scaling));
                        dstPtr++;
                        srcPtr++;
                    }
				break;
				case 3:
                    for(int i = 0; i < pixelCnt; i++)
                    {
                        *dstPtr = cv::saturate_cast<ito::uint8>((arg(*srcPtr) - m_minZValue)*(scaling));
                        dstPtr++;
                        srcPtr++;
                    }
				break;
			}


            return retImage;
        }
        break;
        case ito::tComplex128:
        {
            // check if it is scale according to bitdepth
            QImage retImage(m_dataObjWhileRastering->getSize(m_dataObjWhileRastering->getDims() - 1), m_dataObjWhileRastering->getSize(m_dataObjWhileRastering->getDims() - 2), QImage::Format_Indexed8);
            retImage.setColorTable(m_colorTable);
            
            int pixelCnt = m_dataObjWhileRastering->getSize(m_dataObjWhileRastering->getDims() - 1) * m_dataObjWhileRastering->getSize(m_dataObjWhileRastering->getDims() - 2);
            ito::complex128 *srcPtr = (ito::complex128*)((cv::Mat*)m_dataObjWhileRastering->get_mdata()[0])->ptr();
            unsigned char *dstPtr = retImage.bits();
            double scaling = 255.0 / (m_maxZValue - m_minZValue);

			switch (m_cmplxState)
			{
				default:
				case 0:
                    for(int i = 0; i < pixelCnt; i++)
                    {
                        *dstPtr = cv::saturate_cast<ito::uint8>((abs(*srcPtr) - m_minZValue)*(scaling));
                        dstPtr++;
                        srcPtr++;
                    }
				break;
				case 1:
                    for(int i = 0; i < pixelCnt; i++)
                    {
                        *dstPtr = cv::saturate_cast<ito::uint8>(((*srcPtr).real() - m_minZValue)*(scaling));
                        dstPtr++;
                        srcPtr++;
                    }
				break;
				case 2:
                    for(int i = 0; i < pixelCnt; i++)
                    {
                        *dstPtr = cv::saturate_cast<ito::uint8>(((*srcPtr).imag() - m_minZValue)*(scaling));
                        dstPtr++;
                        srcPtr++;
                    }
				break;
				case 3:
                    for(int i = 0; i < pixelCnt; i++)
                    {
                        *dstPtr = cv::saturate_cast<ito::uint8>((arg(*srcPtr) - m_minZValue)*(scaling));
                        dstPtr++;
                        srcPtr++;
                    }
				break;
			}

            return retImage;
        }
        break;
        default:
        {
        }
        break;
    }

    //if(m_dataObjWhileRastering)
    //{
    //    m_dataObjWhileRastering->unlock();
    //    m_dataObjWhileRastering.clear();

        //qDebug("raster data: end");
        //if(m_pReplotPending) *m_pReplotPending = false;
    //}
    return QImage();
}
//----------------------------------------------------------------------------------------------------------------------------------------------
ito::float64 RasterToQImageObj::getPixel(const QPointF &coords, bool &isInt)
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
			switch (m_cmplxState)
			{
				default:
				case 0:
					ret = abs(retTemp);
				break;
				case 1:
					ret = retTemp.real();
				break;
				case 2:
					ret = retTemp.imag();
				break;
				case 3:
					ret = arg(retTemp);
				break;
			}
            isInt = false;
            break;
        }
        case ito::tComplex128:
        {
            ito::complex128 retTemp = (ito::complex128)m_dataObj->at<ito::complex128>(index);
			switch (m_cmplxState)
			{
				default:
				case 0:
					ret = abs(retTemp);
				break;
				case 1:
					ret = retTemp.real();
				break;
				case 2:
					ret = retTemp.imag();
				break;
				case 3:
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
    };

    AValue = (unsigned char)((val & 0xFF000000) >> 24); 
    RValue = (unsigned char)((val & 0x00FF0000) >> 16);
    GValue = (unsigned char)((val & 0x0000FF00) >> 8);
    BValue = (unsigned char)(val & 0x000000FF);

    delete index;

    m_dataObj->unlock();

    return ret; 
}