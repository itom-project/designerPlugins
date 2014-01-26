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
RasterToQImageObj::RasterToQImageObj(InternalData *m_internalData) :
    m_validData(false),
    m_dataHash(),
    m_plane(NULL),
    m_hashGenerator(QCryptographicHash::Md5),
    m_dataObjPlane(NULL),
    m_pInternalData(m_internalData)
{
    m_D.m_planeIdx = 0;
    m_D.m_yScaling = 1.0;
    m_D.m_xScaling = 1.0;
    m_D.m_yOffset = 0.0;
    m_D.m_xOffset = 0.0;
    m_D.m_ySize = 0;
    m_D.m_xSize = 0;
    m_D.m_dataPtr = NULL;
}

//----------------------------------------------------------------------------------------------------------------------------------
void RasterToQImageObj::deleteCache()
{
    m_validData = false;
}
//----------------------------------------------------------------------------------------------------------------------------------
RasterToQImageObj::~RasterToQImageObj()
{
    deleteCache();

    bool dataObjPlaneWasShallow = (&m_dataObj != m_dataObjPlane);
    if (m_dataObjPlane && dataObjPlaneWasShallow) //m_dataObjPlane was a shallow copy -> delete it
    {
        delete m_dataObjPlane;
        m_dataObjPlane = NULL;
    }
}
//----------------------------------------------------------------------------------------------------------------------------------
QByteArray RasterToQImageObj::calcHash(const ito::DataObject *dObj)
{
    if (dObj == NULL) dObj = &m_dataObj;

    if(dObj->getDims() < 2)
    {
        return m_hashGenerator.hash("", QCryptographicHash::Md5);
    }
    else
    {
        QByteArray ba;

        int dims = dObj->getDims();
        ba.append( dims );
        ba.append( m_pInternalData->m_cmplxType );
        ba.append( m_pInternalData->m_yaxisFlipped );
        ba.append( (char)m_D.m_planeIdx );

        if( dims > 0 )
        {
            cv::Mat *m = (cv::Mat*)dObj->get_mdata()[ dObj->seekMat(0) ];
            uchar* d = m->data;
            ba.append( QByteArray( (const char*)&d, (sizeof(int)/sizeof(char)))); //address to data of first plane

            ba.append( QByteArray().setNum( m->size[0] ));
        }

        return m_hashGenerator.hash(ba, QCryptographicHash::Md5);
    }
}
//----------------------------------------------------------------------------------------------------------------------------------
bool RasterToQImageObj::updateDataObject(const ito::DataObject *dataObj, int planeIdx /*= -1*/)
{
    //the base idea behind simple pointer copying (instead of shallow copies or shared pointer)
    // is that AbstractDObjFigure always keeps shallow copies of all data objects and therefore is 
    // responsible that no dataObject is deleted when it is still in use by any object of this entire plot plugin.
    
    bool newHash = false;
    bool dataObjPlaneWasShallow = (&m_dataObj != m_dataObjPlane);

    if(dataObj)
    {
        int d = dataObj->getDims();
        m_D.m_yScaling = d > 1 ? dataObj->getAxisScale(d-2) : 1.0;
        m_D.m_xScaling = d > 1 ? dataObj->getAxisScale(d-1) : 1.0;
        m_D.m_yOffset = d > 1 ? dataObj->getAxisOffset(d-2) : 0.0;
        m_D.m_xOffset = d > 1 ? dataObj->getAxisOffset(d-1) : 0.0;
        m_D.m_ySize = d > 1 ? dataObj->getSize(d-2) : 0;
        m_D.m_xSize = d > 1 ? dataObj->getSize(d-1) : 0;
        m_D.m_dataPtr = NULL; //dataObj->get_mdata();

        if (planeIdx >= 0 && m_D.m_planeIdx != planeIdx)
        {
            deleteCache();
            m_D.m_planeIdx = planeIdx;
        }

        QByteArray hash = calcHash(dataObj);

        if (m_hash != hash)
        {
            m_hash = hash;

            if (planeIdx == -1)
            {
                m_D.m_planeIdx = 0;
            }

            newHash = true;

            deleteCache();

            m_dataObj = *dataObj;
            m_plane = m_dataObj.getDims() > 1 ? (cv::Mat*)(m_dataObj.get_mdata()[ m_dataObj.seekMat( (int)m_D.m_planeIdx )]) : NULL;

            cv::Size whole;
            cv::Point offSets;
            m_plane->locateROI(whole, offSets);

            if(offSets.x != 0 || offSets.y != 0 || whole.height != m_plane->rows || whole.width != m_plane->cols)
            {
                m_D.m_hasROI = true;
            }
            else
            {
                m_D.m_hasROI = false;
            }

            if (m_dataObjPlane && dataObjPlaneWasShallow) //m_dataObjPlane was a shallow copy -> delete it
            {
                delete m_dataObjPlane;
                m_dataObjPlane = NULL;
            }

            if (m_dataObj.getDims() > 2)
            {
                int sizes[2] = { m_D.m_ySize, m_D.m_xSize };
                m_dataObjPlane = new ito::DataObject( 2, sizes, m_dataObj.getType(), m_plane, 1);
                m_dataObj.copyTagMapTo( *m_dataObjPlane );

                m_dataObjPlane->setValueDescription( m_dataObj.getValueDescription() );
                m_dataObjPlane->setValueUnit( m_dataObj.getValueUnit() );

                m_dataObj.copyAxisTagsTo( *m_dataObjPlane );
            }
            else
            {
                m_dataObjPlane = &m_dataObj;
            }

            //m_pInternalData->m_pConstOutput->operator[]("sourceout")->setVal<void*>((void*)&m_dataObj);
            //m_pInternalData->m_pConstOutput->operator[]("displayed")->setVal<void*>((void*)m_dataObjPlane);

        }

        if(m_pInternalData->m_valueScaleAuto)
        {
            ito::float64 zMin;
            ito::float64 zMax;
            ito::uint32 locMin[3], locMax[3];
            ito::dObjHelper::minMaxValue(m_dataObjPlane, zMin, locMin, zMax, locMax, true, m_pInternalData->m_cmplxType);
            m_pInternalData->m_valueMin = zMin;
            m_pInternalData->m_valueMax = zMax;
        }
    }
    else
    {
        if (m_dataObjPlane && dataObjPlaneWasShallow) //m_dataObjPlane was a shallow copy -> delete it
        {
            delete m_dataObjPlane;
        }

        m_dataObjPlane = NULL;
        m_dataObj = ito::DataObject();
        //m_pInternalData->m_pConstOutput->operator[]("sourceout")->setVal<void*>(NULL);
        //m_pInternalData->m_pConstOutput->operator[]("output")->setVal<void*>(NULL);

        m_plane = NULL;

        m_D.m_dataPtr = NULL;
        m_D.m_yScaling = (1.0);
        m_D.m_xScaling = (1.0);
        m_D.m_yOffset = (0.0);
        m_D.m_xOffset = (0.0);
        m_D.m_ySize = 0;
        m_D.m_xSize = 0;
        m_D.m_planeIdx = 0;

        deleteCache();
        m_hash = QByteArray();
        newHash = true;
    }
    
    
    return newHash;
}

QImage RasterToQImageObj::convert2QImage()
{
    if(!m_plane)
    {
        QImage temp(100, 100, QImage::Format_ARGB32);
        temp.setText("", "No Image Data");
        return temp;    
    }

    int type = m_dataObjPlane->getType();

    if(m_pInternalData->m_colorMode == ColorIndex8Bitshift)
    {
        switch(type)
        {
            case ito::tUInt8:
            {
                // this is a 8bit image which shall be scaled to 8bit -> just copy the image
                if(m_D.m_hasROI)
                {
                    QImage retImage(m_D.m_xSize, m_D.m_ySize, QImage::Format_Indexed8);
                    retImage.setColorTable(m_pInternalData->m_colorTable);
                    ito::uint8 *dstPtr = retImage.bits();

                    size_t nextDstRow = m_D.m_xSize % 4;
                    if(nextDstRow > 0) nextDstRow = 4 - nextDstRow;
                    nextDstRow += m_D.m_xSize;
                    for(int y = 0; y < m_D.m_ySize; y++)
                    {
                        ito::uint8 *srcPtr = m_plane->ptr<ito::uint8>(y);
                        memcpy(dstPtr, srcPtr, m_D.m_xSize);
                        dstPtr +=  nextDstRow;
                    }
                    return retImage;
                }
                else
                {
                    QImage retImage(m_plane->ptr<ito::uint8>(), m_D.m_xSize, m_D.m_ySize, m_D.m_xSize, QImage::Format_Indexed8);
                    retImage.setColorTable(m_pInternalData->m_colorTable);
                    return retImage;
                }
            }
            case ito::tUInt16:
            {
                QImage retImage(m_D.m_xSize, m_D.m_ySize, QImage::Format_Indexed8);
                retImage.setColorTable(m_pInternalData->m_colorTable);

                
                ito::uint16 *srcPtr = m_plane->ptr<ito::uint16>();
                ito::uint8 *dstPtr = retImage.bits();

                unsigned char bitShift = m_pInternalData->m_numBits > 8 ? (m_pInternalData->m_numBits - 8) : 0;
                bitShift = bitShift > 8 ? 8 : bitShift;
            
                size_t nextRow = (size_t)((m_plane->step[0] - m_D.m_xSize * m_plane->step[1]) / 2);
                size_t nextDstRow = m_D.m_xSize % 4;
                if(nextDstRow > 0) nextDstRow = 4 - nextDstRow;

                if(m_D.m_hasROI || nextDstRow != 0)
                {
                    for(int y = 0; y < m_D.m_ySize; y++)
                    {
                        for(int i = 0; i < m_D.m_xSize; i++)
                        {
                            *dstPtr = cv::saturate_cast<ito::uint8>((*srcPtr) >> bitShift);
                            dstPtr++;
                            srcPtr++;    
                        }
                        srcPtr += nextRow;
                        dstPtr += nextDstRow;
                    }
                }
                else
                {
                    int pixelCnt = m_D.m_xSize* m_D.m_ySize;
                    for(int i = 0; i < pixelCnt; i++)
                    {
                        *dstPtr = cv::saturate_cast<ito::uint8>((*srcPtr) >> bitShift);
                        dstPtr++;
                        srcPtr++;    
                    }
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
    else if(m_pInternalData->m_colorMode == ColorRGB24)
    {
        if (type != ito::tUInt32 && type != ito::tInt32 && type != ito::tRGBA32)
        {
            QImage temp(100, 100, QImage::Format_ARGB32);
            temp.setText("", "Wrong image type");
            return temp;      
        }

        if(m_D.m_hasROI)
        {
            QImage retImage(m_D.m_xSize, m_D.m_ySize, QImage::Format_RGB32);
            retImage.setColorTable(m_pInternalData->m_colorTable);
            ito::uint8 *dstPtr = retImage.bits();
            for(int y = 0; y < m_D.m_ySize; y++)
            {
                ito::uint8 *srcPtr = m_plane->ptr<ito::uint8>(y);
                memcpy(dstPtr, srcPtr, m_D.m_xSize * 4);
                dstPtr += m_D.m_xSize * 4;
            }
            return retImage;
        }
        else
        {
            return QImage(m_plane->ptr(), m_D.m_xSize, m_D.m_ySize, m_D.m_xSize*4 , QImage::Format_RGB32);
        }

        
    }
    else if(m_pInternalData->m_colorMode == ColorRGB32)
    {
        if (type != ito::tUInt32 && type != ito::tInt32 && type != ito::tRGBA32)
        {
            QImage temp(100, 100, QImage::Format_ARGB32);
            temp.setText("", "Wrong image type");
            return temp;      
        }
        if(m_D.m_hasROI)
        {
            QImage retImage(m_D.m_xSize, m_D.m_ySize, QImage::Format_ARGB32);
            retImage.setColorTable(m_pInternalData->m_colorTable);
            ito::uint8 *dstPtr = retImage.bits();
            for(int y = 0; y < m_D.m_ySize; y++)
            {
                ito::uint8 *srcPtr = m_plane->ptr<ito::uint8>(y);
                memcpy(dstPtr, srcPtr, m_D.m_xSize * 4);
                dstPtr += m_D.m_xSize * 4;
            }
            return retImage;
        }
        else
        {
            return QImage(m_plane->ptr(), m_D.m_xSize, m_D.m_ySize, m_D.m_xSize*4 , QImage::Format_ARGB32);
        }
    }
    else
    {


        switch(type)
        {
            case ito::tUInt8:
                return graphicViewHelper::rescaleByScale<ito::uint8>(m_plane, m_D.m_xSize, m_D.m_ySize, m_plane->step[0], m_plane->step[1], m_pInternalData->m_colorTable, m_pInternalData->m_valueMin, m_pInternalData->m_valueMax, 0);
            case ito::tInt8:
                return graphicViewHelper::rescaleByScale<ito::int8>(m_plane, m_D.m_xSize, m_D.m_ySize, m_plane->step[0], m_plane->step[1], m_pInternalData->m_colorTable, m_pInternalData->m_valueMin, m_pInternalData->m_valueMax, 0);
            case ito::tUInt16:
                return graphicViewHelper::rescaleByScale<ito::uint16>(m_plane, m_D.m_xSize, m_D.m_ySize, m_plane->step[0], m_plane->step[1], m_pInternalData->m_colorTable, m_pInternalData->m_valueMin, m_pInternalData->m_valueMax, 0);
            case ito::tInt16:
                return graphicViewHelper::rescaleByScale<ito::int16>(m_plane, m_D.m_xSize, m_D.m_ySize, m_plane->step[0], m_plane->step[1], m_pInternalData->m_colorTable, m_pInternalData->m_valueMin, m_pInternalData->m_valueMax, 0);
            case ito::tUInt32:
                return graphicViewHelper::rescaleByScale<ito::uint32>(m_plane, m_D.m_xSize, m_D.m_ySize, m_plane->step[0], m_plane->step[1], m_pInternalData->m_colorTable, m_pInternalData->m_valueMin, m_pInternalData->m_valueMax, 0);
            case ito::tInt32:
                return graphicViewHelper::rescaleByScale<ito::int32>(m_plane, m_D.m_xSize, m_D.m_ySize, m_plane->step[0], m_plane->step[1], m_pInternalData->m_colorTable, m_pInternalData->m_valueMin, m_pInternalData->m_valueMax, 0);
            case ito::tFloat32:
                return graphicViewHelper::rescaleByScale<ito::float32>(m_plane, m_D.m_xSize, m_D.m_ySize, m_plane->step[0], m_plane->step[1], m_pInternalData->m_colorTable, m_pInternalData->m_valueMin, m_pInternalData->m_valueMax, 0);
            case ito::tFloat64:
                return graphicViewHelper::rescaleByScale<ito::float64>(m_plane, m_D.m_xSize, m_D.m_ySize, m_plane->step[0], m_plane->step[1], m_pInternalData->m_colorTable, m_pInternalData->m_valueMin, m_pInternalData->m_valueMax, 0);
            case ito::tComplex64:
                return graphicViewHelper::rescaleByScale<ito::complex64>(m_plane, m_D.m_xSize, m_D.m_ySize, m_plane->step[0], m_plane->step[1], m_pInternalData->m_colorTable, m_pInternalData->m_valueMin, m_pInternalData->m_valueMax, m_pInternalData->m_cmplxType);
            case ito::tComplex128:
                return graphicViewHelper::rescaleByScale<ito::complex128>(m_plane, m_D.m_xSize, m_D.m_ySize, m_plane->step[0], m_plane->step[1], m_pInternalData->m_colorTable, m_pInternalData->m_valueMin, m_pInternalData->m_valueMax, m_pInternalData->m_cmplxType);
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
    if(!m_dataObjPlane || m_dataObjPlane->getDims() < 2)
        return -666.0;

    ito::float64 ret = -666.0;

    unsigned int *index = new unsigned int[m_dataObjPlane->getDims()];

    memset(index, 0, sizeof(unsigned int)*m_dataObjPlane->getDims());
    
    index[m_dataObjPlane->getDims()-1] = (int)coords.x();
    index[m_dataObjPlane->getDims()-2] = (int)coords.y();

    switch(m_dataObjPlane->getType())
    {
        case ito::tInt8:
            ret = (ito::float64)m_dataObjPlane->at<ito::int8>(index);
            isInt = true;
            break;
        case ito::tInt16:
            ret = (ito::float64)m_dataObjPlane->at<ito::int16>(index);
            isInt = true;
            break;
        case ito::tInt32:
            ret = (ito::float64)m_dataObjPlane->at<ito::int32>(index);
            isInt = true;
            break;
        case ito::tUInt8:
            ret = (ito::float64)m_dataObjPlane->at<ito::uint8>(index);
            isInt = true;
            break;
        case ito::tUInt16:
            ret = (ito::float64)m_dataObjPlane->at<ito::uint16>(index);
            isInt = true;
            break;
        case ito::tUInt32:
            ret = (ito::float64)m_dataObjPlane->at<ito::uint32>(index);
            isInt = true;
            break;
        case ito::tFloat32:
            ret = (ito::float64)m_dataObjPlane->at<ito::float32>(index);
            isInt = false;
            break;
        case ito::tFloat64:
            ret = (ito::float64)m_dataObjPlane->at<ito::float64>(index);
            isInt = false;
            break;
        case ito::tComplex64:
        {
            ito::complex64 retTemp = (ito::complex64)m_dataObjPlane->at<ito::complex64>(index);
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
            ito::complex128 retTemp = (ito::complex128)m_dataObjPlane->at<ito::complex128>(index);
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

    delete[] index;

    return ret; 
}
//----------------------------------------------------------------------------------------------------------------------------------------------
bool RasterToQImageObj::getPixelARGB(const QPointF &coords, unsigned char &AValue, unsigned char &RValue, unsigned char &GValue, unsigned char &BValue)
{
    if(!m_dataObjPlane || m_dataObjPlane->getDims() < 2)
        return false;

    bool ret = false;

    unsigned int *index = new unsigned int[m_dataObjPlane->getDims()];
    memset(index, 0, sizeof(unsigned int)*m_dataObjPlane->getDims());
    
    index[m_dataObjPlane->getDims()-1] = (int)coords.x();
    index[m_dataObjPlane->getDims()-2] = (int)coords.y();

    ito::uint32 val = 0;

    switch(m_dataObjPlane->getType())
    {
        //case ito::tInt16:
        //    ret = (ito::float64)m_dataObj->at<ito::int16>(index);
        //    break;
        case ito::tInt32:
            val = (ito::uint32)m_dataObjPlane->at<ito::int32>(index);
            ret = true;
            break;
        //case ito::tUInt16:
        //    ret = (ito::float64)m_dataObj->at<ito::uint16>(index);
        //    break;
        case ito::tUInt32:
            val = (ito::uint32)m_dataObjPlane->at<ito::uint32>(index);
            ret = true;
            break;
        case ito::tRGBA32:
            val = m_dataObjPlane->at<ito::Rgba32>(index).argb();
            ret = true;
            break;
    };

    AValue = (unsigned char)((val & 0xFF000000) >> 24); 
    RValue = (unsigned char)((val & 0x00FF0000) >> 16);
    GValue = (unsigned char)((val & 0x0000FF00) >> 8);
    BValue = (unsigned char)(val & 0x000000FF);

    delete[] index;

    return ret; 
}