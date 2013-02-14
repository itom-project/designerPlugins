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

#include "dataObjRasterData.h"
#include "../../common/typeDefs.h"

#include "../../DataObject/dataObjectFuncs.h"

#include <qdebug.h>

//----------------------------------------------------------------------------------------------------------------------------------
DataObjRasterData::DataObjRasterData() :
    QwtRasterData(),
    m_validData(false),
    m_dataHash(),
    m_rasteredLinePtr(NULL),
    m_xIndizes(NULL),
    m_plane(NULL),
    m_hashGenerator(QCryptographicHash::Md5)
{
    m_D.m_planeIdx = 0;
    m_D.m_yScaling = 1.0;
    m_D.m_xScaling = 1.0;
    m_D.m_yOffset = 0.0;
    m_D.m_xOffset = 0.0;
    m_D.m_dataPtr = NULL;
}

//----------------------------------------------------------------------------------------------------------------------------------
DataObjRasterData::~DataObjRasterData()
{
    deleteCache();
}

//----------------------------------------------------------------------------------------------------------------------------------
void DataObjRasterData::deleteCache()
{
    if(m_rasteredLinePtr)
    {
        delete[] m_rasteredLinePtr;
    }
    m_rasteredLinePtr = NULL;

    if(m_xIndizes)
    {
        delete[] m_xIndizes;
    }
    m_xIndizes = NULL;
    m_plane = NULL;
}

//----------------------------------------------------------------------------------------------------------------------------------
void DataObjRasterData::updateDataObject(ito::DataObject *dataObj, size_t planeIdx /*= 0*/)
{
    //the base idea behind simple pointer copying (instead of shallow copies or shared pointer)
    // is that AbstractDObjFigure always keeps shallow copies of all data objects and therefore is 
    // responsible that no dataObject is deleted when it is still in use by any object of this entire plot plugin.
    
    m_validData = (dataObj != NULL);
    
    if(dataObj)
    {
        int d = dataObj->getDims();
        m_D.m_yScaling = dataObj->getAxisScale(d-2);
        m_D.m_xScaling = dataObj->getAxisScale(d-1);
        m_D.m_yOffset = dataObj->getAxisOffset(d-2);
        m_D.m_xOffset = dataObj->getAxisOffset(d-1);
        m_D.m_ySize = dataObj->getSize(d-2);
        m_D.m_xSize = dataObj->getSize(d-1);
        m_D.m_dataPtr = NULL; //dataObj->get_mdata();
        m_D.m_planeIdx = planeIdx;

        //check hash
        m_hashGenerator.reset();
        m_hashGenerator.addData( (const char*)&m_D, sizeof(m_D) );
        QByteArray hash = m_hashGenerator.result();

        if(hash != m_dataHash)
        {
            m_dataHash = hash;

            deleteCache();

            //Definition: Scale-Coordinate of dataObject =  ( px-Coordinate - Offset)* Scale
            setInterval(Qt::XAxis, QwtInterval(pxToScaleCoords(0,m_D.m_xOffset,m_D.m_xScaling), pxToScaleCoords(m_D.m_xSize,m_D.m_xOffset,m_D.m_xScaling)) );
            setInterval(Qt::YAxis, QwtInterval(pxToScaleCoords(0,m_D.m_yOffset,m_D.m_yScaling), pxToScaleCoords(m_D.m_ySize,m_D.m_yOffset,m_D.m_yScaling)) );
        }

        ito::float64 min, max;
        ito::uint32 firstMin[3];
        ito::uint32 firstMax[3];
        ito::dObjHelper::minMaxValue(dataObj, min, firstMin, max, firstMax, true);
        setInterval(Qt::ZAxis, QwtInterval(min,max));
    }
    else
    {
        m_D.m_dataPtr = NULL;
        m_D.m_yScaling = (1.0);
        m_D.m_xScaling = (1.0);
        m_D.m_yOffset = (0.0);
        m_D.m_xOffset = (0.0);
        m_D.m_ySize = 0;
        m_D.m_xSize = 0;
        m_D.m_planeIdx = 0;

        m_dataHash = "";
        m_hashGenerator.reset();

        deleteCache();

        setInterval(Qt::XAxis, QwtInterval() );
        setInterval(Qt::YAxis, QwtInterval() );
        setInterval(Qt::ZAxis, QwtInterval() );
    }
    
    m_dataObj = dataObj;
}

//----------------------------------------------------------------------------------------------------------------------------------
double DataObjRasterData::value(double x, double y) const
{
    return std::numeric_limits<double>::signaling_NaN();
}

//----------------------------------------------------------------------------------------------------------------------------------
double DataObjRasterData::value2(int m, int n) const
{
    if(m_validData)
    {
        switch(m_dataObj->getType())
        {
        case ito::tInt8:
            {
                ito::int8 *line = (ito::int8*)m_rasteredLinePtr[m];
                if(!line) return std::numeric_limits<double>::signaling_NaN();
                return line[ m_xIndizes[n] ];
            }
        case ito::tUInt8:
            {
                ito::uint8 *line = (ito::uint8*)m_rasteredLinePtr[m];
                if(!line) return std::numeric_limits<double>::signaling_NaN();
                return line[ m_xIndizes[n] ];
            }
        case ito::tInt16:
            {
                ito::int16 *line = (ito::int16*)m_rasteredLinePtr[m];
                if(!line) return std::numeric_limits<double>::signaling_NaN();
                return line[ m_xIndizes[n] ];
            }
        case ito::tUInt16:
            {
                ito::uint16 *line = (ito::uint16*)m_rasteredLinePtr[m];
                if(!line) return std::numeric_limits<double>::signaling_NaN();
                return line[ m_xIndizes[n] ];
            }
        case ito::tInt32:
            {
                ito::int32 *line = (ito::int32*)m_rasteredLinePtr[m];
                if(!line) return std::numeric_limits<double>::signaling_NaN();
                return line[ m_xIndizes[n] ];
            }
        case ito::tUInt32:
            {
                ito::uint32 *line = (ito::uint32*)m_rasteredLinePtr[m];
                if(!line) return std::numeric_limits<double>::signaling_NaN();
                return line[ m_xIndizes[n] ];
            }
        case ito::tFloat32:
            {
                ito::float32 *line = (ito::float32*)m_rasteredLinePtr[m];
                if(!line) return std::numeric_limits<double>::signaling_NaN();
                return line[ m_xIndizes[n] ];
            }
        case ito::tFloat64:
            {
                ito::float64 *line = (ito::float64*)m_rasteredLinePtr[m];
                if(!line) return std::numeric_limits<double>::signaling_NaN();
                return line[ m_xIndizes[n] ];
            }
        default:
            return std::numeric_limits<double>::signaling_NaN();
        }
    }
    else
    {
        return std::numeric_limits<double>::signaling_NaN();
    }
}

//----------------------------------------------------------------------------------------------------------------------------------
void DataObjRasterData::initRaster( const QRectF& area, const QSize& raster )
{
    if(m_rasteredLinePtr)
    {
        if(m_lastRasteredArea != area || m_lastRasteredRaster != raster)
        {
            m_lastRasteredArea = area;
            m_lastRasteredRaster = raster;
            deleteCache();
        }
    }

    if(m_rasteredLinePtr == NULL) //rebuild cache
    {
        //Definition: Scale-Coordinate of dataObject =  ( px-Coordinate - Offset)* Scale
        //area is in scale coordinates
        qreal left = area.left() / m_D.m_xScaling + m_D.m_xOffset;
        qreal top = area.top() / m_D.m_yScaling + m_D.m_yOffset;
        qreal right = area.right() / m_D.m_xScaling + m_D.m_xOffset;
        qreal bottom = area.bottom() / m_D.m_yScaling + m_D.m_yOffset;
        qreal ySteps = raster.height() > 0 ? (bottom-top) / raster.height() : 0.0;
        qreal xSteps = raster.width() > 0 ? (right-left) / raster.width() : 0.0;

        int j;

        if(m_dataObj)
        {
            m_plane = (cv::Mat*)m_dataObj->get_mdata()[ m_dataObj->seekMat( m_D.m_planeIdx ) ];
            int nrOfLines = raster.height();
            int nrOfCols = raster.width();
        
            m_rasteredLinePtr = new uchar*[nrOfLines];
            for(int i = 0 ; i < nrOfLines ; i++)
            {
                j = qRound( top + ySteps * i );

                if(j >= 0 && j < m_D.m_ySize)
                {
                    m_rasteredLinePtr[i] = m_plane->ptr(j);
                }
                else
                {
                    m_rasteredLinePtr[i] = NULL;
                }
            }

            m_xIndizes = new int[raster.width()];
            for(int i = 0 ; i < nrOfCols ; i++)
            {
                j = qRound( left + xSteps * i );

                if(j >= 0 && j < m_D.m_xSize)
                {
                    m_xIndizes[i] = j;
                }
                else
                {
                    m_xIndizes[i] = -1;
                }
            }

        }
    }
}

//----------------------------------------------------------------------------------------------------------------------------------
void DataObjRasterData::discardRaster()
{
    
}

////----------------------------------------------------------------------------------------------------------------------------------
//DataObjectRasterData::DataObjectRasterData(QSharedPointer<ito::DataObject> dataObj, QList<unsigned int>startPoint, unsigned int wDimIndex, unsigned int width, unsigned int hDimIndex, unsigned int height, bool replotPending) :
//m_wDimIndex(0), m_hDimIndex(0), m_width(0), m_height(0), m_DataObjectWidth(0), m_DataObjectHeight(0), m_xScalingFactor(1), m_yScalingFactor(1),
//		m_xOffset(0), m_yOffset(0), m_cmplxState(0), m_replotPending(replotPending)
//{
//    updateDataObject(dataObj, startPoint, wDimIndex, width, hDimIndex, height);
//
//    if(dataObj.data() != NULL)
//    {
//        setIntervalRange(Qt::ZAxis, true, 0,0);
//    }
//    else
//    {
//        setIntervalRange(Qt::ZAxis, false, 0,255);
//    }
//}
//
////----------------------------------------------------------------------------------------------------------------------------------
//DataObjectRasterData::~DataObjectRasterData()
//{
//    m_dataObjWhileRastering.clear();
//    m_dataObj.clear();
//}
//
////----------------------------------------------------------------------------------------------------------------------------------
//double DataObjectRasterData::value(double x, double y) const
//{
//    if(m_dataObj)
//    {
//        cv::Mat* curPlane = (cv::Mat*)m_dataObj->get_mdata()[m_dataObj->seekMat(0)];
//        switch(m_dataObj->getType())
//        {
//            case ito::tInt8:
//                //return bilinearInterpolation<int8>(m_dataObj.data(), x, y, m_DataObjectWidth, m_DataObjectHeight, m_xScalingFactor, m_xOffset, m_yScalingFactor, m_yOffset);
//                return bilinearInterpolation<int8>(curPlane, x, y, m_DataObjectWidth, m_DataObjectHeight, m_xScalingFactor, m_xOffset, m_yScalingFactor, m_yOffset);
//                break;
//            case ito::tUInt8:
//                return bilinearInterpolation<uint8>(curPlane, x, y, m_DataObjectWidth, m_DataObjectHeight, m_xScalingFactor, m_xOffset, m_yScalingFactor, m_yOffset);
//                break;
//            case ito::tInt16:
//                return bilinearInterpolation<int16>(curPlane, x, y, m_DataObjectWidth, m_DataObjectHeight, m_xScalingFactor, m_xOffset, m_yScalingFactor, m_yOffset);
//                break;
//            case ito::tUInt16:
//                return bilinearInterpolation<uint16>(curPlane, x, y, m_DataObjectWidth, m_DataObjectHeight, m_xScalingFactor, m_xOffset, m_yScalingFactor, m_yOffset);
//                break;
//            case ito::tInt32:
//                return bilinearInterpolation<int32>(curPlane, x, y, m_DataObjectWidth, m_DataObjectHeight, m_xScalingFactor, m_xOffset, m_yScalingFactor, m_yOffset);
//                break;
//            case ito::tUInt32:
//                return bilinearInterpolation<uint32>(curPlane, x, y, m_DataObjectWidth, m_DataObjectHeight, m_xScalingFactor, m_xOffset, m_yScalingFactor, m_yOffset);
//                break;
//            case ito::tFloat32:
//				return bilinearInterpolation<float>(curPlane, x, y, m_DataObjectWidth, m_DataObjectHeight, m_xScalingFactor, m_xOffset, m_yScalingFactor, m_yOffset);
//            break;
//            case ito::tFloat64:
//                return bilinearInterpolation<double>(curPlane, x, y, m_DataObjectWidth, m_DataObjectHeight, m_xScalingFactor, m_xOffset, m_yScalingFactor, m_yOffset);
//                break;
//			case ito::tComplex64:
//				return bilinearInterpolation<ito::complex64>(curPlane, x, y, m_DataObjectWidth, m_DataObjectHeight, m_xScalingFactor, m_xOffset, m_yScalingFactor, m_yOffset, m_cmplxState);
//                break;
//            case ito::tComplex128:
//				return bilinearInterpolation<ito::complex128>(curPlane, x, y, m_DataObjectWidth, m_DataObjectHeight, m_xScalingFactor, m_xOffset, m_yScalingFactor, m_yOffset, m_cmplxState);
//                break;
//            default:
//                return 0;
//        }
//    }
//
//    return 0;
//    //if(m_dataObj)
//    //{
//    //	//double test = m_dataObj->at<uint8>(0,0);
//    //	return m_dataObj->at<uint8>(0,0);
//    //}
//
//        //return x*y;
//}
//
////----------------------------------------------------------------------------------------------------------------------------------
//void DataObjectRasterData::initRaster( const QRectF &, const QSize& /*raster*/ )
//{
//    if(m_dataObj)
//    {
//        //qDebug("raster data: init");
//        m_dataObjWhileRastering = m_dataObj;
//        m_dataObjWhileRastering->lockRead();
//    }
//}
//
////----------------------------------------------------------------------------------------------------------------------------------
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
//
////----------------------------------------------------------------------------------------------------------------------------------
//void DataObjectRasterData::setIntervalRange(Qt::Axis axis, bool autoCalcLimits, double minValue, double maxValue)
//{
//    QwtInterval tempInterval;
//    if(axis == Qt::ZAxis)
//    {
//        if(autoCalcLimits)
//        {
//            if(m_dataObj)
//            {
//                m_dataObj->lockRead();
//                uint32 minPos[3] = {0, 0, 0};
//                uint32 maxPos[3] = {0, 0, 0};
//                ito::dObjHelper::minMaxValue(m_dataObj.data(), minValue, minPos, maxValue, maxPos, true, m_cmplxState);
//				//m_dataObj->getMinMaxValue(minValue, maxValue, true, m_cmplxState);
//                m_dataObj->unlock();
//            }
//        }
//        setInterval(Qt::ZAxis, QwtInterval(minValue, maxValue));
//    }
//    else if(axis == Qt::XAxis)
//    {
//        if(autoCalcLimits)
//        {
//            if(m_dataObj)
//            {
//                bool test = false;
//                m_dataObj->lockRead();
//                minValue = m_dataObj->getPixToPhys(m_hDimIndex, 0.0, test);
//                maxValue = m_dataObj->getPixToPhys(m_hDimIndex, m_dataObj->getSize(m_hDimIndex, true), test);
//                m_dataObj->unlock();
//            }
//        }
//        setInterval(Qt::XAxis, QwtInterval(minValue, maxValue, QwtInterval::IncludeBorders));
//    }
//    else if(axis == Qt::YAxis)
//    {
//        if(autoCalcLimits)
//        {
//            if(m_dataObj)
//            {
//                bool test = false;
//                m_dataObj->lockRead();
//                minValue = m_dataObj->getPixToPhys(m_wDimIndex, 0.0, test);
//                maxValue = m_dataObj->getPixToPhys(m_wDimIndex, m_dataObj->getSize(m_hDimIndex, true), test);
//                m_dataObj->unlock();
//            }
//        }
//        setInterval(Qt::YAxis, QwtInterval(minValue, maxValue, QwtInterval::IncludeBorders));
//    }
//}
//
////----------------------------------------------------------------------------------------------------------------------------------
//void DataObjectRasterData::updateDataObject(QSharedPointer<ito::DataObject> dataObj)
//{
//    m_dataObj = dataObj;
//
//    if(m_dataObj)
//    {
//        m_dataObj->lockRead();
//
//        bool test = true;
//
//        int dimX = dataObj->getDims() - 1;
//        int dimY = dataObj->getDims() - 2;
//
//        double xscale = dataObj->getAxisScale(dimX, test);
//        double yscale = dataObj->getAxisScale(dimY, test);
//
//        setInterval( Qt::XAxis, QwtInterval( dataObj->getPixToPhys(dimX, 0, test), dataObj->getPixToPhys(dimX, m_width, test),  QwtInterval::ExcludeMaximum ) );
//        setInterval( Qt::YAxis, QwtInterval( dataObj->getPixToPhys(dimY, 0, test), dataObj->getPixToPhys(dimY, m_height, test), QwtInterval::ExcludeMaximum ) );
//
//        m_DataObjectWidth = dataObj->getSize(dimX,true); //ly hier bugged was
//        m_DataObjectHeight = dataObj->getSize(dimY,true);//ly hier bugged was
//
//        m_xScalingFactor = ((double) m_DataObjectWidth)/(m_width * xscale);
//        m_yScalingFactor = ((double) m_DataObjectHeight)/(m_height * yscale);
//
//        m_xOffset = dataObj->getAxisOffset(dimX, test);
//        m_yOffset = dataObj->getAxisOffset(dimY, test);
//
//        m_dataObj->unlock();
//    }
//}
//
////----------------------------------------------------------------------------------------------------------------------------------
//void DataObjectRasterData::updateDataObject(QSharedPointer<ito::DataObject> dataObj, QList<unsigned int>startPoint, unsigned int wDimIndex, unsigned int width, unsigned int hDimIndex, unsigned int height)
//{
//    m_startPoint = startPoint;
//    m_wDimIndex = wDimIndex;
//    m_width = width;
//    m_hDimIndex = hDimIndex;
//    m_height = height;
//
//    updateDataObject(dataObj);
//}