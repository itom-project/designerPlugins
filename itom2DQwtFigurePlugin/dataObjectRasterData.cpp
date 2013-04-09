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

#include "dataObjectRasterData.h"
#include "common/typeDefs.h"

#include "DataObject/dataObjectFuncs.h"

#include <qdebug.h>

#define FAST_VALUES // Warning other interpolationmode does not tested with offsets at the moment

using namespace ito;

//----------------------------------------------------------------------------------------------------------------------------------
template<class T> double bilinearInterpolation(cv::Mat* data, double x, double y, int width, int height, double xScalingFactor, double xOffset, double yScalingFactor, double yOffset, const int cmplxState /*= -1*/)
{

    int xDOIndex = (int)(x*xScalingFactor + xOffset) < width-1 ? (int)(x*xScalingFactor + xOffset) : width-1;
    int yDOIndex = (int)(y*yScalingFactor + yOffset) < height-1 ? (int)(y*yScalingFactor + yOffset) : height-1;

    if(xDOIndex < 0) xDOIndex = 0;
    if(yDOIndex < 0) yDOIndex = 0;

#ifdef FAST_VALUES
    return data->at<T>(yDOIndex, xDOIndex);
#else
    double upperLeft, upperRight, lowerLeft, lowerRight; //the values of the four bilinear interpolation pixels

    double deltaX = x*xScalingFactor - ((double) ((int) x*xScalingFactor));
    double deltaY= y*yScalingFactor - ((double) ((int) y*yScalingFactor));
    double tmp1, tmp2;

    upperLeft=data->at<T>(yDOIndex, xDOIndex);
    if(xDOIndex < width-1) //not at the right edge of the image
    {
        //not at the right edge of the image
        upperRight = data->at<T>(yDOIndex, xDOIndex+1);
        if(yDOIndex < height-1)
        {
            //not at the lower edge of the image, everything is fine, interpolate between all four pixels
            lowerLeft = data->at<T>(yDOIndex+1, xDOIndex);
            lowerRight = data->at<T>(yDOIndex+1, xDOIndex+1);
            tmp1 = upperLeft + deltaX * (upperRight - upperLeft);
            tmp2 = lowerLeft + deltaX * (lowerRight - lowerLeft);
            return tmp1 + deltaY * (tmp2-tmp1);
        }
        else
        {
            //at the lower edge of the image, no pixel below, we just need to interpolate between the pixel to the left and the right
            return upperLeft + deltaX * (upperRight - upperLeft);
        }
    }
    else
    {
        //at the right edge of the image, no pixel to the right
        if(yDOIndex < height-1)
        {
            //no pixel to the right, we just need to interpolate between the pixel above and below
            lowerLeft = data->at<T>(yDOIndex+1, xDOIndex);
            return upperLeft + deltaY * (lowerLeft-upperLeft);
        }
        else
        {
            //in the lower right corner of the image, no surrounding pixels
            return upperLeft;
        }
    }
#endif
}

//----------------------------------------------------------------------------------------------------------------------------------
template<> double bilinearInterpolation<ito::complex64>(cv::Mat* data, double x, double y, int width, int height, double xScalingFactor, double xOffset, double yScalingFactor, double yOffset, int cmplxState)
{
    int xDOIndex = (int)(x*xScalingFactor + xOffset) < width-1 ? (int)(x*xScalingFactor + xOffset) : width-1;
    int yDOIndex = (int)(y*yScalingFactor + yOffset) < height-1 ? (int)(y*yScalingFactor + yOffset) : height-1;

    if(xDOIndex < 0) xDOIndex = 0;
    if(yDOIndex < 0) yDOIndex = 0;

#ifdef FAST_VALUES
	switch (cmplxState)
	{
		default:
		case 0:
			return abs(data->at<ito::complex64>(yDOIndex, xDOIndex));
		break;

		case 1:
			return (data->at<ito::complex64>(yDOIndex, xDOIndex)).imag();
		break;

		case 2:
			return (data->at<ito::complex64>(yDOIndex, xDOIndex)).real();
		break;

		case 3:
			return arg(data->at<ito::complex64>(yDOIndex, xDOIndex));
		break;
	}
#endif
}

//----------------------------------------------------------------------------------------------------------------------------------
template<> double bilinearInterpolation<ito::complex128>(cv::Mat* data, double x, double y, int width, int height, double xScalingFactor, double xOffset, double yScalingFactor, double yOffset, int cmplxState)
{
    int xDOIndex = (int)(x*xScalingFactor + xOffset) < width-1 ? (int)(x*xScalingFactor + xOffset) : width-1;
    int yDOIndex = (int)(y*yScalingFactor + yOffset) < height-1 ? (int)(y*yScalingFactor + yOffset) : height-1;

    if(xDOIndex < 0) xDOIndex = 0;
    if(yDOIndex < 0) yDOIndex = 0;

#ifdef FAST_VALUES
	switch (cmplxState)
	{
		default:
		case 0:
			return abs(data->at<ito::complex128>(yDOIndex, xDOIndex));
		break;

		case 1:
			return (data->at<ito::complex128>(yDOIndex, xDOIndex)).imag();
		break;

		case 2:
			return (data->at<ito::complex128>(yDOIndex, xDOIndex)).real();
		break;

		case 3:
			return arg(data->at<ito::complex128>(yDOIndex, xDOIndex));
		break;
	}
#endif
}

//----------------------------------------------------------------------------------------------------------------------------------
DataObjectRasterData::DataObjectRasterData(QSharedPointer<ito::DataObject> dataObj, QList<unsigned int>startPoint, unsigned int wDimIndex, unsigned int width, unsigned int hDimIndex, unsigned int height, bool replotPending) :
m_wDimIndex(0), m_hDimIndex(0), m_width(0), m_height(0), m_DataObjectWidth(0), m_DataObjectHeight(0), m_xScalingFactor(1), m_yScalingFactor(1),
		m_xOffset(0), m_yOffset(0), m_cmplxState(0), m_replotPending(replotPending)
{
    updateDataObject(dataObj, startPoint, wDimIndex, width, hDimIndex, height);

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
DataObjectRasterData::~DataObjectRasterData()
{
    m_dataObjWhileRastering.clear();
    m_dataObj.clear();
}

//----------------------------------------------------------------------------------------------------------------------------------
double DataObjectRasterData::value(double x, double y) const
{
    if(m_dataObj)
    {
        cv::Mat* curPlane = (cv::Mat*)m_dataObj->get_mdata()[m_dataObj->seekMat(0)];
        switch(m_dataObj->getType())
        {
            case ito::tInt8:
                //return bilinearInterpolation<int8>(m_dataObj.data(), x, y, m_DataObjectWidth, m_DataObjectHeight, m_xScalingFactor, m_xOffset, m_yScalingFactor, m_yOffset);
                return bilinearInterpolation<int8>(curPlane, x, y, m_DataObjectWidth, m_DataObjectHeight, m_xScalingFactor, m_xOffset, m_yScalingFactor, m_yOffset);
                break;
            case ito::tUInt8:
                return bilinearInterpolation<uint8>(curPlane, x, y, m_DataObjectWidth, m_DataObjectHeight, m_xScalingFactor, m_xOffset, m_yScalingFactor, m_yOffset);
                break;
            case ito::tInt16:
                return bilinearInterpolation<int16>(curPlane, x, y, m_DataObjectWidth, m_DataObjectHeight, m_xScalingFactor, m_xOffset, m_yScalingFactor, m_yOffset);
                break;
            case ito::tUInt16:
                return bilinearInterpolation<uint16>(curPlane, x, y, m_DataObjectWidth, m_DataObjectHeight, m_xScalingFactor, m_xOffset, m_yScalingFactor, m_yOffset);
                break;
            case ito::tInt32:
                return bilinearInterpolation<int32>(curPlane, x, y, m_DataObjectWidth, m_DataObjectHeight, m_xScalingFactor, m_xOffset, m_yScalingFactor, m_yOffset);
                break;
            case ito::tUInt32:
                return bilinearInterpolation<uint32>(curPlane, x, y, m_DataObjectWidth, m_DataObjectHeight, m_xScalingFactor, m_xOffset, m_yScalingFactor, m_yOffset);
                break;
            case ito::tFloat32:
				return bilinearInterpolation<float>(curPlane, x, y, m_DataObjectWidth, m_DataObjectHeight, m_xScalingFactor, m_xOffset, m_yScalingFactor, m_yOffset);
            break;
            case ito::tFloat64:
                return bilinearInterpolation<double>(curPlane, x, y, m_DataObjectWidth, m_DataObjectHeight, m_xScalingFactor, m_xOffset, m_yScalingFactor, m_yOffset);
                break;
			case ito::tComplex64:
				return bilinearInterpolation<ito::complex64>(curPlane, x, y, m_DataObjectWidth, m_DataObjectHeight, m_xScalingFactor, m_xOffset, m_yScalingFactor, m_yOffset, m_cmplxState);
                break;
            case ito::tComplex128:
				return bilinearInterpolation<ito::complex128>(curPlane, x, y, m_DataObjectWidth, m_DataObjectHeight, m_xScalingFactor, m_xOffset, m_yScalingFactor, m_yOffset, m_cmplxState);
                break;
            default:
                return 0;
        }
    }

    return 0;
    //if(m_dataObj)
    //{
    //	//double test = m_dataObj->at<uint8>(0,0);
    //	return m_dataObj->at<uint8>(0,0);
    //}

        //return x*y;
}

//----------------------------------------------------------------------------------------------------------------------------------
void DataObjectRasterData::initRaster( const QRectF &, const QSize& /*raster*/ )
{
    if(m_dataObj)
    {
        //qDebug("raster data: init");
        m_dataObjWhileRastering = m_dataObj;
        m_dataObjWhileRastering->lockRead();
    }
}

//----------------------------------------------------------------------------------------------------------------------------------
void DataObjectRasterData::discardRaster()
{
    if(m_dataObjWhileRastering)
    {
        m_dataObjWhileRastering->unlock();
        m_dataObjWhileRastering.clear();
        //qDebug("raster data: end");
        //if(m_pReplotPending) *m_pReplotPending = false;
    }
}

//----------------------------------------------------------------------------------------------------------------------------------
void DataObjectRasterData::setIntervalRange(Qt::Axis axis, bool autoCalcLimits, double minValue, double maxValue)
{
    QwtInterval tempInterval;
    if(axis == Qt::ZAxis)
    {
        if(autoCalcLimits)
        {
            if(m_dataObj)
            {
                m_dataObj->lockRead();
                uint32 minPos[3] = {0, 0, 0};
                uint32 maxPos[3] = {0, 0, 0};
                ito::dObjHelper::minMaxValue(m_dataObj.data(), minValue, minPos, maxValue, maxPos, true, m_cmplxState);
				//m_dataObj->getMinMaxValue(minValue, maxValue, true, m_cmplxState);
                m_dataObj->unlock();
            }
        }
        setInterval(Qt::ZAxis, QwtInterval(minValue, maxValue));
    }
    else if(axis == Qt::XAxis)
    {
        if(autoCalcLimits)
        {
            if(m_dataObj)
            {
                bool test = false;
                m_dataObj->lockRead();
                minValue = m_dataObj->getPixToPhys(m_hDimIndex, 0.0, test);
                maxValue = m_dataObj->getPixToPhys(m_hDimIndex, m_dataObj->getSize(m_hDimIndex), test);
                m_dataObj->unlock();
            }
        }
        setInterval(Qt::XAxis, QwtInterval(minValue, maxValue, QwtInterval::IncludeBorders));
    }
    else if(axis == Qt::YAxis)
    {
        if(autoCalcLimits)
        {
            if(m_dataObj)
            {
                bool test = false;
                m_dataObj->lockRead();
                minValue = m_dataObj->getPixToPhys(m_wDimIndex, 0.0, test);
                maxValue = m_dataObj->getPixToPhys(m_wDimIndex, m_dataObj->getSize(m_hDimIndex), test);
                m_dataObj->unlock();
            }
        }
        setInterval(Qt::YAxis, QwtInterval(minValue, maxValue, QwtInterval::IncludeBorders));
    }
}

//----------------------------------------------------------------------------------------------------------------------------------
void DataObjectRasterData::updateDataObject(QSharedPointer<ito::DataObject> dataObj)
{
    m_dataObj = dataObj;

    if(m_dataObj)
    {
        m_dataObj->lockRead();

        bool test = true;

        int dimX = dataObj->getDims() - 1;
        int dimY = dataObj->getDims() - 2;

        double xscale = dataObj->getAxisScale(dimX);
        double yscale = dataObj->getAxisScale(dimY);

        setInterval( Qt::XAxis, QwtInterval( dataObj->getPixToPhys(dimX, 0, test), dataObj->getPixToPhys(dimX, m_width, test),  QwtInterval::ExcludeMaximum ) );
        setInterval( Qt::YAxis, QwtInterval( dataObj->getPixToPhys(dimY, 0, test), dataObj->getPixToPhys(dimY, m_height, test), QwtInterval::ExcludeMaximum ) );

        m_DataObjectWidth = dataObj->getSize(dimX); //ly hier bugged was
        m_DataObjectHeight = dataObj->getSize(dimY);//ly hier bugged was

        m_xScalingFactor = ((double) m_DataObjectWidth)/(m_width * xscale);
        m_yScalingFactor = ((double) m_DataObjectHeight)/(m_height * yscale);

        m_xOffset = dataObj->getAxisOffset(dimX);
        m_yOffset = dataObj->getAxisOffset(dimY);

        m_dataObj->unlock();
    }
}

//----------------------------------------------------------------------------------------------------------------------------------
void DataObjectRasterData::updateDataObject(QSharedPointer<ito::DataObject> dataObj, QList<unsigned int>startPoint, unsigned int wDimIndex, unsigned int width, unsigned int hDimIndex, unsigned int height)
{
    m_startPoint = startPoint;
    m_wDimIndex = wDimIndex;
    m_width = width;
    m_hDimIndex = hDimIndex;
    m_height = height;

    updateDataObject(dataObj);
}

//----------------------------------------------------------------------------------------------------------------------------------
template double bilinearInterpolation<int8>(cv::Mat* data, double x, double y, int width, int height, double xScalingFactor, double xOffset, double yScalingFactor, double yOffset, const int cmplxState);
template double bilinearInterpolation<uint8>(cv::Mat* data, double x, double y, int width, int height, double xScalingFactor, double xOffset, double yScalingFactor, double yOffset, const int cmplxState);
template double bilinearInterpolation<int16>(cv::Mat* data, double x, double y, int width, int height, double xScalingFactor, double xOffset, double yScalingFactor, double yOffset, const int cmplxState);
template double bilinearInterpolation<uint16>(cv::Mat* data, double x, double y, int width, int height, double xScalingFactor, double xOffset, double yScalingFactor, double yOffset, const int cmplxState);
template double bilinearInterpolation<int32>(cv::Mat* data, double x, double y, int width, int height, double xScalingFactor, double xOffset, double yScalingFactor, double yOffset, const int cmplxState);
template double bilinearInterpolation<uint32>(cv::Mat* data, double x, double y, int width, int height, double xScalingFactor, double xOffset, double yScalingFactor, double yOffset, const int cmplxState);
template double bilinearInterpolation<float>(cv::Mat* data, double x, double y, int width, int height, double xScalingFactor, double xOffset, double yScalingFactor, double yOffset, const int cmplxState);
template double bilinearInterpolation<double>(cv::Mat* data, double x, double y, int width, int height, double xScalingFactor, double xOffset, double yScalingFactor, double yOffset, const int cmplxState);
//template double bilinearInterpolation<ito::complex64>(cv::Mat* data, double x, double y, int width, int height, double xScalingFactor, double xOffset, double yScalingFactor, double yOffset, const int cmplxState);
//template double bilinearInterpolation<ito::complex128>(cv::Mat* data, double x, double y, int width, int height, double xScalingFactor, double xOffset, double yScalingFactor, double yOffset, const int cmplxState);

//----------------------------------------------------------------------------------------------------------------------------------
