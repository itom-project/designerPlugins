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

#include "dataObjectSeriesData.h"
#include "common/typeDefs.h"

#include <qdebug.h>
#include <qnumeric.h>

//----------------------------------------------------------------------------------------------------------------------------------
DataObjectSeriesData::DataObjectSeriesData(QSharedPointer<ito::DataObject> dataObj, int startPoint, unsigned int plotDim, unsigned int size, const int fastmode) :
    m_zDirect(false),
    m_fast(fastmode),
    m_startPos(0),
    m_physLength(size),
    m_Scaling(1.0),
    m_autoScaleY(true),
    m_minY(-1.0),
    m_maxY(2.0),
    m_autoScaleX(true),
    m_minX(-1.0),
    m_maxX(1.0),
    m_cmplxState(0)
{
    updateDataObject(dataObj, startPoint, plotDim, size);
}

//----------------------------------------------------------------------------------------------------------------------------------
DataObjectSeriesData::DataObjectSeriesData(QSharedPointer<ito::DataObject> dataObj, QVector<QPointF> pts, const int fastmode) :
    m_zDirect(false),
    m_fast(fastmode),
    m_startPos(0),
    m_physLength(0),
    m_Scaling(1.0),
    m_autoScaleY(true),
    m_minY(-1.0),
    m_maxY(2.0),
    m_autoScaleX(true),
    m_minX(-1.0),
    m_maxX(1.0),
    m_cmplxState(0)
{
    updateDataObject(dataObj, pts);
}

//----------------------------------------------------------------------------------------------------------------------------------
DataObjectSeriesData::~DataObjectSeriesData()
{
    m_dataObjWhileRastering.clear();
    m_dataObj.clear();
    m_plotPts.clear();
    m_points.clear();
}

//----------------------------------------------------------------------------------------------------------------------------------
size_t DataObjectSeriesData::size() const
{
    return m_size;
}
//----------------------------------------------------------------------------------------------------------------------------------
int DataObjectSeriesData::getPosToPix(const double phys)
{
    int value = 0;
    if (m_Scaling > 0) value = (int)((phys - m_startPos) / m_Scaling + 0.5);
    else value = (int)((phys - m_startPos  - m_physLength) / abs(m_Scaling) + 0.5);

    value = value > (m_plotPts.size()-1) ? (m_plotPts.size()-1) : value < 0 ? 0 : value;

    return value;
}
//----------------------------------------------------------------------------------------------------------------------------------
QPointF DataObjectSeriesData::sample(size_t n) const
{
	double rVal, iVal;



    if(m_dataObjWhileRastering && m_dataObjWhileRastering->getDims() == 2)
    {
        double fPos;

        if (m_Scaling > 0) fPos = n * m_Scaling + m_startPos;
        else fPos = n * abs(m_Scaling) + m_startPos  + m_physLength;

        if (m_fast)
        {
            switch(m_dataObjWhileRastering->getType())
            {
                case ito::tInt8:
                    return QPointF(fPos, m_dataObj->at<int8>(m_plotPts[n].rangeY[0], m_plotPts[n].rangeX[0]));
                break;
                case ito::tUInt8:
                    return QPointF(fPos, m_dataObj->at<uint8>(m_plotPts[n].rangeY[0], m_plotPts[n].rangeX[0]));
                break;
                case ito::tInt16:
                    return QPointF(fPos, m_dataObj->at<int16>(m_plotPts[n].rangeY[0], m_plotPts[n].rangeX[0]));
                break;
                case ito::tUInt16:
                    return QPointF(fPos, m_dataObj->at<uint16>(m_plotPts[n].rangeY[0], m_plotPts[n].rangeX[0]));
                break;
                case ito::tInt32:
                    return QPointF(fPos, m_dataObj->at<int32>(m_plotPts[n].rangeY[0], m_plotPts[n].rangeX[0]));
                break;
                case ito::tUInt32:
                    return QPointF(fPos, m_dataObj->at<uint32>(m_plotPts[n].rangeY[0], m_plotPts[n].rangeX[0]));
                break;
                case ito::tFloat32:
                    return QPointF(fPos, m_dataObj->at<float>(m_plotPts[n].rangeY[0], m_plotPts[n].rangeX[0]));
                break;
                case ito::tFloat64:
                    return QPointF(fPos, m_dataObj->at<double>(m_plotPts[n].rangeY[0], m_plotPts[n].rangeX[0]));
                break;
                case ito::tComplex64:
				{
					switch (m_cmplxState)
					{
						default:
						case 0:
							return QPointF(fPos, abs(m_dataObj->at<ito::complex64>(m_plotPts[n].rangeY[0], m_plotPts[n].rangeX[0])));
						break;
						case 1:
							return QPointF(fPos, (m_dataObj->at<ito::complex64>(m_plotPts[n].rangeY[0], m_plotPts[n].rangeX[0]).real()));
						break;
						case 2:
							return QPointF(fPos, (m_dataObj->at<ito::complex64>(m_plotPts[n].rangeY[0], m_plotPts[n].rangeX[0]).imag()));
						break;
						case 3:
							return QPointF(fPos, arg(m_dataObj->at<ito::complex64>(m_plotPts[n].rangeY[0], m_plotPts[n].rangeX[0])));
						break;
					}
				}
                break;
                case ito::tComplex128:
				{
					switch (m_cmplxState)
					{
						default:
						case 0:
							return QPointF(fPos, abs(m_dataObj->at<ito::complex128>(m_plotPts[n].rangeY[0], m_plotPts[n].rangeX[0])));
						break;
						case 1:
							return QPointF(fPos, (m_dataObj->at<ito::complex128>(m_plotPts[n].rangeY[0], m_plotPts[n].rangeX[0]).real()));
						break;
						case 2:
							return QPointF(fPos, (m_dataObj->at<ito::complex128>(m_plotPts[n].rangeY[0], m_plotPts[n].rangeX[0]).imag()));
						break;
						case 3:
							return QPointF(fPos, arg(m_dataObj->at<ito::complex128>(m_plotPts[n].rangeY[0], m_plotPts[n].rangeX[0])));
						break;
					}
				}
                break;
            }
        }
        else // if m_fast
        {
            double val;

            switch(m_dataObjWhileRastering->getType())
            {
                case ito::tInt8:
                    val = m_dataObj->at<int8>(m_plotPts[n].rangeY[0], m_plotPts[n].rangeX[0]) * m_plotPts[n].weights[0] +
                        m_dataObj->at<int8>(m_plotPts[n].rangeY[1], m_plotPts[n].rangeX[0]) * m_plotPts[n].weights[1] +
                        m_dataObj->at<int8>(m_plotPts[n].rangeY[1], m_plotPts[n].rangeX[1]) * m_plotPts[n].weights[2] +
                        m_dataObj->at<int8>(m_plotPts[n].rangeY[0], m_plotPts[n].rangeX[1]) * m_plotPts[n].weights[3];
                break;
                case ito::tUInt8:
                    val = m_dataObj->at<uint8>(m_plotPts[n].rangeY[0], m_plotPts[n].rangeX[0]) * m_plotPts[n].weights[0] +
                        m_dataObj->at<uint8>(m_plotPts[n].rangeY[1], m_plotPts[n].rangeX[0]) * m_plotPts[n].weights[1] +
                        m_dataObj->at<uint8>(m_plotPts[n].rangeY[1], m_plotPts[n].rangeX[1]) * m_plotPts[n].weights[2] +
                        m_dataObj->at<uint8>(m_plotPts[n].rangeY[0], m_plotPts[n].rangeX[1]) * m_plotPts[n].weights[3];
                break;
                case ito::tInt16:
                    val = m_dataObj->at<int16>(m_plotPts[n].rangeY[0], m_plotPts[n].rangeX[0]) * m_plotPts[n].weights[0] +
                        m_dataObj->at<int16>(m_plotPts[n].rangeY[1], m_plotPts[n].rangeX[0]) * m_plotPts[n].weights[1] +
                        m_dataObj->at<int16>(m_plotPts[n].rangeY[1], m_plotPts[n].rangeX[1]) * m_plotPts[n].weights[2] +
                        m_dataObj->at<int16>(m_plotPts[n].rangeY[0], m_plotPts[n].rangeX[1]) * m_plotPts[n].weights[3];
                break;
                case ito::tUInt16:
                    val = m_dataObj->at<uint16>(m_plotPts[n].rangeY[0], m_plotPts[n].rangeX[0]) * m_plotPts[n].weights[0] +
                        m_dataObj->at<uint16>(m_plotPts[n].rangeY[1], m_plotPts[n].rangeX[0]) * m_plotPts[n].weights[1] +
                        m_dataObj->at<uint16>(m_plotPts[n].rangeY[1], m_plotPts[n].rangeX[1]) * m_plotPts[n].weights[2] +
                        m_dataObj->at<uint16>(m_plotPts[n].rangeY[0], m_plotPts[n].rangeX[1]) * m_plotPts[n].weights[3];
                break;
                case ito::tInt32:
                    val = m_dataObj->at<int32>(m_plotPts[n].rangeY[0], m_plotPts[n].rangeX[0]) * m_plotPts[n].weights[0] +
                        m_dataObj->at<int32>(m_plotPts[n].rangeY[1], m_plotPts[n].rangeX[0]) * m_plotPts[n].weights[1] +
                        m_dataObj->at<int32>(m_plotPts[n].rangeY[1], m_plotPts[n].rangeX[1]) * m_plotPts[n].weights[2] +
                        m_dataObj->at<int32>(m_plotPts[n].rangeY[0], m_plotPts[n].rangeX[1]) * m_plotPts[n].weights[3];
                break;
                case ito::tUInt32:
                    val = m_dataObj->at<uint32>(m_plotPts[n].rangeY[0], m_plotPts[n].rangeX[0]) * m_plotPts[n].weights[0] +
                        m_dataObj->at<uint32>(m_plotPts[n].rangeY[1], m_plotPts[n].rangeX[0]) * m_plotPts[n].weights[1] +
                        m_dataObj->at<uint32>(m_plotPts[n].rangeY[1], m_plotPts[n].rangeX[1]) * m_plotPts[n].weights[2] +
                        m_dataObj->at<uint32>(m_plotPts[n].rangeY[0], m_plotPts[n].rangeX[1]) * m_plotPts[n].weights[3];
                break;
                case ito::tFloat32:
                     val = m_dataObj->at<float>(m_plotPts[n].rangeY[0], m_plotPts[n].rangeX[0]) * m_plotPts[n].weights[0] +
                        m_dataObj->at<float>(m_plotPts[n].rangeY[1], m_plotPts[n].rangeX[0]) * m_plotPts[n].weights[1] +
                        m_dataObj->at<float>(m_plotPts[n].rangeY[1], m_plotPts[n].rangeX[1]) * m_plotPts[n].weights[2] +
                        m_dataObj->at<float>(m_plotPts[n].rangeY[0], m_plotPts[n].rangeX[1]) * m_plotPts[n].weights[3];
                break;
                case ito::tFloat64:
                     val = m_dataObj->at<double>(m_plotPts[n].rangeY[0], m_plotPts[n].rangeX[0]) * m_plotPts[n].weights[0] +
                        m_dataObj->at<double>(m_plotPts[n].rangeY[1], m_plotPts[n].rangeX[0]) * m_plotPts[n].weights[1] +
                        m_dataObj->at<double>(m_plotPts[n].rangeY[1], m_plotPts[n].rangeX[1]) * m_plotPts[n].weights[2] +
                        m_dataObj->at<double>(m_plotPts[n].rangeY[0], m_plotPts[n].rangeX[1]) * m_plotPts[n].weights[3];
                break;
                case ito::tComplex64:
				{
					switch (m_cmplxState)
					{
						default:
						case 0:
							rVal = m_dataObj->at<ito::complex64>(m_plotPts[n].rangeY[0], m_plotPts[n].rangeX[0]).real() * m_plotPts[n].weights[0] +
								m_dataObj->at<ito::complex64>(m_plotPts[n].rangeY[1], m_plotPts[n].rangeX[0]).real() * m_plotPts[n].weights[1] +
								m_dataObj->at<ito::complex64>(m_plotPts[n].rangeY[1], m_plotPts[n].rangeX[1]).real() * m_plotPts[n].weights[2] +
								m_dataObj->at<ito::complex64>(m_plotPts[n].rangeY[0], m_plotPts[n].rangeX[1]).real() * m_plotPts[n].weights[3];
							iVal = m_dataObj->at<ito::complex64>(m_plotPts[n].rangeY[0], m_plotPts[n].rangeX[0]).imag() * m_plotPts[n].weights[0] +
								m_dataObj->at<ito::complex64>(m_plotPts[n].rangeY[1], m_plotPts[n].rangeX[0]).imag() * m_plotPts[n].weights[1] +
								m_dataObj->at<ito::complex64>(m_plotPts[n].rangeY[1], m_plotPts[n].rangeX[1]).imag() * m_plotPts[n].weights[2] +
								m_dataObj->at<ito::complex64>(m_plotPts[n].rangeY[0], m_plotPts[n].rangeX[1]).imag() * m_plotPts[n].weights[3];
							val = sqrt(rVal * rVal + iVal * iVal);
						break;
						case 1:
							val = m_dataObj->at<ito::complex64>(m_plotPts[n].rangeY[0], m_plotPts[n].rangeX[0]).real() * m_plotPts[n].weights[0] +
								m_dataObj->at<ito::complex64>(m_plotPts[n].rangeY[1], m_plotPts[n].rangeX[0]).real() * m_plotPts[n].weights[1] +
								m_dataObj->at<ito::complex64>(m_plotPts[n].rangeY[1], m_plotPts[n].rangeX[1]).real() * m_plotPts[n].weights[2] +
								m_dataObj->at<ito::complex64>(m_plotPts[n].rangeY[0], m_plotPts[n].rangeX[1]).real() * m_plotPts[n].weights[3];
						break;
						case 2:
							val = m_dataObj->at<ito::complex64>(m_plotPts[n].rangeY[0], m_plotPts[n].rangeX[0]).imag() * m_plotPts[n].weights[0] +
								m_dataObj->at<ito::complex64>(m_plotPts[n].rangeY[1], m_plotPts[n].rangeX[0]).imag() * m_plotPts[n].weights[1] +
								m_dataObj->at<ito::complex64>(m_plotPts[n].rangeY[1], m_plotPts[n].rangeX[1]).imag() * m_plotPts[n].weights[2] +
								m_dataObj->at<ito::complex64>(m_plotPts[n].rangeY[0], m_plotPts[n].rangeX[1]).imag() * m_plotPts[n].weights[3];
						break;
						case 3:
							rVal = m_dataObj->at<ito::complex64>(m_plotPts[n].rangeY[0], m_plotPts[n].rangeX[0]).real() * m_plotPts[n].weights[0] +
								m_dataObj->at<ito::complex64>(m_plotPts[n].rangeY[1], m_plotPts[n].rangeX[0]).real() * m_plotPts[n].weights[1] +
								m_dataObj->at<ito::complex64>(m_plotPts[n].rangeY[1], m_plotPts[n].rangeX[1]).real() * m_plotPts[n].weights[2] +
								m_dataObj->at<ito::complex64>(m_plotPts[n].rangeY[0], m_plotPts[n].rangeX[1]).real() * m_plotPts[n].weights[3];
							iVal = m_dataObj->at<ito::complex64>(m_plotPts[n].rangeY[0], m_plotPts[n].rangeX[0]).imag() * m_plotPts[n].weights[0] +
								m_dataObj->at<ito::complex64>(m_plotPts[n].rangeY[1], m_plotPts[n].rangeX[0]).imag() * m_plotPts[n].weights[1] +
								m_dataObj->at<ito::complex64>(m_plotPts[n].rangeY[1], m_plotPts[n].rangeX[1]).imag() * m_plotPts[n].weights[2] +
								m_dataObj->at<ito::complex64>(m_plotPts[n].rangeY[0], m_plotPts[n].rangeX[1]).imag() * m_plotPts[n].weights[3];
							val = atan2(iVal, rVal);
						break;
					}
				}
                break;
                case ito::tComplex128:
				{
					switch (m_cmplxState)
					{
						default:
						case 0:
							rVal = m_dataObj->at<ito::complex128>(m_plotPts[n].rangeY[0], m_plotPts[n].rangeX[0]).real() * m_plotPts[n].weights[0] +
								m_dataObj->at<ito::complex128>(m_plotPts[n].rangeY[1], m_plotPts[n].rangeX[0]).real() * m_plotPts[n].weights[1] +
								m_dataObj->at<ito::complex128>(m_plotPts[n].rangeY[1], m_plotPts[n].rangeX[1]).real() * m_plotPts[n].weights[2] +
								m_dataObj->at<ito::complex128>(m_plotPts[n].rangeY[0], m_plotPts[n].rangeX[1]).real() * m_plotPts[n].weights[3];
							iVal = m_dataObj->at<ito::complex128>(m_plotPts[n].rangeY[0], m_plotPts[n].rangeX[0]).imag() * m_plotPts[n].weights[0] +
								m_dataObj->at<ito::complex128>(m_plotPts[n].rangeY[1], m_plotPts[n].rangeX[0]).imag() * m_plotPts[n].weights[1] +
								m_dataObj->at<ito::complex128>(m_plotPts[n].rangeY[1], m_plotPts[n].rangeX[1]).imag() * m_plotPts[n].weights[2] +
								m_dataObj->at<ito::complex128>(m_plotPts[n].rangeY[0], m_plotPts[n].rangeX[1]).imag() * m_plotPts[n].weights[3];
							val = sqrt(rVal * rVal + iVal * iVal);
						break;
						case 1:
							val = m_dataObj->at<ito::complex128>(m_plotPts[n].rangeY[0], m_plotPts[n].rangeX[0]).real() * m_plotPts[n].weights[0] +
								m_dataObj->at<ito::complex128>(m_plotPts[n].rangeY[1], m_plotPts[n].rangeX[0]).real() * m_plotPts[n].weights[1] +
								m_dataObj->at<ito::complex128>(m_plotPts[n].rangeY[1], m_plotPts[n].rangeX[1]).real() * m_plotPts[n].weights[2] +
								m_dataObj->at<ito::complex128>(m_plotPts[n].rangeY[0], m_plotPts[n].rangeX[1]).real() * m_plotPts[n].weights[3];
						break;
						case 2:
							val = m_dataObj->at<ito::complex128>(m_plotPts[n].rangeY[0], m_plotPts[n].rangeX[0]).imag() * m_plotPts[n].weights[0] +
								m_dataObj->at<ito::complex128>(m_plotPts[n].rangeY[1], m_plotPts[n].rangeX[0]).imag() * m_plotPts[n].weights[1] +
								m_dataObj->at<ito::complex128>(m_plotPts[n].rangeY[1], m_plotPts[n].rangeX[1]).imag() * m_plotPts[n].weights[2] +
								m_dataObj->at<ito::complex128>(m_plotPts[n].rangeY[0], m_plotPts[n].rangeX[1]).imag() * m_plotPts[n].weights[3];
						break;
						case 3:
							rVal = m_dataObj->at<ito::complex128>(m_plotPts[n].rangeY[0], m_plotPts[n].rangeX[0]).real() * m_plotPts[n].weights[0] +
								m_dataObj->at<ito::complex128>(m_plotPts[n].rangeY[1], m_plotPts[n].rangeX[0]).real() * m_plotPts[n].weights[1] +
								m_dataObj->at<ito::complex128>(m_plotPts[n].rangeY[1], m_plotPts[n].rangeX[1]).real() * m_plotPts[n].weights[2] +
								m_dataObj->at<ito::complex128>(m_plotPts[n].rangeY[0], m_plotPts[n].rangeX[1]).real() * m_plotPts[n].weights[3];
							iVal = m_dataObj->at<ito::complex128>(m_plotPts[n].rangeY[0], m_plotPts[n].rangeX[0]).imag() * m_plotPts[n].weights[0] +
								m_dataObj->at<ito::complex128>(m_plotPts[n].rangeY[1], m_plotPts[n].rangeX[0]).imag() * m_plotPts[n].weights[1] +
								m_dataObj->at<ito::complex128>(m_plotPts[n].rangeY[1], m_plotPts[n].rangeX[1]).imag() * m_plotPts[n].weights[2] +
								m_dataObj->at<ito::complex128>(m_plotPts[n].rangeY[0], m_plotPts[n].rangeX[1]).imag() * m_plotPts[n].weights[3];
							val = atan2(iVal, rVal);
						break;
					}
				}
                break;
            }
            return QPointF(fPos, val);
        }
    }
    else if(m_dataObjWhileRastering) // if m_dataObjWhileRastering && m_dataObjWhileRastering->getDims() == 2
    {
        if(m_zDirect)
        {

            ito::DataObject tempObj = *m_dataObjWhileRastering;
            int dataObjectDims = tempObj.getDims();
            int* limits = new int[2*dataObjectDims];
            memset(limits, 0, sizeof(int) * 2*dataObjectDims);

            tempObj.locateROI(limits);

            limits[2*(dataObjectDims - 3)] *= -1;
            limits[2*(dataObjectDims - 3) + 1] *= -1;

            tempObj.adjustROI(dataObjectDims,limits);
            delete limits;

            double fPos;

            if (m_Scaling > 0) fPos = n * m_Scaling + m_startPos;
            else fPos = n * abs(m_Scaling) + m_startPos  + m_physLength;

            cv::Mat* curPlane = (cv::Mat*)tempObj.get_mdata()[tempObj.seekMat(n)];
            switch(m_dataObjWhileRastering->getType())
            {
                case ito::tInt8:
                    return QPointF(fPos, curPlane->at<int8>(m_plotPts[n].rangeY[0], m_plotPts[n].rangeX[0]));
                break;
                case ito::tUInt8:
                    return QPointF(fPos, curPlane->at<uint8>(m_plotPts[n].rangeY[0], m_plotPts[n].rangeX[0]));
                break;
                case ito::tInt16:
                    return QPointF(fPos, curPlane->at<int16>(m_plotPts[n].rangeY[0], m_plotPts[n].rangeX[0]));
                break;
                case ito::tUInt16:
                    return QPointF(fPos, curPlane->at<uint16>(m_plotPts[n].rangeY[0], m_plotPts[n].rangeX[0]));
                break;
                case ito::tInt32:
                    return QPointF(fPos, curPlane->at<int32>(m_plotPts[n].rangeY[0], m_plotPts[n].rangeX[0]));
                break;
                case ito::tUInt32:
                    return QPointF(fPos, curPlane->at<uint32>(m_plotPts[n].rangeY[0], m_plotPts[n].rangeX[0]));
                break;
                case ito::tFloat32:
                    return QPointF(fPos, curPlane->at<float>(m_plotPts[n].rangeY[0], m_plotPts[n].rangeX[0]));
                break;
                case ito::tFloat64:
                    return QPointF(fPos, curPlane->at<double>(m_plotPts[n].rangeY[0], m_plotPts[n].rangeX[0]));
                break;
                case ito::tComplex64:
				{
					switch (m_cmplxState)
					{
						default:
						case 0:
							return QPointF(fPos, abs(curPlane->at<ito::complex64>(m_plotPts[n].rangeY[0], m_plotPts[n].rangeX[0])));
						break;
						case 1:
							return QPointF(fPos, (curPlane->at<ito::complex64>(m_plotPts[n].rangeY[0], m_plotPts[n].rangeX[0]).real()));
						break;
						case 2:
							return QPointF(fPos, (curPlane->at<ito::complex64>(m_plotPts[n].rangeY[0], m_plotPts[n].rangeX[0]).imag()));
						break;
						case 3:
							return QPointF(fPos, arg(curPlane->at<ito::complex64>(m_plotPts[n].rangeY[0], m_plotPts[n].rangeX[0])));
						break;
					}
				}
                break;
                case ito::tComplex128:
				{
					switch (m_cmplxState)
					{
						default:
						case 0:
							return QPointF(fPos, abs(curPlane->at<ito::complex128>(m_plotPts[n].rangeY[0], m_plotPts[n].rangeX[0])));
						break;
						case 1:
							return QPointF(fPos, (curPlane->at<ito::complex128>(m_plotPts[n].rangeY[0], m_plotPts[n].rangeX[0]).real()));
						break;
						case 2:
							return QPointF(fPos, (curPlane->at<ito::complex128>(m_plotPts[n].rangeY[0], m_plotPts[n].rangeX[0]).imag()));
						break;
						case 3:
							return QPointF(fPos, arg(curPlane->at<ito::complex128>(m_plotPts[n].rangeY[0], m_plotPts[n].rangeX[0])));
						break;
					}
				}
                break;
            }
            return QPointF();
        } // if m_zDirect
        else
        {
            double fPos;

            if (m_Scaling > 0) fPos = n * m_Scaling + m_startPos;
            else fPos = n * abs(m_Scaling) + m_startPos  + m_physLength;

            cv::Mat* curPlane = (cv::Mat*)m_dataObj->get_mdata()[m_dataObj->seekMat(0)];
            if (m_fast)
            {
                switch(m_dataObjWhileRastering->getType())
                {
                    case ito::tInt8:
                        return QPointF(fPos, curPlane->at<int8>(m_plotPts[n].rangeY[0], m_plotPts[n].rangeX[0]));
                    break;
                    case ito::tUInt8:
                        return QPointF(fPos, curPlane->at<uint8>(m_plotPts[n].rangeY[0], m_plotPts[n].rangeX[0]));
                    break;
                    case ito::tInt16:
                        return QPointF(fPos, curPlane->at<int16>(m_plotPts[n].rangeY[0], m_plotPts[n].rangeX[0]));
                    break;
                    case ito::tUInt16:
                        return QPointF(fPos, curPlane->at<uint16>(m_plotPts[n].rangeY[0], m_plotPts[n].rangeX[0]));
                    break;
                    case ito::tInt32:
                        return QPointF(fPos, curPlane->at<int32>(m_plotPts[n].rangeY[0], m_plotPts[n].rangeX[0]));
                    break;
                    case ito::tUInt32:
                        return QPointF(fPos, curPlane->at<uint32>(m_plotPts[n].rangeY[0], m_plotPts[n].rangeX[0]));
                    break;
                    case ito::tFloat32:
                        return QPointF(fPos, curPlane->at<float>(m_plotPts[n].rangeY[0], m_plotPts[n].rangeX[0]));
                    break;
                    case ito::tFloat64:
                        return QPointF(fPos, curPlane->at<double>(m_plotPts[n].rangeY[0], m_plotPts[n].rangeX[0]));
                    break;
					case ito::tComplex64:
					{
						switch (m_cmplxState)
						{
							default:
							case 0:
								return QPointF(fPos, abs(curPlane->at<ito::complex64>(m_plotPts[n].rangeY[0], m_plotPts[n].rangeX[0])));
							break;
							case 1:
								return QPointF(fPos, (curPlane->at<ito::complex64>(m_plotPts[n].rangeY[0], m_plotPts[n].rangeX[0]).real()));
							break;
							case 2:
								return QPointF(fPos, (curPlane->at<ito::complex64>(m_plotPts[n].rangeY[0], m_plotPts[n].rangeX[0]).imag()));
							break;
							case 3:
								return QPointF(fPos, arg(curPlane->at<ito::complex64>(m_plotPts[n].rangeY[0], m_plotPts[n].rangeX[0])));
							break;
						}
					}
					break;
					case ito::tComplex128:
					{
						switch (m_cmplxState)
						{
							default:
							case 0:
								return QPointF(fPos, abs(curPlane->at<ito::complex128>(m_plotPts[n].rangeY[0], m_plotPts[n].rangeX[0])));
							break;
							case 1:
								return QPointF(fPos, (curPlane->at<ito::complex128>(m_plotPts[n].rangeY[0], m_plotPts[n].rangeX[0]).real()));
							break;
							case 2:
								return QPointF(fPos, (curPlane->at<ito::complex128>(m_plotPts[n].rangeY[0], m_plotPts[n].rangeX[0]).imag()));
							break;
							case 3:
								return QPointF(fPos, arg(curPlane->at<ito::complex128>(m_plotPts[n].rangeY[0], m_plotPts[n].rangeX[0])));
							break;
						}
					}
					break;
                }
            } // if m_fast
            else
            {
                double val;

                switch(m_dataObjWhileRastering->getType())
                {
                    case ito::tInt8:
                        val = curPlane->at<int8>(m_plotPts[n].rangeY[0], m_plotPts[n].rangeX[0]) * m_plotPts[n].weights[0] +
                            curPlane->at<int8>(m_plotPts[n].rangeY[1], m_plotPts[n].rangeX[0]) * m_plotPts[n].weights[1] +
                            curPlane->at<int8>(m_plotPts[n].rangeY[1], m_plotPts[n].rangeX[1]) * m_plotPts[n].weights[2] +
                            curPlane->at<int8>(m_plotPts[n].rangeY[0], m_plotPts[n].rangeX[1]) * m_plotPts[n].weights[3];
                    break;
                    case ito::tUInt8:
                        val = curPlane->at<uint8>(m_plotPts[n].rangeY[0], m_plotPts[n].rangeX[0]) * m_plotPts[n].weights[0] +
                            curPlane->at<uint8>(m_plotPts[n].rangeY[1], m_plotPts[n].rangeX[0]) * m_plotPts[n].weights[1] +
                            curPlane->at<uint8>(m_plotPts[n].rangeY[1], m_plotPts[n].rangeX[1]) * m_plotPts[n].weights[2] +
                            curPlane->at<uint8>(m_plotPts[n].rangeY[0], m_plotPts[n].rangeX[1]) * m_plotPts[n].weights[3];
                    break;
                    case ito::tInt16:
                        val = curPlane->at<int16>(m_plotPts[n].rangeY[0], m_plotPts[n].rangeX[0]) * m_plotPts[n].weights[0] +
                            curPlane->at<int16>(m_plotPts[n].rangeY[1], m_plotPts[n].rangeX[0]) * m_plotPts[n].weights[1] +
                            curPlane->at<int16>(m_plotPts[n].rangeY[1], m_plotPts[n].rangeX[1]) * m_plotPts[n].weights[2] +
                            curPlane->at<int16>(m_plotPts[n].rangeY[0], m_plotPts[n].rangeX[1]) * m_plotPts[n].weights[3];
                    break;
                    case ito::tUInt16:
                        val = curPlane->at<uint16>(m_plotPts[n].rangeY[0], m_plotPts[n].rangeX[0]) * m_plotPts[n].weights[0] +
                            curPlane->at<uint16>(m_plotPts[n].rangeY[1], m_plotPts[n].rangeX[0]) * m_plotPts[n].weights[1] +
                            curPlane->at<uint16>(m_plotPts[n].rangeY[1], m_plotPts[n].rangeX[1]) * m_plotPts[n].weights[2] +
                            curPlane->at<uint16>(m_plotPts[n].rangeY[0], m_plotPts[n].rangeX[1]) * m_plotPts[n].weights[3];
                    break;
                    case ito::tInt32:
                        val = curPlane->at<int32>(m_plotPts[n].rangeY[0], m_plotPts[n].rangeX[0]) * m_plotPts[n].weights[0] +
                            curPlane->at<int32>(m_plotPts[n].rangeY[1], m_plotPts[n].rangeX[0]) * m_plotPts[n].weights[1] +
                            curPlane->at<int32>(m_plotPts[n].rangeY[1], m_plotPts[n].rangeX[1]) * m_plotPts[n].weights[2] +
                            curPlane->at<int32>(m_plotPts[n].rangeY[0], m_plotPts[n].rangeX[1]) * m_plotPts[n].weights[3];
                    break;
                    case ito::tUInt32:
                        val = curPlane->at<uint32>(m_plotPts[n].rangeY[0], m_plotPts[n].rangeX[0]) * m_plotPts[n].weights[0] +
                            curPlane->at<uint32>(m_plotPts[n].rangeY[1], m_plotPts[n].rangeX[0]) * m_plotPts[n].weights[1] +
                            curPlane->at<uint32>(m_plotPts[n].rangeY[1], m_plotPts[n].rangeX[1]) * m_plotPts[n].weights[2] +
                            curPlane->at<uint32>(m_plotPts[n].rangeY[0], m_plotPts[n].rangeX[1]) * m_plotPts[n].weights[3];
                    break;
                    case ito::tFloat32:
                         val = curPlane->at<float>(m_plotPts[n].rangeY[0], m_plotPts[n].rangeX[0]) * m_plotPts[n].weights[0] +
                            curPlane->at<float>(m_plotPts[n].rangeY[1], m_plotPts[n].rangeX[0]) * m_plotPts[n].weights[1] +
                            curPlane->at<float>(m_plotPts[n].rangeY[1], m_plotPts[n].rangeX[1]) * m_plotPts[n].weights[2] +
                            curPlane->at<float>(m_plotPts[n].rangeY[0], m_plotPts[n].rangeX[1]) * m_plotPts[n].weights[3];
                    break;
                    case ito::tFloat64:
                         val = curPlane->at<double>(m_plotPts[n].rangeY[0], m_plotPts[n].rangeX[0]) * m_plotPts[n].weights[0] +
                            curPlane->at<double>(m_plotPts[n].rangeY[1], m_plotPts[n].rangeX[0]) * m_plotPts[n].weights[1] +
                            curPlane->at<double>(m_plotPts[n].rangeY[1], m_plotPts[n].rangeX[1]) * m_plotPts[n].weights[2] +
                            curPlane->at<double>(m_plotPts[n].rangeY[0], m_plotPts[n].rangeX[1]) * m_plotPts[n].weights[3];
                    break;
					case ito::tComplex64:
					{
						switch (m_cmplxState)
						{
							default:
							case 0:
								rVal = curPlane->at<ito::complex64>(m_plotPts[n].rangeY[0], m_plotPts[n].rangeX[0]).real() * m_plotPts[n].weights[0] +
									curPlane->at<ito::complex64>(m_plotPts[n].rangeY[1], m_plotPts[n].rangeX[0]).real() * m_plotPts[n].weights[1] +
									curPlane->at<ito::complex64>(m_plotPts[n].rangeY[1], m_plotPts[n].rangeX[1]).real() * m_plotPts[n].weights[2] +
									curPlane->at<ito::complex64>(m_plotPts[n].rangeY[0], m_plotPts[n].rangeX[1]).real() * m_plotPts[n].weights[3];
								iVal = curPlane->at<ito::complex64>(m_plotPts[n].rangeY[0], m_plotPts[n].rangeX[0]).imag() * m_plotPts[n].weights[0] +
									curPlane->at<ito::complex64>(m_plotPts[n].rangeY[1], m_plotPts[n].rangeX[0]).imag() * m_plotPts[n].weights[1] +
									curPlane->at<ito::complex64>(m_plotPts[n].rangeY[1], m_plotPts[n].rangeX[1]).imag() * m_plotPts[n].weights[2] +
									curPlane->at<ito::complex64>(m_plotPts[n].rangeY[0], m_plotPts[n].rangeX[1]).imag() * m_plotPts[n].weights[3];
								val = sqrt(rVal * rVal + iVal * iVal);
							break;
							case 1:
								val = curPlane->at<ito::complex64>(m_plotPts[n].rangeY[0], m_plotPts[n].rangeX[0]).real() * m_plotPts[n].weights[0] +
									curPlane->at<ito::complex64>(m_plotPts[n].rangeY[1], m_plotPts[n].rangeX[0]).real() * m_plotPts[n].weights[1] +
									curPlane->at<ito::complex64>(m_plotPts[n].rangeY[1], m_plotPts[n].rangeX[1]).real() * m_plotPts[n].weights[2] +
									curPlane->at<ito::complex64>(m_plotPts[n].rangeY[0], m_plotPts[n].rangeX[1]).real() * m_plotPts[n].weights[3];
							break;
							case 2:
								val = curPlane->at<ito::complex64>(m_plotPts[n].rangeY[0], m_plotPts[n].rangeX[0]).imag() * m_plotPts[n].weights[0] +
									curPlane->at<ito::complex64>(m_plotPts[n].rangeY[1], m_plotPts[n].rangeX[0]).imag() * m_plotPts[n].weights[1] +
									curPlane->at<ito::complex64>(m_plotPts[n].rangeY[1], m_plotPts[n].rangeX[1]).imag() * m_plotPts[n].weights[2] +
									curPlane->at<ito::complex64>(m_plotPts[n].rangeY[0], m_plotPts[n].rangeX[1]).imag() * m_plotPts[n].weights[3];
							break;
							case 3:
								rVal = curPlane->at<ito::complex64>(m_plotPts[n].rangeY[0], m_plotPts[n].rangeX[0]).real() * m_plotPts[n].weights[0] +
									curPlane->at<ito::complex64>(m_plotPts[n].rangeY[1], m_plotPts[n].rangeX[0]).real() * m_plotPts[n].weights[1] +
									curPlane->at<ito::complex64>(m_plotPts[n].rangeY[1], m_plotPts[n].rangeX[1]).real() * m_plotPts[n].weights[2] +
									curPlane->at<ito::complex64>(m_plotPts[n].rangeY[0], m_plotPts[n].rangeX[1]).real() * m_plotPts[n].weights[3];
								iVal = curPlane->at<ito::complex64>(m_plotPts[n].rangeY[0], m_plotPts[n].rangeX[0]).imag() * m_plotPts[n].weights[0] +
									curPlane->at<ito::complex64>(m_plotPts[n].rangeY[1], m_plotPts[n].rangeX[0]).imag() * m_plotPts[n].weights[1] +
									curPlane->at<ito::complex64>(m_plotPts[n].rangeY[1], m_plotPts[n].rangeX[1]).imag() * m_plotPts[n].weights[2] +
									curPlane->at<ito::complex64>(m_plotPts[n].rangeY[0], m_plotPts[n].rangeX[1]).imag() * m_plotPts[n].weights[3];
								val = atan2(iVal, rVal);
							break;
						}
					}
					break;
					case ito::tComplex128:
					{
						switch (m_cmplxState)
						{
							default:
							case 0:
								rVal = curPlane->at<ito::complex128>(m_plotPts[n].rangeY[0], m_plotPts[n].rangeX[0]).real() * m_plotPts[n].weights[0] +
									curPlane->at<ito::complex128>(m_plotPts[n].rangeY[1], m_plotPts[n].rangeX[0]).real() * m_plotPts[n].weights[1] +
									curPlane->at<ito::complex128>(m_plotPts[n].rangeY[1], m_plotPts[n].rangeX[1]).real() * m_plotPts[n].weights[2] +
									curPlane->at<ito::complex128>(m_plotPts[n].rangeY[0], m_plotPts[n].rangeX[1]).real() * m_plotPts[n].weights[3];
								iVal = curPlane->at<ito::complex128>(m_plotPts[n].rangeY[0], m_plotPts[n].rangeX[0]).imag() * m_plotPts[n].weights[0] +
									curPlane->at<ito::complex128>(m_plotPts[n].rangeY[1], m_plotPts[n].rangeX[0]).imag() * m_plotPts[n].weights[1] +
									curPlane->at<ito::complex128>(m_plotPts[n].rangeY[1], m_plotPts[n].rangeX[1]).imag() * m_plotPts[n].weights[2] +
									curPlane->at<ito::complex128>(m_plotPts[n].rangeY[0], m_plotPts[n].rangeX[1]).imag() * m_plotPts[n].weights[3];
								val = sqrt(rVal * rVal + iVal * iVal);
							break;
							case 1:
								val = curPlane->at<ito::complex128>(m_plotPts[n].rangeY[0], m_plotPts[n].rangeX[0]).real() * m_plotPts[n].weights[0] +
									curPlane->at<ito::complex128>(m_plotPts[n].rangeY[1], m_plotPts[n].rangeX[0]).real() * m_plotPts[n].weights[1] +
									curPlane->at<ito::complex128>(m_plotPts[n].rangeY[1], m_plotPts[n].rangeX[1]).real() * m_plotPts[n].weights[2] +
									curPlane->at<ito::complex128>(m_plotPts[n].rangeY[0], m_plotPts[n].rangeX[1]).real() * m_plotPts[n].weights[3];
							break;
							case 2:
								val = curPlane->at<ito::complex128>(m_plotPts[n].rangeY[0], m_plotPts[n].rangeX[0]).imag() * m_plotPts[n].weights[0] +
									curPlane->at<ito::complex128>(m_plotPts[n].rangeY[1], m_plotPts[n].rangeX[0]).imag() * m_plotPts[n].weights[1] +
									curPlane->at<ito::complex128>(m_plotPts[n].rangeY[1], m_plotPts[n].rangeX[1]).imag() * m_plotPts[n].weights[2] +
									curPlane->at<ito::complex128>(m_plotPts[n].rangeY[0], m_plotPts[n].rangeX[1]).imag() * m_plotPts[n].weights[3];
							break;
							case 3:
								rVal = curPlane->at<ito::complex128>(m_plotPts[n].rangeY[0], m_plotPts[n].rangeX[0]).real() * m_plotPts[n].weights[0] +
									curPlane->at<ito::complex128>(m_plotPts[n].rangeY[1], m_plotPts[n].rangeX[0]).real() * m_plotPts[n].weights[1] +
									curPlane->at<ito::complex128>(m_plotPts[n].rangeY[1], m_plotPts[n].rangeX[1]).real() * m_plotPts[n].weights[2] +
									curPlane->at<ito::complex128>(m_plotPts[n].rangeY[0], m_plotPts[n].rangeX[1]).real() * m_plotPts[n].weights[3];
								iVal = curPlane->at<ito::complex128>(m_plotPts[n].rangeY[0], m_plotPts[n].rangeX[0]).imag() * m_plotPts[n].weights[0] +
									curPlane->at<ito::complex128>(m_plotPts[n].rangeY[1], m_plotPts[n].rangeX[0]).imag() * m_plotPts[n].weights[1] +
									curPlane->at<ito::complex128>(m_plotPts[n].rangeY[1], m_plotPts[n].rangeX[1]).imag() * m_plotPts[n].weights[2] +
									curPlane->at<ito::complex128>(m_plotPts[n].rangeY[0], m_plotPts[n].rangeX[1]).imag() * m_plotPts[n].weights[3];
								val = atan2(iVal, rVal);
							break;
						}
					}
					break;
                }
                return QPointF(fPos, val);
            }
        }
    }
    return QPointF();
}

//----------------------------------------------------------------------------------------------------------------------------------
void DataObjectSeriesData::updateDataObject(QSharedPointer<ito::DataObject> dataObj)
{
    if (dataObj)
    {
        m_dataObj = dataObj;

        if(m_dataObj)
        {
        }
    }
}

//----------------------------------------------------------------------------------------------------------------------------------
void DataObjectSeriesData::updateDataObject(QSharedPointer<ito::DataObject> dataObj, int startPoint, unsigned int plotDim, unsigned int size)
{
    m_size = size;
    m_plotPts.resize(size);
    int size_int = static_cast<int>(size);

    int dataObjectDims = dataObj->getDims();
    bool testValid;
    if (plotDim == dataObjectDims - 2)
    {
		m_startPos = dataObj->getPixToPhys(dataObjectDims - 2, startPoint, testValid);
		m_physLength = dataObj->getPixToPhys(dataObjectDims - 2, size - startPoint, testValid) - m_startPos;
		m_Scaling = m_physLength / size_int;

        for (int n = startPoint; n < startPoint + size_int; n++)
        {
            m_plotPts[n].rangeX[0] = 0;
            m_plotPts[n].rangeY[0] = n;
            m_plotPts[n].weights[0] = 1;
        }
    }
    else if (plotDim == dataObjectDims - 1)
    {
		m_startPos = dataObj->getPixToPhys(dataObjectDims - 1, startPoint, testValid);
		m_physLength = dataObj->getPixToPhys(dataObjectDims - 1, size - startPoint, testValid) - m_startPos;
		m_Scaling = m_physLength / size_int;
        for (int n = startPoint; n < startPoint + size_int; n++)
        {
            m_plotPts[n].rangeX[0] = n;
            m_plotPts[n].rangeY[0] = 0;
            m_plotPts[n].weights[0] = 1;
        }
    }
    else
    {
		m_startPos = 0.0;
		m_physLength = 1.0;
		m_Scaling = m_physLength / 1.0;
        for (int n = 0; n < 1; n++)
        {
            m_plotPts[n].rangeX[0] = 0;
            m_plotPts[n].rangeY[0] = 0;
            m_plotPts[n].weights[0] = 1;
        }        
    }

    updateDataObject(dataObj);
}

//----------------------------------------------------------------------------------------------------------------------------------
void DataObjectSeriesData::updateDataObject(QSharedPointer<ito::DataObject> dataObj, QVector<QPointF> ptsPhys)
{
    QVector<QPointF> pts;
    pts.resize(2);
    int dataObjectDims = dataObj->getDims();
    bool testValid;


    if((ptsPhys.size() == 1) && (dataObjectDims > 2))
    {
        m_zDirect = true;
    }
    else 
    {
        m_zDirect = false;
    }

    pts[0].setX(static_cast<float>(dataObj->getPhysToPix(dataObjectDims-1,ptsPhys[0].x(),testValid)));
    pts[0].setY(static_cast<float>(dataObj->getPhysToPix(dataObjectDims-2,ptsPhys[0].y(),testValid)));

    if(m_zDirect)
    {
        ito::DataObject tempObj = *dataObj;
        dataObjectDims = tempObj.getDims();
        int* limits = new int[2*dataObjectDims];
        memset(limits, 0, sizeof(int) * 2*dataObjectDims);

        tempObj.locateROI(limits);

        limits[2*(dataObjectDims - 3)] *= -1;
        limits[2*(dataObjectDims - 3) + 1] *= -1;

        tempObj.adjustROI(dataObjectDims,limits);
        delete limits;

        m_size = tempObj.getSize(dataObjectDims - 3);
        m_startPos = tempObj.getPixToPhys(dataObjectDims - 3, 0.0, testValid);
        m_physLength = tempObj.getPixToPhys(dataObjectDims - 3, m_size, testValid) - m_startPos;
        m_Scaling = m_physLength / m_size;

        m_plotPts.resize(m_size);
        for(unsigned int n = 0; n < m_size; n++)
        {
            m_plotPts[n].rangeX[0] = pts[0].x();
            m_plotPts[n].rangeY[0] = pts[0].y();
            m_plotPts[n].weights[0] = 1;        
        }
    }
    else
    {

        bool xdirect = false;
        bool ydirect = false;
        if(ptsPhys.size() == 1)
        {
            pts[1] = pts[0];
        }
        else
        {
            pts[1].setX(static_cast<float>(dataObj->getPhysToPix(dataObjectDims-1,ptsPhys[1].x(),testValid)));
            pts[1].setY(static_cast<float>(dataObj->getPhysToPix(dataObjectDims-2,ptsPhys[1].y(),testValid)));  
        }

        double xscale = dataObj->getAxisScale(dataObjectDims - 1);

        double yscale = 1;
        if(dataObjectDims > 1)
        {
            yscale = dataObj->getAxisScale(dataObjectDims - 2);
        }

        if (m_fast)
        {
            // simple line points calculation using Bresenham
            int dx = abs(pts[1].x() - pts[0].x());
            int sx = pts[0].x() < pts[1].x() ? 1 : -1;
            int dy = -abs(pts[1].y() - pts[0].y());
            int sy = pts[0].y() < pts[1].y() ? 1 : -1;
            int err = dx + dy, e2 = 0; /* error value e_xy */
            int x = pts[0].x();
            int y = pts[0].y();

            if (dx > -dy)
                m_size = dx;
            else
                m_size = -dy;

            m_plotPts.resize(m_size);

            for(unsigned int n = 0; n < m_size; n++)
            {  /* loop */
                m_plotPts[n].rangeX[0] = x;
                m_plotPts[n].rangeY[0] = y;
                m_plotPts[n].weights[0] = 1;

                e2 = 2 * err;
                if (e2 > dy)
                {
                    err += dy;
                    x += sx;
                } /* e_xy+e_x > 0 */
                if (e2 < dx)
                {
                    err += dx;
                    y += sy;
                } /* e_xy+e_y < 0 */
            }
        }
        else
        {
            // "full" calculation of line points using interpolation values for always four neighbouring point
            double dx = (pts[1].x() - pts[0].x());
            double dy = (pts[1].y() - pts[0].y());
            double sizex = dataObj->getSize(dataObjectDims-1);
            double sizey = dataObj->getSize(dataObjectDims-2);

            double length = sqrt((double)(dx * dx + dy * dy));
            double stepx, stepy;
            if (dx == 0)
            {
                stepx = 0;
            }
            else
            {
                xdirect = true;
                stepx = dx / length;
            }

            if (dy == 0)
            {
                stepy = 0;
            }
            else
            {
                ydirect = true;
                stepy = dy / length;
            }
            m_size = floor(length) + 1;

            if(xdirect && ydirect)
            {
                m_startPos = 0.0;
                m_physLength = sqrt((double)(xscale* xscale* dx * dx + yscale * yscale * dy * dy));
            }
            else if(ydirect)
            {
                m_startPos = ptsPhys[0].y();
                m_physLength = yscale* dy ;      
            }
            else
            {
                m_startPos = ptsPhys[0].x();
                m_physLength = xscale* dx ;             
            }

            m_Scaling = m_physLength / length;

            double xsub, ysub, xpos, ypos;

            m_plotPts.resize(m_size);
            for (unsigned int n = 0; n < m_size; n++)
            {
                xpos = pts[0].x() + n * stepx;
                ypos = pts[0].y() + n * stepy;

                if (xpos >= sizex - 1)
                {
                    if(xpos >= sizex)
                    {
                        xpos = sizex - 1;
                    }
                    m_plotPts[n].rangeX[1] =  static_cast<int32>(sizex - 1);
                }
                else
                {
                    m_plotPts[n].rangeX[1] =  static_cast<int32>(floor(xpos) + 1);
                }

                if (ypos >= sizey - 1)
                {
                    if(ypos >= sizey)
                    {
                        ypos = sizey - 1;
                    }
                    m_plotPts[n].rangeY[1] =  static_cast<int32>(sizey - 1);
                }
                else
                {
                    m_plotPts[n].rangeY[1] =  static_cast<int32>(floor(ypos) + 1);
                }

                m_plotPts[n].rangeX[0] = floor(xpos);
                m_plotPts[n].rangeY[0] = floor(ypos);
                xsub = xpos - m_plotPts[n].rangeX[0];
                ysub = ypos - m_plotPts[n].rangeY[0];
                m_plotPts[n].weights[0] = (1 - xsub) * (1 - ysub);
                m_plotPts[n].weights[1] = (1 - xsub) * ysub;
                m_plotPts[n].weights[2] = xsub * ysub;
                m_plotPts[n].weights[3] = xsub * (1 - ysub);
            }
        }
        
    }
    updateDataObject(dataObj);
}

//----------------------------------------------------------------------------------------------------------------------------------
void DataObjectSeriesData::setIntervalRange(Qt::Axis axis, bool autoCalcLimits, double minValue, double maxValue)
{
    if(axis == Qt::YAxis)
    {
        if(autoCalcLimits)
        {
            m_autoScaleY = true;
        }
        else
        {
            m_autoScaleY = false;
            m_minY = minValue;
            m_maxY = maxValue;
        }
    }
    if(axis == Qt::XAxis)
    {
        if(autoCalcLimits)
        {
            m_autoScaleX = true;
        }
        else
        {
            m_autoScaleX = false;
            m_minX = minValue;
            m_maxX = maxValue;
        }
    }
}

//----------------------------------------------------------------------------------------------------------------------------------
template<typename _Tp> void findMinMaxInZ(ito::DataObject &obj, QVector<DataObjectSeriesData::ptsAndWeights> pts, double &min, double &max, const int cmplxState = -1)
{
    min = std::numeric_limits<_Tp>::max();
    if(std::numeric_limits<_Tp>::is_exact)
    {
        max = std::numeric_limits<_Tp>::min(); //integer numbers
    }
    else
    {
        max = -std::numeric_limits<_Tp>::max();
    }

    double val;
    cv::Mat *mat;
    for (int n = 0; n < pts.size(); n++)
    {
        mat = (cv::Mat *)(obj.get_mdata())[obj.seekMat(n)];
        val = mat->at<_Tp>(pts[n].rangeY[0], pts[n].rangeX[0]);
        if (!qIsFinite(val))
            continue;
        if (val > max)
            max = val;
        if (val < min)
            min = val;
    }
}

//----------------------------------------------------------------------------------------------------------------------------------
template<> void findMinMaxInZ<ito::complex64>(ito::DataObject &obj, QVector<DataObjectSeriesData::ptsAndWeights> pts, double &min, double &max, const int cmplxState)
{
    min = std::numeric_limits<float>::max();
    max = -min;
    double val;
    cv::Mat *mat;

	switch (cmplxState)
	{
		default:
		case 0:
		{
			for (int n = 0; n < pts.size(); n++)
			{
				mat = (cv::Mat *)(obj.get_mdata())[obj.seekMat(n)];
				val = abs(mat->at<ito::complex64>(pts[n].rangeY[0], pts[n].rangeX[0]));

				if (!qIsFinite(val))
					continue;
				if (val > max)
					max = val;
				if (val < min)
					min = val;
			}
		}
		break;

		case 1:
		{
			for (int n = 0; n < pts.size(); n++)
			{
				mat = (cv::Mat *)(obj.get_mdata())[obj.seekMat(n)];
				val = (mat->at<ito::complex64>(pts[n].rangeY[0], pts[n].rangeX[0])).imag();

				if (!qIsFinite(val))
					continue;
				if (val > max)
					max = val;
				if (val < min)
					min = val;
			}
		}
		break;

		case 2:
		{
			for (int n = 0; n < pts.size(); n++)
			{
				mat = (cv::Mat *)(obj.get_mdata())[obj.seekMat(n)];
				val = (mat->at<ito::complex64>(pts[n].rangeY[0], pts[n].rangeX[0])).real();

				if (!qIsFinite(val))
					continue;
				if (val > max)
					max = val;
				if (val < min)
					min = val;
			}
		}
		break;

		case 3:
		{
			for (int n = 0; n < pts.size(); n++)
			{
				mat = (cv::Mat *)(obj.get_mdata())[obj.seekMat(n)];
				val = arg(mat->at<ito::complex64>(pts[n].rangeY[0], pts[n].rangeX[0]));

				if (!qIsFinite(val))
					continue;
				if (val > max)
					max = val;
				if (val < min)
					min = val;
			}
		}
		break;
	}
}

//----------------------------------------------------------------------------------------------------------------------------------
template<> void findMinMaxInZ<ito::complex128>(ito::DataObject &obj, QVector<DataObjectSeriesData::ptsAndWeights> pts, double &min, double &max, const int cmplxState)
{
    min = std::numeric_limits<double>::max();
    max = -min;
    double val;
    cv::Mat *mat;

	switch (cmplxState)
	{
		default:
		case 0:
		{
			for (int n = 0; n < pts.size(); n++)
			{
				mat = (cv::Mat *)(obj.get_mdata())[obj.seekMat(n)];
				val = abs(mat->at<ito::complex128>(pts[n].rangeY[0], pts[n].rangeX[0]));

				if (!qIsFinite(val))
					continue;
				if (val > max)
					max = val;
				if (val < min)
					min = val;
			}
		}
		break;

		case 1:
		{
			for (int n = 0; n < pts.size(); n++)
			{
				mat = (cv::Mat *)(obj.get_mdata())[obj.seekMat(n)];
				val = (mat->at<ito::complex128>(pts[n].rangeY[0], pts[n].rangeX[0])).imag();

				if (!qIsFinite(val))
					continue;
				if (val > max)
					max = val;
				if (val < min)
					min = val;
			}
		}
		break;

		case 2:
		{
			for (int n = 0; n < pts.size(); n++)
			{
				mat = (cv::Mat *)(obj.get_mdata())[obj.seekMat(n)];
				val = (mat->at<ito::complex128>(pts[n].rangeY[0], pts[n].rangeX[0])).real();

				if (!qIsFinite(val))
					continue;
				if (val > max)
					max = val;
				if (val < min)
					min = val;
			}
		}
		break;

		case 3:
		{
			for (int n = 0; n < pts.size(); n++)
			{
				mat = (cv::Mat *)(obj.get_mdata())[obj.seekMat(n)];
				val = arg(mat->at<ito::complex128>(pts[n].rangeY[0], pts[n].rangeX[0]));

				if (!qIsFinite(val))
					continue;
				if (val > max)
					max = val;
				if (val < min)
					min = val;
			}
		}
		break;
	}
}

//----------------------------------------------------------------------------------------------------------------------------------
template<typename _Tp> void findMinMax(cv::Mat *mat, QVector<DataObjectSeriesData::ptsAndWeights> pts, int fast, double &min, double &max, const int cmplxState = -1)
{
    min = std::numeric_limits<_Tp>::max();
    if(std::numeric_limits<_Tp>::is_exact)
    {
        max = std::numeric_limits<_Tp>::min(); //integer numbers
    }
    else
    {
        max = -std::numeric_limits<_Tp>::max();
    }
    double val;

    if (fast)
    {
        for (int n = 0; n < pts.size(); n++)
        {
            val = mat->at<_Tp>(pts[n].rangeY[0], pts[n].rangeX[0]);
            //if (qIsInf(val))
            if (!qIsFinite(val))
                continue;
            if (val > max)
                max = val;
            if (val < min)
                min = val;
        }
    }
    else
    {
        for (int n = 0; n < pts.size(); n++)
        {
            val = mat->at<_Tp>(pts[n].rangeY[0], pts[n].rangeX[0]) * pts[n].weights[0] +
                mat->at<_Tp>(pts[n].rangeY[1], pts[n].rangeX[0]) * pts[n].weights[1] +
                mat->at<_Tp>(pts[n].rangeY[1], pts[n].rangeX[1]) * pts[n].weights[2] +
                mat->at<_Tp>(pts[n].rangeY[0], pts[n].rangeX[1]) * pts[n].weights[3];
            if (!qIsFinite(val))
                continue;
            if (val > max)
                max = val;
            if (val < min)
                min = val;
        }
    }
}

//----------------------------------------------------------------------------------------------------------------------------------
template<> void findMinMax<ito::complex64>(cv::Mat *mat, QVector<DataObjectSeriesData::ptsAndWeights> pts, int fast, double &min, double &max, const int cmplxState)
{
    min = std::numeric_limits<float>::max();
	max = -min;
    double val, rVal, iVal;

    if (fast)
    {
		switch (cmplxState)
		{
			default:
			case 0:
			{
				for (int n = 0; n < pts.size(); n++)
				{
					val = abs(mat->at<ito::complex64>(pts[n].rangeY[0], pts[n].rangeX[0]));
					//if (qIsInf(val))
					if (!qIsFinite(val))
						continue;
					if (val > max)
						max = val;
					if (val < min)
						min = val;
				}
			}
			break;

			case 1:
			{
				for (int n = 0; n < pts.size(); n++)
				{
					val = (mat->at<ito::complex64>(pts[n].rangeY[0], pts[n].rangeX[0])).real();
					//if (qIsInf(val))
					if (!qIsFinite(val))
						continue;
					if (val > max)
						max = val;
					if (val < min)
						min = val;
				}
			}
			break;

			case 2:
			{
				for (int n = 0; n < pts.size(); n++)
				{
					val = (mat->at<ito::complex64>(pts[n].rangeY[0], pts[n].rangeX[0])).imag();
					//if (qIsInf(val))
					if (!qIsFinite(val))
						continue;
					if (val > max)
						max = val;
					if (val < min)
						min = val;
				}
			}
			break;

			case 3:
			{
				for (int n = 0; n < pts.size(); n++)
				{
					val = arg(mat->at<ito::complex64>(pts[n].rangeY[0], pts[n].rangeX[0]));
					//if (qIsInf(val))
					if (!qIsFinite(val))
						continue;
					if (val > max)
						max = val;
					if (val < min)
						min = val;
				}
			}
			break;
		}
    }
    else
    {
		switch (cmplxState)
		{
			default:
			case 0:
			{
				for (int n = 0; n < pts.size(); n++)
				{
					rVal = mat->at<ito::complex64>(pts[n].rangeY[0], pts[n].rangeX[0]).real() * pts[n].weights[0] +
						mat->at<ito::complex64>(pts[n].rangeY[1], pts[n].rangeX[0]).real() * pts[n].weights[1] +
						mat->at<ito::complex64>(pts[n].rangeY[1], pts[n].rangeX[1]).real() * pts[n].weights[2] +
						mat->at<ito::complex64>(pts[n].rangeY[0], pts[n].rangeX[1]).real() * pts[n].weights[3];
					iVal = mat->at<ito::complex64>(pts[n].rangeY[0], pts[n].rangeX[0]).imag() * pts[n].weights[0] +
						mat->at<ito::complex64>(pts[n].rangeY[1], pts[n].rangeX[0]).imag() * pts[n].weights[1] +
						mat->at<ito::complex64>(pts[n].rangeY[1], pts[n].rangeX[1]).imag() * pts[n].weights[2] +
						mat->at<ito::complex64>(pts[n].rangeY[0], pts[n].rangeX[1]).imag() * pts[n].weights[3];
					val = sqrt(rVal * rVal + iVal * iVal);
					if (!qIsFinite(val))
						continue;
					if (val > max)
						max = val;
					if (val < min)
						min = val;
				}
			}
			break;

			case 1:
			{
				for (int n = 0; n < pts.size(); n++)
				{
					val = mat->at<ito::complex64>(pts[n].rangeY[0], pts[n].rangeX[0]).real() * pts[n].weights[0] +
						mat->at<ito::complex64>(pts[n].rangeY[1], pts[n].rangeX[0]).real() * pts[n].weights[1] +
						mat->at<ito::complex64>(pts[n].rangeY[1], pts[n].rangeX[1]).real() * pts[n].weights[2] +
						mat->at<ito::complex64>(pts[n].rangeY[0], pts[n].rangeX[1]).real() * pts[n].weights[3];
					if (!qIsFinite(val))
						continue;
					if (val > max)
						max = val;
					if (val < min)
						min = val;
				}
			}
			break;

			case 2:
			{
				for (int n = 0; n < pts.size(); n++)
				{
					val = mat->at<ito::complex64>(pts[n].rangeY[0], pts[n].rangeX[0]).imag() * pts[n].weights[0] +
						mat->at<ito::complex64>(pts[n].rangeY[1], pts[n].rangeX[0]).imag() * pts[n].weights[1] +
						mat->at<ito::complex64>(pts[n].rangeY[1], pts[n].rangeX[1]).imag() * pts[n].weights[2] +
						mat->at<ito::complex64>(pts[n].rangeY[0], pts[n].rangeX[1]).imag() * pts[n].weights[3];
					if (!qIsFinite(val))
						continue;
					if (val > max)
						max = val;
					if (val < min)
						min = val;
				}
			}
			break;

			case 3:
			{
				for (int n = 0; n < pts.size(); n++)
				{
					rVal = mat->at<ito::complex64>(pts[n].rangeY[0], pts[n].rangeX[0]).real() * pts[n].weights[0] +
						mat->at<ito::complex64>(pts[n].rangeY[1], pts[n].rangeX[0]).real() * pts[n].weights[1] +
						mat->at<ito::complex64>(pts[n].rangeY[1], pts[n].rangeX[1]).real() * pts[n].weights[2] +
						mat->at<ito::complex64>(pts[n].rangeY[0], pts[n].rangeX[1]).real() * pts[n].weights[3];
					iVal = mat->at<ito::complex64>(pts[n].rangeY[0], pts[n].rangeX[0]).imag() * pts[n].weights[0] +
						mat->at<ito::complex64>(pts[n].rangeY[1], pts[n].rangeX[0]).imag() * pts[n].weights[1] +
						mat->at<ito::complex64>(pts[n].rangeY[1], pts[n].rangeX[1]).imag() * pts[n].weights[2] +
						mat->at<ito::complex64>(pts[n].rangeY[0], pts[n].rangeX[1]).imag() * pts[n].weights[3];
					val = atan2(iVal, rVal);
					if (!qIsFinite(val))
						continue;
					if (val > max)
						max = val;
					if (val < min)
						min = val;
				}
			}
			break;
		}
    }
}

//----------------------------------------------------------------------------------------------------------------------------------
template<> void findMinMax<ito::complex128>(cv::Mat *mat, QVector<DataObjectSeriesData::ptsAndWeights> pts, int fast, double &min, double &max, const int cmplxState)
{
    min = std::numeric_limits<double>::max();
	max = -min;
    double val, rVal, iVal;

    if (fast)
    {
		switch (cmplxState)
		{
			default:
			case 0:
			{
				for (int n = 0; n < pts.size(); n++)
				{
					val = abs(mat->at<ito::complex128>(pts[n].rangeY[0], pts[n].rangeX[0]));
					//if (qIsInf(val))
					if (!qIsFinite(val))
						continue;
					if (val > max)
						max = val;
					if (val < min)
						min = val;
				}
			}
			break;

			case 1:
			{
				for (int n = 0; n < pts.size(); n++)
				{
					val = (mat->at<ito::complex128>(pts[n].rangeY[0], pts[n].rangeX[0])).real();
					//if (qIsInf(val))
					if (!qIsFinite(val))
						continue;
					if (val > max)
						max = val;
					if (val < min)
						min = val;
				}
			}
			break;

			case 2:
			{
				for (int n = 0; n < pts.size(); n++)
				{
					val = (mat->at<ito::complex128>(pts[n].rangeY[0], pts[n].rangeX[0])).imag();
					//if (qIsInf(val))
					if (!qIsFinite(val))
						continue;
					if (val > max)
						max = val;
					if (val < min)
						min = val;
				}
			}
			break;

			case 3:
			{
				for (int n = 0; n < pts.size(); n++)
				{
					val = arg(mat->at<ito::complex128>(pts[n].rangeY[0], pts[n].rangeX[0]));
					//if (qIsInf(val))
					if (!qIsFinite(val))
						continue;
					if (val > max)
						max = val;
					if (val < min)
						min = val;
				}
			}
			break;
		}
    }
    else
    {
		switch (cmplxState)
		{
			default:
			case 0:
			{
				for (int n = 0; n < pts.size(); n++)
				{
					rVal = mat->at<ito::complex128>(pts[n].rangeY[0], pts[n].rangeX[0]).real() * pts[n].weights[0] +
						mat->at<ito::complex128>(pts[n].rangeY[1], pts[n].rangeX[0]).real() * pts[n].weights[1] +
						mat->at<ito::complex128>(pts[n].rangeY[1], pts[n].rangeX[1]).real() * pts[n].weights[2] +
						mat->at<ito::complex128>(pts[n].rangeY[0], pts[n].rangeX[1]).real() * pts[n].weights[3];
					iVal = mat->at<ito::complex128>(pts[n].rangeY[0], pts[n].rangeX[0]).imag() * pts[n].weights[0] +
						mat->at<ito::complex128>(pts[n].rangeY[1], pts[n].rangeX[0]).imag() * pts[n].weights[1] +
						mat->at<ito::complex128>(pts[n].rangeY[1], pts[n].rangeX[1]).imag() * pts[n].weights[2] +
						mat->at<ito::complex128>(pts[n].rangeY[0], pts[n].rangeX[1]).imag() * pts[n].weights[3];
					val = sqrt(rVal * rVal + iVal * iVal);
					if (!qIsFinite(val))
						continue;
					if (val > max)
						max = val;
					if (val < min)
						min = val;
				}
			}
			break;

			case 1:
			{
				for (int n = 0; n < pts.size(); n++)
				{
					val = mat->at<ito::complex128>(pts[n].rangeY[0], pts[n].rangeX[0]).real() * pts[n].weights[0] +
						mat->at<ito::complex128>(pts[n].rangeY[1], pts[n].rangeX[0]).real() * pts[n].weights[1] +
						mat->at<ito::complex128>(pts[n].rangeY[1], pts[n].rangeX[1]).real() * pts[n].weights[2] +
						mat->at<ito::complex128>(pts[n].rangeY[0], pts[n].rangeX[1]).real() * pts[n].weights[3];
					if (!qIsFinite(val))
						continue;
					if (val > max)
						max = val;
					if (val < min)
						min = val;
				}
			}
			break;

			case 2:
			{
				for (int n = 0; n < pts.size(); n++)
				{
					val = mat->at<ito::complex128>(pts[n].rangeY[0], pts[n].rangeX[0]).imag() * pts[n].weights[0] +
						mat->at<ito::complex128>(pts[n].rangeY[1], pts[n].rangeX[0]).imag() * pts[n].weights[1] +
						mat->at<ito::complex128>(pts[n].rangeY[1], pts[n].rangeX[1]).imag() * pts[n].weights[2] +
						mat->at<ito::complex128>(pts[n].rangeY[0], pts[n].rangeX[1]).imag() * pts[n].weights[3];
					if (!qIsFinite(val))
						continue;
					if (val > max)
						max = val;
					if (val < min)
						min = val;
				}
			}
			break;

			case 3:
			{
				for (int n = 0; n < pts.size(); n++)
				{
					rVal = mat->at<ito::complex128>(pts[n].rangeY[0], pts[n].rangeX[0]).real() * pts[n].weights[0] +
						mat->at<ito::complex128>(pts[n].rangeY[1], pts[n].rangeX[0]).real() * pts[n].weights[1] +
						mat->at<ito::complex128>(pts[n].rangeY[1], pts[n].rangeX[1]).real() * pts[n].weights[2] +
						mat->at<ito::complex128>(pts[n].rangeY[0], pts[n].rangeX[1]).real() * pts[n].weights[3];
					iVal = mat->at<ito::complex128>(pts[n].rangeY[0], pts[n].rangeX[0]).imag() * pts[n].weights[0] +
						mat->at<ito::complex128>(pts[n].rangeY[1], pts[n].rangeX[0]).imag() * pts[n].weights[1] +
						mat->at<ito::complex128>(pts[n].rangeY[1], pts[n].rangeX[1]).imag() * pts[n].weights[2] +
						mat->at<ito::complex128>(pts[n].rangeY[0], pts[n].rangeX[1]).imag() * pts[n].weights[3];
					val = atan2(iVal, rVal);
					if (!qIsFinite(val))
						continue;
					if (val > max)
						max = val;
					if (val < min)
						min = val;
				}
			}
			break;
		}
    }
}
QRectF DataObjectSeriesData::boundingRectMax() const
{
//    double minX = -1.0;
//    double maxX = 2.0;

    double minY = -std::numeric_limits<ito::float64>::max();
    double maxY = std::numeric_limits<ito::float64>::max();
    
    if (m_dataObj != NULL)
    {
        switch(m_dataObj->getType())
        {
            case ito::tInt8:
                minY = std::numeric_limits<ito::int8>::min();
                maxY = std::numeric_limits<ito::int8>::max();
            break;
            case ito::tUInt8:
                minY = std::numeric_limits<ito::int8>::min();
                maxY = std::numeric_limits<ito::int8>::max();
            break;
            case ito::tInt16:
                minY = std::numeric_limits<ito::int16>::min();
                maxY = std::numeric_limits<ito::int16>::max();
            break;
            case ito::tUInt16:
                minY = std::numeric_limits<ito::uint16>::min();
                maxY = std::numeric_limits<ito::uint16>::max();
            break;
            case ito::tInt32:
                minY = std::numeric_limits<ito::int32>::min();
                maxY = std::numeric_limits<ito::int32>::max();
            break;
            case ito::tUInt32:
                minY = std::numeric_limits<ito::uint32>::min();
                maxY = std::numeric_limits<ito::uint32>::max();
            break;
            case ito::tFloat32:
            case ito::tComplex64:
                minY = -std::numeric_limits<ito::float32>::max();
                maxY = std::numeric_limits<ito::float32>::max();
            break;
        }
    }

    if(m_Scaling > 0) return QRectF(m_startPos, minY, m_physLength, maxY - minY);
    else return QRectF(m_startPos + m_physLength, minY, abs(m_physLength), maxY - minY);
}
//------------------------------------------------------------------------------------------------------------
ito::DataObject DataObjectSeriesData::getResampledDataObject()
{
    if(m_plotPts.size() > 1)
    {
        setRasterObj();

        ito::DataObject temp(1, m_plotPts.size(), ito::tFloat64);
        ito::float64* ptrTemp = ((ito::float64*)((cv::Mat*)temp.get_mdata()[0])->ptr(0));

        QPointF val = sample(0);
        
        double offset = val.x();
        double scale = val.x();
        *(ptrTemp++) = val.y();

        val = sample(1);
        scale -= val.x();
        offset = offset/scale;

        *(ptrTemp++) = val.y();

        temp.setAxisScale(1, scale);
        temp.setAxisOffset(1, offset);

        for(int i = 2; i < m_plotPts.size(); i++)
        {
            *(ptrTemp++) = sample(i).y();
        }

        releaseRasterObj();
        return temp;
    }
    

    return ito::DataObject();
}
//----------------------------------------------------------------------------------------------------------------------------------
QRectF DataObjectSeriesData::boundingRect() const
{
    if (m_dataObj != NULL)
    {
        cv::Mat *mat = (cv::Mat *)(m_dataObj->get_mdata())[m_dataObj->seekMat(0)];
        double max = -1.0e308;
        double min = 1.0e308;

        if(m_autoScaleY)
        {
            if(m_zDirect)
            {

                ito::DataObject tempObj = *m_dataObj;
                int dataObjectDims = tempObj.getDims();
                int* limits = new int[2*dataObjectDims];
                memset(limits, 0, sizeof(int) * 2*dataObjectDims);

                tempObj.locateROI(limits);

                limits[2*(dataObjectDims - 3)] *= -1;
                limits[2*(dataObjectDims - 3) + 1] *= -1;

                tempObj.adjustROI(dataObjectDims,limits);
                delete limits;

                switch(m_dataObj->getType())
                {
                    case ito::tInt8:
                        findMinMaxInZ<int8>(tempObj, m_plotPts, min, max);
                    break;
                    case ito::tUInt8:
                        findMinMaxInZ<uint8>(tempObj, m_plotPts, min, max);
                    break;
                    case ito::tInt16:
                        findMinMaxInZ<int16>(tempObj, m_plotPts, min, max);
                    break;
                    case ito::tUInt16:
                        findMinMaxInZ<uint16>(tempObj, m_plotPts, min, max);
                    break;
                    case ito::tInt32:
                        findMinMaxInZ<int32>(tempObj, m_plotPts, min, max);
                    break;
                    case ito::tUInt32:
                        findMinMaxInZ<uint32>(tempObj, m_plotPts, min, max);
                    break;
                    case ito::tFloat32:
                        findMinMaxInZ<float32>(tempObj, m_plotPts, min, max);
                    break;
                    case ito::tFloat64:
                        findMinMaxInZ<float64>(tempObj, m_plotPts, min, max);
                    break;
                    case ito::tComplex64:
                        findMinMaxInZ<complex64>(tempObj, m_plotPts, min, max, m_cmplxState);
                    break;
                    case ito::tComplex128:
                        findMinMaxInZ<complex128>(tempObj, m_plotPts, min, max, m_cmplxState);
                    break;
                }
            }
            else
            {
                switch(m_dataObj->getType())
                {
                    case ito::tInt8:
                        findMinMax<int8>(mat, m_plotPts, m_fast, min, max);
                    break;
                    case ito::tUInt8:
                        findMinMax<uint8>(mat, m_plotPts, m_fast, min, max);
                    break;
                    case ito::tInt16:
                        findMinMax<int16>(mat, m_plotPts, m_fast, min, max);
                    break;
                    case ito::tUInt16:
                        findMinMax<uint16>(mat, m_plotPts, m_fast, min, max);
                    break;
                    case ito::tInt32:
                        findMinMax<int32>(mat, m_plotPts, m_fast, min, max);
                    break;
                    case ito::tUInt32:
                        findMinMax<uint32>(mat, m_plotPts, m_fast, min, max);
                    break;
                    case ito::tFloat32:
                        findMinMax<float32>(mat, m_plotPts, m_fast, min, max);
                    break;
                    case ito::tFloat64:
                        findMinMax<float64>(mat, m_plotPts, m_fast, min, max);
                    break;
					case ito::tComplex64:
                        findMinMax<complex64>(mat, m_plotPts, m_fast, min, max, m_cmplxState);
                    break;
					case ito::tComplex128:
                        findMinMax<complex128>(mat, m_plotPts, m_fast, min, max, m_cmplxState);
                    break;
                }            
            }

        }
        else
        {
            min = m_minY;
            max = m_maxY;
        }
        
        if(m_autoScaleX)
        {
            if(m_Scaling > 0) return QRectF(m_startPos, min, m_physLength, max - min);
            else return QRectF(m_startPos + m_physLength, min, abs(m_physLength), max - min);
        }
        else
        {

            return QRectF(m_minX, min, m_maxX-m_minX, max - min);
        }
    }
    else
    {
        return QRectF( 1.0, 1.0, -2.0, -2.0 ); // invalid
    }
}

//----------------------------------------------------------------------------------------------------------------------------------
