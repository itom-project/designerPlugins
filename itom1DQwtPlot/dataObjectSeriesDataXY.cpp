/* ********************************************************************
   itom measurement system
   URL: http://www.uni-stuttgart.de/ito
   Copyright (C) 2012, Institut fuer Technische Optik (ITO), 
   Universitaet Stuttgart, Germany 
 
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

#include "dataObjectSeriesDataXY.h"
#include "common/typeDefs.h"
#include <qcryptographichash.h>

#include <qdebug.h>
#include <qnumeric.h>

//----------------------------------------------------------------------------------------------------------------------------------
DataObjectSeriesDataXY::DataObjectSeriesDataXY(const int fastmode):
    DataObjectSeriesData(fastmode),
    m_fast(fastmode),
    m_autoScaleY(true),
    m_minY(-1.0),
    m_maxY(2.0),
    m_autoScaleX(true),
    m_minX(-1.0),
    m_maxX(1.0),
    m_cmplxState(ItomQwtPlotEnums::CmplxAbs),
    m_pDataObj(NULL),
    inSamplingMode(false),
    m_colorState(grayColor)
{
    m_dX.nrPoints = 0;
    m_dX.points.clear();
    m_dX.valid = false;
}

//----------------------------------------------------------------------------------------------------------------------------------
DataObjectSeriesDataXY::~DataObjectSeriesDataXY()
{
}
template<typename _T> void DataObjectSeriesDataXY::sortValues(const ito::DataObject* obj)
{
    cv::Mat* mat = (cv::Mat*)obj->get_mdata()[obj->seekMat(m_dX.plane)];
    int val = mat->data[m_dX.matOffset];
    m_dX.matSteps.resize((int)m_dX.nrPoints);
    bool* mask = new bool[(int)m_dX.nrPoints]();

    int j;
    //find smallest number
    for (j = 0; j < m_dX.nrPoints; ++j)
    {
        if(val > mat->data[m_dX.matOffset+j*m_dX.matStepSize])
        {
            val = mat->data[m_dX.matOffset + j*m_dX.matStepSize];
            m_dX.matSteps[0] = j;
            mask[j] = true;
        }
    }
    //fill in the next values
    for (j = 0; j < m_dX.nrPoints - 1; ++j)
    {

    }
    delete[] mask;
    mask = NULL;
}
RetVal DataObjectSeriesDataXY::updateDataObject(const ito::DataObject * dataObj, const ito::DataObject * xVec, QVector<QPointF> bounds, QVector<QPointF> boundsX)
{
    DataObjectSeriesData::updateDataObject(dataObj, bounds);
    RetVal retval;
    bool _unused;
    QRectF p;
    float right;
    cv::Mat *mat;
    int pxX1, pxX2, pxY1, pxY2, pxX1x, pxX2x, pxY1x, pxY2x;
    std::string description, unit;

    if (dataObj == NULL || xVec == NULL)
    {
        //data representing yObj
        m_d.plane = 0;
        m_d.dir = dirZ;
        m_d.nrPoints = 0;
        m_d.points.clear();
        m_d.matSteps.clear();
        m_d.valid = false;
        //data representing xObj
        m_dX.plane = 0;
        m_dX.dir = dirZ;
        m_dX.nrPoints = 0;
        m_dX.points.clear();
        m_dX.matSteps.clear();
        m_dX.valid = false;
    }
    else
    {
        int dims = dataObj->getDims();
        int dimsX = xVec->getDims();

        int prependedOneDims = 0;
        int prependOneDimsX = 0;
        int i;
        for (i = 0; i < dims - 2; ++i)
        {
            if (dataObj->getSize(i) != 1)
            {
                break;
            }
            prependedOneDims++;
        }
        for (i = 0; i < dimsX - 2; ++i)
        {
            if (xVec->getSize(i) != 1)
            {
                break;
            }
            ++prependOneDimsX;
        }

        QVector<QPointF> tmpBounds;
        QVector<QPointF> tmpBoundsX;

        if (bounds.size() == 3) // not sure if needed
        {
            m_d.plane = bounds[0].x();
            m_d.plane = dims > 2 ? std::min(m_d.plane, dataObj->getSize(dims - 3)) : 0;
            m_d.plane = std::max(m_d.plane, 0);
            tmpBounds.resize(2);
            tmpBounds[0] = bounds[1];
            tmpBounds[1] = bounds[2];
        }
        else
        {
            m_d.plane = 0;
            tmpBounds = bounds;
        }

        if (boundsX.size() == 3) // not sure if needed
        {
            m_dX.plane = boundsX[0].x();
            m_dX.plane = dimsX > 2 ? std::min(m_dX.plane, xVec->getSize(dimsX - 3)) : 0;
            m_dX.plane = std::max(m_dX.plane, 0);
            tmpBoundsX.resize(2);
            tmpBoundsX[0] = boundsX[1];
            tmpBoundsX[1] = boundsX[2];
        }
        else
        {
            m_dX.plane = 0;
            tmpBoundsX = bounds;
        }
        if (!dataObj->get_mdata() || !(cv::Mat*)(dataObj->get_mdata()[m_d.plane])->data)
            return ito::RetVal(ito::retError, 0, QObject::tr("cv:Mat in data object seems corrupted").toLatin1().data());
        if (!xVec->get_mdata() || !(cv::Mat*)(xVec->get_mdata()[m_dX.plane])->data)
            return ito::RetVal(ito::retError, 0, QObject::tr("cv::Mat in data Object representing the x-vector seems corrupted").toLatin1().data());

        if (tmpBounds.size() != 2 || tmpBoundsX.size() != 2) //size will be one if a line of a Stack is extracted 
        {
            //Todo
        }
        else
        {
            retval += RetVal(retError, 0, "bounds vector must have 2 entries");
        }
        if (tmpBounds.size() == 2 && tmpBoundsX.size() == 2)
        {
            //dir X, dirY
            if ((dims - prependedOneDims) != 2)//check if data is 2d
            {
                m_d.valid = false;
                retval += RetVal(retError, 0, "line plot requires a 2-dim dataObject or the first (n-2) dimensions must have a size of 1");
            }
            if ((dimsX - prependOneDimsX) != 2)//check if xVec is 2d
            {
                m_dX.valid = false;
                retval += RetVal(retError, 0, "xy line plot requires a 2-dim dataObject or the first (n-2) dimension must have a size of 1 for representing x-vector");
            }
            else if (xVec->getSize(dimsX - 2) != 1)//check if first dimension of xVec is of shape 1
            {
                m_dX.valid = false;
                retval += RetVal(retError, 0, "xy line plot requires a 2-dim dataObject with a size of 1 for the first dimension");
            }
            if (!retval.containsError())
            {
                m_d.valid = true;
                m_dX.valid = true;
                 //bounds phys to pix of dataObj
                pxX1 = qRound(dataObj->getPixToPhys(dims - 1, tmpBounds[0].x(), _unused));
                pxY1 = qRound(dataObj->getPixToPhys(dims - 2, tmpBounds[0].y(), _unused));
                pxX2 = qRound(dataObj->getPixToPhys(dims - 1, tmpBounds[1].x(), _unused));
                pxY2 = qRound(dataObj->getPixToPhys(dims - 2, tmpBounds[1].y(), _unused));

                //bounds phys to pix of xVex
                pxX1x = qRound(xVec->getPixToPhys(dimsX - 1, tmpBoundsX[0].x(), _unused));
                pxY1x = qRound(xVec->getPixToPhys(dimsX - 2, tmpBoundsX[0].y(), _unused));
                pxX2x = qRound(xVec->getPixToPhys(dimsX - 1, tmpBoundsX[1].x(), _unused));
                pxY2x = qRound(xVec->getPixToPhys(dimsX - 2, tmpBoundsX[1].y(), _unused));

                //check if values are in range
                saturation(pxX1, 0, dataObj->getSize(dims - 1) - 1);
                saturation(pxX2, 0, dataObj->getSize(dims - 1) - 1);
                saturation(pxY1, 0, dataObj->getSize(dims - 2) - 1);
                saturation(pxY2, 0, dataObj->getSize(dims - 2) - 1);


                mat = (cv::Mat*)xVec->get_mdata()[xVec->seekMat(m_dX.plane)];
                if (pxY1x == pxY2x) //pure line in x direction of x vector
                {
                    if (pxX2x > pxX1x)
                    {
                        m_dX.nrPoints = 1 + pxX2x - pxX1x;
                        m_dX.startPx.setX(pxX1x);
                        m_dX.startPx.setY(pxY1x);

                        m_dX.matOffset = (int)mat->step[0] * pxY1x + (int)mat->step[1] * pxX1x;
                        m_dX.matStepSize = (int)mat->step[1]; //step in x-direction (in bytes)
                    }
                    else
                    {
                        m_dX.nrPoints = 1 + pxX1x - pxX2x;
                        m_dX.startPx.setX(pxX2x);
                        m_dX.startPx.setY(pxY2x);
                        m_dX.matOffset = (int)mat->step[0] * pxY1x + (int)mat->step[1] * pxX2x;
                        m_dX.matStepSize = (int)mat->step[1];
                    }
                    switch (xVec->getType())
                    {
                    case ito::tUInt8:
                        sortValues<ito::uint8>(xVec);
                        break;
                    case ito::tUInt16:
                        sortValues<ito::uint16>(xVec);
                        break;
                    case ito::tInt8:
                        sortValues<ito::int8>(xVec);
                        break;
                    case ito::tInt16:
                        sortValues<ito::int16>(xVec);
                        break;
                    case ito::tInt32:
                        sortValues<ito::int32>(xVec);
                        break;
                    case ito::tFloat32:
                        sortValues<ito::float32>(xVec);
                        break;
                    case ito::tFloat64:
                        sortValues<ito::float64>(xVec);
                         break;
                    default:
                        retval += RetVal(retError, 0, "unsorported type of the dataObject representing the x-vector");
                    }
                    
                }
            }
        }
    }
        /*       

                        mat = (cv::Mat*)dataObj->get_mdata()[dataObj->seekMat(m_d.plane)]; //first plane in ROI

                        if (pxX1 == pxX2) //pure line in y-direction
                        {
                            m_d.dir = dirY;
                            if (pxY2 >= pxY1)
                            {
                                m_d.nrPoints = 1 + pxY2 - pxY1;
                                m_d.startPhys = dataObj->getPixToPhys(dims - 2, pxY1, _unused); //tmpBounds[0].y() ;
                                right = dataObj->getPixToPhys(dims - 2, pxY2, _unused); //tmpBounds[1].y();
                                m_d.stepSizePhys = m_d.nrPoints > 1 ? (right - m_d.startPhys) / (float)(m_d.nrPoints - 1) : 0.0;

                                m_d.startPx.setX(pxX1);
                                m_d.startPx.setY(pxY1);
                                m_d.stepSizePx.setWidth(0);
                                m_d.stepSizePx.setHeight(1);

                                m_d.matOffset = (int)mat->step[0] * pxY1 + (int)mat->step[1] * pxX1; //(&mat->at<char>(pxY1,pxX1) - &mat->at<char>(0,0));
                                m_d.matStepSize = (int)mat->step[0]; //step in y-direction (in bytes)
                            }
                            else
                            {
                                m_d.nrPoints = 1 + pxY1 - pxY2;
                                m_d.startPhys = dataObj->getPixToPhys(dims - 2, pxY2, _unused); //tmpBounds[1].y();
                                right = dataObj->getPixToPhys(dims - 2, pxY1, _unused); //tmpBounds[0].y();
                                m_d.stepSizePhys = m_d.nrPoints > 1 ? (right - m_d.startPhys) / (float)(m_d.nrPoints - 1) : 0.0;

                                m_d.startPx.setX(pxX1);
                                m_d.startPx.setY(pxY2);
                                m_d.stepSizePx.setWidth(0);
                                m_d.stepSizePx.setHeight(1);

                                m_d.matOffset = (int)mat->step[0] * pxY2 + (int)mat->step[1] * pxX1; //(&mat->at<char>(pxY2,pxX1) - &mat->at<char>(0,0));
                                m_d.matStepSize = (int)mat->step[0]; //step in y-direction (in bytes)
                            }

                            description = dataObj->getAxisDescription(dims - 2, _unused);
                            unit = dataObj->getAxisUnit(dims - 2, _unused);
                            if (description == "")
                            {
                                description = QObject::tr("y-axis").toLatin1().data();
                            }
                            if (unit == "")
                            {
                                m_dObjAxisDescription = fromStdLatin1String(description);
                                m_dObjAxisUnit = "";
                            }
                            else
                            {
                                m_dObjAxisDescription = fromStdLatin1String(description);
                                m_dObjAxisUnit = fromStdLatin1String(unit);
                            }

                            description = dataObj->getValueDescription();
                            unit = dataObj->getValueUnit();
                            if (unit == "")
                            {
                                m_dObjValueDescription = fromStdLatin1String(description);
                                m_dObjValueUnit = "";
                            }
                            else
                            {
                                m_dObjValueDescription = fromStdLatin1String(description);
                                m_dObjValueUnit = fromStdLatin1String(unit);
                            }
                        }
                        else if (pxY1 == pxY2) //pure line in x-direction
                        {
                            m_d.dir = dirX;
                            if (pxX2 >= pxX1)
                            {
                                m_d.nrPoints = 1 + pxX2 - pxX1;
                                m_d.startPhys = dataObj->getPixToPhys(dims - 1, pxX1, _unused); //tmpBounds[0].y() ;
                                right = dataObj->getPixToPhys(dims - 1, pxX2, _unused); //tmpBounds[1].y();
                                m_d.stepSizePhys = m_d.nrPoints > 1 ? (right - m_d.startPhys) / (float)(m_d.nrPoints - 1) : 0.0;

                                m_d.startPx.setX(pxX1);
                                m_d.startPx.setY(pxY1);
                                m_d.stepSizePx.setWidth(1);
                                m_d.stepSizePx.setHeight(0);

                                m_d.matOffset = (int)mat->step[0] * pxY1 + (int)mat->step[1] * pxX1; //(&mat->at<char>(pxY1,pxX1) - &mat->at<char>(0,0));
                                m_d.matStepSize = (int)mat->step[1]; //step in x-direction (in bytes)
                                if (dataObj->getType() == ito::tUInt32) //since uint32 is not supported by openCV the step method returns the wrong result
                                {
                                    m_d.matStepSize = 4;
                                }
                            }
                            else
                            {
                                m_d.nrPoints = 1 + pxX1 - pxX2;
                                m_d.startPhys = dataObj->getPixToPhys(dims - 1, pxX2, _unused); //tmpBounds[1].y();
                                right = dataObj->getPixToPhys(dims - 1, pxX1, _unused); //tmpBounds[0].y();
                                m_d.stepSizePhys = m_d.nrPoints > 1 ? (right - m_d.startPhys) / (float)(m_d.nrPoints - 1) : 0.0;

                                m_d.startPx.setX(pxX1);
                                m_d.startPx.setY(pxY2);
                                m_d.stepSizePx.setWidth(1);
                                m_d.stepSizePx.setHeight(0);

                                m_d.matOffset = (int)mat->step[0] * pxY1 + (int)mat->step[1] * pxX2; //(&mat->at<char>(pxY1,pxX2) - &mat->at<char>(0,0));
                                m_d.matStepSize = (int)mat->step[1]; //step in x-direction (in bytes)
                            }

                            description = dataObj->getAxisDescription(dims - 1, _unused);
                            unit = dataObj->getAxisUnit(dims - 1, _unused);
                            if (description == "")
                            {
                                description = QObject::tr("x-axis").toLatin1().data();
                            }
                            if (unit == "")
                            {
                                m_dObjAxisDescription = fromStdLatin1String(description);
                                m_dObjAxisUnit = "";
                            }
                            else
                            {
                                m_dObjAxisDescription = fromStdLatin1String(description);
                                m_dObjAxisUnit = fromStdLatin1String(unit);
                            }

                            description = dataObj->getValueDescription();
                            unit = dataObj->getValueUnit();
                            if (unit == "")
                            {
                                m_dObjValueDescription = fromStdLatin1String(description);
                                m_dObjValueUnit = "";
                            }
                            else
                            {
                                m_dObjValueDescription = fromStdLatin1String(description);
                                m_dObjValueUnit = fromStdLatin1String(unit);
                            }
                        }
                    }
                    break;
                case 1:
                    if ((dims - prependedOneDims) != 3)
                    {
                        retval += RetVal(retError, 0, "line plot in z-direction requires a 3-dim dataObject");
                        return retval;
                    }
                    else
                    {
                        m_d.valid = true;
                        pxX1 = qRound(dataObj->getPhysToPix(dims - 1, tmpBounds[0].x(), _unused));
                        pxY1 = qRound(dataObj->getPhysToPix(dims - 2, tmpBounds[0].y(), _unused));

                        saturation(pxX1, 0, dataObj->getSize(dims - 1) - 1);
                        saturation(pxX2, 0, dataObj->getSize(dims - 1) - 1);

                        m_d.dir = dirZ;
                        m_d.nrPoints = dataObj->getSize(dims - 3);
                        m_d.startPhys = dataObj->getPixToPhys(dims - 3, 0, _unused);
                        if (m_d.nrPoints > 1)
                        {
                            right = dataObj->getPixToPhys(dims - 3, m_d.nrPoints - 1, _unused);
                            m_d.stepSizePhys = (right - m_d.startPhys) / (float)(m_d.nrPoints - 1);
                        }
                        else
                        {
                            m_d.stepSizePhys = 0.0;
                        }

                        m_d.startPx.setX(pxX1);
                        m_d.startPx.setY(pxY1);
                        m_d.stepSizePx.setWidth(0);
                        m_d.stepSizePx.setHeight(0);

                        mat = (cv::Mat*)dataObj->get_mdata()[dataObj->seekMat(m_d.plane)]; //first plane in ROI
                        m_d.matOffset = (int)mat->step[0] * pxY1 + (int)mat->step[1] * pxX1; //(&mat->at<char>(pxY1,pxX1) - &mat->at<char>(0,0));
                        m_d.matStepSize = 0; //step in x-direction (in bytes)

                        description = dataObj->getAxisDescription(dims - 3, _unused);
                        unit = dataObj->getAxisUnit(dims - 3, _unused);
                        if (description == "")
                        {
                            description = QObject::tr("z-axis").toLatin1().data();
                        }
                        if (unit == "")
                        {
                            m_dObjAxisDescription = fromStdLatin1String(description);
                            m_dObjAxisUnit = "";
                        }
                        else
                        {
                            m_dObjAxisDescription = fromStdLatin1String(description);
                            m_dObjAxisUnit = fromStdLatin1String(unit);
                        }

                        description = dataObj->getValueDescription();
                        unit = dataObj->getValueUnit();
                        if (unit == "")
                        {
                            m_dObjValueDescription = fromStdLatin1String(description);
                            m_dObjValueUnit = "";
                        }
                        else
                        {
                            m_dObjValueDescription = fromStdLatin1String(description);
                            m_dObjValueUnit = fromStdLatin1String(unit);
                        }
                    }

                    break;
                default:
                    retval += RetVal(retError, 0, "bounds vector must have 1 or 2 entries");
                    break;
                }
            }
            m_pDataObj = dataObj;

            calcHash();*/
    
    return retval;
}
//----------------------------------------------------------------------------------------------------------------------------------
//QPointF DataObjectSeriesDataXY::sample(size_t n) const
//{
//
//}
////----------------------------------------------------------------------------------------------------------------------------------
//size_t DataObjectSeriesDataXY::size() const
//{
//    return m_d.nrPoints;
//}
////----------------------------------------------------------------------------------------------------------------------------------
QRectF DataObjectSeriesDataXY::boundingRect() const
{
    return DataObjectSeriesData::boundingRect();
}