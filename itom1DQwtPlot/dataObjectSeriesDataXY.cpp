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
    inSamplingMode(false),
    m_colorState(grayColor)
{
    m_dX.nrPoints = 0;
    m_dX.points.clear();
    m_dX.valid = false;
    hasXObj = true;
}

//----------------------------------------------------------------------------------------------------------------------------------
DataObjectSeriesDataXY::~DataObjectSeriesDataXY()
{
}
template<typename _Tp> void findMinMaxInteger(const ito::DataObject *obj, const DataObjectSeriesData::LineData &d, double &min, double &max, const size_t &nrPoints)
{
    const cv::Mat *mat;
    uchar *ptr;
    min = std::numeric_limits<float64>::max();
    max = -min;
    _Tp val;

    switch (d.dir)
    {
    case DataObjectSeriesData::dirX:
    case DataObjectSeriesData::dirY:
        mat = obj->getCvPlaneMat(d.plane);
        ptr = (mat->data + d.matOffset);
        for (int i = 0; i < nrPoints; i++)
        {
            val = *(reinterpret_cast<_Tp*>(ptr));
            ptr += d.matStepSize;

            if (val > max) { max = val;}
            if (val < min) { min = val;}
        }
        break;

    case DataObjectSeriesData::dirXY:
        mat = obj->getCvPlaneMat(d.plane);
        ptr = (mat->data + d.matOffset);
        for (int i = 0; i < nrPoints; i++)
        {
            val = *(reinterpret_cast<_Tp*>(ptr + d.matSteps[i]));

            if (val > max) { max = val;}
            if (val < min) { min = val;}
        }
        break;
    case DataObjectSeriesData::dirZ:

        for (int i = 0; i < nrPoints; i++)
        {
            mat = obj->getCvPlaneMat(i);
            ptr = (mat->data + d.matOffset);
            val = *(reinterpret_cast<_Tp*>(ptr));

            if (val > max) { max = val;}
            if (val < min) { min = val;}
        }
        break;
    }
}
//----------------------------------------------------------------------------------------------------------------------------------
template<typename _Tp> void findMinMaxFloat(const ito::DataObject *obj, const DataObjectSeriesData::LineData &d, double &min, double &max, const size_t &nrPoints)
{
    const cv::Mat *mat;
    uchar *ptr;
    min = std::numeric_limits<_Tp>::max();
    max = -min;
    float32 val;

    switch (d.dir)
    {
    case DataObjectSeriesData::dirX:
    case DataObjectSeriesData::dirY:
        mat = obj->getCvPlaneMat(d.plane);
        ptr = (mat->data + d.matOffset);
        for (int i = 0; i < nrPoints; i++)
        {
            val = *(reinterpret_cast<_Tp*>(ptr));
            ptr += d.matStepSize;

            if (!qIsFinite(val))
            {
                continue;
            }

            if (val > max) { max = val;}
            if (val < min) { min = val;}
        }
        break;

    case DataObjectSeriesData::dirXY:
        mat = obj->getCvPlaneMat(d.plane);
        ptr = (mat->data + d.matOffset);
        for (int i = 0; i < nrPoints; i++)
        {
            val = *(reinterpret_cast<_Tp*>(ptr + d.matSteps[i]));

            if (!qIsFinite(val))
            {
                continue;
            }

            if (val > max) { max = val;}
            if (val < min) { min = val;}
        }
        break;
    case DataObjectSeriesData::dirZ:

        for (int i = 0; i < nrPoints; i++)
        {
            mat = obj->getCvPlaneMat(i);
            ptr = (mat->data + d.matOffset);
            val = *(reinterpret_cast<_Tp*>(ptr));

            if (!qIsFinite(val))
            {
                continue;
            }

            if (val > max) { max = val;}
            if (val < min) { min = val;}
        }
        break;
    }
}

RetVal DataObjectSeriesDataXY::updateDataObject(const ito::DataObject * dataObj, QVector<QPointF> bounds, const ito::DataObject* xVec /*= NULL*/)
{
    DataObjectSeriesData::updateDataObject(dataObj, bounds);
    RetVal retval;
    bool _unused;
    QRectF p;
    float right;
    cv::Mat *mat;
    int pxX1x, pxX2x, pxY1x, pxY2x;
    std::string description, unit;

    if (dataObj == NULL )
    {
        //data representing xObj
        m_dX.plane = 0;
        m_dX.dir = dirZ;
        m_dX.nrPoints = 0;
        m_dX.points.clear();
        m_dX.matSteps.clear();
        m_dX.valid = false;
    }
    if(xVec != NULL)
    {
        int dimsX = xVec->getDims();
        int width = dimsX > 0 ? xVec->getSize(dimsX - 1) : 0;
        int height = dimsX > 1 ? xVec->getSize(dimsX - 2) : (width == 0) ? 0 : 1;

        int prependOneDimsX = 0;
        int i;
        for (i = 0; i < dimsX - 2; ++i)
        {
            if (xVec->getSize(i) != 1)
            {
                break;
            }
            ++prependOneDimsX;
        }

        QVector<QPointF> tmpBoundsX(2);

        m_dX.plane = 0;
        tmpBoundsX[0].setX(xVec->getPixToPhys(dimsX - 1, 0, _unused));
        tmpBoundsX[1].setX(xVec->getPixToPhys(dimsX - 1, width - 1, _unused));
        tmpBoundsX[0].setY(xVec->getPixToPhys(dimsX - 2,  0 , _unused));
        tmpBoundsX[1].setY(xVec->getPixToPhys(dimsX - 2, height-1, _unused));
        
        if (!xVec->get_mdata() || !(cv::Mat*)(xVec->get_mdata()[m_dX.plane])->data)
            return ito::RetVal(ito::retError, 0, QObject::tr("cv::Mat in data Object representing the x-vector seems corrupted").toLatin1().data());

        if (tmpBoundsX.size() != 2) //size will be one if a line of a Stack is extracted 
        {
            retval += RetVal(retError, 0, "bounds vector must have 2 entries");
        }
        if (!retval.containsError())
        {
            //dir X, dirY
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
                m_dX.valid = true;

                //bounds phys to pix of xVex
                pxX1x = qRound(xVec->getPixToPhys(dimsX - 1, tmpBoundsX[0].x(), _unused));
                pxY1x = qRound(xVec->getPixToPhys(dimsX - 2, tmpBoundsX[0].y(), _unused));
                pxX2x = qRound(xVec->getPixToPhys(dimsX - 1, tmpBoundsX[1].x(), _unused));
                pxY2x = qRound(xVec->getPixToPhys(dimsX - 2, tmpBoundsX[1].y(), _unused));

                saturation(pxX1x, 0, xVec->getSize(dimsX - 1) - 1);
                saturation(pxX2x, 0, xVec->getSize(dimsX - 1) - 1);
                saturation(pxY1x, 0, xVec->getSize(dimsX - 2) - 1);
                saturation(pxY2x, 0, xVec->getSize(dimsX - 2) - 1);


                mat = (cv::Mat*)xVec->get_mdata()[xVec->seekMat(m_dX.plane)];
                if (pxY1x == pxY2x) //pure line in x direction of x vector
                {
                    m_dX.dir = dirX;
                    bool xFit = false;
                    if (pxX2x > pxX1x)
                    {
                        m_dX.nrPoints = 1 + pxX2x - pxX1x;
                        if (m_dX.nrPoints != size())
                        {
                            if (m_dX.nrPoints > size())
                            {
                                retval += RetVal(retWarning, 0, "x-vector contains more values than the source dataObject. The last values will be ignored ignored.");
                            }
                            else
                            {
                                m_dX.valid = false;
                                retval += RetVal(retError, 0, "the x-vector does not contain enough values for the current source dataObject");
                            }
                        }
                        m_dX.startPx.setX(pxX1x);
                        m_dX.startPx.setY(pxY1x);
                        m_dX.matOffset = (int)mat->step[0] * pxY1x + (int)mat->step[1] * pxX1x;
                        m_dX.matStepSize = (int)mat->step[1]; //step in x-direction (in bytes)
                        m_dX.stepSizePx.setWidth(1);
                        m_dX.stepSizePx.setHeight(0);
                    }
                    else
                    {
                        m_dX.nrPoints = 1 + pxX1x - pxX2x;
                        if (m_dX.nrPoints != size())
                        {
                            if (m_dX.nrPoints > size())
                            {
                                retval += RetVal(retWarning, 0, "x-vector contains more values than the source dataObject. The last values will be ignored ignored.");
                            }
                            else
                            {
                                m_dX.valid = false;
                                retval += RetVal(retError, 0, "the x-vector does not contain enough values for the current source dataObject");
                                
                            }
                        }
                        m_dX.startPx.setX(pxX2x);
                        m_dX.startPx.setY(pxY2x);
                        m_dX.matOffset = (int)mat->step[0] * pxY1x + (int)mat->step[1] * pxX2x;
                        m_dX.matStepSize = (int)mat->step[1];
                        m_dX.stepSizePx.setWidth(1);
                        m_dX.stepSizePx.setHeight(0);
                    }                    
                }
                if (!retval.containsError())
                {
                    m_pXVec = xVec;
                    calcHash();
                }
            }
        }
    }
    
    return retval;
}
void DataObjectSeriesDataXY::calcHash()
{
    if (m_pXVec == NULL)
    {
        m_hash = QCryptographicHash::hash("", QCryptographicHash::Md5);
    }
    else
    {
        QByteArray ba;

        int dims = m_pXVec->getDims();
        ba.append(QByteArray().setNum(dims));

        if (dims > 0)
        {
            cv::Mat *m = (cv::Mat*)m_pXVec->get_mdata()[m_pXVec->seekMat(m_dX.plane)];
            uchar* d = m->data;
            ba.append(QByteArray((const char*)&d, (sizeof(int) / sizeof(char)))); //address to data of first plane

            ba.append(QByteArray().setNum(m->size[0]));
        }

        ba.append(QByteArray().setNum((uint)(m_dX.nrPoints)));
        ba.append(QByteArray().setNum(m_dX.startPx.x()));
        ba.append(QByteArray().setNum(m_dX.startPx.y()));
        ba.append(QByteArray().setNum(m_dX.stepSizePx.width()));
        ba.append(QByteArray().setNum(m_dX.stepSizePx.height()));
        m_hash = QCryptographicHash::hash(ba, QCryptographicHash::Md5);
    }
}
//----------------------------------------------------------------------------------------------------------------------------------
QPointF DataObjectSeriesDataXY::sample(size_t n) const
{
    QPointF dObjPoint = DataObjectSeriesData::sample(n);
    const cv::Mat *mat;
    const uchar* ptr[1];
    //float weights[4];
    float fPos;

    if (m_pXVec && m_dX.valid)
    {
            switch (m_dX.dir)
            {
            case dirX:
            case dirY:
                mat = m_pXVec->getCvPlaneMat(m_dX.plane);
                ptr[0] = (mat->data + m_dX.matOffset + m_dX.matStepSize * n);
                //fPos = m_d.startPhys + m_d.stepSizePhys * n;
                break;

            /*case dirZ:
                mat = m_pXVec->getCvPlaneMat((int)n);
                ptr[0] = (mat->data + m_dX.matOffset);
                //fPos = m_dX.startPhys + m_dX.stepSizePhys * n;
                break;

            case dirXY:
                mat = m_pDataObj->getCvPlaneMat(m_d.plane);
                ptr[0] = (mat->data + m_d.matOffset + m_d.matSteps[(int)n]);
                fPos = m_d.startPhys + m_d.stepSizePhys * n;
                break;*/
            default:
                qDebug() << "Type not implemented";
            }
            switch (m_pXVec->getType())
            {
            case ito::tInt8:
                dObjPoint.setX(*(reinterpret_cast<const ito::int8*>(ptr[0])));
                return dObjPoint;
                break;
            case ito::tUInt8:
                dObjPoint.setX(*(reinterpret_cast<const ito::uint8*>(ptr[0])));
                return dObjPoint;
                break;
            case ito::tInt16:
                dObjPoint.setX(*(reinterpret_cast<const ito::int16*>(ptr[0])));
                return dObjPoint;
                break;
            case ito::tUInt16:
                dObjPoint.setX(*(reinterpret_cast<const ito::uint16*>(ptr[0])));
                return dObjPoint;
                break;
            case ito::tInt32:
                dObjPoint.setX(*(reinterpret_cast<const ito::int32*>(ptr[0])));
                return dObjPoint;
                break;
            case ito::tFloat32:
                dObjPoint.setX(*(reinterpret_cast<const ito::float32*>(ptr[0])));
                return dObjPoint;
                break;
            case ito::tFloat64:
                dObjPoint.setX(*(reinterpret_cast<const ito::float64*>(ptr[0])));
                return dObjPoint;
                break;
            default:
                qDebug() << "Type not implemented yet";
            }
        }
    return QPointF();
}
//----------------------------------------------------------------------------------------------------------------------------------
QRectF DataObjectSeriesDataXY::boundingRect() const
{
    QRectF rect = DataObjectSeriesData::boundingRect();
    size_t samplesY = DataObjectSeriesData::size(); //ask for the number of y-values since this is the maximum numbers of points to be displayed


    //cv::Mat *mat;
    //const uchar* ptr[4];
    //float weights[4];

    if (m_pXVec && m_dX.valid)
    {
        double min = 0.0, max = 0.0;
        switch (m_pXVec->getType())
        {
        case ito::tInt8:
            findMinMaxInteger<ito::int8>(m_pXVec, m_dX, min, max, samplesY);
            break;
        case ito::tUInt8:
            findMinMaxInteger<ito::uint8>(m_pXVec, m_dX, min, max, samplesY);
            break;
        case ito::tInt16:
            findMinMaxInteger<ito::int16>(m_pXVec, m_dX, min, max, samplesY);
            break;
        case ito::tUInt16:
            findMinMaxInteger<ito::uint16>(m_pXVec, m_dX, min, max, samplesY);
            break;
        case ito::tInt32:
            findMinMaxInteger<ito::int32>(m_pXVec, m_dX, min, max, samplesY);
            break;
        case ito::tUInt32:
            findMinMaxInteger<ito::uint32>(m_pXVec, m_dX, min, max, samplesY);
            break;
        case ito::tFloat32:
            findMinMaxFloat<ito::float32>(m_pXVec, m_dX, min, max, samplesY);
            break;
        case ito::tFloat64:
            findMinMaxFloat<ito::float64>(m_pXVec, m_dX, min, max, samplesY);
            break;
        }

        if ((max - min) < std::numeric_limits<double>::epsilon())
        {
            if (min > 10.0)
            {
                min *= 0.99;
                max *= 1.01;
            }
            else if (min < -10.0)
            {
                min *= 1.01;
                max *= 0.99;
            }
            else
            {
                min -= 0.1;
                max += 0.1;
            }


        }
        float width = max-min;
        if (width < 0)
        {
            min += width;
            width *= -1;
        }
        rect.setX(min);
        rect.setWidth(width);
        return rect;
    }
}