/* ********************************************************************
   itom measurement system
   URL: http://www.uni-stuttgart.de/ito
   Copyright (C) 2022, Institut für Technische Optik (ITO),
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

#include "dataObjectSeriesDataXY.h"
#include "common/typeDefs.h"
#include "qwt_date.h"
#include <qcryptographichash.h>

#include <qdebug.h>
#include <qnumeric.h>

//-------------------------------------------------------------------------------------
DataObjectSeriesDataXY::DataObjectSeriesDataXY(const int fastmode) : DataObjectSeriesData(fastmode)
{
    m_dX.nrPoints = 0;
    m_dX.points.clear();
    m_dX.valid = false;
    m_hasXObj = true;
}

//-------------------------------------------------------------------------------------
DataObjectSeriesDataXY::~DataObjectSeriesDataXY()
{
}

//-------------------------------------------------------------------------------------
template <typename _Tp>
void findMinMaxInteger(
    const ito::DataObject* obj,
    const DataObjectSeriesData::LineData& d,
    double& min,
    double& max,
    const size_t& nrPoints)
{
    const cv::Mat* mat;
    const uchar* ptr;
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
            val = *(reinterpret_cast<const _Tp*>(ptr));
            ptr += d.matStepSize;

            if (val > max)
            {
                max = val;
            }
            if (val < min)
            {
                min = val;
            }
        }
        break;

    case DataObjectSeriesData::dirXY:
        mat = obj->getCvPlaneMat(d.plane);
        ptr = (mat->data + d.matOffset);
        for (int i = 0; i < nrPoints; i++)
        {
            val = *(reinterpret_cast<const _Tp*>(ptr + d.matSteps[i]));

            if (val > max)
            {
                max = val;
            }
            if (val < min)
            {
                min = val;
            }
        }
        break;
    case DataObjectSeriesData::dirZ:

        for (int i = 0; i < nrPoints; i++)
        {
            mat = obj->getCvPlaneMat(i);
            ptr = (mat->data + d.matOffset);
            val = *(reinterpret_cast<const _Tp*>(ptr));

            if (val > max)
            {
                max = val;
            }
            if (val < min)
            {
                min = val;
            }
        }
        break;
    }
}

//-------------------------------------------------------------------------------------
void findMinMaxDateTime(
    const ito::DataObject* obj,
    const DataObjectSeriesData::LineData& d,
    double& minMsSinceEpoch,
    double& maxMsSinceEpoch,
    const size_t& nrPoints)
{
    const cv::Mat* mat;
    const uchar* ptr;
    minMsSinceEpoch = std::numeric_limits<float64>::max();
    maxMsSinceEpoch = -minMsSinceEpoch;
    double val;

    switch (d.dir)
    {
    case DataObjectSeriesData::dirX:
    case DataObjectSeriesData::dirY:
        mat = obj->getCvPlaneMat(d.plane);
        ptr = (mat->data + d.matOffset);

        for (int i = 0; i < nrPoints; i++)
        {
            val = reinterpret_cast<const ito::DateTime*>(ptr)->datetime / 1000.0;
            ptr += d.matStepSize;

            if (val > maxMsSinceEpoch)
            {
                maxMsSinceEpoch = val;
            }
            if (val < minMsSinceEpoch)
            {
                minMsSinceEpoch = val;
            }
        }
        break;

    case DataObjectSeriesData::dirXY:
        mat = obj->getCvPlaneMat(d.plane);
        ptr = (mat->data + d.matOffset);
        for (int i = 0; i < nrPoints; i++)
        {
            val = reinterpret_cast<const ito::DateTime*>(ptr)->datetime / 1000.0;

            if (val > maxMsSinceEpoch)
            {
                maxMsSinceEpoch = val;
            }
            if (val < minMsSinceEpoch)
            {
                minMsSinceEpoch = val;
            }
        }
        break;
    case DataObjectSeriesData::dirZ:

        for (int i = 0; i < nrPoints; i++)
        {
            mat = obj->getCvPlaneMat(i);
            ptr = (mat->data + d.matOffset);
            val = reinterpret_cast<const ito::DateTime*>(ptr)->datetime / 1000.0;

            if (val > maxMsSinceEpoch)
            {
                maxMsSinceEpoch = val;
            }
            if (val < minMsSinceEpoch)
            {
                minMsSinceEpoch = val;
            }
        }
        break;
    }
}

//-------------------------------------------------------------------------------------
template <typename _Tp>
void findMinMaxFloat(
    const ito::DataObject* obj,
    const DataObjectSeriesData::LineData& d,
    double& min,
    double& max,
    const size_t& nrPoints)
{
    const cv::Mat* mat;
    const uchar* ptr;
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
            val = *(reinterpret_cast<const _Tp*>(ptr));
            ptr += d.matStepSize;

            if (!qIsFinite(val))
            {
                continue;
            }

            if (val > max)
            {
                max = val;
            }
            if (val < min)
            {
                min = val;
            }
        }
        break;

    case DataObjectSeriesData::dirXY:
        mat = obj->getCvPlaneMat(d.plane);
        ptr = (mat->data + d.matOffset);
        for (int i = 0; i < nrPoints; i++)
        {
            val = *(reinterpret_cast<const _Tp*>(ptr + d.matSteps[i]));

            if (!qIsFinite(val))
            {
                continue;
            }

            if (val > max)
            {
                max = val;
            }
            if (val < min)
            {
                min = val;
            }
        }
        break;
    case DataObjectSeriesData::dirZ:

        for (int i = 0; i < nrPoints; i++)
        {
            mat = obj->getCvPlaneMat(i);
            ptr = (mat->data + d.matOffset);
            val = *(reinterpret_cast<const _Tp*>(ptr));

            if (!qIsFinite(val))
            {
                continue;
            }

            if (val > max)
            {
                max = val;
            }
            if (val < min)
            {
                min = val;
            }
        }
        break;
    }
}

//---------------------------------------------------------------------------------------------------
RetVal DataObjectSeriesDataXY::updateDataObject(
    const ito::DataObject* dataObj,
    QVector<QPointF> bounds,
    const ito::DataObject* xVec /*= NULL*/,
    const QVector<QPointF>& boundsX /*= QVector<QPointF>()*/)
{
    DataObjectSeriesData::updateDataObject(dataObj, bounds);
    RetVal retval;
    bool _unused;
    QRectF p;
    cv::Mat* mat;
    int pxX1x, pxX2x, pxY1x, pxY2x;
    std::string description, unit;

    if (dataObj == NULL)
    {
        // data representing xObj
        m_dX.plane = 0;
        m_dX.dir = dirZ;
        m_dX.nrPoints = 0;
        m_dX.points.clear();
        m_dX.matSteps.clear();
        m_dX.valid = false;
    }

    if (xVec != NULL)
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

        m_dX.plane = 0;

        if (!xVec->get_mdata() || !(cv::Mat*)(xVec->get_mdata()[m_dX.plane])->data)
        {
            return ito::RetVal(
                ito::retError,
                0,
                QObject::tr("cv::Mat in data Object representing the xData seems corrupted")
                    .toLatin1()
                    .data());
        }

        if (!retval.containsError())
        {
            m_dX.valid = true;


            pxX1x = qRound(xVec->getPhysToPix(dimsX - 1, boundsX[0].x(), _unused));
            pxY1x = qRound(xVec->getPhysToPix(dimsX - 2, boundsX[0].y(), _unused));
            pxX2x = qRound(xVec->getPhysToPix(dimsX - 1, boundsX[1].x(), _unused));
            pxY2x = qRound(xVec->getPhysToPix(dimsX - 2, boundsX[1].y(), _unused));

            saturation(pxX1x, 0, xVec->getSize(dimsX - 1) - 1);
            saturation(pxX2x, 0, xVec->getSize(dimsX - 1) - 1);
            saturation(pxY1x, 0, xVec->getSize(dimsX - 2) - 1);
            saturation(pxY2x, 0, xVec->getSize(dimsX - 2) - 1);


            mat = (cv::Mat*)xVec->get_mdata()[xVec->seekMat(m_dX.plane)];
            if (pxY1x == pxY2x) // pure line in x direction of x vector
            {
                m_dX.dir = dirX;
                if (pxX2x > pxX1x)
                {
                    m_dX.nrPoints = 1 + pxX2x - pxX1x;
                    if (m_dX.nrPoints != size())
                    {
                        if (m_dX.nrPoints > size())
                        {
                            retval += RetVal(
                                retWarning,
                                0,
                                "xData contains more values than the source dataObject. The last "
                                "values will be ignored ignored.");
                        }
                        else
                        {
                            m_dX.valid = false;
                            retval += RetVal(
                                retError,
                                0,
                                "the xData does not contain enough values for the current source "
                                "dataObject");
                        }
                    }
                    m_dX.startPx.setX(pxX1x);
                    m_dX.startPx.setY(pxY1x);
                    m_dX.matOffset = (int)mat->step[0] * pxY1x + (int)mat->step[1] * pxX1x;
                    m_dX.matStepSize = (int)mat->step[1]; // step in x-direction (in bytes)
                    m_dX.stepSizePx.setWidth(1);
                    m_dX.stepSizePx.setHeight(0);
                }
            }
            else if (pxX1x == pxX2x)
            {
                m_dX.dir = dirY;
                if (pxY2x > pxY1x)
                {
                    m_dX.nrPoints = 1 + pxY2x - pxY1x;
                    if (m_dX.nrPoints != size())
                    {
                        if (m_dX.nrPoints > size())
                        {
                            retval += RetVal(
                                retWarning,
                                0,
                                "xData contains more values than the source dataObject. The last "
                                "values will be ignored ignored.");
                        }
                        else
                        {
                            m_dX.valid = false;
                            retval += RetVal(
                                retError,
                                0,
                                "the xData does not contain enough values for the current source "
                                "dataObject");
                        }
                    }
                }
                m_dX.startPx.setX(pxX1x);
                m_dX.startPx.setY(pxY1x);
                m_dX.matOffset = (int)mat->step[0] * pxY1x + (int)mat->step[1] * pxX1x;
                m_dX.stepSizePx.setWidth(0);
                m_dX.stepSizePx.setHeight(1);
                m_dX.matStepSize = (int)mat->step[0]; // step in y-direction (in bytes)
            }
            else
            {
                retval += RetVal(retError, 0, "received invalid bounds.");
            }
        }

        if (!retval.containsError())
        {
            description = xVec->getValueDescription();
            unit = xVec->getValueUnit();

            if (unit != "" ||
                description !=
                    "") // xVec value description dominates over source axis- and unit description
            {
                if (description == "")
                {
                    description = "x-Axis";
                }
                m_dObjAxisDescription = fromStdLatin1String(description);
                m_dObjAxisUnit = fromStdLatin1String(unit);
            }

            switch (xVec->getType())
            {
            case ito::tUInt8:
            case ito::tInt8:
            case ito::tUInt16:
            case ito::tInt16:
            case ito::tUInt32:
            case ito::tInt32:
                m_xCoordsWholeNumber = true;
                break;
            default:
                m_xCoordsWholeNumber = false;
                break;
            }

            m_pXVec = xVec;

            calcHash();
        }
    }

    return retval;
}

//----------------------------------------------------------------
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
            cv::Mat* m = (cv::Mat*)m_pXVec->get_mdata()[m_pXVec->seekMat(m_dX.plane)];
            uchar* d = m->data;
            ba.append(QByteArray(
                (const char*)&d, (sizeof(int) / sizeof(char)))); // address to data of first plane

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
    const cv::Mat* mat;
    const uchar* ptr[1];

    if (m_pXVec && m_dX.valid)
    {
        switch (m_dX.dir)
        {
        case dirX:
        case dirY:
            mat = m_pXVec->getCvPlaneMat(m_dX.plane);
            ptr[0] = (mat->data + m_dX.matOffset + m_dX.matStepSize * n);
            // fPos = m_d.startPhys + m_d.stepSizePhys * n;
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
            if (qIsNaN(dObjPoint.x()))
            {
                dObjPoint.setY(dObjPoint.x());
            }
            return dObjPoint;
            break;
        case ito::tFloat64:
            dObjPoint.setX(*(reinterpret_cast<const ito::float64*>(ptr[0])));
            if (qIsNaN(dObjPoint.x()))
            {
                dObjPoint.setY(dObjPoint.x());
            }
            return dObjPoint;
            break;

        case ito::tDateTime:

            // equal to QwtDate::toDouble
            dObjPoint.setX((reinterpret_cast<const ito::DateTime*>(ptr[0]))->datetime / 1000.0);

            if (qIsNaN(dObjPoint.x()))
            {
                dObjPoint.setY(dObjPoint.x());
            }

            return dObjPoint;
            break;
        default:
            qDebug() << "Type not implemented yet";
            return QPointF();
            break;
        }
    }
    return QPointF();
}
//----------------------------------------------------------------------------------------------------------------------------------
QRectF DataObjectSeriesDataXY::boundingRect() const
{
    QRectF rect = DataObjectSeriesData::boundingRect();
    size_t samplesY = DataObjectSeriesData::size(); // ask for the number of y-values since this is
                                                    // the maximum numbers of points to be displayed


    // cv::Mat *mat;
    // const uchar* ptr[4];
    // float weights[4];

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
        case ito::tDateTime:
            findMinMaxDateTime(m_pXVec, m_dX, min, max, samplesY);
            break;
        default:
            // todo xy: what about complex and rgba data types? are they blocked (which would be
            // reasonable) they are blocked before
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

        float width = max - min;

        if (width < 0)
        {
            min += width;
            width *= -1;
        }

        rect.setX(min);
        rect.setWidth(width);
    }

    return rect;
}

//-------------------------------------------------------------------------------------
int closestIdx(
    const DataObjectSeriesDataXY* data,
    const ito::DataObject* obj,
    const QPointF& val,
    const int indexHint)
{
    const int numSamples = (int)(data->size());
    int col;
    int idx = 0;
    double length;
    double previous = std::numeric_limits<double>::infinity();
    const double epsilon = std::numeric_limits<double>::epsilon();

    if (indexHint >= 0 && indexHint < numSamples)
    {
        // check if the position of the index hint fits perfectly to
        // val. If so, return this index.
        QPointF candidate = data->sample(indexHint);
        QPointF diff = candidate - val;

        if (qIsFinite(val.y()))
        {
            length = diff.manhattanLength();

            if (length <= epsilon)
            {
                return indexHint;
            }
        }
        else
        {
            if (diff.x() <= epsilon)
            {
                return indexHint;
            }
        }
    }

    if (qIsFinite(val.y()))
    {
        QPointF currentVal;

        for (col = 0; col < numSamples; ++col) // search along one row
        {
            currentVal = data->sample(col) - val;
            length = currentVal.manhattanLength();

            if ((previous - length) > epsilon)
            {
                previous = length;
                idx = col;
            }
        }
    }
    else
    {
        double currentDist;

        for (col = 0; col < numSamples; ++col)
        {
            currentDist = qAbs(data->sample(col).x() - val.x());

            if ((previous - currentDist) > epsilon)
            {
                previous = currentDist;
                idx = col;
            }
        }
    }

    return idx;
}

//----------------------------------------------------------------------------------------------------------------------------------
int DataObjectSeriesDataXY::getPosToPix(
    const double physx, const double physy /*= -1*/, const int indexHint /*= -1*/) const
{
    // in case of DateTime for x-axis, physx are the number of milliseconds since epoch
    // in UTC time (see QwtDate::toDouble).
    const QPointF coord(physx, physy);
    int idx = closestIdx(this, m_pXVec, coord, indexHint);
    return idx;
}

//----------------------------------------------------------------------------------------------------------------------------------
bool DataObjectSeriesDataXY::floatingPointXValues() const
{
    if (m_pXVec)
    {
        switch (m_pXVec->getType())
        {
        case ito::tFloat32:
        case ito::tFloat64:
        case ito::tComplex64:
        case ito::tComplex128:
            return true;
        }
    }
    return false;
}
