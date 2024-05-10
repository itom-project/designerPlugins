/* ********************************************************************
   itom measurement system
   URL: http://www.uni-stuttgart.de/ito
   Copyright (C) 2018, Institut für Technische Optik (ITO),
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
#include <qcryptographichash.h>

#include <qdebug.h>
#include <qnumeric.h>
#include <QtMath>

//----------------------------------------------------------------------------------------------------------------------------------
DataObjectSeriesData::DataObjectSeriesData(const int fastmode) :
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
    m_hasXObj(false),
    m_colorState(grayColor),
    m_xCoordsWholeNumber(false)
{
    m_d.nrPoints = 0;
    m_d.points.clear();
    m_d.valid = false;
}

//----------------------------------------------------------------------------------------------------------------------------------
DataObjectSeriesData::~DataObjectSeriesData()
{
}

//----------------------------------------------------------------------------------------------------------------------------------
void DataObjectSeriesData::beginSampling(const QwtScaleMap &/*xMap*/, const QwtScaleMap &/*yMap*/, const QRectF &/*canvasRect*/)
{
    inSamplingMode = true;
}

//----------------------------------------------------------------------------------------------------------------------------------
void DataObjectSeriesData::endSampling()
{
    inSamplingMode = false;
}

//----------------------------------------------------------------------------------------------------------------------------------
size_t DataObjectSeriesData::size() const
{
    return m_d.nrPoints;
}

//----------------------------------------------------------------------------------------------------------------------------------
bool DataObjectSeriesData::floatingPointValues() const
{
    if (m_pDataObj)
    {
        switch(m_pDataObj->getType())
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

//----------------------------------------------------------------------------------------------------------------------------------
int DataObjectSeriesData::getPosToPix(const double physx, const double physy /*=-1*/, const int indexHint /*= -1*/) const
{
    Q_UNUSED(physy);
    Q_UNUSED(indexHint);

    if (m_d.valid && m_d.nrPoints > 0)
    {
        double dist = (physx - m_d.startPhys);
        return qRound(dist / m_d.stepSizePhys);
    }
    else
    {
        return 0;
    }
}

//----------------------------------------------------------------------------------------------------------------------------------
void DataObjectSeriesData::calcHash()
{
    if (m_pDataObj == NULL)
    {
        m_hash = QCryptographicHash::hash("", QCryptographicHash::Md5);
    }
    else
    {
        QByteArray ba;

        int dims = m_pDataObj->getDims();
        ba.append( QByteArray().setNum( dims ) );

        if ( dims > 0 )
        {
            cv::Mat *m = (cv::Mat*)m_pDataObj->get_mdata()[m_pDataObj->seekMat(m_d.plane)];
            uchar* d = m->data;
            ba.append( QByteArray( (const char*)&d, (sizeof(int)/sizeof(char)))); //address to data of first plane

            ba.append( QByteArray().setNum( m->size[0] ));
        }

        ba.append( QByteArray().setNum( (uint)(m_d.nrPoints) ));
        ba.append( QByteArray().setNum( m_d.startPx.x() ));
        ba.append( QByteArray().setNum( m_d.startPx.y() ));
        ba.append( QByteArray().setNum( m_d.stepSizePx.width() ));
        ba.append( QByteArray().setNum( m_d.stepSizePx.height() ));
        m_hash = QCryptographicHash::hash(ba, QCryptographicHash::Md5);
    }
}

//----------------------------------------------------------------------------------------------------------------------------------
RetVal DataObjectSeriesData::updateDataObject(const ito::DataObject* dataObj, QVector<QPointF> bounds, const ito::DataObject* xVec /*= NULL*/, const QVector<QPointF>& boundsX)
{
    Q_UNUSED(boundsX);
    Q_UNUSED(xVec);
    RetVal retval;
    bool _unused;
    QRectF p;
    float right;
    cv::Mat *mat;
    int pxX1, pxX2, pxY1, pxY2;
    std::string description, unit;

    if (dataObj == NULL)
    {
        m_d.plane = 0;
        m_d.dir = dirZ;
        m_d.nrPoints = 0;
        m_d.points.clear();
        m_d.matSteps.clear();
        m_d.valid = false;
    }
    else
    {
        int dims = dataObj->getDims();

        int prependedOneDims = 0;
        for (int i = 0; i < dims-2; i++)
        {
            if (dataObj->getSize(i) != 1)
            {
                break;
            }
            prependedOneDims++;
        }

        QVector<QPointF> tmpBounds;

        if (bounds.size() == 3)
        {
            m_d.plane = bounds[0].x();
            m_d.plane = dims > 2 ? std::min(m_d.plane, dataObj->getSize(dims-3)) : 0;
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
        if (!dataObj->get_mdata() || !(cv::Mat*)(dataObj->get_mdata()[m_d.plane])->data)
            return ito::RetVal(ito::retError, 0, QObject::tr("cv:Mat in data object seems corrupted").toLatin1().data());

        switch( tmpBounds.size() )
        {
        case 2: //dirX, dirY or dirXY

            if ( (dims-prependedOneDims) != 2 && (dims-prependedOneDims) != 3)
            {
                m_d.valid = false;
                retval += RetVal(retError, 0, "line plot requires a 2-dim dataObject or the first (n-2) dimensions must have a size of 1");
            }
            else
            {
                m_d.valid = true;
                pxX1 = qRound(dataObj->getPhysToPix(dims-1, tmpBounds[0].x(), _unused));
                pxY1 = qRound(dataObj->getPhysToPix(dims-2, tmpBounds[0].y(), _unused));
                pxX2 = qRound(dataObj->getPhysToPix(dims-1, tmpBounds[1].x(), _unused));
                pxY2 = qRound(dataObj->getPhysToPix(dims-2, tmpBounds[1].y(), _unused));

                saturation( pxX1, 0, dataObj->getSize(dims-1)-1 );
                saturation( pxX2, 0, dataObj->getSize(dims-1)-1 );
                saturation( pxY1, 0, dataObj->getSize(dims-2)-1 );
                saturation( pxY2, 0, dataObj->getSize(dims-2)-1 );

                mat = (cv::Mat*)dataObj->get_mdata()[ dataObj->seekMat(m_d.plane) ]; //first plane in ROI

                if ( pxX1 == pxX2 ) //pure line in y-direction
                {
                    m_d.dir = dirY;
                    if (pxY2 >= pxY1)
                    {
                        m_d.nrPoints = 1 + pxY2 - pxY1;
                    }
                    else
                    {
                        m_d.nrPoints = 1 + pxY1 - pxY2;
                    }
                    m_d.startPhys= dataObj->getPixToPhys(dims-2, pxY1, _unused); //tmpBounds[0].y() ;
                    right = dataObj->getPixToPhys(dims-2, pxY2, _unused); //tmpBounds[1].y();
                    m_d.stepSizePhys = m_d.nrPoints > 1 ? (right - m_d.startPhys) / (float)(m_d.nrPoints-1) : 0.0;

                    m_d.startPx.setX(pxX1);
                    m_d.startPx.setY(pxY1);
                    m_d.stepSizePx.setWidth(0);
                    m_d.stepSizePx.setHeight(1);

                    m_d.matOffset = (int)mat->step[0] * pxY1 + (int)mat->step[1] * pxX1; //(&mat->at<char>(pxY1,pxX1) - &mat->at<char>(0,0));
                    if (pxY2 >= pxY1)
                    {
                        m_d.matStepSize = (int)mat->step[0]; //step in y-direction (in bytes)
                    }
                    else
                    {
                        m_d.matStepSize = -(int)mat->step[0]; //step in y-direction (in bytes)
                    }


                    description = dataObj->getAxisDescription(dims-2,_unused);
                    unit = dataObj->getAxisUnit(dims-2,_unused);
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
                else if ( pxY1 == pxY2 ) //pure line in x-direction
                {
                    m_d.dir = dirX;
                    if (pxX2 >= pxX1)
                    {
                        m_d.nrPoints = 1 + pxX2 - pxX1;
                    }
                    else {
                        m_d.nrPoints = 1 + pxX1 - pxX2;
                    }

                    m_d.startPhys= dataObj->getPixToPhys(dims-1, pxX1, _unused); //tmpBounds[0].y() ;
                    right = dataObj->getPixToPhys(dims-1, pxX2, _unused); //tmpBounds[1].y();
                    m_d.stepSizePhys = m_d.nrPoints > 1 ? (right - m_d.startPhys) / (float)(m_d.nrPoints-1) : 0.0;

                    m_d.startPx.setX(pxX1);
                    m_d.startPx.setY(pxY1);
                    m_d.stepSizePx.setWidth(1);
                    m_d.stepSizePx.setHeight(0);

                    m_d.matOffset = (int)mat->step[0] * pxY1 + (int)mat->step[1] * pxX1; //(&mat->at<char>(pxY1,pxX1) - &mat->at<char>(0,0));
                    if (pxX2 >= pxX1)
                    {
                        m_d.matStepSize = (int)mat->step[1]; //step in x-direction (in bytes)
                        if (dataObj->getType() == ito::tUInt32) //since uint32 is not supported by openCV the step method returns the wrong result
                        {
                            m_d.matStepSize = 4;
                        }
                    }
                    else
                    {
                        m_d.matStepSize = -(int)mat->step[1];
                        if (dataObj->getType() == ito::tUInt32) //since uint32 is not supported by openCV the step method returns the wrong result
                        {
                            m_d.matStepSize = -4;
                        }
                    }

                    /*}
                    else
                    {
                        m_d.nrPoints = 1 + pxX1 - pxX2;
                        m_d.startPhys = dataObj->getPixToPhys(dims-1, pxX2, _unused); //tmpBounds[1].y();
                        right = dataObj->getPixToPhys(dims-1, pxX1, _unused); //tmpBounds[0].y();
                        m_d.stepSizePhys = m_d.nrPoints > 1 ? (right - m_d.startPhys) / (float)(m_d.nrPoints-1) : 0.0;

                        m_d.startPx.setX(pxX1);
                        m_d.startPx.setY(pxY2);
                        m_d.stepSizePx.setWidth(1);
                        m_d.stepSizePx.setHeight(0);

                        m_d.matOffset = (int)mat->step[0] * pxY1 + (int)mat->step[1] * pxX2; //(&mat->at<char>(pxY1,pxX2) - &mat->at<char>(0,0));
                        m_d.matStepSize= (int)mat->step[1] ; //step in x-direction (in bytes)
                    }*/

                    description = dataObj->getAxisDescription(dims-1,_unused);
                    unit = dataObj->getAxisUnit(dims-1,_unused);
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
                else
                {
                    m_d.dir = dirXY;

                    if (m_fast)
                    {
                        // simple line points calculation using Bresenham
                        // http://de.wikipedia.org/wiki/Bresenham-Algorithmus#C-Implementierung

                        int dx = abs( pxX2 - pxX1 );
                        int incx = pxX1 <= pxX2 ? 1 : -1;
                        int dy = abs( pxY2 - pxY1 );
                        int incy = pxY1 <= pxY2 ? 1 : -1;

                        m_d.nrPoints = 1 + std::max(dx,dy);

                        m_d.startPhys = 0.0;  //there is no physical starting point for diagonal lines.

                        if (m_d.nrPoints > 0)
                        {
                            double dxPhys = dataObj->getPixToPhys(dims-1, pxX2, _unused) - dataObj->getPixToPhys(dims-1, pxX1, _unused);
                            double dyPhys = dataObj->getPixToPhys(dims-2, pxY2, _unused) - dataObj->getPixToPhys(dims-2, pxY1, _unused);
                            m_d.stepSizePhys = sqrt((dxPhys * dxPhys) + (dyPhys * dyPhys)) / (m_d.nrPoints - 1);
                        }
                        else
                        {
                            m_d.stepSizePhys = 0.0;
                        }

                        m_d.startPx.setX(pxX1);
                        m_d.startPx.setY(pxY1);
                        m_d.stepSizePx.setWidth(incx);
                        m_d.stepSizePx.setHeight(incy);

                        m_d.matOffset = (int)mat->step[0] * pxY1 + (int)mat->step[1] * pxX1; //(&mat->at<char>(pxY1,pxX1) - &mat->at<char>(0,0));
                        m_d.matStepSize= 0 ;

                        int pdx, pdy, ddx, ddy, es, el;
                        if (dx>dy)
                        {
                            pdx = incx;
                            pdy = 0;
                            ddx = incx;
                            ddy = incy;
                            es = dy;
                            el = dx;
                        }
                        else
                        {
                            pdx = 0;
                            pdy = incy;
                            ddx = incx;
                            ddy = incy;
                            es = dx;
                            el = dy;
                        }

                        int err = el / 2; //0; /* error value e_xy */
                        int x = 0; //pxX1;
                        int y = 0; //pxY1;

                        m_d.matSteps.resize((int)m_d.nrPoints);

                        for (unsigned int n = 0; n < (unsigned int)m_d.nrPoints; n++)
                        {  /* loop */
                            //setPixel(x,y)
                            m_d.matSteps[n] = (int)mat->step[0] * y + (int)mat->step[1] * x;

                            err -= es;
                            if (err < 0)
                            {
                                err += el;
                                x += ddx;
                                y += ddy;
                            }
                            else
                            {
                                x += pdx;
                                y += pdy;
                            }
                        }
                    }
                    else
                    {
                        //// "full" calculation of line points using interpolation values for always four neighbouring point
                        //double dx = (tmpBounds[1].x() - tmpBounds[0].x());
                        //double dy = (tmpBounds[1].y() - tmpBounds[0].y());
                        //double sizex = dataObj->getSize(dims-1);
                        //double sizey = dataObj->getSize(dims-2);

                        //double length = sqrt((double)(dx * dx + dy * dy));
                        //double stepx, stepy;
                        //if (dx == 0)
                        //{
                        //    stepx = 0;
                        //}
                        //else
                        //{
                        //    xdirect = true;
                        //    stepx = dx / length;
                        //}

                        //if (dy == 0)
                        //{
                        //    stepy = 0;
                        //}
                        //else
                        //{
                        //    ydirect = true;
                        //    stepy = dy / length;
                        //}
                        //m_size = floor(length) + 1;

                        //if (xdirect && ydirect)
                        //{
                        //    m_startPos = 0.0;
                        //    m_physLength = sqrt((double)(xscale* xscale* dx * dx + yscale * yscale * dy * dy));
                        //}
                        //else if (ydirect)
                        //{
                        //    m_startPos = tmpBounds[0].y();
                        //    m_physLength = yscale* dy ;
                        //}
                        //else
                        //{
                        //    m_startPos = tmpBounds[0].x();
                        //    m_physLength = xscale* dx ;
                        //}

                        //m_Scaling = m_physLength / length;

                        //double xsub, ysub, xpos, ypos;

                        //m_plotPts.resize(m_size);
                        //for (unsigned int n = 0; n < m_size; n++)
                        //{
                        //    xpos = pts[0].x() + n * stepx;
                        //    ypos = pts[0].y() + n * stepy;

                        //    if (xpos >= sizex - 1)
                        //    {
                        //        if (xpos >= sizex)
                        //        {
                        //            xpos = sizex - 1;
                        //        }
                        //        m_plotPts[n].rangeX[1] =  static_cast<int32>(sizex - 1);
                        //    }
                        //    else
                        //    {
                        //        m_plotPts[n].rangeX[1] =  static_cast<int32>(floor(xpos) + 1);
                        //    }

                        //    if (ypos >= sizey - 1)
                        //    {
                        //        if (ypos >= sizey)
                        //        {
                        //            ypos = sizey - 1;
                        //        }
                        //        m_plotPts[n].rangeY[1] =  static_cast<int32>(sizey - 1);
                        //    }
                        //    else
                        //    {
                        //        m_plotPts[n].rangeY[1] =  static_cast<int32>(floor(ypos) + 1);
                        //    }

                        //    m_plotPts[n].rangeX[0] = floor(xpos);
                        //    m_plotPts[n].rangeY[0] = floor(ypos);
                        //    xsub = xpos - m_plotPts[n].rangeX[0];
                        //    ysub = ypos - m_plotPts[n].rangeY[0];
                        //    m_plotPts[n].weights[0] = (1 - xsub) * (1 - ysub);
                        //    m_plotPts[n].weights[1] = (1 - xsub) * ysub;
                        //    m_plotPts[n].weights[2] = xsub * ysub;
                        //    m_plotPts[n].weights[3] = xsub * (1 - ysub);
                        //}
                    }

                    description = dataObj->getAxisDescription(dims-2,_unused);
                    unit = dataObj->getAxisUnit(dims-2,_unused);
                    if (unit == "") unit = "px";

                    std::string descr2 = dataObj->getAxisDescription(dims-1, _unused);
                    std::string unit2 = dataObj->getAxisUnit(dims-1, _unused);
                    if (unit2 == "") unit2 = "px";

                    if (description == "" && descr2 == "")
                    {
                        if (unit == "" && unit2 == "")
                        {
                            m_dObjAxisDescription = QObject::tr("x/y-axis");
                            m_dObjAxisUnit = "";
                        }
                        else
                        {
                            m_dObjAxisDescription = QObject::tr("x/y-axis");
                            m_dObjAxisUnit = QString("%1/%2").arg( fromStdLatin1String(unit), fromStdLatin1String(unit2) );
                        }
                    }
                    else
                    {
                        if (unit == "" && unit2 == "")
                        {
                            m_dObjAxisDescription = QString("%1/%2").arg( fromStdLatin1String(description), fromStdLatin1String(descr2) );
                            m_dObjAxisUnit = "";
                        }
                        else
                        {
                            m_dObjAxisDescription = QString("%1/%2").arg( fromStdLatin1String(description), fromStdLatin1String(descr2) );
                            m_dObjAxisUnit = QString("%1/%2").arg( fromStdLatin1String(unit), fromStdLatin1String(unit2) );
                        }
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

                saturation( pxX1, 0, dataObj->getSize(dims - 1) - 1 );
                saturation( pxX2, 0, dataObj->getSize(dims - 1) - 1 );

                m_d.dir = dirZ;
                m_d.nrPoints = dataObj->getSize(dims - 3);
                m_d.startPhys = dataObj->getPixToPhys(dims - 3,0,_unused);
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

                mat = (cv::Mat*)dataObj->get_mdata()[ dataObj->seekMat(m_d.plane) ]; //first plane in ROI
                m_d.matOffset = (int)mat->step[0] * pxY1 + (int)mat->step[1] * pxX1; //(&mat->at<char>(pxY1,pxX1) - &mat->at<char>(0,0));
                m_d.matStepSize= 0 ; //step in x-direction (in bytes)

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

    if (m_d.valid)
    {
        double dStart = std::abs(m_d.startPhys - (int)m_d.startPhys);
        double dStep = std::abs(m_d.stepSizePhys - (int)m_d.stepSizePhys);
        if (dStart < 1e-10 && dStep < 1e-10)
        {
            m_xCoordsWholeNumber = true;
        }
        else
        {
            m_xCoordsWholeNumber = false;
        }
    }
    else
    {
        m_xCoordsWholeNumber = false;
    }

    calcHash();

    return retval;

    //if ((bounds.size() == 1) && (dataObjectDims > 2))
    //{
    //    m_zDirect = true;
    //}
    //else
    //{
    //    m_zDirect = false;
    //}

    //pts[0].setX(static_cast<float>(dataObj->getPhysToPix(dataObjectDims-1,bounds[0].x(),testValid)));
    //pts[0].setY(static_cast<float>(dataObj->getPhysToPix(dataObjectDims-2,bounds[0].y(),testValid)));

    //if (m_zDirect)
    //{
    //    ito::DataObject tempObj = *dataObj;
    //    dataObjectDims = tempObj.getDims();
    //    int* limits = new int[2*dataObjectDims];
    //    memset(limits, 0, sizeof(int) * 2*dataObjectDims);

    //    tempObj.locateROI(limits);

    //    limits[2*(dataObjectDims - 3)] *= -1;
    //    limits[2*(dataObjectDims - 3) + 1] *= -1;

    //    tempObj.adjustROI(dataObjectDims,limits);
    //    delete limits;

    //    m_size = tempObj.getSize(dataObjectDims - 3);
    //    m_startPos = tempObj.getPixToPhys(dataObjectDims - 3, 0.0, testValid);
    //    m_physLength = tempObj.getPixToPhys(dataObjectDims - 3, m_size, testValid) - m_startPos;
    //    m_Scaling = m_physLength / m_size;

    //    m_plotPts.resize(m_size);
    //    for (unsigned int n = 0; n < m_size; n++)
    //    {
    //        m_plotPts[n].rangeX[0] = pts[0].x();
    //        m_plotPts[n].rangeY[0] = pts[0].y();
    //        m_plotPts[n].weights[0] = 1;
    //    }
    //}
    //else //not z-direct
    //{

    //    bool xdirect = false;
    //    bool ydirect = false;
    //    if (bounds.size() == 1)
    //    {
    //        pts[1] = pts[0];
    //    }
    //    else
    //    {
    //        pts[1].setX(static_cast<float>(dataObj->getPhysToPix(dataObjectDims-1,bounds[1].x(),testValid)));
    //        pts[1].setY(static_cast<float>(dataObj->getPhysToPix(dataObjectDims-2,bounds[1].y(),testValid)));
    //    }

    //    double xscale = dataObj->getAxisScale(dataObjectDims - 1);

    //    double yscale = 1;
    //    if (dataObjectDims > 1)
    //    {
    //        yscale = dataObj->getAxisScale(dataObjectDims - 2);
    //    }

        //if (m_fast)
        //{
        //    // simple line points calculation using Bresenham
        //    int dx = abs(pts[1].x() - pts[0].x());
        //    int sx = pts[0].x() < pts[1].x() ? 1 : -1;
        //    int dy = -abs(pts[1].y() - pts[0].y());
        //    int sy = pts[0].y() < pts[1].y() ? 1 : -1;
        //    int err = dx + dy, e2 = 0; /* error value e_xy */
        //    int x = pts[0].x();
        //    int y = pts[0].y();

        //    if (dx > -dy)
        //        m_size = dx;
        //    else
        //        m_size = -dy;

        //    m_plotPts.resize(m_size);

        //    for (unsigned int n = 0; n < m_size; n++)
        //    {  /* loop */
        //        m_plotPts[n].rangeX[0] = x;
        //        m_plotPts[n].rangeY[0] = y;
        //        m_plotPts[n].weights[0] = 1;

        //        e2 = 2 * err;
        //        if (e2 > dy)
        //        {
        //            err += dy;
        //            x += sx;
        //        } /* e_xy+e_x > 0 */
        //        if (e2 < dx)
        //        {
        //            err += dx;
        //            y += sy;
        //        } /* e_xy+e_y < 0 */
        //    }
        //}
        //else
        //{
        //    // "full" calculation of line points using interpolation values for always four neighbouring point
        //    double dx = (pts[1].x() - pts[0].x());
        //    double dy = (pts[1].y() - pts[0].y());
        //    double sizex = dataObj->getSize(dataObjectDims-1);
        //    double sizey = dataObj->getSize(dataObjectDims-2);

        //    double length = sqrt((double)(dx * dx + dy * dy));
        //    double stepx, stepy;
        //    if (dx == 0)
        //    {
        //        stepx = 0;
        //    }
        //    else
        //    {
        //        xdirect = true;
        //        stepx = dx / length;
        //    }

        //    if (dy == 0)
        //    {
        //        stepy = 0;
        //    }
        //    else
        //    {
        //        ydirect = true;
        //        stepy = dy / length;
        //    }
        //    m_size = floor(length) + 1;

        //    if (xdirect && ydirect)
        //    {
        //        m_startPos = 0.0;
        //        m_physLength = sqrt((double)(xscale* xscale* dx * dx + yscale * yscale * dy * dy));
        //    }
        //    else if (ydirect)
        //    {
        //        m_startPos = bounds[0].y();
        //        m_physLength = yscale* dy ;
        //    }
        //    else
        //    {
        //        m_startPos = bounds[0].x();
        //        m_physLength = xscale* dx ;
        //    }

        //    m_Scaling = m_physLength / length;

        //    double xsub, ysub, xpos, ypos;

        //    m_plotPts.resize(m_size);
        //    for (unsigned int n = 0; n < m_size; n++)
        //    {
        //        xpos = pts[0].x() + n * stepx;
        //        ypos = pts[0].y() + n * stepy;

        //        if (xpos >= sizex - 1)
        //        {
        //            if (xpos >= sizex)
        //            {
        //                xpos = sizex - 1;
        //            }
        //            m_plotPts[n].rangeX[1] =  static_cast<int32>(sizex - 1);
        //        }
        //        else
        //        {
        //            m_plotPts[n].rangeX[1] =  static_cast<int32>(floor(xpos) + 1);
        //        }

        //        if (ypos >= sizey - 1)
        //        {
        //            if (ypos >= sizey)
        //            {
        //                ypos = sizey - 1;
        //            }
        //            m_plotPts[n].rangeY[1] =  static_cast<int32>(sizey - 1);
        //        }
        //        else
        //        {
        //            m_plotPts[n].rangeY[1] =  static_cast<int32>(floor(ypos) + 1);
        //        }

        //        m_plotPts[n].rangeX[0] = floor(xpos);
        //        m_plotPts[n].rangeY[0] = floor(ypos);
        //        xsub = xpos - m_plotPts[n].rangeX[0];
        //        ysub = ypos - m_plotPts[n].rangeY[0];
        //        m_plotPts[n].weights[0] = (1 - xsub) * (1 - ysub);
        //        m_plotPts[n].weights[1] = (1 - xsub) * ysub;
        //        m_plotPts[n].weights[2] = xsub * ysub;
        //        m_plotPts[n].weights[3] = xsub * (1 - ysub);
        //    }
        //}
    //}
    //m_pDataObj = dataObj;
}

//----------------------------------------------------------------------------------------------------------------------------------
QPoint DataObjectSeriesData::indexRange(const QwtScaleMap &xMap, bool clipMargin) const
{
    if (m_pDataObj && m_d.valid && m_d.points.size() == 0)
    {
        int start, end;
        double xMapStartPhys = std::min(xMap.s1(), xMap.s2());
        double xMapEndPhys = std::max(xMap.s1(), xMap.s2());
        double start_ = (xMapStartPhys - m_d.startPhys) / m_d.stepSizePhys;
        double end_ = (xMapEndPhys - m_d.startPhys) / m_d.stepSizePhys;
        // z-stack with negative stepSize does not get plot due to bounding of values below, if we do not do order checking here
        if (end_ < start_)
            qSwap(start_, end_);
        start = clipMargin ? qBound(0, qCeil(start_), m_d.nrPoints - 1) : qBound(0, qCeil(start_) - 1, m_d.nrPoints - 1);
        end = clipMargin ? qBound(start, qFloor(end_), m_d.nrPoints - 1) : qBound(start, qFloor(end_) + 1, m_d.nrPoints - 1);
        return QPoint(start, end);
    }
    else
    {
        return QPoint(0, m_d.points.size() - 1);
    }
}

//----------------------------------------------------------------------------------------------------------------------------------
QPointF DataObjectSeriesData::sample(size_t n) const
{
    const cv::Mat *mat;
    const uchar* ptr[4];
    //float weights[4];
    float fPos;

    if (m_pDataObj && m_d.valid)
    {
        if (m_d.points.size() == 0) //no weighted stuff
        {
            switch (m_d.dir)
            {
                case dirX:
                case dirY:
                    mat = m_pDataObj->getCvPlaneMat(m_d.plane);
                    ptr[0] = (mat->data + m_d.matOffset + m_d.matStepSize * n);
                    fPos = m_d.startPhys + m_d.stepSizePhys * n;
                break;

                case dirZ:
                    mat = m_pDataObj->getCvPlaneMat((int)n);
                    ptr[0] = (mat->data + m_d.matOffset);
                    fPos = m_d.startPhys + m_d.stepSizePhys * n;
                break;

                case dirXY:
                    mat = m_pDataObj->getCvPlaneMat(m_d.plane);
                    ptr[0] = (mat->data + m_d.matOffset + m_d.matSteps[(int)n]);
                    fPos = m_d.startPhys + m_d.stepSizePhys * n;
                break;
            }

            switch(m_pDataObj->getType())
            {
                case ito::tInt8:
                    return QPointF(fPos, *(reinterpret_cast<const ito::int8*>(ptr[0])) );
                break;
                case ito::tUInt8:
                    return QPointF(fPos, *(reinterpret_cast<const ito::uint8*>(ptr[0])) );
                break;
                case ito::tInt16:
                    return QPointF(fPos, *(reinterpret_cast<const ito::int16*>(ptr[0])) );
                break;
                case ito::tUInt16:
                    return QPointF(fPos, *(reinterpret_cast<const ito::uint16*>(ptr[0])) );
                break;
                case ito::tInt32:
                    return QPointF(fPos, *(reinterpret_cast<const ito::int32*>(ptr[0])) );
                break;
                case ito::tUInt32:
                    return QPointF(fPos, *(reinterpret_cast<const ito::uint32*>(ptr[0])) );
                break;
                case ito::tFloat32:
                    return QPointF(fPos, *(reinterpret_cast<const ito::float32*>(ptr[0])) );
                break;
                case ito::tFloat64:
                    return QPointF(fPos, *(reinterpret_cast<const ito::float64*>(ptr[0])) );
                break;
                case ito::tComplex64:
                {
                    ito::complex64 val = *(reinterpret_cast<const ito::complex64*>(ptr[0]));
                    switch (m_cmplxState)
                    {
                    default:
                    case ItomQwtPlotEnums::CmplxAbs:
                        return QPointF(fPos, abs(val));
                    break;
                    case ItomQwtPlotEnums::CmplxReal:
                        return QPointF(fPos, val.real());
                    break;
                    case ItomQwtPlotEnums::CmplxImag:
                        return QPointF(fPos, val.imag());
                    break;
                    case ItomQwtPlotEnums::CmplxArg:
                        return QPointF(fPos, arg(val));
                    break;
                    }
                    }
                break;
                case ito::tComplex128:
                {
                    ito::complex128 val = *(reinterpret_cast<const ito::complex128*>(ptr[0]));
                    switch (m_cmplxState)
                    {
                        default:
                    case ItomQwtPlotEnums::CmplxAbs:
                            return QPointF(fPos, abs(val));
                        break;
                    case ItomQwtPlotEnums::CmplxReal:
                            return QPointF(fPos, val.real());
                        break;
                    case ItomQwtPlotEnums::CmplxImag:
                            return QPointF(fPos, val.imag());
                        break;
                    case ItomQwtPlotEnums::CmplxArg:
                            return QPointF(fPos, arg(val));
                        break;
                    }
                }
                break;
                case ito::tRGBA32:
                    switch (m_colorState)
                    {
                        default:
                        case grayColor:
                            return QPointF(fPos, (reinterpret_cast<const ito::Rgba32*>(ptr[0]))->gray() );
                        break;
                        case blueColor:
                            return QPointF(fPos, (reinterpret_cast<const ito::Rgba32*>(ptr[0]))->blue() );
                        break;
                        case greenColor:
                            return QPointF(fPos, (reinterpret_cast<const ito::Rgba32*>(ptr[0]))->green() );
                        break;
                        case redColor:
                            return QPointF(fPos, (reinterpret_cast<const ito::Rgba32*>(ptr[0]))->red() );
                        break;
                        case alphaColor:
                            return QPointF(fPos, (reinterpret_cast<const ito::Rgba32*>(ptr[0]))->alpha() );
                        break;
                    }

                break;
            }
        }
        else
        {
            qDebug() << "not yet implemented";
            //not implemented yet
            return QPointF();
        }
    }

    return QPointF();

    //if (m_pDataObj)
    //{
    //    if (m_pDataObj->getDims() == 2)
    //    {
    //        double fPos;

    //        if (m_Scaling > 0) fPos = n * m_Scaling + m_startPos;
    //        else fPos = n * abs(m_Scaling) + m_startPos  + m_physLength;

    //        if (m_fast)
    //        {
    //            switch(m_pDataObj->getType())
    //            {
    //                case ito::tInt8:
    //                    return QPointF(fPos, m_pDataObj->at<int8>(m_plotPts[n].rangeY[0], m_plotPts[n].rangeX[0]));
    //                break;
    //                case ito::tUInt8:
    //                    return QPointF(fPos, m_pDataObj->at<uint8>(m_plotPts[n].rangeY[0], m_plotPts[n].rangeX[0]));
    //                break;
    //                case ito::tInt16:
    //                    return QPointF(fPos, m_pDataObj->at<int16>(m_plotPts[n].rangeY[0], m_plotPts[n].rangeX[0]));
    //                break;
    //                case ito::tUInt16:
    //                    return QPointF(fPos, m_pDataObj->at<uint16>(m_plotPts[n].rangeY[0], m_plotPts[n].rangeX[0]));
    //                break;
    //                case ito::tInt32:
    //                    return QPointF(fPos, m_pDataObj->at<int32>(m_plotPts[n].rangeY[0], m_plotPts[n].rangeX[0]));
    //                break;
    //                case ito::tUInt32:
    //                    return QPointF(fPos, m_pDataObj->at<uint32>(m_plotPts[n].rangeY[0], m_plotPts[n].rangeX[0]));
    //                break;
    //                case ito::tFloat32:
    //                    return QPointF(fPos, m_pDataObj->at<float>(m_plotPts[n].rangeY[0], m_plotPts[n].rangeX[0]));
    //                break;
    //                case ito::tFloat64:
    //                    return QPointF(fPos, m_pDataObj->at<double>(m_plotPts[n].rangeY[0], m_plotPts[n].rangeX[0]));
    //                break;
    //                case ito::tComplex64:
                //    {
                //        switch (m_cmplxState)
                //        {
                //            default:
    //                        case cmplxAbs:
                //                return QPointF(fPos, abs(m_pDataObj->at<ito::complex64>(m_plotPts[n].rangeY[0], m_plotPts[n].rangeX[0])));
                //            break;
                //            case cmplxReal:
                //                return QPointF(fPos, (m_pDataObj->at<ito::complex64>(m_plotPts[n].rangeY[0], m_plotPts[n].rangeX[0]).real()));
                //            break;
                //            case cmplxImag:
                //                return QPointF(fPos, (m_pDataObj->at<ito::complex64>(m_plotPts[n].rangeY[0], m_plotPts[n].rangeX[0]).imag()));
                //            break;
                //            case cmplxArg:
                //                return QPointF(fPos, arg(m_pDataObj->at<ito::complex64>(m_plotPts[n].rangeY[0], m_plotPts[n].rangeX[0])));
                //            break;
                //        }
                //    }
    //                break;
    //                case ito::tComplex128:
                //    {
                //        switch (m_cmplxState)
                //        {
                //            default:
                //            case cmplxAbs:
                //                return QPointF(fPos, abs(m_pDataObj->at<ito::complex128>(m_plotPts[n].rangeY[0], m_plotPts[n].rangeX[0])));
                //            break;
                //            case cmplxReal:
                //                return QPointF(fPos, (m_pDataObj->at<ito::complex128>(m_plotPts[n].rangeY[0], m_plotPts[n].rangeX[0]).real()));
                //            break;
                //            case cmplxImag:
                //                return QPointF(fPos, (m_pDataObj->at<ito::complex128>(m_plotPts[n].rangeY[0], m_plotPts[n].rangeX[0]).imag()));
                //            break;
                //            case cmplxArg:
                //                return QPointF(fPos, arg(m_pDataObj->at<ito::complex128>(m_plotPts[n].rangeY[0], m_plotPts[n].rangeX[0])));
                //            break;
                //        }
                //    }
    //                break;
    //            }
    //        }
    //        else // if m_fast
    //        {
    //            double val;

    //            switch(m_pDataObj->getType())
    //            {
    //                case ito::tInt8:
    //                    val = m_pDataObj->at<int8>(m_plotPts[n].rangeY[0], m_plotPts[n].rangeX[0]) * m_plotPts[n].weights[0] +
    //                        m_pDataObj->at<int8>(m_plotPts[n].rangeY[1], m_plotPts[n].rangeX[0]) * m_plotPts[n].weights[1] +
    //                        m_pDataObj->at<int8>(m_plotPts[n].rangeY[1], m_plotPts[n].rangeX[1]) * m_plotPts[n].weights[2] +
    //                        m_pDataObj->at<int8>(m_plotPts[n].rangeY[0], m_plotPts[n].rangeX[1]) * m_plotPts[n].weights[3];
    //                break;
    //                case ito::tUInt8:
    //                    val = m_pDataObj->at<uint8>(m_plotPts[n].rangeY[0], m_plotPts[n].rangeX[0]) * m_plotPts[n].weights[0] +
    //                        m_pDataObj->at<uint8>(m_plotPts[n].rangeY[1], m_plotPts[n].rangeX[0]) * m_plotPts[n].weights[1] +
    //                        m_pDataObj->at<uint8>(m_plotPts[n].rangeY[1], m_plotPts[n].rangeX[1]) * m_plotPts[n].weights[2] +
    //                        m_pDataObj->at<uint8>(m_plotPts[n].rangeY[0], m_plotPts[n].rangeX[1]) * m_plotPts[n].weights[3];
    //                break;
    //                case ito::tInt16:
    //                    val = m_pDataObj->at<int16>(m_plotPts[n].rangeY[0], m_plotPts[n].rangeX[0]) * m_plotPts[n].weights[0] +
    //                        m_pDataObj->at<int16>(m_plotPts[n].rangeY[1], m_plotPts[n].rangeX[0]) * m_plotPts[n].weights[1] +
    //                        m_pDataObj->at<int16>(m_plotPts[n].rangeY[1], m_plotPts[n].rangeX[1]) * m_plotPts[n].weights[2] +
    //                        m_pDataObj->at<int16>(m_plotPts[n].rangeY[0], m_plotPts[n].rangeX[1]) * m_plotPts[n].weights[3];
    //                break;
    //                case ito::tUInt16:
    //                    val = m_pDataObj->at<uint16>(m_plotPts[n].rangeY[0], m_plotPts[n].rangeX[0]) * m_plotPts[n].weights[0] +
    //                        m_pDataObj->at<uint16>(m_plotPts[n].rangeY[1], m_plotPts[n].rangeX[0]) * m_plotPts[n].weights[1] +
    //                        m_pDataObj->at<uint16>(m_plotPts[n].rangeY[1], m_plotPts[n].rangeX[1]) * m_plotPts[n].weights[2] +
    //                        m_pDataObj->at<uint16>(m_plotPts[n].rangeY[0], m_plotPts[n].rangeX[1]) * m_plotPts[n].weights[3];
    //                break;
    //                case ito::tInt32:
    //                    val = m_pDataObj->at<int32>(m_plotPts[n].rangeY[0], m_plotPts[n].rangeX[0]) * m_plotPts[n].weights[0] +
    //                        m_pDataObj->at<int32>(m_plotPts[n].rangeY[1], m_plotPts[n].rangeX[0]) * m_plotPts[n].weights[1] +
    //                        m_pDataObj->at<int32>(m_plotPts[n].rangeY[1], m_plotPts[n].rangeX[1]) * m_plotPts[n].weights[2] +
    //                        m_pDataObj->at<int32>(m_plotPts[n].rangeY[0], m_plotPts[n].rangeX[1]) * m_plotPts[n].weights[3];
    //                break;
    //                case ito::tUInt32:
    //                    val = m_pDataObj->at<uint32>(m_plotPts[n].rangeY[0], m_plotPts[n].rangeX[0]) * m_plotPts[n].weights[0] +
    //                        m_pDataObj->at<uint32>(m_plotPts[n].rangeY[1], m_plotPts[n].rangeX[0]) * m_plotPts[n].weights[1] +
    //                        m_pDataObj->at<uint32>(m_plotPts[n].rangeY[1], m_plotPts[n].rangeX[1]) * m_plotPts[n].weights[2] +
    //                        m_pDataObj->at<uint32>(m_plotPts[n].rangeY[0], m_plotPts[n].rangeX[1]) * m_plotPts[n].weights[3];
    //                break;
    //                case ito::tFloat32:
    //                     val = m_pDataObj->at<float>(m_plotPts[n].rangeY[0], m_plotPts[n].rangeX[0]) * m_plotPts[n].weights[0] +
    //                        m_pDataObj->at<float>(m_plotPts[n].rangeY[1], m_plotPts[n].rangeX[0]) * m_plotPts[n].weights[1] +
    //                        m_pDataObj->at<float>(m_plotPts[n].rangeY[1], m_plotPts[n].rangeX[1]) * m_plotPts[n].weights[2] +
    //                        m_pDataObj->at<float>(m_plotPts[n].rangeY[0], m_plotPts[n].rangeX[1]) * m_plotPts[n].weights[3];
    //                break;
    //                case ito::tFloat64:
    //                     val = m_pDataObj->at<double>(m_plotPts[n].rangeY[0], m_plotPts[n].rangeX[0]) * m_plotPts[n].weights[0] +
    //                        m_pDataObj->at<double>(m_plotPts[n].rangeY[1], m_plotPts[n].rangeX[0]) * m_plotPts[n].weights[1] +
    //                        m_pDataObj->at<double>(m_plotPts[n].rangeY[1], m_plotPts[n].rangeX[1]) * m_plotPts[n].weights[2] +
    //                        m_pDataObj->at<double>(m_plotPts[n].rangeY[0], m_plotPts[n].rangeX[1]) * m_plotPts[n].weights[3];
    //                break;
    //                case ito::tComplex64:
                //    {
                //        switch (m_cmplxState)
                //        {
                //            default:
                //            case cmplxAbs:
                //                rVal = m_pDataObj->at<ito::complex64>(m_plotPts[n].rangeY[0], m_plotPts[n].rangeX[0]).real() * m_plotPts[n].weights[0] +
                //                    m_pDataObj->at<ito::complex64>(m_plotPts[n].rangeY[1], m_plotPts[n].rangeX[0]).real() * m_plotPts[n].weights[1] +
                //                    m_pDataObj->at<ito::complex64>(m_plotPts[n].rangeY[1], m_plotPts[n].rangeX[1]).real() * m_plotPts[n].weights[2] +
                //                    m_pDataObj->at<ito::complex64>(m_plotPts[n].rangeY[0], m_plotPts[n].rangeX[1]).real() * m_plotPts[n].weights[3];
                //                iVal = m_pDataObj->at<ito::complex64>(m_plotPts[n].rangeY[0], m_plotPts[n].rangeX[0]).imag() * m_plotPts[n].weights[0] +
                //                    m_pDataObj->at<ito::complex64>(m_plotPts[n].rangeY[1], m_plotPts[n].rangeX[0]).imag() * m_plotPts[n].weights[1] +
                //                    m_pDataObj->at<ito::complex64>(m_plotPts[n].rangeY[1], m_plotPts[n].rangeX[1]).imag() * m_plotPts[n].weights[2] +
                //                    m_pDataObj->at<ito::complex64>(m_plotPts[n].rangeY[0], m_plotPts[n].rangeX[1]).imag() * m_plotPts[n].weights[3];
                //                val = sqrt(rVal * rVal + iVal * iVal);
                //            break;
                //            case cmplxReal:
                //                val = m_pDataObj->at<ito::complex64>(m_plotPts[n].rangeY[0], m_plotPts[n].rangeX[0]).real() * m_plotPts[n].weights[0] +
                //                    m_pDataObj->at<ito::complex64>(m_plotPts[n].rangeY[1], m_plotPts[n].rangeX[0]).real() * m_plotPts[n].weights[1] +
                //                    m_pDataObj->at<ito::complex64>(m_plotPts[n].rangeY[1], m_plotPts[n].rangeX[1]).real() * m_plotPts[n].weights[2] +
                //                    m_pDataObj->at<ito::complex64>(m_plotPts[n].rangeY[0], m_plotPts[n].rangeX[1]).real() * m_plotPts[n].weights[3];
                //            break;
                //            case cmplxImag:
                //                val = m_pDataObj->at<ito::complex64>(m_plotPts[n].rangeY[0], m_plotPts[n].rangeX[0]).imag() * m_plotPts[n].weights[0] +
                //                    m_pDataObj->at<ito::complex64>(m_plotPts[n].rangeY[1], m_plotPts[n].rangeX[0]).imag() * m_plotPts[n].weights[1] +
                //                    m_pDataObj->at<ito::complex64>(m_plotPts[n].rangeY[1], m_plotPts[n].rangeX[1]).imag() * m_plotPts[n].weights[2] +
                //                    m_pDataObj->at<ito::complex64>(m_plotPts[n].rangeY[0], m_plotPts[n].rangeX[1]).imag() * m_plotPts[n].weights[3];
                //            break;
                //            case cmplxArg:
                //                rVal = m_pDataObj->at<ito::complex64>(m_plotPts[n].rangeY[0], m_plotPts[n].rangeX[0]).real() * m_plotPts[n].weights[0] +
                //                    m_pDataObj->at<ito::complex64>(m_plotPts[n].rangeY[1], m_plotPts[n].rangeX[0]).real() * m_plotPts[n].weights[1] +
                //                    m_pDataObj->at<ito::complex64>(m_plotPts[n].rangeY[1], m_plotPts[n].rangeX[1]).real() * m_plotPts[n].weights[2] +
                //                    m_pDataObj->at<ito::complex64>(m_plotPts[n].rangeY[0], m_plotPts[n].rangeX[1]).real() * m_plotPts[n].weights[3];
                //                iVal = m_pDataObj->at<ito::complex64>(m_plotPts[n].rangeY[0], m_plotPts[n].rangeX[0]).imag() * m_plotPts[n].weights[0] +
                //                    m_pDataObj->at<ito::complex64>(m_plotPts[n].rangeY[1], m_plotPts[n].rangeX[0]).imag() * m_plotPts[n].weights[1] +
                //                    m_pDataObj->at<ito::complex64>(m_plotPts[n].rangeY[1], m_plotPts[n].rangeX[1]).imag() * m_plotPts[n].weights[2] +
                //                    m_pDataObj->at<ito::complex64>(m_plotPts[n].rangeY[0], m_plotPts[n].rangeX[1]).imag() * m_plotPts[n].weights[3];
                //                val = atan2(iVal, rVal);
                //            break;
                //        }
                //    }
    //                break;
    //                case ito::tComplex128:
                //    {
                //        switch (m_cmplxState)
                //        {
                //            default:
                //            case cmplxAbs:
                //                rVal = m_pDataObj->at<ito::complex128>(m_plotPts[n].rangeY[0], m_plotPts[n].rangeX[0]).real() * m_plotPts[n].weights[0] +
                //                    m_pDataObj->at<ito::complex128>(m_plotPts[n].rangeY[1], m_plotPts[n].rangeX[0]).real() * m_plotPts[n].weights[1] +
                //                    m_pDataObj->at<ito::complex128>(m_plotPts[n].rangeY[1], m_plotPts[n].rangeX[1]).real() * m_plotPts[n].weights[2] +
                //                    m_pDataObj->at<ito::complex128>(m_plotPts[n].rangeY[0], m_plotPts[n].rangeX[1]).real() * m_plotPts[n].weights[3];
                //                iVal = m_pDataObj->at<ito::complex128>(m_plotPts[n].rangeY[0], m_plotPts[n].rangeX[0]).imag() * m_plotPts[n].weights[0] +
                //                    m_pDataObj->at<ito::complex128>(m_plotPts[n].rangeY[1], m_plotPts[n].rangeX[0]).imag() * m_plotPts[n].weights[1] +
                //                    m_pDataObj->at<ito::complex128>(m_plotPts[n].rangeY[1], m_plotPts[n].rangeX[1]).imag() * m_plotPts[n].weights[2] +
                //                    m_pDataObj->at<ito::complex128>(m_plotPts[n].rangeY[0], m_plotPts[n].rangeX[1]).imag() * m_plotPts[n].weights[3];
                //                val = sqrt(rVal * rVal + iVal * iVal);
                //            break;
                //            case cmplxReal:
                //                val = m_pDataObj->at<ito::complex128>(m_plotPts[n].rangeY[0], m_plotPts[n].rangeX[0]).real() * m_plotPts[n].weights[0] +
                //                    m_pDataObj->at<ito::complex128>(m_plotPts[n].rangeY[1], m_plotPts[n].rangeX[0]).real() * m_plotPts[n].weights[1] +
                //                    m_pDataObj->at<ito::complex128>(m_plotPts[n].rangeY[1], m_plotPts[n].rangeX[1]).real() * m_plotPts[n].weights[2] +
                //                    m_pDataObj->at<ito::complex128>(m_plotPts[n].rangeY[0], m_plotPts[n].rangeX[1]).real() * m_plotPts[n].weights[3];
                //            break;
                //            case cmplxImag:
                //                val = m_pDataObj->at<ito::complex128>(m_plotPts[n].rangeY[0], m_plotPts[n].rangeX[0]).imag() * m_plotPts[n].weights[0] +
                //                    m_pDataObj->at<ito::complex128>(m_plotPts[n].rangeY[1], m_plotPts[n].rangeX[0]).imag() * m_plotPts[n].weights[1] +
                //                    m_pDataObj->at<ito::complex128>(m_plotPts[n].rangeY[1], m_plotPts[n].rangeX[1]).imag() * m_plotPts[n].weights[2] +
                //                    m_pDataObj->at<ito::complex128>(m_plotPts[n].rangeY[0], m_plotPts[n].rangeX[1]).imag() * m_plotPts[n].weights[3];
                //            break;
    //                        case cmplxArg:
                //                rVal = m_pDataObj->at<ito::complex128>(m_plotPts[n].rangeY[0], m_plotPts[n].rangeX[0]).real() * m_plotPts[n].weights[0] +
                //                    m_pDataObj->at<ito::complex128>(m_plotPts[n].rangeY[1], m_plotPts[n].rangeX[0]).real() * m_plotPts[n].weights[1] +
                //                    m_pDataObj->at<ito::complex128>(m_plotPts[n].rangeY[1], m_plotPts[n].rangeX[1]).real() * m_plotPts[n].weights[2] +
                //                    m_pDataObj->at<ito::complex128>(m_plotPts[n].rangeY[0], m_plotPts[n].rangeX[1]).real() * m_plotPts[n].weights[3];
                //                iVal = m_pDataObj->at<ito::complex128>(m_plotPts[n].rangeY[0], m_plotPts[n].rangeX[0]).imag() * m_plotPts[n].weights[0] +
                //                    m_pDataObj->at<ito::complex128>(m_plotPts[n].rangeY[1], m_plotPts[n].rangeX[0]).imag() * m_plotPts[n].weights[1] +
                //                    m_pDataObj->at<ito::complex128>(m_plotPts[n].rangeY[1], m_plotPts[n].rangeX[1]).imag() * m_plotPts[n].weights[2] +
                //                    m_pDataObj->at<ito::complex128>(m_plotPts[n].rangeY[0], m_plotPts[n].rangeX[1]).imag() * m_plotPts[n].weights[3];
                //                val = atan2(iVal, rVal);
                //            break;
                //        }
                //    }
    //                break;
    //            }
    //            return QPointF(fPos, val);
    //        }
    //    }
    //    else if (m_zDirect)
    //    {

    //        ito::DataObject tempObj = *m_pDataObj;
    //        int dataObjectDims = tempObj.getDims();
    //        int* limits = new int[2*dataObjectDims];
    //        memset(limits, 0, sizeof(int) * 2*dataObjectDims);

    //        tempObj.locateROI(limits);

    //        limits[2*(dataObjectDims - 3)] *= -1;
    //        limits[2*(dataObjectDims - 3) + 1] *= -1;

    //        tempObj.adjustROI(dataObjectDims,limits);
    //        delete limits;

    //        double fPos;

    //        if (m_Scaling > 0) fPos = n * m_Scaling + m_startPos;
    //        else fPos = n * abs(m_Scaling) + m_startPos  + m_physLength;

    //        cv::Mat* curPlane = (cv::Mat*)tempObj.get_mdata()[tempObj.seekMat(n)];
    //        switch(m_pDataObj->getType())
    //        {
    //            case ito::tInt8:
    //                return QPointF(fPos, curPlane->at<int8>(m_plotPts[n].rangeY[0], m_plotPts[n].rangeX[0]));
    //            break;
    //            case ito::tUInt8:
    //                return QPointF(fPos, curPlane->at<uint8>(m_plotPts[n].rangeY[0], m_plotPts[n].rangeX[0]));
    //            break;
    //            case ito::tInt16:
    //                return QPointF(fPos, curPlane->at<int16>(m_plotPts[n].rangeY[0], m_plotPts[n].rangeX[0]));
    //            break;
    //            case ito::tUInt16:
    //                return QPointF(fPos, curPlane->at<uint16>(m_plotPts[n].rangeY[0], m_plotPts[n].rangeX[0]));
    //            break;
    //            case ito::tInt32:
    //                return QPointF(fPos, curPlane->at<int32>(m_plotPts[n].rangeY[0], m_plotPts[n].rangeX[0]));
    //            break;
    //            case ito::tUInt32:
    //                return QPointF(fPos, curPlane->at<uint32>(m_plotPts[n].rangeY[0], m_plotPts[n].rangeX[0]));
    //            break;
    //            case ito::tFloat32:
    //                return QPointF(fPos, curPlane->at<float>(m_plotPts[n].rangeY[0], m_plotPts[n].rangeX[0]));
    //            break;
    //            case ito::tFloat64:
    //                return QPointF(fPos, curPlane->at<double>(m_plotPts[n].rangeY[0], m_plotPts[n].rangeX[0]));
    //            break;
    //            case ito::tComplex64:
                //{
                //    switch (m_cmplxState)
                //    {
                //        default:
                //        case cmplxAbs:
                //            return QPointF(fPos, abs(curPlane->at<ito::complex64>(m_plotPts[n].rangeY[0], m_plotPts[n].rangeX[0])));
                //        break;
                //        case cmplxReal:
                //            return QPointF(fPos, (curPlane->at<ito::complex64>(m_plotPts[n].rangeY[0], m_plotPts[n].rangeX[0]).real()));
                //        break;
                //        case cmplxImag:
                //            return QPointF(fPos, (curPlane->at<ito::complex64>(m_plotPts[n].rangeY[0], m_plotPts[n].rangeX[0]).imag()));
                //        break;
                //        case cmplxArg:
                //            return QPointF(fPos, arg(curPlane->at<ito::complex64>(m_plotPts[n].rangeY[0], m_plotPts[n].rangeX[0])));
                //        break;
                //    }
                //}
    //            break;
    //            case ito::tComplex128:
                //{
                //    switch (m_cmplxState)
                //    {
                //        default:
                //        case cmplxAbs:
                //            return QPointF(fPos, abs(curPlane->at<ito::complex128>(m_plotPts[n].rangeY[0], m_plotPts[n].rangeX[0])));
                //        break;
                //        case cmplxReal:
                //            return QPointF(fPos, (curPlane->at<ito::complex128>(m_plotPts[n].rangeY[0], m_plotPts[n].rangeX[0]).real()));
                //        break;
                //        case cmplxImag:
                //            return QPointF(fPos, (curPlane->at<ito::complex128>(m_plotPts[n].rangeY[0], m_plotPts[n].rangeX[0]).imag()));
                //        break;
                //        case cmplxArg:
                //            return QPointF(fPos, arg(curPlane->at<ito::complex128>(m_plotPts[n].rangeY[0], m_plotPts[n].rangeX[0])));
                //        break;
                //    }
                //}
    //            break;
    //        }
    //        return QPointF();
    //    } // if m_zDirect
    //    else
    //    {
    //        double fPos;

    //        if (m_Scaling > 0) fPos = n * m_Scaling + m_startPos;
    //        else fPos = n * abs(m_Scaling) + m_startPos  + m_physLength;

    //        cv::Mat* curPlane = (cv::Mat*)m_pDataObj->get_mdata()[m_pDataObj->seekMat(0)];
    //        if (m_fast)
    //        {
    //            switch(m_pDataObj->getType())
    //            {
    //                case ito::tInt8:
    //                    return QPointF(fPos, curPlane->at<int8>(m_plotPts[n].rangeY[0], m_plotPts[n].rangeX[0]));
    //                break;
    //                case ito::tUInt8:
    //                    return QPointF(fPos, curPlane->at<uint8>(m_plotPts[n].rangeY[0], m_plotPts[n].rangeX[0]));
    //                break;
    //                case ito::tInt16:
    //                    return QPointF(fPos, curPlane->at<int16>(m_plotPts[n].rangeY[0], m_plotPts[n].rangeX[0]));
    //                break;
    //                case ito::tUInt16:
    //                    return QPointF(fPos, curPlane->at<uint16>(m_plotPts[n].rangeY[0], m_plotPts[n].rangeX[0]));
    //                break;
    //                case ito::tInt32:
    //                    return QPointF(fPos, curPlane->at<int32>(m_plotPts[n].rangeY[0], m_plotPts[n].rangeX[0]));
    //                break;
    //                case ito::tUInt32:
    //                    return QPointF(fPos, curPlane->at<uint32>(m_plotPts[n].rangeY[0], m_plotPts[n].rangeX[0]));
    //                break;
    //                case ito::tFloat32:
    //                    return QPointF(fPos, curPlane->at<float>(m_plotPts[n].rangeY[0], m_plotPts[n].rangeX[0]));
    //                break;
    //                case ito::tFloat64:
    //                    return QPointF(fPos, curPlane->at<double>(m_plotPts[n].rangeY[0], m_plotPts[n].rangeX[0]));
    //                break;
                //    case ito::tComplex64:
                //    {
                //        switch (m_cmplxState)
                //        {
                //            default:
                //            case cmplxAbs:
                //                return QPointF(fPos, abs(curPlane->at<ito::complex64>(m_plotPts[n].rangeY[0], m_plotPts[n].rangeX[0])));
                //            break;
                //            case cmplxReal:
                //                return QPointF(fPos, (curPlane->at<ito::complex64>(m_plotPts[n].rangeY[0], m_plotPts[n].rangeX[0]).real()));
                //            break;
                //            case cmplxImag:
                //                return QPointF(fPos, (curPlane->at<ito::complex64>(m_plotPts[n].rangeY[0], m_plotPts[n].rangeX[0]).imag()));
                //            break;
                //            case cmplxArg:
                //                return QPointF(fPos, arg(curPlane->at<ito::complex64>(m_plotPts[n].rangeY[0], m_plotPts[n].rangeX[0])));
                //            break;
                //        }
                //    }
                //    break;
                //    case ito::tComplex128:
                //    {
                //        switch (m_cmplxState)
                //        {
                //            default:
                //            case cmplxAbs:
                //                return QPointF(fPos, abs(curPlane->at<ito::complex128>(m_plotPts[n].rangeY[0], m_plotPts[n].rangeX[0])));
                //            break;
                //            case cmplxReal:
                //                return QPointF(fPos, (curPlane->at<ito::complex128>(m_plotPts[n].rangeY[0], m_plotPts[n].rangeX[0]).real()));
                //            break;
                //            case cmplxImag:
                //                return QPointF(fPos, (curPlane->at<ito::complex128>(m_plotPts[n].rangeY[0], m_plotPts[n].rangeX[0]).imag()));
                //            break;
                //            case cmplxArg:
                //                return QPointF(fPos, arg(curPlane->at<ito::complex128>(m_plotPts[n].rangeY[0], m_plotPts[n].rangeX[0])));
                //            break;
                //        }
                //    }
                //    break;
    //            }
    //        } // if m_fast
    //        else
    //        {
    //            double val;

    //            switch(m_pDataObj->getType())
    //            {
    //                case ito::tInt8:
    //                    val = curPlane->at<int8>(m_plotPts[n].rangeY[0], m_plotPts[n].rangeX[0]) * m_plotPts[n].weights[0] +
    //                        curPlane->at<int8>(m_plotPts[n].rangeY[1], m_plotPts[n].rangeX[0]) * m_plotPts[n].weights[1] +
    //                        curPlane->at<int8>(m_plotPts[n].rangeY[1], m_plotPts[n].rangeX[1]) * m_plotPts[n].weights[2] +
    //                        curPlane->at<int8>(m_plotPts[n].rangeY[0], m_plotPts[n].rangeX[1]) * m_plotPts[n].weights[3];
    //                break;
    //                case ito::tUInt8:
    //                    val = curPlane->at<uint8>(m_plotPts[n].rangeY[0], m_plotPts[n].rangeX[0]) * m_plotPts[n].weights[0] +
    //                        curPlane->at<uint8>(m_plotPts[n].rangeY[1], m_plotPts[n].rangeX[0]) * m_plotPts[n].weights[1] +
    //                        curPlane->at<uint8>(m_plotPts[n].rangeY[1], m_plotPts[n].rangeX[1]) * m_plotPts[n].weights[2] +
    //                        curPlane->at<uint8>(m_plotPts[n].rangeY[0], m_plotPts[n].rangeX[1]) * m_plotPts[n].weights[3];
    //                break;
    //                case ito::tInt16:
    //                    val = curPlane->at<int16>(m_plotPts[n].rangeY[0], m_plotPts[n].rangeX[0]) * m_plotPts[n].weights[0] +
    //                        curPlane->at<int16>(m_plotPts[n].rangeY[1], m_plotPts[n].rangeX[0]) * m_plotPts[n].weights[1] +
    //                        curPlane->at<int16>(m_plotPts[n].rangeY[1], m_plotPts[n].rangeX[1]) * m_plotPts[n].weights[2] +
    //                        curPlane->at<int16>(m_plotPts[n].rangeY[0], m_plotPts[n].rangeX[1]) * m_plotPts[n].weights[3];
    //                break;
    //                case ito::tUInt16:
    //                    val = curPlane->at<uint16>(m_plotPts[n].rangeY[0], m_plotPts[n].rangeX[0]) * m_plotPts[n].weights[0] +
    //                        curPlane->at<uint16>(m_plotPts[n].rangeY[1], m_plotPts[n].rangeX[0]) * m_plotPts[n].weights[1] +
    //                        curPlane->at<uint16>(m_plotPts[n].rangeY[1], m_plotPts[n].rangeX[1]) * m_plotPts[n].weights[2] +
    //                        curPlane->at<uint16>(m_plotPts[n].rangeY[0], m_plotPts[n].rangeX[1]) * m_plotPts[n].weights[3];
    //                break;
    //                case ito::tInt32:
    //                    val = curPlane->at<int32>(m_plotPts[n].rangeY[0], m_plotPts[n].rangeX[0]) * m_plotPts[n].weights[0] +
    //                        curPlane->at<int32>(m_plotPts[n].rangeY[1], m_plotPts[n].rangeX[0]) * m_plotPts[n].weights[1] +
    //                        curPlane->at<int32>(m_plotPts[n].rangeY[1], m_plotPts[n].rangeX[1]) * m_plotPts[n].weights[2] +
    //                        curPlane->at<int32>(m_plotPts[n].rangeY[0], m_plotPts[n].rangeX[1]) * m_plotPts[n].weights[3];
    //                break;
    //                case ito::tUInt32:
    //                    val = curPlane->at<uint32>(m_plotPts[n].rangeY[0], m_plotPts[n].rangeX[0]) * m_plotPts[n].weights[0] +
    //                        curPlane->at<uint32>(m_plotPts[n].rangeY[1], m_plotPts[n].rangeX[0]) * m_plotPts[n].weights[1] +
    //                        curPlane->at<uint32>(m_plotPts[n].rangeY[1], m_plotPts[n].rangeX[1]) * m_plotPts[n].weights[2] +
    //                        curPlane->at<uint32>(m_plotPts[n].rangeY[0], m_plotPts[n].rangeX[1]) * m_plotPts[n].weights[3];
    //                break;
    //                case ito::tFloat32:
    //                     val = curPlane->at<float>(m_plotPts[n].rangeY[0], m_plotPts[n].rangeX[0]) * m_plotPts[n].weights[0] +
    //                        curPlane->at<float>(m_plotPts[n].rangeY[1], m_plotPts[n].rangeX[0]) * m_plotPts[n].weights[1] +
    //                        curPlane->at<float>(m_plotPts[n].rangeY[1], m_plotPts[n].rangeX[1]) * m_plotPts[n].weights[2] +
    //                        curPlane->at<float>(m_plotPts[n].rangeY[0], m_plotPts[n].rangeX[1]) * m_plotPts[n].weights[3];
    //                break;
    //                case ito::tFloat64:
    //                     val = curPlane->at<double>(m_plotPts[n].rangeY[0], m_plotPts[n].rangeX[0]) * m_plotPts[n].weights[0] +
    //                        curPlane->at<double>(m_plotPts[n].rangeY[1], m_plotPts[n].rangeX[0]) * m_plotPts[n].weights[1] +
    //                        curPlane->at<double>(m_plotPts[n].rangeY[1], m_plotPts[n].rangeX[1]) * m_plotPts[n].weights[2] +
    //                        curPlane->at<double>(m_plotPts[n].rangeY[0], m_plotPts[n].rangeX[1]) * m_plotPts[n].weights[3];
    //                break;
                //    case ito::tComplex64:
                //    {
                //        switch (m_cmplxState)
                //        {
                //            default:
                //            case cmplxAbs:
                //                rVal = curPlane->at<ito::complex64>(m_plotPts[n].rangeY[0], m_plotPts[n].rangeX[0]).real() * m_plotPts[n].weights[0] +
                //                    curPlane->at<ito::complex64>(m_plotPts[n].rangeY[1], m_plotPts[n].rangeX[0]).real() * m_plotPts[n].weights[1] +
                //                    curPlane->at<ito::complex64>(m_plotPts[n].rangeY[1], m_plotPts[n].rangeX[1]).real() * m_plotPts[n].weights[2] +
                //                    curPlane->at<ito::complex64>(m_plotPts[n].rangeY[0], m_plotPts[n].rangeX[1]).real() * m_plotPts[n].weights[3];
                //                iVal = curPlane->at<ito::complex64>(m_plotPts[n].rangeY[0], m_plotPts[n].rangeX[0]).imag() * m_plotPts[n].weights[0] +
                //                    curPlane->at<ito::complex64>(m_plotPts[n].rangeY[1], m_plotPts[n].rangeX[0]).imag() * m_plotPts[n].weights[1] +
                //                    curPlane->at<ito::complex64>(m_plotPts[n].rangeY[1], m_plotPts[n].rangeX[1]).imag() * m_plotPts[n].weights[2] +
                //                    curPlane->at<ito::complex64>(m_plotPts[n].rangeY[0], m_plotPts[n].rangeX[1]).imag() * m_plotPts[n].weights[3];
                //                val = sqrt(rVal * rVal + iVal * iVal);
                //            break;
                //            case cmplxReal:
                //                val = curPlane->at<ito::complex64>(m_plotPts[n].rangeY[0], m_plotPts[n].rangeX[0]).real() * m_plotPts[n].weights[0] +
                //                    curPlane->at<ito::complex64>(m_plotPts[n].rangeY[1], m_plotPts[n].rangeX[0]).real() * m_plotPts[n].weights[1] +
                //                    curPlane->at<ito::complex64>(m_plotPts[n].rangeY[1], m_plotPts[n].rangeX[1]).real() * m_plotPts[n].weights[2] +
                //                    curPlane->at<ito::complex64>(m_plotPts[n].rangeY[0], m_plotPts[n].rangeX[1]).real() * m_plotPts[n].weights[3];
                //            break;
                //            case cmplxImag:
                //                val = curPlane->at<ito::complex64>(m_plotPts[n].rangeY[0], m_plotPts[n].rangeX[0]).imag() * m_plotPts[n].weights[0] +
                //                    curPlane->at<ito::complex64>(m_plotPts[n].rangeY[1], m_plotPts[n].rangeX[0]).imag() * m_plotPts[n].weights[1] +
                //                    curPlane->at<ito::complex64>(m_plotPts[n].rangeY[1], m_plotPts[n].rangeX[1]).imag() * m_plotPts[n].weights[2] +
                //                    curPlane->at<ito::complex64>(m_plotPts[n].rangeY[0], m_plotPts[n].rangeX[1]).imag() * m_plotPts[n].weights[3];
                //            break;
                //            case cmplxArg:
                //                rVal = curPlane->at<ito::complex64>(m_plotPts[n].rangeY[0], m_plotPts[n].rangeX[0]).real() * m_plotPts[n].weights[0] +
                //                    curPlane->at<ito::complex64>(m_plotPts[n].rangeY[1], m_plotPts[n].rangeX[0]).real() * m_plotPts[n].weights[1] +
                //                    curPlane->at<ito::complex64>(m_plotPts[n].rangeY[1], m_plotPts[n].rangeX[1]).real() * m_plotPts[n].weights[2] +
                //                    curPlane->at<ito::complex64>(m_plotPts[n].rangeY[0], m_plotPts[n].rangeX[1]).real() * m_plotPts[n].weights[3];
                //                iVal = curPlane->at<ito::complex64>(m_plotPts[n].rangeY[0], m_plotPts[n].rangeX[0]).imag() * m_plotPts[n].weights[0] +
                //                    curPlane->at<ito::complex64>(m_plotPts[n].rangeY[1], m_plotPts[n].rangeX[0]).imag() * m_plotPts[n].weights[1] +
                //                    curPlane->at<ito::complex64>(m_plotPts[n].rangeY[1], m_plotPts[n].rangeX[1]).imag() * m_plotPts[n].weights[2] +
                //                    curPlane->at<ito::complex64>(m_plotPts[n].rangeY[0], m_plotPts[n].rangeX[1]).imag() * m_plotPts[n].weights[3];
                //                val = atan2(iVal, rVal);
                //            break;
                //        }
                //    }
                //    break;
                //    case ito::tComplex128:
                //    {
                //        switch (m_cmplxState)
                //        {
                //            default:
                //            case cmplxAbs:
                //                rVal = curPlane->at<ito::complex128>(m_plotPts[n].rangeY[0], m_plotPts[n].rangeX[0]).real() * m_plotPts[n].weights[0] +
                //                    curPlane->at<ito::complex128>(m_plotPts[n].rangeY[1], m_plotPts[n].rangeX[0]).real() * m_plotPts[n].weights[1] +
                //                    curPlane->at<ito::complex128>(m_plotPts[n].rangeY[1], m_plotPts[n].rangeX[1]).real() * m_plotPts[n].weights[2] +
                //                    curPlane->at<ito::complex128>(m_plotPts[n].rangeY[0], m_plotPts[n].rangeX[1]).real() * m_plotPts[n].weights[3];
                //                iVal = curPlane->at<ito::complex128>(m_plotPts[n].rangeY[0], m_plotPts[n].rangeX[0]).imag() * m_plotPts[n].weights[0] +
                //                    curPlane->at<ito::complex128>(m_plotPts[n].rangeY[1], m_plotPts[n].rangeX[0]).imag() * m_plotPts[n].weights[1] +
                //                    curPlane->at<ito::complex128>(m_plotPts[n].rangeY[1], m_plotPts[n].rangeX[1]).imag() * m_plotPts[n].weights[2] +
                //                    curPlane->at<ito::complex128>(m_plotPts[n].rangeY[0], m_plotPts[n].rangeX[1]).imag() * m_plotPts[n].weights[3];
                //                val = sqrt(rVal * rVal + iVal * iVal);
                //            break;
                //            case cmplxReal:
                //                val = curPlane->at<ito::complex128>(m_plotPts[n].rangeY[0], m_plotPts[n].rangeX[0]).real() * m_plotPts[n].weights[0] +
                //                    curPlane->at<ito::complex128>(m_plotPts[n].rangeY[1], m_plotPts[n].rangeX[0]).real() * m_plotPts[n].weights[1] +
                //                    curPlane->at<ito::complex128>(m_plotPts[n].rangeY[1], m_plotPts[n].rangeX[1]).real() * m_plotPts[n].weights[2] +
                //                    curPlane->at<ito::complex128>(m_plotPts[n].rangeY[0], m_plotPts[n].rangeX[1]).real() * m_plotPts[n].weights[3];
                //            break;
                //            case cmplxImag:
                //                val = curPlane->at<ito::complex128>(m_plotPts[n].rangeY[0], m_plotPts[n].rangeX[0]).imag() * m_plotPts[n].weights[0] +
                //                    curPlane->at<ito::complex128>(m_plotPts[n].rangeY[1], m_plotPts[n].rangeX[0]).imag() * m_plotPts[n].weights[1] +
                //                    curPlane->at<ito::complex128>(m_plotPts[n].rangeY[1], m_plotPts[n].rangeX[1]).imag() * m_plotPts[n].weights[2] +
                //                    curPlane->at<ito::complex128>(m_plotPts[n].rangeY[0], m_plotPts[n].rangeX[1]).imag() * m_plotPts[n].weights[3];
                //            break;
                //            case cmplxArg:
                //                rVal = curPlane->at<ito::complex128>(m_plotPts[n].rangeY[0], m_plotPts[n].rangeX[0]).real() * m_plotPts[n].weights[0] +
                //                    curPlane->at<ito::complex128>(m_plotPts[n].rangeY[1], m_plotPts[n].rangeX[0]).real() * m_plotPts[n].weights[1] +
                //                    curPlane->at<ito::complex128>(m_plotPts[n].rangeY[1], m_plotPts[n].rangeX[1]).real() * m_plotPts[n].weights[2] +
                //                    curPlane->at<ito::complex128>(m_plotPts[n].rangeY[0], m_plotPts[n].rangeX[1]).real() * m_plotPts[n].weights[3];
                //                iVal = curPlane->at<ito::complex128>(m_plotPts[n].rangeY[0], m_plotPts[n].rangeX[0]).imag() * m_plotPts[n].weights[0] +
                //                    curPlane->at<ito::complex128>(m_plotPts[n].rangeY[1], m_plotPts[n].rangeX[0]).imag() * m_plotPts[n].weights[1] +
                //                    curPlane->at<ito::complex128>(m_plotPts[n].rangeY[1], m_plotPts[n].rangeX[1]).imag() * m_plotPts[n].weights[2] +
                //                    curPlane->at<ito::complex128>(m_plotPts[n].rangeY[0], m_plotPts[n].rangeX[1]).imag() * m_plotPts[n].weights[3];
                //                val = atan2(iVal, rVal);
                //            break;
                //        }
                //    }
                //    break;
    //            }
    //            return QPointF(fPos, val);
    //        }
    //    }
    //}
    //return QPointF();
}

//----------------------------------------------------------------------------------------------------------------------------------
void DataObjectSeriesData::setIntervalRange(Qt::Axis axis, bool autoCalcLimits, double minValue, double maxValue)
{
    if (axis == Qt::YAxis)
    {
        if (autoCalcLimits)
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
    if (axis == Qt::XAxis)
    {
        if (autoCalcLimits)
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
/*
This helper method returns the min/max values for an fixed-point typed dataObject and the line, which is represented
by this dataObjectSeriesData.
*/
template<typename _Tp> void findMinMaxNonWeightedInteger(const ito::DataObject *obj, const DataObjectSeriesData::LineData &d, double &min, double &max, int &minIdx, int &maxIdx)
{
    const cv::Mat *mat;
    uchar *ptr;
    min = std::numeric_limits<float64>::max();
    max = -min;
    _Tp val;

    switch(d.dir)
    {
    case DataObjectSeriesData::dirX:
    case DataObjectSeriesData::dirY:
        mat = obj->getCvPlaneMat(d.plane);
        ptr = (mat->data + d.matOffset);
        for (int i = 0 ; i < d.nrPoints ; i++)
        {
            val = *(reinterpret_cast<_Tp*>(ptr));
            ptr += d.matStepSize;

            if (val > max) { max = val; maxIdx = i; }
            if (val < min) { min = val; minIdx = i; }
        }
        break;

    case DataObjectSeriesData::dirXY:
        mat = obj->getCvPlaneMat(d.plane);
        ptr = (mat->data + d.matOffset);
        for (int i = 0 ; i < d.nrPoints ; i++)
        {
            val = *(reinterpret_cast<_Tp*>(ptr + d.matSteps[i]));

            if (val > max) { max = val; maxIdx = i; }
            if (val < min) { min = val; minIdx = i; }
        }
        break;
    case DataObjectSeriesData::dirZ:

        for (int i = 0 ; i < d.nrPoints ; i++)
        {
            mat = obj->getCvPlaneMat(i);
            ptr = (mat->data + d.matOffset);
            val = *(reinterpret_cast<_Tp*>(ptr));

            if (val > max) { max = val; maxIdx = i; }
            if (val < min) { min = val; minIdx = i; }
        }
        break;
    }

    if (max < min)
    {
        // no points found.
        min = -1.0;
        max = 1.0;
    }
}

//----------------------------------------------------------------------------------------------------------------------------------
/*
This helper method returns the min/max values for an floating-point typed dataObject and the line, which is represented
by this dataObjectSeriesData.
*/
template<typename _Tp> void findMinMaxNonWeightedFloat(const ito::DataObject *obj, const DataObjectSeriesData::LineData &d, double &min, double &max, int &minIdx, int &maxIdx)
{
    const cv::Mat *mat;
    uchar *ptr;
    min = std::numeric_limits<_Tp>::max();
    max = -min;
    float32 val;

    switch(d.dir)
    {
    case DataObjectSeriesData::dirX:
    case DataObjectSeriesData::dirY:
        mat = obj->getCvPlaneMat(d.plane);
        ptr = (mat->data + d.matOffset);
        for (int i = 0 ; i < d.nrPoints ; i++)
        {
            val = *(reinterpret_cast<_Tp*>(ptr));
            ptr += d.matStepSize;

            if (!qIsFinite(val))
            {
                continue;
            }

            if (val > max) { max = val; maxIdx = i; }
            if (val < min) { min = val; minIdx = i; }
        }
        break;

    case DataObjectSeriesData::dirXY:
        mat = obj->getCvPlaneMat(d.plane);
        ptr = (mat->data + d.matOffset);
        for (int i = 0 ; i < d.nrPoints ; i++)
        {
            val = *(reinterpret_cast<_Tp*>(ptr + d.matSteps[i]));

            if (!qIsFinite(val))
            {
                continue;
            }

            if (val > max) { max = val; maxIdx = i; }
            if (val < min) { min = val; minIdx = i; }
        }
        break;
    case DataObjectSeriesData::dirZ:

        for (int i = 0 ; i < d.nrPoints ; i++)
        {
            mat = obj->getCvPlaneMat(i);
            ptr = (mat->data + d.matOffset);
            val = *(reinterpret_cast<_Tp*>(ptr));

            if (!qIsFinite(val))
            {
                continue;
            }

            if (val > max) { max = val; maxIdx = i; }
            if (val < min) { min = val; minIdx = i; }
        }
        break;
    }

    if (max < min)
    {
        // no points found.
        min = -1.0;
        max = 1.0;
    }
}

//----------------------------------------------------------------------------------------------------------------------------------
/*
This helper method returns the min/max values for a complex typed dataObject and the line, which is represented
by this dataObjectSeriesData.

The complex selector can be defined separately
*/
template<typename _Tp, typename _Tp2> void findMinMaxNonWeightedComplex(const ito::DataObject *obj, const DataObjectSeriesData::LineData &d, double &min, double &max, int &minIdx, int &maxIdx, ItomQwtPlotEnums::ComplexType cmplxState = ItomQwtPlotEnums::CmplxAbs)
{
    const cv::Mat *mat;
    uchar *ptr;
    min = std::numeric_limits<_Tp>::max();
    max = -min;
    ito::float32 val;
    ito::complex64 val_;

    switch(d.dir)
    {
    case DataObjectSeriesData::dirX:
    case DataObjectSeriesData::dirY:
        mat = obj->getCvPlaneMat(d.plane);
        if (!mat->data)
            return;
        ptr = (mat->data + d.matOffset);
        for (int i = 0 ; i < d.nrPoints ; i++)
        {
            val_ = *(reinterpret_cast<_Tp2*>(ptr));
            switch(cmplxState)
            {
            case ItomQwtPlotEnums::CmplxAbs:
                val = abs(val_);
                break;
            case ItomQwtPlotEnums::CmplxReal:
                val = val_.real();
                break;
            case ItomQwtPlotEnums::CmplxImag:
                val = val_.imag();
                break;
            case ItomQwtPlotEnums::CmplxArg:
                val = arg(val_);
                break;
            }

            ptr += d.matStepSize;

            if (!qIsFinite(val))
            {
                continue;
            }

            if (val > max) { max = val; maxIdx = i; }
            if (val < min) { min = val; minIdx = i; }
        }
        break;

    case DataObjectSeriesData::dirXY:
        mat = obj->getCvPlaneMat(d.plane);
        ptr = (mat->data + d.matOffset);
        for (int i = 0 ; i < d.nrPoints ; i++)
        {
            val_ = *(reinterpret_cast<_Tp2*>(ptr + d.matSteps[i]));
            switch(cmplxState)
            {
            case ItomQwtPlotEnums::CmplxAbs:
                val = abs(val_);
                break;
            case ItomQwtPlotEnums::CmplxReal:
                val = val_.real();
                break;
            case ItomQwtPlotEnums::CmplxImag:
                val = val_.imag();
                break;
            case ItomQwtPlotEnums::CmplxArg:
                val = arg(val_);
                break;
            }

            if (!qIsFinite(val))
            {
                continue;
            }

            if (val > max) { max = val; maxIdx = i; }
            if (val < min) { min = val; minIdx = i; }
        }
        break;

    case DataObjectSeriesData::dirZ:

        for (int i = 0 ; i < d.nrPoints ; i++)
        {
            mat = obj->getCvPlaneMat(i);
            ptr = (mat->data + d.matOffset);
            val_ = *(reinterpret_cast<_Tp2*>(ptr));
            switch(cmplxState)
            {
            case ItomQwtPlotEnums::CmplxAbs:
                val = abs(val_);
                break;
            case ItomQwtPlotEnums::CmplxReal:
                val = val_.real();
                break;
            case ItomQwtPlotEnums::CmplxImag:
                val = val_.imag();
                break;
            case ItomQwtPlotEnums::CmplxArg:
                val = arg(val_);
                break;
            }

            if (!qIsFinite(val))
            {
                continue;
            }

            if (val > max) { max = val; maxIdx = i; }
            if (val < min) { min = val; minIdx = i; }
        }
        break;
    }

    if (max < min)
    {
        // no points found.
        min = -1.0;
        max = 1.0;
    }
}


//----------------------------------------------------------------------------------------------------------------------------------
template<typename _Tp> bool findMinMaxNonWeightedIntegerCropped(const ito::DataObject *obj, const DataObjectSeriesData::LineData &d, const QwtInterval &xInterval, const QwtInterval &yInterval, double &min, double &max, int &minIdx, int &maxIdx)
{
    const cv::Mat *mat;
    uchar *ptr;
    min = std::numeric_limits<float64>::max();
    max = -min;
    _Tp val;

    int startIdx = qBound(0, qRound((xInterval.minValue() - d.startPhys) / d.stepSizePhys), d.nrPoints - 1);
    int endIdx = qBound(0, qRound((xInterval.maxValue() - d.startPhys) / d.stepSizePhys), d.nrPoints - 1);

    if (startIdx > endIdx)
    {
        std::swap(startIdx, endIdx);
    }

    double yAxisRange[] = { yInterval.minValue(), yInterval.maxValue() };

    switch (d.dir)
    {
    case DataObjectSeriesData::dirX:
    case DataObjectSeriesData::dirY:
        mat = obj->getCvPlaneMat(d.plane);
        ptr = (mat->data + d.matOffset + startIdx * d.matStepSize);
        for (int i = startIdx; i <= endIdx; ++i)
        {
            val = *(reinterpret_cast<_Tp*>(ptr));
            ptr += d.matStepSize;

            if (val > max && val <= yAxisRange[1]) { max = val; maxIdx = i; }
            if (val < min && val >= yAxisRange[0]) { min = val; minIdx = i; }
        }
        break;

    case DataObjectSeriesData::dirXY:
        mat = obj->getCvPlaneMat(d.plane);
        ptr = (mat->data + d.matOffset + startIdx * d.matStepSize);
        for (int i = startIdx; i <= endIdx; ++i)
        {
            val = *(reinterpret_cast<_Tp*>(ptr + d.matSteps[i]));

            if (val > max && val <= yAxisRange[1]) { max = val; maxIdx = i; }
            if (val < min && val >= yAxisRange[0]) { min = val; minIdx = i; }
        }
        break;
    case DataObjectSeriesData::dirZ:

        for (int i = startIdx; i <= endIdx; ++i)
        {
            mat = obj->getCvPlaneMat(i);
            ptr = (mat->data + d.matOffset);
            val = *(reinterpret_cast<_Tp*>(ptr));

            if (val > max && val <= yAxisRange[1]) { max = val; maxIdx = i; }
            if (val < min && val >= yAxisRange[0]) { min = val; minIdx = i; }
        }
        break;
    }

    return endIdx >= startIdx;
}

//----------------------------------------------------------------------------------------------------------------------------------
template<typename _Tp> bool findMinMaxNonWeightedFloatCropped(const ito::DataObject *obj, const DataObjectSeriesData::LineData &d, const QwtInterval &xInterval, const QwtInterval &yInterval, double &min, double &max, int &minIdx, int &maxIdx)
{
    const cv::Mat *mat;
    uchar *ptr;
    min = std::numeric_limits<_Tp>::max();
    max = -min;
    float32 val;

    int startIdx = qBound(0, qRound((xInterval.minValue() - d.startPhys) / d.stepSizePhys), d.nrPoints - 1);
    int endIdx = qBound(0, qRound((xInterval.maxValue() - d.startPhys) / d.stepSizePhys), d.nrPoints - 1);

    if (startIdx > endIdx)
    {
        std::swap(startIdx, endIdx);
    }
    double yAxisRange[] = { yInterval.minValue(), yInterval.maxValue() };

    switch (d.dir)
    {
    case DataObjectSeriesData::dirX:
    case DataObjectSeriesData::dirY:
        mat = obj->getCvPlaneMat(d.plane);
        ptr = (mat->data + d.matOffset + startIdx * d.matStepSize);
        for (int i = startIdx; i <= endIdx; ++i)
        {
            val = *(reinterpret_cast<_Tp*>(ptr));
            ptr += d.matStepSize;

            if (!qIsFinite(val))
            {
                continue;
            }

            if (val > max && val <= yAxisRange[1]) { max = val; maxIdx = i; }
            if (val < min && val >= yAxisRange[0]) { min = val; minIdx = i; }
        }
        break;

    case DataObjectSeriesData::dirXY:
        mat = obj->getCvPlaneMat(d.plane);
        ptr = (mat->data + d.matOffset + startIdx * d.matStepSize);
        for (int i = startIdx; i <= endIdx; ++i)
        {
            val = *(reinterpret_cast<_Tp*>(ptr + d.matSteps[i]));

            if (!qIsFinite(val))
            {
                continue;
            }

            if (val > max && val <= yAxisRange[1]) { max = val; maxIdx = i; }
            if (val < min && val >= yAxisRange[0]) { min = val; minIdx = i; }
        }
        break;
    case DataObjectSeriesData::dirZ:

        for (int i = startIdx; i <= endIdx; ++i)
        {
            mat = obj->getCvPlaneMat(i);
            ptr = (mat->data + d.matOffset);
            val = *(reinterpret_cast<_Tp*>(ptr));

            if (!qIsFinite(val))
            {
                continue;
            }

            if (val > max && val <= yAxisRange[1]) { max = val; maxIdx = i; }
            if (val < min && val >= yAxisRange[0]) { min = val; minIdx = i; }
        }
        break;
    }

    return endIdx >= startIdx;
}

//----------------------------------------------------------------------------------------------------------------------------------
template<typename _Tp, typename _Tp2> bool findMinMaxNonWeightedComplexCropped(const ito::DataObject *obj, const DataObjectSeriesData::LineData &d, const QwtInterval &xInterval, const QwtInterval &yInterval, double &min, double &max, int &minIdx, int &maxIdx, ItomQwtPlotEnums::ComplexType cmplxState = ItomQwtPlotEnums::CmplxAbs)
{
    const cv::Mat *mat;
    uchar *ptr;
    min = std::numeric_limits<_Tp>::max();
    max = -min;
    ito::float32 val;
    ito::complex64 val_;

    int startIdx = qBound(0, qRound((xInterval.minValue() - d.startPhys) / d.stepSizePhys), d.nrPoints - 1);
    int endIdx = qBound(0, qRound((xInterval.maxValue() - d.startPhys) / d.stepSizePhys), d.nrPoints - 1);

    if (startIdx > endIdx)
    {
        std::swap(startIdx, endIdx);
    }

    double yAxisRange[] = { yInterval.minValue(), yInterval.maxValue() };

    switch (d.dir)
    {
    case DataObjectSeriesData::dirX:
    case DataObjectSeriesData::dirY:
        mat = obj->getCvPlaneMat(d.plane);
        if (!mat->data)
            return false;
        ptr = (mat->data + d.matOffset + startIdx * d.matStepSize);
        for (int i = startIdx; i <= endIdx; ++i)
        {
            val_ = *(reinterpret_cast<_Tp2*>(ptr));
            switch (cmplxState)
            {
            case ItomQwtPlotEnums::CmplxAbs:
                val = abs(val_);
                break;
            case ItomQwtPlotEnums::CmplxReal:
                val = val_.real();
                break;
            case ItomQwtPlotEnums::CmplxImag:
                val = val_.imag();
                break;
            case ItomQwtPlotEnums::CmplxArg:
                val = arg(val_);
                break;
            }

            ptr += d.matStepSize;

            if (!qIsFinite(val))
            {
                continue;
            }

            if (val > max && val <= yAxisRange[1]) { max = val; maxIdx = i; }
            if (val < min && val >= yAxisRange[0]) { min = val; minIdx = i; }
        }
        break;

    case DataObjectSeriesData::dirXY:
        mat = obj->getCvPlaneMat(d.plane);
        ptr = (mat->data + d.matOffset + startIdx * d.matStepSize);
        for (int i = startIdx; i <= endIdx; ++i)
        {
            val_ = *(reinterpret_cast<_Tp2*>(ptr + d.matSteps[i]));
            switch (cmplxState)
            {
            case ItomQwtPlotEnums::CmplxAbs:
                val = abs(val_);
                break;
            case ItomQwtPlotEnums::CmplxReal:
                val = val_.real();
                break;
            case ItomQwtPlotEnums::CmplxImag:
                val = val_.imag();
                break;
            case ItomQwtPlotEnums::CmplxArg:
                val = arg(val_);
                break;
            }

            if (!qIsFinite(val))
            {
                continue;
            }

            if (val > max && val <= yAxisRange[1]) { max = val; maxIdx = i; }
            if (val < min && val >= yAxisRange[0]) { min = val; minIdx = i; }
        }
        break;

    case DataObjectSeriesData::dirZ:

        for (int i = startIdx; i <= endIdx; ++i)
        {
            mat = obj->getCvPlaneMat(i);
            ptr = (mat->data + d.matOffset);
            val_ = *(reinterpret_cast<_Tp2*>(ptr));

            switch (cmplxState)
            {
            case ItomQwtPlotEnums::CmplxAbs:
                val = abs(val_);
                break;
            case ItomQwtPlotEnums::CmplxReal:
                val = val_.real();
                break;
            case ItomQwtPlotEnums::CmplxImag:
                val = val_.imag();
                break;
            case ItomQwtPlotEnums::CmplxArg:
                val = arg(val_);
                break;
            }

            if (!qIsFinite(val))
            {
                continue;
            }

            if (val > max && val <= yAxisRange[1]) { max = val; maxIdx = i; }
            if (val < min && val >= yAxisRange[0]) { min = val; minIdx = i; }
        }
        break;
    }

    return endIdx >= startIdx;
}


//----------------------------------------------------------------------------------------------------------------------------------
void findMinMaxNonWeightedRGBA(const ito::DataObject *obj, const DataObjectSeriesData::LineData &d, double &min, double &max, int &minIdx, int &maxIdx, DataObjectSeriesData::ColorType type)
{
    const cv::Mat *mat;
    uchar *ptr;
    min = std::numeric_limits<ito::uint8>::max();
    max = -min;
    ito::uint8 val;

    switch (d.dir)
    {
    case DataObjectSeriesData::dirX:
    case DataObjectSeriesData::dirY:
        mat = obj->getCvPlaneMat(d.plane);
        ptr = (mat->data + d.matOffset);
        if(type == DataObjectSeriesData::grayColor)
        {
            float valf;
            for (int i = 0; i < d.nrPoints; i++)
            {
                valf = (*(reinterpret_cast<ito::Rgba32*>(ptr))).gray();
                ptr += d.matStepSize;
                if (!qIsFinite(valf))
                {
                    continue;
                }
                if (valf > max) { max = valf; maxIdx = i; }
                if (valf < min) { min = valf; minIdx = i; }
            }
        }
        else if (type == DataObjectSeriesData::blueColor)
        {
            for (int i = 0; i < d.nrPoints; i++)
            {
                val = (*(reinterpret_cast<ito::Rgba32*>(ptr))).b;
                ptr += d.matStepSize;

                if (val > max) { max = val; maxIdx = i; }
                if (val < min) { min = val; minIdx = i; }
            }
        }
        else if (type == DataObjectSeriesData::greenColor)
        {
            for (int i = 0; i < d.nrPoints; i++)
            {
                val = (*(reinterpret_cast<ito::Rgba32*>(ptr))).g;
                ptr += d.matStepSize;

                if (val > max) { max = val; maxIdx = i; }
                if (val < min) { min = val; minIdx = i; }
            }
        }
        else if (type == DataObjectSeriesData::redColor)
        {
            for (int i = 0; i < d.nrPoints; i++)
            {
                val = (*(reinterpret_cast<ito::Rgba32*>(ptr))).r;
                ptr += d.matStepSize;

                if (val > max) { max = val; maxIdx = i; }
                if (val < min) { min = val; minIdx = i; }
            }
        }
        else if (type == DataObjectSeriesData::alphaColor)
        {
            for (int i = 0; i < d.nrPoints; i++)
            {
                val = (*(reinterpret_cast<ito::Rgba32*>(ptr))).a;
                ptr += d.matStepSize;

                if (val > max) { max = val; maxIdx = i; }
                if (val < min) { min = val; minIdx = i; }
            }
        }
        break;

    case DataObjectSeriesData::dirXY:

        mat = obj->getCvPlaneMat(d.plane);
        ptr = (mat->data + d.matOffset);
        if (type == DataObjectSeriesData::grayColor)
        {
            float valf;
            for (int i = 0; i < d.nrPoints; i++)
            {
                valf = (*(reinterpret_cast<ito::Rgba32*>(ptr+d.matSteps[i]))).gray();
                ptr += d.matStepSize;
                if (!qIsFinite(valf))
                {
                    continue;
                }
                if (valf > max) { max = valf; maxIdx = i; }
                if (valf < min) { min = valf; minIdx = i; }
            }
        }
        else if (type == DataObjectSeriesData::blueColor)
        {
            for (int i = 0; i < d.nrPoints; i++)
            {
                val = (*(reinterpret_cast<ito::Rgba32*>(ptr + d.matSteps[i]))).b;
                ptr += d.matStepSize;

                if (val > max) { max = val; maxIdx = i; }
                if (val < min) { min = val; minIdx = i; }
            }
        }
        else if (type == DataObjectSeriesData::greenColor)
        {
            for (int i = 0; i < d.nrPoints; i++)
            {
                val = (*(reinterpret_cast<ito::Rgba32*>(ptr + d.matSteps[i]))).g;
                ptr += d.matStepSize;

                if (val > max) { max = val; maxIdx = i; }
                if (val < min) { min = val; minIdx = i; }
            }
        }
        else if (type == DataObjectSeriesData::redColor)
        {
            for (int i = 0; i < d.nrPoints; i++)
            {
                val = (*(reinterpret_cast<ito::Rgba32*>(ptr + d.matSteps[i]))).r;
                ptr += d.matStepSize;

                if (val > max) { max = val; maxIdx = i; }
                if (val < min) { min = val; minIdx = i; }
            }
        }
        else if (type == DataObjectSeriesData::alphaColor)
        {
            for (int i = 0; i < d.nrPoints; i++)
            {
                val = (*(reinterpret_cast<ito::Rgba32*>(ptr + d.matSteps[i]))).a;
                ptr += d.matStepSize;

                if (val > max) { max = val; maxIdx = i; }
                if (val < min) { min = val; minIdx = i; }
            }
        }
        break;
    case DataObjectSeriesData::dirZ:
        if (type == DataObjectSeriesData::grayColor)
        {
            float valf;
            for (int i = 0; i < d.nrPoints; i++)
            {
                mat = obj->getCvPlaneMat(i);
                ptr = (mat->data + d.matOffset);
                valf = (*(reinterpret_cast<ito::Rgba32*>(ptr))).gray();

                if (!qIsFinite(valf))
                {
                    continue;
                }

                if (valf > max) { max = valf; maxIdx = i; }
                if (valf < min) { min = valf; minIdx = i; }
            }
        }
        else if (type == DataObjectSeriesData::redColor)
        {
            for (int i = 0; i < d.nrPoints; i++)
            {
                mat = obj->getCvPlaneMat(i);
                ptr = (mat->data + d.matOffset);
                val = (*(reinterpret_cast<ito::Rgba32*>(ptr))).r;

                if (val > max) { max = val; maxIdx = i; }
                if (val < min) { min = val; minIdx = i; }
            }
        }
        else if (type == DataObjectSeriesData::greenColor)
        {
            for (int i = 0; i < d.nrPoints; i++)
            {
                mat = obj->getCvPlaneMat(i);
                ptr = (mat->data + d.matOffset);
                val = (*(reinterpret_cast<ito::Rgba32*>(ptr))).g;

                if (val > max) { max = val; maxIdx = i; }
                if (val < min) { min = val; minIdx = i; }
            }
        }
        else if (type == DataObjectSeriesData::blueColor)
        {
            for (int i = 0; i < d.nrPoints; i++)
            {
                mat = obj->getCvPlaneMat(i);
                ptr = (mat->data + d.matOffset);
                val = (*(reinterpret_cast<ito::Rgba32*>(ptr))).b;

                if (val > max) { max = val; maxIdx = i; }
                if (val < min) { min = val; minIdx = i; }
            }
        }
        else if (type == DataObjectSeriesData::alphaColor)
        {
            for (int i = 0; i < d.nrPoints; i++)
            {
                mat = obj->getCvPlaneMat(i);
                ptr = (mat->data + d.matOffset);
                val = (*(reinterpret_cast<ito::Rgba32*>(ptr))).a;

                if (val > max) { max = val; maxIdx = i; }
                if (val < min) { min = val; minIdx = i; }
            }
        }
        break;
    }
}

//-----------------------------------------------------------------------------------

QRectF DataObjectSeriesData::boundingRect() const
{
    QRectF res;

    if (m_pDataObj && m_d.valid)
    {
        double min = 0.0, max = 0.0;
        int minIdx = 0.0, maxIdx = 0.0;
        switch(m_pDataObj->getType())
        {
            case ito::tInt8:
                findMinMaxNonWeightedInteger<ito::int8>(m_pDataObj, m_d, min, max, minIdx, maxIdx);
            break;
            case ito::tUInt8:
                findMinMaxNonWeightedInteger<ito::uint8>(m_pDataObj, m_d, min, max, minIdx, maxIdx);
            break;
            case ito::tInt16:
                findMinMaxNonWeightedInteger<ito::int16>(m_pDataObj, m_d, min, max, minIdx, maxIdx);
            break;
            case ito::tUInt16:
                findMinMaxNonWeightedInteger<ito::uint16>(m_pDataObj, m_d, min, max, minIdx, maxIdx);
            break;
            case ito::tInt32:
                findMinMaxNonWeightedInteger<ito::int32>(m_pDataObj, m_d, min, max, minIdx, maxIdx);
            break;
            case ito::tUInt32:
                findMinMaxNonWeightedInteger<ito::uint32>(m_pDataObj, m_d, min, max, minIdx, maxIdx);
            break;
            case ito::tFloat32:
                findMinMaxNonWeightedFloat<ito::float32>(m_pDataObj, m_d, min, max, minIdx, maxIdx);
            break;
            case ito::tFloat64:
                findMinMaxNonWeightedFloat<ito::float64>(m_pDataObj, m_d, min, max, minIdx, maxIdx);
            break;
            case ito::tComplex64:
                findMinMaxNonWeightedComplex<ito::float32, ito::complex64>(m_pDataObj, m_d, min, max, minIdx, maxIdx, m_cmplxState);
            break;
            case ito::tComplex128:
                findMinMaxNonWeightedComplex<ito::float64, ito::complex128>(m_pDataObj, m_d, min, max, minIdx, maxIdx, m_cmplxState);
            break;
            case ito::tRGBA32:
                findMinMaxNonWeightedRGBA(m_pDataObj, m_d, min, max, minIdx, maxIdx, m_colorState);
            break;
        }

        if ( (max-min) < std::numeric_limits<double>::epsilon() )
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

        if (m_d.points.size() == 0) //no weighted stuff
        {
            float minX = m_d.startPhys;
            float width = m_d.stepSizePhys * (m_d.nrPoints - 1);

            if (std::abs(width) < std::numeric_limits<float>::epsilon())
            {
                minX -= 0.001f;
                width = 0.002f;
            }

            res = QRectF(minX, min, width, max - min);
        }
        else
        {
            qDebug() << "not yet implemented";
            //not implemented yet
        }
    }

    return res;
}

//----------------------------------------------------------------------------------------------------------------------
RetVal DataObjectSeriesData::getMinMaxLoc(double &min, double &max, int &minSampleIdx, int &maxSampleIdx) const
{
    QRectF res;

    minSampleIdx = 0;
    maxSampleIdx = 0;
    RetVal retval;

    if (m_pDataObj && m_d.valid)
    {
        switch(m_pDataObj->getType())
        {
            case ito::tInt8:
                findMinMaxNonWeightedInteger<ito::int8>(m_pDataObj, m_d, min, max, minSampleIdx, maxSampleIdx);
            break;
            case ito::tUInt8:
                findMinMaxNonWeightedInteger<ito::uint8>(m_pDataObj, m_d, min, max, minSampleIdx, maxSampleIdx);
            break;
            case ito::tInt16:
                findMinMaxNonWeightedInteger<ito::int16>(m_pDataObj, m_d, min, max, minSampleIdx, maxSampleIdx);
            break;
            case ito::tUInt16:
                findMinMaxNonWeightedInteger<ito::uint16>(m_pDataObj, m_d, min, max, minSampleIdx, maxSampleIdx);
            break;
            case ito::tInt32:
                findMinMaxNonWeightedInteger<ito::int32>(m_pDataObj, m_d, min, max, minSampleIdx, maxSampleIdx);
            break;
            case ito::tUInt32:
                findMinMaxNonWeightedInteger<ito::uint32>(m_pDataObj, m_d, min, max, minSampleIdx, maxSampleIdx);
            break;
            case ito::tFloat32:
                findMinMaxNonWeightedFloat<ito::float32>(m_pDataObj, m_d, min, max, minSampleIdx, maxSampleIdx);
            break;
            case ito::tFloat64:
                findMinMaxNonWeightedFloat<ito::float64>(m_pDataObj, m_d, min, max, minSampleIdx, maxSampleIdx);
            break;
            case ito::tComplex64:
                findMinMaxNonWeightedComplex<ito::float32, ito::complex64>(m_pDataObj, m_d, min, max, minSampleIdx, maxSampleIdx, m_cmplxState);
            break;
            case ito::tComplex128:
                findMinMaxNonWeightedComplex<ito::float64, ito::complex128>(m_pDataObj, m_d, min, max, minSampleIdx, maxSampleIdx, m_cmplxState);
            break;
            case ito::tRGBA32:
                min = 0.0;
                max = 255.0;
            break;
        }

        if ( max-min < std::numeric_limits<double>::epsilon() )
        {
            min *= 0.99;
            max *= 1.01;
        }
    }
    else
    {
        retval += RetVal(retError, 0, "no dataObject");
    }

    return retval;
}

//----------------------------------------------------------------------------------------------------------------------
RetVal DataObjectSeriesData::getMinMaxLocCropped(const QwtInterval &xInterval, const QwtInterval &yInterval, double &min, double &max, int &minSampleIdx, int &maxSampleIdx) const
{
    QRectF res;

    minSampleIdx = 0;
    maxSampleIdx = 0;
    RetVal retval;
    bool result;

    if (m_pDataObj && m_d.valid)
    {
        switch (m_pDataObj->getType())
        {
        case ito::tInt8:
            result = findMinMaxNonWeightedIntegerCropped<ito::int8>(m_pDataObj, m_d, xInterval, yInterval, min, max, minSampleIdx, maxSampleIdx);
            break;
        case ito::tUInt8:
            result = findMinMaxNonWeightedIntegerCropped<ito::uint8>(m_pDataObj, m_d, xInterval, yInterval, min, max, minSampleIdx, maxSampleIdx);
            break;
        case ito::tInt16:
            result = findMinMaxNonWeightedIntegerCropped<ito::int16>(m_pDataObj, m_d, xInterval, yInterval, min, max, minSampleIdx, maxSampleIdx);
            break;
        case ito::tUInt16:
            result = findMinMaxNonWeightedIntegerCropped<ito::uint16>(m_pDataObj, m_d, xInterval, yInterval, min, max, minSampleIdx, maxSampleIdx);
            break;
        case ito::tInt32:
            result = findMinMaxNonWeightedIntegerCropped<ito::int32>(m_pDataObj, m_d, xInterval, yInterval, min, max, minSampleIdx, maxSampleIdx);
            break;
        case ito::tUInt32:
            result = findMinMaxNonWeightedIntegerCropped<ito::uint32>(m_pDataObj, m_d, xInterval, yInterval, min, max, minSampleIdx, maxSampleIdx);
            break;
        case ito::tFloat32:
            result = findMinMaxNonWeightedFloatCropped<ito::float32>(m_pDataObj, m_d, xInterval, yInterval, min, max, minSampleIdx, maxSampleIdx);
            break;
        case ito::tFloat64:
            result = findMinMaxNonWeightedFloatCropped<ito::float64>(m_pDataObj, m_d, xInterval, yInterval, min, max, minSampleIdx, maxSampleIdx);
            break;
        case ito::tComplex64:
            result = findMinMaxNonWeightedComplexCropped<ito::float32, ito::complex64>(m_pDataObj, m_d, xInterval, yInterval, min, max, minSampleIdx, maxSampleIdx, m_cmplxState);
            break;
        case ito::tComplex128:
            result = findMinMaxNonWeightedComplexCropped<ito::float64, ito::complex128>(m_pDataObj, m_d, xInterval, yInterval, min, max, minSampleIdx, maxSampleIdx, m_cmplxState);
            break;
        case ito::tRGBA32:
            min = 0.0;
            max = 255.0;
            break;
        }

        if (!result)
        {
            retval += ito::RetVal(retWarning, 0, "dataObject has no valid points within the given axis range.");
        }
        else if (max - min < std::numeric_limits<double>::epsilon())
        {
            min *= 0.99;
            max *= 1.01;
        }
    }
    else
    {
        retval += RetVal(retError, 0, "no dataObject");
    }

    return retval;
}

//----------------------------------------------------------------------------------------------
QString DataObjectSeriesData::getDObjValueLabel(const AbstractFigure::UnitLabelStyle &unitLabelStyle) const
{
    if (m_dObjValueUnit != "")
    {
        if (m_dObjValueDescription != "")
        {
            switch (unitLabelStyle)
            {
                case AbstractFigure::UnitLabelSlash:
                    return QString("%1 / %2").arg(m_dObjValueDescription, m_dObjValueUnit);
                case AbstractFigure::UnitLabelKeywordIn:
                    return QString("%1 in %2").arg(m_dObjValueDescription, m_dObjValueUnit);
                case AbstractFigure::UnitLabelSquareBrackets:
                    return QString("%1 [%2]").arg(m_dObjValueDescription, m_dObjValueUnit);
            }
        }
        else
        {
            switch (unitLabelStyle)
            {
                case AbstractFigure::UnitLabelSlash:
                    return QString("%1").arg(m_dObjValueUnit); //is this right?
                case AbstractFigure::UnitLabelKeywordIn:
                    return QString("%1").arg(m_dObjValueUnit); //is this right?
                case AbstractFigure::UnitLabelSquareBrackets:
                    return QString("[%1]").arg(m_dObjValueUnit);
            }
        }
    }
    return m_dObjValueDescription;
}

//----------------------------------------------------------------------------------------------
QString DataObjectSeriesData::getDObjAxisLabel(const AbstractFigure::UnitLabelStyle &unitLabelStyle)  const
{
    if (m_dObjAxisUnit != "")
    {
        if (m_dObjAxisDescription != "")
        {
            switch (unitLabelStyle)
            {
                case AbstractFigure::UnitLabelSlash:
                    return QString("%1 / %2").arg(m_dObjAxisDescription, m_dObjAxisUnit);
                case AbstractFigure::UnitLabelKeywordIn:
                    return QString("%1 in %2").arg(m_dObjAxisDescription, m_dObjAxisUnit);
                case AbstractFigure::UnitLabelSquareBrackets:
                    return QString("%1 [%2]").arg(m_dObjAxisDescription, m_dObjAxisUnit);
            }
        }
        else
        {
            switch (unitLabelStyle)
            {
                case AbstractFigure::UnitLabelSlash:
                    return QString("%1").arg(m_dObjAxisUnit); //is this right?
                case AbstractFigure::UnitLabelKeywordIn:
                    return QString("%1").arg(m_dObjAxisUnit); //is this right?
                case AbstractFigure::UnitLabelSquareBrackets:
                    return QString("[%1]").arg(m_dObjAxisUnit);
            }
        }
    }
    return m_dObjAxisDescription;
}

//-------------------------------------------------------------------------
template <typename _Tp> _Tp DataObjectSeriesData::sampleComplex(const size_t& n) const
{
    const cv::Mat *mat;
    const uchar* ptr[4];
    float fPos;

    if (m_pDataObj && m_d.valid)
    {
        if (m_d.points.size() == 0) //no weighted stuff
        {
            switch (m_d.dir)
            {
            case dirX:
            case dirY:
                mat = m_pDataObj->getCvPlaneMat(m_d.plane);
                ptr[0] = (mat->data + m_d.matOffset + m_d.matStepSize * n);
                fPos = m_d.startPhys + m_d.stepSizePhys * n;
                break;

            case dirZ:
                mat = m_pDataObj->getCvPlaneMat((int)n);
                ptr[0] = (mat->data + m_d.matOffset);
                fPos = m_d.startPhys + m_d.stepSizePhys * n;
                break;

            case dirXY:
                mat = m_pDataObj->getCvPlaneMat(m_d.plane);
                ptr[0] = (mat->data + m_d.matOffset + m_d.matSteps[(int)n]);
                fPos = m_d.startPhys + m_d.stepSizePhys * n;
                break;
            }
        }
    }

    return *(reinterpret_cast<const _Tp*>(ptr[0]));
}
template ito::complex64  DataObjectSeriesData::sampleComplex(const size_t& n) const;
template ito::complex128  DataObjectSeriesData::sampleComplex(const size_t& n) const;

//----------------------------------------------------------------------------------------------
void DataObjectSeriesData::getDObjValueDescriptionAndUnit(std::string &description, std::string &unit) const
{
    description = m_dObjValueDescription.toLatin1().toStdString();
    unit = m_dObjValueUnit.toLatin1().toStdString();
}

//----------------------------------------------------------------------------------------------
void DataObjectSeriesData::getDObjAxisDescriptionAndUnit(std::string &description, std::string &unit) const
{
    description = m_dObjAxisDescription.toLatin1().toStdString();
    unit = m_dObjAxisUnit.toLatin1().toStdString();
}
