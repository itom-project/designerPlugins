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
#include <qcryptographichash.h>

#include <qdebug.h>
#include <qnumeric.h>

//----------------------------------------------------------------------------------------------------------------------------------
DataObjectSeriesData::DataObjectSeriesData(const int fastmode) :
    m_fast(fastmode),
    m_autoScaleY(true),
    m_minY(-1.0),
    m_maxY(2.0),
    m_autoScaleX(true),
    m_minX(-1.0),
    m_maxX(1.0),
    m_cmplxState(DataObjectSeriesData::cmplxAbs),
    m_pDataObj(NULL),
    inSamplingMode(false) 
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
    if(m_pDataObj)
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
int DataObjectSeriesData::getPosToPix(const double phys)
{
    if(m_d.valid && m_d.nrPoints > 0)
    {
        double dist = (phys - m_d.startPhys);
        return qRound(dist / m_d.stepSizePhys);
    }
    else
    {
        return 0;
    }
/*
    int value = 0;
    if (m_Scaling > 0) value = (int)((phys - m_startPos) / m_Scaling + 0.5);
    else value = (int)((phys - m_startPos  - m_physLength) / abs(m_Scaling) + 0.5);

    value = value > (m_plotPts.size()-1) ? (m_plotPts.size()-1) : value < 0 ? 0 : value;

    return value;*/
}

//----------------------------------------------------------------------------------------------------------------------------------
void DataObjectSeriesData::calcHash()
{
    if(m_pDataObj == NULL)
    {
        m_hash = QCryptographicHash::hash("", QCryptographicHash::Md5);
    }
    else
    {
        QByteArray ba;

        int dims = m_pDataObj->getDims();
        ba.append( QByteArray().setNum( dims ) );

        if( dims > 0 )
        {
            cv::Mat *m = (cv::Mat*)m_pDataObj->get_mdata()[0];
            uchar* d = m->data;
            ba.append( QByteArray( (const char*)&d, (sizeof(int)/sizeof(char)))); //address to data of first plane

            ba.append( QByteArray().setNum( m->size[0] ));
        }

        ba.append( QByteArray().setNum( (uint)(m_d.nrPoints) ));
        ba.append( QByteArray().setNum( m_d.startPx.x() ));
        ba.append( QByteArray().setNum( m_d.startPx.y() ));
        m_hash = QCryptographicHash::hash(ba, QCryptographicHash::Md5);
    }
}

//----------------------------------------------------------------------------------------------------------------------------------
RetVal DataObjectSeriesData::updateDataObject(const ito::DataObject* dataObj, QVector<QPointF> bounds)
{
    RetVal retval;
    bool _unused;
    QRectF p;
    float right;
    cv::Mat *mat;
    int pxX1, pxX2, pxY1, pxY2;
    std::string description, unit;

    if(dataObj == NULL)
    {
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
        for(int i = 0; i < dims-2; i++)
        {
            if(dataObj->getSize(i) != 1)
            {
                break;
            }
            prependedOneDims++;
        }

        
        switch( bounds.size() )
        {
        case 2: //dirX, dirY or dirXY

            if( (dims-prependedOneDims) != 2)
            {
                m_d.valid = false;
                retval += RetVal(retError,0,"line plot requires a 2-dim dataObject or the first (n-2) dimensions must have a size of 1");
            }
            else
            {
                m_d.valid = true;
                pxX1 = qRound(dataObj->getPhysToPix(dims-1, bounds[0].x(), _unused));
                pxY1 = qRound(dataObj->getPhysToPix(dims-2, bounds[0].y(), _unused));
                pxX2 = qRound(dataObj->getPhysToPix(dims-1, bounds[1].x(), _unused));
                pxY2 = qRound(dataObj->getPhysToPix(dims-2, bounds[1].y(), _unused));

                saturation( pxX1, 0, dataObj->getSize(dims-1)-1 );
                saturation( pxX2, 0, dataObj->getSize(dims-1)-1 );
                saturation( pxY1, 0, dataObj->getSize(dims-2)-1 );
                saturation( pxY2, 0, dataObj->getSize(dims-2)-1 );

                mat = (cv::Mat*)dataObj->get_mdata()[ dataObj->seekMat(0) ]; //first plane in ROI

                if( pxX1 == pxX2 ) //pure line in y-direction
                {
                    m_d.dir = dirY;
                    if(pxY2 >= pxY1)
                    {
                        m_d.nrPoints = 1 + pxY2 - pxY1;
                        m_d.startPhys= dataObj->getPixToPhys(dims-2, pxY1, _unused); //bounds[0].y() ;
                        right = dataObj->getPixToPhys(dims-2, pxY2, _unused); //bounds[1].y();
                        m_d.stepSizePhys = m_d.nrPoints > 1 ? (right - m_d.startPhys) / (float)(m_d.nrPoints-1) : 0.0;
                        
                        m_d.startPx.setX(pxX1);
                        m_d.startPx.setY(pxY1);
                        m_d.stepSizePx.setWidth(0);
                        m_d.stepSizePx.setHeight(1);

                        m_d.matOffset = mat->step[0] * pxY1 + mat->step[1] * pxX1; //(&mat->at<char>(pxY1,pxX1) - &mat->at<char>(0,0));
                        m_d.matStepSize= mat->step[0] ; //step in y-direction (in bytes)
                    }
                    else
                    {
                        m_d.nrPoints = 1 + pxY1 - pxY2;
                        m_d.startPhys = dataObj->getPixToPhys(dims-2, pxY2, _unused); //bounds[1].y();
                        right = dataObj->getPixToPhys(dims-2, pxY1, _unused); //bounds[0].y();
                        m_d.stepSizePhys = m_d.nrPoints > 1 ? (right - m_d.startPhys) / (float)(m_d.nrPoints-1) : 0.0;

                        m_d.startPx.setX(pxX1);
                        m_d.startPx.setY(pxY2);
                        m_d.stepSizePx.setWidth(0);
                        m_d.stepSizePx.setHeight(1);

                        m_d.matOffset = mat->step[0] * pxY2 + mat->step[1] * pxX1; //(&mat->at<char>(pxY2,pxX1) - &mat->at<char>(0,0));
                        m_d.matStepSize= mat->step[0] ; //step in y-direction (in bytes)
                    }

                    description = dataObj->getAxisDescription(dims-2,_unused);
                    unit = dataObj->getAxisUnit(dims-2,_unused);
                    if(description == "") description = "y-axis";
                    if(unit == "")
                    {
                        m_dObjAxisLabel = QString::fromStdString(description);
                    }
                    else
                    {
                        m_dObjAxisLabel = QString("%1 [%2]").arg( QString::fromStdString(description) ).arg( QString::fromStdString(unit) );
                    }
                   
                    description = dataObj->getValueDescription();
                    unit = dataObj->getValueUnit();
                    if(unit == "")
                    {
                        m_dObjValueLabel = QString::fromStdString(description);
                    }
                    else
                    {
                        m_dObjValueLabel = QString("%1 [%2]").arg( QString::fromStdString(description) ).arg( QString::fromStdString(unit) );
                    }
                    
                }
                else if( pxY1 == pxY2 ) //pure line in x-direction
                {
                    m_d.dir = dirX;
                    if(pxX2 >= pxX1)
                    {
                        m_d.nrPoints = 1 + pxX2 - pxX1;
                        m_d.startPhys= dataObj->getPixToPhys(dims-1, pxX1, _unused); //bounds[0].y() ;
                        right = dataObj->getPixToPhys(dims-1, pxX2, _unused); //bounds[1].y();
                        m_d.stepSizePhys = m_d.nrPoints > 1 ? (right - m_d.startPhys) / (float)(m_d.nrPoints-1) : 0.0;
                        
                        m_d.startPx.setX(pxX1);
                        m_d.startPx.setY(pxY1);
                        m_d.stepSizePx.setWidth(1);
                        m_d.stepSizePx.setHeight(0);

                        m_d.matOffset = mat->step[0] * pxY1 + mat->step[1] * pxX1; //(&mat->at<char>(pxY1,pxX1) - &mat->at<char>(0,0));
                        m_d.matStepSize= mat->step[1] ; //step in x-direction (in bytes)
                    }
                    else
                    {
                        m_d.nrPoints = 1 + pxX1 - pxX2;
                        m_d.startPhys = dataObj->getPixToPhys(dims-1, pxX2, _unused); //bounds[1].y();
                        right = dataObj->getPixToPhys(dims-1, pxX1, _unused); //bounds[0].y();
                        m_d.stepSizePhys = m_d.nrPoints > 1 ? (right - m_d.startPhys) / (float)(m_d.nrPoints-1) : 0.0;

                        m_d.startPx.setX(pxX1);
                        m_d.startPx.setY(pxY2);
                        m_d.stepSizePx.setWidth(1);
                        m_d.stepSizePx.setHeight(0);

                        m_d.matOffset = mat->step[0] * pxY1 + mat->step[1] * pxX2; //(&mat->at<char>(pxY1,pxX2) - &mat->at<char>(0,0));
                        m_d.matStepSize= mat->step[1] ; //step in x-direction (in bytes)
                    }

                    description = dataObj->getAxisDescription(dims-1,_unused);
                    unit = dataObj->getAxisUnit(dims-1,_unused);
                    if(description == "") description = "x-axis";
                    if(unit == "")
                    {
                        m_dObjAxisLabel = QString::fromStdString(description);
                    }
                    else
                    {
                        m_dObjAxisLabel = QString("%1 [%2]").arg( QString::fromStdString(description) ).arg( QString::fromStdString(unit) );
                    }
                   
                    description = dataObj->getValueDescription();
                    unit = dataObj->getValueUnit();
                    if(unit == "")
                    {
                        m_dObjValueLabel = QString::fromStdString(description);
                    }
                    else
                    {
                        m_dObjValueLabel = QString("%1 [%2]").arg( QString::fromStdString(description) ).arg( QString::fromStdString(unit) );
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

                        m_d.startPhys= 0.0;  //there is no physical starting point for diagonal lines.

                        if(m_d.nrPoints > 0)
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

                        m_d.matOffset = mat->step[0] * pxY1 + mat->step[1] * pxX1; //(&mat->at<char>(pxY1,pxX1) - &mat->at<char>(0,0));
                        m_d.matStepSize= 0 ; 

                        int pdx, pdy, ddx, ddy, es, el;
                        if(dx>dy)
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

                        m_d.matSteps.resize(m_d.nrPoints);

                        for(unsigned int n = 0; n < m_d.nrPoints; n++)
                        {  /* loop */
                            //setPixel(x,y)
                            m_d.matSteps[n] = mat->step[0] * y + mat->step[1] * x;

                            err -= es;
                            if(err < 0)
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
                        //double dx = (bounds[1].x() - bounds[0].x());
                        //double dy = (bounds[1].y() - bounds[0].y());
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

                        //if(xdirect && ydirect)
                        //{
                        //    m_startPos = 0.0;
                        //    m_physLength = sqrt((double)(xscale* xscale* dx * dx + yscale * yscale * dy * dy));
                        //}
                        //else if(ydirect)
                        //{
                        //    m_startPos = bounds[0].y();
                        //    m_physLength = yscale* dy ;      
                        //}
                        //else
                        //{
                        //    m_startPos = bounds[0].x();
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
                        //        if(xpos >= sizex)
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
                        //        if(ypos >= sizey)
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
                    if(unit == "") unit = "px";

                    std::string descr2 = dataObj->getAxisDescription(dims-1, _unused);
                    std::string unit2 = dataObj->getAxisUnit(dims-1, _unused);
                    if(unit2 == "") unit2 = "px";

                    if(description == "" && descr2 == "")
                    {
                        if(unit == "" && unit2 == "")
                        {
                            m_dObjAxisLabel = "x/y-axis";
                        }
                        else
                        {
                            m_dObjAxisLabel = QString("x/y-axis [%1/%2]").arg( QString::fromStdString(unit), QString::fromStdString(unit2) );
                        }
                    }
                    else
                    {
                        if(unit == "" && unit2 == "")
                        {
                            m_dObjAxisLabel = QString("%1/%2").arg( QString::fromStdString(description), QString::fromStdString(descr2) );
                        }
                        else
                        {
                            m_dObjAxisLabel = QString("%1/%2 [%3/%4]").arg( QString::fromStdString(description), QString::fromStdString(descr2), QString::fromStdString(unit), QString::fromStdString(unit2) );
                        }
                    }
                   
                    description = dataObj->getValueDescription();
                    unit = dataObj->getValueUnit();
                    if(unit == "")
                    {
                        m_dObjValueLabel = QString::fromStdString(description);
                    }
                    else
                    {
                        m_dObjValueLabel = QString("%1 [%2]").arg( QString::fromStdString(description) ).arg( QString::fromStdString(unit) );
                    }
                }
            }

            break;
        case 1:
            if((dims - prependedOneDims) != 3)
            {
                retval += RetVal(retError,0,"line plot in z-direction requires a 3-dim dataObject");
                return retval;
            }
            else
            {
                m_d.valid = true;
                pxX1 = dataObj->getPhysToPix(dims - 1, bounds[0].x(), _unused);
                pxY1 = dataObj->getPhysToPix(dims - 2, bounds[0].y(), _unused);

                saturation( pxX1, 0, dataObj->getSize(dims - 1) - 1 );
                saturation( pxX2, 0, dataObj->getSize(dims - 1) - 1 );

                m_d.dir = dirZ;
                m_d.nrPoints = dataObj->getSize(dims - 3);
                m_d.startPhys = dataObj->getPixToPhys(dims - 3,0,_unused);
                if(m_d.nrPoints > 1)
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

                mat = (cv::Mat*)dataObj->get_mdata()[ dataObj->seekMat(0) ]; //first plane in ROI
                m_d.matOffset = mat->step[0] * pxY1 + mat->step[1] * pxX1; //(&mat->at<char>(pxY1,pxX1) - &mat->at<char>(0,0));
                m_d.matStepSize= 0 ; //step in x-direction (in bytes)

                description = dataObj->getAxisDescription(dims - 3, _unused);
                unit = dataObj->getAxisUnit(dims - 3, _unused);
                if(description == "") description = "z-axis";
                if(unit == "")
                {
                    m_dObjAxisLabel = QString::fromStdString(description);
                }
                else
                {
                    m_dObjAxisLabel = QString("%1 [%2]").arg( QString::fromStdString(description) ).arg( QString::fromStdString(unit) );
                }
                   
                description = dataObj->getValueDescription();
                unit = dataObj->getValueUnit();
                if(unit == "")
                {
                    m_dObjValueLabel = QString::fromStdString(description);
                }
                else
                {
                    m_dObjValueLabel = QString("%1 [%2]").arg( QString::fromStdString(description) ).arg( QString::fromStdString(unit) );
                }
            }

            break;
        default:
            retval += RetVal(retError,0,"bounds vector must have 1 or 2 entries");
            break;
        }    
    }
    m_pDataObj = dataObj;

    calcHash();

    return retval;

    //if((bounds.size() == 1) && (dataObjectDims > 2))
    //{
    //    m_zDirect = true;
    //}
    //else 
    //{
    //    m_zDirect = false;
    //}

    //pts[0].setX(static_cast<float>(dataObj->getPhysToPix(dataObjectDims-1,bounds[0].x(),testValid)));
    //pts[0].setY(static_cast<float>(dataObj->getPhysToPix(dataObjectDims-2,bounds[0].y(),testValid)));

    //if(m_zDirect)
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
    //    for(unsigned int n = 0; n < m_size; n++)
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
    //    if(bounds.size() == 1)
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
    //    if(dataObjectDims > 1)
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

        //    for(unsigned int n = 0; n < m_size; n++)
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

        //    if(xdirect && ydirect)
        //    {
        //        m_startPos = 0.0;
        //        m_physLength = sqrt((double)(xscale* xscale* dx * dx + yscale * yscale * dy * dy));
        //    }
        //    else if(ydirect)
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
        //            if(xpos >= sizex)
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
        //            if(ypos >= sizey)
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
QPointF DataObjectSeriesData::sample(size_t n) const
{
    cv::Mat *mat;
    const uchar* ptr[4];
    //float weights[4];
    float fPos;

    if(m_pDataObj && m_d.valid)
    {
        if(m_d.points.size() == 0) //no weighted stuff
        {
            switch (m_d.dir)
            {
                case dirX:
                case dirY:
                    mat = (cv::Mat*)(m_pDataObj->get_mdata()[ m_pDataObj->seekMat(0) ]);
                    ptr[0] = (mat->data + m_d.matOffset + m_d.matStepSize * n);
                    fPos = m_d.startPhys + m_d.stepSizePhys * n;
                break;

                case dirZ:
                    mat = (cv::Mat*)(m_pDataObj->get_mdata()[ m_pDataObj->seekMat(n) ]);
                    ptr[0] = (mat->data + m_d.matOffset);
                    fPos = m_d.startPhys + m_d.stepSizePhys * n;
                break;

                case dirXY:
                    mat = (cv::Mat*)(m_pDataObj->get_mdata()[ m_pDataObj->seekMat(0) ]);
                    ptr[0] = (mat->data + m_d.matOffset + m_d.matSteps[n]);
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
                    case cmplxAbs:
                        return QPointF(fPos, abs(val));
                    break;
                    case cmplxReal:
                        return QPointF(fPos, val.real());
                    break;
                    case cmplxImag:
                        return QPointF(fPos, val.imag());
                    break;
                    case cmplxArg:
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
                        case cmplxAbs:
			                return QPointF(fPos, abs(val));
		                break;
		                case cmplxReal:
			                return QPointF(fPos, val.real());
		                break;
		                case cmplxImag:
			                return QPointF(fPos, val.imag());
		                break;
		                case cmplxArg:
			                return QPointF(fPos, arg(val));
		                break;
	                }
                }
                break;
                case ito::tRGBA32:
                    return QPointF(fPos, (reinterpret_cast<const ito::Rgba32*>(ptr[0]))->gray() );
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

    //if(m_pDataObj)
    //{
    //    if(m_pDataObj->getDims() == 2)
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
				//	    switch (m_cmplxState)
				//	    {
				//		    default:
    //                        case cmplxAbs:
				//			    return QPointF(fPos, abs(m_pDataObj->at<ito::complex64>(m_plotPts[n].rangeY[0], m_plotPts[n].rangeX[0])));
				//		    break;
				//		    case cmplxReal:
				//			    return QPointF(fPos, (m_pDataObj->at<ito::complex64>(m_plotPts[n].rangeY[0], m_plotPts[n].rangeX[0]).real()));
				//		    break;
				//		    case cmplxImag:
				//			    return QPointF(fPos, (m_pDataObj->at<ito::complex64>(m_plotPts[n].rangeY[0], m_plotPts[n].rangeX[0]).imag()));
				//		    break;
				//		    case cmplxArg:
				//			    return QPointF(fPos, arg(m_pDataObj->at<ito::complex64>(m_plotPts[n].rangeY[0], m_plotPts[n].rangeX[0])));
				//		    break;
				//	    }
				//    }
    //                break;
    //                case ito::tComplex128:
				//    {
				//	    switch (m_cmplxState)
				//	    {
				//		    default:
				//		    case cmplxAbs:
				//			    return QPointF(fPos, abs(m_pDataObj->at<ito::complex128>(m_plotPts[n].rangeY[0], m_plotPts[n].rangeX[0])));
				//		    break;
				//		    case cmplxReal:
				//			    return QPointF(fPos, (m_pDataObj->at<ito::complex128>(m_plotPts[n].rangeY[0], m_plotPts[n].rangeX[0]).real()));
				//		    break;
				//		    case cmplxImag:
				//			    return QPointF(fPos, (m_pDataObj->at<ito::complex128>(m_plotPts[n].rangeY[0], m_plotPts[n].rangeX[0]).imag()));
				//		    break;
				//		    case cmplxArg:
				//			    return QPointF(fPos, arg(m_pDataObj->at<ito::complex128>(m_plotPts[n].rangeY[0], m_plotPts[n].rangeX[0])));
				//		    break;
				//	    }
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
				//	    switch (m_cmplxState)
				//	    {
				//		    default:
				//		    case cmplxAbs:
				//			    rVal = m_pDataObj->at<ito::complex64>(m_plotPts[n].rangeY[0], m_plotPts[n].rangeX[0]).real() * m_plotPts[n].weights[0] +
				//				    m_pDataObj->at<ito::complex64>(m_plotPts[n].rangeY[1], m_plotPts[n].rangeX[0]).real() * m_plotPts[n].weights[1] +
				//				    m_pDataObj->at<ito::complex64>(m_plotPts[n].rangeY[1], m_plotPts[n].rangeX[1]).real() * m_plotPts[n].weights[2] +
				//				    m_pDataObj->at<ito::complex64>(m_plotPts[n].rangeY[0], m_plotPts[n].rangeX[1]).real() * m_plotPts[n].weights[3];
				//			    iVal = m_pDataObj->at<ito::complex64>(m_plotPts[n].rangeY[0], m_plotPts[n].rangeX[0]).imag() * m_plotPts[n].weights[0] +
				//				    m_pDataObj->at<ito::complex64>(m_plotPts[n].rangeY[1], m_plotPts[n].rangeX[0]).imag() * m_plotPts[n].weights[1] +
				//				    m_pDataObj->at<ito::complex64>(m_plotPts[n].rangeY[1], m_plotPts[n].rangeX[1]).imag() * m_plotPts[n].weights[2] +
				//				    m_pDataObj->at<ito::complex64>(m_plotPts[n].rangeY[0], m_plotPts[n].rangeX[1]).imag() * m_plotPts[n].weights[3];
				//			    val = sqrt(rVal * rVal + iVal * iVal);
				//		    break;
				//		    case cmplxReal:
				//			    val = m_pDataObj->at<ito::complex64>(m_plotPts[n].rangeY[0], m_plotPts[n].rangeX[0]).real() * m_plotPts[n].weights[0] +
				//				    m_pDataObj->at<ito::complex64>(m_plotPts[n].rangeY[1], m_plotPts[n].rangeX[0]).real() * m_plotPts[n].weights[1] +
				//				    m_pDataObj->at<ito::complex64>(m_plotPts[n].rangeY[1], m_plotPts[n].rangeX[1]).real() * m_plotPts[n].weights[2] +
				//				    m_pDataObj->at<ito::complex64>(m_plotPts[n].rangeY[0], m_plotPts[n].rangeX[1]).real() * m_plotPts[n].weights[3];
				//		    break;
				//		    case cmplxImag:
				//			    val = m_pDataObj->at<ito::complex64>(m_plotPts[n].rangeY[0], m_plotPts[n].rangeX[0]).imag() * m_plotPts[n].weights[0] +
				//				    m_pDataObj->at<ito::complex64>(m_plotPts[n].rangeY[1], m_plotPts[n].rangeX[0]).imag() * m_plotPts[n].weights[1] +
				//				    m_pDataObj->at<ito::complex64>(m_plotPts[n].rangeY[1], m_plotPts[n].rangeX[1]).imag() * m_plotPts[n].weights[2] +
				//				    m_pDataObj->at<ito::complex64>(m_plotPts[n].rangeY[0], m_plotPts[n].rangeX[1]).imag() * m_plotPts[n].weights[3];
				//		    break;
				//		    case cmplxArg:
				//			    rVal = m_pDataObj->at<ito::complex64>(m_plotPts[n].rangeY[0], m_plotPts[n].rangeX[0]).real() * m_plotPts[n].weights[0] +
				//				    m_pDataObj->at<ito::complex64>(m_plotPts[n].rangeY[1], m_plotPts[n].rangeX[0]).real() * m_plotPts[n].weights[1] +
				//				    m_pDataObj->at<ito::complex64>(m_plotPts[n].rangeY[1], m_plotPts[n].rangeX[1]).real() * m_plotPts[n].weights[2] +
				//				    m_pDataObj->at<ito::complex64>(m_plotPts[n].rangeY[0], m_plotPts[n].rangeX[1]).real() * m_plotPts[n].weights[3];
				//			    iVal = m_pDataObj->at<ito::complex64>(m_plotPts[n].rangeY[0], m_plotPts[n].rangeX[0]).imag() * m_plotPts[n].weights[0] +
				//				    m_pDataObj->at<ito::complex64>(m_plotPts[n].rangeY[1], m_plotPts[n].rangeX[0]).imag() * m_plotPts[n].weights[1] +
				//				    m_pDataObj->at<ito::complex64>(m_plotPts[n].rangeY[1], m_plotPts[n].rangeX[1]).imag() * m_plotPts[n].weights[2] +
				//				    m_pDataObj->at<ito::complex64>(m_plotPts[n].rangeY[0], m_plotPts[n].rangeX[1]).imag() * m_plotPts[n].weights[3];
				//			    val = atan2(iVal, rVal);
				//		    break;
				//	    }
				//    }
    //                break;
    //                case ito::tComplex128:
				//    {
				//	    switch (m_cmplxState)
				//	    {
				//		    default:
				//		    case cmplxAbs:
				//			    rVal = m_pDataObj->at<ito::complex128>(m_plotPts[n].rangeY[0], m_plotPts[n].rangeX[0]).real() * m_plotPts[n].weights[0] +
				//				    m_pDataObj->at<ito::complex128>(m_plotPts[n].rangeY[1], m_plotPts[n].rangeX[0]).real() * m_plotPts[n].weights[1] +
				//				    m_pDataObj->at<ito::complex128>(m_plotPts[n].rangeY[1], m_plotPts[n].rangeX[1]).real() * m_plotPts[n].weights[2] +
				//				    m_pDataObj->at<ito::complex128>(m_plotPts[n].rangeY[0], m_plotPts[n].rangeX[1]).real() * m_plotPts[n].weights[3];
				//			    iVal = m_pDataObj->at<ito::complex128>(m_plotPts[n].rangeY[0], m_plotPts[n].rangeX[0]).imag() * m_plotPts[n].weights[0] +
				//				    m_pDataObj->at<ito::complex128>(m_plotPts[n].rangeY[1], m_plotPts[n].rangeX[0]).imag() * m_plotPts[n].weights[1] +
				//				    m_pDataObj->at<ito::complex128>(m_plotPts[n].rangeY[1], m_plotPts[n].rangeX[1]).imag() * m_plotPts[n].weights[2] +
				//				    m_pDataObj->at<ito::complex128>(m_plotPts[n].rangeY[0], m_plotPts[n].rangeX[1]).imag() * m_plotPts[n].weights[3];
				//			    val = sqrt(rVal * rVal + iVal * iVal);
				//		    break;
				//		    case cmplxReal:
				//			    val = m_pDataObj->at<ito::complex128>(m_plotPts[n].rangeY[0], m_plotPts[n].rangeX[0]).real() * m_plotPts[n].weights[0] +
				//				    m_pDataObj->at<ito::complex128>(m_plotPts[n].rangeY[1], m_plotPts[n].rangeX[0]).real() * m_plotPts[n].weights[1] +
				//				    m_pDataObj->at<ito::complex128>(m_plotPts[n].rangeY[1], m_plotPts[n].rangeX[1]).real() * m_plotPts[n].weights[2] +
				//				    m_pDataObj->at<ito::complex128>(m_plotPts[n].rangeY[0], m_plotPts[n].rangeX[1]).real() * m_plotPts[n].weights[3];
				//		    break;
				//		    case cmplxImag:
				//			    val = m_pDataObj->at<ito::complex128>(m_plotPts[n].rangeY[0], m_plotPts[n].rangeX[0]).imag() * m_plotPts[n].weights[0] +
				//				    m_pDataObj->at<ito::complex128>(m_plotPts[n].rangeY[1], m_plotPts[n].rangeX[0]).imag() * m_plotPts[n].weights[1] +
				//				    m_pDataObj->at<ito::complex128>(m_plotPts[n].rangeY[1], m_plotPts[n].rangeX[1]).imag() * m_plotPts[n].weights[2] +
				//				    m_pDataObj->at<ito::complex128>(m_plotPts[n].rangeY[0], m_plotPts[n].rangeX[1]).imag() * m_plotPts[n].weights[3];
				//		    break;
    //                        case cmplxArg:
				//			    rVal = m_pDataObj->at<ito::complex128>(m_plotPts[n].rangeY[0], m_plotPts[n].rangeX[0]).real() * m_plotPts[n].weights[0] +
				//				    m_pDataObj->at<ito::complex128>(m_plotPts[n].rangeY[1], m_plotPts[n].rangeX[0]).real() * m_plotPts[n].weights[1] +
				//				    m_pDataObj->at<ito::complex128>(m_plotPts[n].rangeY[1], m_plotPts[n].rangeX[1]).real() * m_plotPts[n].weights[2] +
				//				    m_pDataObj->at<ito::complex128>(m_plotPts[n].rangeY[0], m_plotPts[n].rangeX[1]).real() * m_plotPts[n].weights[3];
				//			    iVal = m_pDataObj->at<ito::complex128>(m_plotPts[n].rangeY[0], m_plotPts[n].rangeX[0]).imag() * m_plotPts[n].weights[0] +
				//				    m_pDataObj->at<ito::complex128>(m_plotPts[n].rangeY[1], m_plotPts[n].rangeX[0]).imag() * m_plotPts[n].weights[1] +
				//				    m_pDataObj->at<ito::complex128>(m_plotPts[n].rangeY[1], m_plotPts[n].rangeX[1]).imag() * m_plotPts[n].weights[2] +
				//				    m_pDataObj->at<ito::complex128>(m_plotPts[n].rangeY[0], m_plotPts[n].rangeX[1]).imag() * m_plotPts[n].weights[3];
				//			    val = atan2(iVal, rVal);
				//		    break;
				//	    }
				//    }
    //                break;
    //            }
    //            return QPointF(fPos, val);
    //        }
    //    }
    //    else if(m_zDirect)
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
				//	switch (m_cmplxState)
				//	{
				//		default:
				//		case cmplxAbs:
				//			return QPointF(fPos, abs(curPlane->at<ito::complex64>(m_plotPts[n].rangeY[0], m_plotPts[n].rangeX[0])));
				//		break;
				//		case cmplxReal:
				//			return QPointF(fPos, (curPlane->at<ito::complex64>(m_plotPts[n].rangeY[0], m_plotPts[n].rangeX[0]).real()));
				//		break;
				//		case cmplxImag:
				//			return QPointF(fPos, (curPlane->at<ito::complex64>(m_plotPts[n].rangeY[0], m_plotPts[n].rangeX[0]).imag()));
				//		break;
				//		case cmplxArg:
				//			return QPointF(fPos, arg(curPlane->at<ito::complex64>(m_plotPts[n].rangeY[0], m_plotPts[n].rangeX[0])));
				//		break;
				//	}
				//}
    //            break;
    //            case ito::tComplex128:
				//{
				//	switch (m_cmplxState)
				//	{
				//		default:
				//		case cmplxAbs:
				//			return QPointF(fPos, abs(curPlane->at<ito::complex128>(m_plotPts[n].rangeY[0], m_plotPts[n].rangeX[0])));
				//		break;
				//		case cmplxReal:
				//			return QPointF(fPos, (curPlane->at<ito::complex128>(m_plotPts[n].rangeY[0], m_plotPts[n].rangeX[0]).real()));
				//		break;
				//		case cmplxImag:
				//			return QPointF(fPos, (curPlane->at<ito::complex128>(m_plotPts[n].rangeY[0], m_plotPts[n].rangeX[0]).imag()));
				//		break;
				//		case cmplxArg:
				//			return QPointF(fPos, arg(curPlane->at<ito::complex128>(m_plotPts[n].rangeY[0], m_plotPts[n].rangeX[0])));
				//		break;
				//	}
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
				//	case ito::tComplex64:
				//	{
				//		switch (m_cmplxState)
				//		{
				//			default:
				//			case cmplxAbs:
				//				return QPointF(fPos, abs(curPlane->at<ito::complex64>(m_plotPts[n].rangeY[0], m_plotPts[n].rangeX[0])));
				//			break;
				//			case cmplxReal:
				//				return QPointF(fPos, (curPlane->at<ito::complex64>(m_plotPts[n].rangeY[0], m_plotPts[n].rangeX[0]).real()));
				//			break;
				//			case cmplxImag:
				//				return QPointF(fPos, (curPlane->at<ito::complex64>(m_plotPts[n].rangeY[0], m_plotPts[n].rangeX[0]).imag()));
				//			break;
				//			case cmplxArg:
				//				return QPointF(fPos, arg(curPlane->at<ito::complex64>(m_plotPts[n].rangeY[0], m_plotPts[n].rangeX[0])));
				//			break;
				//		}
				//	}
				//	break;
				//	case ito::tComplex128:
				//	{
				//		switch (m_cmplxState)
				//		{
				//			default:
				//			case cmplxAbs:
				//				return QPointF(fPos, abs(curPlane->at<ito::complex128>(m_plotPts[n].rangeY[0], m_plotPts[n].rangeX[0])));
				//			break;
				//			case cmplxReal:
				//				return QPointF(fPos, (curPlane->at<ito::complex128>(m_plotPts[n].rangeY[0], m_plotPts[n].rangeX[0]).real()));
				//			break;
				//			case cmplxImag:
				//				return QPointF(fPos, (curPlane->at<ito::complex128>(m_plotPts[n].rangeY[0], m_plotPts[n].rangeX[0]).imag()));
				//			break;
				//			case cmplxArg:
				//				return QPointF(fPos, arg(curPlane->at<ito::complex128>(m_plotPts[n].rangeY[0], m_plotPts[n].rangeX[0])));
				//			break;
				//		}
				//	}
				//	break;
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
				//	case ito::tComplex64:
				//	{
				//		switch (m_cmplxState)
				//		{
				//			default:
				//			case cmplxAbs:
				//				rVal = curPlane->at<ito::complex64>(m_plotPts[n].rangeY[0], m_plotPts[n].rangeX[0]).real() * m_plotPts[n].weights[0] +
				//					curPlane->at<ito::complex64>(m_plotPts[n].rangeY[1], m_plotPts[n].rangeX[0]).real() * m_plotPts[n].weights[1] +
				//					curPlane->at<ito::complex64>(m_plotPts[n].rangeY[1], m_plotPts[n].rangeX[1]).real() * m_plotPts[n].weights[2] +
				//					curPlane->at<ito::complex64>(m_plotPts[n].rangeY[0], m_plotPts[n].rangeX[1]).real() * m_plotPts[n].weights[3];
				//				iVal = curPlane->at<ito::complex64>(m_plotPts[n].rangeY[0], m_plotPts[n].rangeX[0]).imag() * m_plotPts[n].weights[0] +
				//					curPlane->at<ito::complex64>(m_plotPts[n].rangeY[1], m_plotPts[n].rangeX[0]).imag() * m_plotPts[n].weights[1] +
				//					curPlane->at<ito::complex64>(m_plotPts[n].rangeY[1], m_plotPts[n].rangeX[1]).imag() * m_plotPts[n].weights[2] +
				//					curPlane->at<ito::complex64>(m_plotPts[n].rangeY[0], m_plotPts[n].rangeX[1]).imag() * m_plotPts[n].weights[3];
				//				val = sqrt(rVal * rVal + iVal * iVal);
				//			break;
				//			case cmplxReal:
				//				val = curPlane->at<ito::complex64>(m_plotPts[n].rangeY[0], m_plotPts[n].rangeX[0]).real() * m_plotPts[n].weights[0] +
				//					curPlane->at<ito::complex64>(m_plotPts[n].rangeY[1], m_plotPts[n].rangeX[0]).real() * m_plotPts[n].weights[1] +
				//					curPlane->at<ito::complex64>(m_plotPts[n].rangeY[1], m_plotPts[n].rangeX[1]).real() * m_plotPts[n].weights[2] +
				//					curPlane->at<ito::complex64>(m_plotPts[n].rangeY[0], m_plotPts[n].rangeX[1]).real() * m_plotPts[n].weights[3];
				//			break;
				//			case cmplxImag:
				//				val = curPlane->at<ito::complex64>(m_plotPts[n].rangeY[0], m_plotPts[n].rangeX[0]).imag() * m_plotPts[n].weights[0] +
				//					curPlane->at<ito::complex64>(m_plotPts[n].rangeY[1], m_plotPts[n].rangeX[0]).imag() * m_plotPts[n].weights[1] +
				//					curPlane->at<ito::complex64>(m_plotPts[n].rangeY[1], m_plotPts[n].rangeX[1]).imag() * m_plotPts[n].weights[2] +
				//					curPlane->at<ito::complex64>(m_plotPts[n].rangeY[0], m_plotPts[n].rangeX[1]).imag() * m_plotPts[n].weights[3];
				//			break;
				//			case cmplxArg:
				//				rVal = curPlane->at<ito::complex64>(m_plotPts[n].rangeY[0], m_plotPts[n].rangeX[0]).real() * m_plotPts[n].weights[0] +
				//					curPlane->at<ito::complex64>(m_plotPts[n].rangeY[1], m_plotPts[n].rangeX[0]).real() * m_plotPts[n].weights[1] +
				//					curPlane->at<ito::complex64>(m_plotPts[n].rangeY[1], m_plotPts[n].rangeX[1]).real() * m_plotPts[n].weights[2] +
				//					curPlane->at<ito::complex64>(m_plotPts[n].rangeY[0], m_plotPts[n].rangeX[1]).real() * m_plotPts[n].weights[3];
				//				iVal = curPlane->at<ito::complex64>(m_plotPts[n].rangeY[0], m_plotPts[n].rangeX[0]).imag() * m_plotPts[n].weights[0] +
				//					curPlane->at<ito::complex64>(m_plotPts[n].rangeY[1], m_plotPts[n].rangeX[0]).imag() * m_plotPts[n].weights[1] +
				//					curPlane->at<ito::complex64>(m_plotPts[n].rangeY[1], m_plotPts[n].rangeX[1]).imag() * m_plotPts[n].weights[2] +
				//					curPlane->at<ito::complex64>(m_plotPts[n].rangeY[0], m_plotPts[n].rangeX[1]).imag() * m_plotPts[n].weights[3];
				//				val = atan2(iVal, rVal);
				//			break;
				//		}
				//	}
				//	break;
				//	case ito::tComplex128:
				//	{
				//		switch (m_cmplxState)
				//		{
				//			default:
				//			case cmplxAbs:
				//				rVal = curPlane->at<ito::complex128>(m_plotPts[n].rangeY[0], m_plotPts[n].rangeX[0]).real() * m_plotPts[n].weights[0] +
				//					curPlane->at<ito::complex128>(m_plotPts[n].rangeY[1], m_plotPts[n].rangeX[0]).real() * m_plotPts[n].weights[1] +
				//					curPlane->at<ito::complex128>(m_plotPts[n].rangeY[1], m_plotPts[n].rangeX[1]).real() * m_plotPts[n].weights[2] +
				//					curPlane->at<ito::complex128>(m_plotPts[n].rangeY[0], m_plotPts[n].rangeX[1]).real() * m_plotPts[n].weights[3];
				//				iVal = curPlane->at<ito::complex128>(m_plotPts[n].rangeY[0], m_plotPts[n].rangeX[0]).imag() * m_plotPts[n].weights[0] +
				//					curPlane->at<ito::complex128>(m_plotPts[n].rangeY[1], m_plotPts[n].rangeX[0]).imag() * m_plotPts[n].weights[1] +
				//					curPlane->at<ito::complex128>(m_plotPts[n].rangeY[1], m_plotPts[n].rangeX[1]).imag() * m_plotPts[n].weights[2] +
				//					curPlane->at<ito::complex128>(m_plotPts[n].rangeY[0], m_plotPts[n].rangeX[1]).imag() * m_plotPts[n].weights[3];
				//				val = sqrt(rVal * rVal + iVal * iVal);
				//			break;
				//			case cmplxReal:
				//				val = curPlane->at<ito::complex128>(m_plotPts[n].rangeY[0], m_plotPts[n].rangeX[0]).real() * m_plotPts[n].weights[0] +
				//					curPlane->at<ito::complex128>(m_plotPts[n].rangeY[1], m_plotPts[n].rangeX[0]).real() * m_plotPts[n].weights[1] +
				//					curPlane->at<ito::complex128>(m_plotPts[n].rangeY[1], m_plotPts[n].rangeX[1]).real() * m_plotPts[n].weights[2] +
				//					curPlane->at<ito::complex128>(m_plotPts[n].rangeY[0], m_plotPts[n].rangeX[1]).real() * m_plotPts[n].weights[3];
				//			break;
				//			case cmplxImag:
				//				val = curPlane->at<ito::complex128>(m_plotPts[n].rangeY[0], m_plotPts[n].rangeX[0]).imag() * m_plotPts[n].weights[0] +
				//					curPlane->at<ito::complex128>(m_plotPts[n].rangeY[1], m_plotPts[n].rangeX[0]).imag() * m_plotPts[n].weights[1] +
				//					curPlane->at<ito::complex128>(m_plotPts[n].rangeY[1], m_plotPts[n].rangeX[1]).imag() * m_plotPts[n].weights[2] +
				//					curPlane->at<ito::complex128>(m_plotPts[n].rangeY[0], m_plotPts[n].rangeX[1]).imag() * m_plotPts[n].weights[3];
				//			break;
				//			case cmplxArg:
				//				rVal = curPlane->at<ito::complex128>(m_plotPts[n].rangeY[0], m_plotPts[n].rangeX[0]).real() * m_plotPts[n].weights[0] +
				//					curPlane->at<ito::complex128>(m_plotPts[n].rangeY[1], m_plotPts[n].rangeX[0]).real() * m_plotPts[n].weights[1] +
				//					curPlane->at<ito::complex128>(m_plotPts[n].rangeY[1], m_plotPts[n].rangeX[1]).real() * m_plotPts[n].weights[2] +
				//					curPlane->at<ito::complex128>(m_plotPts[n].rangeY[0], m_plotPts[n].rangeX[1]).real() * m_plotPts[n].weights[3];
				//				iVal = curPlane->at<ito::complex128>(m_plotPts[n].rangeY[0], m_plotPts[n].rangeX[0]).imag() * m_plotPts[n].weights[0] +
				//					curPlane->at<ito::complex128>(m_plotPts[n].rangeY[1], m_plotPts[n].rangeX[0]).imag() * m_plotPts[n].weights[1] +
				//					curPlane->at<ito::complex128>(m_plotPts[n].rangeY[1], m_plotPts[n].rangeX[1]).imag() * m_plotPts[n].weights[2] +
				//					curPlane->at<ito::complex128>(m_plotPts[n].rangeY[0], m_plotPts[n].rangeX[1]).imag() * m_plotPts[n].weights[3];
				//				val = atan2(iVal, rVal);
				//			break;
				//		}
				//	}
				//	break;
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

//------------------------------------------------------------------------------------------------------------
ito::DataObject DataObjectSeriesData::getResampledDataObject()
{
    //if(m_plotPts.size() > 1)
    //{
    //    //setRasterObj();

    //    ito::DataObject temp(1, m_plotPts.size(), ito::tFloat64);
    //    ito::float64* ptrTemp = ((ito::float64*)((cv::Mat*)temp.get_mdata()[0])->ptr(0));

    //    QPointF val = sample(0);
    //    
    //    double offset = val.x();
    //    double scale = val.x();
    //    *(ptrTemp++) = val.y();

    //    val = sample(1);
    //    scale -= val.x();
    //    offset = offset/scale;

    //    *(ptrTemp++) = val.y();

    //    temp.setAxisScale(1, scale);
    //    temp.setAxisOffset(1, offset);

    //    for(int i = 2; i < m_plotPts.size(); i++)
    //    {
    //        *(ptrTemp++) = sample(i).y();
    //    }

    //    //releaseRasterObj();
    //    return temp;
    //}
    

    return ito::DataObject();
}
////----------------------------------------------------------------------------------------------------------------------------------
//QRectF DataObjectSeriesData::boundingRect() const
//{
//    QRectF res;
//
//    if (m_pDataObj)
//    {
//        double max = -1.0e308;
//        double min = 1.0e308;
//
//        switch(m_dir)
//        {
//        case dirZ:
//
//            res.setX( m_pointsZ.xLeft );
//            res.setY( m_pointsZ.xLeft + m_pointsZ.xStepSize * (m_numPts -1) );
//
//            switch(m_pDataObj->getType())
//            {
//                case ito::tInt8:
//                    findMinMaxInZ<int8>(m_pDataObj, m_startPhysOrg, min, max);
//                break;
//                case ito::tUInt8:
//                    findMinMaxInZ<uint8>(m_pDataObj, m_startPhysOrg, min, max);
//                break;
//                case ito::tInt16:
//                    findMinMaxInZ<int16>(m_pDataObj, m_startPhysOrg, min, max);
//                break;
//                case ito::tUInt16:
//                    findMinMaxInZ<uint16>(m_pDataObj, m_startPhysOrg, min, max);
//                break;
//                case ito::tInt32:
//                    findMinMaxInZ<int32>(m_pDataObj, m_startPhysOrg, min, max);
//                break;
//                case ito::tUInt32:
//                    findMinMaxInZ<uint32>(m_pDataObj, m_startPhysOrg, min, max);
//                break;
//                case ito::tFloat32:
//                    findMinMaxInZ<float32>(m_pDataObj, m_startPhysOrg, min, max);
//                break;
//                case ito::tFloat64:
//                    findMinMaxInZ<float64>(m_pDataObj, m_startPhysOrg, min, max);
//                break;
//                case ito::tComplex64:
//                    findMinMaxInZ<complex64>(m_pDataObj, m_startPhysOrg, min, max, m_cmplxState);
//                break;
//                case ito::tComplex128:
//                    findMinMaxInZ<complex128>(m_pDataObj, m_startPhysOrg, min, max, m_cmplxState);
//                break;
//            }
//            break;
//
//        case dirX:
//
//            res.setX( m_pointsX.xLeft );
//            res.setY( m_pointsX.xLeft + m_pointsX.xStepSize * (m_numPts -1) );
//
//            break;
//
//        case dirY:
//
//            res.setX( m_pointsY.yTop );
//            res.setY( m_pointsY.yTop + m_pointsY.yStepSize * (m_numPts -1) );
//
//            break;
//        case dirXY:
//
//            break;
//        }
//            /*}
//            else
//            {
//                switch(m_pDataObj->getType())
//                {
//                    case ito::tInt8:
//                        findMinMax<int8>(mat, m_plotPts, m_fast, min, max);
//                    break;
//                    case ito::tUInt8:
//                        findMinMax<uint8>(mat, m_plotPts, m_fast, min, max);
//                    break;
//                    case ito::tInt16:
//                        findMinMax<int16>(mat, m_plotPts, m_fast, min, max);
//                    break;
//                    case ito::tUInt16:
//                        findMinMax<uint16>(mat, m_plotPts, m_fast, min, max);
//                    break;
//                    case ito::tInt32:
//                        findMinMax<int32>(mat, m_plotPts, m_fast, min, max);
//                    break;
//                    case ito::tUInt32:
//                        findMinMax<uint32>(mat, m_plotPts, m_fast, min, max);
//                    break;
//                    case ito::tFloat32:
//                        findMinMax<float32>(mat, m_plotPts, m_fast, min, max);
//                    break;
//                    case ito::tFloat64:
//                        findMinMax<float64>(mat, m_plotPts, m_fast, min, max);
//                    break;
//					case ito::tComplex64:
//                        findMinMax<complex64>(mat, m_plotPts, m_fast, min, max, m_cmplxState);
//                    break;
//					case ito::tComplex128:
//                        findMinMax<complex128>(mat, m_plotPts, m_fast, min, max, m_cmplxState);
//                    break;
//                }            
//            }
//
//        }
//        else
//        {
//            min = m_minY;
//            max = m_maxY;
//        }
//        
//        if(m_autoScaleX)
//        {
//            if(m_Scaling > 0) return QRectF(m_startPos, min, m_physLength, max - min);
//            else return QRectF(m_startPos + m_physLength, min, abs(m_physLength), max - min);
//        }
//        else
//        {
//
//            return QRectF(m_minX, min, m_maxX-m_minX, max - min);
//        }*/
//    }
//    
//    return res;
//}

////------------------------------------------------------------------------------------------------------------
//QRectF DataObjectSeriesData::boundingRectMax() const
//{
////    double minX = -1.0;
////    double maxX = 2.0;
//
//    double minY = -std::numeric_limits<ito::float64>::max();
//    double maxY = std::numeric_limits<ito::float64>::max();
//    
//    if (m_pDataObj != NULL)
//    {
//        switch(m_pDataObj->getType())
//        {
//            case ito::tInt8:
//                minY = std::numeric_limits<ito::int8>::min();
//                maxY = std::numeric_limits<ito::int8>::max();
//            break;
//            case ito::tUInt8:
//                minY = std::numeric_limits<ito::int8>::min();
//                maxY = std::numeric_limits<ito::int8>::max();
//            break;
//            case ito::tInt16:
//                minY = std::numeric_limits<ito::int16>::min();
//                maxY = std::numeric_limits<ito::int16>::max();
//            break;
//            case ito::tUInt16:
//                minY = std::numeric_limits<ito::uint16>::min();
//                maxY = std::numeric_limits<ito::uint16>::max();
//            break;
//            case ito::tInt32:
//                minY = std::numeric_limits<ito::int32>::min();
//                maxY = std::numeric_limits<ito::int32>::max();
//            break;
//            case ito::tUInt32:
//                minY = std::numeric_limits<ito::uint32>::min();
//                maxY = std::numeric_limits<ito::uint32>::max();
//            break;
//            case ito::tFloat32:
//            case ito::tComplex64:
//                minY = -std::numeric_limits<ito::float32>::max();
//                maxY = std::numeric_limits<ito::float32>::max();
//            break;
//        }
//    }
//
//    if(m_Scaling > 0) return QRectF(m_startPos, minY, m_physLength, maxY - minY);
//    else return QRectF(m_startPos + m_physLength, minY, abs(m_physLength), maxY - minY);
//}

//----------------------------------------------------------------------------------------------------------------------------------
template<typename _Tp> void findMinMaxNonWeighted(const ito::DataObject *obj, const DataObjectSeriesData::LineData &d, double &min, double &max, int &minIdx, int &maxIdx, DataObjectSeriesData::ComplexType cmplxState = DataObjectSeriesData::cmplxAbs)
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
        mat = (cv::Mat*)(obj->get_mdata()[ obj->seekMat(0) ]);
        ptr = (mat->data + d.matOffset);
        for(size_t i = 0 ; i < d.nrPoints ; i++)
        {
            val = *(reinterpret_cast<_Tp*>(ptr));
            ptr += d.matStepSize;

            if (val > max) { max = val; maxIdx = i; }
            if (val < min) { min = val; minIdx = i; }
            
        }
        break;

    case DataObjectSeriesData::dirXY:
        mat = (cv::Mat*)(obj->get_mdata()[ obj->seekMat(0) ]);
        ptr = (mat->data + d.matOffset);
        for(size_t i = 0 ; i < d.nrPoints ; i++)
        {
            val = *(reinterpret_cast<_Tp*>(ptr + d.matSteps[i]));
            ptr += d.matStepSize;

            if (val > max) { max = val; maxIdx = i; }
            if (val < min) { min = val; minIdx = i; }
            
        }
        break;
    case DataObjectSeriesData::dirZ:

        for(size_t i = 0 ; i < d.nrPoints ; i++)
        {
            mat = (cv::Mat*)(obj->get_mdata()[ obj->seekMat(i) ]);
            ptr = (mat->data + d.matOffset);
            val = *(reinterpret_cast<_Tp*>(ptr));

            if (val > max) { max = val; maxIdx = i; }
            if (val < min) { min = val; minIdx = i; }           
        }
        break;

    }
}

//----------------------------------------------------------------------------------------------------------------------------------
template<> void findMinMaxNonWeighted<ito::float32>(const ito::DataObject *obj, const DataObjectSeriesData::LineData &d, double &min, double &max, int &minIdx, int &maxIdx, DataObjectSeriesData::ComplexType cmplxState)
{
    const cv::Mat *mat;
    uchar *ptr;
    min = std::numeric_limits<float64>::max();
    max = -min;
    float32 val;

    switch(d.dir)
    {
    case DataObjectSeriesData::dirX:
    case DataObjectSeriesData::dirY:
        mat = (cv::Mat*)(obj->get_mdata()[ obj->seekMat(0) ]);
        ptr = (mat->data + d.matOffset);
        for(size_t i = 0 ; i < d.nrPoints ; i++)
        {
            val = *(reinterpret_cast<float32*>(ptr));
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
        mat = (cv::Mat*)(obj->get_mdata()[ obj->seekMat(0) ]);
        ptr = (mat->data + d.matOffset);
        for(size_t i = 0 ; i < d.nrPoints ; i++)
        {
            val = *(reinterpret_cast<float32*>(ptr + d.matSteps[i]));

            if (!qIsFinite(val))
            {
                continue;
            }

            if (val > max) { max = val; maxIdx = i; }
            if (val < min) { min = val; minIdx = i; }
            
        }
        break;
    case DataObjectSeriesData::dirZ:

        for(size_t i = 0 ; i < d.nrPoints ; i++)
        {
            mat = (cv::Mat*)(obj->get_mdata()[ obj->seekMat(i) ]);
            ptr = (mat->data + d.matOffset);
            val = *(reinterpret_cast<float32*>(ptr));

            if (!qIsFinite(val))
            {
                continue;
            }

            if (val > max) { max = val; maxIdx = i; }
            if (val < min) { min = val; minIdx = i; }           
        }
        break;

    }
}

//----------------------------------------------------------------------------------------------------------------------------------
template<> void findMinMaxNonWeighted<ito::float64>(const ito::DataObject *obj, const DataObjectSeriesData::LineData &d, double &min, double &max, int &minIdx, int &maxIdx, DataObjectSeriesData::ComplexType cmplxState)
{
    const cv::Mat *mat;
    uchar *ptr;
    min = std::numeric_limits<float64>::max();
    max = -min;
    float64 val;

    switch(d.dir)
    {
    case DataObjectSeriesData::dirX:
    case DataObjectSeriesData::dirY:
        mat = (cv::Mat*)(obj->get_mdata()[ obj->seekMat(0) ]);
        ptr = (mat->data + d.matOffset);
        for(size_t i = 0 ; i < d.nrPoints ; i++)
        {
            val = *(reinterpret_cast<float64*>(ptr));
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
        mat = (cv::Mat*)(obj->get_mdata()[ obj->seekMat(0) ]);
        ptr = (mat->data + d.matOffset);
        for(size_t i = 0 ; i < d.nrPoints ; i++)
        {
            val = *(reinterpret_cast<float64*>(ptr + d.matSteps[i]));

            if (!qIsFinite(val))
            {
                continue;
            }

            if (val > max) { max = val; maxIdx = i; }
            if (val < min) { min = val; minIdx = i; } 
            
        }
        break;

    case DataObjectSeriesData::dirZ:

        for(size_t i = 0 ; i < d.nrPoints ; i++)
        {
            mat = (cv::Mat*)(obj->get_mdata()[ obj->seekMat(i) ]);
            ptr = (mat->data + d.matOffset);
            val = *(reinterpret_cast<float64*>(ptr));

            if (!qIsFinite(val))
            {
                continue;
            }

            if (val > max) { max = val; maxIdx = i; }
            if (val < min) { min = val; minIdx = i; }           
        }
        break;

    }
}

//----------------------------------------------------------------------------------------------------------------------------------
template<> void findMinMaxNonWeighted<ito::complex64>(const ito::DataObject *obj, const DataObjectSeriesData::LineData &d, double &min, double &max, int &minIdx, int &maxIdx, DataObjectSeriesData::ComplexType cmplxState)
{
    const cv::Mat *mat;
    uchar *ptr;
    min = std::numeric_limits<float64>::max();
    max = -min;
    ito::float32 val;
    ito::complex64 val_;

    switch(d.dir)
    {
    case DataObjectSeriesData::dirX:
    case DataObjectSeriesData::dirY:
        mat = (cv::Mat*)(obj->get_mdata()[ obj->seekMat(0) ]);
        ptr = (mat->data + d.matOffset);
        for(size_t i = 0 ; i < d.nrPoints ; i++)
        {
            val_ = *(reinterpret_cast<complex64*>(ptr));
            switch(cmplxState)
            {
            case DataObjectSeriesData::cmplxAbs:
                val = abs(val_);
                break;
            case DataObjectSeriesData::cmplxReal:
                val = val_.real();
                break;
            case DataObjectSeriesData::cmplxImag:
                val = val_.imag();
                break;
            case DataObjectSeriesData::cmplxArg:
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
        mat = (cv::Mat*)(obj->get_mdata()[ obj->seekMat(0) ]);
        ptr = (mat->data + d.matOffset);
        for(size_t i = 0 ; i < d.nrPoints ; i++)
        {
            val_ = *(reinterpret_cast<complex64*>(ptr + d.matSteps[i]));
            switch(cmplxState)
            {
            case DataObjectSeriesData::cmplxAbs:
                val = abs(val_);
                break;
            case DataObjectSeriesData::cmplxReal:
                val = val_.real();
                break;
            case DataObjectSeriesData::cmplxImag:
                val = val_.imag();
                break;
            case DataObjectSeriesData::cmplxArg:
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

        for(size_t i = 0 ; i < d.nrPoints ; i++)
        {
            mat = (cv::Mat*)(obj->get_mdata()[ obj->seekMat(i) ]);
            ptr = (mat->data + d.matOffset);
            val_ = *(reinterpret_cast<complex64*>(ptr));
            switch(cmplxState)
            {
            case DataObjectSeriesData::cmplxAbs:
                val = abs(val_);
                break;
            case DataObjectSeriesData::cmplxReal:
                val = val_.real();
                break;
            case DataObjectSeriesData::cmplxImag:
                val = val_.imag();
                break;
            case DataObjectSeriesData::cmplxArg:
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
}

//----------------------------------------------------------------------------------------------------------------------------------
template<> void findMinMaxNonWeighted<ito::complex128>(const ito::DataObject *obj, const DataObjectSeriesData::LineData &d, double &min, double &max, int &minIdx, int &maxIdx, DataObjectSeriesData::ComplexType cmplxState)
{
    const cv::Mat *mat;
    uchar *ptr;
    min = std::numeric_limits<float64>::max();
    max = -min;
    ito::float64 val;
    ito::complex128 val_;

    switch(d.dir)
    {
    case DataObjectSeriesData::dirX:
    case DataObjectSeriesData::dirY:
        mat = (cv::Mat*)(obj->get_mdata()[ obj->seekMat(0) ]);
        ptr = (mat->data + d.matOffset);
        for(size_t i = 0 ; i < d.nrPoints ; i++)
        {
            val_ = *(reinterpret_cast<complex128*>(ptr));
            switch(cmplxState)
            {
            case DataObjectSeriesData::cmplxAbs:
                val = abs(val_);
                break;
            case DataObjectSeriesData::cmplxReal:
                val = val_.real();
                break;
            case DataObjectSeriesData::cmplxImag:
                val = val_.imag();
                break;
            case DataObjectSeriesData::cmplxArg:
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
        mat = (cv::Mat*)(obj->get_mdata()[ obj->seekMat(0) ]);
        ptr = (mat->data + d.matOffset);
        for(size_t i = 0 ; i < d.nrPoints ; i++)
        {
            val_ = *(reinterpret_cast<complex128*>(ptr + d.matSteps[i]));
            switch(cmplxState)
            {
            case DataObjectSeriesData::cmplxAbs:
                val = abs(val_);
                break;
            case DataObjectSeriesData::cmplxReal:
                val = val_.real();
                break;
            case DataObjectSeriesData::cmplxImag:
                val = val_.imag();
                break;
            case DataObjectSeriesData::cmplxArg:
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

        for(size_t i = 0 ; i < d.nrPoints ; i++)
        {
            mat = (cv::Mat*)(obj->get_mdata()[ obj->seekMat(i) ]);
            ptr = (mat->data + d.matOffset);
            val_ = *(reinterpret_cast<complex128*>(ptr));
            switch(cmplxState)
            {
            case DataObjectSeriesData::cmplxAbs:
                val = abs(val_);
                break;
            case DataObjectSeriesData::cmplxReal:
                val = val_.real();
                break;
            case DataObjectSeriesData::cmplxImag:
                val = val_.imag();
                break;
            case DataObjectSeriesData::cmplxArg:
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
}

////----------------------------------------------------------------------------------------------------------------------------------
//template<typename _Tp> void findMinMaxInZ(const ito::DataObject *obj, const QPointF &xyPos, double &min, double &max, DataObjectSeriesData::ComplexType cmplxState = DataObjectSeriesData::cmplxAbs)
//{
//    min = std::numeric_limits<_Tp>::max();
//    if(std::numeric_limits<_Tp>::is_exact)
//    {
//        max = std::numeric_limits<_Tp>::min(); //integer numbers
//    }
//    else
//    {
//        max = -std::numeric_limits<_Tp>::max();
//    }
//
//    int dims = obj->getDims();
//    bool _unused;
//    int x = obj->getPhysToPix( dims - 2, xyPos.x(), _unused);
//    int y = obj->getPhysToPix( dims - 1, xyPos.y(), _unused);
//
//    x = std::max(x,0);
//    x = std::min(x, obj->getSize( dims - 2 ) - 1);
//
//    y = std::max(y,0);
//    y = std::min(y, obj->getSize( dims - 1 ) - 1);
//
//    int sizeZ = obj->getSize( dims - 3 );
//    
//    double val;
//    cv::Mat *mat;
//
//    for (int n = 0; n < sizeZ; n++)
//    {
//        mat = (cv::Mat *)(obj.get_mdata())[obj.seekMat(n)];
//        val = mat->at<_Tp>(y, x);
//        if (!qIsFinite(val))
//            continue;
//        if (val > max)
//            max = val;
//        if (val < min)
//            min = val;
//    }
//}
//
////----------------------------------------------------------------------------------------------------------------------------------
//template<> void findMinMaxInZ<ito::complex64>(const ito::DataObject *obj, const QPointF &xyPos, double &min, double &max, DataObjectSeriesData::ComplexType cmplxState)
//{
//    min = std::numeric_limits<_Tp>::max();
//    if(std::numeric_limits<_Tp>::is_exact)
//    {
//        max = std::numeric_limits<_Tp>::min(); //integer numbers
//    }
//    else
//    {
//        max = -std::numeric_limits<_Tp>::max();
//    }
//
//    int dims = obj->getDims();
//    bool _unused;
//    int x = obj->getPhysToPix( dims - 2, xyPos.x(), _unused);
//    int y = obj->getPhysToPix( dims - 1, xyPos.y(), _unused);
//
//    x = std::max(x,0);
//    x = std::min(x, obj->getSize( dims - 2 ) - 1);
//
//    y = std::max(y,0);
//    y = std::min(y, obj->getSize( dims - 1 ) - 1);
//
//    int sizeZ = obj->getSize( dims - 3 );
//    
//    double val;
//    cv::Mat *mat;
//
//	switch (cmplxState)
//	{
//		default:
//        case DataObjectSeriesData::cmplxAbs:
//		{
//			for (int n = 0; n < sizeZ; n++)
//			{
//				mat = (cv::Mat *)(obj.get_mdata())[obj.seekMat(n)];
//				val = abs( mat->at<ito::complex64>(y, x) );
//
//				if (!qIsFinite(val))
//					continue;
//				if (val > max)
//					max = val;
//				if (val < min)
//					min = val;
//			}
//		}
//		break;
//
//		case DataObjectSeriesData::cmplxReal:
//		{
//			for (int n = 0; n < sizeZ; n++)
//			{
//				mat = (cv::Mat *)(obj.get_mdata())[obj.seekMat(n)];
//				val = (mat->at<ito::complex64>(y, x)).real();
//
//				if (!qIsFinite(val))
//					continue;
//				if (val > max)
//					max = val;
//				if (val < min)
//					min = val;
//			}
//		}
//		break;
//
//		case DataObjectSeriesData::cmplxImag:
//		{
//			for (int n = 0; n < sizeZ; n++)
//			{
//				mat = (cv::Mat *)(obj.get_mdata())[obj.seekMat(n)];
//				val = (mat->at<ito::complex64>(y, x)).imag();
//
//				if (!qIsFinite(val))
//					continue;
//				if (val > max)
//					max = val;
//				if (val < min)
//					min = val;
//			}
//		}
//		break;
//
//		case DataObjectSeriesData::cmplxArg:
//		{
//			for (int n = 0; n < sizeZ; n++)
//			{
//				mat = (cv::Mat *)(obj.get_mdata())[obj.seekMat(n)];
//				val = arg(mat->at<ito::complex64>(y, x));
//
//				if (!qIsFinite(val))
//					continue;
//				if (val > max)
//					max = val;
//				if (val < min)
//					min = val;
//			}
//		}
//		break;
//	}
//}
//
////----------------------------------------------------------------------------------------------------------------------------------
//template<> void findMinMaxInZ<ito::complex128>(const ito::DataObject *obj, const QPointF &xyPos, double &min, double &max, DataObjectSeriesData::ComplexType cmplxState)
//{
//    min = std::numeric_limits<_Tp>::max();
//    if(std::numeric_limits<_Tp>::is_exact)
//    {
//        max = std::numeric_limits<_Tp>::min(); //integer numbers
//    }
//    else
//    {
//        max = -std::numeric_limits<_Tp>::max();
//    }
//
//    int dims = obj->getDims();
//    bool _unused;
//    int x = obj->getPhysToPix( dims - 2, xyPos.x(), _unused);
//    int y = obj->getPhysToPix( dims - 1, xyPos.y(), _unused);
//
//    x = std::max(x,0);
//    x = std::min(x, obj->getSize( dims - 2 ) - 1);
//
//    y = std::max(y,0);
//    y = std::min(y, obj->getSize( dims - 1 ) - 1);
//
//    int sizeZ = obj->getSize( dims - 3 );
//    
//    double val;
//    cv::Mat *mat;
//
//	switch (cmplxState)
//	{
//		default:
//		case DataObjectSeriesData::cmplxAbs:
//		{
//			for (int n = 0; n < sizeZ; n++)
//			{
//				mat = (cv::Mat *)(obj.get_mdata())[obj.seekMat(n)];
//				val = abs(mat->at<ito::complex64>(y, x));
//
//				if (!qIsFinite(val))
//					continue;
//				if (val > max)
//					max = val;
//				if (val < min)
//					min = val;
//			}
//		}
//		break;
//
//        case DataObjectSeriesData::cmplxReal:
//		{
//			for (int n = 0; n < sizeZ; n++)
//			{
//				mat = (cv::Mat *)(obj.get_mdata())[obj.seekMat(n)];
//				val = (mat->at<ito::complex64>(y, x)).real();
//
//				if (!qIsFinite(val))
//					continue;
//				if (val > max)
//					max = val;
//				if (val < min)
//					min = val;
//			}
//		}
//		break;
//
//		case DataObjectSeriesData::cmplxImag:
//		{
//			for (int n = 0; n < sizeZ; n++)
//			{
//				mat = (cv::Mat *)(obj.get_mdata())[obj.seekMat(n)];
//				val = (mat->at<ito::complex64>(y, x)).imag();
//
//				if (!qIsFinite(val))
//					continue;
//				if (val > max)
//					max = val;
//				if (val < min)
//					min = val;
//			}
//		}
//		break;
//
//        case DataObjectSeriesData::cmplxArg:
//		{
//			for (int n = 0; n < sizeZ; n++)
//			{
//				mat = (cv::Mat *)(obj.get_mdata())[obj.seekMat(n)];
//				val = arg(mat->at<ito::complex64>(y, x));
//
//				if (!qIsFinite(val))
//					continue;
//				if (val > max)
//					max = val;
//				if (val < min)
//					min = val;
//			}
//		}
//		break;
//	}
//}
//
//
//
//
////----------------------------------------------------------------------------------------------------------------------------------
//template<typename _Tp> void findMinMaxInXY(const ito::DataObject *obj, const QPointF &xyStart, const QPointF &xyEnd, double &min, double &max, DataObjectSeriesData::ComplexType cmplxState = DataObjectSeriesData::cmplxAbs)
//{
//    min = std::numeric_limits<_Tp>::max();
//    if(std::numeric_limits<_Tp>::is_exact)
//    {
//        max = std::numeric_limits<_Tp>::min(); //integer numbers
//    }
//    else
//    {
//        max = -std::numeric_limits<_Tp>::max();
//    }
//
//    int dims = obj->getDims();
//    bool _unused;
//    int x = obj->getPhysToPix( dims - 2, xyPos.x(), _unused);
//    int y = obj->getPhysToPix( dims - 1, xyPos.y(), _unused);
//
//    x = std::max(x,0);
//    x = std::min(x, obj->getSize( dims - 2 ) - 1);
//
//    y = std::max(y,0);
//    y = std::min(y, obj->getSize( dims - 1 ) - 1);
//
//    int sizeZ = obj->getSize( dims - 3 );
//    
//    double val;
//    cv::Mat *mat;
//
//    for (int n = 0; n < sizeZ; n++)
//    {
//        mat = (cv::Mat *)(obj.get_mdata())[obj.seekMat(n)];
//        val = mat->at<_Tp>(y, x);
//        if (!qIsFinite(val))
//            continue;
//        if (val > max)
//            max = val;
//        if (val < min)
//            min = val;
//    }
//}
//
//




//----------------------------------------------------------------------------------------------------------------------------------
QRectF DataObjectSeriesData::boundingRect() const
{
    QRectF res;

    //cv::Mat *mat;
    //const uchar* ptr[4];
    //float weights[4];

    if(m_pDataObj && m_d.valid)
    {
        double min = 0.0, max = 0.0;
        int minIdx = 0.0, maxIdx = 0.0;
        switch(m_pDataObj->getType())
        {
            case ito::tInt8:
                findMinMaxNonWeighted<ito::int8>(m_pDataObj, m_d, min, max, minIdx, maxIdx);
            break;
            case ito::tUInt8:
                findMinMaxNonWeighted<ito::uint8>(m_pDataObj, m_d, min, max, minIdx, maxIdx);
            break;
            case ito::tInt16:
                findMinMaxNonWeighted<ito::int16>(m_pDataObj, m_d, min, max, minIdx, maxIdx);
            break;
            case ito::tUInt16:
                findMinMaxNonWeighted<ito::uint16>(m_pDataObj, m_d, min, max, minIdx, maxIdx);
            break;
            case ito::tInt32:
                findMinMaxNonWeighted<ito::int32>(m_pDataObj, m_d, min, max, minIdx, maxIdx);
            break;
            case ito::tUInt32:
                findMinMaxNonWeighted<ito::uint32>(m_pDataObj, m_d, min, max, minIdx, maxIdx);
            break;
            case ito::tFloat32:
                findMinMaxNonWeighted<ito::float32>(m_pDataObj, m_d, min, max, minIdx, maxIdx);
            break;
            case ito::tFloat64:
                findMinMaxNonWeighted<ito::float64>(m_pDataObj, m_d, min, max, minIdx, maxIdx);
            break;
            case ito::tComplex64:
                findMinMaxNonWeighted<ito::complex64>(m_pDataObj, m_d, min, max, minIdx, maxIdx, m_cmplxState);
            break;
            case ito::tComplex128:
                findMinMaxNonWeighted<ito::complex128>(m_pDataObj, m_d, min, max, minIdx, maxIdx, m_cmplxState);
            break;
            case ito::tRGBA32:
                min = 0.0;
                max = 255.0;
            break;
        }

        if( (max-min) < std::numeric_limits<double>::epsilon() )
        {
			if (min > 10.0)
			{
				min *= 0.99;
				max *= 1.01;
			}
			else if(min < -10.0)
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
        

        if(m_d.points.size() == 0) //no weighted stuff
        {
            res = QRectF(m_d.startPhys, min, m_d.stepSizePhys * (m_d.nrPoints-1), max-min);
            //switch (m_d.dir)
            //{
            //case dirX:
            //case dirY:
            //    res = QRectF(m_d.startPhys, min, m_d.stepSizePhys * (m_d.nrPoints-1), max-min);
            //break;

            //case dirZ:
            //    res = QRectF(m_d.startPhys, min, m_d.stepSizePhys * (m_d.nrPoints-1), max-min);
            //break;

            //case dirXY:
            //    res = QRectF(m_d.startPhys, min, m_d.stepSizePhys * (m_d.nrPoints-1), max-min);
            //    return res;
            //break;
            //}
        }
        else
        {
            qDebug() << "not yet implemented";
            //not implemented yet
        }
    }
    
    return res;
}


RetVal DataObjectSeriesData::getMinMaxLoc(double &min, double &max, int &minSampleIdx, int &maxSampleIdx) const
{
    QRectF res;

    //cv::Mat *mat;
    //const uchar* ptr[4];
    //float weights[4];
    minSampleIdx = 0;
    maxSampleIdx = 0;
    RetVal retval;

    if(m_pDataObj && m_d.valid)
    {
        switch(m_pDataObj->getType())
        {
            case ito::tInt8:
                findMinMaxNonWeighted<ito::int8>(m_pDataObj, m_d, min, max, minSampleIdx, maxSampleIdx);
            break;
            case ito::tUInt8:
                findMinMaxNonWeighted<ito::uint8>(m_pDataObj, m_d, min, max, minSampleIdx, maxSampleIdx);
            break;
            case ito::tInt16:
                findMinMaxNonWeighted<ito::int16>(m_pDataObj, m_d, min, max, minSampleIdx, maxSampleIdx);
            break;
            case ito::tUInt16:
                findMinMaxNonWeighted<ito::uint16>(m_pDataObj, m_d, min, max, minSampleIdx, maxSampleIdx);
            break;
            case ito::tInt32:
                findMinMaxNonWeighted<ito::int32>(m_pDataObj, m_d, min, max, minSampleIdx, maxSampleIdx);
            break;
            case ito::tUInt32:
                findMinMaxNonWeighted<ito::uint32>(m_pDataObj, m_d, min, max, minSampleIdx, maxSampleIdx);
            break;
            case ito::tFloat32:
                findMinMaxNonWeighted<ito::float32>(m_pDataObj, m_d, min, max, minSampleIdx, maxSampleIdx);
            break;
            case ito::tFloat64:
                findMinMaxNonWeighted<ito::float64>(m_pDataObj, m_d, min, max, minSampleIdx, maxSampleIdx);
            break;
            case ito::tComplex64:
                findMinMaxNonWeighted<ito::complex64>(m_pDataObj, m_d, min, max, minSampleIdx, maxSampleIdx, m_cmplxState);
            break;
            case ito::tComplex128:
                findMinMaxNonWeighted<ito::complex128>(m_pDataObj, m_d, min, max, minSampleIdx, maxSampleIdx, m_cmplxState);
            break;
            case ito::tRGBA32:
                min = 0.0;
                max = 255.0;
            break;
        }

        if( max-min < std::numeric_limits<double>::epsilon() )
        {
            min *= 0.99;
            max *= 1.01;
        }
    }
    else
    {
        retval += RetVal(retError,0,"no dataObject");
    }
    
    return retval;
}
