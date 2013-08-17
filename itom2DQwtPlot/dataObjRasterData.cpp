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
#include "common/typeDefs.h"

#include "DataObject/dataObjectFuncs.h"

#include <qdebug.h>

//----------------------------------------------------------------------------------------------------------------------------------
DataObjRasterData::DataObjRasterData(const InternalData *m_internalData) :
    QwtRasterData(),
    m_validData(false),
    m_dataHash(),
    m_rasteredLinePtr(NULL),
    m_xIndizes(NULL),
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
    m_D.m_dataPtr = NULL;
}

//----------------------------------------------------------------------------------------------------------------------------------
DataObjRasterData::~DataObjRasterData()
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
QByteArray DataObjRasterData::calcHash(const ito::DataObject *dObj)
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
        ba.append( m_D.m_planeIdx );

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
void DataObjRasterData::deleteCache()
{
    m_validData = false;

    m_rasteredLines = 0;

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
}

//----------------------------------------------------------------------------------------------------------------------------------
bool DataObjRasterData::updateDataObject(const ito::DataObject *dataObj, int planeIdx /*= -1*/) //true if hash has changed
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
            m_plane = m_dataObj.getDims() > 1 ? (cv::Mat*)(m_dataObj.get_mdata()[ m_dataObj.seekMat( m_D.m_planeIdx )]) : NULL;

            if (m_dataObjPlane && dataObjPlaneWasShallow) //m_dataObjPlane was a shallow copy -> delete it
            {
                delete m_dataObjPlane;
                m_dataObjPlane = NULL;
            }

            if (m_dataObj.getDims() > 2)
            {
                size_t sizes[2] = { m_D.m_ySize, m_D.m_xSize };
                m_dataObjPlane = new ito::DataObject( 2, sizes, m_dataObj.getType(), m_plane, 1);
                m_dataObj.copyTagMapTo( *m_dataObjPlane );

                bool v;
                for (int i = 0; i < 2; ++i)
                {
                    m_dataObjPlane->setAxisDescription(i, m_dataObj.getAxisDescription( d - (2 - i), v ));
                    m_dataObjPlane->setAxisScale(i, m_dataObj.getAxisScale( d - (2 - i) ));
                    m_dataObjPlane->setAxisOffset(i, m_dataObj.getAxisOffset( d - (2 - i) ));
                }

                m_dataObjPlane->setValueDescription( m_dataObj.getValueDescription() );
                m_dataObjPlane->setValueUnit( m_dataObj.getValueUnit() );

                m_dataObj.copyAxisTagsTo( *m_dataObjPlane );
            }
            else
            {
                m_dataObjPlane = &m_dataObj;
            }

            m_pInternalData->m_pConstOutput->operator[]("sourceout")->setVal<void*>((void*)&m_dataObj);
            m_pInternalData->m_pConstOutput->operator[]("displayed")->setVal<void*>((void*)m_dataObjPlane);

            //Definition: Scale-Coordinate of dataObject =  ( px-Coordinate - Offset)* Scale
            setInterval(Qt::XAxis, QwtInterval(pxToScaleCoords(0,m_D.m_xOffset,m_D.m_xScaling), pxToScaleCoords(m_D.m_xSize-1,m_D.m_xOffset,m_D.m_xScaling)) );
            setInterval(Qt::YAxis, QwtInterval(pxToScaleCoords(0,m_D.m_yOffset,m_D.m_yScaling), pxToScaleCoords(m_D.m_ySize-1,m_D.m_yOffset,m_D.m_yScaling)) );
        }

        ito::float64 min, max;
        ito::uint32 firstMin[3];
        ito::uint32 firstMax[3];

        
        if (m_pInternalData->m_valueScaleAuto)
        {
            ito::dObjHelper::minMaxValue(m_dataObjPlane, min, firstMin, max, firstMax, true, m_pInternalData->m_cmplxType);

			if (min == std::numeric_limits<ito::float64>::max()) min = -10.0;
			if (max == -std::numeric_limits<ito::float64>::max()) max = 10.0;

			if ((max-min) < std::numeric_limits<double>::epsilon()) //the data object only contains the same value, then make the min-max range a little bit bigger in order to ensure a nice colorbar
			{
				switch (m_pInternalData->m_dataType)
				{
				case ito::tUInt8:
				case ito::tInt8:
				case ito::tUInt16:
				case ito::tInt16:
				case ito::tUInt32:
				case ito::tInt32:
					min -= 1;
					max += 1;
					break;
				case ito::tFloat32:
				case ito::tFloat64:
				case ito::tComplex64:
				case ito::tComplex128:
				default:
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
					break;
				}
			}
            setInterval(Qt::ZAxis, QwtInterval(min,max));
        }
        else
        {
            setInterval(Qt::ZAxis, QwtInterval(m_pInternalData->m_valueMin, m_pInternalData->m_valueMax));
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
        m_pInternalData->m_pConstOutput->operator[]("sourceout")->setVal<void*>(NULL);
        m_pInternalData->m_pConstOutput->operator[]("output")->setVal<void*>(NULL);

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

        setInterval(Qt::XAxis, QwtInterval() );
        setInterval(Qt::YAxis, QwtInterval() );
        setInterval(Qt::ZAxis, QwtInterval() );
    }
    
    
    return newHash;
}

bool DataObjRasterData::pointValid(const QPointF &point) const
{
    return interval(Qt::XAxis).contains( point.x() ) && interval(Qt::YAxis).contains( point.y() );
}

//----------------------------------------------------------------------------------------------------------------------------------
double DataObjRasterData::value(double x, double y) const
{
    bool inside1, inside2;
    if (m_dataObj.getDims() > 0 && m_plane)
    {
        int d = m_dataObj.getDims();

        if ( d>1 )
        {
            int n = qRound(m_dataObj.getPhysToPix(d-1, x, inside1));
            int m = qRound(m_dataObj.getPhysToPix(d-2, y, inside2));

            if (inside1 && inside2)
            {
                switch(m_dataObj.getType())
                {
                case ito::tInt8:
                    {
                        return m_plane->at<ito::int8>(m,n);
                    }
                case ito::tUInt8:
                    {
                        return m_plane->at<ito::uint8>(m,n);
                    }
                case ito::tInt16:
                    {
                        return m_plane->at<ito::int16>(m,n);
                    }
                case ito::tUInt16:
                    {
                        return m_plane->at<ito::uint16>(m,n);
                    }
                case ito::tInt32:
                    {
                        return m_plane->at<ito::int32>(m,n);
                    }
                case ito::tUInt32:
                    {
                        return m_plane->at<ito::uint32>(m,n);
                    }
                case ito::tFloat32:
                    {
                        return m_plane->at<ito::float32>(m,n);
                    }
                case ito::tFloat64:
                    {
                        return m_plane->at<ito::float64>(m,n);
                    }
                case ito::tComplex64:
                    {
                        if (m_pInternalData->m_cmplxType == PlotCanvas::Real)
                        {
                            return m_plane->at<ito::complex64>(m,n).real();
                        }
                        if (m_pInternalData->m_cmplxType == PlotCanvas::Imag)
                        {
                            return m_plane->at<ito::complex64>(m,n).imag();
                        }
                        if (m_pInternalData->m_cmplxType == PlotCanvas::Phase)
                        {
                            return std::arg( m_plane->at<ito::complex64>(m,n) );
                        }
                        else //if (m_pInternalData->m_cmplxType == PlotCanvas::Abs)
                        {
                            return std::abs( m_plane->at<ito::complex64>(m,n) );
                        }
                    }
                case ito::tComplex128:
                    {
                        ito::complex128 *line = (ito::complex128*)m_rasteredLinePtr[m];
                        if(!line) return std::numeric_limits<double>::signaling_NaN();
                        ito::complex128 i = line[ m_xIndizes[n] ];
                
                        if (m_pInternalData->m_cmplxType == PlotCanvas::Real)
                        {
                            return m_plane->at<ito::complex128>(m,n).real();
                        }
                        if (m_pInternalData->m_cmplxType == PlotCanvas::Imag)
                        {
                            return m_plane->at<ito::complex128>(m,n).imag();
                        }
                        if (m_pInternalData->m_cmplxType == PlotCanvas::Phase)
                        {
                            return std::arg( m_plane->at<ito::complex128>(m,n) );
                        }
                        else //if (m_pInternalData->m_cmplxType == PlotCanvas::Abs)
                        {
                            return std::abs( m_plane->at<ito::complex128>(m,n) );
                        }
                    }
                default:
                    return std::numeric_limits<double>::signaling_NaN();
                }
            }
        }
    }
    return std::numeric_limits<double>::signaling_NaN();
}

//----------------------------------------------------------------------------------------------------------------------------------
double DataObjRasterData::value2(int m, int n) const
{
    if(m_validData)
    {
        switch(m_dataObj.getType())
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
        case ito::tComplex64:
            {
                ito::complex64 *line = (ito::complex64*)m_rasteredLinePtr[m];
                if(!line) return std::numeric_limits<double>::signaling_NaN();
                ito::complex64 i = line[ m_xIndizes[n] ];

                if (m_pInternalData->m_cmplxType == PlotCanvas::Real)
                {
                    return line[ m_xIndizes[n] ].real();
                }
                if (m_pInternalData->m_cmplxType == PlotCanvas::Imag)
                {
                    return line[ m_xIndizes[n] ].imag();
                }
                if (m_pInternalData->m_cmplxType == PlotCanvas::Phase)
                {
                    return std::arg( line[ m_xIndizes[n] ] );
                }
                else //if (m_pInternalData->m_cmplxType == PlotCanvas::Abs)
                {
                    return std::abs( line[ m_xIndizes[n] ] );
                }
            }
        case ito::tComplex128:
            {
                ito::complex128 *line = (ito::complex128*)m_rasteredLinePtr[m];
                if(!line) return std::numeric_limits<double>::signaling_NaN();
                ito::complex128 i = line[ m_xIndizes[n] ];
                
                if (m_pInternalData->m_cmplxType == PlotCanvas::Real)
                {
                    return line[ m_xIndizes[n] ].real();
                }
                if (m_pInternalData->m_cmplxType == PlotCanvas::Imag)
                {
                    return line[ m_xIndizes[n] ].imag();
                }
                if (m_pInternalData->m_cmplxType == PlotCanvas::Phase)
                {
                    return std::arg( line[ m_xIndizes[n] ] );
                }
                else //if (m_pInternalData->m_cmplxType == PlotCanvas::Abs)
                {
                    return std::abs( line[ m_xIndizes[n] ] );
                }
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
double DataObjRasterData::value2_yinv(int m, int n) const
{
    if(m_validData)
    {
        m = m_rasteredLines - m - 1; //invert y-coordinate

        switch(m_dataObj.getType())
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
        case ito::tComplex64:
            {
                ito::complex64 *line = (ito::complex64*)m_rasteredLinePtr[m];
                if(!line) return std::numeric_limits<double>::signaling_NaN();
                ito::complex64 i = line[ m_xIndizes[n] ];

                if (m_pInternalData->m_cmplxType == PlotCanvas::Real)
                {
                    return line[ m_xIndizes[n] ].real();
                }
                if (m_pInternalData->m_cmplxType == PlotCanvas::Imag)
                {
                    return line[ m_xIndizes[n] ].imag();
                }
                if (m_pInternalData->m_cmplxType == PlotCanvas::Phase)
                {
                    return std::arg( line[ m_xIndizes[n] ] );
                }
                else //if (m_pInternalData->m_cmplxType == PlotCanvas::Abs)
                {
                    return std::abs( line[ m_xIndizes[n] ] );
                }
            }
        case ito::tComplex128:
            {
                ito::complex128 *line = (ito::complex128*)m_rasteredLinePtr[m];
                if(!line) return std::numeric_limits<double>::signaling_NaN();
                ito::complex128 i = line[ m_xIndizes[n] ];
                
                if (m_pInternalData->m_cmplxType == PlotCanvas::Real)
                {
                    return line[ m_xIndizes[n] ].real();
                }
                if (m_pInternalData->m_cmplxType == PlotCanvas::Imag)
                {
                    return line[ m_xIndizes[n] ].imag();
                }
                if (m_pInternalData->m_cmplxType == PlotCanvas::Phase)
                {
                    return std::arg( line[ m_xIndizes[n] ] );
                }
                else //if (m_pInternalData->m_cmplxType == PlotCanvas::Abs)
                {
                    return std::abs( line[ m_xIndizes[n] ] );
                }
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
    //timer1.restart();    
    //qDebug() << "framerate: " << (1000.0) / ((float)timer2.elapsed());
    //timer2.restart();

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

        /*if (m_pInternalData->m_yaxisFlipped)
        {
            std::swap(top,bottom);
            ySteps *= -1;
        }*/

        int j;

        if(m_dataObj.getDims() > 1)
        {
            //m_plane = (cv::Mat*)m_dataObj->get_mdata()[ m_dataObj->seekMat( m_D.m_planeIdx ) ];
            m_rasteredLines = raster.height();
            int nrOfCols = raster.width();
        
            m_rasteredLinePtr = new uchar*[m_rasteredLines];
            for(int i = 0 ; i < m_rasteredLines ; i++)
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
            //m_validData = true; // moved here by Wolfram Lyda
        }
        else
        {
            m_rasteredLines = 0;
        }

        m_validData = true;
    }
}

//----------------------------------------------------------------------------------------------------------------------------------
void DataObjRasterData::discardRaster()
{
    //qDebug() << "rendering time" << timer1.elapsed();
}
