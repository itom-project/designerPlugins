/* ********************************************************************
   itom measurement system
   URL: http://www.uni-stuttgart.de/ito
   Copyright (C) 2020, Institut fuer Technische Optik (ITO), 
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

#include "dataObjRasterData.h"
#include "common/typeDefs.h"

#include "DataObject/dataObjectFuncs.h"
#include "itomQwtPlotEnums.h"
#include <qdebug.h>

double DataObjRasterData::quietNaN = std::numeric_limits<double>::quiet_NaN();
QRgb DataObjRasterData::transparentColor = 0x00ffffff;

//----------------------------------------------------------------------------------------------------------------------------------
DataObjRasterData::DataObjRasterData(const PlotCanvas::InternalData *m_internalData,
                                     const bool isOverlayData /*= false*/) :
    QwtRasterData(),
    m_validData(false),
    m_rasteredLinePtr(NULL),
    m_xIndizes(NULL),
    m_plane(NULL),
    m_hashGenerator(QCryptographicHash::Md5),
    m_dataObjPlane(NULL),
    m_pInternalData(m_internalData),
    m_isOverlayData(isOverlayData)
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
void DataObjRasterData::setInterval(Qt::Axis axis, const QwtInterval& interval)
{
    if (axis >= 0 && axis <= 2)
    {
        m_intervals[axis] = interval;
    }
}

//----------------------------------------------------------------------------------------------------------------------------------
QwtInterval DataObjRasterData::interval(Qt::Axis axis) const
{
    if (axis >= 0 && axis <= 2)
        return m_intervals[axis];

    return QwtInterval();
}

//----------------------------------------------------------------------------------------------------------------------------------
/*
dataHash only concerns fundamental items like the the number of dimensions or the pointer address to the real data
apearanceHash contains things that might require a redraw of the data (however the current zoom... can be unchanged)
*/
void DataObjRasterData::calcHash(const ito::DataObject *dObj, QByteArray &dataHash, QByteArray &appearanceHash)
{
    if (dObj == NULL) dObj = &m_dataObj;

    if(dObj->getDims() < 2)
    {
        appearanceHash = dataHash = m_hashGenerator.hash("", QCryptographicHash::Md5);
    }
    else
    {
        QByteArray ba;
        QByteArray ba2;

        int dims = dObj->getDims();
        ba.append( dims );
        ba2.append( m_pInternalData->m_cmplxType );
        ba2.append( m_pInternalData->m_yaxisFlipped );
        ba2.append( (char)m_D.m_planeIdx );
        


        if( dims > 0 )
        {
            cv::Mat *m = (cv::Mat*)dObj->get_mdata()[ dObj->seekMat(0) ];
            uchar* d = m->data;
            ba.append( QByteArray( (const char*)&d, (sizeof(int)/sizeof(char)))); //address to data of first plane

            ba.append( QByteArray().setNum( m->size[0] ));
        }

        dataHash = m_hashGenerator.hash(ba, QCryptographicHash::Md5);
        appearanceHash = m_hashGenerator.hash(ba2, QCryptographicHash::Md5);
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
/*
returns 0 if nothing changed, 1 if only the appearance changed and 3 if data and appearance changed
*/
ito::uint8 DataObjRasterData::updateDataObject(const ito::DataObject *dataObj, int planeIdx /*= -1*/)
{
    //the base idea behind simple pointer copying (instead of shallow copies or shared pointer)
    // is that AbstractDObjFigure always keeps shallow copies of all data objects and therefore is 
    // responsible that no dataObject is deleted when it is still in use by any object of this entire plot plugin.
    
    ito::uint8 newHash = PlotCanvas::changeNo;
    bool dataObjPlaneWasShallow = (&m_dataObj != m_dataObjPlane);

    if (dataObj)
    {
        int d = dataObj->getDims();

            m_D.m_yScaling = d > 1 ? dataObj->getAxisScale(d - 2) : 1.0;
            m_D.m_xScaling = d > 1 ? dataObj->getAxisScale(d - 1) : 1.0;
            m_D.m_yOffset = d > 1 ? dataObj->getAxisOffset(d - 2) : 0.0;
            m_D.m_xOffset = d > 1 ? dataObj->getAxisOffset(d - 1) : 0.0;
            m_D.m_ySize = d > 1 ? dataObj->getSize(d - 2) : 0;
            m_D.m_xSize = d > 1 ? dataObj->getSize(d - 1) : 0;
            m_D.m_dataPtr = NULL; //dataObj->get_mdata();

            if (planeIdx >= 0 && m_D.m_planeIdx != planeIdx)
            {
                deleteCache();
                m_D.m_planeIdx = planeIdx;
            }

            QByteArray dataHash;
            QByteArray appearanceHash;
            calcHash(dataObj, dataHash, appearanceHash);

            if (m_appearanceHash != appearanceHash || m_dataHash != dataHash)
            {
                newHash |= PlotCanvas::changeAppearance;

                if (planeIdx == -1)
                {
                    if ((m_dataHash != dataHash) || (m_D.m_planeIdx < 0))
                    {
                        m_D.m_planeIdx = 0;
                    }
                }
            }

            if (m_dataHash != dataHash)
            {
                newHash |= PlotCanvas::changeData;
            }

            if (m_appearanceHash != appearanceHash || m_dataHash != dataHash)
            {
                m_dataHash = dataHash;
                m_appearanceHash = appearanceHash;

                deleteCache();

                m_dataObj = *dataObj;
                m_plane = m_dataObj.getDims() > 1 ? m_dataObj.get_mdata()[m_dataObj.seekMat((int)m_D.m_planeIdx)] : NULL;

                if (m_dataObjPlane && dataObjPlaneWasShallow) //m_dataObjPlane was a shallow copy -> delete it
                {
                    delete m_dataObjPlane;
                    m_dataObjPlane = NULL;
                }

                if (m_dataObj.getDims() > 2)
                {
                    int sizes[2] = { m_D.m_ySize, m_D.m_xSize };
                    m_dataObjPlane = new ito::DataObject(2, sizes, m_dataObj.getType(), m_plane, 1);
                    m_dataObj.copyTagMapTo(*m_dataObjPlane);

                    bool v;
                    for (int i = 0; i < 2; ++i)
                    {
                        m_dataObjPlane->setAxisDescription(i, m_dataObj.getAxisDescription(d - (2 - i), v));
                        m_dataObjPlane->setAxisScale(i, m_dataObj.getAxisScale(d - (2 - i)));
                        m_dataObjPlane->setAxisOffset(i, m_dataObj.getAxisOffset(d - (2 - i)));
                    }

                    m_dataObjPlane->setValueDescription(m_dataObj.getValueDescription());
                    m_dataObjPlane->setValueUnit(m_dataObj.getValueUnit());

                    m_dataObj.copyAxisTagsTo(*m_dataObjPlane);
                }
                else
                {
                    m_dataObjPlane = &m_dataObj;
                }

                if (!m_isOverlayData)
                {
                    m_pInternalData->m_selectedOutputParameters["sourceout"]->setVal<void*>((void*)&m_dataObj);
                    m_pInternalData->m_selectedOutputParameters["displayed"]->setVal<void*>((void*)m_dataObjPlane);
                }

                //Definition: Scale-Coordinate of dataObject =  ( px-Coordinate - Offset)* Scale
                if (m_D.m_xScaling >= 0.0)
                {
                    setInterval(Qt::XAxis, QwtInterval(pxToScaleCoords(0, m_D.m_xOffset, m_D.m_xScaling), pxToScaleCoords(m_D.m_xSize - 1, m_D.m_xOffset, m_D.m_xScaling)));
                }
                else
                {
                    setInterval(Qt::XAxis, QwtInterval(pxToScaleCoords(m_D.m_xSize - 1, m_D.m_xOffset, m_D.m_xScaling), pxToScaleCoords(0, m_D.m_xOffset, m_D.m_xScaling)));
                }

                if (m_D.m_yScaling >= 0.0)
                {
                    setInterval(Qt::YAxis, QwtInterval(pxToScaleCoords(0, m_D.m_yOffset, m_D.m_yScaling), pxToScaleCoords(m_D.m_ySize - 1, m_D.m_yOffset, m_D.m_yScaling)));
                }
                else
                {
                    setInterval(Qt::YAxis, QwtInterval(pxToScaleCoords(m_D.m_ySize - 1, m_D.m_yOffset, m_D.m_yScaling), pxToScaleCoords(0, m_D.m_yOffset, m_D.m_yScaling)));
                }
            }

            ito::float64 min, max;
            ito::uint32 firstMin[3];
            ito::uint32 firstMax[3];


            if ((m_pInternalData->m_valueScaleAuto && !m_isOverlayData) || (m_pInternalData->m_overlayScaleAuto && m_isOverlayData))
            {
                ito::dObjHelper::minMaxValue(m_dataObjPlane, min, firstMin, max, firstMax, true, m_pInternalData->m_cmplxType);

                if (min == std::numeric_limits<ito::float64>::max()) min = -10.0;
                if (max == -std::numeric_limits<ito::float64>::max()) max = 10.0;

                if ((max - min) < std::numeric_limits<double>::epsilon()) //the data object only contains the same value, then make the min-max range a little bit bigger in order to ensure a nice colorbar
                {
                    switch (m_pInternalData->m_dataType)
                    {
                    case ito::tUInt8:
                    case ito::tInt8:
                    case ito::tUInt16:
                    case ito::tInt16:
                    case ito::tUInt32:
                    case ito::tInt32:
                    case ito::tRGBA32:
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
                        break;
                    }
                }

                QwtInterval zInterval = interval(Qt::ZAxis);
                setInterval(Qt::ZAxis, QwtInterval(min, max));

                if (qFuzzyCompare(zInterval.minValue(), min) == false || \
                    qFuzzyCompare(zInterval.maxValue(), max) == false)
                {
                    newHash |= PlotCanvas::changeAppearance;
                }

            }
            else
            {
                if (m_isOverlayData)
                {
                    setInterval(Qt::ZAxis, QwtInterval(m_pInternalData->m_overlayMin, m_pInternalData->m_overlayMax));
                }
                else
                {
                    setInterval(Qt::ZAxis, QwtInterval(m_pInternalData->m_valueMin, m_pInternalData->m_valueMax));
                }
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
        if(!m_isOverlayData)
        {
            m_pInternalData->m_selectedOutputParameters["sourceout"]->setVal<void*>(NULL);
            m_pInternalData->m_selectedOutputParameters["output"]->setVal<void*>(NULL);
        }
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
        m_dataHash = QByteArray();
        m_appearanceHash = QByteArray();
        newHash = PlotCanvas::changeAppearance | PlotCanvas::changeData;

        setInterval(Qt::XAxis, QwtInterval() );
        setInterval(Qt::YAxis, QwtInterval() );
        setInterval(Qt::ZAxis, QwtInterval() );
    }
    
    
    return newHash;
}
//----------------------------------------------------------------------------------------------------------------------------------
bool DataObjRasterData::pointValid(const QPointF &point) const
{
    return interval(Qt::XAxis).contains( point.x() ) && interval(Qt::YAxis).contains( point.y() );
}

//----------------------------------------------------------------------------------------------------------------------------------
DataObjRasterData::RasterDataType DataObjRasterData::getTypeFlag() const 
{ 
    int type = m_dataObj.getType();
    if(type == ito::tRGBA32)
    {
        switch (m_pInternalData->m_dataChannel)
        {
            case ItomQwtPlotEnums::ChannelAuto:
            case ItomQwtPlotEnums::ChannelRGBA:
                return tRGB;
            default:
                return tInteger;
        }
    }
    else if (type == ito::tFloat32 || type == ito::tFloat64 || type == ito::tComplex64 || type == ito::tComplex128) 
    {
        return tFloatOrComplex;
    }

    return tInteger; 
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

                    int n = qRound(m_dataObj.getPhysToPix(d - 1, x, inside1));
                    int m = qRound(m_dataObj.getPhysToPix(d - 2, y, inside2));

                    if (inside1 && inside2)
                    {
                        switch (m_dataObj.getType())
                        {
                        case ito::tInt8:
                        {
                            return m_plane->at<ito::int8>(m, n);
                        }
                        case ito::tUInt8:
                        {
                            return m_plane->at<ito::uint8>(m, n);
                        }
                        case ito::tInt16:
                        {
                            return m_plane->at<ito::int16>(m, n);
                        }
                        case ito::tUInt16:
                        {
                            return m_plane->at<ito::uint16>(m, n);
                        }
                        case ito::tInt32:
                        {
                            return m_plane->at<ito::int32>(m, n);
                        }
                        case ito::tUInt32:
                        {
                            return m_plane->at<ito::uint32>(m, n);
                        }
                        case ito::tFloat32:
                        {
                            return m_plane->at<ito::float32>(m, n);
                        }
                        case ito::tFloat64:
                        {
                            return m_plane->at<ito::float64>(m, n);
                        }
                        case ito::tComplex64:
                        {
                            if (m_pInternalData->m_cmplxType == ItomQwtPlotEnums::CmplxReal)
                            {
                                return m_plane->at<ito::complex64>(m, n).real();
                            }
                            if (m_pInternalData->m_cmplxType == ItomQwtPlotEnums::CmplxImag)
                            {
                                return m_plane->at<ito::complex64>(m, n).imag();
                            }
                            if (m_pInternalData->m_cmplxType == ItomQwtPlotEnums::CmplxArg)
                            {
                                return std::arg(m_plane->at<ito::complex64>(m, n));
                            }
                            else //if (m_pInternalData->m_cmplxType == PlotCanvas::CmplxAbs)
                            {
                                return std::abs(m_plane->at<ito::complex64>(m, n));
                            }
                        }
                        case ito::tComplex128:
                        {
                            if (m_pInternalData->m_cmplxType == ItomQwtPlotEnums::CmplxReal)
                            {
                                return m_plane->at<ito::complex128>(m, n).real();
                            }
                            if (m_pInternalData->m_cmplxType == ItomQwtPlotEnums::CmplxImag)
                            {
                                return m_plane->at<ito::complex128>(m, n).imag();
                            }
                            if (m_pInternalData->m_cmplxType == ItomQwtPlotEnums::CmplxArg)
                            {
                                return std::arg(m_plane->at<ito::complex128>(m, n));
                            }
                            else //if (m_pInternalData->m_cmplxType == PlotCanvas::CmplxAbs)
                            {
                                return std::abs(m_plane->at<ito::complex128>(m, n));
                            }
                        }
                        case ito::tRGBA32:
                        {

                            if (m_pInternalData->m_dataChannel == ItomQwtPlotEnums::ChannelAuto) //channelAuto
                            {
                                return m_plane->at<ito::Rgba32>(m, n).gray();
                            }
                            else if (m_pInternalData->m_dataChannel == ItomQwtPlotEnums::ChannelRGBA)//rgba
                            {
                                return m_plane->at<ito::Rgba32>(m, n).gray();
                            }
                            else if (m_pInternalData->m_dataChannel == ItomQwtPlotEnums::ChannelGray)//gray
                            {
                                return m_plane->at<ito::Rgba32>(m, n).gray();
                            }
                            else if (m_pInternalData->m_dataChannel == ItomQwtPlotEnums::ChannelRed)//red
                            {
                                return m_plane->at<ito::Rgba32>(m, n).red();
                            }
                            else if (m_pInternalData->m_dataChannel == ItomQwtPlotEnums::ChannelGreen)//green
                            {
                                return m_plane->at<ito::Rgba32>(m, n).green();
                            }
                            else if (m_pInternalData->m_dataChannel == ItomQwtPlotEnums::ChannelBlue)//blue
                            {
                                return m_plane->at<ito::Rgba32>(m, n).blue();
                            }
                            else if (m_pInternalData->m_dataChannel == ItomQwtPlotEnums::ChannelAlpha)//alpha
                            {
                                return m_plane->at<ito::Rgba32>(m, n).alpha();
                            }

                        }
                        default:
                            return quietNaN;
                        }
                        
                }
               
        }
    }
    return quietNaN;
}

//----------------------------------------------------------------------------------------------------------------------------------
QRgb DataObjRasterData::value_rgb(double x, double y) const
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
                case ito::tRGBA32:
                    {
                        return m_plane->at<ito::Rgba32>(m,n).argb();
                    }
                default:
                    return transparentColor;
                }
            }
        }
    }
    return quietNaN;
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
                if(!line) return quietNaN;
                return line[ m_xIndizes[n] ];
            }
        case ito::tUInt8:
            {
                ito::uint8 *line = (ito::uint8*)m_rasteredLinePtr[m];
                if(!line) return quietNaN;
                return line[ m_xIndizes[n] ];
            }
        case ito::tInt16:
            {
                ito::int16 *line = (ito::int16*)m_rasteredLinePtr[m];
                if(!line) return quietNaN;
                return line[ m_xIndizes[n] ];
            }
        case ito::tUInt16:
            {
                ito::uint16 *line = (ito::uint16*)m_rasteredLinePtr[m];
                if(!line) return quietNaN;
                return line[ m_xIndizes[n] ];
            }
        case ito::tInt32:
            {
                ito::int32 *line = (ito::int32*)m_rasteredLinePtr[m];
                if(!line) return quietNaN;
                return line[ m_xIndizes[n] ];
            }
        case ito::tUInt32:
            {
                ito::uint32 *line = (ito::uint32*)m_rasteredLinePtr[m];
                if(!line) return quietNaN;
                return line[ m_xIndizes[n] ];
            }
        case ito::tFloat32:
            {
                ito::float32 *line = (ito::float32*)m_rasteredLinePtr[m];
                if(!line) return quietNaN;
                return line[ m_xIndizes[n] ];
            }
        case ito::tFloat64:
            {
                ito::float64 *line = (ito::float64*)m_rasteredLinePtr[m];
                if(!line) return quietNaN;
                return line[ m_xIndizes[n] ];
            }
        case ito::tComplex64:
            {
                ito::complex64 *line = (ito::complex64*)m_rasteredLinePtr[m];
                if(!line) return quietNaN;
                ito::complex64 i = line[ m_xIndizes[n] ];

                if (m_pInternalData->m_cmplxType == ItomQwtPlotEnums::CmplxReal)
                {
                    return line[ m_xIndizes[n] ].real();
                }
                if (m_pInternalData->m_cmplxType == ItomQwtPlotEnums::CmplxImag)
                {
                    return line[ m_xIndizes[n] ].imag();
                }
                if (m_pInternalData->m_cmplxType == ItomQwtPlotEnums::CmplxArg)
                {
                    return std::arg( line[ m_xIndizes[n] ] );
                }
                else //if (m_pInternalData->m_cmplxType == PlotCanvas::CmplxAbs)
                {
                    return std::abs( line[ m_xIndizes[n] ] );
                }
            }
        case ito::tComplex128:
            {
                ito::complex128 *line = (ito::complex128*)m_rasteredLinePtr[m];
                if(!line) return quietNaN;
                ito::complex128 i = line[ m_xIndizes[n] ];
                
                if (m_pInternalData->m_cmplxType == ItomQwtPlotEnums::CmplxReal)
                {
                    return line[ m_xIndizes[n] ].real();
                }
                if (m_pInternalData->m_cmplxType == ItomQwtPlotEnums::CmplxImag)
                {
                    return line[ m_xIndizes[n] ].imag();
                }
                if (m_pInternalData->m_cmplxType == ItomQwtPlotEnums::CmplxArg)
                {
                    return std::arg( line[ m_xIndizes[n] ] );
                }
                else //if (m_pInternalData->m_cmplxType == PlotCanvas::CmplxAbs)
                {
                    return std::abs( line[ m_xIndizes[n] ] );
                }
            }
        case ito::tRGBA32:
            {
                ito::Rgba32 *line = (ito::Rgba32*)m_rasteredLinePtr[m];
                if(!line) return quietNaN;

                switch (m_pInternalData->m_dataChannel)
                {
                case ItomQwtPlotEnums::ChannelAuto:
                case ItomQwtPlotEnums::ChannelRGBA:
                case ItomQwtPlotEnums::ChannelGray:
                    {
                        return line[ m_xIndizes[n] ].gray();
                    }
                    break;
                case ItomQwtPlotEnums::ChannelRed:
                    {
                        return line[ m_xIndizes[n] ].r;
                    }
                    break;
                case ItomQwtPlotEnums::ChannelGreen:
                    {
                        return line[ m_xIndizes[n] ].g;
                    }
                    break;
                case ItomQwtPlotEnums::ChannelBlue:
                    {
                        return line[ m_xIndizes[n] ].b;
                    }
                    break;
                case ItomQwtPlotEnums::ChannelAlpha:
                    {
                        return line[ m_xIndizes[n] ].a;
                    }
                    break;
                }
            }
        default:
            return quietNaN;
        }
    }
    else
    {
        return quietNaN;
    }
}

//----------------------------------------------------------------------------------------------------------------------------------
double DataObjRasterData::value2_yinv(int m, int n) const
{
    if(m_validData)
    {
        m = m_rasteredLines - m - 1; //invert y-coordinate
        switch (m_dataObj.getType())
        {
        case ito::tInt8:
        {
            ito::int8 *line = (ito::int8*)m_rasteredLinePtr[m];
            if (!line) return quietNaN;
            return line[m_xIndizes[n]];
        }
        case ito::tUInt8:
        {
            ito::uint8 *line = (ito::uint8*)m_rasteredLinePtr[m];
            if (!line) return quietNaN;
            return line[m_xIndizes[n]];
        }
        case ito::tInt16:
        {
            ito::int16 *line = (ito::int16*)m_rasteredLinePtr[m];
            if (!line) return quietNaN;
            return line[m_xIndizes[n]];
        }
        case ito::tUInt16:
        {
            ito::uint16 *line = (ito::uint16*)m_rasteredLinePtr[m];
            if (!line) return quietNaN;
            return line[m_xIndizes[n]];
        }
        case ito::tInt32:
        {
            ito::int32 *line = (ito::int32*)m_rasteredLinePtr[m];
            if (!line) return quietNaN;
            return line[m_xIndizes[n]];
        }
        case ito::tUInt32:
        {
            ito::uint32 *line = (ito::uint32*)m_rasteredLinePtr[m];
            if (!line) return quietNaN;
            return line[m_xIndizes[n]];
        }
        case ito::tFloat32:
        {
            ito::float32 *line = (ito::float32*)m_rasteredLinePtr[m];
            if (!line) return quietNaN;
            return line[m_xIndizes[n]];
        }
        case ito::tFloat64:
        {
            ito::float64 *line = (ito::float64*)m_rasteredLinePtr[m];
            if (!line) return quietNaN;
            return line[m_xIndizes[n]];
        }
        case ito::tComplex64:
        {
            ito::complex64 *line = (ito::complex64*)m_rasteredLinePtr[m];
            if (!line) return quietNaN;
            ito::complex64 i = line[m_xIndizes[n]];

            if (m_pInternalData->m_cmplxType == ItomQwtPlotEnums::CmplxReal)
            {
                return line[m_xIndizes[n]].real();
            }
            if (m_pInternalData->m_cmplxType == ItomQwtPlotEnums::CmplxImag)
            {
                return line[m_xIndizes[n]].imag();
            }
            if (m_pInternalData->m_cmplxType == ItomQwtPlotEnums::CmplxArg)
            {
                return std::arg(line[m_xIndizes[n]]);
            }
            else //if (m_pInternalData->m_cmplxType == PlotCanvas::CmplxAbs)
            {
                return std::abs(line[m_xIndizes[n]]);
            }
        }
        case ito::tComplex128:
        {
            ito::complex128 *line = (ito::complex128*)m_rasteredLinePtr[m];
            if (!line) return quietNaN;
            ito::complex128 i = line[m_xIndizes[n]];

            if (m_pInternalData->m_cmplxType == ItomQwtPlotEnums::CmplxReal)
            {
                return line[m_xIndizes[n]].real();
            }
            if (m_pInternalData->m_cmplxType == ItomQwtPlotEnums::CmplxImag)
            {
                return line[m_xIndizes[n]].imag();
            }
            if (m_pInternalData->m_cmplxType == ItomQwtPlotEnums::CmplxArg)
            {
                return std::arg(line[m_xIndizes[n]]);
            }
            else //if (m_pInternalData->m_cmplxType == PlotCanvas::CmplxAbs)
            {
                return std::abs(line[m_xIndizes[n]]);
            }
        }
        case ito::tRGBA32:
        {
            ito::Rgba32 *line = (ito::Rgba32*)m_rasteredLinePtr[m];
            if (!line) return quietNaN;

            switch (m_pInternalData->m_dataChannel)
            {
            case ItomQwtPlotEnums::ChannelAuto:
            case ItomQwtPlotEnums::ChannelRGBA:
            case ItomQwtPlotEnums::ChannelGray:
            {
                return line[m_xIndizes[n]].gray();
            }
            break;
            case ItomQwtPlotEnums::ChannelRed:
            {
                return line[m_xIndizes[n]].r;
            }
            break;
            case ItomQwtPlotEnums::ChannelGreen:
            {
                return line[m_xIndizes[n]].g;
            }
            break;
            case ItomQwtPlotEnums::ChannelBlue:
            {
                return line[m_xIndizes[n]].b;
            }
            break;
            case ItomQwtPlotEnums::ChannelAlpha:
            {
                return line[m_xIndizes[n]].a;
            }
            break;
            }
        }
        default:
            return quietNaN;
        }
        
    }
    else
    {
        return quietNaN;
    }
}


//----------------------------------------------------------------------------------------------------------------------------------
QRgb DataObjRasterData::value2_rgb(int m, int n) const
{
    if(m_validData)
    {
        switch(m_dataObj.getType())
        {
        case ito::tRGBA32:
            {
                ito::Rgba32 *line = (ito::Rgba32*)m_rasteredLinePtr[m];
                if(!line) return transparentColor;

                switch (m_pInternalData->m_dataChannel)
                {
                case ItomQwtPlotEnums::ChannelAuto:
                case ItomQwtPlotEnums::ChannelRGBA:
                    return line[ m_xIndizes[n] ].argb();
                    break;
                case ItomQwtPlotEnums::ChannelGray:
                    {
                        int gray = qRound(line[m_xIndizes[n]].gray());
                        return qRgb(gray, gray, gray);
                    }
                    break;
                case ItomQwtPlotEnums::ChannelRed:
                    {
                        ito::uint8 c = line[m_xIndizes[n]].r;
                        return qRgb(c, c, c);
                    }
                    break;
                case ItomQwtPlotEnums::ChannelGreen:
                    {
                        ito::uint8 c = line[m_xIndizes[n]].g;
                        return qRgb(c, c, c);
                    }
                    break;
                case ItomQwtPlotEnums::ChannelBlue:
                    {
                        ito::uint8 c = line[m_xIndizes[n]].b;
                        return qRgb(c, c, c);
                    }
                    break;
                case ItomQwtPlotEnums::ChannelAlpha:
                    {
                        ito::uint8 c = line[m_xIndizes[n]].a;
                        return qRgb(c, c, c);
                    }
                    break;
                }
            }
        default:
            return transparentColor;
        }
    }
    else
    {
        return transparentColor;
    }
}

//----------------------------------------------------------------------------------------------------------------------------------
QRgb DataObjRasterData::value2_yinv_rgb(int m, int n) const
{
    if(m_validData)
    {
        m = m_rasteredLines - m - 1; //invert y-coordinate

        switch(m_dataObj.getType())
        {
        case ito::tRGBA32:
            {
                ito::Rgba32 *line = (ito::Rgba32*)m_rasteredLinePtr[m];
                if(!line) return transparentColor;
                
                switch (m_pInternalData->m_dataChannel)
                {
                case ItomQwtPlotEnums::ChannelAuto:
                case ItomQwtPlotEnums::ChannelRGBA:
                    return line[ m_xIndizes[n] ].argb();
                    break;
                case ItomQwtPlotEnums::ChannelGray:
                    {
                        int gray = qRound(line[m_xIndizes[n]].gray());
                        return qRgb(gray, gray, gray);
                    }
                    break;
                case ItomQwtPlotEnums::ChannelRed:
                    {
                        ito::uint8 c = line[m_xIndizes[n]].r;
                        return qRgb(c, c, c);
                    }
                    break;
                case ItomQwtPlotEnums::ChannelGreen:
                    {
                        ito::uint8 c = line[m_xIndizes[n]].g;
                        return qRgb(c, c, c);
                    }
                    break;
                case ItomQwtPlotEnums::ChannelBlue:
                    {
                        ito::uint8 c = line[m_xIndizes[n]].b;
                        return qRgb(c, c, c);
                    }
                    break;
                case ItomQwtPlotEnums::ChannelAlpha:
                    {
                        ito::uint8 c = line[m_xIndizes[n]].a;
                        return qRgb(c, c, c);
                    }
                    break;
                }
            }
        default:
            return transparentColor;
        }
    }
    else
    {
        return transparentColor;
    }
}

//----------------------------------------------------------------------------------------------------------------------------------
void DataObjRasterData::getPlaneScaleAndOffset(double &scaleY, double &scaleX, double &offsetY, double &offsetX) const
{
    int dims = m_dataObj.getDims();

    if (dims >= 2)
    {
        scaleX = m_dataObj.getAxisScale(dims - 1);
        scaleY = m_dataObj.getAxisScale(dims - 2);
        offsetX = m_dataObj.getAxisOffset(dims - 1);
        offsetY = m_dataObj.getAxisOffset(dims - 2);
    }
    else
    {
        scaleY = scaleX = 1.0;
        offsetY = offsetX = 1.0;
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
            deleteCache();
        }
    }

    if(m_rasteredLinePtr == NULL) //rebuild cache
    {
        m_lastRasteredArea = area;
        m_lastRasteredRaster = raster;

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

                for (int i = 0; i < m_rasteredLines; i++)
                {
                    j = qRound(top + ySteps * i);

                    if (j >= 0 && j < m_D.m_ySize)
                    {
                        m_rasteredLinePtr[i] = m_plane->ptr(j);
                    }
                    else
                    {
                        m_rasteredLinePtr[i] = NULL;
                    }
                }

                m_xIndizes = new int[raster.width()];
                for (int i = 0; i < nrOfCols; i++)
                {
                    j = qRound(left + xSteps * i);

                    if (j >= 0 && j < m_D.m_xSize)
                    {
                        m_xIndizes[i] = j;
                    }
                    else
                    {
                        m_xIndizes[i] = -1;
                    }
                }
                
            /*
            todo: delete if pointer aproach fails
                else if (m_D.m_dir == dirY)
                {
                    //obtain start coordinate of volume cut
                    int startY = m_D.m_startPx.y();
                    int startX = m_D.m_startPx.x();
                    m_rasteredLines = raster.height();
                    int nrOfCols = raster.width();
                    m_rasteredLinePtr = new uchar*[m_rasteredLines];
                    int i;
                    
                    int lineOffset = (m_dataObj.elemSize()*startX);
                    for (i=0; i < m_rasteredLines; i++)
                    {
                        j = qRound(top + ySteps * i); //px
                        if (j >= 0 && j < m_D.m_ySize)
                        {
                            m_rasteredLinePtr[i] = m_dataObj.rowPtr(j, startY + left)+lineOffset;
                        }
                        else
                        {
                            m_rasteredLinePtr[i] = NULL;
                        }
                    }

                    m_xIndizes = new int[raster.width()];
                    int rowStep = m_dataObj.getCvPlaneMat(0)->step1(0); //row step in type of obj
                    for (i = 0; i < nrOfCols; i++)
                    {
                        j = qRound(left + xSteps*i);
                        if (j >= 0 && j < m_D.m_ySize)
                        {
                            //m_dataObj.getCvPlaneMat(0)->step*j
                            m_xIndizes[i] = j*rowStep;
                        }
                        else
                        {
                            m_xIndizes[i] = NULL;
                        }
                    }

                }
                */
        }
        else
        {
            m_rasteredLines = 0;
        }

        m_validData = true;
    }
}
//----------------------------------------------------------------------------------------------------------------------------------
QSharedPointer<ito::DataObject> DataObjRasterData::rasterToObject(const QwtInterval &xInterval, const QwtInterval &yInterval, const bool copyDisplayedAsComplex, const int cmplxState)
{
//    const QwtInterval xInterval = interval( Qt::XAxis );
//    const QwtInterval yInterval = interval( Qt::YAxis );

    if(!m_dataObjPlane || m_dataObjPlane->getDims() < 2)
    {
        return QSharedPointer<ito::DataObject>();
    }

    double yd0, yd1, xd0, xd1, temp;
    int yi0, yi1, xi0, xi1;
    int dims = m_dataObjPlane->getDims();
    int width = m_dataObjPlane->getSize(dims - 1);
    int height = m_dataObjPlane->getSize(dims - 2);

    bool validY0, validX0, validY1, validX1;

    m_dataObjPlane->getPhysToPix2D(yInterval.minValue(), yd0, validY0, xInterval.minValue(), xd0, validX0);

    m_dataObjPlane->getPhysToPix2D(yInterval.maxValue(), yd1, validY1, xInterval.maxValue(), xd1, validX1);

    temp = std::max(yd0, yd1);
    yd0 = std::min(yd0, yd1);
    yd1 = temp;

    temp = std::max(xd0, xd1);
    xd0 = std::min(xd0, xd1);
    xd1 = temp;

    if (!validX0 && !validX1)
    {
        if (!xInterval.normalized().contains(m_dataObjPlane->getPixToPhys(dims - 1, 0)))
        {
            return QSharedPointer<ito::DataObject>(new ito::DataObject());
        }
    }
    else if (!validY0 && !validY1)
    {
        if (!yInterval.normalized().contains(m_dataObjPlane->getPixToPhys(dims - 2, 0)))
        {
            return QSharedPointer<ito::DataObject>(new ito::DataObject());
        }
    }

    if (!validY1)
    {
        yd1 = height - 1;
    }
    if (!validX1)
    {
        xd1 = width - 1;
    }

    if (!validY0)
    {
        yd0 = 0;
    }
    if (!validX0)
    {
        xd0 = 0;
    }

    yi0 = yd0 < 0 ? 0.0 : (int)(yd0);
    yi1 = yd1 < 0 ? 0.0 : (int)(yd1 + 0.9);

    xi0 = xd0 < 0 ? 0.0 : (int)(xd0);
    xi1 = xd1 < 0 ? 0.0 : (int)(xd1 + 0.9);

    xi0 = xi0 > width -1 ? width -1 : xi0;
    xi1 = xi1 > width -1 ? width -1 : xi1;

    yi0 = yi0 > height -1 ? height -1 : yi0;
    yi1 = yi1 > height -1 ? height -1 : yi1;

    ito::Range* curRange = new ito::Range[dims];

    for(int i = 0; i < dims - 2; i++)
    {
        curRange[i] = ito::Range(0, 0);
    }

    //if(dims > 2) curRange[dims - 3] = ito::Range(getCurrentPlane(), getCurrentPlane());
    curRange[dims - 2] = ito::Range(yi0, yi1 + 1);
    curRange[dims - 1] = ito::Range(xi0, xi1 + 1);

    ito::DataObject dataObjectOut;
    int type = m_dataObjPlane->getType();
    if (copyDisplayedAsComplex == false)
    {
        m_dataObjPlane->at(curRange).copyTo(dataObjectOut);
    }
    else if(type == ito::tComplex64 || type == ito::tComplex128)
    {
        ito::DataObject temp;

        switch (cmplxState)
        {
        default:
        case ItomQwtPlotEnums::CmplxAbs:
            temp = ito::abs(*(m_dataObjPlane));
            break;
        case ItomQwtPlotEnums::CmplxReal:
            temp = ito::real(*(m_dataObjPlane));
            break;
        case ItomQwtPlotEnums::CmplxImag:
            temp = ito::imag(*(m_dataObjPlane));
            break;
        case ItomQwtPlotEnums::CmplxArg:
            temp = ito::arg(*(m_dataObjPlane));
            break;
        }

        temp.at(curRange).copyTo(dataObjectOut);       
    }
            
    DELETE_AND_SET_NULL_ARRAY(curRange)
    return QSharedPointer<ito::DataObject>(new ito::DataObject(dataObjectOut));
}
//----------------------------------------------------------------------------------------------------------------------------------
QSharedPointer<ito::DataObject> DataObjRasterData::rasterToObject()
{
    if(!m_dataObjPlane || m_dataObjPlane->getDims() < 2)
    {
        return QSharedPointer<ito::DataObject>();
    }
    ito::DataObject dataObjectOut;
    m_dataObjPlane->copyTo(dataObjectOut);
    return QSharedPointer<ito::DataObject>(new ito::DataObject(dataObjectOut));
}

//----------------------------------------------------------------------------------------------------------------------------------
void DataObjRasterData::discardRaster()
{
}

//----------------------------------------------------------------------------------------------------------------------------------
void DataObjRasterData::getMinMaxLoc(double &min, ito::uint32 *minLoc, double &max, ito::uint32 *maxLoc) const
{
    if(!m_dataObjPlane || m_dataObjPlane->getDims() < 2)
    {
        min = std::numeric_limits<double>::quiet_NaN();
        max = min;
    }
    else
    {
        ito::dObjHelper::minMaxValue(m_dataObjPlane, min, minLoc, max, maxLoc, true, m_pInternalData->m_cmplxType);
    }
}
