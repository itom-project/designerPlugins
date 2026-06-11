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

#include "dataObjItem.h"

#include <qwt_scale_map.h>
#include <qwt_color_map.h>
#include <qwt_raster_data.h>

#include <qthread.h>
#include <qfuture.h>
#include <QtConcurrent/qtconcurrentrun.h>

#include "dataObjRasterData.h"


DataObjItem::DataObjItem( const QString &title ):
    QwtPlotSpectrogram( title ),
    m_pContourPalette()
{
    m_counter = 0;
}

//! Destructor
DataObjItem::~DataObjItem()
{
}

QPen DataObjItem::contourPen(double level) const
{
    QList<double> list(contourLevels());
    std::sort(list.begin(), list.end());
    double min = list.first();
    double max = list.last();
    double range = max - min;
    QPen pen(defaultContourPen());

    int idx = qRound(255.0 / range*(level-min));
    ito::uint32 color = m_pContourPalette.colorVector256 [idx];
    ito::uint8 r, g, b;
    r = (ito::uint8)(color >> 16);
    g = (ito::uint8)(color >> 8);
    b = (ito::uint8)(color);

    pen.setColor(qRgb(r, g, b));
    pen.setStyle(Qt::SolidLine);
    return pen;
}

void DataObjItem::setContourPalette(const ito::ItomPalette &palette)
{
    m_pContourPalette = palette;
    QPen pen(defaultContourPen());
    pen.setStyle(Qt::NoPen);
    setDefaultContourPen(pen);

}

/*!
   \brief Render an image from data and color map.

   For each pixel of rect the value is mapped into a color.

  \param xMap X-Scale Map
  \param yMap Y-Scale Map
  \param area Requested area for the image in scale coordinates
  \param imageSize Size of the requested image

   \return A QImage::Format_Indexed8 or QImage::Format_ARGB32 depending
           on the color map.

   \sa QwtRasterData::value(), QwtColorMap::rgb(),
       QwtColorMap::colorIndex()
*/
QImage DataObjItem::renderImage(
    const QwtScaleMap &xMap, const QwtScaleMap &yMap,
    const QRectF &area, const QSize &imageSize ) const
{
    if ( imageSize.isEmpty() || data() == NULL
        || colorMap() == NULL )
    {
        return QImage();
    }

    DataObjRasterData *dObjRasterData = (DataObjRasterData*)(data());

    if (dObjRasterData->getSize().isNull() )
    {
        return QImage();
    }

    const QwtInterval intensityRange = dObjRasterData->interval( Qt::ZAxis );
    if ( !intensityRange.isValid() )
    {
        return QImage();
    }

    const QwtInterval xRange = dObjRasterData->interval( Qt::XAxis );
    if ( !xRange.isValid() )
    {
        return QImage();
    }

    DataObjRasterData::RasterDataType dataTypeFlag = dObjRasterData->getTypeFlag();

    QImage::Format format = ( colorMap()->format() == QwtColorMap::RGB || dataTypeFlag != DataObjRasterData::tInteger )
        ? QImage::Format_ARGB32 : QImage::Format_Indexed8;

    QImage image( imageSize, format );

    if ( format == QImage::Format_Indexed8 ) //colorMap()->format() == QwtColorMap::Indexed )
    {
        image.setColorTable( colorMap()->colorTable256() );
    }

    dObjRasterData->initRaster( area, image.size() );
    int* t =const_cast<int*>(&m_counter); //)++;
    (*t) ++;

#if !defined(QT_NO_QFUTURE)
    uint numThreads = renderThreadCount();

    if ( numThreads <= 0 )
    {
        numThreads = QThread::idealThreadCount();
        if (numThreads > 1)
        {
            numThreads = std::max((uint)1, numThreads - 2); //reduce the maximum number of threads by 2 in order to provide free threads for other jobs
        }
    }

    if ( numThreads <= 0 )
        numThreads = 1;

    const int numRows = imageSize.height() / numThreads;

    QList< QFuture<void> > futures;
    for ( uint i = 0; i < numThreads; i++ )
    {
        QRect tile( 0, i * numRows, image.width(), numRows );
        if ( i == numThreads - 1 )
        {
            tile.setHeight( image.height() - i * numRows );
            renderTile( xMap, yMap, dataTypeFlag, tile, &image );
        }
        else
        {
#if QT_VERSION < QT_VERSION_CHECK(6,0,0)
            futures += QtConcurrent::run(
                this, &DataObjItem::renderTile,
                xMap, yMap, dataTypeFlag, tile, &image );
#else
            futures += QtConcurrent::run(
                &DataObjItem::renderTile, this,
                xMap, yMap, dataTypeFlag, tile, &image);
#endif

        }
    }
    for ( int i = 0; i < futures.size(); i++ )
        futures[i].waitForFinished();

#else // QT_VERSION < 0x040400
    const QRect tile( 0, 0, image.width(), image.height() );
    renderTile( xMap, yMap, dataTypeFlag, tile, &image );
#endif

    dObjRasterData->discardRaster();

    return image;
}

/*!
    \brief Render a tile of an image.

    Rendering in tiles can be used to composite an image in parallel
    threads.

    \param xMap X-Scale Map
    \param yMap Y-Scale Map
    \param tile Geometry of the tile in image coordinates
    \param image Image to be rendered
*/
void DataObjItem::renderTile(
    const QwtScaleMap &xMap, const QwtScaleMap &yMap, const char dataTypeFlag, const QRect &tile, QImage *image ) const
{
    DataObjRasterData *dataObjRasterData = (DataObjRasterData*)data();

    const QwtInterval range = dataObjRasterData->interval( Qt::ZAxis );

    if ( !range.isValid() )
    {
        return;
    }

    if (dataTypeFlag == DataObjRasterData::tRGB)
    {
        if (yMap.isInverting())
        {
            for ( int y = tile.top(); y <= tile.bottom(); y++ )
            {
                QRgb *line = ( QRgb * )image->scanLine( y );
                line += tile.left();

                for ( int x = tile.left(); x <= tile.right(); x++ )
                {
                    *line++ = dataObjRasterData->value2_yinv_rgb( y, x );
                }
            }
        }
        else
        {
            for ( int y = tile.top(); y <= tile.bottom(); y++ )
            {
                QRgb *line = ( QRgb * )image->scanLine( y );
                line += tile.left();

                for ( int x = tile.left(); x <= tile.right(); x++ )
                {
                    *line++ = dataObjRasterData->value2_rgb( y, x );
                }
            }
        }
    }
    else //single-value data types, color map of qwt plot is considered (either index8 or rgb)
    {
        if (colorMap()->format() == QwtColorMap::RGB || dataTypeFlag == DataObjRasterData::tFloatOrComplex)
        {
            if (yMap.isInverting())
            {
                for ( int y = tile.top(); y <= tile.bottom(); y++ )
                {
                    QRgb *line = ( QRgb * )image->scanLine( y );
                    line += tile.left();

                    for ( int x = tile.left(); x <= tile.right(); x++ )
                    {
                        *line++ = colorMap()->rgb( range,
                            dataObjRasterData->value2_yinv( y, x ) );
                    }
                }
            }
            else
            {
                for ( int y = tile.top(); y <= tile.bottom(); y++ )
                {
                    QRgb *line = ( QRgb * )image->scanLine( y );
                    line += tile.left();

                    for ( int x = tile.left(); x <= tile.right(); x++ )
                    {
                        *line++ = colorMap()->rgb( range,
                            dataObjRasterData->value2( y, x ) );
                    }
                }
            }
        }
        else if ( colorMap()->format() == QwtColorMap::Indexed )
        {
            if (yMap.isInverting())
            {
                for ( int y = tile.top(); y <= tile.bottom(); y++ )
                {
                    unsigned char *line = image->scanLine( y );
                    line += tile.left();

                    for ( int x = tile.left(); x <= tile.right(); x++ )
                    {
                        *line++ = colorMap()->colorIndex( 256, range,
                            dataObjRasterData->value2_yinv( y, x ) );
                    }
                }
            }
            else
            {
                for ( int y = tile.top(); y <= tile.bottom(); y++ )
                {
                    unsigned char *line = image->scanLine( y );
                    line += tile.left();

                    for ( int x = tile.left(); x <= tile.right(); x++ )
                    {
                        *line++ = colorMap()->colorIndex( 256, range,
                            dataObjRasterData->value2( y, x ) );
                    }
                }
            }
        }
    }
}
