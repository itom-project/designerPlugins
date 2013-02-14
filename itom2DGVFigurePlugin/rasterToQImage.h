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

#ifndef RASTERTOQIMAGE_H
#define RASTERTOQIMAGE_H

//#include "../../Qitom/global.h"
#include "common/sharedStructures.h"
#include "DataObject/dataobj.h"

#include <qimage.h>
#include <qlist.h>
#include <qrect.h>
#include <qsharedpointer.h>

//using namespace ito;

//----------------------------------------------------------------------------------------------------------------------------------
class RasterToQImageObj : QObject
{
    public:
        explicit RasterToQImageObj(QSharedPointer<ito::DataObject> dataObj, const QRect ROI, /*const unsigned int XDimIndex, const unsigned int YDimIndex,*/ bool replotPending);
        ~RasterToQImageObj();

        QImage getRastersImage();

        void updateDataObject(QSharedPointer<ito::DataObject> dataObj);
        void updateDataObject(QSharedPointer<ito::DataObject> dataObj, const QRect ROI /*, const unsigned int XDimIndex, const unsigned int YDimIndex*/);
		inline QSharedPointer<ito::DataObject> getDataObject(void) { return m_dataObj; };

        void setIntervalRange(Qt::Axis axis, bool autoCalcLimits, double minValue, double maxValue);
        void setColorTable(const QVector<ito::uint32> newColorTable);
		inline int getDataObjWidth() { return m_DataObjectWidth; };
		inline int getDataObjHeight() { return m_DataObjectHeight; };
		
        inline void setCmplxState(const int state) { m_cmplxState = state; };
        inline int getCmplxState() { return m_cmplxState;};
        
        void setColorMode(const unsigned char colorMode);
        inline int getColorMode(){return m_colorMode;};

        ito::float64 getPixel(const QPointF &coords, bool &isInt);
        bool getPixelARGB(const QPointF &coords, unsigned char &AValue, unsigned char &RValue, unsigned char &GValue, unsigned char &BValue);
        
        enum tZValueMode{
            ColorIndex8Scaled   = 0x00,
            ColorIndex8Bitshift = 0x01,
            ColorRGB24          = 0x02,
            ColorRGB32          = 0x04
        };

    protected:

    private:

        QVector<uint> m_colorTable;

        QSharedPointer<ito::DataObject> m_dataObj;               /*!< borrowed reference, do not delete here */
        QSharedPointer<ito::DataObject> m_dataObjWhileRastering;

        //bool *m_pReplotPending;  /*!< pointer to m_refreshPending of plot2DImage (1 => update requested, 0 => repaint done or no update requested)*/
        bool m_replotPending;
        unsigned char  m_colorMode;
        unsigned char  m_numBits;

        double m_minZValue;
        double m_maxZValue;

        QRect m_ROI;
        QRect m_physROI;

        unsigned int m_wDimIndex;
        unsigned int m_hDimIndex;

        int m_DataObjectWidth;
        int m_DataObjectHeight;
		int m_cmplxState;
};

//----------------------------------------------------------------------------------------------------------------------------------

#endif
