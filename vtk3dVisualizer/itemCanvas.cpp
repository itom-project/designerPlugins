/* ********************************************************************
   itom measurement system
   URL: http://www.uni-stuttgart.de/ito
   Copyright (C) 2015, Institut für Technische Optik (ITO), 
   Universität Stuttgart, Germany 
 
   This file is part of the designer widget 'vtk3dVisualizer' for itom.

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

#include "itemCanvas.h"

void ItemCanvas::setBackgroundColor(const QColor& color)
{
    QRgb rgb = color.rgb();
    double r = qRed(rgb)/256.0;
    double g = qGreen(rgb)/256.0;
    double b = qBlue(rgb)/256.0;
    m_visualizer->setBackgroundColor( r,g,b);
    m_backgroundColor = color;

    emit updateCanvasRequest();
}


void ItemCanvas::setCoordSysPos( const Vec3f& coordSysPos )
{
    m_coordinateSysPos = coordSysPos;
    changeCoordSys();
}

void ItemCanvas::setCoordSysVisible( const bool& coordSysVisible )
{
    m_coordinateSysVisible = coordSysVisible;
    changeCoordSys();
}

void ItemCanvas::setCoordSysScale( const double& coordSysScale )
{
    m_coordinateSysScale = coordSysScale;
    changeCoordSys();
}

void ItemCanvas::changeCoordSys()
{
#if PCL_VERSION_COMPARE(>=,1,7,1)
    m_visualizer->removeCoordinateSystem("mainCoordinateSystem");
    if(m_coordinateSysVisible)
    {
        m_visualizer->addCoordinateSystem( m_coordinateSysScale, m_coordinateSysPos.X, m_coordinateSysPos.Y, m_coordinateSysPos.Z, "mainCoordinateSystem");
    }
#else
	m_visualizer->removeCoordinateSystem();
    if(m_coordinateSysVisible)
    {
        m_visualizer->addCoordinateSystem( m_coordinateSysScale, m_coordinateSysPos.X, m_coordinateSysPos.Y, m_coordinateSysPos.Z);
    }
#endif

    emit updateCanvasRequest();
}