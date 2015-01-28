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