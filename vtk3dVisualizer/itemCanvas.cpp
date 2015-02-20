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

#include "vtkRenderWindow.h"

#include <pcl/visualization/common/common.h>

ItemCanvas::ItemCanvas(boost::shared_ptr<pcl::visualization::PCLVisualizer> visualizer, QTreeWidgetItem *treeItem) :
        Item("canvas", treeItem),
        m_visualizer(visualizer),
        m_showFPS(true),
        m_stereoType(No)
{
    m_type = "canvas";
    m_coordinateSysPos = Vec3f(0,0,0);
    m_coordinateSysVisible = true;
    m_coordinateSysScale = 1.0;
}

//-----------------------------------------------------------------------
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

//-----------------------------------------------------------------------
void ItemCanvas::setCoordSysPos( const Vec3f& coordSysPos )
{
    m_coordinateSysPos = coordSysPos;
    changeCoordSys();
}

//-----------------------------------------------------------------------
void ItemCanvas::setCoordSysVisible( const bool& coordSysVisible )
{
    m_coordinateSysVisible = coordSysVisible;
    changeCoordSys();
}

//-----------------------------------------------------------------------
void ItemCanvas::setCoordSysScale( const double& coordSysScale )
{
    m_coordinateSysScale = coordSysScale;
    changeCoordSys();
}

//-----------------------------------------------------------------------
void ItemCanvas::setShowFPS( const bool& showFPS )
{
    m_visualizer->setShowFPS(showFPS);
    m_showFPS = showFPS;
}

//-----------------------------------------------------------------------
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


//-----------------------------------------------------------------------
void ItemCanvas::setStereoType( const Stereo& stereoType )
{
    int type;

    switch (stereoType)
    {
    case No:
        type = 0;
        break;
    case CrystalEyes:
        type = VTK_STEREO_CRYSTAL_EYES;
        break;
    case RedBlue:
        type = VTK_STEREO_RED_BLUE;
        break;
    case Interlaced:
        type = VTK_STEREO_INTERLACED;
        break;
    case Left:
        type = VTK_STEREO_LEFT;
        break;
    case Right:
        type = VTK_STEREO_RIGHT;
        break;
    case Dresden:
        type = VTK_STEREO_DRESDEN;
        break;
    case Anaglyph:
        type = VTK_STEREO_ANAGLYPH;
        break;
    case Checkerboard:
        type = VTK_STEREO_CHECKERBOARD;
        break;
    }

    m_stereoType = stereoType;

    vtkSmartPointer<vtkRenderWindow> win = m_visualizer->getRenderWindow();

    if (type != 0)
    {
        win->SetStereoType(type);
        win->StereoRenderOn();
        win->StereoUpdate();
    }
    else
    {
        win->StereoRenderOff();
        win->StereoUpdate();
    }

    emit updateCanvasRequest();
}

//-----------------------------------------------------------------------
Vec3f ItemCanvas::cameraPosition() const
{
    std::vector<pcl::visualization::Camera> cameras;
    m_visualizer->getCameras(cameras);
    if (cameras.size() > 0)
    {
        return Vec3f(cameras[0].pos[0], cameras[0].pos[1], cameras[0].pos[2]);
    } 

    return Vec3f(0.0, 0.0, 0.0);
}

//-----------------------------------------------------------------------
void ItemCanvas::setCameraPosition( const Vec3f& cameraPosition )
{
    std::vector<pcl::visualization::Camera> cameras;
    m_visualizer->getCameras(cameras);
    if (cameras.size() > 0)
    {
        m_visualizer->setCameraPosition(cameraPosition.X, cameraPosition.Y, cameraPosition.Z, cameras[0].view[0], cameras[0].view[1], cameras[0].view[2]);
        emit updateCanvasRequest();
    } 
}

//-----------------------------------------------------------------------
Vec3f ItemCanvas::cameraView() const
{
    std::vector<pcl::visualization::Camera> cameras;
    m_visualizer->getCameras(cameras);
    if (cameras.size() > 0)
    {
        return Vec3f(cameras[0].view[0], cameras[0].view[1], cameras[0].view[2]);
    } 

    return Vec3f(0.0, 0.0, 0.0);
}

//-----------------------------------------------------------------------
void ItemCanvas::setCameraView( const Vec3f& cameraView )
{
    std::vector<pcl::visualization::Camera> cameras;
    m_visualizer->getCameras(cameras);
    if (cameras.size() > 0)
    {
        m_visualizer->setCameraPosition(cameras[0].pos[0], cameras[0].pos[1], cameras[0].pos[2], cameraView.X, cameraView.Y, cameraView.Z);
        emit updateCanvasRequest();
    } 
}