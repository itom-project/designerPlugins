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

#ifndef ITEMCANVAS_H
#define ITEMCANVAS_H

#include "item.h"

#include "pcl/visualization/pcl_visualizer.h"
#include <qcolor.h>
#include <qvector3d.h>

#include "CustomTypes.h"


class ItemCanvas : public Item
{
    Q_OBJECT

    Q_PROPERTY(QColor BackgroundColor READ backgroundColor WRITE setBackgroundColor DESIGNABLE true USER true);

    Q_PROPERTY(bool CoordSysVisible READ coordSysVisible WRITE setCoordSysVisible DESIGNABLE true USER true)
    Q_PROPERTY(double CoordSysScale READ coordSysScale WRITE setCoordSysScale DESIGNABLE true USER true)
    Q_PROPERTY(Vec3f CoordSysPos READ coordSysPos WRITE setCoordSysPos DESIGNABLE true USER true)
	Q_CLASSINFO("CoordSysPos", "minimumX=-2147483647;maximumX=2147483647;minimumY=-2147483647;maximumY=2147483647;minimumZ=-2147483647;maximumZ=2147483647;");

public:
    ItemCanvas(boost::shared_ptr<pcl::visualization::PCLVisualizer> visualizer, QTreeWidgetItem *treeItem) :
        Item("canvas", treeItem),
        m_visualizer(visualizer)
    {
        m_type = "canvas";
        m_coordinateSysPos = Vec3f(0,0,0);
        m_coordinateSysVisible = true;
        m_coordinateSysScale = 1.0;
    }

    ~ItemCanvas() {};

    boost::shared_ptr<pcl::visualization::PCLVisualizer> visualizer() const { return m_visualizer; }

    //properties
    QColor backgroundColor() const { return m_backgroundColor;}
    void setBackgroundColor(const QColor& color);

    Vec3f coordSysPos() const { return m_coordinateSysPos; }
    void setCoordSysPos( const Vec3f& coordSysPos );

    bool coordSysVisible() const { return m_coordinateSysVisible; }
    void setCoordSysVisible( const bool& coordSysVisible );

    double coordSysScale() const { return m_coordinateSysScale; }
    void setCoordSysScale( const double& coordSysScale );



protected:
    void changeCoordSys();

private:
    boost::shared_ptr<pcl::visualization::PCLVisualizer> m_visualizer;

    QColor m_backgroundColor;
    bool m_coordinateSysVisible;
    double m_coordinateSysScale;
    Vec3f m_coordinateSysPos; 
};

#endif