/* ********************************************************************
   itom measurement system
   URL: http://www.uni-stuttgart.de/ito
   Copyright (C) 2015, Institut fuer Technische Optik (ITO), 
   Universitaet Stuttgart, Germany 
 
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

#ifndef ITEMPOINTCLOUD_H
#define ITEMPOINTCLOUD_H

#include "common/sharedStructures.h"
#include "common/interval.h"
#include "PointCloud/pclStructures.h"
#include "pcl/visualization/pcl_visualizer.h"

#include "item.h"

#include <qcolor.h>


class ItemPointCloud : public Item
{
    Q_OBJECT

    Q_ENUMS(ColorMode);

    Q_PROPERTY(int PointSize READ pointSize WRITE setPointSize DESIGNABLE true USER true);
    Q_PROPERTY(int LineWidth READ lineWidth WRITE setLineWidth DESIGNABLE true USER true);
    Q_PROPERTY(ColorMode ColorMode READ colorMode WRITE setColorMode DESIGNABLE true USER true);
    Q_PROPERTY(QColor Color READ color WRITE setColor DESIGNABLE true USER true);

public:
    ItemPointCloud(boost::shared_ptr<pcl::visualization::PCLVisualizer> visualizer, const QString &name, QTreeWidgetItem *treeItem);
    virtual ~ItemPointCloud();

    enum ColorMode { SolidColor, X, Y, Z, XYZ, XY, YZ, XZ, Intensity, NormalX, NormalY, NormalZ, NormalXYZ, RGB, Curvature };

    ito::RetVal addPointCloud(const ito::PCLPointCloud &cloud);

    ito::RetVal updatePointCloud(const ito::PCLPointCloud &cloud);

    //properties
    virtual void setVisible(bool value);

    int pointSize() const { return m_pointSize; }
    void setPointSize(int value);

    int lineWidth() const { return m_lineWidth; }
    void setLineWidth(int value);

    ColorMode colorMode() const { return m_colorMode; }
    void setColorMode(ColorMode mode);

    QColor color() const { return m_color; }
    void setColor(QColor value);



protected:
    boost::shared_ptr<pcl::visualization::PCLVisualizer> m_visualizer;
    ito::PCLPointCloud m_cloud;

    int m_pointSize;
    int m_lineWidth;
    QColor m_color;
    ColorMode m_colorMode;



    template <typename PointT> ito::RetVal addPointCloudTmpl(typename pcl::PointCloud<PointT>::Ptr cloud, bool update = false);
    template <typename PointT> ito::RetVal addPointCloudTmplRgba(typename pcl::PointCloud<PointT>::Ptr cloud, bool update = false);

};





#endif //ITEM_H