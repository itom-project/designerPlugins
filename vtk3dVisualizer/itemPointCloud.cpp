/* ********************************************************************
   itom measurement system
   URL: http://www.uni-stuttgart.de/ito
   Copyright (C) 2015, Institut f�r Technische Optik (ITO), 
   Universit�t Stuttgart, Germany 
 
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

#include "itemPointCloud.h"

//-------------------------------------------------------------------------------------------
ItemPointCloud::ItemPointCloud(boost::shared_ptr<pcl::visualization::PCLVisualizer> visualizer, const QString &name, QTreeWidgetItem *treeItem)
    : Item(name, treeItem),
    m_visualizer(visualizer)
    //m_selected(false)
{
    m_pointSize = 2;
    m_lineWidth = 1;
    m_type = "point cloud";
}

//-------------------------------------------------------------------------------------------
ItemPointCloud::~ItemPointCloud()
{
    m_visualizer->removePointCloud( m_name.toStdString() );
}

//-------------------------------------------------------------------------------------------
ito::RetVal ItemPointCloud::addPointCloud(const ito::PCLPointCloud &cloud)
{
    ito::RetVal retval;

    if(cloud.getType() == ito::pclXYZ)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr pc = cloud.toPointXYZ();

        if(0)
        {
            pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color(pc, 255, 255, 0);
            m_visualizer->addPointCloud<pcl::PointXYZ>(pc, single_color, m_name.toStdString());
        }
        else if(1)
        {
            std::vector<int> indices;
            //pcl::removeNaNFromPointCloud( *cloud, *cloud, indices );
            pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZ> colorHandler(pc, "z");
            m_visualizer->addPointCloud<pcl::PointXYZ>(pc, colorHandler, m_name.toStdString());
        }
        else
        {
            m_visualizer->addPointCloud<pcl::PointXYZ>(pc, m_name.toStdString());
            setPointSize(2);
        }
    }
    else if(cloud.getType() == ito::pclXYZI)
    {
        pcl::PointCloud<pcl::PointXYZI>::Ptr pc = cloud.toPointXYZI();

        if(0)
        {
            pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZI> single_color(pc, 255, 255, 0);
            m_visualizer->addPointCloud<pcl::PointXYZI>(pc, single_color, m_name.toStdString());
        }
        else if(1)
        {
            std::vector<int> indices;
            //pcl::removeNaNFromPointCloud( *cloud, *cloud, indices );
            pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZI> colorHandler(pc, "intensity");
            m_visualizer->addPointCloud<pcl::PointXYZI>(pc, colorHandler, m_name.toStdString());
        }
        else
        {
            m_visualizer->addPointCloud<pcl::PointXYZI>(pc, m_name.toStdString());
            setPointSize(2);
        }
    }
    else if(cloud.getType() == ito::pclXYZNormal)
    {
        pcl::PointCloud<pcl::PointNormal>::Ptr pc = cloud.toPointXYZNormal();
        m_visualizer->addPointCloud<pcl::PointNormal>(pc, m_name.toStdString());
        //m_visualizer->addPointCloudNormals<pcl::PointNormal>(pc, 100 /*level*/, 0.02 /*scale*/, m_name.toStdString());
    }
    else if(cloud.getType() == ito::pclXYZRGBNormal)
    {
        pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr pc = cloud.toPointXYZRGBNormal();
        pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGBNormal> colorHandler(pc);
        m_visualizer->addPointCloud<pcl::PointXYZRGBNormal>(pc, colorHandler, m_name.toStdString());
    }
    else if (cloud.getType() == ito::pclXYZRGBA)
    {
        pcl::PointCloud<pcl::PointXYZRGBA>::Ptr pc = cloud.toPointXYZRGBA();
        pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGBA> colorHandler(pc);
        m_visualizer->addPointCloud<pcl::PointXYZRGBA>(pc, colorHandler, m_name.toStdString());
    }
    else
    {
        retval += ito::RetVal(ito::retError,0,"type of point cloud not supported");
    }
    return retval;
}

ito::RetVal ItemPointCloud::updatePointCloud(const ito::PCLPointCloud &cloud)
{
    ito::RetVal retval;

    if(cloud.getType() == ito::pclXYZ)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr pc = cloud.toPointXYZ();

        if(0)
        {
            pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color(pc, 255, 255, 0);
            m_visualizer->updatePointCloud<pcl::PointXYZ>(pc, single_color, m_name.toStdString());
        }
        else if(1)
        {
            std::vector<int> indices;
            //pcl::removeNaNFromPointCloud( *cloud, *cloud, indices );
            pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZ> colorHandler(pc, "z");
            m_visualizer->updatePointCloud<pcl::PointXYZ>(pc, colorHandler, m_name.toStdString());
        }
        else
        {
            m_visualizer->updatePointCloud<pcl::PointXYZ>(pc, m_name.toStdString());
            setPointSize(2);
        }
    }
    else if(cloud.getType() == ito::pclXYZI)
    {
        pcl::PointCloud<pcl::PointXYZI>::Ptr pc = cloud.toPointXYZI();

        if(0)
        {
            pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZI> single_color(pc, 255, 255, 0);
            m_visualizer->updatePointCloud<pcl::PointXYZI>(pc, single_color, m_name.toStdString());
        }
        else if(1)
        {
            std::vector<int> indices;
            //pcl::removeNaNFromPointCloud( *cloud, *cloud, indices );
            pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZI> colorHandler(pc, "intensity");
            m_visualizer->updatePointCloud<pcl::PointXYZI>(pc, colorHandler, m_name.toStdString());
        }
        else
        {
            m_visualizer->updatePointCloud<pcl::PointXYZI>(pc, m_name.toStdString());
            setPointSize(2);
        }
    }
    else if(cloud.getType() == ito::pclXYZNormal)
    {
        pcl::PointCloud<pcl::PointNormal>::Ptr pc = cloud.toPointXYZNormal();
        m_visualizer->updatePointCloud<pcl::PointNormal>(pc, m_name.toStdString());
        //m_visualizer->addPointCloudNormals<pcl::PointNormal>(pc, 100 /*level*/, 0.02 /*scale*/, m_name.toStdString());
    }
    else if(cloud.getType() == ito::pclXYZRGBNormal)
    {
        pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr pc = cloud.toPointXYZRGBNormal();
        pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGBNormal> colorHandler(pc);
        m_visualizer->updatePointCloud<pcl::PointXYZRGBNormal>(pc, colorHandler, m_name.toStdString());
    }
    else if (cloud.getType() == ito::pclXYZRGBA)
    {
        pcl::PointCloud<pcl::PointXYZRGBA>::Ptr pc = cloud.toPointXYZRGBA();
        pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGBA> colorHandler(pc);
        m_visualizer->updatePointCloud<pcl::PointXYZRGBA>(pc, colorHandler, m_name.toStdString());
    }
    else
    {
        retval += ito::RetVal(ito::retError,0,"type of point cloud not supported");
    }
    return retval;
}

//-------------------------------------------------------------------------------------------
void ItemPointCloud::setVisible(bool value)
{
    m_visible = value;

    double val = value ? 1.0 : 0.0;
    m_visualizer->setPointCloudRenderingProperties( pcl::visualization::PCL_VISUALIZER_OPACITY, val, m_name.toStdString());
    emit updateCanvasRequest();

    Item::setVisible(value);
}

//-------------------------------------------------------------------------------------------
void ItemPointCloud::setPointSize(int value)
{
    m_pointSize = value;
    m_visualizer->setPointCloudRenderingProperties( pcl::visualization::PCL_VISUALIZER_POINT_SIZE, value, m_name.toStdString() );
    emit updateCanvasRequest();
}

//-------------------------------------------------------------------------------------------
void ItemPointCloud::setLineWidth(int value)
{
    m_lineWidth = value;
    m_visualizer->setPointCloudRenderingProperties( pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, value, m_name.toStdString() );
    emit updateCanvasRequest();
}

//-------------------------------------------------------------------------------------------
void ItemPointCloud::setColor(QColor value)
{
    m_color = value;
    m_visualizer->setPointCloudRenderingProperties( pcl::visualization::PCL_VISUALIZER_COLOR, value.red()/255.0, value.green()/255.0, value.blue()/255.0, m_name.toStdString() );
    emit updateCanvasRequest();
}

//-------------------------------------------------------------------------------------------
//void ItemPointCloud::setSelected(bool value)
//{
//    m_selected = value;
//    m_visualizer->setPointCloudSelected(value, m_name.toStdString());
//}