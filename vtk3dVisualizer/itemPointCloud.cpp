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

#include "itemPointCloud.h"

#include "pointCloudHandlerGenericFields.h"

//-------------------------------------------------------------------------------------------
ItemPointCloud::ItemPointCloud(boost::shared_ptr<pcl::visualization::PCLVisualizer> visualizer, const QString &name, QTreeWidgetItem *treeItem)
    : Item(name, Item::rttiPointCloud, treeItem),
    m_visualizer(visualizer)
{
    m_pointSize = 2;
    m_lineWidth = 1;
    m_type = "point cloud";
    m_colorMode = ColorStatic;
    m_color = QColor("white");
}

//-------------------------------------------------------------------------------------------
ItemPointCloud::~ItemPointCloud()
{
    m_visualizer->removePointCloud( m_name.toStdString() );
}

//-------------------------------------------------------------------------------------------
template <typename PointT> ito::RetVal ItemPointCloud::addPointCloudTmpl(typename pcl::PointCloud<PointT>::Ptr cloud, bool update /*=false*/)
{
    ito::RetVal retval;
    ColorMode mode = m_colorMode;
    bool colorFound = false;

    while (!colorFound)
    {
        switch (mode)
        {
        case ColorStatic:
            {
                pcl::visualization::PointCloudColorHandlerCustom<PointT> color_handler(cloud, m_color.red(), m_color.green(), m_color.blue());
                if (color_handler.isCapable())
                {
                    if (update)
                    {
                        m_visualizer->updatePointCloud<PointT>(cloud, color_handler, m_name.toStdString());
                    }
                    else
                    {
                        m_visualizer->addPointCloud<PointT>(cloud, color_handler, m_name.toStdString());
                    }
                    colorFound = true;
                }
                break;
            }
        case ColorX:
        case ColorY:
        case ColorZ:
        case ColorIntensity:
        case ColorNormalX:
        case ColorNormalY:
        case ColorNormalZ:
        case ColorCurvature:
            {
                std::string f = "x";
                if (mode == ColorY) f = "y";
                if (mode == ColorZ) f = "z";
                if (mode == ColorIntensity) f = "intensity";
                if (mode == ColorNormalX) f = "normal_x";
                if (mode == ColorNormalY) f = "normal_y";
                if (mode == ColorNormalZ) f = "normal_z";
                if (mode == ColorCurvature) f = "curvature";
                pcl::visualization::PointCloudColorHandlerGenericField<PointT> color_handler(cloud, f);
                if (color_handler.isCapable())
                {
                    if (update)
                    {
                        m_visualizer->updatePointCloud<PointT>(cloud, color_handler, m_name.toStdString());
                    }
                    else
                    {
                        m_visualizer->addPointCloud<PointT>(cloud, color_handler, m_name.toStdString());
                    }
                    colorFound = true;
                }
                break;
            }
        //the PointCloudColorHandlerRGBField does not compile if PointT is a point type without rgba information
        /*case ColorRGB:
            {
                pcl::visualization::PointCloudColorHandlerRGBField<PointT> color_handler(cloud);
                if (color_handler.isCapable())
                {
                    if (update)
                    {
                        m_visualizer->updatePointCloud<PointT>(cloud, color_handler, m_name.toStdString());
                    }
                    else
                    {
                        m_visualizer->addPointCloud<PointT>(cloud, color_handler, m_name.toStdString());
                    }
                    colorFound = true;
                }
                break;
            }*/
        case ColorAbsXYZ:
            {
                PointCloudColorHandlerGenericFields<PointT> color_handler(cloud, "x", "y", "z");
                if (color_handler.isCapable())
                {
                    if (update)
                    {
                        m_visualizer->updatePointCloud<PointT>(cloud, color_handler, m_name.toStdString());
                    }
                    else
                    {
                        m_visualizer->addPointCloud<PointT>(cloud, color_handler, m_name.toStdString());
                    }
                    colorFound = true;
                }
                break;
            }
        case ColorNormalAbsXYZ:
            {
                PointCloudColorHandlerGenericFields<PointT> color_handler(cloud, "normal_x", "normal_y", "normal_z");
                if (color_handler.isCapable())
                {
                    if (update)
                    {
                        m_visualizer->updatePointCloud<PointT>(cloud, color_handler, m_name.toStdString());
                    }
                    else
                    {
                        m_visualizer->addPointCloud<PointT>(cloud, color_handler, m_name.toStdString());
                    }
                    colorFound = true;
                }
                break;
            }
        default:
            {
                if (update)
                {
                    m_visualizer->updatePointCloud<PointT>(cloud, m_name.toStdString());
                }
                else
                {
                    m_visualizer->addPointCloud<PointT>(cloud, m_name.toStdString());
                }
                colorFound = true;
            }
        }

        if (!colorFound)
        {
            mode = ColorStatic;
            m_colorMode = mode;
        }
    }

    if (!retval.containsError())
    {
        m_visualizer->setPointCloudRenderingProperties( pcl::visualization::PCL_VISUALIZER_POINT_SIZE, m_pointSize, m_name.toStdString() );
    }

    return retval;
}

//-------------------------------------------------------------------------------------------
template <typename PointT> ito::RetVal ItemPointCloud::addPointCloudTmplRgba(typename pcl::PointCloud<PointT>::Ptr cloud, bool update /*=false*/)
{
    ito::RetVal retval;
    ColorMode mode = m_colorMode;
    bool colorFound = false;

    while (!colorFound)
    {
        switch (mode)
        {
        case ColorStatic:
            {
                pcl::visualization::PointCloudColorHandlerCustom<PointT> color_handler(cloud, m_color.red(), m_color.green(), m_color.blue());
                if (color_handler.isCapable())
                {
                    if (update)
                    {
                        m_visualizer->updatePointCloud<PointT>(cloud, color_handler, m_name.toStdString());
                    }
                    else
                    {
                        m_visualizer->addPointCloud<PointT>(cloud, color_handler, m_name.toStdString());
                    }
                    colorFound = true;
                }
                break;
            }
        case ColorX:
        case ColorY:
        case ColorZ:
        case ColorIntensity:
        case ColorNormalX:
        case ColorNormalY:
        case ColorNormalZ:
        case ColorCurvature:
            {
                std::string f = "x";
                if (mode == ColorY) f = "y";
                if (mode == ColorZ) f = "z";
                if (mode == ColorIntensity) f = "intensity";
                if (mode == ColorNormalX) f = "normal_x";
                if (mode == ColorNormalY) f = "normal_y";
                if (mode == ColorNormalZ) f = "normal_z";
                if (mode == ColorCurvature) f = "curvature";
                pcl::visualization::PointCloudColorHandlerGenericField<PointT> color_handler(cloud, f);
                if (color_handler.isCapable())
                {
                    if (update)
                    {
                        m_visualizer->updatePointCloud<PointT>(cloud, color_handler, m_name.toStdString());
                    }
                    else
                    {
                        m_visualizer->addPointCloud<PointT>(cloud, color_handler, m_name.toStdString());
                    }
                    colorFound = true;
                }
                break;
            }
        case ColorRGB:
            {
                pcl::visualization::PointCloudColorHandlerRGBField<PointT> color_handler(cloud);
                if (color_handler.isCapable())
                {
                    if (update)
                    {
                        m_visualizer->updatePointCloud<PointT>(cloud, color_handler, m_name.toStdString());
                    }
                    else
                    {
                        m_visualizer->addPointCloud<PointT>(cloud, color_handler, m_name.toStdString());
                    }
                    colorFound = true;
                }
                break;
            }
        case ColorAbsXYZ:
            {
                PointCloudColorHandlerGenericFields<PointT> color_handler(cloud, "x", "y", "z");
                if (color_handler.isCapable())
                {
                    if (update)
                    {
                        m_visualizer->updatePointCloud<PointT>(cloud, color_handler, m_name.toStdString());
                    }
                    else
                    {
                        m_visualizer->addPointCloud<PointT>(cloud, color_handler, m_name.toStdString());
                    }
                    colorFound = true;
                }
                break;
            }
        case ColorNormalAbsXYZ:
            {
                PointCloudColorHandlerGenericFields<PointT> color_handler(cloud, "normal_x", "normal_y", "normal_z");
                if (color_handler.isCapable())
                {
                    if (update)
                    {
                        m_visualizer->updatePointCloud<PointT>(cloud, color_handler, m_name.toStdString());
                    }
                    else
                    {
                        m_visualizer->addPointCloud<PointT>(cloud, color_handler, m_name.toStdString());
                    }
                    colorFound = true;
                }
                break;
            }
        default:
            {
                if (update)
                {
                    m_visualizer->updatePointCloud<PointT>(cloud, m_name.toStdString());
                }
                else
                {
                    m_visualizer->addPointCloud<PointT>(cloud, m_name.toStdString());
                }
                colorFound = true;
            }
        }

        if (!colorFound)
        {
            mode = ColorStatic;
            m_colorMode = mode;
        }
    }

    if (!retval.containsError())
    {
        m_visualizer->setPointCloudRenderingProperties( pcl::visualization::PCL_VISUALIZER_POINT_SIZE, m_pointSize, m_name.toStdString() );
    }

    return retval;
}

//-------------------------------------------------------------------------------------------
ito::RetVal ItemPointCloud::addPointCloud(const ito::PCLPointCloud &cloud)
{
    ito::RetVal retval;

    if(cloud.getType() == ito::pclXYZ)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr pc = cloud.toPointXYZ();
        retval += addPointCloudTmpl<pcl::PointXYZ>(pc);
    }
    else if(cloud.getType() == ito::pclXYZI)
    {
        pcl::PointCloud<pcl::PointXYZI>::Ptr pc = cloud.toPointXYZI();
        retval += addPointCloudTmpl<pcl::PointXYZI>(pc);
    }
    else if(cloud.getType() == ito::pclXYZNormal)
    {
        pcl::PointCloud<pcl::PointNormal>::Ptr pc = cloud.toPointXYZNormal();
        retval += addPointCloudTmpl<pcl::PointNormal>(pc);
    }
    else if(cloud.getType() == ito::pclXYZRGBNormal)
    {
        pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr pc = cloud.toPointXYZRGBNormal();
        retval += addPointCloudTmplRgba<pcl::PointXYZRGBNormal>(pc);
    }
    else if (cloud.getType() == ito::pclXYZRGBA)
    {
        pcl::PointCloud<pcl::PointXYZRGBA>::Ptr pc = cloud.toPointXYZRGBA();
        retval += addPointCloudTmplRgba<pcl::PointXYZRGBA>(pc);
    }
    else
    {
        retval += ito::RetVal(ito::retError,0,"type of point cloud not supported");
    }

    m_cloud = cloud;

    return retval;
}

//-----------------------------------------------------------------------------------------------------------------------------
ito::RetVal ItemPointCloud::updatePointCloud(const ito::PCLPointCloud &cloud)
{
    ito::RetVal retval;

    if(cloud.getType() == ito::pclXYZ)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr pc = cloud.toPointXYZ();
        retval += addPointCloudTmpl<pcl::PointXYZ>(pc, true);
    }
    else if(cloud.getType() == ito::pclXYZI)
    {
        pcl::PointCloud<pcl::PointXYZI>::Ptr pc = cloud.toPointXYZI();
        retval += addPointCloudTmpl<pcl::PointXYZI>(pc, true);
    }
    else if(cloud.getType() == ito::pclXYZNormal)
    {
        pcl::PointCloud<pcl::PointNormal>::Ptr pc = cloud.toPointXYZNormal();
        retval += addPointCloudTmpl<pcl::PointNormal>(pc, true);
    }
    else if(cloud.getType() == ito::pclXYZRGBNormal)
    {
        pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr pc = cloud.toPointXYZRGBNormal();
        retval += addPointCloudTmplRgba<pcl::PointXYZRGBNormal>(pc, true);
    }
    else if (cloud.getType() == ito::pclXYZRGBA)
    {
        pcl::PointCloud<pcl::PointXYZRGBA>::Ptr pc = cloud.toPointXYZRGBA();
        retval += addPointCloudTmplRgba<pcl::PointXYZRGBA>(pc, true);
    }
    else
    {
        retval += ito::RetVal(ito::retError,0,"type of point cloud not supported");
    }

    m_cloud = cloud;

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
    if (m_colorMode == ColorStatic && m_color != value)
    {
        m_color = value;
        updatePointCloud(m_cloud);
    }
    else
    {
        m_color = value;
    }
    emit updateCanvasRequest();
}

//-------------------------------------------------------------------------------------------
void ItemPointCloud::setColorMode(ColorMode mode)
{
    if (m_colorMode != mode)
    {
        m_colorMode = mode;
        updatePointCloud(m_cloud);
    }
    else
    {
        m_colorMode = mode;
    }
    
    emit updateCanvasRequest();
}

//-------------------------------------------------------------------------------------------
//void ItemPointCloud::setSelected(bool value)
//{
//    m_selected = value;
//    m_visualizer->setPointCloudSelected(value, m_name.toStdString());
//}