/* ********************************************************************
   itom measurement system
   URL: http://www.uni-stuttgart.de/ito
   Copyright (C) 2018, Institut für Technische Optik (ITO),
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

#include "itemPointCloudNormal.h"

//-------------------------------------------------------------------------------------------
ItemPointCloudNormal::ItemPointCloudNormal(boost::shared_ptr<pcl::visualization::PCLVisualizer> visualizer, const QString &name, QTreeWidgetItem *treeItem)
    : Item(name, Item::rttiPointCloudNormal, treeItem),
    m_visualizer(visualizer)
{
    m_pointSize = 2;
    m_lineWidth = 1;
    m_color = QColor("white");
    m_type = "point cloud normal";
}

//-------------------------------------------------------------------------------------------
ItemPointCloudNormal::~ItemPointCloudNormal()
{
    m_visualizer->removePointCloud( m_name.toStdString() );
}

//-------------------------------------------------------------------------------------------
ito::RetVal ItemPointCloudNormal::addPointCloud(const ito::PCLPointCloud &cloud)
{
    ito::RetVal retval;

    if(cloud.getType() == ito::pclXYZNormal)
    {
        pcl::PointCloud<pcl::PointNormal>::Ptr pc = cloud.toPointXYZNormal();
        int level = pc->points.size() / 200;
        level = std::max(level,1);
        m_level = level;
        m_scale = 0.2f;
        m_visualizer->addPointCloudNormals<pcl::PointNormal>(pc, m_level /*level*/, m_scale /*scale*/, m_name.toStdString());
        m_cloud = cloud;
    }
    else if(cloud.getType() == ito::pclXYZRGBNormal)
    {
        pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr pc = cloud.toPointXYZRGBNormal();
        int level = pc->points.size() / 200;
        level = std::max(level,1);
        m_level = level;
        m_scale = 0.2f;
        m_visualizer->addPointCloudNormals<pcl::PointXYZRGBNormal>(pc, m_level /*level*/, m_scale /*scale*/, m_name.toStdString());
        m_cloud = cloud;
    }
    else if(cloud.getType() == ito::pclXYZINormal)
    {
        pcl::PointCloud<pcl::PointXYZINormal>::Ptr pc = cloud.toPointXYZINormal();
        int level = pc->points.size() / 200;
        level = std::max(level,1);
        m_level = level;
        m_scale = 0.2f;
        m_visualizer->addPointCloudNormals<pcl::PointXYZINormal>(pc, m_level /*level*/, m_scale /*scale*/, m_name.toStdString());
        m_cloud = cloud;
    }
    else
    {
        retval += ito::RetVal(ito::retError, 0, tr("type of point cloud not supported for normals").toLatin1().data());
    }
    return retval;
}

//-------------------------------------------------------------------------------------
void ItemPointCloudNormal::setLevel(int value)
{
    if (m_level != value)
    {
        m_level = std::max(0,value);
        updatePointCloudNormal();
    }
    else
    {
        m_level = std::max(0,value);
    }
}

//-------------------------------------------------------------------------------------
void ItemPointCloudNormal::setScale(float value)
{
    if (m_scale != value)
    {
        m_scale = value;
        updatePointCloudNormal();
    }
    else
    {
        m_scale = value;
    }
}

//-------------------------------------------------------------------------------------------
void ItemPointCloudNormal::setVisible(bool value)
{
    m_visible = value;

    double val = value ? 1.0 : 0.0;
    m_visualizer->setPointCloudRenderingProperties( pcl::visualization::PCL_VISUALIZER_OPACITY, val, m_name.toStdString());
    emit updateCanvasRequest();

    Item::setVisible(value);
}

//-------------------------------------------------------------------------------------------
void ItemPointCloudNormal::setPointSize(int value)
{
    m_pointSize = value;
    m_visualizer->setPointCloudRenderingProperties( pcl::visualization::PCL_VISUALIZER_POINT_SIZE, value, m_name.toStdString() );
    emit updateCanvasRequest();
}

//-------------------------------------------------------------------------------------------
void ItemPointCloudNormal::setLineWidth(int value)
{
    m_lineWidth = value;
    m_visualizer->setPointCloudRenderingProperties( pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, value, m_name.toStdString() );
    emit updateCanvasRequest();
}

void ItemPointCloudNormal::setColor(QColor value)
{
    m_color = value;
    m_visualizer->setPointCloudRenderingProperties( pcl::visualization::PCL_VISUALIZER_COLOR, value.red()/255.0, value.green()/255.0, value.blue()/255.0, m_name.toStdString() );
    emit updateCanvasRequest();
}

//-------------------------------------------------------------------------------------
void ItemPointCloudNormal::updatePointCloudNormal()
{
    pcl::visualization::CloudActorMap *map = m_visualizer->getCloudActorMap().get();
    pcl::visualization::CloudActorMap::iterator it = map->find( m_name.toStdString() );

    if(it != map->end())
    {
        vtkSmartPointer<vtkPoints> points = vtkSmartPointer<vtkPoints>::New();
        points->SetDataTypeToFloat ();

        vtkSmartPointer<vtkCellArray> lines = vtkSmartPointer<vtkCellArray>::New();

        vtkSmartPointer<vtkFloatArray> data = vtkSmartPointer<vtkFloatArray>::New ();
        data->SetNumberOfComponents (3);

        vtkIdType nr_normals = 0;
        float* pts = NULL;
        vtkIdType point_step = static_cast<vtkIdType> (sqrt (double (m_level)));
        vtkIdType nrOfPoints = m_cloud.width() * m_cloud.height();

        if (m_cloud.isOrganized() && point_step > 0)
        {
            nr_normals = (static_cast<vtkIdType> ((m_cloud.width() - 1) / point_step) + 1) *
                                    (static_cast<vtkIdType> ((m_cloud.height() - 1) / point_step) + 1);
            pts = new float[2 * nr_normals * 3];
            vtkIdType cell_count = 0;

            if(m_cloud.getType() == ito::pclXYZNormal)
            {
                pcl::PointCloud<pcl::PointNormal>::Ptr pc = m_cloud.toPointXYZNormal();

                for (vtkIdType y = 0; y < pc->height; y += point_step)
                {
                    for (vtkIdType x = 0; x < pc->width; x += point_step)
                    {
                        pcl::PointNormal p = (*pc)(x, y);

                        pts[2 * cell_count * 3 + 0] = p.x;
                        pts[2 * cell_count * 3 + 1] = p.y;
                        pts[2 * cell_count * 3 + 2] = p.z;
                        pts[2 * cell_count * 3 + 3] = p.x + p.normal_x * m_scale;
                        pts[2 * cell_count * 3 + 4] = p.y + p.normal_y * m_scale;
                        pts[2 * cell_count * 3 + 5] = p.z + p.normal_z * m_scale;

                        lines->InsertNextCell (2);
                        lines->InsertCellPoint (2 * cell_count);
                        lines->InsertCellPoint (2 * cell_count + 1);
                        cell_count ++;
                    }
                }
            }
            else if(m_cloud.getType() == ito::pclXYZRGBNormal)
            {
                pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr pc = m_cloud.toPointXYZRGBNormal();

                for (vtkIdType y = 0; y < pc->height; y += point_step)
                {
                    for (vtkIdType x = 0; x < pc->width; x += point_step)
                    {
                        pcl::PointXYZRGBNormal p = (*pc)(x, y);

                        pts[2 * cell_count * 3 + 0] = p.x;
                        pts[2 * cell_count * 3 + 1] = p.y;
                        pts[2 * cell_count * 3 + 2] = p.z;
                        pts[2 * cell_count * 3 + 3] = p.x + p.normal_x * m_scale;
                        pts[2 * cell_count * 3 + 4] = p.y + p.normal_y * m_scale;
                        pts[2 * cell_count * 3 + 5] = p.z + p.normal_z * m_scale;

                        lines->InsertNextCell (2);
                        lines->InsertCellPoint (2 * cell_count);
                        lines->InsertCellPoint (2 * cell_count + 1);
                        cell_count ++;
                    }
                }
            }
            else if(m_cloud.getType() == ito::pclXYZINormal)
            {
                pcl::PointCloud<pcl::PointXYZINormal>::Ptr pc = m_cloud.toPointXYZINormal();

                for (vtkIdType y = 0; y < pc->height; y += point_step)
                {
                    for (vtkIdType x = 0; x < pc->width; x += point_step)
                    {
                        pcl::PointXYZINormal p = (*pc)(x, y);

                        pts[2 * cell_count * 3 + 0] = p.x;
                        pts[2 * cell_count * 3 + 1] = p.y;
                        pts[2 * cell_count * 3 + 2] = p.z;
                        pts[2 * cell_count * 3 + 3] = p.x + p.normal_x * m_scale;
                        pts[2 * cell_count * 3 + 4] = p.y + p.normal_y * m_scale;
                        pts[2 * cell_count * 3 + 5] = p.z + p.normal_z * m_scale;

                        lines->InsertNextCell (2);
                        lines->InsertCellPoint (2 * cell_count);
                        lines->InsertCellPoint (2 * cell_count + 1);
                        cell_count ++;
                    }
                }
            }

            data->SetArray (&pts[0], 2 * nr_normals * 3, 0);
        }
        else if( point_step > 0) //non organized
        {
            nr_normals = (nrOfPoints - 1) / m_level + 1 ;
            pts = new float[2 * nr_normals * 3];

            if(m_cloud.getType() == ito::pclXYZNormal)
            {
                pcl::PointCloud<pcl::PointNormal>::Ptr pc = m_cloud.toPointXYZNormal();

                for (vtkIdType i = 0, j = 0; j < nr_normals; j++, i = j * m_level)
                {
                    pcl::PointNormal p = pc->points[i];
                    pts[2 * j * 3 + 0] = p.x;
                    pts[2 * j * 3 + 1] = p.y;
                    pts[2 * j * 3 + 2] = p.z;
                    pts[2 * j * 3 + 3] = p.x + p.normal[0] * m_scale;
                    pts[2 * j * 3 + 4] = p.y + p.normal[1] * m_scale;
                    pts[2 * j * 3 + 5] = p.z + p.normal[2] * m_scale;

                    lines->InsertNextCell (2);
                    lines->InsertCellPoint (2 * j);
                    lines->InsertCellPoint (2 * j + 1);
                }
            }
            else if(m_cloud.getType() == ito::pclXYZRGBNormal)
            {
                pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr pc = m_cloud.toPointXYZRGBNormal();

                for (vtkIdType i = 0, j = 0; j < nr_normals; j++, i = j * m_level)
                {
                    pcl::PointXYZRGBNormal p = pc->points[i];
                    pts[2 * j * 3 + 0] = p.x;
                    pts[2 * j * 3 + 1] = p.y;
                    pts[2 * j * 3 + 2] = p.z;
                    pts[2 * j * 3 + 3] = p.x + p.normal[0] * m_scale;
                    pts[2 * j * 3 + 4] = p.y + p.normal[1] * m_scale;
                    pts[2 * j * 3 + 5] = p.z + p.normal[2] * m_scale;

                    lines->InsertNextCell (2);
                    lines->InsertCellPoint (2 * j);
                    lines->InsertCellPoint (2 * j + 1);
                }
            }
            else if(m_cloud.getType() == ito::pclXYZINormal)
            {
                pcl::PointCloud<pcl::PointXYZINormal>::Ptr pc = m_cloud.toPointXYZINormal();

                for (vtkIdType i = 0, j = 0; j < nr_normals; j++, i = j * m_level)
                {
                    pcl::PointXYZINormal p = pc->points[i];
                    pts[2 * j * 3 + 0] = p.x;
                    pts[2 * j * 3 + 1] = p.y;
                    pts[2 * j * 3 + 2] = p.z;
                    pts[2 * j * 3 + 3] = p.x + p.normal[0] * m_scale;
                    pts[2 * j * 3 + 4] = p.y + p.normal[1] * m_scale;
                    pts[2 * j * 3 + 5] = p.z + p.normal[2] * m_scale;

                    lines->InsertNextCell (2);
                    lines->InsertCellPoint (2 * j);
                    lines->InsertCellPoint (2 * j + 1);
                }
            }

            data->SetArray (&pts[0], 2 * nr_normals * 3, 0);
        }
        else // no normal vectors (m_level == 0)
        {
            pts = new float[nrOfPoints * 3];

            if(m_cloud.getType() == ito::pclXYZNormal)
            {
                pcl::PointCloud<pcl::PointNormal>::Ptr pc = m_cloud.toPointXYZNormal();

                for (vtkIdType i = 0; i < nrOfPoints; ++i)
                {
                    pcl::PointNormal p = pc->points[i];
                    pts[i * 3 + 0] = pc->points[i].x;
                    pts[i * 3 + 1] = pc->points[i].y;
                    pts[i * 3 + 2] = pc->points[i].z;
                }
            }
            else if(m_cloud.getType() == ito::pclXYZRGBNormal)
            {
                pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr pc = m_cloud.toPointXYZRGBNormal();

                for (vtkIdType i = 0; i < nrOfPoints; ++i)
                {
                    pcl::PointXYZRGBNormal p = pc->points[i];
                    pts[i * 3 + 0] = pc->points[i].x;
                    pts[i * 3 + 1] = pc->points[i].y;
                    pts[i * 3 + 2] = pc->points[i].z;
                }
            }

            data->SetArray (&pts[0], nrOfPoints * 3, 0);
        }

        points->SetData (data);

        vtkSmartPointer<vtkPolyData> polyData = vtkSmartPointer<vtkPolyData>::New();
        polyData->SetPoints (points);
        polyData->SetLines (lines);

        vtkDataSetMapper *mapper = (vtkDataSetMapper*)it->second.actor->GetMapper();
#if (VTK_MAJOR_VERSION == 5)
        mapper->SetInput (polyData);
#else
        mapper->SetInputData (polyData);
#endif

        mapper->SetColorModeToMapScalars();
        mapper->SetScalarModeToUsePointData();

        emit updateCanvasRequest();
    }


  }
