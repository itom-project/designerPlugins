/* ********************************************************************
   itom measurement system
   URL: http://www.uni-stuttgart.de/ito
   Copyright (C) 2018, Institut fuer Technische Optik (ITO),
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

#include "itemGeometry.h"

#include "vtkPolyLine.h"
#include "vtkTextActor.h"

//-------------------------------------------------------------------------------------------
ItemGeometry::ItemGeometry(boost::shared_ptr<pcl::visualization::PCLVisualizer> visualizer, const QString &name, QTreeWidgetItem *treeItem)
    : Item(name, Item::rttiGeometry, treeItem),
    m_visualizer(visualizer),
    m_representation(Wireframe),
    m_interpolation(Flat),
    m_lineWidth(1.0),
    m_opacity(1.0),
    m_lighting(false),
    m_nrOfShapes(1),
    m_specular(0.8),
    m_specularPower(0.8)
    //m_selected(false)
{
    m_type = "geometry";
}

//-------------------------------------------------------------------------------------------
ItemGeometry::~ItemGeometry()
{
    if (m_nrOfShapes == 1)
    {
        m_visualizer->removeShape( m_name.toStdString() );
    }
    else
    {
        QString name;
        for (int i = 0; i < m_nrOfShapes; ++i)
        {
            name = QString("%1_%2").arg(m_name).arg(i);
            m_visualizer->removeShape( name.toStdString() );
        }
    }
}

//-------------------------------------------------------------------------------------------
ito::RetVal ItemGeometry::addCylinder(const pcl::ModelCoefficients &coefficients, const QColor &color)
{
    m_geometryType = tCylinder;
    m_nrOfShapes = 1;

    if (m_visualizer->addCylinder( coefficients, m_name.toStdString() ))
    {
        vtkProp *a = getLastActor();
        syncActorProperties(a);
        m_actors.clear();
        m_actors.append(a);

        setLineColor(color);
    }

    return ito::retOk;
}

//-------------------------------------------------------------------------------------------
ito::RetVal ItemGeometry::addSphere(const pcl::PointXYZ &center, double radius, const QColor &color)
{
    m_geometryType = tSphere;
    m_nrOfShapes = 1;

    if (m_visualizer->addSphere(center, radius, color.redF(), color.greenF(), color.blueF(), m_name.toStdString()))
    {
        vtkProp *a = getLastActor();
        syncActorProperties(a);
        m_actors.clear();
        m_actors.append(a);

        m_lineColor = color;
    }

    return ito::retOk;
}

//-------------------------------------------------------------------------------------------
ito::RetVal ItemGeometry::addPolygon(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr polygon, const QColor &color)
{
    m_geometryType = tPolygon;
    m_nrOfShapes = 1;
    if (m_visualizer->addPolygon<pcl::PointXYZ>(polygon, color.redF(), color.greenF(), color.blueF(), m_name.toStdString()))
    {
        vtkProp *a = getLastActor();
        syncActorProperties(a);
        m_actors.clear();
        m_actors.append(a);

        m_lineColor = color;
    }

    return ito::retOk;
}

//-------------------------------------------------------------------------------------------
ito::RetVal ItemGeometry::addText(const QString &text, const int x, const int y, const int fontsize, const QColor &color)
{
    m_geometryType = tText;
    m_nrOfShapes = 1;

    if (m_visualizer->addText(text.toStdString(), x, y, fontsize, color.redF(), color.greenF(), color.blueF(), m_name.toStdString() ))
    {
        vtkProp *a = getLastActor();
        syncActorProperties(a);
        m_actors.clear();
        m_actors.append(a);

        m_lineColor = color;
    }

    return ito::retOk;
}

//-------------------------------------------------------------------------------------------
ito::RetVal ItemGeometry::updateText(const QString &text, const int x, const int y, const int fontsize, const QColor &color)
{
    ito::RetVal retVal;
    if (m_geometryType == tText)
    {
        if (m_visualizer->updateText(text.toStdString(), x, y, fontsize, color.redF(), color.greenF(), color.blueF(), m_name.toStdString()) == false)
        {
            retVal += ito::RetVal(ito::retError, 0, tr("error updating text").toLatin1().data());
        }
    }
    else
    {
        retVal += ito::RetVal(ito::retError, 0, tr("item was no text item before").toLatin1().data());
    }

    return retVal;
}

//-------------------------------------------------------------------------------------------
ito::RetVal ItemGeometry::addPyramid(const ito::DataObject *points, const QColor &color)
{
    m_geometryType = tPyramid;
    m_nrOfShapes = 5;

    const ito::float32 *xPtr = (ito::float32*)points->rowPtr(0,0);
    const ito::float32 *yPtr = (ito::float32*)points->rowPtr(0,1);
    const ito::float32 *zPtr = (ito::float32*)points->rowPtr(0,2);

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
    cloud->resize(3);

    (*cloud)[1] = pcl::PointXYZ(xPtr[0], yPtr[0], zPtr[0]);
    (*cloud)[0] = pcl::PointXYZ(xPtr[1], yPtr[1], zPtr[1]);
    (*cloud)[2] = pcl::PointXYZ(xPtr[4], yPtr[4], zPtr[4]);
    if (m_visualizer->addPolygon<pcl::PointXYZ>(cloud, color.redF(), color.greenF(), color.blueF(), QString("%1_0").arg(m_name).toStdString()))
    {
        vtkProp *a = getLastActor();
        syncActorProperties(a);
        m_actors.clear();
        m_actors.append(a);
    }

    (*cloud)[1] = pcl::PointXYZ(xPtr[1], yPtr[1], zPtr[1]);
    (*cloud)[0] = pcl::PointXYZ(xPtr[2], yPtr[2], zPtr[2]);
    if (m_visualizer->addPolygon<pcl::PointXYZ>(cloud, color.redF(), color.greenF(), color.blueF(), QString("%1_1").arg(m_name).toStdString()))
    {
        vtkProp *a = getLastActor();
        syncActorProperties(a);
        m_actors.append(a);
    }

    (*cloud)[0] = pcl::PointXYZ(xPtr[2], yPtr[2], zPtr[2]);
    (*cloud)[1] = pcl::PointXYZ(xPtr[3], yPtr[3], zPtr[3]);
    if (m_visualizer->addPolygon<pcl::PointXYZ>(cloud, color.redF(), color.greenF(), color.blueF(), QString("%1_2").arg(m_name).toStdString()))
    {
        vtkProp *a = getLastActor();
        syncActorProperties(a);
        m_actors.append(a);
    }

    (*cloud)[1] = pcl::PointXYZ(xPtr[3], yPtr[3], zPtr[3]);
    (*cloud)[0] = pcl::PointXYZ(xPtr[0], yPtr[0], zPtr[0]);
    if (m_visualizer->addPolygon<pcl::PointXYZ>(cloud, color.redF(), color.greenF(), color.blueF(), QString("%1_3").arg(m_name).toStdString()))
    {
        vtkProp *a = getLastActor();
        syncActorProperties(a);
        m_actors.append(a);
    }

    (*cloud)[0] = pcl::PointXYZ(xPtr[0], yPtr[0], zPtr[0]);
    (*cloud)[1] = pcl::PointXYZ(xPtr[1], yPtr[1], zPtr[1]);
    (*cloud)[2] = pcl::PointXYZ(xPtr[2], yPtr[2], zPtr[2]);
    cloud->push_back(pcl::PointXYZ(xPtr[3], yPtr[3], zPtr[3]));
    if (m_visualizer->addPolygon<pcl::PointXYZ>(cloud, color.redF(), color.greenF(), color.blueF(), QString("%1_4").arg(m_name).toStdString()))
    {
        vtkProp *a = getLastActor();
        syncActorProperties(a);
        m_actors.append(a);
    }

    m_lineColor = color;

    /*pcl::PolygonMesh mesh;
    pcl::Vertices indices;
    pcl::PointCloud<pcl::PointXYZ> cloud;
    cloud.reserve(5);
    indices.vertices.resize(3);

    for(int i = 0; i < 5; ++i)
    {
        cloud.push_back( pcl::PointXYZ( xPtr[i], yPtr[i], zPtr[i] ) );
    }

    pcl::PCLPointCloud2 msg;
    pcl::toPCLPointCloud2(cloud, msg);
    mesh.cloud = msg;

    indices.vertices[2] = 4;
    indices.vertices[0] = 0;
    indices.vertices[1] = 1;
    mesh.polygons.push_back( indices );

    indices.vertices[0] = 1;
    indices.vertices[1] = 2;
    mesh.polygons.push_back( indices );

    indices.vertices[0] = 2;
    indices.vertices[1] = 3;
    mesh.polygons.push_back( indices );

    indices.vertices[0] = 3;
    indices.vertices[1] = 0;
    mesh.polygons.push_back( indices );

    if (m_visualizer->addPolylineFromPolygonMesh( mesh, m_name.toStdString() ))
    {
        vtkProp *a = getLastActor();
        syncActorProperties(a);
        m_actors.clear();
        m_actors.append(a);

        setLineColor(color);
    }*/

    return ito::retOk;
}

//-------------------------------------------------------------------------------------------
ito::RetVal ItemGeometry::addCuboid(const ito::DataObject *points, const QColor &color)
{
    /* the points are aligned as follows:

    back:  5 - - - - 6
           |         |
           |         |
           4 - - - - 7

    front: 1 - - - - 2
           |         |
           |         |
           0 - - - - 3

    Therefore the four faces are 0,3,2,1 ; 4,5,6,7 ; 0,1,5,4 ; 2,3,7,6 (right hand, index normal towards outer side of the box)
    */

    const ito::float32 *xPtr = (ito::float32*)points->rowPtr(0,0);
    const ito::float32 *yPtr = (ito::float32*)points->rowPtr(0,1);
    const ito::float32 *zPtr = (ito::float32*)points->rowPtr(0,2);

    m_geometryType = tCuboid;
    m_nrOfShapes = 1;
    pcl::PolygonMesh mesh;
    pcl::Vertices indices;
    pcl::PointCloud<pcl::PointXYZ> cloud;
    cloud.reserve(8);
    indices.vertices.resize(5);

    for(int i = 0; i < 8; ++i)
    {
        cloud.push_back( pcl::PointXYZ( xPtr[i], yPtr[i], zPtr[i] ) );
    }

    pcl::PCLPointCloud2 msg;
    pcl::toPCLPointCloud2(cloud, msg);
    mesh.cloud = msg;

    indices.vertices[0] = 0;
    indices.vertices[1] = 3;
    indices.vertices[2] = 2;
    indices.vertices[3] = 1;
    indices.vertices[4] = 0;
    mesh.polygons.push_back( indices );

    indices.vertices[0] = 4;
    indices.vertices[1] = 5;
    indices.vertices[2] = 6;
    indices.vertices[3] = 7;
    indices.vertices[4] = 4;
    mesh.polygons.push_back( indices );

    indices.vertices[0] = 0;
    indices.vertices[1] = 1;
    indices.vertices[2] = 5;
    indices.vertices[3] = 4;
    indices.vertices[4] = 0;
    mesh.polygons.push_back( indices );

    indices.vertices[0] = 2;
    indices.vertices[1] = 3;
    indices.vertices[2] = 7;
    indices.vertices[3] = 6;
    indices.vertices[4] = 2;
    mesh.polygons.push_back( indices );

    if (m_visualizer->addPolylineFromPolygonMesh( mesh, m_name.toStdString() ))
    {
        vtkProp *a = getLastActor();
        syncActorProperties(a);
        m_actors.clear();
        m_actors.append(a);

        setLineColor(color);
    }

    return ito::retOk;
}

//-------------------------------------------------------------------------------------------
ito::RetVal ItemGeometry::addCube(const Eigen::Vector3f &size, const Eigen::Affine3f &pose, const QColor &color)
{
    m_geometryType = tCube;
    m_nrOfShapes = 1;

    Eigen::Vector3f translation(1,0,0);
    Eigen::Quaternionf rotation_(pose.rotation());

    if (m_visualizer->addCube(pose.translation(), rotation_, size.x(), size.y(), size.z(), m_name.toStdString()))
    {
        vtkProp *a = getLastActor();
        vtkActor::SafeDownCast(a)->GetProperty()->SetDiffuseColor(1.0, 0.0, 0.5);
        vtkActor::SafeDownCast(a)->GetProperty()->SetAmbientColor(0.0, 1.0, 0.5);
        syncActorProperties(a);
        m_actors.clear();
        m_actors.append(a);

        setLineColor(color);
    }

    return ito::retOk;
}


//-------------------------------------------------------------------------------------------
ito::RetVal ItemGeometry::addLines(const ito::DataObject *points, const QColor &color)
{
    m_geometryType = tLines;

    pcl::PolygonMesh mesh;
    pcl::Vertices indices;
    pcl::PointCloud<pcl::PointXYZ> cloud;
    indices.vertices.resize(2);

    ito::float32 *linePtr;
    int numLines = points->getSize(0);
    int pointsIndex = 0;

    cloud.reserve(numLines * 2);
    mesh.polygons.reserve(numLines);

    for (int idx=0; idx < numLines; ++idx)
    {
        linePtr = (ito::float32*)points->rowPtr(0,idx);

        cloud.push_back(pcl::PointXYZ(linePtr[0], linePtr[1], linePtr[2]));
        cloud.push_back(pcl::PointXYZ(linePtr[3], linePtr[4], linePtr[5]));

        indices.vertices[0] = pointsIndex ++;
        indices.vertices[1] = pointsIndex ++;
        mesh.polygons.push_back(indices);
    }

    pcl::PCLPointCloud2 msg;
    pcl::toPCLPointCloud2(cloud, msg);
    mesh.cloud = msg;

    if (m_visualizer->addPolylineFromPolygonMesh(mesh, m_name.toStdString()))
    {
        m_actors.clear();
        m_actors.append(getLastActor());
        m_visualizer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, color.redF(), color.greenF(), color.blueF(), m_name.toStdString());
        m_lineColor = color;
    }

    return ito::retOk;
}

//-------------------------------------------------------------------------------------------
void ItemGeometry::setVisible(bool value)
{
    m_visible = value;

    double val = value ? ((double)m_opacity / 100.0) : 0.0;

    if (m_geometryType == tText)
    {
        QVector<vtkProp*> props =  getSafeActors();
        vtkTextActor *ta;
        foreach (vtkProp *p, props)
        {
            ta = vtkTextActor::SafeDownCast(p);
            if (ta)
            {
                ta->SetVisibility(value);
            }
        }
    }
    else
    {
        if (m_nrOfShapes == 1)
        {
            m_visualizer->setShapeRenderingProperties( pcl::visualization::PCL_VISUALIZER_OPACITY, val, m_name.toStdString());
        }
        else
        {
            QString name;
            for (int i = 0; i < m_nrOfShapes; ++i)
            {
                name = QString("%1_%2").arg(m_name).arg(i);
                m_visualizer->setShapeRenderingProperties( pcl::visualization::PCL_VISUALIZER_OPACITY, val, name.toStdString());
            }
        }
    }

    emit updateCanvasRequest();

    Item::setVisible(value);
}


//-------------------------------------------------------------------------------------------
void ItemGeometry::setRepresentation(Representation value)
{
    int val = 0;
    if (m_geometryType != tText && m_geometryType != tCuboid && m_geometryType != tLines)
    {
        switch (int (value))
        {
            case ItemGeometry::Points:
            {
                val = pcl::visualization::PCL_VISUALIZER_REPRESENTATION_POINTS;
                break;
            }
            case ItemGeometry::Wireframe:
            {
                val = pcl::visualization::PCL_VISUALIZER_REPRESENTATION_WIREFRAME;
                break;
            }
            case ItemGeometry::Surface:
            {
                val = pcl::visualization::PCL_VISUALIZER_REPRESENTATION_SURFACE;
                break;
            }
            default:
                return;
        }

        if (m_nrOfShapes == 1)
        {
            m_visualizer->setShapeRenderingProperties( pcl::visualization::PCL_VISUALIZER_REPRESENTATION, val, m_name.toStdString());
        }
        else
        {
            QString name;
            for (int i = 0; i < m_nrOfShapes; ++i)
            {
                name = QString("%1_%2").arg(m_name).arg(i);
                m_visualizer->setShapeRenderingProperties( pcl::visualization::PCL_VISUALIZER_REPRESENTATION, val, name.toStdString());
            }
        }

        m_representation = value;

        emit updateCanvasRequest();
    }
}

//-------------------------------------------------------------------------------------------
void ItemGeometry::setInterpolation(Interpolation value)
{
    if (m_geometryType != tText && m_geometryType != tPyramid && m_geometryType != tCuboid && m_geometryType != tLines && m_geometryType != tPolygon)
    {
        int val = 0;
        switch (int (value))
        {
            case ItemGeometry::Flat:
            {
                val = pcl::visualization::PCL_VISUALIZER_SHADING_FLAT;
                break;
            }
            case ItemGeometry::Phong:
            {
                val = pcl::visualization::PCL_VISUALIZER_SHADING_PHONG;
                break;
            }
            case ItemGeometry::Gouraud:
            {
                val = pcl::visualization::PCL_VISUALIZER_SHADING_GOURAUD;
                break;
            }
            default:
                return;
        }

        if (m_nrOfShapes == 1)
        {
            m_visualizer->setShapeRenderingProperties( pcl::visualization::PCL_VISUALIZER_SHADING, val, m_name.toStdString());
        }
        else
        {
            QString name;
            for (int i = 0; i < m_nrOfShapes; ++i)
            {
                name = QString("%1_%2").arg(m_name).arg(i);
                m_visualizer->setShapeRenderingProperties( pcl::visualization::PCL_VISUALIZER_SHADING, val, name.toStdString());
            }
        }

        m_interpolation = value;

        emit updateCanvasRequest();
    }
}

//-------------------------------------------------------------------------------------------
void ItemGeometry::setLineColor(QColor color)
{
    QVector<vtkProp*> actors = getSafeActors();
    vtkActor *a;
    vtkTextActor *ta;

    foreach (vtkProp *p, actors)
    {
        a = vtkActor::SafeDownCast(p);
        if (a)
        {
            a->GetProperty()->SetColor(color.redF(), color.greenF(), color.blueF());
            a->GetProperty ()->SetEdgeColor (color.redF(), color.greenF(), color.blueF());
        }
    }

    foreach (vtkProp *p, actors)
    {
        ta = vtkTextActor::SafeDownCast(p);
        if (ta)
        {
            ta->GetProperty()->SetColor(color.redF(), color.greenF(), color.blueF());
        }
    }

    if (actors.size() == 0)
    {
        if (m_nrOfShapes == 1)
        {
            m_visualizer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, color.redF(), color.greenF(), color.blueF(), m_name.toStdString());
        }
        else
        {
            QString name;
            for (int i = 0; i < m_nrOfShapes; ++i)
            {
                name = QString("%1_%2").arg(m_name).arg(i);
                m_visualizer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, color.redF(), color.greenF(), color.blueF(), name.toStdString());
            }
        }
    }

    m_lineColor = color;
    emit updateCanvasRequest();
}

//-------------------------------------------------------------------------------------------
void ItemGeometry::setLineWidth(double value)
{
    if (m_geometryType != tText)
    {
        if (m_nrOfShapes == 1)
        {
            m_visualizer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, value, m_name.toStdString());
        }
        else
        {
            QString name;
            for (int i = 0; i < m_nrOfShapes; ++i)
            {
                name = QString("%1_%2").arg(m_name).arg(i);
                m_visualizer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, value, name.toStdString());
            }
        }

        m_lineWidth = value;

        emit updateCanvasRequest();
    }
}

//-------------------------------------------------------------------------------------------
void ItemGeometry::setOpacity(int value)
{
    value = qBound(0, value, 100);

    double v_ = (double)value / 100.0;

    if (m_geometryType == tText)
    {
        vtkTextActor *ta;
        foreach (vtkProp *p, getSafeActors())
        {
            ta = vtkTextActor::SafeDownCast(p);
            if (ta)
            {
                ta->GetProperty()->SetOpacity(v_);
            }
        }
    }
    else
    {
        if (m_nrOfShapes == 1)
        {
            m_visualizer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, v_, m_name.toStdString());
        }
        else
        {
            QString name;
            for (int i = 0; i < m_nrOfShapes; ++i)
            {
                name = QString("%1_%2").arg(m_name).arg(i);
                m_visualizer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, v_, name.toStdString());
            }
        }
    }

    m_opacity = value;

    emit updateCanvasRequest();
}

//-------------------------------------------------------------------------------------------
void ItemGeometry::setSpecular(double value)
{
    value = qBound(0.0, value, 1.0);

    if (m_geometryType != tText && m_geometryType != tCuboid && m_geometryType != tLines)
    {
        vtkActor *a;
        foreach (vtkProp *p, getSafeActors())
        {
            a = vtkActor::SafeDownCast(p);

            if (a)
            {
                a->GetProperty()->SetSpecular(value);
            }
        }

        m_specular = value;

        emit updateCanvasRequest();
    }
}

//-------------------------------------------------------------------------------------------
void ItemGeometry::setSpecularPower(double value)
{
    value = qBound(0.0, value, 1.0);

    if (m_geometryType != tText && m_geometryType != tCuboid && m_geometryType != tLines)
    {
        vtkActor *a;
        foreach (vtkProp *p, getSafeActors())
        {
            a = vtkActor::SafeDownCast(p);
            if (a)
            {
                a->GetProperty()->SetSpecularPower(value);
            }
        }

        m_specularPower = value;

        emit updateCanvasRequest();
    }
}

//-------------------------------------------------------------------------------------------
void ItemGeometry::setSpecularColor(QColor color)
{
    if (m_geometryType != tText && m_geometryType != tCuboid && m_geometryType != tLines)
    {
        vtkActor *a;
        foreach (vtkProp *p, getSafeActors())
        {
            a = vtkActor::SafeDownCast(p);
            if (a)
            {
                a->GetProperty()->SetSpecularColor(color.redF(), color.greenF(), color.blueF());
            }
        }

        m_specularColor = color;

        emit updateCanvasRequest();
    }
}

//-------------------------------------------------------------------------------------------
void ItemGeometry::setLighting(bool value)
{
    QVector<vtkProp*> actors = getSafeActors();
    vtkActor *a;

    foreach (vtkProp *p, actors)
    {
        a = vtkActor::SafeDownCast(p);
        if (a)
        {
            m_lighting = value;
            a->GetProperty()->SetLighting(value);
        }
    }

    emit updateCanvasRequest();
}


//-------------------------------------------------------------------------------------------
//void ItemPointCloud::setSelected(bool value)
//{
//    m_selected = value;
//    m_visualizer->setPointCloudSelected(value, m_name.toStdString());
//}

//-------------------------------------------------------------------------------------------
ito::RetVal ItemGeometry::updatePose(const Eigen::Affine3f &pose)
{
    if (m_nrOfShapes == 1)
    {
        if (!m_visualizer->updateShapePose(m_name.toStdString(), pose))
        {
            return ito::RetVal::format(ito::retError, 0, tr("could not set pose of geometry '%s'").toLatin1().data(), m_name.toLatin1().data());
        }
    }
    else
    {
        QString name;
        for (int i = 0; i < m_nrOfShapes; ++i)
        {
            name = QString("%1_%2").arg(m_name).arg(i);
            if (!m_visualizer->updateShapePose(m_name.toStdString(), pose))
            {
                return ito::RetVal::format(ito::retError, 0, tr("could not set pose of geometry '%s'").toLatin1().data(), name.toLatin1().data());
            }
        }
    }

    return ito::retOk;
}

//-------------------------------------------------------------------------------------------
QVector<vtkProp*> ItemGeometry::getSafeActors()
{
    vtkSmartPointer<vtkRendererCollection> _rens = m_visualizer->getRendererCollection();
    vtkRenderer *rend = m_visualizer->getRendererCollection()->GetFirstRenderer();
    vtkPropCollection *pc = rend->GetViewProps();

    QVector<vtkProp*>::iterator it = m_actors.begin();
    bool found;

    while(it != m_actors.end())
    {
        found = false;
        pc->InitTraversal();
        vtkProp *a = NULL;
        while((a =pc->GetNextProp()) != NULL)
        {
            if (a == *it)
            {
                found = true;
                break;
            }
        }

        if (!found || (*it == NULL))
        {
            it = m_actors.erase(it);
        }
        else
        {
            it++;
        }
    }

    return m_actors;
}

//-------------------------------------------------------------------------------------------
vtkProp *ItemGeometry::getLastActor()
{
    vtkSmartPointer<vtkRendererCollection> _rens = m_visualizer->getRendererCollection();
    vtkRenderer *rend = m_visualizer->getRendererCollection()->GetFirstRenderer();
    return rend->GetViewProps()->GetLastProp();
}

//-------------------------------------------------------------------------------------------
void ItemGeometry::syncActorProperties(vtkProp *prop)
{
    vtkActor *a = vtkActor::SafeDownCast (prop);
    if (a)
    {
        vtkProperty *p = a->GetProperty();
        m_lighting = p->GetLighting();
        m_specular = p->GetSpecular();
        m_specularPower = p->GetSpecularPower();
        double color[3];
        p->GetSpecularColor(color);
        m_specularColor = QColor::fromRgbF(color[0], color[1], color[2]);
    }
    else
    {
        m_lighting = false;
    }
}
