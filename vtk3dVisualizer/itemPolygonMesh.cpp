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

#include "itemPolygonMesh.h"

#include "common/apiFunctionsGraphInc.h"

#include <qdebug.h>
#include <qmetaobject.h>

//-------------------------------------------------------------------------------------------
ItemPolygonMesh::ItemPolygonMesh(boost::shared_ptr<pcl::visualization::PCLVisualizer> visualizer, const QString &name, QTreeWidgetItem *treeItem)
    : Item(name, Item::rttiMesh, treeItem),
    m_visualizer(visualizer),
    m_representation(Surface),
    m_interpolation(Flat),
    m_culling(ShowAll),
    m_edgeVisibility(false),
    m_opacity(100),
    m_colorMode(RGB),
    m_colorMap(gray),
    m_colorValueRange(ito::AutoInterval())
    //m_selected(false)
{
    apiPaletteGetColorBarName("gray", m_colorMapData);
}

//-------------------------------------------------------------------------------------------
ItemPolygonMesh::~ItemPolygonMesh()
{
    m_visualizer->removePolygonMesh( m_name.toStdString() );
}

//-------------------------------------------------------------------------------------------
ito::RetVal ItemPolygonMesh::addPolygonMesh(const ito::PCLPolygonMesh &mesh)
{
    const pcl::PolygonMesh &mesh_ = *(mesh.polygonMesh().get());

    if (mesh_.polygons.size() == 0)
    {
        return ito::RetVal(ito::retError, 0, "The polygon mesh must contain at least one polygon.");
    }

    m_visualizer->addPolygonMesh(mesh_, m_name.toStdString() );
    m_mesh = mesh;
    return ito::retOk;
}

//-------------------------------------------------------------------------------------------
void ItemPolygonMesh::setVisible(bool value)
{
    m_visible = value;

    double val = value ? (m_opacity / 100.0) : 0.0;
    m_visualizer->setPointCloudRenderingProperties( pcl::visualization::PCL_VISUALIZER_OPACITY, val, m_name.toStdString());
    emit updateCanvasRequest();

    Item::setVisible(value);
}

//-------------------------------------------------------------------------------------------
void ItemPolygonMesh::setRepresentation(Representation value)
{
//#if PCL_VERSION_COMPARE(>=,1,7,0)
//    pcl::visualization::CloudActorMap *map = m_visualizer->getInteractorStyle()->getCloudActorMap().get();
//#else
    pcl::visualization::CloudActorMap *map = m_visualizer->getCloudActorMap().get();
//#endif
    pcl::visualization::CloudActorMap::iterator it = map->find( m_name.toStdString() );

    if(it != map->end())
    {

        vtkProperty *prop = it->second.actor->GetProperty();

        switch(value)
        {
        case Points:
            prop->SetRepresentationToPoints();
            break;
        case Surface:
            prop->SetRepresentationToSurface();
            break;
        case Wireframe:
            prop->SetRepresentationToWireframe();
            break;
        }

        it->second.actor->Modified();

        m_representation = value;
    }

    emit updateCanvasRequest();
}

//-------------------------------------------------------------------------------------------
void ItemPolygonMesh::setFun(int value)
{
    pcl::visualization::CloudActorMap *map = m_visualizer->getCloudActorMap().get();
    pcl::visualization::CloudActorMap::iterator it = map->find( m_name.toStdString() );

    if(it != map->end())
    {

        vtkProperty *prop = it->second.actor->GetProperty();
        double color[] = {1.0, 0.0, 0.0};

        switch(value)
        {
        case 0:
            {
            vtkPolyData *d = (vtkPolyData*)it->second.actor->GetMapper()->GetInput();
            d->GetPointData()->SetScalars(NULL);
            //prop->BackfaceCullingOff();
            break;
            }
        case 1:
            prop->BackfaceCullingOn();
            break;
        case 2:
            prop->EdgeVisibilityOff();
            break;
        case 3:
            prop->EdgeVisibilityOn();
            break;
        case 4:
            prop->FrontfaceCullingOff();
            break;
        case 5:
            prop->FrontfaceCullingOn();
            break;
        case 6:
            prop->LightingOff();
            break;
        case 7:
            prop->LightingOn();
            break;
        case 8:
            prop->SetColor( color );
            break;
        case 9:
            prop->SetInterpolationToFlat();
            break;
        case 10:
            prop->SetInterpolationToGouraud();
            break;
        case 11:
            prop->SetInterpolationToPhong();
            break;
        }

        it->second.actor->Modified();

        m_fun = value;
    }

    emit updateCanvasRequest();
}
//
////-------------------------------------------------------------------------------------------
////void ItemPolygonMesh::setSelected(bool value)
////{
////    m_selected = value;
////    m_visualizer->setPointCloudSelected(value, m_name.toStdString());
////}



//-------------------------------------------------------------------------------------------
void ItemPolygonMesh::setInterpolation(Interpolation value)
{
    pcl::visualization::CloudActorMap *map = m_visualizer->getCloudActorMap().get();
    pcl::visualization::CloudActorMap::iterator it = map->find( m_name.toStdString() );

    if(it != map->end())
    {
        vtkProperty *prop = it->second.actor->GetProperty();

        switch(value)
        {
        case Flat:
            prop->SetInterpolationToFlat();
            break;
        case Gouraud:
            prop->SetInterpolationToGouraud();
            break;
        case Phong:
            prop->SetInterpolationToPhong();
            break;
        }

        it->second.actor->Modified();
        m_interpolation = value;
    }

    emit updateCanvasRequest();
}

//-------------------------------------------------------------------------------------------
void ItemPolygonMesh::setCulling(Culling value)
{
    pcl::visualization::CloudActorMap *map = m_visualizer->getCloudActorMap().get();
    pcl::visualization::CloudActorMap::iterator it = map->find( m_name.toStdString() );

    if(it != map->end())
    {
        vtkProperty *prop = it->second.actor->GetProperty();

        switch(value)
        {
        case ShowAll:
            prop->FrontfaceCullingOn();
            prop->BackfaceCullingOn();
            break;
        case BackfaceOnly:
            prop->BackfaceCullingOn();
            prop->FrontfaceCullingOff();
            break;
        case FrontfaceOnly:
            prop->FrontfaceCullingOn();
            prop->BackfaceCullingOff();
            break;
        }

        it->second.actor->Modified();
        m_culling = value;
    }

    emit updateCanvasRequest();
}

//-------------------------------------------------------------------------------------------
void ItemPolygonMesh::setEdgeVisibility(bool value)
{
    pcl::visualization::CloudActorMap *map = m_visualizer->getCloudActorMap().get();
    pcl::visualization::CloudActorMap::iterator it = map->find( m_name.toStdString() );

    if(it != map->end())
    {
        vtkProperty *prop = it->second.actor->GetProperty();

        if (value)
        {
            prop->EdgeVisibilityOn();
        }
        else
        {
            prop->EdgeVisibilityOff();
        }

        it->second.actor->Modified();

        m_edgeVisibility = value;
    }

    emit updateCanvasRequest();
}

//-------------------------------------------------------------------------------------------
void ItemPolygonMesh::setOpacity(int value)
{
    m_opacity = value;

    if (m_visible)
    {
        m_visualizer->setPointCloudRenderingProperties( pcl::visualization::PCL_VISUALIZER_OPACITY, (double)value / 100.0, m_name.toStdString());
        emit updateCanvasRequest();
    }
}

//-------------------------------------------------------------------------------------------
void ItemPolygonMesh::setColorMode(const ColorMode& mode)
{
    pcl::PolygonMesh::ConstPtr mesh = m_mesh.polygonMesh();

    pcl::visualization::CloudActorMap *map = m_visualizer->getCloudActorMap().get();
    pcl::visualization::CloudActorMap::iterator it = map->find( m_name.toStdString() );


    if (mesh.get() && it != map->end())
    {
        m_colorMode = mode;

        if (mode == X || mode == Y || mode == Z || mode == XY || mode == YZ || mode == XZ || mode == XYZ)
        {
            int x_idx = pcl::getFieldIndex(mesh->cloud, "x");
            int y_idx = pcl::getFieldIndex(mesh->cloud, "y");
            int z_idx = pcl::getFieldIndex(mesh->cloud, "z");

            if (x_idx >= 0 && y_idx >= 0 && z_idx >= 0)
            {
                float limits[] = {std::numeric_limits<float>::max(), std::numeric_limits<float>::min()};

                uint32_t xOffset = mesh->cloud.fields[x_idx].offset;
                uint32_t yOffset = mesh->cloud.fields[y_idx].offset;
                uint32_t zOffset = mesh->cloud.fields[z_idx].offset;
                uint32_t offset;

                float r,g,b;

                const std::vector<uchar>* cloudData = &(mesh->cloud.data);
                const uchar* cloudDataPtr = cloudData->data();

                int points = (mesh->cloud.height * mesh->cloud.width);
                int point_size = points == 0 ? 0 : static_cast<int> (cloudData->size () / points);

                vtkSmartPointer<vtkUnsignedCharArray> colors = vtkSmartPointer<vtkUnsignedCharArray>::New ();
                colors->SetNumberOfComponents (3);

                vtkProperty *prop = it->second.actor->GetProperty();
                vtkPolyData *d = (vtkPolyData*)it->second.actor->GetMapper()->GetInput();

                if (mode == X || mode == Y || mode == Z)
                {
                    offset = mode == X ? xOffset : (mode == Y ? yOffset : zOffset);

                    if (m_colorValueRange.isAuto())
                    {
                        for (int i = 0; i < points; ++i)
                        {
                            limits[0] = std::min(limits[0], *( (float*)(cloudDataPtr + i * point_size + offset)));
                            limits[1] = std::max(limits[1], *( (float*)(cloudDataPtr + i * point_size + offset)));
                        }
                        m_colorValueRange.setRange(limits[0],limits[1]);
                    }
                    else
                    {
                        limits[0] = m_colorValueRange.minimum();
                        limits[1] = m_colorValueRange.maximum();
                    }

                    for (int i = 0; i < points; ++i)
                    {
                        evalColorMap(*( (float*)(cloudDataPtr + i * point_size + offset)), r, g, b, limits);
                        colors->InsertNextTuple3( r, g, b );
                    }
                }
                else if (mode == XY)
                {
                    float v1, v2, v;

                    if (m_colorValueRange.isAuto())
                    {
                        for (int i = 0; i < points; ++i)
                        {
                            v1 = *( (float*)(cloudDataPtr + i * point_size + xOffset));
                            v2 = *( (float*)(cloudDataPtr + i * point_size + yOffset));
                            v = std::sqrt(v1*v1+v2*v2);
                            limits[0] = std::min(limits[0], v);
                            limits[1] = std::max(limits[1], v);
                        }
                        m_colorValueRange.setRange(limits[0],limits[1]);
                    }
                    else
                    {
                        limits[0] = m_colorValueRange.minimum();
                        limits[1] = m_colorValueRange.maximum();
                    }

                    for (int i = 0; i < points; ++i)
                    {
                        v1 = *( (float*)(cloudDataPtr + i * point_size + xOffset));
                        v2 = *( (float*)(cloudDataPtr + i * point_size + yOffset));
                        evalColorMap(std::sqrt(v1*v1+v2*v2), r, g, b, limits);
                        colors->InsertNextTuple3( r, g, b );
                    }
                }
                else if (mode == XZ)
                {
                    float v1, v2, v;

                    if (m_colorValueRange.isAuto())
                    {
                        for (int i = 0; i < points; ++i)
                        {
                            v1 = *( (float*)(cloudDataPtr + i * point_size + xOffset));
                            v2 = *( (float*)(cloudDataPtr + i * point_size + zOffset));
                            v = std::sqrt(v1*v1+v2*v2);
                            limits[0] = std::min(limits[0], v);
                            limits[1] = std::max(limits[1], v);
                        }
                        m_colorValueRange.setRange(limits[0],limits[1]);
                    }
                    else
                    {
                        limits[0] = m_colorValueRange.minimum();
                        limits[1] = m_colorValueRange.maximum();
                    }

                    for (int i = 0; i < points; ++i)
                    {
                        v1 = *( (float*)(cloudDataPtr + i * point_size + xOffset));
                        v2 = *( (float*)(cloudDataPtr + i * point_size + zOffset));
                        evalColorMap(std::sqrt(v1*v1+v2*v2), r, g, b, limits);
                        colors->InsertNextTuple3( r, g, b );
                    }
                }
                else if (mode == YZ)
                {
                    float v1, v2, v;

                    if (m_colorValueRange.isAuto())
                    {
                        for (int i = 0; i < points; ++i)
                        {
                            v1 = *( (float*)(cloudDataPtr + i * point_size + yOffset));
                            v2 = *( (float*)(cloudDataPtr + i * point_size + zOffset));
                            v = std::sqrt(v1*v1+v2*v2);
                            limits[0] = std::min(limits[0], v);
                            limits[1] = std::max(limits[1], v);
                        }
                        m_colorValueRange.setRange(limits[0],limits[1]);
                    }
                    else
                    {
                        limits[0] = m_colorValueRange.minimum();
                        limits[1] = m_colorValueRange.maximum();
                    }

                    for (int i = 0; i < points; ++i)
                    {
                        v1 = *( (float*)(cloudDataPtr + i * point_size + yOffset));
                        v2 = *( (float*)(cloudDataPtr + i * point_size + zOffset));
                        evalColorMap(std::sqrt(v1*v1+v2*v2), r, g, b, limits);
                        colors->InsertNextTuple3( r, g, b );
                    }
                }
                else // mode == XYZ
                {
                    float v1, v2, v3, v;

                    if (m_colorValueRange.isAuto())
                    {
                        for (int i = 0; i < points; ++i)
                        {
                            v1 = *( (float*)(cloudDataPtr + i * point_size + xOffset));
                            v2 = *( (float*)(cloudDataPtr + i * point_size + yOffset));
                            v3 = *( (float*)(cloudDataPtr + i * point_size + zOffset));
                            v = std::sqrt(v1*v1+v2*v2+v3*v3);
                            limits[0] = std::min(limits[0], v);
                            limits[1] = std::max(limits[1], v);
                        }
                        m_colorValueRange.setRange(limits[0],limits[1]);
                    }
                    else
                    {
                        limits[0] = m_colorValueRange.minimum();
                        limits[1] = m_colorValueRange.maximum();
                    }


                    for (int i = 0; i < points; ++i)
                    {
                        v1 = *( (float*)(cloudDataPtr + i * point_size + xOffset));
                        v2 = *( (float*)(cloudDataPtr + i * point_size + yOffset));
                        v3 = *( (float*)(cloudDataPtr + i * point_size + zOffset));
                        evalColorMap(std::sqrt(v1*v1+v2*v2+v3*v3), r, g, b, limits);
                        colors->InsertNextTuple3( r, g, b );
                    }
                }

                colors->SetName ("Colors");
                d->GetPointData()->SetScalars(colors);
                it->second.actor->Modified();

                emit updateCanvasRequest();
            }
        }
        else if (mode == NormalX || mode == NormalY || mode == NormalZ || mode == NormalMagnitude)
        {
            int x_idx = pcl::getFieldIndex(mesh->cloud, "normal_x");
            int y_idx = pcl::getFieldIndex(mesh->cloud, "normal_y");
            int z_idx = pcl::getFieldIndex(mesh->cloud, "normal_z");

            if (x_idx >= 0 && y_idx >= 0 && z_idx >= 0)
            {
                float limits[] = {std::numeric_limits<float>::max(), std::numeric_limits<float>::min()};

                uint32_t xOffset = mesh->cloud.fields[x_idx].offset;
                uint32_t yOffset = mesh->cloud.fields[y_idx].offset;
                uint32_t zOffset = mesh->cloud.fields[z_idx].offset;
                uint32_t offset;

                float r,g,b;

                const std::vector<uchar>* cloudData = &(mesh->cloud.data);
                const uchar* cloudDataPtr = cloudData->data();

                int points = (mesh->cloud.height * mesh->cloud.width);
                int point_size = points == 0 ? 0 : static_cast<int> (cloudData->size () / points);

                vtkSmartPointer<vtkUnsignedCharArray> colors = vtkSmartPointer<vtkUnsignedCharArray>::New ();
                colors->SetNumberOfComponents (3);

                vtkProperty *prop = it->second.actor->GetProperty();
                vtkPolyData *d = (vtkPolyData*)it->second.actor->GetMapper()->GetInput();

                if (mode == NormalX || mode == NormalY || mode == NormalZ)
                {
                    offset = mode == NormalX ? xOffset : (mode == NormalY ? yOffset : zOffset);

                    if (m_colorValueRange.isAuto())
                    {
                        for (int i = 0; i < points; ++i)
                        {
                            limits[0] = std::min(limits[0], *( (float*)(cloudDataPtr + i * point_size + offset)));
                            limits[1] = std::max(limits[1], *( (float*)(cloudDataPtr + i * point_size + offset)));
                        }
                    }
                    else
                    {
                        limits[0] = m_colorValueRange.minimum();
                        limits[1] = m_colorValueRange.maximum();
                    }

                    for (int i = 0; i < points; ++i)
                    {
                        evalColorMap(*( (float*)(cloudDataPtr + i * point_size + offset)), r, g, b, limits);
                        colors->InsertNextTuple3( r, g, b );
                    }
                }
                else // mode == NormalMagnitude
                {
                    float v1, v2, v3, v;

                    if (m_colorValueRange.isAuto())
                    {
                        for (int i = 0; i < points; ++i)
                        {
                            v1 = *( (float*)(cloudDataPtr + i * point_size + xOffset));
                            v2 = *( (float*)(cloudDataPtr + i * point_size + yOffset));
                            v3 = *( (float*)(cloudDataPtr + i * point_size + zOffset));
                            v = std::sqrt(v1*v1+v2*v2+v3*v3);
                            limits[0] = std::min(limits[0], v);
                            limits[1] = std::max(limits[1], v);
                        }
                    }
                    else
                    {
                        limits[0] = m_colorValueRange.minimum();
                        limits[1] = m_colorValueRange.maximum();
                    }

                    for (int i = 0; i < points; ++i)
                    {
                        v1 = *( (float*)(cloudDataPtr + i * point_size + xOffset));
                        v2 = *( (float*)(cloudDataPtr + i * point_size + yOffset));
                        v3 = *( (float*)(cloudDataPtr + i * point_size + zOffset));
                        evalColorMap(std::sqrt(v1*v1+v2*v2+v3*v3), r, g, b, limits);
                        colors->InsertNextTuple3( r, g, b );
                    }
                }

                colors->SetName ("Colors");
                d->GetPointData()->SetScalars(colors);
                it->second.actor->Modified();

                emit updateCanvasRequest();
            }
        }
        else if (mode == RGB)
        {
            //check whether RGBA type is available and return its index
            int rgba_idx = pcl::getFieldIndex(mesh->cloud, "rgba");
            if (rgba_idx == -1) rgba_idx = pcl::getFieldIndex(mesh->cloud, "rgb"); //both types are mixed up in PCL < 2.0


            if (rgba_idx == -1)
            {
                setColorMode( SolidColor );
            }
            else
            {
                uint32_t fieldOffset = mesh->cloud.fields[rgba_idx].offset;

                vtkSmartPointer<vtkUnsignedCharArray> colors = vtkSmartPointer<vtkUnsignedCharArray>::New ();
                colors->SetNumberOfComponents (3);

                pcl::PointXYZRGBA rgba;
                const std::vector<uchar>* cloudData = &(mesh->cloud.data);
                const uchar* cloudDataPtr = cloudData->data();

                int points = (mesh->cloud.height * mesh->cloud.width);
                int point_size = points == 0 ? 0 : static_cast<int> (cloudData->size () / points);

                for (int i = 0; i < points; ++i)
                {
                    rgba.rgba = *( (uint32_t*)(cloudDataPtr + i * point_size + fieldOffset ) ); //this strange assignment to a point is easier, since order of rgb is not always the same
                    colors->InsertNextTuple3( rgba.r, rgba.g, rgba.b );
                }

                vtkProperty *prop = it->second.actor->GetProperty();
                vtkPolyData *d = (vtkPolyData*)it->second.actor->GetMapper()->GetInput();

                colors->SetName ("Colors");
                d->GetPointData()->SetScalars(colors);
                it->second.actor->Modified();

                emit updateCanvasRequest();
            }
        }
		else if (mode == Curvature)
		{
			int c_idx = pcl::getFieldIndex(mesh->cloud, "curvature");

			if (c_idx >= 0)
			{
				float limits[] = { std::numeric_limits<float>::max(), std::numeric_limits<float>::min() };

				uint32_t cOffset = mesh->cloud.fields[c_idx].offset;

				float r, g, b;

				const std::vector<uchar>* cloudData = &(mesh->cloud.data);
				const uchar* cloudDataPtr = cloudData->data();

				int points = (mesh->cloud.height * mesh->cloud.width);
				int point_size = points == 0 ? 0 : static_cast<int> (cloudData->size() / points);

				vtkSmartPointer<vtkUnsignedCharArray> colors = vtkSmartPointer<vtkUnsignedCharArray>::New();
				colors->SetNumberOfComponents(3);

				vtkProperty *prop = it->second.actor->GetProperty();
				vtkPolyData *d = (vtkPolyData*)it->second.actor->GetMapper()->GetInput();

				if (m_colorValueRange.isAuto())
				{
					for (int i = 0; i < points; ++i)
					{
						limits[0] = std::min(limits[0], *((float*)(cloudDataPtr + i * point_size + cOffset)));
						limits[1] = std::max(limits[1], *((float*)(cloudDataPtr + i * point_size + cOffset)));
					}
				}
				else
				{
					limits[0] = m_colorValueRange.minimum();
					limits[1] = m_colorValueRange.maximum();
				}

				for (int i = 0; i < points; ++i)
				{
					evalColorMap(*((float*)(cloudDataPtr + i * point_size + cOffset)), r, g, b, limits);
					colors->InsertNextTuple3(r, g, b);
				}

				colors->SetName("Colors");
				d->GetPointData()->SetScalars(colors);
				it->second.actor->Modified();

				emit updateCanvasRequest();
			}
		}
        else if(mode == SolidColor)
        {
            setFaceColor(m_faceColor);
        }
    }

}


//-------------------------------------------------------------------------------------------
void ItemPolygonMesh::setFaceColor(const QColor& color)
{
    m_faceColor = color;

    if (m_colorMode == SolidColor)
    {

        pcl::visualization::CloudActorMap *map = m_visualizer->getCloudActorMap().get();
        pcl::visualization::CloudActorMap::iterator it = map->find( m_name.toStdString() );

        if(it != map->end())
        {
            vtkProperty *prop = it->second.actor->GetProperty();
            unsigned char ucharColor[] = {static_cast<unsigned char>(color.red()), static_cast<unsigned char>(color.green()), static_cast<unsigned char>(color.blue())};

            vtkPolyData *d = (vtkPolyData*)it->second.actor->GetMapper()->GetInput();

            vtkSmartPointer<vtkUnsignedCharArray> colors = vtkSmartPointer<vtkUnsignedCharArray>::New ();
            colors->SetNumberOfComponents (3);

            int nrOfPoints = m_mesh.polygonMesh()->cloud.height * m_mesh.polygonMesh()->cloud.width;
            for (int i = 0; i < nrOfPoints; i++)
            {
                #if (VTK_MAJOR_VERSION < 7) || (VTK_MAJOR_VERSION==7 && VTK_MINOR_VERSION==0) //VTK version < 7 the name is changed in newer versions of VTK. https://github.com/PointCloudLibrary/pcl/issues/2060
                    colors->InsertNextTupleValue(ucharColor);
                #else
                    colors->InsertNextTypedTuple(ucharColor);
                #endif
            }

            colors->SetName ("SolidColor");
            d->GetPointData()->SetScalars(colors);
            it->second.actor->Modified();
        }

        emit updateCanvasRequest();
    }
}

//-------------------------------------------------------------------------------------------
void ItemPolygonMesh::setColorMap(const ColorMap& map)
{
    int idx = metaObject()->indexOfEnumerator("ColorMap");
    QString mapString = metaObject()->enumerator(idx).key(map);
    ito::RetVal ret = apiPaletteGetColorBarName(mapString, m_colorMapData);

    if (!ret.containsError())
    {
        m_colorMap = map;
        setColorMode(m_colorMode); //in order to redraw
    }
}

//-------------------------------------------------------------------------------------------
void ItemPolygonMesh::setColorValueRange(const ito::AutoInterval& range)
{
    m_colorValueRange = range;
    setColorMode(m_colorMode); //in order to redraw
}

//-------------------------------------------------------------------------------------------------
void ItemPolygonMesh::evalColorMap(float value, float &red, float &green, float &blue, const float *limits)
{
    if (m_colorMapData.type | ito::tPaletteIndexed)
    {
        if (limits)
        {
            value = (value - limits[0]) / (limits[1] - limits[0]);
        }

        QColor c = m_colorMapData.colorVector256[qBound<int>(0, qRound(value*255.0), 255)];

        red   = c.red();
        green = c.green();
        blue  = c.blue();
    }
}
