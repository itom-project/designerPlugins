#include "itemGeometry.h"

#include "vtkPolyLine.h"

//-------------------------------------------------------------------------------------------
ItemGeometry::ItemGeometry(boost::shared_ptr<pcl::visualization::PCLVisualizer> visualizer, const QString &name, QTreeWidgetItem *treeItem)
    : Item(name, treeItem),
    m_visualizer(visualizer),
    m_representation(Wireframe),
    m_lineWidth(1.0),
    m_opacity(1.0)
    //m_selected(false)
{
    m_type = "geometry";
}

//-------------------------------------------------------------------------------------------
ItemGeometry::~ItemGeometry()
{
    m_visualizer->removeShape( m_name.toStdString() );
}

//-------------------------------------------------------------------------------------------
ito::RetVal ItemGeometry::addCylinder(const pcl::ModelCoefficients &coefficients, const QColor &color)
{
    m_geometryType = tCylinder;

    if (m_visualizer->addCylinder( coefficients, m_name.toStdString() ))
    {
        m_visualizer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, color.redF(), color.greenF(), color.blueF(), m_name.toStdString());
        m_lineColor = color;
    }

    return ito::retOk;
}

//-------------------------------------------------------------------------------------------
ito::RetVal ItemGeometry::addPyramid(const ito::DataObject *points, const QColor &color)
{
    m_geometryType = tPyramid;

    const ito::float32 *xPtr = (ito::float32*)points->rowPtr(0,0);
    const ito::float32 *yPtr = (ito::float32*)points->rowPtr(0,1);
    const ito::float32 *zPtr = (ito::float32*)points->rowPtr(0,2);

    pcl::PolygonMesh mesh;
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
        m_visualizer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, color.redF(), color.greenF(), color.blueF(), m_name.toStdString());
        m_lineColor = color;
    }

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

    m_geometryType = tCuboid;

    const ito::float32 *xPtr = (ito::float32*)points->rowPtr(0,0);
    const ito::float32 *yPtr = (ito::float32*)points->rowPtr(0,1);
    const ito::float32 *zPtr = (ito::float32*)points->rowPtr(0,2);

    pcl::PolygonMesh mesh;
    pcl::Vertices indices;
    pcl::PointCloud<pcl::PointXYZ> cloud;
    cloud.reserve(5);
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
        m_visualizer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, color.redF(), color.greenF(), color.blueF(), m_name.toStdString());
        m_lineColor = color;
    }

    return ito::retOk;
}

//-------------------------------------------------------------------------------------------
ito::RetVal ItemGeometry::addLines(const ito::DataObject *points, const QColor &color)
{
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
        m_visualizer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, color.redF(), color.greenF(), color.blueF(), m_name.toStdString());
        m_lineColor = color;
    }

    return ito::retOk;
}

//-------------------------------------------------------------------------------------------
void ItemGeometry::setVisible(bool value)
{
    m_visible = value;

    double val = value ? 1.0 : 0.0;
    m_visualizer->setShapeRenderingProperties( pcl::visualization::PCL_VISUALIZER_OPACITY, val, m_name.toStdString());
    emit updateCanvasRequest();

    Item::setVisible(value);
}


//-------------------------------------------------------------------------------------------
void ItemGeometry::setRepresentation(Representation value)
{
    int val = 0;
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


    m_visualizer->setShapeRenderingProperties( pcl::visualization::PCL_VISUALIZER_OPACITY, val, m_name.toStdString());
    m_representation = value;

    emit updateCanvasRequest();
}

//-------------------------------------------------------------------------------------------
void ItemGeometry::setLineColor(QColor color)
{
    m_visualizer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, color.redF(), color.greenF(), color.blueF(), m_name.toStdString());
    m_lineColor = color;
    emit updateCanvasRequest();
}

//-------------------------------------------------------------------------------------------
void ItemGeometry::setLineWidth(double value)
{
    if (m_visualizer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, value, m_name.toStdString()))
    {
        m_lineWidth = value;
    }
    emit updateCanvasRequest();
}

//-------------------------------------------------------------------------------------------
void ItemGeometry::setOpacity(double value)
{
    if (m_visualizer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, value, m_name.toStdString()))
    {
        m_opacity = value;
    }
    emit updateCanvasRequest();
}


//-------------------------------------------------------------------------------------------
//void ItemPointCloud::setSelected(bool value)
//{
//    m_selected = value;
//    m_visualizer->setPointCloudSelected(value, m_name.toStdString());
//}