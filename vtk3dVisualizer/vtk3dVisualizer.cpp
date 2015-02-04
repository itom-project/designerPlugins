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

#include "vtk3DVisualizer.h"

#include "CustomTypes.h"

#include "pcl/point_cloud.h"
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>

#include <pcl/visualization/point_cloud_handlers.h>
#include <pcl/filters/filter.h>
#include "pcl/search/kdtree.h"
#include "pcl/octree/octree.h"
#include <qsharedpointer.h>
#include <qcolor.h>
#include <qdebug.h>

#include "QVTKWidget.h"

#include "itemCanvas.h"

#include "vtkSmartPointer.h"

#include "QPropertyEditor/QPropertyEditorWidget.h"

#include "itemGeometry.h"
#include "itemCanvas.h"
#include "itemPolygonMesh.h"
#include "itemPointCloud.h"
#include "itemPointCloudNormal.h"
#include "item.h"

#include "ui_vtk3dVisualizer.h"

#include "common/apiFunctionsInc.h"

#if PCL_VERSION_COMPARE(>=,1,7,0)
	#include <vtkRenderWindow.h>
#endif

Q_DECLARE_METATYPE ( SharedItemPtr )

class Vtk3dVisualizerPrivate 
{
public:
    Vtk3dVisualizerPrivate() : 
        propertyWidget(NULL),
        treeWidget(NULL),
        canvasItem(NULL),
        meshItem(NULL),
        cloudItem(NULL),
        geometryItem(NULL),
        pointPickSphereRadius(0.0),
        pointPickSphereColor(QColor(255,0,0)),
        pointPickSearch(pcl::octree::OctreePointCloudSearch<pcl::PointXYZ>(1.0)),
        pointPickSearchNormal(pcl::octree::OctreePointCloudSearch<pcl::PointNormal>(1.0)),
        pointPickSearchHasNormals(false)
    {}
    
    double pointPickSphereRadius;
    QColor pointPickSphereColor;
    std::string pointPickSphereName;
    std::string pointPickArrowName;
    boost::signals2::connection pointPickConnection;

    QPropertyEditorWidget *propertyWidget;
    boost::shared_ptr<pcl::visualization::PCLVisualizer> PCLVis;

    QTreeWidget *treeWidget;
    QTreeWidgetItem *canvasItem;
    QTreeWidgetItem *meshItem;
    QTreeWidgetItem *cloudItem;
    QTreeWidgetItem *geometryItem;

    pcl::octree::OctreePointCloudSearch<pcl::PointXYZ> pointPickSearch;
    pcl::octree::OctreePointCloudSearch<pcl::PointNormal> pointPickSearchNormal;
    bool pointPickSearchHasNormals;
    Ui::Vtk3dVisualizer ui;
};



Vtk3dVisualizer::Vtk3dVisualizer(const QString &itomSettingsFile, AbstractFigure::WindowMode windowMode, QWidget *parent) :
    AbstractDObjFigure(itomSettingsFile, windowMode, parent),
    d(NULL)
{
    d = new Vtk3dVisualizerPrivate();

    d->ui.setupUi(this);

    //pcl::visualization::PCLVisualizer pviz ("PCLVisualizer", false);
    d->PCLVis = boost::shared_ptr<pcl::visualization::PCLVisualizer>(new pcl::visualization::PCLVisualizer("PCLVisualizer", false) );

	/*Eigen::Vector4f v1;
	Eigen::Quaternion<float,0> q;
	vtkSmartPointer<vtkMatrix4x4> m;
	pcl::visualization::PCLVisualizer::convertToVtkMatrix(v1, q, m);*/

    vtkSmartPointer<vtkRenderWindow> win = d->PCLVis->getRenderWindow();

    win->SetStereoCapableWindow(1);
    ////win->StereoCapableWindowOn();
    //
    //win->SetStereoTypeToCrystalEyes();
    //win->SetStereoTypeToAnaglyph();
    win->StereoRenderOff();
    ////win->StereoUpdate();

    d->ui.pclCanvas->SetRenderWindow(win); //pviz.getRenderWindow());
    QVTKInteractor *interactor = d->ui.pclCanvas->GetInteractor();
    

    d->PCLVis->setupInteractor(interactor, d->ui.pclCanvas->GetRenderWindow());
    d->PCLVis->getInteractorStyle()->setKeyboardModifier(pcl::visualization::INTERACTOR_KB_MOD_SHIFT);
    d->PCLVis->getRenderWindow()->Render(); //wichtig, dass dieser befehl vor dem ersten Hinzufügen von Elementen oder setzen von visuellen Eigenschaften kommt, da sonst addPointCloud crashed, alternativ kann auch setBackgroundColor gerufen werden, aber das ruft intern auch render() auf.

    if (interactor->HasObserver(vtkCommand::ExitEvent))
    {
        interactor->RemoveObservers(vtkCommand::ExitEvent);
    }
    

    //win->SetStereoTypeToRedBlue();
    //win->SetStereoTypeToCrystalEyes();
    //win->SetStereoTypeToRedBlue();
    //win->SetStereoTypeToInterlaced();
    //win->SetStereoTypeToDresden();

    //m_pPCLVis = boost::shared_ptr<pcl::visualization::PCLVisualizer>(new pcl::visualization::PCLVisualizer("PCLVisualization") );

    d->PCLVis->initCameraParameters ();
    d->PCLVis->setCameraPosition(5.0, -8.0, 4.0, -0.5, -1.0, 1.0);

    //prepare QPropertyEditor for visualization
    CustomTypes::registerTypes();
    d->propertyWidget = new QPropertyEditorWidget(this);
    d->ui.dockSettings->setWidget(d->propertyWidget);
    d->propertyWidget->registerCustomPropertyCB( CustomTypes::createCustomProperty);

    //create canvas item
    d->canvasItem = new QTreeWidgetItem();

    SharedItemPtr canvas( new ItemCanvas(d->PCLVis, d->canvasItem) );
    ItemCanvas *c = (ItemCanvas*)canvas.data();
    connect(c, SIGNAL(updateCanvasRequest()), d->ui.pclCanvas, SLOT(update()));
    c->setBackgroundColor(QColor(0,0,0));
    c->setCoordSysVisible(true);

    d->canvasItem->setData(0, Qt::DisplayRole, "canvas");
    d->canvasItem->setData(0, Item::itemRole, QVariant::fromValue(canvas) );
    d->ui.treeWidget->addTopLevelItem( d->canvasItem );

    //create root for meshes
    d->meshItem = new QTreeWidgetItem();
    SharedItemPtr i1 = QSharedPointer<Item>( new Item("mesh",d->meshItem) );
    connect(i1.data(), SIGNAL(updateCanvasRequest()), d->ui.pclCanvas, SLOT(update()));
    d->meshItem->setData(0, Qt::DisplayRole, "mesh");
    d->meshItem->setData(0, Item::itemRole, QVariant::fromValue(i1) );
    d->ui.treeWidget->addTopLevelItem( d->meshItem );

    //create root for point clouds
    d->cloudItem = new QTreeWidgetItem();
    SharedItemPtr i2 = QSharedPointer<Item>( new Item("clouds",d->cloudItem) );
    connect(i2.data(), SIGNAL(updateCanvasRequest()), d->ui.pclCanvas, SLOT(update()));
    d->cloudItem->setData(0, Qt::DisplayRole, "clouds");
    d->cloudItem->setData(0, Item::itemRole, QVariant::fromValue(i2) );
    d->ui.treeWidget->addTopLevelItem( d->cloudItem );

    //create root for geometries
    d->geometryItem = new QTreeWidgetItem();
    SharedItemPtr i3 = QSharedPointer<Item>( new Item("geometries",d->geometryItem) );
    connect(i3.data(), SIGNAL(updateCanvasRequest()), d->ui.pclCanvas, SLOT(update()));
    d->geometryItem->setData(0, Qt::DisplayRole, "geometries");
    d->geometryItem->setData(0, Item::itemRole, QVariant::fromValue(i3) );
    d->ui.treeWidget->addTopLevelItem( d->geometryItem );

    connect(d->ui.treeWidget, SIGNAL(itemClicked(QTreeWidgetItem*,int)), this, SLOT(itemClicked(QTreeWidgetItem*,int)));

    d->ui.pclCanvas->update();
}

//-------------------------------------------------------------------------------------
Vtk3dVisualizer::~Vtk3dVisualizer()
{
    //this timerEvent must be removed, else crashes can occure in some situations if visualization is already destroyed 
    //and the timer event is fired afterwards.
    d->ui.pclCanvas->GetInteractor()->RemoveObservers(vtkCommand::TimerEvent);

    d->PCLVis->getInteractorStyle()->SetEnabled(0);

    d->propertyWidget = NULL;

    delete d;
}

//-------------------------------------------------------------------------------------
ito::RetVal Vtk3dVisualizer::applyUpdate()
{
    return ito::retOk;
}

//-------------------------------------------------------------------------------------
ito::RetVal Vtk3dVisualizer::init()
{
    return ito::retOk;
}

//-------------------------------------------------------------------------------------
ito::RetVal Vtk3dVisualizer::createRecursiveTree(QString &path, QTreeWidgetItem *currentParent, QTreeWidgetItem **newParent)
{
    Q_ASSERT(newParent != NULL);
    Q_ASSERT(currentParent != NULL);

    ito::RetVal retval;
    QStringList pathSplit = path.split("/");
    QTreeWidgetItem *item = NULL;
    SharedItemPtr _item;

    Q_ASSERT(pathSplit.size() > 0);

    if(pathSplit.size() == 1)
    {
        *newParent = currentParent;
        path = pathSplit[0];
    }
    else
    {
        QString first = pathSplit[0];
        pathSplit.removeFirst();
        QString others = pathSplit.join("/");
        bool found = false;

        //check whether first from pathSplit already exists as child of currentParent
        for (int i = 0; i < currentParent->childCount() && !found; ++i)
        {
            item = currentParent->child(i);
            _item = item->data(0, Item::itemRole).value<SharedItemPtr>();
            if (_item->name() == first)
            {
                found = true;
                retval += createRecursiveTree(others, item, newParent);
                path = others;
            }
        }

        if (!found) //create new item
        {
            item = new QTreeWidgetItem();
            _item = SharedItemPtr(new Item(first,item));
            item->setData(0, Qt::DisplayRole, first);
            item->setData(0, Item::itemRole, QVariant::fromValue(_item) );
            currentParent->addChild(item);
            retval += createRecursiveTree(others, item, newParent);
            path = others;
        }
    }

    return retval;
}

//-------------------------------------------------------------------------------------
ito::RetVal Vtk3dVisualizer::searchRecursiveTree(const QString &path, QTreeWidgetItem *currentParent, QTreeWidgetItem **item)
{
    Q_ASSERT(item != NULL);
    Q_ASSERT(currentParent != NULL);

    ito::RetVal retval;
    QStringList pathSplit = path.split("/");
    QTreeWidgetItem *temp = NULL;

    if(pathSplit.size() == 0 || path == "")
    {
        *item = currentParent;
    }
    else
    {
        QString first = pathSplit[0];
        pathSplit.removeFirst();
        QString others = pathSplit.join("/");
        bool found = false;

        //check whether first from pathSplit already exists as child of currentParent
        for (int i = 0; i < currentParent->childCount() && !found; ++i)
        {
            temp = currentParent->child(i);
            if (temp->data(0, Qt::DisplayRole) == first)
            {
                found = true;
                retval += searchRecursiveTree(others, temp, item);
                break;
            }
        }

        if (!found) //create new item
        {
            retval += ito::RetVal(ito::retError,0,"item not found.");
        }
    }

    return retval;
}

//-------------------------------------------------------------------------------------
bool Vtk3dVisualizer::propertiesSorted() const
{
    if (d->propertyWidget)
    {
        return d->propertyWidget->sorted();
    }
    return false;
}

//-------------------------------------------------------------------------------------
void Vtk3dVisualizer::setPropertiesSorted(bool value)
{
    if (d->propertyWidget)
    {
        d->propertyWidget->setSorted(value);
    }
}

//-------------------------------------------------------------------------------------
void Vtk3dVisualizer::setPointPickSphereRadius(double radius)
{
    d->pointPickSphereRadius = radius;
}

//-------------------------------------------------------------------------------------
double Vtk3dVisualizer::pointPickSphereRadius() const 
{ 
    return d->pointPickSphereRadius; 
}

//-------------------------------------------------------------------------------------
void Vtk3dVisualizer::setpointPickSphereColor(QColor color)
{
    d->pointPickSphereColor = color;
}

//-------------------------------------------------------------------------------------
QColor Vtk3dVisualizer::pointPickSphereColor() const 
{ 
    return d->pointPickSphereColor; 
}


//-------------------------------------------------------------------------------------
ito::RetVal Vtk3dVisualizer::addPointCloud(ito::PCLPointCloud pc, const QString &fullname)
{
    ito::RetVal retval;
    QTreeWidgetItem *parent;
    QString name = fullname;
    retval += createRecursiveTree(name, d->cloudItem, &parent);

    if (!retval.containsError())
    {
        //check if item already exists with this name
        QTreeWidgetItem *item = NULL;
        for (int i = 0; i < parent->childCount(); i++)
        {
            if (parent->child(i)->data(0, Qt::ToolTipRole) == fullname)
            {
                item = parent->child(i);
                break;
            }
        }

        if (!item)
        {
            item = new QTreeWidgetItem();
            item->setData(0, Qt::DisplayRole, name);
            item->setData(0, Qt::ToolTipRole, fullname);
            parent->addChild(item);
        }

        SharedItemPtr i;

        if (pc.hasNormal())
        {
            i = SharedItemPtr(new ItemPointCloudNormal(d->PCLVis, fullname, item));
            retval += ((ItemPointCloudNormal*)(i.data()))->addPointCloud(pc);
        }
        else
        {
            i = SharedItemPtr(new ItemPointCloud(d->PCLVis, fullname, item));
            retval += ((ItemPointCloud*)(i.data()))->addPointCloud(pc);
        }

        connect(i.data(), SIGNAL(updateCanvasRequest()), d->ui.pclCanvas, SLOT(update()));
        item->setData(0, Item::itemRole, QVariant::fromValue(i));

    }

    d->ui.pclCanvas->update();

    return retval;
}

//-------------------------------------------------------------------------------------
ito::RetVal Vtk3dVisualizer::addPointCloudNormal(ito::PCLPointCloud pcl, const QString &fullname)
{
    ito::RetVal retval;
    QTreeWidgetItem *parent;
    QString name = fullname;
    retval += createRecursiveTree(name, d->cloudItem, &parent);

    if (!retval.containsError())
    {
        //check if item already exists with this name
        QTreeWidgetItem *item = NULL;
        for (int i = 0; i < parent->childCount(); i++)
        {
            if (parent->child(i)->data(0, Qt::ToolTipRole) == fullname)
            {
                item = parent->child(i);
                break;
            }
        }

        SharedItemPtr i;
        
        if(pcl.getType() == ito::pclXYZNormal || pcl.getType() == ito::pclXYZINormal || pcl.getType() == ito::pclXYZRGBNormal)
        {
            if (!item)
            {
                item = new QTreeWidgetItem();
                item->setData(0, Qt::DisplayRole, name);
                item->setData(0, Qt::ToolTipRole, fullname);
                parent->addChild(item);
            }

            //ready
            i = SharedItemPtr(new ItemPointCloudNormal(d->PCLVis, fullname, item));
            retval += ((ItemPointCloudNormal*)(i.data()))->addPointCloud(pcl);
        }
        else
        {
            retval += ito::RetVal(ito::retError,0,"the given point cloud has no normal component");
        }

        if (!retval.containsError())
        {
            connect(i.data(), SIGNAL(updateCanvasRequest()), d->ui.pclCanvas, SLOT(update()));
            item->setData(0, Item::itemRole, QVariant::fromValue(i));
        }

    }

    d->ui.pclCanvas->update();

    return retval;
}

//-------------------------------------------------------------------------------------
ito::RetVal Vtk3dVisualizer::updatePointCloud(ito::PCLPointCloud pcl, const QString &name, bool createIfNotExists /*= false*/)
{
    ito::RetVal retval;
    QTreeWidgetItem *item = NULL;
    QString n = name;
    bool found = false;
    retval += searchRecursiveTree(name, d->cloudItem, &item);

    if (!retval.containsError())
    {
        SharedItemPtr i = item->data(0, Item::itemRole).value<SharedItemPtr>();

        if (i.data())
        {
            ItemPointCloud *ipc = (ItemPointCloud*)(i.data());
            retval += ipc->updatePointCloud(pcl);
        }
    }
    else if (createIfNotExists)
    {
        retval = addPointCloud(pcl, name); //retval is assigned, no += since not-found error from above should be handled.
    }

    return retval;
}

//-------------------------------------------------------------------------------------
ito::RetVal Vtk3dVisualizer::addCylinder(QVector<double> point, QVector<double> orientation, double radius, const QString &fullname, const QColor &color /*= Qt::white*/)
{
    ito::RetVal retval;
    pcl::ModelCoefficients coefficients;
    coefficients.values.resize(7);

    if(point.size() != 3) return ito::RetVal(ito::retError);
    if(orientation.size() != 3) return ito::RetVal(ito::retError);

    coefficients.values[0] = point[0];
    coefficients.values[1] = point[1];
    coefficients.values[2] = point[2];

    coefficients.values[3] = orientation[0];
    coefficients.values[4] = orientation[1];
    coefficients.values[5] = orientation[2];

    coefficients.values[6] = radius;

    QTreeWidgetItem *parent;
    QString name = fullname;

    retval += createRecursiveTree(name, d->geometryItem, &parent);

    if (!retval.containsError())
    {
        //check if item already exists with this name
        QTreeWidgetItem *item = NULL;
        for (int i = 0; i < parent->childCount(); i++)
        {
            if (parent->child(i)->data(0, Qt::ToolTipRole) == fullname)
            {
                item = parent->child(i);
                break;
            }
        }

        if (!item)
        {
            item = new QTreeWidgetItem();
            item->setData(0, Qt::DisplayRole, name);
            item->setData(0, Qt::ToolTipRole, fullname);
            parent->addChild(item);
        }        
        SharedItemPtr i = SharedItemPtr(new ItemGeometry(d->PCLVis, fullname, item));
        item->setData(0, Item::itemRole, QVariant::fromValue(i)); //add it before adding any VTK or PCL geometry such that possible existing item, previously stored in the same user data, is deleted.
        connect(i.data(), SIGNAL(updateCanvasRequest()), d->ui.pclCanvas, SLOT(update()));
        ((ItemGeometry*)(i.data()))->addCylinder(coefficients, color);
    }

    d->ui.pclCanvas->update();

    return retval;
}

//-------------------------------------------------------------------------------------
ito::RetVal Vtk3dVisualizer::addPyramid(const ito::DataObject &points, const QString &fullname, const QColor &color /*= Qt::white*/) //points must be a 3x5 float matrix
{
    ito::RetVal retval;
    int sizes[] = {3,3,5,5};
    ito::DataObject *points2 = apiCreateFromDataObject(&points, 2, ito::tFloat32, sizes, &retval);

    if (!retval.containsError())
    {
        QTreeWidgetItem *parent;
        QString name = fullname;

        retval += createRecursiveTree(name, d->geometryItem, &parent);

        if (!retval.containsError())
        {
            //check if item already exists with this name
            QTreeWidgetItem *item = NULL;
            for (int i = 0; i < parent->childCount(); i++)
            {
                if (parent->child(i)->data(0, Qt::ToolTipRole) == fullname)
                {
                    item = parent->child(i);
                    break;
                }
            }

            if (!item)
            {
                item = new QTreeWidgetItem();
                item->setData(0, Qt::DisplayRole, name);
                item->setData(0, Qt::ToolTipRole, fullname);
                parent->addChild(item);
            }        
            SharedItemPtr i = SharedItemPtr(new ItemGeometry(d->PCLVis, fullname, item));
            item->setData(0, Item::itemRole, QVariant::fromValue(i)); //add it before adding any VTK or PCL geometry such that possible existing item, previously stored in the same user data, is deleted.
            connect(i.data(), SIGNAL(updateCanvasRequest()), d->ui.pclCanvas, SLOT(update()));
            ((ItemGeometry*)(i.data()))->addPyramid(points2, color);
        }

        d->ui.pclCanvas->update();
    }

    if (points2) delete points2;

    return retval;
}

//-------------------------------------------------------------------------------------
ito::RetVal Vtk3dVisualizer::addCuboid(const ito::DataObject &points, const QString &fullname, const QColor &color /*= Qt::white*/)  //points must be a 3x8 float matrix
{
    ito::RetVal retval;
    int sizes[] = {3,3,8,8};
    ito::DataObject *points2 = apiCreateFromDataObject(&points, 2, ito::tFloat32, sizes, &retval);

    if (!retval.containsError())
    {
        QTreeWidgetItem *parent;
        QString name = fullname;

        retval += createRecursiveTree(name, d->geometryItem, &parent);

        if (!retval.containsError())
        {
            //check if item already exists with this name
            QTreeWidgetItem *item = NULL;
            for (int i = 0; i < parent->childCount(); i++)
            {
                if (parent->child(i)->data(0, Qt::ToolTipRole) == fullname)
                {
                    item = parent->child(i);
                    break;
                }
            }

            if (!item)
            {
                item = new QTreeWidgetItem();
                item->setData(0, Qt::DisplayRole, name);
                item->setData(0, Qt::ToolTipRole, fullname);
                parent->addChild(item);
            }        
            SharedItemPtr i = SharedItemPtr(new ItemGeometry(d->PCLVis, fullname, item));
            item->setData(0, Item::itemRole, QVariant::fromValue(i)); //add it before adding any VTK or PCL geometry such that possible existing item, previously stored in the same user data, is deleted.
            connect(i.data(), SIGNAL(updateCanvasRequest()), d->ui.pclCanvas, SLOT(update()));
            ((ItemGeometry*)(i.data()))->addCuboid(points2, color);
        }

        d->ui.pclCanvas->update();
    }

    if (points2) delete points2;

    return retval;
}

//-------------------------------------------------------------------------------------
ito::RetVal Vtk3dVisualizer::addCube(QVector<double> size, QVector<double> translation, QVector<double> rotation, const QString &fullname, const QColor &color /*= Qt::white*/)
{
    ito::RetVal retval;

    float x,y,z;
    float rx,ry,rz;

    if (rotation.size() == 0)
    {
        rx = ry = rz = 0.0;
    }
    else if (rotation.size() == 3)
    {
        rx = rotation[0];
        ry = rotation[1];
        rz = rotation[2];
    }
    else
    {
        retval += ito::RetVal(ito::retError, 0, "rotation must have zero or 3 euler angles (in rad)");
    }

    if (translation.size() == 0)
    {
        x = y = z = 0.0;
    }
    else if (translation.size() == 3)
    {
        x = translation[0];
        y = translation[1];
        z = translation[2];
    }
    else
    {
        retval += ito::RetVal(ito::retError, 0, "translation must have zero or 3 values");
    }

    if (size.size() != 3)
    {
        retval += ito::RetVal(ito::retError, 0, "size must have 3 values.");
    }

    if (!retval.containsError())
    {
        Eigen::Affine3f trafo = pcl::getTransformation(x,y,z,rx,ry,rz);
        Eigen::Vector3f s(size[0], size[1], size[2]);

        QTreeWidgetItem *parent;
        QString name = fullname;

        retval += createRecursiveTree(name, d->geometryItem, &parent);

        if (!retval.containsError())
        {
            //check if item already exists with this name
            QTreeWidgetItem *item = NULL;
            for (int i = 0; i < parent->childCount(); i++)
            {
                if (parent->child(i)->data(0, Qt::ToolTipRole) == fullname)
                {
                    item = parent->child(i);
                    break;
                }
            }

            if (!item)
            {
                item = new QTreeWidgetItem();
                item->setData(0, Qt::DisplayRole, name);
                item->setData(0, Qt::ToolTipRole, fullname);
                parent->addChild(item);
            }        
            SharedItemPtr i = SharedItemPtr(new ItemGeometry(d->PCLVis, fullname, item));
            item->setData(0, Item::itemRole, QVariant::fromValue(i)); //add it before adding any VTK or PCL geometry such that possible existing item, previously stored in the same user data, is deleted.
            connect(i.data(), SIGNAL(updateCanvasRequest()), d->ui.pclCanvas, SLOT(update()));
            ((ItemGeometry*)(i.data()))->addCube(s, trafo, color);
        }

        d->ui.pclCanvas->update();
    }
        
        //Eigen::Matrix<float, 3, 8, Eigen::RowMajor> points;
        //points.col(0) = Eigen::Vector3f(minimums[0], minimums[1], minimums[2]); //p0
        //points.col(1) = Eigen::Vector3f(minimums[0], minimums[1], maximums[2]); //p1
        //points.col(2) = Eigen::Vector3f(maximums[0], minimums[1], maximums[2]); //p2
        //points.col(3) = Eigen::Vector3f(maximums[0], minimums[1], minimums[2]); //p3
        //points.col(4) = Eigen::Vector3f(minimums[0], maximums[1], minimums[2]); //p4
        //points.col(5) = Eigen::Vector3f(minimums[0], maximums[1], maximums[2]); //p5
        //points.col(6) = Eigen::Vector3f(maximums[0], maximums[1], maximums[2]); //p6
        //points.col(7) = Eigen::Vector3f(maximums[0], maximums[1], minimums[2]); //p7

        //points = trafo * points;

        //int sizes[] = {3,8};
        //ito::DataObject points_(2, sizes,ito::tFloat32, (uchar*)(points.data()));

        //retval += addCuboid(points_, fullname, color);

    return retval;

}

//-------------------------------------------------------------------------------------
ito::RetVal Vtk3dVisualizer::addSphere(QVector<double> point, double radius, const QString &fullname, const QColor &color /*= Qt::red*/)
{
    ito::RetVal retval;

    pcl::PointXYZ center;

    if (point.size() == 3)
    {
        center.x = point[0];
        center.y = point[1];
        center.z = point[2];
    }
    else
    {
        retval += ito::RetVal(ito::retError, 0, "center point must have three values.");
    }

    if (!retval.containsError())
    {
        QTreeWidgetItem *parent;
        QString name = fullname;

        retval += createRecursiveTree(name, d->geometryItem, &parent);

        if (!retval.containsError())
        {
            //check if item already exists with this name
            QTreeWidgetItem *item = NULL;
            for (int i = 0; i < parent->childCount(); i++)
            {
                if (parent->child(i)->data(0, Qt::ToolTipRole) == fullname)
                {
                    item = parent->child(i);
                    break;
                }
            }

            if (!item)
            {
                item = new QTreeWidgetItem();
                item->setData(0, Qt::DisplayRole, name);
                item->setData(0, Qt::ToolTipRole, fullname);
                parent->addChild(item);
            }        
            SharedItemPtr i = SharedItemPtr(new ItemGeometry(d->PCLVis, fullname, item));
            item->setData(0, Item::itemRole, QVariant::fromValue(i)); //add it before adding any VTK or PCL geometry such that possible existing item, previously stored in the same user data, is deleted.
            connect(i.data(), SIGNAL(updateCanvasRequest()), d->ui.pclCanvas, SLOT(update()));
            ((ItemGeometry*)(i.data()))->addSphere(center, radius, color);
        }

        d->ui.pclCanvas->update();
    }

    return retval;
}


//-------------------------------------------------------------------------------------
ito::RetVal Vtk3dVisualizer::addLines(const ito::DataObject &points, const QString &fullname, const QColor &color /*= Qt::red*/)
{
    ito::RetVal retval;
    int sizes[] = {1,10000000,6,6};
    ito::DataObject *points2 = apiCreateFromDataObject(&points, 2, ito::tFloat32, sizes, &retval);

    if (!retval.containsError())
    {
        QTreeWidgetItem *parent;
        QString name = fullname;

        retval += createRecursiveTree(name, d->geometryItem, &parent);

        if (!retval.containsError())
        {
            //check if item already exists with this name
            QTreeWidgetItem *item = NULL;
            for (int i = 0; i < parent->childCount(); i++)
            {
                if (parent->child(i)->data(0, Qt::ToolTipRole) == fullname)
                {
                    item = parent->child(i);
                    break;
                }
            }

            if (!item)
            {
                item = new QTreeWidgetItem();
                item->setData(0, Qt::DisplayRole, name);
                item->setData(0, Qt::ToolTipRole, fullname);
                parent->addChild(item);
            }        
            SharedItemPtr i = SharedItemPtr(new ItemGeometry(d->PCLVis, fullname, item));
            item->setData(0, Item::itemRole, QVariant::fromValue(i)); //add it before adding any VTK or PCL geometry such that possible existing item, previously stored in the same user data, is deleted.
            connect(i.data(), SIGNAL(updateCanvasRequest()), d->ui.pclCanvas, SLOT(update()));
            ((ItemGeometry*)(i.data()))->addLines(points2, color);
        }

        d->ui.pclCanvas->update();
    }

    if (points2) delete points2;

    return retval;
}

//-------------------------------------------------------------------------------------
ito::RetVal Vtk3dVisualizer::setGeometryPose(const QString &name, QVector<double> translation, QVector<double> rotation)
{
    ito::RetVal retval;

    float x,y,z;
    float rx,ry,rz;

    if (rotation.size() == 0)
    {
        rx = ry = rz = 0.0;
    }
    else if (rotation.size() == 3)
    {
        rx = rotation[0];
        ry = rotation[1];
        rz = rotation[2];
    }
    else
    {
        retval += ito::RetVal(ito::retError, 0, "rotation must have zero or 3 euler angles (in rad)");
    }

    if (translation.size() == 0)
    {
        x = y = z = 0.0;
    }
    else if (translation.size() == 3)
    {
        x = translation[0];
        y = translation[1];
        z = translation[2];
    }
    else
    {
        retval += ito::RetVal(ito::retError, 0, "translation must have zero or 3 values");
    }

    if (!retval.containsError())
    {
        Eigen::Affine3f trafo = pcl::getTransformation(x,y,z,rx,ry,rz);

        QTreeWidgetItem *item = NULL;
        QString n = name;
        bool found = false;

        //test all categories
        retval += searchRecursiveTree(name, d->geometryItem, &item);

        if (!retval.containsError())
        {
            SharedItemPtr obj = item->data(0, Item::itemRole).value<SharedItemPtr>();
            retval += ((ItemGeometry*)(&(*obj)))->updatePose(trafo);
            d->ui.pclCanvas->update();
        }
    }

    return retval;
}

//-------------------------------------------------------------------------------------
void Vtk3dVisualizer::registerModel(ito::PCLPolygonMesh mesh, QString modelName)
{
    addMesh(mesh, modelName);
}

//-------------------------------------------------------------------------------------
ito::RetVal Vtk3dVisualizer::addMesh(ito::PCLPolygonMesh mesh, const QString &fullname)
{
    ito::RetVal retval;
    QTreeWidgetItem *parent;
    QString name = fullname;
    retval += createRecursiveTree(name, d->meshItem, &parent);

    if (!retval.containsError())
    {
        //check if item already exists with this name
        QTreeWidgetItem *item = NULL;
        for (int i = 0; i < parent->childCount(); i++)
        {
            if (parent->child(i)->data(0, Qt::ToolTipRole) == fullname)
            {
                item = parent->child(i);
                break;
            }
        }

        if (!item)
        {
            item = new QTreeWidgetItem();
            item->setData(0, Qt::DisplayRole, name);
            item->setData(0, Qt::ToolTipRole, fullname);
            parent->addChild(item);
        }
        
        SharedItemPtr i = SharedItemPtr(new ItemPolygonMesh(d->PCLVis, fullname, item));
        connect(i.data(), SIGNAL(updateCanvasRequest()), d->ui.pclCanvas, SLOT(update()));
        retval += ((ItemPolygonMesh*)(i.data()))->addPolygonMesh(mesh);

        item->setData(0, Item::itemRole, QVariant::fromValue(i));
    }

    d->ui.pclCanvas->update();

    return retval;
}

//--------------------------------------------------------------------------------------
ito::RetVal Vtk3dVisualizer::deletePointCloud(const QString &name)
{
    return deleteItem(name, d->cloudItem);
}

//--------------------------------------------------------------------------------------
ito::RetVal Vtk3dVisualizer::deleteMesh(const QString &name)
{
    return deleteItem(name, d->meshItem);
}

//--------------------------------------------------------------------------------------
ito::RetVal Vtk3dVisualizer::deleteGeometry(const QString &name)
{
    return deleteItem(name, d->geometryItem);
}

//--------------------------------------------------------------------------------------
ito::RetVal Vtk3dVisualizer::deleteItem(const QString &name, QTreeWidgetItem *rootItem)
{
    ito::RetVal retval;
    QTreeWidgetItem *item = NULL;
    QString n = name;
    bool found = false;
    retval += searchRecursiveTree(name, rootItem, &item);

    if (!retval.containsError())
    {
        //invalidate property widget for safety reason
        d->propertyWidget->setObject(NULL);

        delete item;
    }

    d->ui.pclCanvas->update();

    return retval;
}

//--------------------------------------------------------------------------------------
ito::RetVal Vtk3dVisualizer::setItemProperty(const QString &name, const QByteArray &property, const QVariant &value)
{
    ito::RetVal retval;
    QTreeWidgetItem *item = NULL;
    QString n = name;
    bool found = false;

    if (name == "canvas")
    {
        item = d->canvasItem;
    }
    else
    {
        //test all categories
        retval += searchRecursiveTree(name, d->cloudItem, &item);

        if (retval.containsError())
        {
            retval = ito::retOk;
            retval += searchRecursiveTree(name, d->meshItem, &item);

            if (retval.containsError())
            {
                retval = ito::retOk;
                retval += searchRecursiveTree(name, d->geometryItem, &item);
            }
        }
    }

    if (!retval.containsError())
    {
        SharedItemPtr obj = item->data(0, Item::itemRole).value<SharedItemPtr>();
        if (obj.data())
        {
            QVariant prop = obj->property(property.data());
            if (prop.isValid() == false)
            {
                retval += ito::RetVal::format(ito::retError,0,"Property %s does not exist", property.data());
            }
            else if (prop.userType() == QMetaType::type("ito::AutoInterval"))
            {
                bool ok;
                QString text = value.toString();
                QVariant value_;
                if (QString::compare(text,"auto",Qt::CaseInsensitive) == 0)
                {
                    value_ = QVariant::fromValue<ito::AutoInterval>(ito::AutoInterval(0, 0, true));
                }
                else
                {
                    QList<QVariant> list = value.toList();
                    if (list.size() == 2)
                    {
                        float v1 = list[0].toFloat(&ok);
                        if (!ok)
                        {
                            retval += ito::RetVal(ito::retError, 0, "first value could not be converted to float");
                        }
                        else
                        {
                            float v2 = list[1].toFloat(&ok);
                            if (!ok)
                            {
                                retval += ito::RetVal(ito::retError, 0, "second value could not be converted to float");
                            }
                            else
                            {
                                value_ = QVariant::fromValue<ito::AutoInterval>(ito::AutoInterval(v1, v2, false));
                            }
                        }
                    }
                }

                if (value_.isValid())
                {
                    if (!obj->setProperty( property.data(), value_ ))
                    {
                        retval += ito::RetVal::format(ito::retError,0,"Property %s could not be set", property.data());
                    }
                }
                else
                {
                    retval += ito::RetVal::format(ito::retError,0,"Given value could not be parsed to the required ito::AutoInterval type ('auto' or [min,max] requested)", property.data());
                }
            }
            else if (!obj->setProperty( property.data(), value ))
            {
                retval += ito::RetVal::format(ito::retError,0,"Property %s could not be set", property.data());
            }
        }
        else
        {
            retval += ito::RetVal::format(ito::retError,0,"Item %s is NULL", name.toLatin1().data());
        }
    }

    d->ui.pclCanvas->update();

    return retval;
}

//--------------------------------------------------------------------------------------
//point cloud for pick point event, nearest point from (x,y,z) to the given cloud is searched and
//its index is finally returned, too.
ito::RetVal Vtk3dVisualizer::setPickPointCloud(ito::PCLPointCloud &pcl)
{
    ito::RetVal retval;
    bool xyzNotNormal = true;

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointNormal>::Ptr cloudNormal(new pcl::PointCloud<pcl::PointNormal>);

    switch(pcl.getType())
    {
    case ito::pclXYZ:
        cloud = pcl.toPointXYZ();
        break;
    case ito::pclXYZI:
        {
            pcl::copyPointCloud(*pcl.toPointXYZIConst(), *cloud);
        }
        break;
    case ito::pclXYZRGBA:
        {
            pcl::copyPointCloud(*pcl.toPointXYZRGBAConst(), *cloud);
        }
        break;
    case ito::pclXYZNormal:
        {
            pcl::copyPointCloud(*pcl.toPointXYZNormalConst(), *cloudNormal);
            xyzNotNormal = false;
        }
        break;
    case ito::pclXYZINormal:
        {
            pcl::copyPointCloud(*pcl.toPointXYZINormalConst(), *cloudNormal);
            xyzNotNormal = false;
        }
        break;
    case ito::pclXYZRGBNormal:
        {
            pcl::copyPointCloud(*pcl.toPointXYZRGBNormalConst(), *cloudNormal);
            xyzNotNormal = false;
        }
        break;
    default:
        retval += ito::RetVal(ito::retError, 0, "point cloud is invalid");
        break;
    }

    if (!retval.containsError())
    {
        d->pointPickSearch.deleteTree();
        d->pointPickSearch.setResolution(1.0);
        d->pointPickSearchNormal.deleteTree();
        d->pointPickSearchNormal.setResolution(1.0);

        if (xyzNotNormal)
        {
            d->pointPickSearch.setInputCloud(cloud);
            d->pointPickSearch.addPointsFromInputCloud();
            d->pointPickSearchHasNormals = false;
        }
        else
        {
            d->pointPickSearchNormal.setInputCloud(cloudNormal);
            d->pointPickSearchNormal.addPointsFromInputCloud();
            d->pointPickSearchHasNormals = true;
        }
    }

    return retval;
}

//--------------------------------------------------------------------------------------
ito::RetVal Vtk3dVisualizer::setPickPointMesh(ito::PCLPolygonMesh &mesh)
{
    if (!mesh.valid())
    {
        return ito::RetVal(ito::retError, 0, "invalid polygon mesh");
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromPCLPointCloud2(mesh.polygonMesh()->cloud, *cloud);

    d->pointPickSearch.deleteTree();
    d->pointPickSearch.setResolution(1.0);
    d->pointPickSearchNormal.deleteTree();
    d->pointPickSearchNormal.setResolution(1.0);

    d->pointPickSearch.setInputCloud(cloud);
    d->pointPickSearch.addPointsFromInputCloud();
    d->pointPickSearchHasNormals = false;
    return ito::retOk;
}

//--------------------------------------------------------------------------------------
void Vtk3dVisualizer::point_picking_callback (const pcl::visualization::PointPickingEvent& event, void* cookie)
{
    if (event.getPointIndex() != -1)
    {
        pcl::PointXYZ pt;
        event.getPoint(pt.x,pt.y,pt.z);
        qDebug() << "Point (" << pt.x << "," << pt.y << "," << pt.z << ") was picked";

        pcl::PointNormal ptN;
        ptN.x = pt.x;
        ptN.y = pt.y;
        ptN.z = pt.z;
        ptN.normal_x = 0.0;
        ptN.normal_y = 0.0;
        ptN.normal_z = 0.0;

        if (!d->pointPickSearchHasNormals && d->pointPickSearch.getInputCloud().get() != NULL)
        {
            // Return the correct index in the cloud instead of the index on the screen
            std::vector<int> indices (1);
            std::vector<float> distances (1);

            d->pointPickSearch.nearestKSearch(pt, 1, indices, distances);
            if (indices[0] >= 0 && indices[0] < d->pointPickSearch.getInputCloud()->size())
            {
                pt = d->pointPickSearch.getInputCloud()->at(indices[0]);
                qDebug() << " --> Point (" << pt.x << "," << pt.y << "," << pt.z << ") was finally estimated (index " << indices[0] << ")";
                emit pointPicked(pt.x, pt.y, pt.z, indices[0]);
            }
            else
            {
                emit pointPicked(pt.x, pt.y, pt.z, -1);
            }
        }
        else if (d->pointPickSearchHasNormals && d->pointPickSearchNormal.getInputCloud().get() != NULL)
        {
            // Return the correct index in the cloud instead of the index on the screen
            std::vector<int> indices (1);
            std::vector<float> distances (1);

            

            d->pointPickSearchNormal.nearestKSearch(ptN, 1, indices, distances);
            if (indices[0] >= 0 && indices[0] < d->pointPickSearchNormal.getInputCloud()->size())
            {
                ptN = d->pointPickSearchNormal.getInputCloud()->at(indices[0]);
                pt.x = ptN.x;
                pt.y = ptN.y;
                pt.z = ptN.z;
                qDebug() << " --> Point (" << pt.x << "," << pt.y << "," << pt.z << ") was finally estimated (index " << indices[0] << ")";
                emit pointPicked(pt.x, pt.y, pt.z, indices[0]);
            }
            else
            {
                emit pointPicked(pt.x, pt.y, pt.z, -1);
            }
        }
        else
        {
            emit pointPicked(pt.x, pt.y, pt.z, -1);
        }

        if (d->pointPickSphereRadius > 0.0)
        {
            if (d->pointPickSphereName != "")
            {
                d->PCLVis->updateSphere(pt, d->pointPickSphereRadius, d->pointPickSphereColor.red() / 256.0, d->pointPickSphereColor.green() / 256.0, d->pointPickSphereColor.blue() / 256.0, "__pointPickingSphere");
            }
            else
            {
                
                d->PCLVis->addSphere(pt, d->pointPickSphereRadius, d->pointPickSphereColor.red() / 256.0, d->pointPickSphereColor.green() / 256.0, d->pointPickSphereColor.blue() / 256.0, "__pointPickingSphere");
                d->PCLVis->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION ,pcl::visualization::PCL_VISUALIZER_REPRESENTATION_WIREFRAME, "__pointPickingSphere");
                d->PCLVis->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH ,2.0, "__pointPickingSphere");
                d->pointPickSphereName = "__pointPickingSphere";
            }

            if (d->pointPickArrowName != "")
            {
                d->PCLVis->removeShape(d->pointPickArrowName);
                d->pointPickArrowName = "";
            }

            if (d->pointPickSearchHasNormals)
            {
                pcl::PointXYZ pt2 = pt;
                pt2.x += ptN.normal_x;
                pt2.y += ptN.normal_y;
                pt2.z += ptN.normal_z;
                
                d->PCLVis->addArrow(pt2, pt, d->pointPickSphereColor.red() / 256.0, d->pointPickSphereColor.green() / 256.0, d->pointPickSphereColor.blue() / 256.0, false, "__pointPickingArrow");
                d->pointPickArrowName = "__pointPickingArrow";
            }
        }
    }
    else
    {
        if (d->pointPickSphereName != "")
        {
            d->PCLVis->removeShape(d->pointPickSphereName);
            d->pointPickSphereName = "";
        }

        if (d->pointPickArrowName != "")
        {
            d->PCLVis->removeShape(d->pointPickArrowName);
            d->pointPickArrowName = "";
        }

        d->ui.pclCanvas->update();
    }
}


//------------------------------------------------------------------------------------------------------------
void Vtk3dVisualizer::setEnablePointPick(bool enabled)
{
    if (enabled && !d->pointPickConnection.connected())
    {
        d->pointPickConnection = d->PCLVis->registerPointPickingCallback(&Vtk3dVisualizer::point_picking_callback,*this,0);
    }
    else if (!enabled && d->pointPickConnection.connected())
    {
        d->pointPickConnection.disconnect();

        if (d->pointPickSphereName != "")
        {
            d->PCLVis->removeShape(d->pointPickSphereName);
            d->pointPickSphereName = "";
        }

        if (d->pointPickArrowName != "")
        {
            d->PCLVis->removeShape(d->pointPickArrowName);
            d->pointPickArrowName = "";
        }

        d->ui.pclCanvas->update();
    }
}

//------------------------------------------------------------------------------------------------------------
bool Vtk3dVisualizer::enablePointPick() const 
{ 
    return d->pointPickConnection.connected(); 
}

//------------------------------------------------------------------------------------------------------------
void Vtk3dVisualizer::itemClicked(QTreeWidgetItem *item, int column)
{
    if(item)
    {
        SharedItemPtr i = item->data(0, Item::itemRole).value<SharedItemPtr>();
        d->propertyWidget->setObject(i.data());
    }
    else
    {
        d->propertyWidget->setObject(NULL);
    }
}