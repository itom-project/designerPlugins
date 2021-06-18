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

#include "vtk3dVisualizer.h"

#include "CustomTypes.h"

#include "pcl/point_cloud.h"
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/common/eigen.h>
#include <pcl/visualization/point_cloud_handlers.h>
#include <pcl/filters/filter.h>
#include "pcl/search/kdtree.h"
#include "pcl/octree/octree.h"
#include "pcl/visualization/pcl_visualizer.h"
#include "vtkOutputWindow.h"
#include "vtkFileOutputWindow.h"
#include <qsharedpointer.h>
#include <qcolor.h>
#include <qdebug.h>
#include <qstatusbar.h>
#include <qevent.h>
#include <qaction.h>
#include <qmap.h>
#include <qstring.h>

#ifdef LEGACY_VTK
    #include "QVTKWidget.h"
    #include "QVTKInteractor.h"
#else
    #include "QVTKOpenGLNativeWidget.h"
    #include "vtkRenderWindowInteractor.h"
    #include "vtkGenericOpenGLRenderWindow.h"
#endif

#include "vtkSmartPointer.h"
#include "vtkCubeAxesActor.h"

#include "vtkCamera.h"
#include "vtkVersion.h"

#include "QPropertyEditor/QPropertyEditorWidget.h"

#include "itemGeometry.h"
#include "itemPolygonMesh.h"
#include "itemPointCloud.h"
#include "itemPointCloudNormal.h"
#include "item.h"
#include "DataObject/dataObjectFuncs.h"
#include "treeWidgetKeyEater.h"

#include "common/apiFunctionsInc.h"

#include <qtimer.h>

#if PCL_VERSION_COMPARE(>=,1,7,0)
    #include <vtkRenderWindow.h>
#endif

Q_DECLARE_METATYPE ( SharedItemPtr )

//------------------------------------------------------------------------------------------------------------------------
class Vtk3dVisualizerPrivate 
{
public:
    Vtk3dVisualizerPrivate() : 
        propertyWidget(NULL),
        treeWidget(NULL),
        meshItem(NULL),
        cloudItem(NULL),
        geometryItem(NULL),
        pointPickSphereRadius(0.0),
        pointPickSphereColor(QColor(255,0,0)),
        pointPickSphereCurrentPosition(0,0,0),
        pointPickSearch(pcl::octree::OctreePointCloudSearch<pcl::PointXYZ>(1.0)),
        pointPickSearchNormal(pcl::octree::OctreePointCloudSearch<pcl::PointNormal>(1.0)),
        pointPickSearchHasNormals(false),
        showFPS(true),
        stereoType(Vtk3dVisualizer::No),
        backgroundColor(Qt::black),
        coordinateSysScale(1.0),
        coordinateSysVisible(true),
        coordinateSysPos(0,0,0),
        canvasUpdateQueued(false)
    {}
    
    double pointPickSphereRadius;
    QColor pointPickSphereColor;
    pcl::PointXYZ pointPickSphereCurrentPosition;
    std::string pointPickSphereName;
    std::string pointPickArrowName;
    boost::signals2::connection pointPickConnection;

    QDockWidget *dockSettings;
    QDockWidget *dockItems;

    QPropertyEditorWidget *propertyWidget;
    boost::shared_ptr<pcl::visualization::PCLVisualizer> PCLVis;
    vtkSmartPointer<vtkCubeAxesActor> cubeAxesActor;

    QTreeWidget *treeWidget;
    QTreeWidgetItem *meshItem;
    QTreeWidgetItem *cloudItem;
    QTreeWidgetItem *geometryItem;

    pcl::octree::OctreePointCloudSearch<pcl::PointXYZ> pointPickSearch;
    pcl::octree::OctreePointCloudSearch<pcl::PointNormal> pointPickSearchNormal;
    bool pointPickSearchHasNormals;
    
#ifdef LEGACY_VTK
    QVTKWidget *pclCanvas;
#else
    QVTKOpenGLNativeWidget *pclCanvas;
#endif

    QColor backgroundColor;

    bool showFPS;
    Vtk3dVisualizer::Stereo stereoType;

    double coordinateSysScale;
    bool coordinateSysVisible;
    QVector3D coordinateSysPos;

    QMap<QByteArray, QAction*> actions;
    QString title;
    bool canvasUpdateQueued;
};




//------------------------------------------------------------------------------------------------------------------------
Vtk3dVisualizer::Vtk3dVisualizer(const QString &itomSettingsFile, AbstractFigure::WindowMode windowMode, QWidget *parent) :
    AbstractDObjPclFigure(itomSettingsFile, windowMode, parent),
    d(NULL)
{
    d = new Vtk3dVisualizerPrivate();

    //d->ui.setupUi(this);

#ifdef LEGACY_VTK
    d->PCLVis = boost::shared_ptr<pcl::visualization::PCLVisualizer>(new pcl::visualization::PCLVisualizer("PCLVisualizer", false));
#else
    auto renderer = vtkSmartPointer<vtkRenderer>::New();
    vtkSmartPointer<vtkGenericOpenGLRenderWindow> _renderWindow = vtkSmartPointer<vtkGenericOpenGLRenderWindow>::New();
    _renderWindow->AddRenderer(renderer);
    d->PCLVis = boost::shared_ptr<pcl::visualization::PCLVisualizer>(
        new pcl::visualization::PCLVisualizer(renderer, _renderWindow, "PCLVisualizer", false)
        );
#endif
    vtkSmartPointer<vtkRenderWindow> win = d->PCLVis->getRenderWindow();

    win->SetStereoCapableWindow(1);
    win->StereoRenderOff();

#ifdef LEGACY_VTK
    d->pclCanvas = new QVTKWidget(this);
#else
    d->pclCanvas = new QVTKOpenGLNativeWidget(this);
#endif
    this->setCentralWidget(d->pclCanvas);

    d->pclCanvas->SetRenderWindow(win); //pviz.getRenderWindow());
#ifdef LEGACY_VTK
    QVTKInteractor *interactor = d->pclCanvas->GetInteractor();
#else
    vtkRenderWindowInteractor *interactor = d->pclCanvas->GetInteractor();
#endif

    d->PCLVis->setShowFPS(true);
    

    d->PCLVis->setupInteractor(interactor, d->pclCanvas->GetRenderWindow());
    d->PCLVis->getInteractorStyle()->setKeyboardModifier(pcl::visualization::INTERACTOR_KB_MOD_SHIFT);
    d->PCLVis->getRenderWindow()->Render(); //wichtig, dass dieser befehl vor dem ersten Hinzufuegen von Elementen oder setzen von visuellen Eigenschaften kommt, da sonst addPointCloud crashed, alternativ kann auch setBackgroundColor gerufen werden, aber das ruft intern auch render() auf.

    if (interactor->HasObserver(vtkCommand::ExitEvent))
    {
        interactor->RemoveObservers(vtkCommand::ExitEvent);
    }

    //m_pPCLVis = boost::shared_ptr<pcl::visualization::PCLVisualizer>(new pcl::visualization::PCLVisualizer("PCLVisualization") );
    d->PCLVis->initCameraParameters ();
    d->PCLVis->setCameraPosition(0.0, 0.0, 16.0, 0.0, -1.0, 0.0);

    //create dock widgets
    d->dockItems = new QDockWidget(this);
    d->dockItems->setObjectName(QStringLiteral("dockItems"));
    d->dockItems->setWindowTitle("Items");
    d->dockItems->setVisible(false);
    d->treeWidget = new QTreeWidget(d->dockItems);
    d->treeWidget->setObjectName(QStringLiteral("treeWidget"));
    d->treeWidget->setHeaderHidden(true);
    d->dockItems->setWidget(d->treeWidget);
    addToolbox(d->dockItems, "dockItems", Qt::LeftDockWidgetArea);

    d->dockSettings = new QDockWidget(this);
    d->dockSettings->setObjectName(QStringLiteral("itemsSettings"));
    d->dockSettings->setWindowTitle("Items Settings");
    d->dockSettings->setVisible(false);
    addToolbox(d->dockSettings, "itemsSettings", Qt::LeftDockWidgetArea);

    //prepare QPropertyEditor for visualization
    CustomTypes::registerTypes();
    d->propertyWidget = new QPropertyEditorWidget(d->dockSettings);
    d->dockSettings->setWidget(d->propertyWidget);
    d->propertyWidget->registerCustomPropertyCB( CustomTypes::createCustomProperty);

    //create root for meshes
    d->meshItem = new QTreeWidgetItem();
    SharedItemPtr i1 = QSharedPointer<Item>( new Item("mesh", Item::rttiRoot, d->meshItem) );
    connect(i1.data(), &Item::updateCanvasRequest, this, &Vtk3dVisualizer::updateCanvas);
    d->meshItem->setData(0, Qt::DisplayRole, "mesh");
    d->meshItem->setData(0, Item::itemRole, QVariant::fromValue(i1) );
    d->treeWidget->addTopLevelItem( d->meshItem );

    //create root for point clouds
    d->cloudItem = new QTreeWidgetItem();
    SharedItemPtr i2 = QSharedPointer<Item>( new Item("clouds", Item::rttiRoot, d->cloudItem) );
    connect(i2.data(), &Item::updateCanvasRequest, this, &Vtk3dVisualizer::updateCanvas);
    d->cloudItem->setData(0, Qt::DisplayRole, "clouds");
    d->cloudItem->setData(0, Item::itemRole, QVariant::fromValue(i2) );
    d->treeWidget->addTopLevelItem( d->cloudItem );

    //create root for geometries
    d->geometryItem = new QTreeWidgetItem();
    SharedItemPtr i3 = QSharedPointer<Item>( new Item("geometries", Item::rttiRoot, d->geometryItem) );
    connect(i3.data(), &Item::updateCanvasRequest, this, &Vtk3dVisualizer::updateCanvas);
    d->geometryItem->setData(0, Qt::DisplayRole, "geometries");
    d->geometryItem->setData(0, Item::itemRole, QVariant::fromValue(i3) );
    d->treeWidget->addTopLevelItem( d->geometryItem );

    connect(d->treeWidget, SIGNAL(itemClicked(QTreeWidgetItem*,int)), this, SLOT(itemClicked(QTreeWidgetItem*,int)));
    TreeWidgetKeyEater *treeWidgetKeyEater = new TreeWidgetKeyEater(this, d->propertyWidget); //will be deleted upon deletion of this
    d->treeWidget->installEventFilter(treeWidgetKeyEater);


    //create axes
    d->cubeAxesActor = vtkSmartPointer<vtkCubeAxesActor>::New();
    d->cubeAxesActor->SetBounds(-5, 5, -5, 5, -5, 5);
    d->cubeAxesActor->SetCamera(d->PCLVis->getRendererCollection()->GetFirstRenderer()->GetActiveCamera());

    d->cubeAxesActor->DrawXGridlinesOff();
    d->cubeAxesActor->DrawYGridlinesOff();
    d->cubeAxesActor->DrawZGridlinesOff();

#if VTK_MAJOR_VERSION >= 7 && VTK_MINOR_VERSION > 0
    d->cubeAxesActor->SetGridLineLocation(vtkCubeAxesActor::VTK_GRID_LINES_FURTHEST);
#elif VTK_MAJOR_VERSION > 5
    d->cubeAxesActor->SetGridLineLocation(VTK_GRID_LINES_FURTHEST);
#endif	

    d->cubeAxesActor->XAxisMinorTickVisibilityOff();
    d->cubeAxesActor->YAxisMinorTickVisibilityOff();
    d->cubeAxesActor->ZAxisMinorTickVisibilityOff();
    d->cubeAxesActor->SetVisibility(false);

    setBackgroundColor(QColor(0, 0, 0));
    setCoordSysVisible(true);

    d->PCLVis->getRendererCollection()->GetFirstRenderer()->AddActor(d->cubeAxesActor);

    updateCanvas();

    setPropertyObservedObject(this);

    //init actions
    createActions();

    //initialize actions
    QToolBar *mainTb = new QToolBar(tr("toolbars"), this);
    addToolBar(mainTb, "toolbars");
    mainTb->addAction(d->actions["itemsDock"]);
    mainTb->addAction(d->actions["settingsDock"]);
    mainTb->addAction(d->actions["propertyDock"]);

    //redirect any vtk warnings or errors to a log-file vtk_errors.txt in the current directory.
    //If desired, the vtkOutputWindow class can also be derived in order to redirect
    //errors and warnings to the std::cout and std::cerr instead of the default channels stdout or stderr.
    //see: http://vtk.1045678.n5.nabble.com/How-to-avoid-VTK-warning-window-td3297363.html
    vtkFileOutputWindow *w = vtkFileOutputWindow::New();
    w->SetFileName("vtk_errors.txt");
    vtkOutputWindow::SetInstance(w);
    w->Delete(); // now SetInstance owns the reference
}

//-------------------------------------------------------------------------------------
Vtk3dVisualizer::~Vtk3dVisualizer()
{
    //this timerEvent must be removed, else crashes can occure in some situations if visualization is already destroyed 
    //and the timer event is fired afterwards.
    d->pclCanvas->GetInteractor()->RemoveObservers(vtkCommand::TimerEvent);

    d->PCLVis->getInteractorStyle()->SetEnabled(0);

    d->propertyWidget = NULL;

    delete d;
}

//-------------------------------------------------------------------------------------
ito::RetVal Vtk3dVisualizer::applyUpdate()
{
    ito::RetVal retval;
    switch (m_inpType)
    {
    case ito::ParamBase::PointCloudPtr:
        {
            const ito::PCLPointCloud *cloud = getInputParam("pointCloud")->getVal<const ito::PCLPointCloud*>();
            if (cloud)
            {
                if (cloud->hasNormal())
                {
                    retval += this->addPointCloudNormal(*cloud, "source_cloud_normal");
                }
                else
                {
                    retval += this->addPointCloud(*cloud, "source_cloud_normal");
                }
            }
        }
        break;
    case ito::ParamBase::PolygonMeshPtr:
        {
            const ito::PCLPolygonMesh *mesh = getInputParam("polygonMesh")->getVal<const ito::PCLPolygonMesh*>();
            if (mesh)
            {
                retval += this->addMesh(*mesh, "source_mesh");
            }
        }
        break;
    default:
        retval += ito::RetVal(ito::retError, 0, tr("unsupported input type").toLatin1().data());
    }
    
    return retval;
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
            _item = SharedItemPtr(new Item(first,Item::rttiRoot, item));
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
            retval += ito::RetVal(ito::retError, 0, tr("item not found.").toLatin1().data());
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
    if (d->pointPickSphereRadius != radius)
    {
        d->pointPickSphereRadius = radius;

        if (d->pointPickSphereRadius > 0.0 && d->pointPickSphereName != "")
        {
            d->PCLVis->updateSphere(d->pointPickSphereCurrentPosition, 
                d->pointPickSphereRadius, d->pointPickSphereColor.red() / 256.0, 
                d->pointPickSphereColor.green() / 256.0, 
                d->pointPickSphereColor.blue() / 256.0, "__pointPickingSphere");

            updateCanvas();
        }
    }
}

//-------------------------------------------------------------------------------------
double Vtk3dVisualizer::pointPickSphereRadius() const 
{ 
    return d->pointPickSphereRadius; 
}

//-------------------------------------------------------------------------------------
void Vtk3dVisualizer::setpointPickSphereColor(QColor color)
{
    if (d->pointPickSphereColor != color)
    {
        d->pointPickSphereColor = color;

        if (d->pointPickSphereRadius > 0.0 && d->pointPickSphereName != "")
        {
            d->PCLVis->updateSphere(d->pointPickSphereCurrentPosition,
                d->pointPickSphereRadius, d->pointPickSphereColor.red() / 256.0,
                d->pointPickSphereColor.green() / 256.0,
                d->pointPickSphereColor.blue() / 256.0, "__pointPickingSphere");

            updateCanvas();
        }
    }
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

    bool nameAlreadyInUse = searchRecursiveTree(fullname, d->geometryItem, &parent) == ito::retOk;
    nameAlreadyInUse |= (bool)(searchRecursiveTree(fullname, d->meshItem, &parent) == ito::retOk);

    if (nameAlreadyInUse)
    {
        return ito::RetVal(ito::retError, 0, "The given name is already used in another item group.");
    }

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

        connect(i.data(), &Item::updateCanvasRequest, this, &Vtk3dVisualizer::updateCanvas);
        item->setData(0, Item::itemRole, QVariant::fromValue(i));

    }

    updateCanvas();

    return retval;
}

//-------------------------------------------------------------------------------------
ito::RetVal Vtk3dVisualizer::addPointCloudNormal(ito::PCLPointCloud pcl, const QString &fullname)
{
    ito::RetVal retval;
    QTreeWidgetItem *parent;

    bool nameAlreadyInUse = searchRecursiveTree(fullname, d->geometryItem, &parent) == ito::retOk;
    nameAlreadyInUse |= (bool)(searchRecursiveTree(fullname, d->meshItem, &parent) == ito::retOk);

    if (nameAlreadyInUse)
    {
        return ito::RetVal(ito::retError, 0, "The given name is already used in another item group.");
    }

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
            retval += ito::RetVal(ito::retError, 0, tr("the given point cloud has no normal component").toLatin1().data());
        }

        if (!retval.containsError())
        {
            connect(i.data(), &Item::updateCanvasRequest, this, &Vtk3dVisualizer::updateCanvas);
            item->setData(0, Item::itemRole, QVariant::fromValue(i));
        }

    }

    updateCanvas();

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
            if (i->rtti() == Item::rttiPointCloud)
            {
                ItemPointCloud *ipc = (ItemPointCloud*)(i.data());
                retval += ipc->updatePointCloud(pcl);
            }
            else if (i->rtti() == Item::rttiPointCloudNormal)
            {
                retval += ito::RetVal(ito::retError, 0, tr("an item of type 'point cloud normal' cannot be updated").toLatin1().data());
            }
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
    QTreeWidgetItem *parent;
    bool nameAlreadyInUse = searchRecursiveTree(fullname, d->cloudItem, &parent) == ito::retOk;
    nameAlreadyInUse |= (bool)(searchRecursiveTree(fullname, d->meshItem, &parent) == ito::retOk);

    if (nameAlreadyInUse)
    {
        return ito::RetVal(ito::retError, 0, "The given name is already used in another item group.");
    }

    ito::RetVal retval;
    pcl::ModelCoefficients coefficients;
    coefficients.values.resize(7);

    if (point.size() != 3)
    {
        return ito::RetVal(ito::retError, 0, "Point must have three components");
    }

    if (orientation.size() != 3)
    {
        return ito::RetVal(ito::retError, 0, "Orientation must have three components");
    }

    coefficients.values[0] = point[0];
    coefficients.values[1] = point[1];
    coefficients.values[2] = point[2];

    coefficients.values[3] = orientation[0];
    coefficients.values[4] = orientation[1];
    coefficients.values[5] = orientation[2];

    coefficients.values[6] = radius;

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
        connect(i.data(), &Item::updateCanvasRequest, this, &Vtk3dVisualizer::updateCanvas);
        ((ItemGeometry*)(i.data()))->addCylinder(coefficients, color);
    }

    updateCanvas();

    return retval;
}

//-------------------------------------------------------------------------------------
ito::RetVal Vtk3dVisualizer::addPyramid(const ito::DataObject &points, const QString &fullname, const QColor &color /*= Qt::white*/) //points must be a 3x5 float matrix
{
    QTreeWidgetItem *parent;
    bool nameAlreadyInUse = searchRecursiveTree(fullname, d->cloudItem, &parent) == ito::retOk;
    nameAlreadyInUse |= (bool)(searchRecursiveTree(fullname, d->meshItem, &parent) == ito::retOk);

    if (nameAlreadyInUse)
    {
        return ito::RetVal(ito::retError, 0, "The given name is already used in another item group.");
    }

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
            connect(i.data(), &Item::updateCanvasRequest, this, &Vtk3dVisualizer::updateCanvas);
            ((ItemGeometry*)(i.data()))->addPyramid(points2, color);
        }

        updateCanvas();
    }

    if (points2) delete points2;

    return retval;
}

//-------------------------------------------------------------------------------------
ito::RetVal Vtk3dVisualizer::addCuboid(const ito::DataObject &points, const QString &fullname, const QColor &color /*= Qt::white*/)  //points must be a 3x8 float matrix
{
    QTreeWidgetItem *parent;
    bool nameAlreadyInUse = searchRecursiveTree(fullname, d->cloudItem, &parent) == ito::retOk;
    nameAlreadyInUse |= (bool)(searchRecursiveTree(fullname, d->meshItem, &parent) == ito::retOk);

    if (nameAlreadyInUse)
    {
        return ito::RetVal(ito::retError, 0, "The given name is already used in another item group.");
    }

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
            connect(i.data(), &Item::updateCanvasRequest, this, &Vtk3dVisualizer::updateCanvas);
            ((ItemGeometry*)(i.data()))->addCuboid(points2, color);
        }

        updateCanvas();
    }

    if (points2) delete points2;

    return retval;
}

//-------------------------------------------------------------------------------------
ito::RetVal Vtk3dVisualizer::addCube(QVector<double> size, QVector<double> translation, QVector<double> rotation, const QString &fullname, const QColor &color /*= Qt::white*/)
{
    QTreeWidgetItem *parent;
    bool nameAlreadyInUse = searchRecursiveTree(fullname, d->cloudItem, &parent) == ito::retOk;
    nameAlreadyInUse |= (bool)(searchRecursiveTree(fullname, d->meshItem, &parent) == ito::retOk);

    if (nameAlreadyInUse)
    {
        return ito::RetVal(ito::retError, 0, "The given name is already used in another item group.");
    }

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
        retval += ito::RetVal(ito::retError, 0, tr("rotation must have zero or 3 euler angles (in rad)").toLatin1().data());
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
        retval += ito::RetVal(ito::retError, 0, tr("translation must have zero or 3 values").toLatin1().data());
    }

    if (size.size() != 3)
    {
        retval += ito::RetVal(ito::retError, 0, tr("size must have 3 values.").toLatin1().data());
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
            connect(i.data(), &Item::updateCanvasRequest, this, &Vtk3dVisualizer::updateCanvas);
            ((ItemGeometry*)(i.data()))->addCube(s, trafo, color);
        }

        updateCanvas();
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
    QTreeWidgetItem *parent;
    bool nameAlreadyInUse = searchRecursiveTree(fullname, d->cloudItem, &parent) == ito::retOk;
    nameAlreadyInUse |= (bool)(searchRecursiveTree(fullname, d->meshItem, &parent) == ito::retOk);

    if (nameAlreadyInUse)
    {
        return ito::RetVal(ito::retError, 0, "The given name is already used in another item group.");
    }

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
        retval += ito::RetVal(ito::retError, 0, tr("center point must have three values.").toLatin1().data());
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
            connect(i.data(), &Item::updateCanvasRequest, this, &Vtk3dVisualizer::updateCanvas);
            ((ItemGeometry*)(i.data()))->addSphere(center, radius, color);
        }

        updateCanvas();
    }

    return retval;
}

//-------------------------------------------------------------------------------------
ito::RetVal Vtk3dVisualizer::addPolygon(const ito::DataObject &points, const QString &fullname, const QColor &color /*= Qt::red*/)
{
    QTreeWidgetItem *parent;
    bool nameAlreadyInUse = searchRecursiveTree(fullname, d->cloudItem, &parent) == ito::retOk;
    nameAlreadyInUse |= (bool)(searchRecursiveTree(fullname, d->meshItem, &parent) == ito::retOk);

    if (nameAlreadyInUse)
    {
        return ito::RetVal(ito::retError, 0, "The given name is already used in another item group.");
    }

    ito::RetVal retval;

    ito::DataObject points2 = ito::dObjHelper::squeezeConvertCheck2DDataObject(&points, "points", ito::Range(1,std::numeric_limits<int>::max()), ito::Range(3,3), retval, ito::tFloat32, 0);
    

    if (!retval.containsError())
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
        cloud->reserve(points2.getSize(0));
        ito::float32 *ptr = NULL;
        for (int r = 0; r < points2.getSize(0); ++r)
        {
            ptr = (ito::float32*)points2.rowPtr(0, r);
            cloud->push_back(pcl::PointXYZ(ptr[0], ptr[1], ptr[2]));
        }

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
            connect(i.data(), &Item::updateCanvasRequest, this, &Vtk3dVisualizer::updateCanvas);
            ((ItemGeometry*)(i.data()))->addPolygon(cloud, color);
        }

        updateCanvas();
    }

    return retval;
}

//-------------------------------------------------------------------------------------
ito::RetVal Vtk3dVisualizer::addText(const QString &text, const int x, const int y, const int fontsize, const QString &fullname, const QColor &color /*= Qt::white*/)
{
    QTreeWidgetItem *parent;
    bool nameAlreadyInUse = searchRecursiveTree(fullname, d->cloudItem, &parent) == ito::retOk;
    nameAlreadyInUse |= (bool)(searchRecursiveTree(fullname, d->meshItem, &parent) == ito::retOk);

    if (nameAlreadyInUse)
    {
        return ito::RetVal(ito::retError, 0, "The given name is already used in another item group.");
    }

    ito::RetVal retval;
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
        connect(i.data(), &Item::updateCanvasRequest, this, &Vtk3dVisualizer::updateCanvas);
        ((ItemGeometry*)(i.data()))->addText(text, x, y, fontsize, color);
    }

    updateCanvas();

    return retval;
}

//-------------------------------------------------------------------------------------
ito::RetVal Vtk3dVisualizer::updateText(const QString &text, const int x, const int y, const int fontsize, const QString &name, const QColor &color, bool createIfNotExists)
{
    ito::RetVal retval;
    QTreeWidgetItem *item = NULL;
    QString n = name;
    bool found = false;
    retval += searchRecursiveTree(name, d->geometryItem, &item);

    if (!retval.containsError())
    {
        SharedItemPtr i = item->data(0, Item::itemRole).value<SharedItemPtr>();

        if (i.data())
        {
            ItemGeometry *tg = (ItemGeometry*)(i.data());
            retval += tg->updateText(text, x, y, fontsize, color);

            updateCanvas();
        }
    }
    else if (createIfNotExists)
    {
        retval = addText(text, x, y, fontsize, name, color); //retval is assigned, no += since not-found error from above should be handled.
    }

    return retval;
}


//-------------------------------------------------------------------------------------
ito::RetVal Vtk3dVisualizer::addLines(const ito::DataObject &points, const QString &fullname, const QColor &color /*= Qt::red*/)
{
    QTreeWidgetItem *parent;
    bool nameAlreadyInUse = searchRecursiveTree(fullname, d->cloudItem, &parent) == ito::retOk;
    nameAlreadyInUse |= (bool)(searchRecursiveTree(fullname, d->meshItem, &parent) == ito::retOk);

    if (nameAlreadyInUse)
    {
        return ito::RetVal(ito::retError, 0, "The given name is already used in another item group.");
    }

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
            connect(i.data(), &Item::updateCanvasRequest, this, &Vtk3dVisualizer::updateCanvas);
            ((ItemGeometry*)(i.data()))->addLines(points2, color);
        }

        updateCanvas();
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
        retval += ito::RetVal(ito::retError, 0, tr("rotation must have zero or 3 euler angles (in rad)").toLatin1().data());
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
        retval += ito::RetVal(ito::retError, 0, tr("translation must have zero or 3 values").toLatin1().data());
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
            updateCanvas();
        }
    }

    return retval;
}

//-------------------------------------------------------------------------------------
ito::RetVal Vtk3dVisualizer::setGeometriesPosition(const QStringList &names, QVector<double> positions)
{
    ito::RetVal retval;

    if (positions.size() != 3*names.size())
    {
        retval += ito::RetVal(ito::retError, 0, tr("positions must have three values per named item.").toLatin1().data());
    }

    if (!retval.containsError())
    {
        for (int i = 0; i < names.size(); ++i)
        {
            Eigen::Affine3f trafo = pcl::getTransformation(positions[i*3+0],positions[i*3+1],positions[i*3+2],0,0,0);

            QTreeWidgetItem *item = NULL;
            bool found = false;

            //test all categories
            retval += searchRecursiveTree(names[i], d->geometryItem, &item);

            if (!retval.containsError())
            {
                SharedItemPtr obj = item->data(0, Item::itemRole).value<SharedItemPtr>();
                retval += ((ItemGeometry*)(&(*obj)))->updatePose(trafo);
            }
        }
        updateCanvas();
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
    QTreeWidgetItem *parent;
    bool nameAlreadyInUse = searchRecursiveTree(fullname, d->cloudItem, &parent) == ito::retOk;
    nameAlreadyInUse |= (bool)(searchRecursiveTree(fullname, d->geometryItem, &parent) == ito::retOk);

    if (nameAlreadyInUse)
    {
        return ito::RetVal(ito::retError, 0, "The given name is already used in another item group.");
    }

    ito::RetVal retval;

    if (mesh.valid() == false)
    {
        retval += ito::RetVal(ito::retError, 0, tr("given mesh is not valid").toLatin1().data());
    }
    else
    {
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
            connect(i.data(), &Item::updateCanvasRequest, this, &Vtk3dVisualizer::updateCanvas);
            retval += ((ItemPolygonMesh*)(i.data()))->addPolygonMesh(mesh);

            item->setData(0, Item::itemRole, QVariant::fromValue(i));
        }

        updateCanvas();
    }

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

    updateCanvas();

    return retval;
}

//--------------------------------------------------------------------------------------
ito::RetVal Vtk3dVisualizer::setItemProperty(const QString &name, const QByteArray &property, const QVariant &value)
{
    ito::RetVal retval;
    QTreeWidgetItem *item = NULL;
    QString n = name;
    bool found = false;

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

    if (!retval.containsError())
    {
        SharedItemPtr obj = item->data(0, Item::itemRole).value<SharedItemPtr>();
        if (obj.data())
        {
            QVariant prop = obj->property(property.data());
            if (prop.isValid() == false)
            {
                retval += ito::RetVal::format(ito::retError, 0, tr("Property %s does not exist").toLatin1().data(), property.data());
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
                            retval += ito::RetVal(ito::retError, 0, tr("first value could not be converted to float").toLatin1().data());
                        }
                        else
                        {
                            float v2 = list[1].toFloat(&ok);
                            if (!ok)
                            {
                                retval += ito::RetVal(ito::retError, 0, tr("second value could not be converted to float").toLatin1().data());
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
                        retval += ito::RetVal::format(ito::retError, 0, tr("Property %s could not be set").toLatin1().data(), property.data());
                    }
                }
                else
                {
                    retval += ito::RetVal::format(ito::retError, 0, tr("Given value could not be parsed to the required ito::AutoInterval type ('auto' or [min,max] requested)").toLatin1().data(), 
                        property.data());
                }
            }
            else if (!obj->setProperty( property.data(), value ))
            {
                retval += ito::RetVal::format(ito::retError, 0, tr("Property %s could not be set").toLatin1().data(), property.data());
            }
        }
        else
        {
            retval += ito::RetVal::format(ito::retError, 0, tr("Item %s is NULL").toLatin1().data(), name.toLatin1().data());
        }
    }

    updateCanvas();

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
        retval += ito::RetVal(ito::retError, 0, tr("point cloud is invalid").toLatin1().data());
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
        return ito::RetVal(ito::retError, 0, tr("invalid polygon mesh").toLatin1().data());
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
        d->pointPickSphereCurrentPosition = pt;

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

        updateCanvas();
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

        updateCanvas();
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


//----------------------------------------------------------------------------------------------------------------------------------
ito::AutoInterval Vtk3dVisualizer::getXAxisInterval(void) const
{
    double *bounds = d->cubeAxesActor->GetBounds();
    return ito::AutoInterval(bounds[0], bounds[1]);
}

//----------------------------------------------------------------------------------------------------------------------------------
void Vtk3dVisualizer::setXAxisInterval(ito::AutoInterval interval)
{
    double bounds[6];
    d->cubeAxesActor->GetBounds(bounds);
    if (!interval.isAuto())
    {
        bounds[0] = interval.minimum();
        bounds[1] = interval.maximum();
        d->cubeAxesActor->SetBounds(bounds);
    }
    updateCanvas();
    updatePropertyDock();
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::AutoInterval Vtk3dVisualizer::getYAxisInterval(void) const
{
    double *bounds = d->cubeAxesActor->GetBounds();
    return ito::AutoInterval(bounds[2], bounds[3]);
}

//----------------------------------------------------------------------------------------------------------------------------------
void Vtk3dVisualizer::setYAxisInterval(ito::AutoInterval interval)
{
    double bounds[6];
    d->cubeAxesActor->GetBounds(bounds);
    if (!interval.isAuto())
    {
        bounds[2] = interval.minimum();
        bounds[3] = interval.maximum();
        d->cubeAxesActor->SetBounds(bounds);
    }
    updateCanvas();
    updatePropertyDock();
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::AutoInterval Vtk3dVisualizer::getZAxisInterval(void) const
{
    double *bounds = d->cubeAxesActor->GetBounds();
    return ito::AutoInterval(bounds[4], bounds[5]);
}

//----------------------------------------------------------------------------------------------------------------------------------
void Vtk3dVisualizer::setZAxisInterval(ito::AutoInterval interval)
{
    double bounds[6];
    d->cubeAxesActor->GetBounds(bounds);
    if (!interval.isAuto())
    {
        bounds[4] = interval.minimum();
        bounds[5] = interval.maximum();
        d->cubeAxesActor->SetBounds(bounds);
    }
    updateCanvas();
    updatePropertyDock();
}


//----------------------------------------------------------------------------------------------------------------------------------
bool Vtk3dVisualizer::getxAxisVisible() const
{
    return d->cubeAxesActor->GetXAxisVisibility();
}

//----------------------------------------------------------------------------------------------------------------------------------
void Vtk3dVisualizer::setxAxisVisible(const bool &value)
{
    d->cubeAxesActor->SetXAxisVisibility(value);
    updateCanvas();
    updatePropertyDock();
}

//----------------------------------------------------------------------------------------------------------------------------------
bool Vtk3dVisualizer::getyAxisVisible() const
{
    return d->cubeAxesActor->GetYAxisVisibility();
}

//----------------------------------------------------------------------------------------------------------------------------------
void Vtk3dVisualizer::setyAxisVisible(const bool &value)
{
    d->cubeAxesActor->SetYAxisVisibility(value);
    updateCanvas();
    updatePropertyDock();
}

//----------------------------------------------------------------------------------------------------------------------------------
bool Vtk3dVisualizer::getzAxisVisible() const
{
    return d->cubeAxesActor->GetZAxisVisibility();
}

//----------------------------------------------------------------------------------------------------------------------------------
void Vtk3dVisualizer::setzAxisVisible(const bool &value)
{
    d->cubeAxesActor->SetZAxisVisibility(value);
    updateCanvas();
    updatePropertyDock();
}


//----------------------------------------------------------------------------------------------------------------------------------
QString Vtk3dVisualizer::getxAxisLabel() const
{
    return d->cubeAxesActor->GetXTitle();
}

//----------------------------------------------------------------------------------------------------------------------------------
void Vtk3dVisualizer::setxAxisLabel(const QString &label)
{
    d->cubeAxesActor->SetXTitle(label.toLatin1().data());
    d->cubeAxesActor->SetXAxisLabelVisibility(label != "");
    updateCanvas();
    updatePropertyDock();
}

//----------------------------------------------------------------------------------------------------------------------------------
QString Vtk3dVisualizer::getyAxisLabel() const
{
    return d->cubeAxesActor->GetYTitle();
}

//----------------------------------------------------------------------------------------------------------------------------------
void Vtk3dVisualizer::setyAxisLabel(const QString &label)
{
    d->cubeAxesActor->SetYTitle(label.toLatin1().data());
    d->cubeAxesActor->SetYAxisLabelVisibility(label != "");
    updateCanvas();
    updatePropertyDock();
}

//----------------------------------------------------------------------------------------------------------------------------------
QString Vtk3dVisualizer::getzAxisLabel() const
{
    return d->cubeAxesActor->GetZTitle();
}

//----------------------------------------------------------------------------------------------------------------------------------
void Vtk3dVisualizer::setzAxisLabel(const QString &label)
{
    d->cubeAxesActor->SetZTitle(label.toLatin1().data());
    d->cubeAxesActor->SetZAxisLabelVisibility(label != "");
    updateCanvas();
    updatePropertyDock();
}

//----------------------------------------------------------------------------------------------------------------------------------
bool Vtk3dVisualizer::getCubeAxesVisible() const
{
    return d->cubeAxesActor->GetVisibility();
}

//----------------------------------------------------------------------------------------------------------------------------------
void Vtk3dVisualizer::setCubeAxesVisible(const bool &visible)
{
    d->cubeAxesActor->SetVisibility(visible);
    updateCanvas();
    updatePropertyDock();
}

//----------------------------------------------------------------------------------------------------------------------------------
QColor Vtk3dVisualizer::getCubeAxesColor() const
{
#if VTK_MAJOR_VERSION >= 6
    double *colors = d->cubeAxesActor->GetXAxesLinesProperty()->GetColor();
    return QColor(colors[0] * 255, colors[1] * 255, colors[2] * 255);
#else
    return QColor(Qt::white);
#endif
}

//----------------------------------------------------------------------------------------------------------------------------------
void Vtk3dVisualizer::setCubeAxesColor(const QColor &color)
{
#if VTK_MAJOR_VERSION >= 6
    vtkProperty *prop = d->cubeAxesActor->GetXAxesLinesProperty();
    prop->SetColor(color.redF(), color.greenF(), color.blueF());
    d->cubeAxesActor->SetXAxesLinesProperty(prop);

    prop = d->cubeAxesActor->GetYAxesLinesProperty();
    prop->SetColor(color.redF(), color.greenF(), color.blueF());
    d->cubeAxesActor->SetYAxesLinesProperty(prop);

    prop = d->cubeAxesActor->GetZAxesLinesProperty();
    prop->SetColor(color.redF(), color.greenF(), color.blueF());
    d->cubeAxesActor->SetZAxesLinesProperty(prop);

    d->cubeAxesActor->GetLabelTextProperty(0)->SetColor(color.redF(), color.greenF(), color.blueF());
    d->cubeAxesActor->GetLabelTextProperty(1)->SetColor(color.redF(), color.greenF(), color.blueF());
    d->cubeAxesActor->GetLabelTextProperty(2)->SetColor(color.redF(), color.greenF(), color.blueF());
    d->cubeAxesActor->GetTitleTextProperty(0)->SetColor(color.redF(), color.greenF(), color.blueF());
    d->cubeAxesActor->GetTitleTextProperty(1)->SetColor(color.redF(), color.greenF(), color.blueF());
    d->cubeAxesActor->GetTitleTextProperty(2)->SetColor(color.redF(), color.greenF(), color.blueF());

    updateCanvas();
    updatePropertyDock();
#endif
}

//----------------------------------------------------------------------------------------------------------------------------------
QColor Vtk3dVisualizer::getCubeGridlinesColor() const
{
#if VTK_MAJOR_VERSION >= 6
    double *colors = d->cubeAxesActor->GetXAxesGridlinesProperty()->GetColor();
    return QColor(colors[0] * 255, colors[1] * 255, colors[2] * 255);
#else
    return QColor(Qt::white);
#endif
}

//----------------------------------------------------------------------------------------------------------------------------------
void Vtk3dVisualizer::setCubeGridlinesColor(const QColor &color)
{
#if VTK_MAJOR_VERSION >= 6
    vtkProperty *prop = d->cubeAxesActor->GetXAxesGridpolysProperty();
    prop->SetColor(color.redF(), color.greenF(), color.blueF());
    d->cubeAxesActor->SetXAxesGridpolysProperty(prop);

    prop = d->cubeAxesActor->GetYAxesGridpolysProperty();
    prop->SetColor(color.redF(), color.greenF(), color.blueF());
    d->cubeAxesActor->SetYAxesGridpolysProperty(prop);

    prop = d->cubeAxesActor->GetZAxesGridpolysProperty();
    prop->SetColor(color.redF(), color.greenF(), color.blueF());
    d->cubeAxesActor->SetZAxesGridpolysProperty(prop);

    prop = d->cubeAxesActor->GetXAxesGridlinesProperty();
    prop->SetColor(color.redF(), color.greenF(), color.blueF());
    d->cubeAxesActor->SetXAxesGridlinesProperty(prop);

    prop = d->cubeAxesActor->GetYAxesGridlinesProperty();
    prop->SetColor(color.redF(), color.greenF(), color.blueF());
    d->cubeAxesActor->SetYAxesGridlinesProperty(prop);

    prop = d->cubeAxesActor->GetZAxesGridlinesProperty();
    prop->SetColor(color.redF(), color.greenF(), color.blueF());
    d->cubeAxesActor->SetZAxesGridlinesProperty(prop);
    updateCanvas();
    updatePropertyDock();
#endif
}

//----------------------------------------------------------------------------------------------------------------------------------
Vtk3dVisualizer::FlyMode Vtk3dVisualizer::getCubeAxesFlyMode() const
{
    return (FlyMode)d->cubeAxesActor->GetFlyMode();
}

//----------------------------------------------------------------------------------------------------------------------------------
void Vtk3dVisualizer::setCubeAxesFlyMode(const FlyMode &mode)
{
    d->cubeAxesActor->SetFlyMode(mode);
    updateCanvas();
    updatePropertyDock();
}

//----------------------------------------------------------------------------------------------------------------------------------
Vtk3dVisualizer::TickLocation Vtk3dVisualizer::getCubeAxesTickLocation() const
{
    return (TickLocation)d->cubeAxesActor->GetTickLocation();
}

//----------------------------------------------------------------------------------------------------------------------------------
void Vtk3dVisualizer::setCubeAxesTickLocation(const TickLocation &location)
{
    d->cubeAxesActor->SetTickLocation(location);
    updateCanvas();
    updatePropertyDock();
}

//----------------------------------------------------------------------------------------------------------------------------------
bool Vtk3dVisualizer::getEnableDistanceLOD() const
{
#if VTK_MAJOR_VERSION >= 6
    return d->cubeAxesActor->GetEnableDistanceLOD();
#else
    return false;
#endif
}

//----------------------------------------------------------------------------------------------------------------------------------
void Vtk3dVisualizer::setEnableDistanceLOD(const bool &enable)
{
#if VTK_MAJOR_VERSION >= 6
    d->cubeAxesActor->SetEnableDistanceLOD(enable);
    updateCanvas();
    updatePropertyDock();
#endif
}

//----------------------------------------------------------------------------------------------------------------------------------
bool Vtk3dVisualizer::getEnableViewAngleLOD() const
{
#if VTK_MAJOR_VERSION >= 6
    return d->cubeAxesActor->GetEnableViewAngleLOD();
#else
    return false;
#endif
}

//----------------------------------------------------------------------------------------------------------------------------------
void Vtk3dVisualizer::setEnableViewAngleLOD(const bool &enable)
{
#if VTK_MAJOR_VERSION >= 6
    d->cubeAxesActor->SetEnableViewAngleLOD(enable);
    updateCanvas();
    updatePropertyDock();
#endif
}

//----------------------------------------------------------------------------------------------------------------------------------
bool Vtk3dVisualizer::getDrawXGridlines() const
{
    return d->cubeAxesActor->GetDrawXGridlines();
}

//----------------------------------------------------------------------------------------------------------------------------------
void Vtk3dVisualizer::setDrawXGridlines(const bool &draw)
{
    d->cubeAxesActor->SetDrawXGridlines(draw);
    updateCanvas();
    updatePropertyDock();
}

//----------------------------------------------------------------------------------------------------------------------------------
bool Vtk3dVisualizer::getDrawYGridlines() const
{
    return d->cubeAxesActor->GetDrawYGridlines();
}

//----------------------------------------------------------------------------------------------------------------------------------
void Vtk3dVisualizer::setDrawYGridlines(const bool &draw)
{
    d->cubeAxesActor->SetDrawYGridlines(draw);
    updateCanvas();
    updatePropertyDock();
}

//----------------------------------------------------------------------------------------------------------------------------------
bool Vtk3dVisualizer::getDrawZGridlines() const
{
    return d->cubeAxesActor->GetDrawZGridlines();
}

//----------------------------------------------------------------------------------------------------------------------------------
void Vtk3dVisualizer::setDrawZGridlines(const bool &draw)
{
    d->cubeAxesActor->SetDrawZGridlines(draw);
    updateCanvas();
    updatePropertyDock();
}

//----------------------------------------------------------------------------------------------------------------------------------
bool Vtk3dVisualizer::getxTicksVisibility() const
{
    return d->cubeAxesActor->GetXAxisTickVisibility();
}

//----------------------------------------------------------------------------------------------------------------------------------
void Vtk3dVisualizer::setxTicksVisibility(const bool &visible)
{
    d->cubeAxesActor->SetXAxisTickVisibility(visible);
    updateCanvas();
    updatePropertyDock();
}

//----------------------------------------------------------------------------------------------------------------------------------
bool Vtk3dVisualizer::getyTicksVisibility() const
{
    return d->cubeAxesActor->GetYAxisTickVisibility();
}

//----------------------------------------------------------------------------------------------------------------------------------
void Vtk3dVisualizer::setyTicksVisibility(const bool &visible)
{
    d->cubeAxesActor->SetYAxisTickVisibility(visible);
    updateCanvas();
    updatePropertyDock();
}

//----------------------------------------------------------------------------------------------------------------------------------
bool Vtk3dVisualizer::getzTicksVisibility() const
{
    return d->cubeAxesActor->GetZAxisTickVisibility();
}

//----------------------------------------------------------------------------------------------------------------------------------
void Vtk3dVisualizer::setzTicksVisibility(const bool &visible)
{
    d->cubeAxesActor->SetZAxisTickVisibility(visible);
    updateCanvas();
    updatePropertyDock();
}

//----------------------------------------------------------------------------------------------------------------------------------
bool Vtk3dVisualizer::getxMinorTicksVisibility() const
{
    return d->cubeAxesActor->GetXAxisMinorTickVisibility();
}

//----------------------------------------------------------------------------------------------------------------------------------
void Vtk3dVisualizer::setxMinorTicksVisibility(const bool &visible)
{
    d->cubeAxesActor->SetXAxisMinorTickVisibility(visible);
    updateCanvas();
    updatePropertyDock();
}

//----------------------------------------------------------------------------------------------------------------------------------
bool Vtk3dVisualizer::getyMinorTicksVisibility() const
{
    return d->cubeAxesActor->GetYAxisMinorTickVisibility();
}

//----------------------------------------------------------------------------------------------------------------------------------
void Vtk3dVisualizer::setyMinorTicksVisibility(const bool &visible)
{
    d->cubeAxesActor->SetYAxisMinorTickVisibility(visible);
    updateCanvas();
    updatePropertyDock();
}

//----------------------------------------------------------------------------------------------------------------------------------
bool Vtk3dVisualizer::getzMinorTicksVisibility() const
{
    return d->cubeAxesActor->GetZAxisMinorTickVisibility();
}

//----------------------------------------------------------------------------------------------------------------------------------
void Vtk3dVisualizer::setzMinorTicksVisibility(const bool &visible)
{
    d->cubeAxesActor->SetZAxisMinorTickVisibility(visible);
    updateCanvas();
    updatePropertyDock();
}


//-----------------------------------------------------------------------
QColor Vtk3dVisualizer::getBackgroundColor() const
{
    return d->backgroundColor;
}

//-----------------------------------------------------------------------
void Vtk3dVisualizer::setBackgroundColor(const QColor& color)
{
    QRgb rgb = color.rgb();
    double r = qRed(rgb) / 256.0;
    double g = qGreen(rgb) / 256.0;
    double b = qBlue(rgb) / 256.0;
    d->PCLVis->setBackgroundColor(r, g, b);
    d->backgroundColor = color;
    updateCanvas();
    updatePropertyDock();
}


//-----------------------------------------------------------------------
bool Vtk3dVisualizer::getShowFPS() const
{
    return d->showFPS;
}

//-----------------------------------------------------------------------
void Vtk3dVisualizer::setShowFPS(const bool& showFPS)
{
    d->PCLVis->setShowFPS(showFPS);
    d->showFPS = showFPS;
    updateCanvas();
    updatePropertyDock();
}



//-----------------------------------------------------------------------
Vtk3dVisualizer::Stereo Vtk3dVisualizer::getStereoType() const
{
    return d->stereoType;
}

//-----------------------------------------------------------------------
void Vtk3dVisualizer::setStereoType(const Stereo& stereoType)
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

    d->stereoType = stereoType;

    vtkSmartPointer<vtkRenderWindow> win = d->PCLVis->getRenderWindow();

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

    updateCanvas();
    updatePropertyDock();
}

//-----------------------------------------------------------------------
bool Vtk3dVisualizer::getCoordSysVisible() const
{
    return d->coordinateSysVisible;
}

//-----------------------------------------------------------------------
void Vtk3dVisualizer::setCoordSysVisible(const bool& coordSysVisible)
{
    d->coordinateSysVisible = coordSysVisible;
#if PCL_VERSION_COMPARE(>=,1,7,1)
    d->PCLVis->removeCoordinateSystem("mainCoordinateSystem");
    if (d->coordinateSysVisible)
    {
        d->PCLVis->addCoordinateSystem(d->coordinateSysScale, d->coordinateSysPos.x(), d->coordinateSysPos.y(), d->coordinateSysPos.z(), "mainCoordinateSystem");
    }
#else
    d->PCLVis->removeCoordinateSystem();
    if (d->coordinateSysVisible)
    {
        d->PCLVis->addCoordinateSystem(d->coordinateSysScale, d->coordinateSysPos.x(), d->coordinateSysPos.y(), d->coordinateSysPos.z());
    }
#endif
    updateCanvas();
    updatePropertyDock();
}

//-----------------------------------------------------------------------
double Vtk3dVisualizer::getCoordSysScale() const
{
    return d->coordinateSysScale;
}

//-----------------------------------------------------------------------
void Vtk3dVisualizer::setCoordSysScale(const double& coordSysScale)
{
    d->coordinateSysScale = coordSysScale;
#if PCL_VERSION_COMPARE(>=,1,7,1)
    d->PCLVis->removeCoordinateSystem("mainCoordinateSystem");
    if (d->coordinateSysVisible)
    {
        d->PCLVis->addCoordinateSystem(d->coordinateSysScale, d->coordinateSysPos.x(), d->coordinateSysPos.y(), d->coordinateSysPos.z(), "mainCoordinateSystem");
    }
#else
    d->PCLVis->removeCoordinateSystem();
    if (d->coordinateSysVisible)
    {
        d->PCLVis->addCoordinateSystem(d->coordinateSysScale, d->coordinateSysPos.x(), d->coordinateSysPos.y(), d->coordinateSysPos.z());
    }
#endif
    updateCanvas();
    updatePropertyDock();
}

//-----------------------------------------------------------------------
QVector3D Vtk3dVisualizer::getCameraPosition() const
{
    std::vector<pcl::visualization::Camera> cameras;
    d->PCLVis->getCameras(cameras);
    if (cameras.size() > 0)
    {
        return QVector3D(cameras[0].pos[0], cameras[0].pos[1], cameras[0].pos[2]);
    }

    return QVector3D(0.0, 0.0, 0.0);
}

//-----------------------------------------------------------------------
void Vtk3dVisualizer::setCameraPosition(const QVector3D& cameraPosition)
{
    std::vector<pcl::visualization::Camera> cameras;
    d->PCLVis->getCameras(cameras);
    if (cameras.size() > 0)
    {
        d->PCLVis->setCameraPosition(cameraPosition.x(), cameraPosition.y(), cameraPosition.z(), \
            cameras[0].focal[0], cameras[0].focal[1], cameras[0].focal[2], \
            cameras[0].view[0], cameras[0].view[1], cameras[0].view[2]);
        updateCanvas();
        updatePropertyDock();
    }
}

//-----------------------------------------------------------------------
QVector3D Vtk3dVisualizer::getCameraView() const
{
    std::vector<pcl::visualization::Camera> cameras;
    d->PCLVis->getCameras(cameras);
    if (cameras.size() > 0)
    {
        return QVector3D(cameras[0].view[0], cameras[0].view[1], cameras[0].view[2]);
    }

    return QVector3D(0.0, 0.0, 0.0);
}

//-----------------------------------------------------------------------
void Vtk3dVisualizer::setCameraView(const QVector3D& cameraView)
{
    std::vector<pcl::visualization::Camera> cameras;
    d->PCLVis->getCameras(cameras);
    if (cameras.size() > 0)
    {
        d->PCLVis->setCameraPosition(cameras[0].pos[0], cameras[0].pos[1], cameras[0].pos[2], \
            cameras[0].focal[0], cameras[0].focal[1], cameras[0].focal[2], \
            cameraView.x(), cameraView.y(), cameraView.z());
        updateCanvas();
        updatePropertyDock();
    }
}

//-----------------------------------------------------------------------
QVector3D Vtk3dVisualizer::getCameraFocalPoint() const
{
    std::vector<pcl::visualization::Camera> cameras;
    d->PCLVis->getCameras(cameras);
    if (cameras.size() > 0)
    {
        return QVector3D(cameras[0].focal[0], cameras[0].focal[1], cameras[0].focal[2]);
    }

    return QVector3D(0.0, 0.0, 0.0);
}

//-----------------------------------------------------------------------
void Vtk3dVisualizer::setCameraFocalPoint(const QVector3D& focalPoint)
{
    std::vector<pcl::visualization::Camera> cameras;
    d->PCLVis->getCameras(cameras);
    if (cameras.size() > 0)
    {
        d->PCLVis->setCameraPosition(cameras[0].pos[0], cameras[0].pos[1], cameras[0].pos[2], \
            focalPoint.x(), focalPoint.y(), focalPoint.z(), \
            cameras[0].view[0], cameras[0].view[1], cameras[0].view[2]);
        updateCanvas();
        updatePropertyDock();
    }
}

//-----------------------------------------------------------------------
QVector3D Vtk3dVisualizer::getCoordSysPos() const
{
    return d->coordinateSysPos;
}

//-----------------------------------------------------------------------
void Vtk3dVisualizer::setCoordSysPos(const QVector3D& coordSysPos)
{
    d->coordinateSysPos = coordSysPos;
#if PCL_VERSION_COMPARE(>=,1,7,1)
    d->PCLVis->removeCoordinateSystem("mainCoordinateSystem");
    if (d->coordinateSysVisible)
    {
        d->PCLVis->addCoordinateSystem(d->coordinateSysScale, d->coordinateSysPos.x(), d->coordinateSysPos.y(), d->coordinateSysPos.z(), "mainCoordinateSystem");
    }
#else
    d->PCLVis->removeCoordinateSystem();
    if (d->coordinateSysVisible)
    {
        d->PCLVis->addCoordinateSystem(d->coordinateSysScale, d->coordinateSysPos.x(), d->coordinateSysPos.y(), d->coordinateSysPos.z());
    }
#endif
    updateCanvas();
    updatePropertyDock();
}

//-----------------------------------------------------------------------
bool Vtk3dVisualizer::getParallelProjection() const
{
    return d->PCLVis->getRendererCollection()->GetFirstRenderer()->GetActiveCamera()->GetParallelProjection() > 0;
}

//-----------------------------------------------------------------------
void Vtk3dVisualizer::setParallelProjection(const bool& on)
{
    d->PCLVis->getRendererCollection()->GetFirstRenderer()->GetActiveCamera()->SetParallelProjection(on ? 1 : 0);
    updateCanvas();
    updatePropertyDock();
}

//-----------------------------------------------------------------------
QString Vtk3dVisualizer::getTitle() const
{
    return d->title;
}

//-----------------------------------------------------------------------
void Vtk3dVisualizer::setTitle(const QString& title)
{
    if (title != d->title)
    {
        this->setWindowTitleExtension(title);
        d->title = title;
    }
    
}

//----------------------------------------------------------------------------------------------------------------------------------
void Vtk3dVisualizer::createActions()
{
    QAction *a = NULL;

    d->actions["propertyDock"] = a = getPropertyDockWidget()->toggleViewAction();
    a->setIcon(QIcon(":/vtk3dVisualizer/icons/settings.png"));
    d->actions["settingsDock"] = a = d->dockSettings->toggleViewAction();
    a->setIcon(QIcon(":/vtk3dVisualizer/icons/itemsSettings.png"));
    d->actions["itemsDock"] = a = d->dockItems->toggleViewAction();
    a->setIcon(QIcon(":/vtk3dVisualizer/icons/items.png"));
}

//-------------------------------------------------------------------------------------
void Vtk3dVisualizer::updateCanvas()
{
#ifdef LEGACY_VTK
    //old opengl
    d->pclCanvas->update();
#else
    //new opengl
    if (!d->canvasUpdateQueued)
    {
        // the following delayed update should avoid updating to often if
        // many things are changed at almost the same time.
        d->canvasUpdateQueued = true;
        QTimer::singleShot(10, this, &Vtk3dVisualizer::updateCanvasImmediately);
    }
#endif
}

//-------------------------------------------------------------------------------------
void Vtk3dVisualizer::updateCanvasImmediately()
{
#ifdef LEGACY_VTK
    // old opengl
    d->pclCanvas->update();
#else
    //new opengl
    d->canvasUpdateQueued = false;
    d->pclCanvas->GetRenderWindow()->Render();
#endif
}