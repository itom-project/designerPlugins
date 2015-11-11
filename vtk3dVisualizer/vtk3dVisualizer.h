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

#ifndef VTK3DVISUALIZER_H
#define VTK3DVISUALIZER_H

#if defined(ITOMSHAREDDESIGNER)
    #define VTK3DVISUALIZER_EXPORT Q_DECL_EXPORT
#else
    #define VTK3DVISUALIZER_EXPORT Q_DECL_IMPORT
#endif

#include "plot/AbstractDObjFigure.h"

#include "common/sharedStructures.h"
#include "PointCloud/pclStructures.h"

#include "DataObject/dataobj.h"

#include "pcl/visualization/pcl_visualizer.h"

#include <qmainwindow.h>
#include <qwidget.h>
#include <qstring.h>
#include <qtreewidget.h>

#include <string>



class QPropertyEditorWidget; //forward declaration
class Vtk3dVisualizerPrivate; //forward declaration
class QEvent;

class VTK3DVISUALIZER_EXPORT Vtk3dVisualizer : public ito::AbstractDObjFigure
{
    Q_OBJECT

    Q_PROPERTY(bool propertiesSorted READ propertiesSorted WRITE setPropertiesSorted DESIGNABLE true USER true);
    Q_PROPERTY(bool enablePointPick READ enablePointPick WRITE setEnablePointPick DESIGNABLE true USER true);
    Q_PROPERTY(double pointPickSphereRadius READ pointPickSphereRadius WRITE setPointPickSphereRadius DESIGNABLE true USER true);
    Q_PROPERTY(QColor pointPickSphereColor READ pointPickSphereColor WRITE setpointPickSphereColor DESIGNABLE true USER true);

    Q_CLASSINFO("prop://propertiesSorted", "sort the properties of one item in an alphabetical order or not")
    Q_CLASSINFO("prop://enablePointPick", "if True, a click to any point of the canvas emits the signal pointPicked that emits the currently clicked 3d coordinate and the index of the closest point of the cloud / mesh that has been given as pickPointCloud or pickMesh.")
    Q_CLASSINFO("prop://pointPickSphereRadius", "If > 0, a sphere with the given radius is printed around the center point of the point pick event (if enabled)")
    Q_CLASSINFO("prop://pointPickSphereColor", "Color of the possible sphere of the point pick event (see pointPickShereRadius and enablePointPick)")

    Q_CLASSINFO("slot://registerModel", "see addMesh")
    Q_CLASSINFO("slot://addMesh", "add the given mesh to the tree with a key name (arguments: mesh, key)")
    Q_CLASSINFO("slot://addPointCloud", "add the given cloud to the tree with a key name (arguments: cloud, key)")
    Q_CLASSINFO("slot://addPointCloudNormal", "add the given cloud with normal vectors to the tree with a key name (arguments: cloud, key)")
    Q_CLASSINFO("slot://updatePointCloud", "updates an existing cloud (arguments: cloud, key, createIfNotExists=false)")
    Q_CLASSINFO("slot://addCylinder", "add a cylinder (arguments: (center_x, center_y, center_y), (orientation_x, orientation_y, orientation_z), radius, key, color=white)")
    Q_CLASSINFO("slot://addPyramid", "add pyramid. The 5 corner points are given in a 3x5 data object (arguments: points, key, color=white)")
    Q_CLASSINFO("slot://addCuboid", "add cube. The 8 corner points are given in a 3x8 data object (arguments: points, key, color=white)")
    Q_CLASSINFO("slot://addCube", "add cube (arguments: [size_x, size_y, size_z], [t_x, t_y, t_z], [r_x, r_y, r_z], key, color=white)")
    Q_CLASSINFO("slot://addLines", "add m lines to the canvas. The coordintates are given by a float32 data object [m x 6] where one row is (x0,y0,z0,x1,y1,z1) (arguments: points, key, color=red).")
    Q_CLASSINFO("slot://addSphere", "add a sphere (arguments: [center_x, center_y, center_z], radius, key, color = red]).")
    Q_CLASSINFO("slot://addText", "add a 2d text to a specific position (arguments: text, x, y, fontsize, key, color = white)")
    Q_CLASSINFO("slot://addPolygon", "adds a 2d polygon (arguments: points, key, color = white) where points is a Mx3 data object where each line is the (x,y,z) coordinate of a point of the polygon. The point values will internally be converted to float32.")
    Q_CLASSINFO("slot://updateText", "updates or creates new 2d text (arguments: text, x, y, fontsize, key, color = white, createIfNotExists = false)")
    Q_CLASSINFO("slot://deletePointCloud", "delete the cloud with key")
    Q_CLASSINFO("slot://deleteMesh", "delete the mesh with key")
    Q_CLASSINFO("slot://deleteGeometry", "delete the geometry with key")
    Q_CLASSINFO("slot://setGeometryPose", "moves and/or rotates a given geometry (arguments: key, (t_x, t_y, t_z), (r_x, r_y, r_z)")
    Q_CLASSINFO("slot://setGeometriesPosition", "changes the position of various geometries (arguments: list of keys, list of (pos_x, pos_y, pos_z)")
    Q_CLASSINFO("slot://setItemProperty", "set the property of an item (arguments: key, property-name, value)")
    Q_CLASSINFO("slot://setPickPointCloud", "set cloud for pick point event. Nearest point from the position of the cursor (x,y,z) position is searched (arguments: cloud)")
    Q_CLASSINFO("slot://setPickPointMesh", "set mesh for pick point event. The cloud of the mesh is used only (arguments: mesh)")
    
    DESIGNER_PLUGIN_ITOM_API

public:
    Vtk3dVisualizer(const QString &itomSettingsFile, AbstractFigure::WindowMode windowMode, QWidget *parent = 0);
    virtual ~Vtk3dVisualizer();

    ito::RetVal applyUpdate(); 

    bool propertiesSorted() const;
    void setPropertiesSorted(bool value);

    bool enablePointPick() const;
    void setEnablePointPick(bool enabled);

    double pointPickSphereRadius() const;
    void setPointPickSphereRadius(double radius);

    QColor pointPickSphereColor() const;
    void setpointPickSphereColor(QColor color);

    bool getContextMenuEnabled() const { return false; };
    void setContextMenuEnabled(bool show) {}; 

protected:
    ito::RetVal init();

private:
    Vtk3dVisualizerPrivate* d; /*! private data pointer of this class. */

    void point_picking_callback (const pcl::visualization::PointPickingEvent& event, void* cookie);

    ito::RetVal createRecursiveTree(QString &path, QTreeWidgetItem *currentParent, QTreeWidgetItem **newParent);
    ito::RetVal searchRecursiveTree(const QString &path, QTreeWidgetItem *currentParent, QTreeWidgetItem **item);
    ito::RetVal deleteItem(const QString &name, QTreeWidgetItem *rootItem);

public slots:
    void registerModel(ito::PCLPolygonMesh mesh, QString modelName);
    ito::RetVal addMesh(ito::PCLPolygonMesh mesh, const QString &fullname);

    ito::RetVal addPointCloud(ito::PCLPointCloud pcl, const QString &name);
    ito::RetVal addPointCloudNormal(ito::PCLPointCloud pcl, const QString &fullname);

    ito::RetVal updatePointCloud(ito::PCLPointCloud pcl, const QString &name, bool createIfNotExists = false);

    ito::RetVal addCylinder(QVector<double> point, QVector<double> orientation, double radius, const QString &fullname, const QColor &color = Qt::white);

    ito::RetVal addPyramid(const ito::DataObject &points, const QString &fullname, const QColor &color = Qt::white); //points must be a 3x5 float matrix

    ito::RetVal addCuboid(const ito::DataObject &points, const QString &fullname, const QColor &color = Qt::white); //points must be a 3x8 float matrix

    ito::RetVal addCube(QVector<double> size, QVector<double> translation, QVector<double> rotation, const QString &fullname, const QColor &color = Qt::white);

    ito::RetVal addLines(const ito::DataObject &points, const QString &fullname, const QColor &color = Qt::red);

    ito::RetVal addSphere(QVector<double> point, double radius, const QString &fullname, const QColor &color = Qt::red);

    ito::RetVal addPolygon(const ito::DataObject &points, const QString &fullname, const QColor &color = Qt::red);

    ito::RetVal addText(const QString &text, const int x, const int y, const int fontsize, const QString &fullname, const QColor &color = Qt::white);
    ito::RetVal updateText(const QString &text, const int x, const int y, const int fontsize, const QString &name, const QColor &color = Qt::white, bool createIfNotExists = false);

    ito::RetVal deletePointCloud(const QString &name);
    ito::RetVal deleteMesh(const QString &name);
    ito::RetVal deleteGeometry(const QString &name);
    ito::RetVal setGeometryPose(const QString &name, QVector<double> translation, QVector<double> rotation);
    ito::RetVal setGeometriesPosition(const QStringList &names, QVector<double> positions);

    ito::RetVal setItemProperty(const QString &name, const QByteArray &property, const QVariant &value);

    ito::RetVal setPickPointCloud(ito::PCLPointCloud &pcl);
    ito::RetVal setPickPointMesh(ito::PCLPolygonMesh &mesh);

private slots:
    void itemClicked(QTreeWidgetItem *item, int column);

signals:
    void pointPicked(float x, float y, float z, int pointIndex);

};

#endif //VTK3DVISUALIZER_H
