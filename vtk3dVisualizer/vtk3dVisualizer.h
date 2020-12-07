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

#ifndef VTK3DVISUALIZER_H
#define VTK3DVISUALIZER_H

#if defined(ITOMSHAREDDESIGNER)
#define VTK3DVISUALIZER_EXPORT Q_DECL_EXPORT
#else
#define VTK3DVISUALIZER_EXPORT Q_DECL_IMPORT
#endif

#include "plot/AbstractDObjPCLFigure.h"

#include "common/sharedStructures.h"
#include "PointCloud/pclStructures.h"

#include "DataObject/dataobj.h"

#include "pcl/visualization/pcl_visualizer.h"

#include <qmainwindow.h>
#include <qwidget.h>
#include <qstring.h>
#include <qtreewidget.h>

#include <string>
#include <QVector3D>




class QPropertyEditorWidget; //forward declaration
class Vtk3dVisualizerPrivate; //forward declaration
class QEvent;

class VTK3DVISUALIZER_EXPORT Vtk3dVisualizer : public ito::AbstractDObjPclFigure
{
    Q_OBJECT

    Q_PROPERTY(bool propertiesSorted READ propertiesSorted WRITE setPropertiesSorted DESIGNABLE true USER true);
    Q_PROPERTY(bool enablePointPick READ enablePointPick WRITE setEnablePointPick DESIGNABLE true USER true);
    Q_PROPERTY(double pointPickSphereRadius READ pointPickSphereRadius WRITE setPointPickSphereRadius DESIGNABLE true USER true);
    Q_PROPERTY(QColor pointPickSphereColor READ pointPickSphereColor WRITE setpointPickSphereColor DESIGNABLE true USER true);
    
    Q_PROPERTY(bool cubeAxesVisible READ getCubeAxesVisible WRITE setCubeAxesVisible USER true)
    Q_PROPERTY(QColor cubeAxesColor READ getCubeAxesColor WRITE setCubeAxesColor USER true)
    Q_PROPERTY(QColor cubeGridlinesColor READ getCubeGridlinesColor WRITE setCubeGridlinesColor USER true)
    Q_PROPERTY(FlyMode cubeAxesFlyMode READ getCubeAxesFlyMode WRITE setCubeAxesFlyMode USER true)
    Q_PROPERTY(TickLocation cubeAxesTickLocation READ getCubeAxesTickLocation WRITE setCubeAxesTickLocation USER true)
    Q_PROPERTY(bool enableDistanceLOD READ getEnableDistanceLOD WRITE setEnableDistanceLOD USER true)
    Q_PROPERTY(bool enableViewAngleLOD READ getEnableViewAngleLOD WRITE setEnableViewAngleLOD USER true)
    
    Q_PROPERTY(QString xAxisLabel READ getxAxisLabel WRITE setxAxisLabel USER true)
    Q_PROPERTY(QString yAxisLabel READ getyAxisLabel WRITE setyAxisLabel USER true)
    Q_PROPERTY(QString zAxisLabel READ getzAxisLabel WRITE setzAxisLabel USER true)

    Q_PROPERTY(bool xAxisVisible READ getxAxisVisible WRITE setxAxisVisible USER true)
    Q_PROPERTY(bool yAxisVisible READ getyAxisVisible WRITE setyAxisVisible USER true)
    Q_PROPERTY(bool zAxisVisible READ getzAxisVisible WRITE setzAxisVisible USER true)

    Q_PROPERTY(bool xDrawGridlines READ getDrawXGridlines WRITE setDrawXGridlines USER true)
    Q_PROPERTY(bool yDrawGridlines READ getDrawYGridlines WRITE setDrawYGridlines USER true)
    Q_PROPERTY(bool zDrawGridlines READ getDrawZGridlines WRITE setDrawZGridlines USER true)

    Q_PROPERTY(bool xAxisTickVisibility READ getxTicksVisibility WRITE setxTicksVisibility USER true)
    Q_PROPERTY(bool yAxisTickVisibility READ getyTicksVisibility WRITE setyTicksVisibility USER true)
    Q_PROPERTY(bool zAxisTickVisibility READ getzTicksVisibility WRITE setzTicksVisibility USER true)

    Q_PROPERTY(bool xAxisMinorTickVisibility READ getxMinorTicksVisibility WRITE setxMinorTicksVisibility USER true)
    Q_PROPERTY(bool yAxisMinorTickVisibility READ getyMinorTicksVisibility WRITE setyMinorTicksVisibility USER true)
    Q_PROPERTY(bool zAxisMinorTickVisibility READ getzMinorTicksVisibility WRITE setzMinorTicksVisibility USER true)

    Q_PROPERTY(QColor backgroundColor READ getBackgroundColor WRITE setBackgroundColor DESIGNABLE true USER true);
    Q_PROPERTY(bool showFPS READ getShowFPS WRITE setShowFPS DESIGNABLE true USER true)
    Q_PROPERTY(Stereo stereoType READ getStereoType WRITE setStereoType DESIGNABLE true USER true)
    Q_PROPERTY(double coordSysScale READ getCoordSysScale WRITE setCoordSysScale DESIGNABLE true USER true)
    Q_PROPERTY(bool coordSysVisible READ getCoordSysVisible WRITE setCoordSysVisible DESIGNABLE true USER true)
    Q_PROPERTY(bool parallelProjection READ getParallelProjection WRITE setParallelProjection DESIGNABLE true USER true)

    Q_PROPERTY(QVector3D coordSysPos READ getCoordSysPos WRITE setCoordSysPos DESIGNABLE true USER true)
    Q_CLASSINFO("coordSysPos", "minimumX=-2147483647;maximumX=2147483647;minimumY=-2147483647;maximumY=2147483647;minimumZ=-2147483647;maximumZ=2147483647;");
    Q_PROPERTY(QVector3D cameraPosition READ getCameraPosition WRITE setCameraPosition DESIGNABLE true USER true)
    Q_CLASSINFO("cameraPosition", "minimumX=-2147483647;maximumX=2147483647;minimumY=-2147483647;maximumY=2147483647;minimumZ=-2147483647;maximumZ=2147483647;");
    Q_PROPERTY(QVector3D cameraView READ getCameraView WRITE setCameraView DESIGNABLE true USER true)
    Q_CLASSINFO("cameraView", "minimumX=-2147483647;maximumX=2147483647;minimumY=-2147483647;maximumY=2147483647;minimumZ=-2147483647;maximumZ=2147483647;");
    Q_PROPERTY(QVector3D cameraFocalPoint READ getCameraFocalPoint WRITE setCameraFocalPoint DESIGNABLE true USER true)
    Q_CLASSINFO("cameraFocalPoint", "minimumX=-2147483647;maximumX=2147483647;minimumY=-2147483647;maximumY=2147483647;minimumZ=-2147483647;maximumZ=2147483647;");
        

    Q_CLASSINFO("prop://propertiesSorted", "sort the properties of one item in an alphabetical order or not")
    Q_CLASSINFO("prop://enablePointPick", "if True, a click to any point of the canvas emits the signal pointPicked that emits the currently clicked 3d coordinate and the index of the closest point of the cloud / mesh that has been given as pickPointCloud or pickMesh.")
    Q_CLASSINFO("prop://pointPickSphereRadius", "If > 0, a sphere with the given radius is printed around the center point of the point pick event (if enabled)")
    Q_CLASSINFO("prop://pointPickSphereColor", "Color of the possible sphere of the point pick event (see pointPickShereRadius and enablePointPick)")

    Q_CLASSINFO("prop://xAxisLabel", "Label of the x-axis.")
    Q_CLASSINFO("prop://yAxisLabel", "Label of the y-axis.")
    Q_CLASSINFO("prop://zAxisLabel", "Label of the z-axis.")

    Q_CLASSINFO("prop://xAxisVisible", "Sets the visibility of the x-axis.")
    Q_CLASSINFO("prop://yAxisVisible", "Sets the visibility of the y-axis.")
    Q_CLASSINFO("prop://zAxisVisible", "Sets the visibility of the z-axis.")

    Q_CLASSINFO("prop://xDrawGridlines", "Sets the visibility of gridlines along the x-axis.")
    Q_CLASSINFO("prop://yDrawGridlines", "Sets the visibility of gridlines along the y-axis.")
    Q_CLASSINFO("prop://zDrawGridlines", "Sets the visibility of gridlines along the z-axis.")

    Q_CLASSINFO("prop://xAxisTickVisibility", "Sets the visibility of major ticks along the x-axis.")
    Q_CLASSINFO("prop://yAxisTickVisibility", "Sets the visibility of major ticks along the y-axis.")
    Q_CLASSINFO("prop://zAxisTickVisibility", "Sets the visibility of major ticks along the z-axis.")

    Q_CLASSINFO("prop://xAxisMinorTickVisibility", "Sets the visibility of minor ticks along the x-axis.")
    Q_CLASSINFO("prop://yAxisMinorTickVisibility", "Sets the visibility of minor ticks along the y-axis.")
    Q_CLASSINFO("prop://zAxisMinorTickVisibility", "Sets the visibility of minor ticks along the z-axis.")

    Q_CLASSINFO("prop://cubeAxesVisible", "Overall visibility of the cube axes (must be set to True in order to see grids, labels, axes...)")
    Q_CLASSINFO("prop://cubeAxesColor", "sets the color of the cube axes")
    Q_CLASSINFO("prop://cubeGridlinesColor", "sets the color of the cube gridlines")
    Q_CLASSINFO("prop://cubeAxesFlyMode", "defines how the cube axes are positioned depending on the current camera")
    Q_CLASSINFO("prop://cubeAxesTickLocation", "defines the location of ticks for the cube axes")
    Q_CLASSINFO("prop://enableDistanceLOD", "If enabled the actor will not be visible at a certain distance from the camera")
    Q_CLASSINFO("prop://enableViewAngleLOD", "If enabled the actor will not be visible at a certain view angle")

    Q_CLASSINFO("prop://backgroundColor", "background color of the canvas")
    Q_CLASSINFO("prop://showFPS", "shows the FPS counter or not")
    Q_CLASSINFO("prop://stereoType", "sets the stereo type of the canvas")
    Q_CLASSINFO("prop://coordSysScale", "sets the length / scaling of the coordinate axes")
    Q_CLASSINFO("prop://coordSysVisible", "sets the visibility of a coordinate system")

    Q_CLASSINFO("prop://coordSysPos", "position of the coordinate system")
    Q_CLASSINFO("prop://cameraPosition", "position of the camera")
    Q_CLASSINFO("prop://cameraView", "view direction of the camera")
    Q_CLASSINFO("prop://cameraFocalPoint", "focal point of the camera")
    Q_CLASSINFO("prop://parallelProjection", "if true a parallel projection is used, else the perspective projection")

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
    enum FlyMode { flyOuterEdges = 0, flyClostestTriad = 1, flyFurthestTriad = 2, flyStaticTriad = 3, flyStaticEdges = 4 };
    enum TickLocation { ticksInside = 0, ticksOutside = 1, ticksBoth = 2 };
    enum Stereo { No, CrystalEyes, RedBlue, Interlaced, Left, Right, Dresden, Anaglyph, Checkerboard };

    //Q_ENUM exposes a meta object to the enumeration types, such that the key names for the enumeration
    //values are always accessible.
    Q_ENUM(FlyMode);
    Q_ENUM(TickLocation);
    Q_ENUM(Stereo);

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

    virtual ito::AutoInterval getXAxisInterval(void) const;
    virtual void setXAxisInterval(ito::AutoInterval interval);

    virtual ito::AutoInterval getYAxisInterval(void) const;
    virtual void setYAxisInterval(ito::AutoInterval interval);

    virtual ito::AutoInterval getZAxisInterval(void) const;
    virtual void setZAxisInterval(ito::AutoInterval interval);

    bool getxAxisVisible() const;
    void setxAxisVisible(const bool &value);

    bool getyAxisVisible() const;
    void setyAxisVisible(const bool &value);

    bool getzAxisVisible() const;
    void setzAxisVisible(const bool &value);

    QString getxAxisLabel() const;
    void setxAxisLabel(const QString &label);

    QString getyAxisLabel() const;
    void setyAxisLabel(const QString &label);

    QString getzAxisLabel() const;
    void setzAxisLabel(const QString &label);

    bool getCubeAxesVisible() const;
    void setCubeAxesVisible(const bool &visible);

    QColor getCubeAxesColor() const;
    void setCubeAxesColor(const QColor &color);

    QColor getCubeGridlinesColor() const;
    void setCubeGridlinesColor(const QColor &color);

    bool getEnableDistanceLOD() const;
    void setEnableDistanceLOD(const bool &enable);

    bool getEnableViewAngleLOD() const;
    void setEnableViewAngleLOD(const bool &enable);

    bool getDrawXGridlines() const;
    void setDrawXGridlines(const bool &draw);

    bool getDrawYGridlines() const;
    void setDrawYGridlines(const bool &draw);

    bool getDrawZGridlines() const;
    void setDrawZGridlines(const bool &draw);

    bool getxTicksVisibility() const;
    void setxTicksVisibility(const bool &visible);

    bool getyTicksVisibility() const;
    void setyTicksVisibility(const bool &visible);

    bool getzTicksVisibility() const;
    void setzTicksVisibility(const bool &visible);

    bool getxMinorTicksVisibility() const;
    void setxMinorTicksVisibility(const bool &visible);

    bool getyMinorTicksVisibility() const;
    void setyMinorTicksVisibility(const bool &visible);

    bool getzMinorTicksVisibility() const;
    void setzMinorTicksVisibility(const bool &visible);

    FlyMode getCubeAxesFlyMode() const;
    void setCubeAxesFlyMode(const FlyMode &mode);

    TickLocation getCubeAxesTickLocation() const;
    void setCubeAxesTickLocation(const TickLocation &location);

    QColor getBackgroundColor() const;
    void setBackgroundColor(const QColor& color);

    bool getShowFPS() const;
    void setShowFPS(const bool& showFPS);

    Stereo getStereoType() const;
    void setStereoType(const Stereo& stereoType);

    bool getCoordSysVisible() const;
    void setCoordSysVisible(const bool& coordSysVisible);

    double getCoordSysScale() const;
    void setCoordSysScale(const double& coordSysScale);

    QVector3D getCameraPosition() const;
    void setCameraPosition(const QVector3D& cameraPosition);

    QVector3D getCameraView() const;
    void setCameraView(const QVector3D& cameraView);

    QVector3D getCameraFocalPoint() const;
    void setCameraFocalPoint(const QVector3D& focalPoint);

    QVector3D getCoordSysPos() const;
    void setCoordSysPos(const QVector3D& coordSysPos);

    bool getParallelProjection() const;
    void setParallelProjection(const bool& on);

protected:
    ito::RetVal init();

    void createActions();

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
