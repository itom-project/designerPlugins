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

    ito::RetVal addBox(QVector<double> minimums, QVector<double> maximums, QVector<double> translation, QVector<double> rotation, const QString &fullname, const QColor &color = Qt::white);

    ito::RetVal addLines(const ito::DataObject &points, const QString &fullname, const QColor &color = Qt::red);

    ito::RetVal deletePointCloud(const QString &name);
    ito::RetVal deleteMesh(const QString &name);
    ito::RetVal deleteGeometry(const QString &name);

    ito::RetVal setItemProperty(const QString &name, const QByteArray &property, const QVariant &value);

    ito::RetVal setPickPointCloud(ito::PCLPointCloud &pcl);
    ito::RetVal setPickPointMesh(ito::PCLPolygonMesh &mesh);

private slots:
    void itemClicked(QTreeWidgetItem *item, int column);

signals:
    void pointPicked(float x, float y, float z, int pointIndex);

};

#endif //VTK3DVISUALIZER_H
