#ifndef ITEMGEOMETRY_H
#define ITEMGEOMETRY_H

#include "common/sharedStructures.h"
#include "DataObject/dataobj.h"
#include "PointCloud/pclStructures.h"
#include "pcl/visualization/pcl_visualizer.h"

#include "item.h"

#include <qcolor.h>

class ItemGeometry : public Item
{
    Q_OBJECT
    Q_ENUMS(Representation)

    //Q_PROPERTY(bool selected READ selected WRITE setSelected DESIGNABLE true USER true);
    //Q_PROPERTY(int PointSize READ pointSize WRITE setPointSize DESIGNABLE true USER true);
    Q_PROPERTY(double LineWidth READ lineWidth WRITE setLineWidth DESIGNABLE true USER true);
    Q_PROPERTY(double Opacity READ opacity WRITE setOpacity DESIGNABLE true USER true);
    Q_PROPERTY(Representation Representation READ representation WRITE setRepresentation DESIGNABLE true USER true);
    Q_PROPERTY(QColor LineColor READ lineColor WRITE setLineColor DESIGNABLE true USER true);

public:
    enum Type { tCylinder, tPlane, tCircle, tCone, tCube, tPyramid, tCuboid };
    ItemGeometry(boost::shared_ptr<pcl::visualization::PCLVisualizer> visualizer, const QString &name, QTreeWidgetItem *treeItem);
    ~ItemGeometry();

    enum Representation { Points , Wireframe, Surface }; //equals pcl::visualization::RenderingRepresentationProperties

    ito::RetVal addCylinder(const pcl::ModelCoefficients &coefficients, const QColor &color);
    ito::RetVal addPyramid(const ito::DataObject *points, const QColor &color);
    ito::RetVal addCuboid(const ito::DataObject *points, const QColor &color);
    ito::RetVal addLines(const ito::DataObject *points, const QColor &color);

    //properties
    virtual void setVisible(bool value);

    Representation representation() const { return m_representation; }
    void setRepresentation(Representation value);

    QColor lineColor() const { return m_lineColor; }
    void setLineColor(QColor color);

    double lineWidth() const { return m_lineWidth; }
    void setLineWidth(double value);

    double opacity() const { return m_opacity; }
    void setOpacity(double value);

protected:
    boost::shared_ptr<pcl::visualization::PCLVisualizer> m_visualizer;
    
    Type m_geometryType;
    pcl::ModelCoefficients m_coefficients;
    Representation m_representation;
    QColor m_lineColor;
    double m_lineWidth;
    double m_opacity;

};





#endif //ITEM_H