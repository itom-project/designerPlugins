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