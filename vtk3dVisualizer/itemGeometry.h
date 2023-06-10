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

#ifndef ITEMGEOMETRY_H
#define ITEMGEOMETRY_H

#include "common/sharedStructures.h"
#include "DataObject/dataobj.h"
#include "PointCloud/pclStructures.h"
#include "pcl/visualization/pcl_visualizer.h"

#include "item.h"

#include <qcolor.h>
#include <qvector.h>

class ItemGeometry : public Item
{
    Q_OBJECT


    //Q_PROPERTY(bool selected READ selected WRITE setSelected DESIGNABLE true USER true);
    //Q_PROPERTY(int PointSize READ pointSize WRITE setPointSize DESIGNABLE true USER true);
    Q_PROPERTY(double LineWidth READ lineWidth WRITE setLineWidth DESIGNABLE true USER true);
    Q_PROPERTY(int Opacity READ opacity WRITE setOpacity DESIGNABLE true USER true);
    Q_CLASSINFO("Opacity", "minimum=0;maximum=100;singleStep=1;");
    Q_PROPERTY(bool Lighting READ lighting WRITE setLighting DESIGNABLE true USER true);
    Q_PROPERTY(Representation Representation READ representation WRITE setRepresentation DESIGNABLE true USER true);
    Q_PROPERTY(Interpolation Interpolation READ interpolation WRITE setInterpolation DESIGNABLE true USER true);
    Q_PROPERTY(QColor LineColor READ lineColor WRITE setLineColor DESIGNABLE true USER true);
    Q_PROPERTY(double Specular READ specular WRITE setSpecular DESIGNABLE true USER true);
    Q_CLASSINFO("Specular", "minimum=0.0;maximum=1.0;singleStep=0.1;");
    Q_PROPERTY(double SpecularPower READ specularPower WRITE setSpecularPower DESIGNABLE true USER true);
    Q_CLASSINFO("SpecularPower", "minimum=0.0;maximum=1.0;singleStep=0.1;");
    Q_PROPERTY(QColor SpecularColor READ specularColor WRITE setSpecularColor DESIGNABLE true USER true);

public:
    ItemGeometry(boost::shared_ptr<pcl::visualization::PCLVisualizer> visualizer, const QString &name, QTreeWidgetItem *treeItem);
    ~ItemGeometry();

    enum Type { tCylinder, tPlane, tCircle, tCone, tCube, tPyramid, tCuboid, tSphere, tText, tLines, tPolygon };
    enum Representation { Points , Wireframe, Surface }; //equals pcl::visualization::RenderingRepresentationProperties
    enum Interpolation { Flat, Gouraud, Phong };

    //Q_ENUM exposes a meta object to the enumeration types, such that the key names for the enumeration
    //values are always accessible.
    Q_ENUM(Representation)
    Q_ENUM(Interpolation)

    ito::RetVal addText(const QString &text, const int x, const int y, const int fontsize, const QColor &color);
    ito::RetVal addCylinder(const pcl::ModelCoefficients &coefficients, const QColor &color);
    ito::RetVal addPyramid(const ito::DataObject *points, const QColor &color);
    ito::RetVal addCuboid(const ito::DataObject *points, const QColor &color);
    ito::RetVal addCube(const Eigen::Vector3f &size, const Eigen::Affine3f &pose, const QColor &color);
    ito::RetVal addLines(const ito::DataObject *points, const QColor &color);
    ito::RetVal addSphere(const pcl::PointXYZ &center, double radius, const QColor &color);
    ito::RetVal addPolygon(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr polygon, const QColor &color);
    ito::RetVal updatePose(const Eigen::Affine3f &pose);
    ito::RetVal updateText(const QString &text, const int x, const int y, const int fontsize, const QColor &color);

    //properties
    virtual void setVisible(bool value);

    Representation representation() const { return m_representation; }
    void setRepresentation(Representation value);

    Interpolation interpolation() const { return m_interpolation; }
    void setInterpolation(Interpolation value);

    QColor lineColor() const { return m_lineColor; }
    void setLineColor(QColor color);

    double lineWidth() const { return m_lineWidth; }
    void setLineWidth(double value);

    int opacity() const { return m_opacity; }
    void setOpacity(int value);

    bool lighting() const { return m_lighting; }
    void setLighting(bool value);

    double specular() const { return m_specular; }
    void setSpecular(double value);

    double specularPower() const { return m_specularPower; }
    void setSpecularPower(double value);

    QColor specularColor() const { return m_specularColor; }
    void setSpecularColor(QColor color);

protected:
    QVector<vtkProp*> getSafeActors(); //tries to find m_actor in list of current actors of renderer
    vtkProp *getLastActor();
    void syncActorProperties(vtkProp *actor);

    boost::shared_ptr<pcl::visualization::PCLVisualizer> m_visualizer;

    Type m_geometryType;
    pcl::ModelCoefficients m_coefficients;
    Representation m_representation;
    QColor m_lineColor;
    double m_lineWidth;
    int m_opacity;
    Interpolation m_interpolation;
    QVector<vtkProp*> m_actors; //reference to actor(s), this is a guess since no access to the shapeactormap of m_visualizer is available. don't use this, always get the actor using getSafeActor.
    bool m_lighting;
    int m_nrOfShapes;
    double m_specular;
    double m_specularPower;
    QColor m_specularColor;

};





#endif //ITEM_H
