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

#ifndef ITEMPOLYGONMESH_H
#define ITEMPOLYGONMESH_H

#include "common/qtMetaTypeDeclarations.h"
#include "common/sharedStructures.h"
#include "common/interval.h"
#include "PointCloud/pclStructures.h"
#include "pcl/visualization/pcl_visualizer.h"

#include "common/sharedStructuresGraphics.h"

#include "item.h"
#include <qcolor.h>
#include <qmetatype.h>

//Q_DECLARE_METATYPE(ito::AutoInterval)


class ItemPolygonMesh : public Item
{
    Q_OBJECT

    //Q_PROPERTY(bool selected READ selected WRITE setSelected DESIGNABLE true USER true);
    Q_PROPERTY(Representation Representation READ representation WRITE setRepresentation DESIGNABLE true USER true);
    Q_PROPERTY(Interpolation Interpolation READ interpolation WRITE setInterpolation DESIGNABLE true USER true);
    Q_PROPERTY(Culling Culling READ culling WRITE setCulling DESIGNABLE true USER true);
    Q_PROPERTY(bool EdgeVisibility READ edgeVisibility WRITE setEdgeVisibility DESIGNABLE true USER true);
    Q_PROPERTY(int Opacity READ opacity WRITE setOpacity DESIGNABLE true USER true);
    Q_CLASSINFO("Opacity", "minimum=0;maximum=100;singleStep=1;");
    Q_PROPERTY(ColorMode ColorMode READ colorMode WRITE setColorMode DESIGNABLE true USER true);
    Q_PROPERTY(QColor FaceColor READ faceColor WRITE setFaceColor DESIGNABLE true USER true);
    Q_PROPERTY(ColorMap ColorMap READ colorMap WRITE setColorMap DESIGNABLE true USER true);
    Q_PROPERTY(ito::AutoInterval ColorValueRange READ colorValueRange WRITE setColorValueRange DESIGNABLE true USER true);

public:
    ItemPolygonMesh(boost::shared_ptr<pcl::visualization::PCLVisualizer> visualizer, const QString &name, QTreeWidgetItem *treeItem);
    virtual ~ItemPolygonMesh();

    ito::RetVal addPolygonMesh(const ito::PCLPolygonMesh &mesh);

    enum Representation { Points , Wireframe, Surface }; //equals pcl::visualization::RenderingRepresentationProperties
    enum Interpolation { Flat, Gouraud, Phong };
    enum Culling { ShowAll, BackfaceOnly, FrontfaceOnly };
    enum ColorMode { RGB, SolidColor, NormalX, NormalY, NormalZ, NormalMagnitude, X, Y, Z, XY, YZ, XZ, XYZ, Curvature};
    enum ColorMap { gray, grayMarked, falseColor, falseColorIR, hotIron, red, blue, green, viridis};

    //Q_ENUM exposes a meta object to the enumeration types, such that the key names for the enumeration
    //values are always accessible.
    Q_ENUM(Representation)
    Q_ENUM(Interpolation)
    Q_ENUM(Culling)
    Q_ENUM(ColorMode)
    Q_ENUM(ColorMap)

    //properties
    virtual void setVisible(bool value);

    //bool selected() const { return m_selected; }
    //void setSelected(bool value);

    Representation representation() const { return m_representation; }
    void setRepresentation(Representation value);

    Interpolation interpolation() const { return m_interpolation; }
    void setInterpolation(Interpolation value);

    Culling culling() const { return m_culling; }
    void setCulling(Culling value);

    bool edgeVisibility() const { return m_edgeVisibility; }
    void setEdgeVisibility(bool value);

    int opacity() const { return m_opacity; }
    void setOpacity(int value);

    int fun() const { return m_fun; }
    void setFun(int value);

    QColor faceColor() const { return m_faceColor;}
    void setFaceColor(const QColor& color);

    ColorMode colorMode() const { return m_colorMode;}
    void setColorMode(const ColorMode& mode);

    ColorMap colorMap() const { return m_colorMap;}
    void setColorMap(const ColorMap& map);

    ito::AutoInterval colorValueRange() const { return m_colorValueRange;}
    void setColorValueRange(const ito::AutoInterval& range);

protected:
    boost::shared_ptr<pcl::visualization::PCLVisualizer> m_visualizer;
    ito::PCLPolygonMesh m_mesh;
    Representation m_representation;
    //bool m_selected;
    int m_fun;
    Interpolation m_interpolation;
    Culling m_culling;
    bool m_edgeVisibility;
    int m_opacity;
    QColor m_faceColor;
    ColorMode m_colorMode;
    ColorMap m_colorMap;
    ito::AutoInterval m_colorValueRange;

    ito::ItomPalette m_colorMapData;

private:
    void evalColorMap(float value, float &red, float &green, float &blue, const float *limits = NULL);

};





#endif //ITEM_H