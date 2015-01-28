#ifndef ITEMPOINTCLOUD_H
#define ITEMPOINTCLOUD_H

#include "common/sharedStructures.h"
#include "PointCloud/pclStructures.h"
#include "pcl/visualization/pcl_visualizer.h"

#include "item.h"

#include <qcolor.h>


class ItemPointCloud : public Item
{
    Q_OBJECT

    //Q_PROPERTY(bool selected READ selected WRITE setSelected DESIGNABLE true USER true);
    Q_PROPERTY(int PointSize READ pointSize WRITE setPointSize DESIGNABLE true USER true);
    Q_PROPERTY(int LineWidth READ lineWidth WRITE setLineWidth DESIGNABLE true USER true);
    Q_PROPERTY(QColor Color READ color WRITE setColor DESIGNABLE true USER true);

public:
    ItemPointCloud(boost::shared_ptr<pcl::visualization::PCLVisualizer> visualizer, const QString &name, QTreeWidgetItem *treeItem);
    virtual ~ItemPointCloud();

    ito::RetVal addPointCloud(const ito::PCLPointCloud &cloud);

    ito::RetVal updatePointCloud(const ito::PCLPointCloud &cloud);

    //properties
    virtual void setVisible(bool value);

    //bool selected() const { return m_selected; }
    //void setSelected(bool value);

    int pointSize() const { return m_pointSize; }
    void setPointSize(int value);

    int lineWidth() const { return m_lineWidth; }
    void setLineWidth(int value);

    QColor color() const { return m_color; }
    void setColor(QColor value);



protected:
    boost::shared_ptr<pcl::visualization::PCLVisualizer> m_visualizer;
    ito::PCLPointCloud m_cloud;

    pcl::visualization::PCLVisualizer::ColorHandlerPtr m_colorHandler;
    pcl::visualization::PCLVisualizer::GeometryHandlerPtr m_geometryHandler;
    //bool m_selected;

    int m_pointSize;
    int m_lineWidth;
    QColor m_color;

};





#endif //ITEM_H