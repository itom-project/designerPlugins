#ifndef ITEMPOINTCLOUDNORMAL_H
#define ITEMPOINTCLOUDNORMAL_H

#include "itemPointCloud.h"



class ItemPointCloudNormal : public ItemPointCloud
{
    Q_OBJECT

    //Q_PROPERTY(bool selected READ selected WRITE setSelected DESIGNABLE true USER true);
    /*Q_PROPERTY(int PointSize READ pointSize WRITE setPointSize DESIGNABLE true USER true);
    Q_PROPERTY(int LineWidth READ lineWidth WRITE setLineWidth DESIGNABLE true USER true);
    Q_PROPERTY(QColor Color READ color WRITE setColor DESIGNABLE true USER true);*/
    Q_PROPERTY(int Level READ level WRITE setLevel DESIGNABLE true USER true);
    Q_PROPERTY(float Scale READ scale WRITE setScale DESIGNABLE true USER true);

public:
    ItemPointCloudNormal(boost::shared_ptr<pcl::visualization::PCLVisualizer> visualizer, const QString &name, QTreeWidgetItem *treeItem);
    virtual ~ItemPointCloudNormal();

    ito::RetVal addPointCloud(const ito::PCLPointCloud &cloud);

    //properties
    //virtual void setVisible(bool value);

    ////bool selected() const { return m_selected; }
    ////void setSelected(bool value);

    //int pointSize() const { return m_pointSize; }
    //void setPointSize(int value);

    //int lineWidth() const { return m_lineWidth; }
    //void setLineWidth(int value);

    //QColor color() const { return m_color; }
    //void setColor(QColor value);



    int level() const { return m_level; }
    void setLevel(int value);

    float scale() const { return m_scale; }
    void setScale(float value);

protected:

    void updatePointCloudNormal();

    int m_level;
    float m_scale;

};





#endif //ITEMPOINTCLOUDNORMAL_H