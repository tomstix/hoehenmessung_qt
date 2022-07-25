#pragma once

#include <QObject>
#include <QDebug>
#include <QtDataVisualization>
#include <QScatterDataProxy>
#include <QAbstractItemModel>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

class DataVisualizer : public QObject
{
    Q_OBJECT
public:
    explicit DataVisualizer(QObject *parent = nullptr);

public slots:
    void receiveData(pcl::PointCloud<pcl::PointXYZ>::Ptr pointcloud, std::vector<int> inliers);

signals:
    void dataReady();

private:
    QtDataVisualization::QScatter3DSeries *series = new QtDataVisualization::QScatter3DSeries;

};

class PointCloudModel : public QAbstractListModel
{
    Q_OBJECT
    
};
