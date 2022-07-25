#include "datavisualizer.h"

DataVisualizer::DataVisualizer(QObject *parent)
    : QObject{parent}
{

}

void DataVisualizer::receiveData(pcl::PointCloud<pcl::PointXYZ>::Ptr pointcloud, std::vector<int> inliers)
{
    using namespace QtDataVisualization;

    qDebug() << "New Data for Visualizer with " << inliers.size() << " points.";

    /*auto proxy = series->dataProxy();

    for (auto point : pointcloud->points)
    {
        QScatterDataItem item;
        item.setX(point.x);
        item.setY(point.y);
        item.setZ(point.z);
        proxy->addItem(item);
    }
    emit dataReady();*/
}
