#include "datavisualizer.h"

DataVisualizer::DataVisualizer(QObject *parent)
    : QObject{parent}
{

}

void DataVisualizer::receiveData(pcl::PointCloud<pcl::PointXYZ>::Ptr pointcloud)
{
    using namespace QtDataVisualization;

    auto proxy = series->dataProxy();

    for (auto point : pointcloud->points)
    {
        QScatterDataItem item;
        item.setX(point.x);
        item.setY(point.y);
        item.setZ(point.z);
        proxy->addItem(item);
    }
    emit dataReady();
}
