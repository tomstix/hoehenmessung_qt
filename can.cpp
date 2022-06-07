#include "can.h"

CAN::CAN(QObject *parent)
    : QObject{parent}
{
    qDebug() << "Starting CAN Bus";
    if (!QCanBus::instance()->plugins().contains(QStringLiteral("socketcan")))
    {
        qDebug() << "Socketcan not available!";
    }
    QString errorString;
    const auto devices = QCanBus::instance()->availableDevices(
        QStringLiteral("socketcan"), &errorString);
    if (!errorString.isEmpty())
        qDebug() << errorString;

    qDebug() << "Available devices: ";
    foreach(auto &dev, devices)
    {
        qDebug() << dev.name();
    }
}
