#pragma once

#include <QObject>
#include <QCanBus>
#include <QDebug>

class CAN : public QObject
{
    Q_OBJECT
public:
    explicit CAN(QObject *parent = nullptr);

    QVariantList deviceList;

signals:

private:
};