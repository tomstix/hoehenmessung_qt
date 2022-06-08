#pragma once

#include <QObject>
#include <QCanBus>
#include <QDebug>

class CAN : public QObject
{
    Q_OBJECT
    Q_PROPERTY(QVariantList baudrates READ baudrates NOTIFY baudrateChanged)
    Q_PROPERTY(int baudrate READ baudrate WRITE setBaudrate NOTIFY baudrateChanged)
public:
    explicit CAN(QObject *parent = nullptr);

    const QVariantList &deviceList() const;

    const QVariantList &baudrates() const;

    int baudrate() const;

public slots:
    void setCanDevice(int deviceIndex);
    void setBaudrate(QVariant rate);
    void setBaudrate(int idx);
    void connect();
    bool connected() const;
    void canStateChanged(QCanBusDevice::CanBusDeviceState state);
    void sendCANMessage(int id, QByteArray data, bool extended = true);

signals:
    void deviceListChanged();
    void baudrateChangeable(bool b);
    void baudrateChanged(int i);
    void deviceConnected(bool success);

private:
    QCanBusDevice *m_device;
    bool deviceCreated = false;
    QList<QCanBusDeviceInfo> m_devices;
    QString pluginName;
    int numPeakDevices = 0;
    int numSocketDevices = 0;
    QVariantList m_deviceList;
    Q_PROPERTY(QVariantList deviceList READ deviceList NOTIFY deviceListChanged)

    QVariantList m_baudrates = {
        125000,
        250000,
        500000,
        1000000};
    int m_baudrateIndex = 1;
};
