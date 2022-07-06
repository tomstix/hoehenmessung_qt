#pragma once

#include <QObject>
#include <QCanBus>
#include <QDebug>
#include <QThread>
#include <QDataStream>

class CAN : public QObject
{
    Q_OBJECT
    Q_PROPERTY(QStringList plugins MEMBER m_plugins NOTIFY pluginsChanged)
    Q_PROPERTY(QString pluginName MEMBER m_pluginName NOTIFY pluginNameChanged)
    Q_PROPERTY(QVariantList deviceList MEMBER m_deviceList NOTIFY deviceListChanged)
    Q_PROPERTY(QVariantList baudrates MEMBER m_baudrates NOTIFY baudratesChanged)
    Q_PROPERTY(bool rateChangeable MEMBER m_rateChangeable NOTIFY rateChangeableChanged)
    Q_PROPERTY(bool connected MEMBER m_connected NOTIFY connectedChanged)

public:
    CAN();
    ~CAN();
    const QVariantList &deviceList() const;
    const QVariantList &baudrates() const;
    int baudrate() const;

public slots:
    void setPlugin(const int index);
    void setBaudrate(int index);
    void setCanDevice(int index);
    void connect();
    void stateChanged(QCanBusDevice::CanBusDeviceState state);
    void sendCANMessage(int id, QByteArray data, bool extended = true);
    void framesReceived();
signals:
    void pluginsChanged();
    void pluginNameChanged();
    void deviceListChanged();
    void baudratesChanged();
    void rateChangeableChanged();
    void connectedChanged();
    void newHeaderMessage(int id, QByteArray data);
private:
    QList<QCanBusDeviceInfo> m_devices;
    QStringList m_plugins;
    QString m_pluginName;
    QVariantList m_deviceList;
    int m_deviceIndex;
    QVariantList m_baudrates = {
        125000,
        250000,
        500000,
        1000000};
    QVariant m_baudrate = m_baudrates.at(1);
    bool m_rateChangeable;
    bool m_connected = false;

    QCanBusDevice* m_device;
    QThread* canThread = new QThread();
};
