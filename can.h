#pragma once

#include <QObject>
#include <QCanBus>
#include <QDebug>
#include <QThread>

class CanWorker : public QObject
{
    Q_OBJECT

    public:
        CanWorker(int index, int baudrate, QString plugin);
        ~CanWorker();
    public slots:
        void startWorker();
        void process();
        void exit();
        void sendCANMessage(int id, QByteArray data, bool extended);
    signals:
        void finished();
        void error(QString err);
        void newCanData();
    private:
        bool m_exitFlag = false;
        bool connected = false;
        QCanBusDevice* m_device;
};

class CAN : public QObject
{
    Q_OBJECT
    Q_PROPERTY(QVariantList baudrates READ baudrates NOTIFY baudrateChanged)
    Q_PROPERTY(int baudrate READ baudrate WRITE setBaudrate NOTIFY baudrateChanged)
    Q_PROPERTY(QVariantList deviceList READ deviceList NOTIFY deviceListChanged)

public:
    CAN();
    ~CAN();
    const QVariantList &deviceList() const;
    const QVariantList &baudrates() const;
    int baudrate() const;

public slots:
    void setCanDevice(int deviceIndex);
    void setBaudrate(QVariant rate);
    void setBaudrate(int idx);
    void connectCAN();
    bool connected() const;
    void canStateChanged(QCanBusDevice::CanBusDeviceState state);
    void sendCANMessage(int id, QByteArray data, bool extended = true);
    void sendTableSetpoint(bool active, float setpoint);

signals:
    void deviceListChanged();
    void baudrateChangeable(bool b);
    void baudrateChanged(int i);
    void deviceConnected(bool success);
    void finished();
    void sendCAN(int id, QByteArray data, bool extended = true);

private:
    QCanBusDevice *m_device;
    bool deviceCreated = false;
    QList<QCanBusDeviceInfo> m_devices;
    QString pluginName;
    int numPeakDevices = 0;
    int numSocketDevices = 0;
    QVariantList m_deviceList;
    int m_deviceIndex;

    QThread *canThread = new QThread();
    CanWorker *canWorker;

    bool m_tableCalibrated = false;
    uint16_t m_tableSensorRaw;
    float m_tableLength;

    QVariantList m_baudrates = {
        125000,
        250000,
        500000,
        1000000};
    int m_baudrateIndex = 1;
    int m_baudrate;
};