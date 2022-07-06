#include "can.h"

CAN::CAN()
{
    
    qDebug() << "Starting CAN Bus";
    if (!QCanBus::instance()->plugins().contains(QStringLiteral("peakcan")))
    {
        qDebug() << "PEAK CAN driver not available!";
    }
    if (!QCanBus::instance()->plugins().contains(QStringLiteral("socketcan")))
    {
        qDebug() << "Socketcan driver not available!";
    }
    QString errorString;
    m_devices = QCanBus::instance()->availableDevices(
        QStringLiteral("peakcan"), &errorString);
    numPeakDevices = m_devices.size();
    m_devices.append(QCanBus::instance()->availableDevices(
        QStringLiteral("socketcan"), &errorString));
    numSocketDevices = m_devices.size() - numPeakDevices;
    if (!errorString.isEmpty())
        qDebug() << errorString;

    foreach (auto &dev, m_devices)
    {
        m_deviceList.append(dev.description() + " (" + dev.name() + ")");
    }
    emit deviceListChanged();
}
CAN::~CAN(){};

const QVariantList &CAN::deviceList() const
{
    return m_deviceList;
}
void CAN::setBaudrate(int idx)
{
    if (pluginName != "socketcan")
    {
        m_baudrateIndex = idx;
        m_baudrate = m_baudrates.at(idx).toInt();
        emit baudrateChanged(idx);
    }
}
void CAN::setCanDevice(int deviceIndex)
{
    m_deviceIndex = deviceIndex;
    if (m_deviceIndex < numPeakDevices)
    {
        pluginName = "peakcan";
        emit baudrateChangeable(true);
    }
    else if (m_deviceIndex >= numPeakDevices)
    {
        pluginName = "socketcan";
        emit baudrateChangeable(false);
    }
    m_device = m_devices.at(m_deviceIndex);
}
int CAN::baudrate() const
{
    return m_baudrateIndex;
}
void CAN::canStateChanged(QCanBusDevice::CanBusDeviceState state)
{
    qDebug() << "CAN state changed to: " << state;
    emit deviceConnected(state == QCanBusDevice::ConnectedState);
}
void CAN::connectCAN()
{
    canWorker = new CanWorker(m_deviceIndex, m_baudrate, pluginName);
    canWorker->moveToThread(canThread);
    QObject::connect(canThread, &QThread::started, canWorker, &CanWorker::startWorker);
    QObject::connect(canWorker, &CanWorker::finished, canThread, &QThread::quit);
    QObject::connect(canWorker, &CanWorker::finished, canWorker, &CanWorker::deleteLater);
    QObject::connect(canThread, &QThread::finished, canThread, &QThread::deleteLater);
    QObject::connect(this, &CAN::sendCAN, canWorker, &CanWorker::sendCANMessage);
    canThread->start();
}
bool CAN::connected() const
{
    if (deviceCreated)
    {
        auto state = m_device->state();
        return (state == QCanBusDevice::ConnectedState);
    }
    return false;
}
const QVariantList &CAN::baudrates() const
{
    return m_baudrates;
}
void CAN::sendCANMessage(int id, QByteArray data, bool extended)
{
    qDebug() << "Trying to forwared CAN message " << id << " to CAN Thread";
    emit sendCAN(id, data, extended);
}
void CAN::sendTableSetpoint(bool active, float setpoint)
{
}

CanWorker::CanWorker(int index, int baudrate, QString plugin)
{
    auto devices = QCanBus::instance()->availableDevices();
    auto currentDeviceInfo = devices.at(index);
    m_device = QCanBus::instance()->createDevice(plugin, currentDeviceInfo.name());
    m_device->setConfigurationParameter(QCanBusDevice::BitRateKey, baudrate);
    // QObject::connect(m_device, &QCanBusDevice::stateChanged, this, &CAN::canStateChanged);
    qDebug() << "CAN device set to " << plugin << " device" << currentDeviceInfo.description();
}
CanWorker::~CanWorker()
{
}
void CanWorker::startWorker()
{
    sendCANMessage(0x100, QByteArray(8, 0xF1), true);
    process();
    emit finished();
}
void CanWorker::exit()
{
    m_exitFlag = true;
}
void CanWorker::process()
{
    qDebug() << "CAN Processing Loop Started!";
    while (1)
    {
    }
}
void CanWorker::sendCANMessage(int id, QByteArray data, bool extended)
{
    qDebug() << "Sending CAN message..." << id << " " << data;
    QCanBusFrame frame;
    frame.setFrameId(id);
    frame.setExtendedFrameFormat(extended);
    frame.setPayload(data);
    m_device->writeFrame(frame);
}