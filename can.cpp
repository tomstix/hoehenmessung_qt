#include "can.h"

CAN::CAN(QObject *parent)
    : QObject{parent}
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

const QVariantList &CAN::deviceList() const
{
    return m_deviceList;
}

void CAN::setCanDevice(int deviceIndex)
{
    if (!connected())
    {
        auto currentDeviceInfo = m_devices.at(deviceIndex);
        if (deviceIndex < numPeakDevices)
        {
            pluginName = "peakcan";
            emit baudrateChangeable(true);
        }
        else if (deviceIndex >= numPeakDevices)
        {
            pluginName = "socketcan";
            emit baudrateChangeable(false);
        }
        m_device = QCanBus::instance()->createDevice(pluginName, currentDeviceInfo.name());
        QObject::connect(m_device, &QCanBusDevice::stateChanged, this, &CAN::canStateChanged);
        deviceCreated = true;
        setBaudrate(m_device->configurationParameter(QCanBusDevice::BitRateKey));
        qDebug() << "CAN device set to " << pluginName << " device" << currentDeviceInfo.description();
    }
}

void CAN::setBaudrate(QVariant rate)
{
    auto i = std::find(m_baudrates.begin(), m_baudrates.end(), rate);
    auto index = i - m_baudrates.begin();
    setBaudrate(index);
}

void CAN::setBaudrate(int idx)
{
    if (deviceCreated && pluginName != "socketcan")
    {
        if (m_device->state() == QCanBusDevice::UnconnectedState)
        {
            m_device->setConfigurationParameter(QCanBusDevice::BitRateKey, m_baudrates.at(idx));
            m_baudrateIndex = idx;
            emit baudrateChanged(idx);
        }
    }
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

void CAN::connect()
{
    if (deviceCreated)
    {
        if (m_device->state() == QCanBusDevice::UnconnectedState)
        {
            m_device->connectDevice();
        }
        else
        {
            m_device->disconnectDevice();
        }
        emit baudrateChangeable(m_device->state() == QCanBusDevice::UnconnectedState);
    }
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
    if (connected())
    {
        QCanBusFrame frame;
        frame.setFrameId(id);
        frame.setExtendedFrameFormat(extended);
        frame.setPayload(data);
        m_device->writeFrame(frame);
    }
}