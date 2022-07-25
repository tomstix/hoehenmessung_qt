#include "can.h"

CAN::CAN()
{
    auto plugins = QCanBus::instance()->plugins();
    for(auto plugin : plugins)
    {
        auto devices = QCanBus::instance()->availableDevices(plugin);
        if(devices.isEmpty())
        {
            qDebug() << "No devices found for " << plugin;
        }
        else
        {
            m_plugins.append(plugin);
        }
    }
}
CAN::~CAN()
{
}
void CAN::setPlugin(const int index)
{
    m_pluginName = m_plugins.at(index);
    m_devices = QCanBus::instance()->availableDevices(m_pluginName);
    m_deviceList.clear();
    foreach (auto &dev, m_devices)
    {
        m_deviceList.append(dev.description() + " (" + dev.name() + ")");
    }
    emit deviceListChanged();
    if (m_pluginName == "socketcan")
    {
        m_rateChangeable = false;
    }
    else m_rateChangeable = true;
    emit rateChangeableChanged();
}
void CAN::setBaudrate(int index)
{
    m_baudrate = m_baudrates.at(index);
    qDebug() << "setBaudrate: " << m_baudrate;
}
void CAN::setCanDevice(int index)
{
    m_deviceIndex = index;
}
void CAN::connect()
{
    if (!m_connected)
    {
        auto deviceInfo = m_devices.at(m_deviceIndex);
        QCanBus::instance()->moveToThread(canThread);
        m_device = QCanBus::instance()->createDevice(m_pluginName, deviceInfo.name());
        m_device->setConfigurationParameter(QCanBusDevice::BitRateKey, m_baudrate);
        QObject::connect(m_device, &QCanBusDevice::stateChanged, this, &CAN::stateChanged);
        QObject::connect(m_device, &QCanBusDevice::framesReceived, this, &CAN::framesReceived);
        m_device->connectDevice();
    }
    else
    {
        m_device->disconnectDevice();
        delete m_device;
    }
    emit connectedChanged();
}
void CAN::stateChanged(QCanBusDevice::CanBusDeviceState state)
{
    m_connected = (state == QCanBusDevice::ConnectedState);
    if(m_connected)
    {
        sendCANMessage(0x100, QByteArray("Hello", 5));
    }
    qDebug() << "Can Bus state changed to: " << state;
    emit connectedChanged();
}
void CAN::sendCANMessage(int id, QByteArray data, bool extended)
{
    if (!m_connected) return;
    QCanBusFrame frame(id, data);
    frame.setExtendedFrameFormat(extended);
    m_device->writeFrame(frame);
}
void CAN::framesReceived()
{
    auto frames = m_device->readAllFrames();
    for(auto &frame : frames)
    {
        if(frame.frameId() == 0x200)
        {
            emit newHeaderMessage(frame.frameId(), frame.payload());
        }
    }
}
