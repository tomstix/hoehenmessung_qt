#include "rs.hpp"

#include <string>

int RealsenseDataProvider::color_width() const
{
    return m_color_width;
}

int RealsenseDataProvider::color_height() const
{
    return m_color_height;
}

int RealsenseDataProvider::depth_width() const
{
    return m_depth_width;
}

int RealsenseDataProvider::depth_height() const
{
    return m_depth_height;
}

const rs2::frameset &RealsenseDataProvider::frameset() const
{
    return m_frameset;
}

void RealsenseDataProvider::setMotion_frame(const rs2::motion_frame &newMotion_frame)
{
    m_motion_frame = newMotion_frame;
}

RealsenseDeviceList::RealsenseDeviceList(QObject *parent) : QAbstractListModel(parent)
{
    try {
        auto devices = m_ctx.query_devices();
        for (const auto& dev : devices)
        {
            m_devices.append(dev);
        }
        auto devices_changed_callback = [&](rs2::event_information& info)
        {
            beginResetModel();
            m_devices.clear();
            auto devices = m_ctx.query_devices();
            endResetModel();
        };
        m_ctx.set_devices_changed_callback(devices_changed_callback);
    }
    catch (const rs2::error &e)
    {
        std::cerr << "Realsense error calling " << e.get_failed_function() << "(" << e.get_failed_args() << "):\n   " << e.what() << std::endl;
    }
}

int RealsenseDeviceList::rowCount(const QModelIndex &parent) const
{
    Q_UNUSED(parent)
    return m_devices.size() + m_bagFile_url.isValid();
}

QVariant RealsenseDeviceList::data(const QModelIndex &index, int role) const
{
    if (index.row() < 0 || index.row() > m_devices.size())
        return QVariant();
    if (index.row() == m_devices.size())
    {
        std::stringstream ss;
        ss << "File: " << m_bagFile_url.toDisplayString(QUrl::RemoveScheme).toStdString();
        return QString::fromStdString(ss.str());
    }
    auto device = m_devices[index.row()];
    auto info = rs2_camera_info(index.row());
    if (device.supports(info))
    {
        return device.get_info(info);
    }
    return QVariant();
}

QHash<int, QByteArray> RealsenseDeviceList::roleNames() const
{
    QHash<int, QByteArray> roles;
    roles[RS2_CAMERA_INFO_NAME_ROLE] = "NAME";
    roles[RS2_CAMERA_INFO_SERIAL_NUMBER_ROLE] = "SERIAL_NUMBER";
    roles[RS2_CAMERA_INFO_FIRMWARE_VERSION_ROLE] = "FIRMWARE_VERSION";
    roles[RS2_CAMERA_INFO_RECOMMENDED_FIRMWARE_VERSION_ROLE] = "RECOMMENDED_FIRMWARE_VERSION";
    roles[RS2_CAMERA_INFO_PHYSICAL_PORT_ROLE] = "PHYSICAL_PORT";
    roles[RS2_CAMERA_INFO_DEBUG_OP_CODE_ROLE] = "DEBUG_OP_CODE";
    roles[RS2_CAMERA_INFO_ADVANCED_MODE_ROLE] = "ADVANCED_MODE";
    roles[RS2_CAMERA_INFO_PRODUCT_ID_ROLE] = "PRODUCT_ID";
    roles[RS2_CAMERA_INFO_CAMERA_LOCKED_ROLE] = "CAMERA_LOCKED";
    roles[RS2_CAMERA_INFO_USB_TYPE_DESCRIPTOR_ROLE] = "USB_TYPE_DESCRIPTOR";
    roles[RS2_CAMERA_INFO_PRODUCT_LINE_ROLE] = "PRODUCT_LINE";
    roles[RS2_CAMERA_INFO_ASIC_SERIAL_NUMBER_ROLE] = "ASIC_SERIAL_NUMBER";
    roles[RS2_CAMERA_INFO_FIRMWARE_UPDATE_ID_ROLE] = "FIRWMARE_UPDATE_ID";
    roles[RS2_CAMERA_INFO_IP_ADDRESS_ROLE] = "IP_ADDRESS";
    roles[RS2_CAMERA_INFO_COUNT_ROLE] = "COUNT";
    return roles;
}

bool RealsenseDeviceList::use_bag() const
{
    return m_use_bag;
}

RealsenseDataProvider2* RealsenseDeviceList::realsenseDataProvider() const
{
    return m_realsenseDataProvider;
}

void RealsenseDeviceList::set_device(int index)
{
    if (index == m_devices.size() && m_bagFile_url.isValid())
    {
        m_selected_device = rs2::device();
        m_use_bag = true;
        emit use_bagChanged();
    }
    else if (m_devices.size() > 0)
    {
        m_selected_device = m_devices[index];
        m_use_bag = false;
        emit use_bagChanged();

    }
}

const rs2::device &RealsenseDeviceList::selected_device() const
{
    return m_selected_device;
}

const QUrl &RealsenseDeviceList::bagFile_url() const
{
    return m_bagFile_url;
}

void RealsenseDeviceList::setBagFile_url(const QUrl &newBagFile_url)
{
    if (m_bagFile_url == newBagFile_url)
        return;
    beginInsertRows(QModelIndex(), m_devices.size(), m_devices.size());
    m_bagFile_url = newBagFile_url;
    endInsertRows();
    emit bagFile_urlChanged();
}

RealsenseDataProvider2::RealsenseDataProvider2(const rs2::device &device)
                : m_device(device)
{
    m_color_resolutions.clear();
    m_depth_resolutions.clear();
    for (auto& sensor : m_device.query_sensors())
    {
        for (auto& profile : sensor.get_stream_profiles())
        {
            if (auto video = profile.as<rs2::video_stream_profile>())
            {
                if (video.fps() == 30)
                {
                    if ((strcmp(sensor.get_info(RS2_CAMERA_INFO_NAME), "RGB Camera") == 0) && video.format() == RS2_FORMAT_RGB8)
                    {
                        std::stringstream ss;
                        ss << video.width() << "x" << video.height();
                        m_color_resolutions << QString::fromStdString(ss.str());
                    }
                    else if ((strcmp(sensor.get_info(RS2_CAMERA_INFO_NAME), "Stereo Module") == 0) && video.format() == RS2_FORMAT_Z16)
                    {
                        std::stringstream ss;
                        ss << video.width() << "x" << video.height();
                        m_depth_resolutions << QString::fromStdString(ss.str());
                    }
                }
            }
        }
    }
}

QStringList RealsenseDataProvider2::color_resolutions() const
{
    return m_color_resolutions;
}

QStringList RealsenseDataProvider2::depth_resolutions() const
{
    return m_depth_resolutions;
}

void RealsenseDataProvider2::set_color_resolution(QString res)
{
    qDebug() << "Setting color resolution to" << res;
}

void RealsenseDataProvider2::set_depth_resolution(QString res)
{
    qDebug() << "Setting depth resolution to" << res;
}
