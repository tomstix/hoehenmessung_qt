#pragma once

#include <iostream>

#include <QObject>
#include <QQuickImageProvider>
#include <QImage>
#include <QPixmap>
#include <QAbstractListModel>
#include <QString>

#include <librealsense2/rs.hpp>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_perpendicular_plane.h>

class RealsenseDataProvider : public QObject
{
    Q_OBJECT
public:
    int color_width() const;
    int color_height() const;
    int depth_width() const;
    int depth_height() const;

    const rs2::frameset &frameset() const;
    const rs2::motion_frame &motion_frame() const;

protected:
    int m_color_width;
    int m_color_height;
    int m_depth_width;
    int m_depth_height;
    rs2::frameset m_frameset;
    rs2::motion_frame m_motion_frame;
};

class RealsenseDataProvider2 : public QObject
{
    Q_OBJECT
public:
    RealsenseDataProvider2(const rs2::device& device = rs2::device());
    QStringList color_resolutions() const;
    QStringList depth_resolutions() const;

public slots:
    void set_color_resolution(QString res);
    void set_depth_resolution(QString res);

signals:
    void color_resolutionsChanged();
    void depth_resolutionsChanged();

private:
    rs2::device m_device;
    QStringList m_color_resolutions;
    Q_PROPERTY(QStringList color_resolutions READ color_resolutions NOTIFY color_resolutionsChanged)
    QStringList m_depth_resolutions;
    Q_PROPERTY(QStringList depth_resolutions READ depth_resolutions NOTIFY depth_resolutionsChanged)
};

class RealsenseDeviceList : public QAbstractListModel
{
    Q_OBJECT
public:
    RealsenseDeviceList(QObject *parent = nullptr);
    enum RealsenseDeviceRoles
    {
        RS2_CAMERA_INFO_NAME_ROLE = Qt::UserRole,
        RS2_CAMERA_INFO_SERIAL_NUMBER_ROLE,
        RS2_CAMERA_INFO_FIRMWARE_VERSION_ROLE,
        RS2_CAMERA_INFO_RECOMMENDED_FIRMWARE_VERSION_ROLE,
        RS2_CAMERA_INFO_PHYSICAL_PORT_ROLE,
        RS2_CAMERA_INFO_DEBUG_OP_CODE_ROLE,
        RS2_CAMERA_INFO_ADVANCED_MODE_ROLE,
        RS2_CAMERA_INFO_PRODUCT_ID_ROLE,
        RS2_CAMERA_INFO_CAMERA_LOCKED_ROLE,
        RS2_CAMERA_INFO_USB_TYPE_DESCRIPTOR_ROLE,
        RS2_CAMERA_INFO_PRODUCT_LINE_ROLE,
        RS2_CAMERA_INFO_ASIC_SERIAL_NUMBER_ROLE,
        RS2_CAMERA_INFO_FIRMWARE_UPDATE_ID_ROLE,
        RS2_CAMERA_INFO_IP_ADDRESS_ROLE,
        RS2_CAMERA_INFO_COUNT_ROLE
    };
    int rowCount(const QModelIndex& parent = QModelIndex()) const;
    QVariant data(const QModelIndex& index, int role = Qt::DisplayRole) const;

    const QUrl &bagFile_url() const;
    void setBagFile_url(const QUrl &newBagFile_url);

    const rs2::device &selected_device() const;
    void setSelected_device(const rs2::device &newSelected_device);

    bool use_bag() const;
    void setUse_bag(bool newUse_bag);

signals:
    void bagFile_urlChanged();
    void selected_deviceChanged();
    void use_bagChanged();

public slots:
    void set_device(int index);

protected:
    QHash<int, QByteArray> roleNames() const;

private:
    rs2::context m_ctx;
    QVector<rs2::device> m_devices;
    QUrl m_bagFile_url;
    Q_PROPERTY(QUrl bagFile_url READ bagFile_url WRITE setBagFile_url NOTIFY bagFile_urlChanged)
    rs2::device m_selected_device;
    Q_PROPERTY(rs2::device selected_device READ selected_device NOTIFY selected_deviceChanged)
    bool m_use_bag = false;
    Q_PROPERTY(bool use_bag READ use_bag NOTIFY use_bagChanged)
    void refresh();
};

class RealsenseManager : public QObject
{
    Q_OBJECT
};
