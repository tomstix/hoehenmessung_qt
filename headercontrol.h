#pragma once

#include <QObject>
#include <QDebug>
#include <QDataStream>

class Headercontrol : public QObject
{
    Q_OBJECT
public:
    explicit Headercontrol(QObject *parent = nullptr);

    float height() const;
    void setHeight(float newHeight);

    int tableSetpoint() const;
    void setTableSetpoint(int newTableSetpoint);

    int tableSetpointOffset() const;
    void setTableSetpointOffset(int newTableSetpointOffset);

public slots:
    void calibrateMin();
    void calibrateMax();
    void updateHeight(float height);
    void processHeaderMessage(int id, QByteArray data);

signals:
    void sendCanMessage(int id, QByteArray data, bool extended);
    void heightChanged();
    void tableRawChanged();
    void tableCalibratedChanged();
    void tableLengthChanged();

    void tableSetpointChanged();

    void tableSetpointOffsetChanged();

private:
    float m_height;
    Q_PROPERTY(int height MEMBER m_height NOTIFY heightChanged)
    int m_tableRaw;
    Q_PROPERTY(int tableRaw MEMBER m_tableRaw NOTIFY tableRawChanged)
    int m_tableLength;
    Q_PROPERTY(int tableLength MEMBER m_tableLength NOTIFY tableLengthChanged)
    bool m_tableCalibrated = false;
    Q_PROPERTY(bool tableCalibrated MEMBER m_tableCalibrated NOTIFY tableCalibratedChanged)
    int m_tableSetpoint = 490;
    Q_PROPERTY(int tableSetpoint READ tableSetpoint NOTIFY tableSetpointChanged)
    int m_tableSetpointOffset = 0;
    Q_PROPERTY(int tableSetpointOffset READ tableSetpointOffset WRITE setTableSetpointOffset NOTIFY tableSetpointOffsetChanged)
};
