#include "headercontrol.h"

Headercontrol::Headercontrol(QObject *parent)
    : QObject{parent}
{

}

void Headercontrol::calibrateMin()
{
    QByteArray data;
    data.resize(2);
    data[0] = 0x01;
    data[1] = 0x00;
    emit sendCanMessage(0x600, data, true);
}

void Headercontrol::calibrateMax()
{
    QByteArray data;
    data.resize(2);
    data[0] = 0x01;
    data[1] = 0x01;
    emit sendCanMessage(0x600, data, true);
}

void Headercontrol::updateHeight(float height)
{
    m_height = (int)(height * 1000);
    m_tableSetpoint = m_height + m_tableSetpointOffset;
    emit heightChanged();
    emit tableSetpointChanged();
}

void Headercontrol::processHeaderMessage(int id, QByteArray data)
{
    auto ds = QDataStream(data);
    quint16 tableRaw;
    ds >> tableRaw;
    m_tableRaw = tableRaw;
    emit tableRawChanged();

    quint16 tableLength;
    ds >> tableLength;
    m_tableCalibrated = !(tableLength == 0xFFFF);
    emit tableCalibratedChanged();
    m_tableLength = tableLength;
    emit tableLengthChanged();
}

int Headercontrol::tableSetpointOffset() const
{
    return m_tableSetpointOffset;
}

void Headercontrol::setTableSetpointOffset(int newTableSetpointOffset)
{
    if (m_tableSetpointOffset == newTableSetpointOffset)
        return;
    m_tableSetpointOffset = newTableSetpointOffset;
    m_tableSetpoint = (int)m_height*100 + m_tableSetpointOffset;
    emit tableSetpointChanged();
    emit tableSetpointOffsetChanged();
}

int Headercontrol::tableSetpoint() const
{
    return m_tableSetpoint;
}
