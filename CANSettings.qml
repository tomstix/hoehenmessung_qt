import QtQuick 2.15
import QtQuick.Window 2.15
import QtQuick.Controls 2.15
import QtQuick.Layouts 1.15

Window {
    width: 300
    height: 200
    visible: true
    title: "CAN Settings"

    property bool connected: CAN.connected()

    Connections {
        target: CAN
        function onDeviceConnected(success) {
            connected = success
        }
    }

    ColumnLayout {
        anchors.fill: parent
        ComboBox {
            Layout.fillWidth: true
            Layout.alignment: Qt.AlignTop
            model: CAN.deviceList
            enabled: !connected
            onCurrentIndexChanged: CAN.setCanDevice(currentIndex)
        }
        ComboBox {
            id: baudrateSelector
            Layout.fillWidth: true
            Layout.alignment: Qt.AlignTop
            model: CAN.baudrates
            currentIndex: 1
            onCurrentIndexChanged: CAN.baudrate = currentIndex
            enabled: !connected
            Connections {
                target: CAN
                function onBaudrateChangeable(b)
                {
                    baudrateSelector.enabled = b
                }
                function onBaudrateChanged(i)
                {
                    baudrateSelector.currentIndex = i
                }
            }
        }
        Button {
            id: connectButton
            Layout.fillWidth: true
            text: connected ? "Disconnect" : "Connect"
            onPressed: CAN.connectCAN()
        }
    }

    onClosing: canSettingsLoader.source = ""
}
