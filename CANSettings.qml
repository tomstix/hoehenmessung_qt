import QtQuick 2.12
import QtQuick.Window 2.12
import QtQuick.Controls 2.12
import QtQuick.Layouts 1.12

Window {
    width: 300
    height: 200
    visible: true
    title: "CAN Settings"

    /*property bool connected: CAN.connected()

    Connections {
        target: CAN
        function onDeviceConnected(success) {
            connected = success
        }
    }*/

    ColumnLayout {
        anchors.fill: parent
        ComboBox {
            id: pluginSelector
            Layout.fillWidth: true
            Layout.alignment: Qt.AlignTop
            model: CAN.plugins
            enabled: !CAN.connected
            onCurrentIndexChanged: CAN.setPlugin(currentIndex)
        }
        ComboBox {
            Layout.fillWidth: true
            Layout.alignment: Qt.AlignTop
            model: CAN.deviceList
            enabled: !CAN.connected
            onCurrentIndexChanged: CAN.setCanDevice(currentIndex)
        }
        ComboBox {
            id: baudrateSelector
            Layout.fillWidth: true
            Layout.alignment: Qt.AlignTop
            model: CAN.baudrates
            currentIndex: 1
            onCurrentIndexChanged: CAN.setBaudrate(currentIndex)
            enabled: CAN.rateChangeable && !CAN.connected
        }
        Button {
            id: connectButton
            Layout.fillWidth: true
            text: CAN.connected ? "Disconnect" : "Connect"
            onPressed: CAN.connect()
        }
    }

    onClosing: canSettingsLoader.source = ""
}
