import QtQuick 2.15
import QtQuick.Window 2.15
import QtQuick.Controls 2.15
import QtQuick.Layouts 1.15

Window {
    width: 300
    height: 200
    visible: true
    title: "CAN Settings"

    ColumnLayout {
        anchors.fill: parent
        ComboBox {
            Layout.fillWidth: true
            Layout.alignment: Qt.AlignTop
        }
        Button {
            Layout.fillWidth: true
            text: "Connect"
        }
    }

    onClosing: canSettingsLoader.source = ""
}
