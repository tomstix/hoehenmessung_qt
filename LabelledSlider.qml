import QtQuick 2.0
import QtQuick.Controls 2.15
import QtQuick.Layouts 1.15

GridLayout {

    property string labeltext: "Text"
    property string unitSuffix
    property real maxValue
    property real minValue
    property real step
    property var initialValue

    signal sliderValueChanged(real value)

    Layout.fillWidth: true
    Layout.columnSpan: 2
    rows: 2
    columns: 2
    Slider {
        id: slider
        Layout.rowSpan: 2
        width: parent.width * 0.7
        value: initialValue
        from: minValue
        to: maxValue
        stepSize: step
        onValueChanged: sliderValueChanged(value)
    }
    Label {
        id: textLabel
        Layout.fillWidth: true
        text: labeltext
        horizontalAlignment: Text.AlignRight
    }
    Label {
        id: valueLabel
        Layout.fillWidth: true
        text: slider.value.toLocaleString() + unitSuffix
        horizontalAlignment: Text.AlignRight
    }
}
