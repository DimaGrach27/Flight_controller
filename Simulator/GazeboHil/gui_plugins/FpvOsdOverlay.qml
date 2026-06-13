import QtQuick 2.15
import QtQuick.Controls 2.15

Item {
  id: root
  anchors.fill: parent

  Rectangle {
    anchors.fill: parent
    color: "transparent"
    border.width: 1
    border.color: "#66d7ff7d"
  }

  Rectangle {
    anchors.left: parent.left
    anchors.right: parent.right
    anchors.top: parent.top
    height: 36
    color: "#22000000"
  }

  Text {
    id: topText
    anchors.top: parent.top
    anchors.horizontalCenter: parent.horizontalCenter
    anchors.topMargin: 8
    text: "FLIGHT CONTROLLER HIL"
    color: "#d7ff7d"
    font.family: "Menlo"
    font.pixelSize: 15
    font.bold: true
    style: Text.Outline
    styleColor: "#88000000"
  }

  Text {
    id: leftOsd
    anchors.left: parent.left
    anchors.top: parent.top
    anchors.leftMargin: 24
    anchors.topMargin: 52
    width: Math.min(parent.width * 0.5, 520)
    text: FpvOsdOverlay.osdText
    color: "#d7ff7d"
    font.family: "Menlo"
    font.pixelSize: 15
    font.bold: true
    lineHeight: 1.05
    wrapMode: Text.NoWrap
    style: Text.Outline
    styleColor: "#aa000000"
  }

  Text {
    anchors.right: parent.right
    anchors.bottom: parent.bottom
    anchors.rightMargin: 26
    anchors.bottomMargin: 22
    text: "HIL LINK"
    color: "#ffcc66"
    font.family: "Menlo"
    font.pixelSize: 15
    font.bold: true
    style: Text.Outline
    styleColor: "#aa000000"
  }

  Repeater {
    model: [
      {"x": 0.12, "y": 0.50, "rot": 0},
      {"x": 0.88, "y": 0.50, "rot": 180},
      {"x": 0.50, "y": 0.18, "rot": 90},
      {"x": 0.50, "y": 0.82, "rot": 270}
    ]

    Canvas {
      width: 48
      height: 24
      x: root.width * modelData.x - width / 2
      y: root.height * modelData.y - height / 2
      rotation: modelData.rot
      opacity: 0.75

      onPaint: {
        const ctx = getContext("2d");
        ctx.clearRect(0, 0, width, height);
        ctx.strokeStyle = "#d7ff7d";
        ctx.lineWidth = 2;
        ctx.beginPath();
        ctx.moveTo(4, height / 2);
        ctx.lineTo(width - 4, height / 2);
        ctx.moveTo(4, height / 2);
        ctx.lineTo(14, 4);
        ctx.moveTo(4, height / 2);
        ctx.lineTo(14, height - 4);
        ctx.stroke();
      }
    }
  }

  Rectangle {
    anchors.centerIn: parent
    width: 68
    height: 2
    color: "#d7ff7d"
    opacity: 0.85
  }

  Rectangle {
    anchors.centerIn: parent
    width: 2
    height: 42
    color: "#d7ff7d"
    opacity: 0.85
  }
}
