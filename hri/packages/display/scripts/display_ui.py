#!/usr/bin/env python3
"""
Display UI - PyQt5 replacement for the Next.js HRI display.
Shows speech/interaction messages, audio state, camera feed, map and
question modals. Subscribes directly to ROS2 topics (no rosbridge /
web_video_server) to cut CPU usage vs the browser-based version.
"""

import json
import sys
import threading
from datetime import datetime

import rclpy
from rclpy.node import Node
from std_msgs.msg import Empty, Float32, String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

from PyQt5.QtCore import Qt, QObject, QTimer, pyqtSignal
from PyQt5.QtGui import QColor, QFont, QImage, QPainter, QPixmap
from PyQt5.QtWidgets import (
    QApplication,
    QDialog,
    QHBoxLayout,
    QLabel,
    QMainWindow,
    QPushButton,
    QScrollArea,
    QTextEdit,
    QVBoxLayout,
    QWidget,
)

DEFAULT_VIDEO_TOPIC = "/zed/zed_node/rgb/image_rect_color"

# Colors lifted from the original Next.js app's globals.css (--bg-dark, --blue, etc).
BG_DARK = "#0a0a0a"
BG_DARKER = "#1a1a1a"
TEXT_LIGHT = "#fafafa"
TEXT_GRAY = "#b4b4b4"
BORDER_LIGHT = "#333333"
BLUE = "#3b6fe0"
BLUE_HOVER = "#5a8aea"
ORANGE = "#e8895a"
PURPLE = "#9d5fd6"

MESSAGE_COLORS = {
    "heard": BLUE,
    "spoken": PURPLE,
    "text_spoken": PURPLE,
    "keyword": ORANGE,
    "answer": "#4caf50",
    "user_message": "#4caf50",
}

STYLESHEET = f"""
QMainWindow, QDialog, QWidget {{
    background-color: {BG_DARK};
    color: {TEXT_LIGHT};
}}
QScrollArea, QTextEdit {{
    background-color: {BG_DARK};
    border: 1px solid {BORDER_LIGHT};
    color: {TEXT_LIGHT};
}}
QLabel {{
    color: {TEXT_LIGHT};
}}
QPushButton {{
    background-color: {BLUE};
    color: white;
    border: none;
    border-radius: 8px;
    font-weight: bold;
    padding: 8px;
}}
QPushButton:hover {{
    background-color: {BLUE_HOVER};
}}
QPushButton:disabled {{
    background-color: {TEXT_GRAY};
    color: {BG_DARKER};
}}
"""

MESSAGE_ICONS = {
    "heard": "\U0001f3a4",  # mic
    "spoken": "\U0001f50a",  # speaker
    "text_spoken": "\U0001f50a",
    "keyword": "\U0001f3f7",  # tag
    "answer": "\U0001f4ac",  # speech bubble
    "user_message": "\U0001f4ac",
}

AUDIO_STATE_TIMEOUT_MS = 10_000


class DisplaySignals(QObject):
    """Bridge between ROS callbacks (spin thread) and Qt widgets (main thread)."""

    audio_state_changed = pyqtSignal(str)
    vad_level_changed = pyqtSignal(float)
    message_received = pyqtSignal(str, str)  # type, content
    question_received = pyqtSignal(str)  # empty string closes the modal
    task_status_changed = pyqtSignal(bool)
    map_received = pyqtSignal(dict)
    video_topic_changed = pyqtSignal(str)
    frame_received = pyqtSignal(QImage)


class DisplayRosNode(Node):
    def __init__(self, signals: DisplaySignals):
        super().__init__("display_ui")
        self.signals = signals
        self.bridge = CvBridge()
        self.video_topic = self.declare_parameter(
            "default_video_topic", DEFAULT_VIDEO_TOPIC
        ).value
        self._video_sub = None

        self.create_subscription(String, "/AudioState", self._on_audio_state, 10)
        self.create_subscription(Float32, "/hri/speech/vad", self._on_vad, 10)
        self.create_subscription(
            String, "/speech/raw_command", lambda m: self._on_message("heard", m), 10
        )
        self.create_subscription(
            String, "/speech/text_spoken", lambda m: self._on_message("spoken", m), 10
        )
        self.create_subscription(String, "/hri/speech/kws", self._on_kws, 10)
        self.create_subscription(
            String, "/hri/display/answers", lambda m: self._on_message("answer", m), 10
        )
        self.create_subscription(
            String, "/hri/display/frida_questions", self._on_question, 10
        )
        self.create_subscription(
            String, "/hri/display/task_status", self._on_task_status, 10
        )
        self.create_subscription(String, "/hri/display/map", self._on_map, 10)
        self.create_subscription(
            String, "/hri/display/change_video", self._on_change_video, 10
        )

        self.button_pub = self.create_publisher(Empty, "/hri/display/button_press", 10)
        self.answer_pub = self.create_publisher(String, "/hri/display/answers", 10)

        self._subscribe_video(self.video_topic)

    def _subscribe_video(self, topic: str):
        if self._video_sub is not None:
            self.destroy_subscription(self._video_sub)
        self._video_sub = self.create_subscription(Image, topic, self._on_frame, 1)
        self.video_topic = topic
        self.signals.video_topic_changed.emit(topic)

    def _on_audio_state(self, msg: String):
        self.signals.audio_state_changed.emit(msg.data)

    def _on_vad(self, msg: Float32):
        self.signals.vad_level_changed.emit(msg.data)

    def _on_message(self, msg_type: str, msg: String):
        self.signals.message_received.emit(msg_type, msg.data)

    def _on_kws(self, msg: String):
        try:
            data = json.loads(msg.data)
            keyword = data.get("keyword")
            if keyword:
                self.signals.message_received.emit("keyword", str(keyword))
        except (json.JSONDecodeError, AttributeError) as e:
            self.get_logger().warn(f"Error parsing KWS message: {e}")

    def _on_question(self, msg: String):
        self.signals.question_received.emit(msg.data)

    def _on_task_status(self, msg: String):
        self.signals.task_status_changed.emit(msg.data == "active")

    def _on_map(self, msg: String):
        try:
            data = json.loads(msg.data)
            if data.get("image_path", "").strip():
                self.signals.map_received.emit(data)
        except json.JSONDecodeError as e:
            self.get_logger().warn(f"Error parsing map message: {e}")

    def _on_change_video(self, msg: String):
        self._subscribe_video(msg.data)

    def _on_frame(self, msg: Image):
        try:
            cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding="rgb8")
        except Exception as e:
            self.get_logger().warn(f"Error converting frame: {e}")
            return
        h, w, ch = cv_img.shape
        qimg = QImage(cv_img.data, w, h, ch * w, QImage.Format_RGB888).copy()
        self.signals.frame_received.emit(qimg)

    def publish_button_press(self):
        self.button_pub.publish(Empty())

    def publish_answer(self, text: str):
        self.answer_pub.publish(String(data=text))


class MapDialog(QDialog):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setWindowTitle("Map View")
        self.resize(700, 700)
        self.markers = []
        self.image_label = QLabel()
        self.image_label.setAlignment(Qt.AlignCenter)

        layout = QVBoxLayout(self)
        layout.addWidget(self.image_label)

    def show_map(self, data: dict):
        self.markers = data.get("markers", [])
        pixmap = QPixmap(data["image_path"])
        if not pixmap.isNull():
            painted = QPixmap(pixmap)
            painter = QPainter(painted)
            for marker in self.markers:
                x = marker["x"] / 100.0 * pixmap.width()
                y = marker["y"] / 100.0 * pixmap.height()
                painter.setPen(QColor("white"))
                painter.setBrush(QColor(marker.get("color", "#ffffff")))
                painter.drawEllipse(int(x) - 8, int(y) - 8, 16, 16)
            painter.end()
            self.image_label.setPixmap(painted)
        self.show()
        self.raise_()


class QuestionDialog(QDialog):
    answered = pyqtSignal(str)

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setWindowTitle("Question")
        self.resize(500, 300)

        layout = QVBoxLayout(self)
        self.question_label = QLabel()
        self.question_label.setWordWrap(True)
        self.question_label.setFont(QFont("Sans Serif", 13))
        layout.addWidget(self.question_label)

        self.answer_edit = QTextEdit()
        self.answer_edit.setPlaceholderText("Type your answer here...")
        layout.addWidget(self.answer_edit)

        send_btn = QPushButton("Send Answer (Ctrl+Enter)")
        send_btn.clicked.connect(self._send)
        layout.addWidget(send_btn)

    def _send(self):
        text = self.answer_edit.toPlainText().strip()
        if text:
            self.answered.emit(text)
            self.answer_edit.clear()
            self.accept()

    def ask(self, question: str):
        self.question_label.setText(question)
        self.answer_edit.clear()
        self.show()
        self.raise_()

    def keyPressEvent(self, event):
        if (
            event.key() in (Qt.Key_Return, Qt.Key_Enter)
            and event.modifiers() & Qt.ControlModifier
        ):
            self._send()
        else:
            super().keyPressEvent(event)


class AudioOverlay(QWidget):
    """Full-window overlay for listening/thinking/loading states."""

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setAttribute(Qt.WA_TransparentForMouseEvents)
        layout = QVBoxLayout(self)
        self.label = QLabel()
        self.label.setAlignment(Qt.AlignCenter)
        self.label.setFont(QFont("Sans Serif", 28, QFont.Bold))
        self.label.setStyleSheet(f"color: {TEXT_LIGHT}; background: rgba(0,0,0,150);")
        layout.addWidget(self.label)
        self.hide()

    def show_state(self, text: str):
        self.label.setText(text)
        self.setGeometry(self.parent().rect())
        self.show()
        self.raise_()

    def clear(self):
        self.hide()


class MessagesPanel(QScrollArea):
    def __init__(self):
        super().__init__()
        self.setWidgetResizable(True)
        self._container = QWidget()
        self._layout = QVBoxLayout(self._container)
        self._layout.setAlignment(Qt.AlignTop)
        self.setWidget(self._container)

    def add_message(self, msg_type: str, content: str):
        color = MESSAGE_COLORS.get(msg_type, "#9ca3af")
        icon = MESSAGE_ICONS.get(msg_type, "")
        timestamp = datetime.now().strftime("%H:%M:%S")

        entry = QLabel(
            f"<b style='color:{color}'>{icon} {msg_type.upper()}</b> "
            f"<span style='color:#9ca3af'>{timestamp}</span><br>{content}"
        )
        entry.setWordWrap(True)
        entry.setStyleSheet(
            f"padding:10px; border:1px solid {BORDER_LIGHT}; border-radius:8px;"
            f"margin-bottom:6px; background-color:{BG_DARKER};"
        )
        self._layout.insertWidget(0, entry)


class DisplayWindow(QMainWindow):
    def __init__(self, ros_node: DisplayRosNode):
        super().__init__()
        self.ros_node = ros_node
        self.setWindowTitle("Display UI")
        self.resize(1100, 700)

        central = QWidget()
        self.setCentralWidget(central)
        root = QHBoxLayout(central)

        self.messages_panel = MessagesPanel()
        root.addWidget(self.messages_panel, 1)

        right = QVBoxLayout()
        self.start_button = QPushButton("Start")
        self.start_button.setFixedHeight(60)
        self.start_button.clicked.connect(self._on_start_clicked)
        right.addWidget(self.start_button)

        self.video_topic_label = QLabel(f"Video feed at {ros_node.video_topic}")
        right.addWidget(self.video_topic_label)

        self.video_label = QLabel()
        self.video_label.setAlignment(Qt.AlignCenter)
        self.video_label.setMinimumSize(480, 360)
        self.video_label.setStyleSheet(
            f"background-color: {BG_DARKER}; border: 1px solid {BORDER_LIGHT}; border-radius: 8px;"
        )
        right.addWidget(self.video_label, 1)

        root.addLayout(right, 1)

        self.audio_overlay = AudioOverlay(central)
        self.map_dialog = MapDialog(self)
        self.question_dialog = QuestionDialog(self)
        self.question_dialog.answered.connect(self.ros_node.publish_answer)

        self._audio_timeout_timer = QTimer(self)
        self._audio_timeout_timer.setSingleShot(True)
        self._audio_timeout_timer.timeout.connect(lambda: self._on_audio_state("idle"))

        signals = ros_node.signals
        signals.audio_state_changed.connect(self._on_audio_state)
        signals.message_received.connect(self.messages_panel.add_message)
        signals.question_received.connect(self._on_question)
        signals.task_status_changed.connect(self._on_task_status)
        signals.map_received.connect(self.map_dialog.show_map)
        signals.video_topic_changed.connect(
            lambda topic: self.video_topic_label.setText(f"Video feed at {topic}")
        )
        signals.frame_received.connect(self._on_frame)

    def _on_start_clicked(self):
        self.ros_node.publish_button_press()

    def _on_audio_state(self, state: str):
        if state in ("thinking", "loading", "listening"):
            label = {
                "thinking": "Thinking...",
                "loading": "Loading...",
                "listening": "Listening...",
            }[state]
            self.audio_overlay.show_state(label)
            self._audio_timeout_timer.start(AUDIO_STATE_TIMEOUT_MS)
        else:
            self.audio_overlay.clear()
            self._audio_timeout_timer.stop()

    def _on_question(self, question: str):
        if question.strip():
            self.question_dialog.ask(question)
        else:
            self.question_dialog.hide()

    def _on_task_status(self, active: bool):
        self.start_button.setEnabled(not active)

    def _on_frame(self, qimg: QImage):
        pixmap = QPixmap.fromImage(qimg).scaled(
            self.video_label.size(), Qt.KeepAspectRatio, Qt.SmoothTransformation
        )
        self.video_label.setPixmap(pixmap)

    def resizeEvent(self, event):
        super().resizeEvent(event)
        self.audio_overlay.setGeometry(self.centralWidget().rect())


def main():
    rclpy.init()
    signals = DisplaySignals()
    ros_node = DisplayRosNode(signals)

    spin_thread = threading.Thread(target=rclpy.spin, args=(ros_node,), daemon=True)
    spin_thread.start()

    app = QApplication(sys.argv)
    app.setApplicationName("Display UI")
    app.setStyleSheet(STYLESHEET)
    window = DisplayWindow(ros_node)
    window.show()

    ret = app.exec_()
    ros_node.destroy_node()
    rclpy.shutdown()
    sys.exit(ret)


if __name__ == "__main__":
    main()
