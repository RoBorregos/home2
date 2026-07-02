#!/usr/bin/env python3
"""
Display UI - PyQt5 replacement for the Next.js HRI display.
Shows speech/interaction messages, audio state, camera feed, map and
question modals. Subscribes directly to ROS2 topics (no rosbridge /
web_video_server) to cut CPU usage vs the browser-based version.

The `task` ROS parameter selects the view, mirroring the Next.js routes:
default (/), gpsr, hric, laundry, ppc, restaurant, storing_groceries.
"""

import json
import sys
import threading
from datetime import datetime

import rclpy
from rclpy.node import Node
from std_msgs.msg import Empty, Float32, Int32, String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

from PyQt5.QtCore import Qt, QObject, QTimer, pyqtSignal
from PyQt5.QtGui import QColor, QFont, QImage, QPainter, QPixmap
from PyQt5.QtWidgets import (
    QApplication,
    QDialog,
    QFrame,
    QHBoxLayout,
    QLabel,
    QMainWindow,
    QPushButton,
    QScrollArea,
    QStackedWidget,
    QTextEdit,
    QVBoxLayout,
    QWidget,
)

DEFAULT_VIDEO_TOPIC = "/zed/zed_node/rgb/image_rect_color"

# Per-task default video topic (VideoFeed defaultTopic prop in the web app).
TASK_DEFAULT_VIDEO = {
    "ppc": "/vision/detections_image",
}

# Where each task's manager publishes its FSM step.
TASK_STEP_TOPICS = {
    "gpsr": "/gpsr/display/task_step",
    "hric": "/hri/display/task_step",
    "laundry": "/hri/display/task_step",
    "ppc": "/pickandplace/display/task_step",
}

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
EMERALD = "#34d399"
AMBER = "#fbbf24"

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

# Display modes shared by the GPSR/HRIC step machines.
MODE_BUTTON = "button"
MODE_CAMERA = "camera"
MODE_LOGS = "logs"
MODE_BOTH = "both"

# (key, label, icon) — same order as FSM_STEPS in gpsr/page.tsx.
GPSR_FSM_STEPS = [
    ("waiting_for_button", "Wait Start", "\U0001f525"),
    ("start", "Start", "\U0001f916"),
    ("wait_button_command", "Ready", "\U0001f525"),
    ("waiting_for_command", "Listening", "\U0001f3a4"),
    ("plan_and_execute_batch", "Merged Plan", "\U0001f4ca"),
    ("executing", "Executing", "▶️"),
    ("done", "Done", "✅"),
]

GPSR_COMMAND_DISPLAY = {
    "go_to": MODE_CAMERA,
    "pick_object": MODE_CAMERA,
    "place_object": MODE_CAMERA,
    "say_with_context": MODE_LOGS,
    "say": MODE_LOGS,
    "answer_question": MODE_BOTH,
    "get_visual_info": MODE_CAMERA,
    "give_object": MODE_BOTH,
    "follow_person_until": MODE_CAMERA,
    "guide_person_to": MODE_CAMERA,
    "get_person_info": MODE_CAMERA,
    "count": MODE_CAMERA,
    "find_person": MODE_CAMERA,
    "find_person_by_name": MODE_CAMERA,
}

GPSR_COMMAND_INFO = {
    "go_to": ("Go To", "\U0001f9ed"),
    "pick_object": ("Pick Object", "\U0001f4e6"),
    "place_object": ("Place Object", "\U0001f4e6"),
    "say_with_context": ("Say", "\U0001f4ac"),
    "say": ("Say", "\U0001f4ac"),
    "answer_question": ("Answer", "\U0001f4ac"),
    "get_visual_info": ("Visual Info", "\U0001f441"),
    "give_object": ("Give Object", "\U0001f91d"),
    "follow_person_until": ("Follow Person", "\U0001f9cd"),
    "guide_person_to": ("Guide Person", "\U0001f9ed"),
    "get_person_info": ("Person Info", "\U0001faaa"),
    "count": ("Count", "\U0001f4ca"),
    "find_person": ("Find Person", "\U0001f50d"),
    "find_person_by_name": ("Find by Name", "\U0001faaa"),
}

# (key, label, icon, display_mode) — same order as TASK_STEPS in laundry/page.tsx.
LAUNDRY_TASK_STEPS = [
    ("wait_for_button", "Waiting for Start", "\U0001f525", MODE_BUTTON),
    ("start", "Starting", "\U0001f916", MODE_CAMERA),
    ("navigate_to_basket", "Navigate to Basket", "\U0001f9ed", MODE_CAMERA),
    ("pick_laundry_basket", "Picking Basket", "\U0001f9fa", MODE_CAMERA),
    ("navigate_to_laundry_table", "Navigate to Table", "\U0001f9ed", MODE_CAMERA),
    ("unload_laundry", "Unloading Basket", "\U0001f9fa", MODE_CAMERA),
    ("pick_clothes_basket", "Picking Clothes", "\U0001f455", MODE_CAMERA),
    ("place_clothes_table", "Placing Clothes on Table", "\U0001f455", MODE_CAMERA),
    ("navigate_to_laundry_machine", "Navigate to Machine", "\U0001f9ed", MODE_CAMERA),
    ("pick_clothes_wm", "Picking from Machine", "\U0001f9fc", MODE_CAMERA),
    ("close_laundry_machine", "Closing Machine", "\U0001f6aa", MODE_CAMERA),
    ("navigate_to_table_with_clothes", "Navigate to Table", "\U0001f9ed", MODE_CAMERA),
    ("end", "Finished", "✅", MODE_LOGS),
]

# (key, label, icon, display_mode) — same order as TASK_STEPS in ppc/page.tsx.
PPC_TASK_STEPS = [
    ("wait_for_button", "Wait Start", "\U0001f525", MODE_BUTTON),
    ("start", "Starting", "▶️", MODE_CAMERA),
    ("perceive_table", "Perceive Table", "\U0001f441", MODE_BOTH),
    ("cleanup_phase", "Cleanup Phase", "\U0001f4e6", MODE_BOTH),
    ("breakfast_phase", "Breakfast Phase", "☕", MODE_BOTH),
    ("end", "Finished", "✅", MODE_LOGS),
]

# Raw FSM state -> macro step shown in the pill bar (getStepKey in ppc/page.tsx).
PPC_STEP_KEYS = {
    "wait_for_button": "wait_for_button",
    "start": "start",
    "perceive_table": "perceive_table",
    "announce_objects": "perceive_table",
    "sort_objects": "perceive_table",
    "scan_cabinet_shelves": "cleanup_phase",
    "cleanup_loop": "cleanup_phase",
    "pick_object": "cleanup_phase",
    "determine_placement": "cleanup_phase",
    "check_dishwasher": "cleanup_phase",
    "request_dishwasher_help": "cleanup_phase",
    "navigate_to_placement": "cleanup_phase",
    "place_object": "cleanup_phase",
    "start_breakfast_prep": "breakfast_phase",
    "get_breakfast_items": "breakfast_phase",
    "navigate_to_item_source": "breakfast_phase",
    "pick_breakfast_item": "breakfast_phase",
    "navigate_to_dining": "breakfast_phase",
    "pour_into_bowl": "breakfast_phase",
    "place_breakfast_item": "breakfast_phase",
    "end": "end",
    "debug": "end",
}

# (key, label, icon, display_mode) — same order as TASK_STEPS in hric/page.tsx.
HRIC_TASK_STEPS = [
    ("wait_for_button", "Waiting for Start", "\U0001f525", MODE_BUTTON),
    ("start", "Starting", "\U0001f4f7", MODE_CAMERA),
    ("wait_for_guest", "Waiting for Guest", "\U0001f441", MODE_CAMERA),
    ("greeting", "Greeting", "\U0001f4ac", MODE_BOTH),
    ("save_face", "Saving Face", "\U0001faaa", MODE_CAMERA),
    ("take_bag", "Taking Bag", "\U0001f6cd", MODE_BOTH),
    ("navigate_to_living_room", "Navigate to Living Room", "\U0001f9ed", MODE_CAMERA),
    ("find_seat", "Finding Seat", "\U0001fa91", MODE_CAMERA),
    ("navigate_to_entrance", "Navigate to Entrance", "\U0001f9ed", MODE_CAMERA),
    ("introduction", "Introduction", "\U0001f9cd", MODE_BOTH),
    ("take_bag_deliver", "Deliver Bag", "\U0001f91d", MODE_BOTH),
]


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
    task_step_changed = pyqtSignal(str)
    command_index_changed = pyqtSignal(int)


class DisplayRosNode(Node):
    def __init__(self, signals: DisplaySignals):
        super().__init__("display_ui")
        self.signals = signals
        self.bridge = CvBridge()
        self.task = self.declare_parameter("task", "default").value
        video_param = self.declare_parameter("default_video_topic", "").value
        self.video_topic = video_param or TASK_DEFAULT_VIDEO.get(
            self.task, DEFAULT_VIDEO_TOPIC
        )
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

        step_topic = TASK_STEP_TOPICS.get(self.task)
        if step_topic is not None:
            self.create_subscription(String, step_topic, self._on_task_step, 10)
        if self.task == "gpsr":
            self.create_subscription(
                Int32, "/gpsr/display/command_index", self._on_command_index, 10
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

    def _on_task_step(self, msg: String):
        self.signals.task_step_changed.emit(msg.data.strip().lower())

    def _on_command_index(self, msg: Int32):
        self.signals.command_index_changed.emit(msg.data)

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


class AudioPill(QLabel):
    """Small header pill mirroring AudioStateIndicator's idle/saying chips."""

    def __init__(self, signals: DisplaySignals):
        super().__init__()
        signals.audio_state_changed.connect(self.set_state)
        self.set_state("idle")

    def set_state(self, state: str):
        if state == "saying":
            self.setText("\U0001f50a Speaking")
            self.setStyleSheet(
                f"color: {PURPLE}; background-color: rgba(157,95,214,50);"
                "border-radius: 12px; padding: 4px 12px; font-weight: bold;"
            )
            self.show()
        elif state == "idle":
            self.setText("\U0001f507 Idle")
            self.setStyleSheet(
                f"color: {TEXT_GRAY}; background-color: transparent;"
                "border-radius: 12px; padding: 4px 12px;"
            )
            self.show()
        else:
            # listening/thinking/loading are shown by the full-window overlay.
            self.hide()


class StartButton(QPushButton):
    """Publishes /hri/display/button_press; disabled while a task is active."""

    def __init__(self, ros_node: DisplayRosNode, xl: bool = False):
        super().__init__("\U0001f525 Start")
        self.setFixedHeight(128 if xl else 60)
        font = self.font()
        font.setPointSize(24 if xl else 12)
        font.setBold(True)
        self.setFont(font)
        self.clicked.connect(ros_node.publish_button_press)
        ros_node.signals.task_status_changed.connect(
            lambda active: self.setEnabled(not active)
        )


class VideoView(QWidget):
    """Topic label + live frame, fed by the shared ROS video subscription."""

    def __init__(self, signals: DisplaySignals, initial_topic: str):
        super().__init__()
        layout = QVBoxLayout(self)
        self.topic_label = QLabel(f"Video feed at {initial_topic}")
        self.topic_label.setAlignment(Qt.AlignCenter)
        layout.addWidget(self.topic_label)

        self.video_label = QLabel()
        self.video_label.setAlignment(Qt.AlignCenter)
        self.video_label.setMinimumSize(480, 360)
        self.video_label.setStyleSheet(
            f"background-color: {BG_DARKER}; border: 1px solid {BORDER_LIGHT}; border-radius: 8px;"
        )
        layout.addWidget(self.video_label, 1)

        signals.video_topic_changed.connect(
            lambda topic: self.topic_label.setText(f"Video feed at {topic}")
        )
        signals.frame_received.connect(self._on_frame)

    def _on_frame(self, qimg: QImage):
        pixmap = QPixmap.fromImage(qimg).scaled(
            self.video_label.size(), Qt.KeepAspectRatio, Qt.SmoothTransformation
        )
        self.video_label.setPixmap(pixmap)


class MessagesPanel(QScrollArea):
    def __init__(self, signals: DisplaySignals = None):
        super().__init__()
        self.setWidgetResizable(True)
        self._container = QWidget()
        self._layout = QVBoxLayout(self._container)
        self._layout.setAlignment(Qt.AlignTop)
        self.setWidget(self._container)
        if signals is not None:
            signals.message_received.connect(self.add_message)

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


class StepPillBar(QWidget):
    """Horizontal done/active/pending progress pills (StepPill in the web app)."""

    def __init__(self, steps):
        super().__init__()
        self._pills = []
        layout = QHBoxLayout(self)
        layout.setContentsMargins(8, 6, 8, 6)
        layout.setSpacing(6)
        for _key, label, icon in steps:
            pill = QLabel(f"{icon} {label}")
            self._pills.append(pill)
            layout.addWidget(pill)
        layout.addStretch()
        self.set_active_index(-1)

    def set_active_index(self, active: int):
        for i, pill in enumerate(self._pills):
            if active >= 0 and i < active:
                style = f"color: {EMERALD}; background-color: rgba(16,185,129,50);"
            elif i == active:
                style = (
                    f"color: {BLUE_HOVER}; background-color: rgba(59,111,224,50);"
                    f"border: 1px solid {BLUE};"
                )
            else:
                style = "color: rgba(180,180,180,150); background-color: rgba(255,255,255,13);"
            pill.setStyleSheet(
                style + "border-radius: 11px; padding: 3px 10px; font-size: 11px;"
            )


class CommandBadge(QLabel):
    """Amber chip naming the GPSR command currently executing."""

    def __init__(self):
        super().__init__()
        self.setStyleSheet(
            f"color: {AMBER}; background-color: rgba(245,158,11,38);"
            "border: 1px solid rgba(245,158,11,77); border-radius: 11px;"
            "padding: 3px 10px; font-size: 11px; font-weight: bold;"
        )
        self.set_command(None)

    def set_command(self, command):
        info = GPSR_COMMAND_INFO.get(command) if command else None
        if info is None:
            self.hide()
        else:
            label, icon = info
            self.setText(f"{icon} {label}")
            self.show()


class CommandCounter(QWidget):
    """CMD 1-2-3 progress circles for the GPSR command index."""

    def __init__(self, count: int = 3):
        super().__init__()
        layout = QHBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(4)
        caption = QLabel("CMD")
        caption.setStyleSheet(f"color: {TEXT_GRAY}; font-size: 10px;")
        layout.addWidget(caption)
        self._circles = []
        for i in range(count):
            circle = QLabel(str(i + 1))
            circle.setFixedSize(22, 22)
            circle.setAlignment(Qt.AlignCenter)
            self._circles.append(circle)
            layout.addWidget(circle)
        self.set_index(0)

    def set_index(self, index: int):
        for i, circle in enumerate(self._circles):
            if i < index:
                style = f"color: {EMERALD}; background-color: rgba(16,185,129,50);"
            elif i == index:
                style = (
                    f"color: {BLUE_HOVER}; background-color: rgba(59,111,224,50);"
                    f"border: 1px solid {BLUE};"
                )
            else:
                style = "color: rgba(180,180,180,100); background-color: rgba(255,255,255,13);"
            circle.setStyleSheet(
                style + "border-radius: 11px; font-size: 10px; font-weight: bold;"
            )


def horizontal_separator():
    line = QFrame()
    line.setFrameShape(QFrame.HLine)
    line.setStyleSheet(f"color: {BORDER_LIGHT};")
    return line


class BaseWindow(QMainWindow):
    """Common wiring: audio overlay + timeout, optional map/question dialogs."""

    def __init__(self, ros_node: DisplayRosNode, title: str, with_dialogs: bool = True):
        super().__init__()
        self.ros_node = ros_node
        self.setWindowTitle(title)
        self.resize(1100, 700)

        self._central = QWidget()
        self.setCentralWidget(self._central)

        self.audio_overlay = AudioOverlay(self._central)
        self._audio_timeout_timer = QTimer(self)
        self._audio_timeout_timer.setSingleShot(True)
        self._audio_timeout_timer.timeout.connect(lambda: self._on_audio_state("idle"))
        ros_node.signals.audio_state_changed.connect(self._on_audio_state)

        if with_dialogs:
            self.map_dialog = MapDialog(self)
            self.question_dialog = QuestionDialog(self)
            self.question_dialog.answered.connect(ros_node.publish_answer)
            ros_node.signals.map_received.connect(self.map_dialog.show_map)
            ros_node.signals.question_received.connect(self._on_question)

    def make_header(self, title: str, extra_left=(), extra_right=()):
        header = QWidget()
        layout = QHBoxLayout(header)
        layout.setContentsMargins(12, 8, 12, 8)
        title_label = QLabel(title)
        title_label.setStyleSheet("font-size: 16px; font-weight: bold;")
        layout.addWidget(title_label)
        for widget in extra_left:
            layout.addWidget(widget)
        layout.addStretch()
        for widget in extra_right:
            layout.addWidget(widget)
        layout.addWidget(AudioPill(self.ros_node.signals))
        return header

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

    def resizeEvent(self, event):
        super().resizeEvent(event)
        self.audio_overlay.setGeometry(self._central.rect())


class SplitWindow(BaseWindow):
    """Messages on the left, start button + video on the right.

    Layout of the default (/), ppc and restaurant pages.
    """

    def __init__(self, ros_node: DisplayRosNode, title: str):
        super().__init__(ros_node, title)
        signals = ros_node.signals

        root = QVBoxLayout(self._central)
        root.setContentsMargins(0, 0, 0, 0)
        root.setSpacing(0)
        root.addWidget(self.make_header(title))
        root.addWidget(horizontal_separator())

        content = QHBoxLayout()
        content.addWidget(MessagesPanel(signals), 1)

        right = QVBoxLayout()
        right.addWidget(StartButton(ros_node))
        right.addWidget(VideoView(signals, ros_node.video_topic), 1)
        content.addLayout(right, 1)
        root.addLayout(content, 1)


class CenteredWindow(BaseWindow):
    """Start button above a centered video feed (laundry / storing groceries)."""

    def __init__(self, ros_node: DisplayRosNode, title: str):
        super().__init__(ros_node, title, with_dialogs=False)
        signals = ros_node.signals

        root = QVBoxLayout(self._central)
        root.setContentsMargins(0, 0, 0, 0)
        root.setSpacing(0)
        root.addWidget(self.make_header(title))
        root.addWidget(horizontal_separator())

        body = QVBoxLayout()
        body.setContentsMargins(24, 24, 24, 24)
        start = StartButton(ros_node)
        start.setMaximumWidth(500)
        body.addWidget(start, 0, Qt.AlignHCenter)
        body.addWidget(VideoView(signals, ros_node.video_topic), 1)
        root.addLayout(body, 1)


class SteppedWindow(BaseWindow):
    """Step pill bar + button/camera/logs/both stacked content (gpsr / hric)."""

    MODES = (MODE_BUTTON, MODE_CAMERA, MODE_LOGS, MODE_BOTH)

    def __init__(self, ros_node: DisplayRosNode, title: str, steps, extra_right=()):
        super().__init__(ros_node, title)
        signals = ros_node.signals
        self.steps = steps

        root = QVBoxLayout(self._central)
        root.setContentsMargins(0, 0, 0, 0)
        root.setSpacing(0)
        root.addWidget(self.make_header(title, extra_right=extra_right))
        root.addWidget(horizontal_separator())

        self.pill_bar = StepPillBar(steps)
        root.addWidget(self.pill_bar)
        root.addWidget(horizontal_separator())

        self.stack = QStackedWidget()

        button_page = QWidget()
        button_layout = QVBoxLayout(button_page)
        start = StartButton(ros_node, xl=True)
        start.setMaximumWidth(500)
        button_layout.addWidget(start, 0, Qt.AlignCenter)
        self.stack.addWidget(button_page)

        self.stack.addWidget(VideoView(signals, ros_node.video_topic))
        self.stack.addWidget(MessagesPanel(signals))

        both_page = QWidget()
        both_layout = QHBoxLayout(both_page)
        both_layout.addWidget(MessagesPanel(signals), 1)
        both_layout.addWidget(VideoView(signals, ros_node.video_topic), 1)
        self.stack.addWidget(both_page)

        root.addWidget(self.stack, 1)
        signals.task_step_changed.connect(self._on_task_step)
        self.set_mode(MODE_BUTTON)

    def set_mode(self, mode: str):
        self.stack.setCurrentIndex(self.MODES.index(mode))

    def step_index(self, key: str) -> int:
        for i, (step_key, _label, _icon) in enumerate(self.steps):
            if step_key == key:
                return i
        return -1

    def _on_task_step(self, step: str):
        raise NotImplementedError


class GpsrWindow(SteppedWindow):
    def __init__(self, ros_node: DisplayRosNode):
        self.command_badge = CommandBadge()
        self.command_counter = CommandCounter()
        super().__init__(
            ros_node,
            "\U0001f916 GPSR",
            GPSR_FSM_STEPS,
            extra_right=(self.command_badge, self.command_counter),
        )
        ros_node.signals.command_index_changed.connect(self.command_counter.set_index)

    def _on_task_step(self, step: str):
        # gpsr publishes "executing:<command>" while running a parsed command.
        if step.startswith("executing:"):
            fsm_state, command = "executing", step[len("executing:") :]
        else:
            fsm_state, command = step, None

        self.command_badge.set_command(command)
        self.pill_bar.set_active_index(self.step_index(fsm_state))
        self.set_mode(self._display_mode(fsm_state, command))

    @staticmethod
    def _display_mode(fsm_state: str, command) -> str:
        if fsm_state in ("waiting_for_button", "wait_button_command"):
            return MODE_BUTTON
        if fsm_state == "start":
            return MODE_CAMERA
        if fsm_state in (
            "waiting_for_command",
            "plan_and_execute_batch",
            "finished_command",
            "done",
        ):
            return MODE_LOGS
        if fsm_state == "executing" and command:
            return GPSR_COMMAND_DISPLAY.get(command, MODE_CAMERA)
        return MODE_CAMERA


class KeyedStepsWindow(SteppedWindow):
    """Stepped view driven by exact step keys, ignoring unknown ones (hric / laundry)."""

    def __init__(self, ros_node: DisplayRosNode, title: str, steps_with_modes):
        self._modes = {key: mode for key, _l, _i, mode in steps_with_modes}
        steps = [(key, label, icon) for key, label, icon, _mode in steps_with_modes]
        super().__init__(ros_node, title, steps)

    def _on_task_step(self, step: str):
        if step not in self._modes:
            return  # only accept valid step keys, like the web page
        self.pill_bar.set_active_index(self.step_index(step))
        self.set_mode(self._modes[step])


class PpcWindow(SteppedWindow):
    """PPC groups many FSM states into macro pill steps via PPC_STEP_KEYS."""

    def __init__(self, ros_node: DisplayRosNode):
        self._modes = {key: mode for key, _l, _i, mode in PPC_TASK_STEPS}
        steps = [(key, label, icon) for key, label, icon, _mode in PPC_TASK_STEPS]
        super().__init__(ros_node, "\U0001f4ac Pick and Place Challenge", steps)

    def _on_task_step(self, step: str):
        key = PPC_STEP_KEYS.get(step, "wait_for_button")
        self.pill_bar.set_active_index(self.step_index(key))
        self.set_mode(self._modes[key])


TASK_WINDOWS = {
    "default": lambda node: SplitWindow(node, "\U0001f4ac ROS2 Messages"),
    "gpsr": GpsrWindow,
    "hric": lambda node: KeyedStepsWindow(
        node, "\U0001f4ac HRI Challenge", HRIC_TASK_STEPS
    ),
    "laundry": lambda node: KeyedStepsWindow(
        node, "\U0001f9fa Doing Laundry", LAUNDRY_TASK_STEPS
    ),
    "ppc": PpcWindow,
    "restaurant": lambda node: SplitWindow(node, "\U0001f4ac Restaurant Task"),
    "storing_groceries": lambda node: CenteredWindow(
        node, "\U0001f4ac Storing Groceries"
    ),
}


def main():
    rclpy.init()
    signals = DisplaySignals()
    ros_node = DisplayRosNode(signals)

    spin_thread = threading.Thread(target=rclpy.spin, args=(ros_node,), daemon=True)
    spin_thread.start()

    app = QApplication(sys.argv)
    app.setApplicationName("Display UI")
    app.setStyleSheet(STYLESHEET)

    factory = TASK_WINDOWS.get(ros_node.task)
    if factory is None:
        ros_node.get_logger().warn(
            f"Unknown task '{ros_node.task}', falling back to default view. "
            f"Valid tasks: {', '.join(TASK_WINDOWS)}"
        )
        factory = TASK_WINDOWS["default"]
    window = factory(ros_node)
    window.show()

    ret = app.exec_()
    ros_node.destroy_node()
    rclpy.shutdown()
    sys.exit(ret)


if __name__ == "__main__":
    main()
