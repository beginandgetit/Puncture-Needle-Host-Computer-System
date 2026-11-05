"""Main window: 3D view + deviation + quaternion + log + connection/model controls.

Prototype scope: rigid needle, angle guidance only (no depth, no position).
Layout change 2026-11-05: the XYZ triad moved to a corner gnomon, the bullseye
was replaced by a log panel, the pose block shows the quaternion, and the
buttons are now connect / disconnect / import model / import base.
"""

from __future__ import annotations

import csv
import time
from datetime import datetime
from typing import Optional

from PySide6.QtCore import Qt, QThread, QTimer
from PySide6.QtGui import QFont, QKeySequence, QShortcut
from PySide6.QtWidgets import (
    QCheckBox,
    QFileDialog,
    QFrame,
    QGridLayout,
    QHBoxLayout,
    QLabel,
    QMainWindow,
    QMessageBox,
    QPlainTextEdit,
    QPushButton,
    QVBoxLayout,
    QWidget,
)

from ..devices import save_last_device
from ..estimator import NeedlePose, NeedleTracker
from ..pose import Quaternion
from . import theme
from .scene import NeedleView
from .stl import load_stl
from .widgets import DeviationReadout, ValueRow, color_for_deviation
from .worker import SensorWorker

STALE_AFTER_S = 0.4
MODEL_TARGET_MM = 170.0


class QuaternionPanel(QFrame):
    """Compact W/X/Y/Z readout in a 2x2 grid."""

    _KEYS = ("W", "X", "Y", "Z")

    def __init__(self, parent: QWidget | None = None) -> None:
        super().__init__(parent)
        layout = QGridLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setHorizontalSpacing(10)
        layout.setVerticalSpacing(2)

        self._values: dict[str, QLabel] = {}
        font = QFont(theme.FONT_MONO, 11)
        for i, key in enumerate(self._KEYS):
            name = QLabel(key)
            name.setStyleSheet(f"color:{theme.DIM};")
            value = QLabel("—")
            value.setFont(font)
            value.setStyleSheet(f"color:{theme.TEXT};")
            value.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
            row, col = divmod(i, 2)
            layout.addWidget(name, row, col * 2)
            layout.addWidget(value, row, col * 2 + 1)
            self._values[key] = value

    def set_quaternion(self, q: Optional[Quaternion]) -> None:
        if q is None:
            for label in self._values.values():
                label.setText("—")
            return
        for key, component in zip(self._KEYS, (q.w, q.x, q.y, q.z)):
            self._values[key].setText(f"{component:+.4f}")


class MainWindow(QMainWindow):
    def __init__(self, config, simulated: bool = False, autostart: bool = True) -> None:
        super().__init__()
        self.setWindowTitle("WT901 穿刺针角度引导（样机）")
        self.resize(1240, 760)
        self.setStyleSheet(theme.STYLE_SHEET)

        self._config = config
        self._simulated = simulated
        self._tracker = NeedleTracker(smoothing_s=0.06)
        self._pose: Optional[NeedlePose] = None
        self._quat: Optional[Quaternion] = None
        self._last_rx: Optional[float] = None
        self._stale = True
        self._paused = False
        self._status = None
        self._saved_address: Optional[str] = None
        self._recorder = None
        self._record_rows = 0
        self._thread: Optional[QThread] = None
        self._worker: Optional[SensorWorker] = None
        self._model_mesh = None
        self._base_mesh = None

        self._build_ui()
        if autostart:
            self.start_connection()

        self._timer = QTimer(self)
        self._timer.timeout.connect(self._tick)
        self._timer.start(100)

        QShortcut(QKeySequence("R"), self).activated.connect(self.set_reference)

    # ------------------------------------------------------------------- ui
    def _build_ui(self) -> None:
        central = QWidget()
        root = QVBoxLayout(central)
        root.setContentsMargins(14, 14, 14, 10)
        root.setSpacing(10)

        self._banner = QLabel("数据中断 — 请检查传感器")
        self._banner.setObjectName("Banner")
        self._banner.setAlignment(Qt.AlignCenter)
        self._banner.hide()
        root.addWidget(self._banner)

        body = QHBoxLayout()
        body.setSpacing(12)

        self._scene = NeedleView()
        body.addWidget(self._scene, 3)

        panel = QFrame()
        panel.setObjectName("Card")
        panel.setFixedWidth(368)
        right = QVBoxLayout(panel)
        right.setContentsMargins(16, 16, 16, 16)
        right.setSpacing(10)

        title = QLabel("针尖角度引导")
        title.setObjectName("PanelTitle")
        right.addWidget(title)

        self._readout = DeviationReadout()
        right.addWidget(self._readout)

        self._log_view = QPlainTextEdit()
        self._log_view.setReadOnly(True)
        self._log_view.setMaximumBlockCount(400)
        self._log_view.setMinimumHeight(190)
        self._log_view.setStyleSheet(
            f"background:{theme.BG}; border:1px solid {theme.BORDER}; border-radius:8px;"
            f" color:{theme.MUTED}; font-family:'{theme.FONT_MONO}'; font-size:11px;"
        )
        right.addWidget(self._log_view, 1)

        pose_title = QLabel("姿态四元数")
        pose_title.setObjectName("PanelTitle")
        right.addWidget(pose_title)

        self._quat_panel = QuaternionPanel()
        right.addWidget(self._quat_panel)

        self._row_tilt = ValueRow("总倾角", "°")
        right.addWidget(self._row_tilt)

        buttons = QHBoxLayout()
        self._btn_connect = QPushButton("连接蓝牙")
        self._btn_connect.setObjectName("Primary")
        self._btn_connect.clicked.connect(self.start_connection)
        self._btn_disconnect = QPushButton("断开连接")
        self._btn_disconnect.clicked.connect(self.stop_connection)
        buttons.addWidget(self._btn_connect)
        buttons.addWidget(self._btn_disconnect)
        right.addLayout(buttons)

        io_row = QHBoxLayout()
        self._btn_model = QPushButton("导入模型 (STL)")
        self._btn_model.clicked.connect(self.import_model)
        self._btn_base = QPushButton("导入基座")
        self._btn_base.clicked.connect(self.import_base)
        io_row.addWidget(self._btn_model)
        io_row.addWidget(self._btn_base)
        right.addLayout(io_row)

        actions = QHBoxLayout()
        self._btn_ref = QPushButton("设为参考  (R)")
        self._btn_ref.clicked.connect(self.set_reference)
        self._btn_rec = QPushButton("开始记录")
        self._btn_rec.clicked.connect(self.toggle_recording)
        actions.addWidget(self._btn_ref)
        actions.addWidget(self._btn_rec)
        right.addLayout(actions)

        options = QHBoxLayout()
        self._chk_lock = QCheckBox("锁定视角")
        self._chk_lock.setChecked(False)
        self._chk_lock.setToolTip("勾选后拖动不会旋转场景，便于固定观察")
        self._chk_lock.toggled.connect(self._on_lock_toggled)

        self._chk_pause = QCheckBox("暂停数据")
        self._chk_pause.setToolTip("冻结画面，便于在不动的情况下观察姿态")
        self._chk_pause.toggled.connect(self._on_pause_toggled)

        options.addWidget(self._chk_lock)
        options.addWidget(self._chk_pause)
        options.addStretch(1)
        right.addLayout(options)

        body.addWidget(panel)
        root.addLayout(body, 1)

        self.setCentralWidget(central)

        self._status_label = QLabel("未连接")
        self.statusBar().addWidget(self._status_label)

        self._log("程序启动" + ("（模拟模式）" if self._simulated else ""))

    # ------------------------------------------------------------- logging
    def _log(self, message: str) -> None:
        stamp = time.strftime("%H:%M:%S")
        self._log_view.appendPlainText(f"[{stamp}] {message}")

    # ------------------------------------------------------------ connection
    def start_connection(self) -> None:
        if self._thread is not None and self._thread.isRunning():
            self._log("已处于连接状态")
            return
        self._thread = QThread(self)
        self._worker = SensorWorker(self._config)
        self._worker.moveToThread(self._thread)
        self._thread.started.connect(self._worker.run)
        self._worker.reading.connect(self._on_reading)
        self._worker.status.connect(self._on_status)
        self._worker.failed.connect(self._on_failed)
        self._thread.start()
        self._log("开始连接 …" if not self._simulated else "模拟数据源已启动")

    def stop_connection(self) -> None:
        if self._thread is None:
            self._log("当前未连接")
            return
        if self._worker is not None:
            self._worker.stop()
        self._thread.quit()
        if not self._thread.wait(3000):
            self._thread.terminate()
            self._thread.wait(1000)
        self._thread = None
        self._worker = None
        self._stale = True
        self._scene.set_stale(True)
        self._log("已断开连接")

    def _on_failed(self, message: str) -> None:
        self._log(f"错误：{message}")
        self._show_banner(f"连接失败：{message}", warn=True)

    def _on_status(self, status) -> None:
        was_connected = self._status is not None and self._status.state.value == "connected"
        self._status = status
        if status.state.value == "connected" and not was_connected:
            self._log(f"已连接 {status.device_name or ''} {status.address}".strip())

        if (
            status.frames > 0
            and status.address
            and status.address != self._saved_address
        ):
            self._saved_address = status.address
            save_last_device(status.address, status.device_name, status.notify_uuid or "")
        self._update_status_label()

    # -------------------------------------------------------------- readings
    def _on_reading(self, reading) -> None:
        if self._paused:
            return

        now = time.monotonic()
        self._last_rx = now

        quat = Quaternion.from_euler(reading.roll, reading.pitch, reading.yaw)
        self._quat = quat

        pose = self._tracker.update(reading, now=now)
        self._pose = pose

        if self._stale:
            self._stale = False
            self._banner.hide()
            self._log("数据恢复")

        self._scene.set_pose(pose.axis, pose.plan, pose.deviation_deg, stale=False)
        self._readout.set_value(pose.deviation_deg)
        self._quat_panel.set_quaternion(pose.quat)
        self._row_tilt.set_value(pose.tilt_deg)
        self._row_tilt.set_color(color_for_deviation(pose.deviation_deg))

        if self._recorder is not None:
            q = pose.quat
            self._recorder.writerow(
                [
                    f"{reading.t_wall:.3f}",
                    f"{q.w:.6f}", f"{q.x:.6f}", f"{q.y:.6f}", f"{q.z:.6f}",
                    f"{pose.tilt_deg:.2f}",
                    f"{pose.deviation_deg:.2f}",
                ]
            )
            self._record_rows += 1

    # ----------------------------------------------------------------- ticks
    def _tick(self) -> None:
        if self._paused:
            self._update_status_label(None)
            return

        age = None if self._last_rx is None else time.monotonic() - self._last_rx
        stale = age is None or age > STALE_AFTER_S
        if stale != self._stale:
            self._stale = stale
            self._scene.set_stale(stale)
            self._readout.set_value(self._pose.deviation_deg if self._pose else 0.0, stale=stale)
            if stale and self._thread is not None:
                self._log("数据中断")

        state = self._status.state.value if self._status is not None else "connecting"
        connected = state == "connected"

        if self._thread is None:
            self._show_banner("未连接 — 点击「连接蓝牙」", warn=True)
        elif not connected:
            self._show_banner(
                f"正在连接 {self._config.name_contains} … "
                "若长时间无响应，请关闭维特上位机 / 手机 App",
                warn=True,
            )
        elif stale:
            self._show_banner("数据中断 — 请检查传感器")
        elif self._banner.isVisible():
            self._banner.hide()

        self._update_status_label(age)

    def _update_status_label(self, age: Optional[float] = None) -> None:
        parts = []
        if self._simulated:
            parts.append("● 模拟模式")
        else:
            state = self._status.state.value if self._status is not None else "connecting"
            parts.append(
                {
                    "connected": "● 已连接",
                    "connecting": "● 连接中",
                    "reconnecting": "● 重连中",
                }.get(state, "● 未连接")
            )
        if self._paused:
            parts.append("已暂停")
        if self._status is not None:
            device_name = self._status.device_name or (
                self._config.name_contains if not self._simulated else ""
            )
            if device_name:
                parts.append(device_name)
            if self._status.state.value != "connected" and self._status.reconnects:
                parts.append(f"第 {self._status.reconnects} 次重试")
            if self._status.frames:
                parts.append(f"{self._status.rate_hz:.0f} Hz")
                if age is not None:
                    parts.append(f"延迟 {age * 1000:.0f} ms")
                parts.append(f"帧 {self._status.frames}")
            if self._status.bad_checksum:
                parts.append(f"坏帧 {self._status.bad_checksum}")
        if self._recorder is not None:
            parts.append(f"记录 {self._record_rows}")
        self._status_label.setText("   ·   ".join(parts))

    # ----------------------------------------------------------------- view
    def _on_lock_toggled(self, checked: bool) -> None:
        self._scene.set_view_locked(checked)

    def _on_pause_toggled(self, checked: bool) -> None:
        self._paused = checked
        self._chk_pause.setText("继续数据" if checked else "暂停数据")
        if not checked:
            self._last_rx = time.monotonic()
        self._log("画面已冻结" if checked else "继续接收数据")
        self._update_status_label()

    # --------------------------------------------------------------- actions
    def set_reference(self) -> None:
        if self._quat is None:
            self._log("尚无数据，无法设为参考")
            return
        self._tracker.capture_reference(self._quat)
        self._readout.set_value(0.0)
        self._quat_panel.set_quaternion(Quaternion.identity())
        self._row_tilt.set_value(0.0)
        self._log("已将当前姿态设为参考（0°）")

    def import_model(self) -> None:
        path, _ = QFileDialog.getOpenFileName(self, "选择 STL 模型", "", "STL 文件 (*.stl)")
        if not path:
            return
        try:
            mesh = load_stl(path)
        except Exception as exc:  # noqa: BLE001 - show the reason to the operator
            QMessageBox.warning(self, "导入失败", str(exc))
            self._log(f"模型导入失败：{exc}")
            return

        original = mesh.size()
        mesh = mesh.simplified(2500).centered(target_max_mm=MODEL_TARGET_MM)
        self._model_mesh = mesh
        self._scene.set_model(mesh)
        self._log(
            f"已导入模型 {mesh.name}：{mesh.triangle_count} 面，"
            f"原始 {original[0]:.1f}×{original[1]:.1f}×{original[2]:.1f} mm，"
            f"原点置于模型中心"
        )

    def import_base(self) -> None:
        path, _ = QFileDialog.getOpenFileName(self, "选择基座 STL", "", "STL 文件 (*.stl)")
        if not path:
            return
        try:
            mesh = load_stl(path)
        except Exception as exc:  # noqa: BLE001
            QMessageBox.warning(self, "导入失败", str(exc))
            self._log(f"基座导入失败：{exc}")
            return

        size = mesh.size()
        mesh = mesh.simplified(2500).with_top_at_origin()
        self._base_mesh = mesh
        self._scene.set_base(mesh)
        self._log(
            f"已导入基座 {mesh.name}：{mesh.triangle_count} 面，"
            f"尺寸 {size[0]:.1f}×{size[1]:.1f}×{size[2]:.1f} mm，顶面置于原点"
        )

    def toggle_recording(self) -> None:
        if self._recorder is None:
            default = f"needle_{datetime.now():%Y%m%d_%H%M%S}.csv"
            path, _ = QFileDialog.getSaveFileName(self, "保存记录", default, "CSV (*.csv)")
            if not path:
                return
            self._recorder_file = open(path, "w", newline="", encoding="utf-8-sig")
            self._recorder = csv.writer(self._recorder_file)
            self._recorder.writerow(["t_wall", "qw", "qx", "qy", "qz", "tilt_deg", "deviation_deg"])
            self._record_rows = 0
            self._btn_rec.setText("停止记录")
            self._log(f"开始记录：{path}")
        else:
            self._recorder_file.close()
            self._recorder = None
            self._recorder_file = None
            self._btn_rec.setText("开始记录")
            self._log(f"记录结束，共 {self._record_rows} 行")

    # -------------------------------------------------------------- lifecycle
    def shutdown(self) -> None:
        self._timer.stop()
        if self._recorder is not None:
            self._recorder_file.close()
            self._recorder = None
        if self._thread is not None:
            if self._worker is not None:
                self._worker.stop()
            self._thread.quit()
            if not self._thread.wait(3000):
                self._thread.terminate()
                self._thread.wait(1000)
            self._thread = None

    def closeEvent(self, event) -> None:  # noqa: N802
        self.shutdown()
        super().closeEvent(event)

    def capture_screenshot(self, path: str) -> bool:
        return self.grab().save(path)

    def _show_banner(self, text: str, warn: bool = False) -> None:
        object_name = "BannerWarn" if warn else "Banner"
        if (
            self._banner.isVisible()
            and self._banner.text() == text
            and self._banner.objectName() == object_name
        ):
            return
        self._banner.setText(text)
        self._banner.setObjectName(object_name)
        self._banner.style().unpolish(self._banner)
        self._banner.style().polish(self._banner)
        self._banner.show()
