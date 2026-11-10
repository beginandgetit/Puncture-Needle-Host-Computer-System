"""Main window: 3D view + deviation + quaternion + log + connection/model controls.

Prototype scope: rigid needle, angle guidance only (no depth, no position).
Layout change 2026-11-05: the XYZ triad moved to a corner gnomon, the bullseye
was replaced by a log panel, the pose block shows the quaternion, and the
buttons are now connect / disconnect / import model / import base.
"""

from __future__ import annotations

import csv
import time
from dataclasses import replace
from datetime import datetime
from typing import Optional

from PySide6.QtCore import Qt, QThread, QTimer
from PySide6.QtGui import QFont, QKeySequence, QShortcut
from PySide6.QtWidgets import (
    QCheckBox,
    QComboBox,
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
from ..mapping import reading_quaternion
from ..rig import Rig
from ..pose import Quaternion
from . import theme
from .model_io import BackendMissing, load_model
from .scene import NeedleView
from .deviation import DeviationPair
from .worker import SensorWorker

STALE_AFTER_S = 0.4
MODEL_TARGET_MM = 170.0
MODEL_FILTER = "三维模型 (*.stl *.stp *.step *.gltf *.glb)"

# GUI polls the worker mailbox instead of receiving one Qt signal per BLE
# sample.  ~30 fps keeps latency bounded even when painting is slower than the
# sensor ODR: intermediate samples are dropped, not queued.
GUI_POLL_MS = 33
STATUS_INTERVAL_S = 0.2
# QPainter is CPU-bound; 8000 triangles cost ~60 ms on the development laptop.
# 3000 is visually sufficient for the needle/handle and cuts that roughly 3x.
MODEL_MAX_FACES = 3000


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
        self.resize(1400, 860)
        self.setStyleSheet(theme.STYLE_SHEET)

        self._config = config
        self._simulated = simulated
        self._rig = Rig.load()
        self._tracker = NeedleTracker(
            needle_axis_sensor=self._rig.needle_axis_sensor,
            # 60 ms smoothing is a direct 60 ms lag; 20 ms only removes the
            # worst sample jitter.  Set 0 for minimum latency.
            smoothing_s=0.02,
            model_axis=self._rig.model_axis,
            model_right=self._rig.model_right,
            invert_quaternion=self._rig.invert_quaternion,
        )
        self._pose: Optional[NeedlePose] = None
        self._quat: Optional[Quaternion] = None
        self._last_rx: Optional[float] = None
        self._last_seq: Optional[int] = None
        self._dropped = 0
        self._last_status_update = 0.0
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
        self._timer.start(GUI_POLL_MS)

        QShortcut(QKeySequence("R"), self).activated.connect(self.set_reference)

    # ------------------------------------------------------------------- ui
    def _build_ui(self) -> None:
        central = QWidget()
        root = QVBoxLayout(central)
        root.setContentsMargins(10, 10, 10, 8)
        root.setSpacing(8)

        self._banner = QLabel("数据中断 — 请检查传感器")
        self._banner.setObjectName("Banner")
        self._banner.setAlignment(Qt.AlignCenter)
        self._banner.hide()
        root.addWidget(self._banner)

        # ---------------------------------------------------------- toolbar
        toolbar = QHBoxLayout()
        title = QLabel("针尖角度引导")
        title.setObjectName("PanelTitle")
        toolbar.addWidget(title)
        toolbar.addStretch(1)

        toolbar.addWidget(QLabel("默认视角"))
        self._cmb_default = QComboBox()
        for label, key in (("俯视", "top"), ("主视", "front"), ("侧视", "side")):
            self._cmb_default.addItem(label, key)
        self._cmb_default.currentIndexChanged.connect(self._on_default_view_changed)
        toolbar.addWidget(self._cmb_default)

        self._btn_reset_view = QPushButton("恢复默认视角")
        self._btn_reset_view.clicked.connect(self.reset_view)
        toolbar.addWidget(self._btn_reset_view)
        root.addLayout(toolbar)

        body = QHBoxLayout()
        body.setSpacing(10)

        # ------------------------------------------------------ left column
        left = QFrame()
        left.setObjectName("Card")
        left.setFixedWidth(296)
        lv = QVBoxLayout(left)
        lv.setContentsMargins(12, 12, 12, 12)
        lv.setSpacing(8)

        view_title = QLabel("切换主视图")
        view_title.setObjectName("PanelTitle")
        lv.addWidget(view_title)

        self._view_buttons = {}
        grid = QGridLayout()
        grid.setSpacing(6)
        for index, (label, key) in enumerate(
            (("主视", "front"), ("侧视", "side"), ("俯视", "top"), ("等轴", "iso"))
        ):
            button = QPushButton(label)
            button.setCheckable(True)
            button.setToolTip("只切换中间的大视图；左侧小视图固定不动")
            button.clicked.connect(lambda _checked=False, k=key: self._set_main_view(k))
            grid.addWidget(button, index // 2, index % 2)
            self._view_buttons[key] = button
        lv.addLayout(grid)

        lv.addWidget(self._small_title("主视图"))
        self._mini_front = NeedleView(compact=True)
        self._mini_front.set_camera_preset("front")
        self._mini_front.setMinimumHeight(132)
        lv.addWidget(self._mini_front)

        lv.addWidget(self._small_title("侧视图"))
        self._mini_side = NeedleView(compact=True)
        self._mini_side.set_camera_preset("side")
        self._mini_side.setMinimumHeight(132)
        lv.addWidget(self._mini_side)
        lv.addStretch(1)
        body.addWidget(left)

        # ----------------------------------------------------------- main
        self._scene = NeedleView()
        self._scene.set_default_preset("top")
        self._scene.set_camera_preset("top")
        body.addWidget(self._scene, 1)

        # ---------------------------------------------------------- right
        panel = QFrame()
        panel.setObjectName("Card")
        panel.setFixedWidth(360)
        right = QVBoxLayout(panel)
        right.setContentsMargins(14, 14, 14, 14)
        right.setSpacing(9)

        self._deviation = DeviationPair()
        right.addWidget(self._deviation)

        self._log_view = QPlainTextEdit()
        self._log_view.setReadOnly(True)
        self._log_view.setMaximumBlockCount(400)
        self._log_view.setMinimumHeight(170)
        self._log_view.setStyleSheet(
            f"background:{theme.BG}; border:1px solid {theme.BORDER}; border-radius:8px;"
            f" color:{theme.MUTED}; font-family:'{theme.FONT_MONO}'; font-size:11px;"
        )
        right.addWidget(self._log_view, 1)

        pose_title = QLabel("姿态四元数（相对参考）")
        pose_title.setObjectName("PanelTitle")
        right.addWidget(pose_title)

        self._quat_panel = QuaternionPanel()
        right.addWidget(self._quat_panel)

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
        self._btn_model = QPushButton("导入模型")
        self._btn_model.setToolTip("支持 STL / STEP(STP) / glTF(GLB)；STEP 需 cascadio，glTF 需 trimesh")
        self._btn_model.clicked.connect(self.import_model)
        self._btn_base = QPushButton("导入基座")
        self._btn_base.clicked.connect(self.import_base)
        io_row.addWidget(self._btn_model)
        io_row.addWidget(self._btn_base)
        right.addLayout(io_row)

        calib_row = QHBoxLayout()
        self._btn_calib = QPushButton("标定针轴（竖直）")
        self._btn_calib.setToolTip(
            "把针竖直向下放稳后点击：用重力反解针轴在传感器系的方向，"
            "并写入 config/rig.json；随后自动重新定参考"
        )
        self._btn_calib.clicked.connect(self.calibrate_axis_vertical)
        calib_row.addWidget(self._btn_calib)
        right.addLayout(calib_row)

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
        self._chk_lock.setToolTip("勾选后拖动不会旋转主视图")
        self._chk_lock.toggled.connect(self._on_lock_toggled)
        self._chk_pause = QCheckBox("暂停数据")
        self._chk_pause.toggled.connect(self._on_pause_toggled)
        self._chk_flip = QCheckBox("针轴反向")
        self._chk_flip.setToolTip("实物方向反了时勾选：把针轴在传感器系里取反并重新定参考")
        self._chk_flip.toggled.connect(self._on_flip_toggled)
        self._chk_cad = QCheckBox("CAD 坐标")
        self._chk_cad.setChecked(self._rig.cad_mode)
        self._chk_cad.setToolTip("按装配真实坐标摆放模型（不居中、不缩放），针座为旋转中心")
        self._chk_cad.toggled.connect(self._on_cad_toggled)
        options.addWidget(self._chk_lock)
        options.addWidget(self._chk_pause)
        options.addWidget(self._chk_flip)
        options.addWidget(self._chk_cad)
        options.addStretch(1)
        right.addLayout(options)

        body.addWidget(panel)
        root.addLayout(body, 1)

        self.setCentralWidget(central)

        self._status_label = QLabel("未连接")
        self.statusBar().addWidget(self._status_label)

        self._log("程序启动" + ("（模拟模式）" if self._simulated else ""))
        self._set_main_view("top")
        for view in self._views():
            view.fit_distance()

    def _small_title(self, text: str) -> QLabel:
        label = QLabel(text)
        label.setStyleSheet(f"color:{theme.MUTED}; font-size:11px;")
        return label

    # ------------------------------------------------------------- views
    def _views(self):
        return (self._scene, self._mini_front, self._mini_side)

    def _set_main_view(self, key: str) -> None:
        self._scene.set_camera_preset(key)
        for name, button in self._view_buttons.items():
            button.setChecked(name == key)
        self._log(f"主视图切换为「{ {'top': '俯视', 'front': '主视', 'side': '侧视', 'iso': '等轴'}[key] }」")

    def _on_default_view_changed(self, index: int) -> None:
        key = self._cmb_default.itemData(index) or "top"
        self._scene.set_default_preset(key)
        self._set_main_view(key)
        self._log(f"默认视角设为「{self._cmb_default.itemText(index)}」")

    def reset_view(self) -> None:
        self._scene.reset_camera()
        self._set_main_view(self._scene.default_preset)
        self._mini_front.set_camera_preset("front")
        self._mini_side.set_camera_preset("side")
        self._log("已恢复默认视角")

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
        self._worker.status.connect(self._on_status)
        self._last_seq = None
        self._last_rx = None
        self._dropped = 0
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
    def _poll_reading(self) -> None:
        """Take the newest sample from the worker mailbox (latest wins)."""

        if self._paused or self._worker is None:
            return
        reading = self._worker.latest()
        if reading is None:
            return
        seq = getattr(reading, "seq", None)
        if seq is not None and seq == self._last_seq:
            return
        if seq is not None and self._last_seq is not None and seq > self._last_seq:
            self._dropped += seq - self._last_seq - 1
        self._last_seq = seq
        self._on_reading(reading)

    def _on_reading(self, reading) -> None:
        if self._paused:
            return

        # t_mono is stamped at BLE decode, so using it (instead of "now")
        # measures arrival-to-display age and keeps smoothing consistent.
        now = getattr(reading, "t_mono", None)
        if now is None:
            now = time.monotonic()
        self._last_rx = now

        # Prefer the device quaternion (0x59).  It carries the full pose and
        # avoids the Euler singularities; fall back to angles for old frames.
        try:
            quat = reading_quaternion(reading, invert=self._rig.invert_quaternion)
        except ValueError:
            quat = None
        if quat is None:
            quat = Quaternion.from_euler(reading.roll, reading.pitch, reading.yaw)
        self._quat = quat

        pose = self._tracker.update(reading, now=now)
        self._pose = pose

        if self._stale:
            self._stale = False
            self._banner.hide()
            self._log("数据恢复")

        for view in self._views():
            view.set_pose(
                pose.axis,
                pose.plan,
                pose.deviation_deg,
                stale=False,
                model_rotation=pose.model_rotation,
            )
        self._quat_panel.set_quaternion(pose.quat)
        self._deviation.set_value(
            pose.deviation_x_deg,
            pose.deviation_y_deg,
            pose.deviation_deg,
            pose.direction,
        )

        if self._recorder is not None:
            q = pose.quat
            self._recorder.writerow(
                [
                    f"{reading.t_wall:.3f}",
                    f"{q.w:.6f}", f"{q.x:.6f}", f"{q.y:.6f}", f"{q.z:.6f}",
                    f"{pose.tilt_deg:.2f}",
                    f"{pose.deviation_deg:.2f}",
                    f"{pose.deviation_x_deg:.2f}",
                    f"{pose.deviation_y_deg:.2f}",
                ]
            )
            self._record_rows += 1

    # ----------------------------------------------------------------- ticks
    def _tick(self) -> None:
        if self._paused:
            self._update_status_label(None)
            return

        self._poll_reading()
        age = None if self._last_rx is None else time.monotonic() - self._last_rx
        stale = age is None or age > STALE_AFTER_S
        if stale != self._stale:
            self._stale = stale
            for view in self._views():
                view.set_stale(stale)
            pose = self._pose
            self._deviation.set_value(
                pose.deviation_x_deg if pose else 0.0,
                pose.deviation_y_deg if pose else 0.0,
                pose.deviation_deg if pose else 0.0,
                pose.direction if pose else "对准",
                stale=stale,
            )
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

        # Rebuilding the status text at 30 Hz would itself cost paint time; 5 Hz
        # is enough to read the latency/drop counters.
        now = time.monotonic()
        if now - self._last_status_update >= STATUS_INTERVAL_S:
            self._last_status_update = now
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
                    parts.append(f"主机延迟 {age * 1000:.0f} ms")
                parts.append(f"帧 {self._status.frames}")
            if self._status.bad_checksum:
                parts.append(f"坏帧 {self._status.bad_checksum}")
        if self._dropped:
            parts.append(f"跳过 {self._dropped}")
        paint_ms = getattr(self._scene, "last_paint_ms", 0.0)
        if paint_ms > 0.0:
            parts.append(f"渲染 {paint_ms:.0f} ms")
        if self._recorder is not None:
            parts.append(f"记录 {self._record_rows}")
        self._status_label.setText("   ·   ".join(parts))

    # ----------------------------------------------------------------- view
    def _on_lock_toggled(self, checked: bool) -> None:
        self._scene.set_view_locked(checked)

    def _on_flip_toggled(self, checked: bool) -> None:
        base = self._rig.needle_axis_sensor
        axis = tuple(-v for v in base) if checked else base
        self._tracker.set_needle_axis(axis)
        if self._quat is not None:
            self._tracker.capture_reference(self._quat)
        self._log(
            "针轴方向：" + ("已反向" if checked else "默认")
            + f" ({axis[0]:g}, {axis[1]:g}, {axis[2]:g})"
        )

    def _on_cad_toggled(self, checked: bool) -> None:
        self._log("CAD 坐标模式：" + ("开（按真实坐标摆放）" if checked else "关（居中缩放）"))

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
        self._quat_panel.set_quaternion(Quaternion.identity())
        self._deviation.set_value(0.0, 0.0, 0.0, "对准")
        self._log("已将当前姿态设为参考（0°）")

    def calibrate_axis_vertical(self) -> None:
        """Derive the needle axis from one vertical pose and persist it."""

        if self._quat is None:
            self._log("尚无数据，无法标定针轴")
            return
        age = None if self._last_rx is None else time.monotonic() - self._last_rx
        if age is None or age > STALE_AFTER_S:
            self._log("数据已过期，请确认传感器在出数后再标定")
            return
        axis = self._tracker.calibration_from_vertical(self._quat)
        self._rig = replace(self._rig, needle_axis_sensor=axis)
        try:
            self._rig.save()
            saved = "已写入 config/rig.json"
        except OSError as exc:
            saved = f"写入配置失败：{exc}"
        self._chk_flip.setChecked(False)
        self._quat_panel.set_quaternion(Quaternion.identity())
        self._deviation.set_value(0.0, 0.0, 0.0, "对准")
        self._log(
            "针轴标定完成：传感器系 "
            f"({axis[0]:+.3f}, {axis[1]:+.3f}, {axis[2]:+.3f})，{saved}"
        )
        self._log("请倾斜 30° 复核：界面倾角应与实际一致，方向不应左右/前后相反")

    def import_model(self) -> None:
        path, _ = QFileDialog.getOpenFileName(self, "选择模型（STL / STEP / glTF）", "", MODEL_FILTER)
        if not path:
            return
        try:
            mesh = load_model(path)
        except BackendMissing as exc:
            QMessageBox.information(self, "需要安装格式支持", str(exc))
            self._log(f"模型导入失败：缺少格式后端（{path.rsplit('.', 1)[-1]}）")
            return
        except Exception as exc:  # noqa: BLE001 - show the reason to the operator
            QMessageBox.warning(self, "导入失败", str(exc))
            self._log(f"模型导入失败：{exc}")
            return

        original = mesh.size()
        if self._chk_cad.isChecked():
            mesh = mesh.simplified(MODEL_MAX_FACES)
            self._model_mesh = mesh
            self._tracker.set_model(self._rig.model_axis, self._rig.model_right)
            for view in self._views():
                view.set_model(mesh, axis=self._rig.model_axis, hub=self._rig.hub_mm)
                view.fit_distance()
            axis = self._rig.model_axis
            hub = self._rig.hub_mm
            self._log(
                f"已按 CAD 坐标导入 {mesh.name}：{mesh.triangle_count} 面，"
                f"原始 {original[0]:.1f}×{original[1]:.1f}×{original[2]:.1f} mm，"
                f"针轴 ({axis[0]:g},{axis[1]:g},{axis[2]:g})，"
                f"针座 ({hub[0]:g},{hub[1]:g},{hub[2]:g}) mm"
            )
        else:
            mesh = mesh.simplified(MODEL_MAX_FACES).centered(target_max_mm=MODEL_TARGET_MM)
            self._model_mesh = mesh
            # Centred demo meshes are assumed to point down -Z.
            self._tracker.set_model((0.0, 0.0, -1.0), (1.0, 0.0, 0.0))
            for view in self._views():
                view.set_model(mesh)
                view.fit_distance()
            self._log(
                f"已导入模型 {mesh.name}：{mesh.triangle_count} 面，"
                f"原始 {original[0]:.1f}×{original[1]:.1f}×{original[2]:.1f} mm，"
                f"原点置于模型中心"
            )

    def import_base(self) -> None:
        path, _ = QFileDialog.getOpenFileName(self, "选择基座（STL / STEP / glTF）", "", MODEL_FILTER)
        if not path:
            return
        try:
            mesh = load_model(path)
        except BackendMissing as exc:
            QMessageBox.information(self, "需要安装格式支持", str(exc))
            self._log(f"基座导入失败：缺少格式后端（{path.rsplit('.', 1)[-1]}）")
            return
        except Exception as exc:  # noqa: BLE001
            QMessageBox.warning(self, "导入失败", str(exc))
            self._log(f"基座导入失败：{exc}")
            return

        size = mesh.size()
        mesh = mesh.simplified(2500).with_top_at_origin()
        self._base_mesh = mesh
        for view in self._views():
            view.set_base(mesh)
            view.fit_distance()
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
            self._recorder.writerow(
                ["t_wall", "qw", "qx", "qy", "qz", "tilt_deg", "deviation_deg", "phi_deg", "psi_deg"]
            )
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
