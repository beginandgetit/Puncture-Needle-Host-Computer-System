"""Headless smoke test for the prototype GUI.

Uses Qt's offscreen platform so it runs on a build machine without a display:
it starts the real window, feeds it simulated readings through the real worker
thread, and checks that a frame renders.
"""

from __future__ import annotations

import math
import os
import time
import unittest

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

try:
    from PySide6.QtWidgets import QApplication
except ImportError:  # pragma: no cover - PySide6 is optional
    QApplication = None  # type: ignore[assignment]


@unittest.skipIf(QApplication is None, "PySide6 not installed")
class TestGuiSmoke(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.app = QApplication.instance() or QApplication([])
        from wt901.gui import _prepare_fonts

        _prepare_fonts()

    def _pump(self, seconds: float) -> None:
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            self.app.processEvents()
            time.sleep(0.01)

    def test_window_runs_with_simulated_sensor(self) -> None:
        from wt901 import WT901Config
        from wt901.gui.window import MainWindow
        from wt901.pose import Quaternion
        from wt901.simulator import SimulatedTransport

        cfg = WT901Config(transport=SimulatedTransport(hz=50.0, pitch_amp=5.0, period_s=4.0))
        window = MainWindow(cfg, simulated=True)
        window.show()
        try:
            self._pump(1.6)
            self.assertIsNotNone(window._pose, "no reading reached the window")
            self.assertIsNotNone(window._quat)
            image = window.grab()
            self.assertFalse(image.isNull())
            self.assertGreater(image.width(), 1000)

            # the readout must be a real angle, not the placeholder
            self.assertGreaterEqual(window._pose.tilt_deg, 0.0)
            self.assertLess(window._pose.tilt_deg, 90.0)

            # the view rotates by default (trackball), and the lock is optional
            self.assertFalse(window._scene.view_locked)
            window._chk_lock.setChecked(True)
            self.assertTrue(window._scene.view_locked)
            window._chk_lock.setChecked(False)
            self.assertFalse(window._scene.view_locked)

            # rotating the camera must move the needle on screen too, i.e. the
            # needle and the axes share one rigid transform
            before_tip = window._scene._project((0.0, 0.0, -170.0), 800.0, 600.0)
            before_axis = window._scene._project((80.0, 0.0, 0.0), 800.0, 600.0)
            window._scene._apply_view(
                Quaternion.from_axis_angle((0.0, 1.0, 0.0), 25.0) * window._scene._view_q
            )
            after_tip = window._scene._project((0.0, 0.0, -170.0), 800.0, 600.0)
            after_axis = window._scene._project((80.0, 0.0, 0.0), 800.0, 600.0)
            self.assertGreater(abs(after_tip[0] - before_tip[0]), 20.0)
            self.assertGreater(abs(after_axis[0] - before_axis[0]), 5.0)

            # new layout: log panel has entries and the quaternion is live
            self.assertTrue(window._log_view.toPlainText().strip())
            w_label = window._quat_panel._values["W"].text()
            self.assertNotEqual(w_label, "—")
            self.assertTrue(w_label.startswith(("+", "-")))

            # pose exposes the quaternion for downstream use
            self.assertIsNotNone(window._pose.quat)
            self.assertAlmostEqual(window._pose.quat.norm(), 1.0, places=6)

            # the deviation card shows both components
            self.assertAlmostEqual(
                window._pose.deviation_deg,
                math.hypot(window._pose.deviation_x_deg, window._pose.deviation_y_deg),
                delta=0.2,
            )
            from wt901.gui.deviation import side_text

            self.assertEqual(side_text(2.0, "偏右", "偏左"), "偏右")
            self.assertEqual(side_text(-2.0, "偏右", "偏左"), "偏左")
            self.assertEqual(side_text(0.0, "偏上", "偏下"), "居中")

            # disconnect stops the worker and shows the "not connected" banner
            window.stop_connection()
            self.assertIsNone(window._thread)
            self._pump(0.3)
            self.assertTrue(window._banner.isVisible())

            # pausing freezes the picture instead of aging it into NO DATA
            window._chk_pause.setChecked(True)
            frozen = window._pose
            self._pump(0.4)
            self.assertIs(window._pose, frozen)
            self.assertFalse(window._stale)

            # the needle-axis flip is a one-click field calibration
            before = window._tracker._s
            window._chk_flip.setChecked(True)
            after = window._tracker._s
            self.assertAlmostEqual(after[0], -before[0], places=6)
            self.assertAlmostEqual(after[1], -before[1], places=6)
            self.assertAlmostEqual(after[2], -before[2], places=6)
            self.assertIn("针轴方向", window._log_view.toPlainText())
        finally:
            window.shutdown()

    def test_camera_presets_and_fit(self) -> None:
        from wt901.gui.scene import NeedleView
        from wt901.gui.stl import load_stl

        view = NeedleView()
        view.resize(800, 600)

        # top view: a point on the Z axis lands in the centre
        view.set_camera_preset("top")
        x, y, _ = view._project((0.0, 0.0, -100.0), 800.0, 600.0)
        self.assertAlmostEqual(x, 400.0, delta=1.0)
        self.assertAlmostEqual(y, 300.0, delta=1.0)

        # front view: +X goes right, +Z goes up (smaller screen y)
        view.set_camera_preset("front")
        x_right, _, _ = view._project((50.0, 0.0, 0.0), 800.0, 600.0)
        _, y_up, _ = view._project((0.0, 0.0, 50.0), 800.0, 600.0)
        self.assertGreater(x_right, 420.0)
        self.assertLess(y_up, 280.0)

        # fit_distance frames the content and centres on its bounding box
        mesh = load_stl("examples/body_demo.stl").with_top_at_origin()
        view.set_model(mesh)
        view.fit_distance()
        self.assertLess(view._target[2], 0.0)
        self.assertGreater(view._dist, 500.0)

        # compact views keep a light copy so all three views stay fast
        compact = NeedleView(compact=True)
        compact.set_model(mesh)
        self.assertIsNotNone(compact._model_compact)
        self.assertLessEqual(compact._model_compact.triangle_count, 1200)

    def test_compact_views_ignore_mouse(self) -> None:
        from PySide6.QtCore import QEvent, QPointF, Qt
        from PySide6.QtGui import QMouseEvent

        from wt901.gui.scene import NeedleView

        view = NeedleView(compact=True)
        view.resize(280, 130)
        self.assertTrue(view._compact)

        press = QMouseEvent(
            QEvent.MouseButtonPress,
            QPointF(20.0, 20.0),
            QPointF(20.0, 20.0),
            Qt.LeftButton,
            Qt.LeftButton,
            Qt.NoModifier,
        )
        view.mousePressEvent(press)
        self.assertIsNone(view._drag, "compact view must not start a drag")

        before = view._dist

        class _Wheel:
            def angleDelta(self):  # noqa: N802 - Qt naming
                raise AssertionError("compact view must not zoom")

        view.wheelEvent(_Wheel())  # type: ignore[arg-type]
        self.assertEqual(view._dist, before, "compact view must not zoom")

    def test_stale_state_flags_when_no_data(self) -> None:
        from wt901 import WT901Config
        from wt901.gui.window import MainWindow

        cfg = WT901Config(transport=object(), name_contains="")
        # autostart=False: this test is about the no-data presentation, not
        # about connection handling.
        window = MainWindow(cfg, simulated=True, autostart=False)
        window.show()
        try:
            self._pump(0.4)
            self.assertTrue(window._stale, "window should start in the stale state")
            self.assertTrue(window._banner.isVisible())
        finally:
            window.shutdown()


@unittest.skipIf(QApplication is None, "PySide6 not installed")
class TestGuiQuaternionMapping(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.app = QApplication.instance() or QApplication([])
        from wt901.gui import _prepare_fonts

        _prepare_fonts()

    def test_quaternion_reading_drives_full_model_rotation(self) -> None:
        from wt901 import WT901Config
        from wt901.gui.window import MainWindow
        from wt901.pose import Quaternion

        class Reading:
            t_wall = 0.0
            roll = 0.0
            pitch = 0.0
            yaw = 0.0

        q = Quaternion.from_euler(10, 20, 30)
        reading = Reading()
        reading.quaternion = (q.w, q.x, q.y, q.z)

        window = MainWindow(
            WT901Config(transport=object(), name_contains=""),
            simulated=True,
            autostart=False,
        )
        try:
            window._on_reading(reading)
            self.assertIsNotNone(window._pose)
            self.assertIsNotNone(window._pose.model_rotation)
            # The renderer must receive the full rotation, not just an axis.
            self.assertIsNotNone(window._scene._model_rotation)
            self.assertAlmostEqual(window._scene._model_rotation.norm(), 1.0, places=6)
        finally:
            window.shutdown()

@unittest.skipIf(QApplication is None, "PySide6 not installed")
class TestLatestWins(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.app = QApplication.instance() or QApplication([])
        from wt901.gui import _prepare_fonts

        _prepare_fonts()

    def test_poll_drops_intermediate_samples(self) -> None:
        from types import SimpleNamespace

        from wt901 import WT901Config
        from wt901.gui.window import MainWindow

        class FakeWorker:
            def __init__(self) -> None:
                self.value = None

            def latest(self):
                return self.value

        def reading(seq: int):
            return SimpleNamespace(
                seq=seq,
                t_mono=1.0 + seq * 0.01,
                t_wall=0.0,
                roll=0.0,
                pitch=0.0,
                yaw=0.0,
                qw=1.0,
                qx=0.0,
                qy=0.0,
                qz=0.0,
            )

        window = MainWindow(
            WT901Config(transport=object(), name_contains=""),
            simulated=True,
            autostart=False,
        )
        try:
            worker = FakeWorker()
            window._worker = worker
            worker.value = reading(1)
            window._poll_reading()
            worker.value = reading(5)
            window._poll_reading()
            self.assertEqual(window._dropped, 3, "samples 2..4 must be counted as dropped")
            self.assertEqual(window._last_seq, 5)
            self.assertIsNotNone(window._pose)
        finally:
            window.shutdown()