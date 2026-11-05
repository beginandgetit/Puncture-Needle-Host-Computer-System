"""Headless smoke test for the prototype GUI.

Uses Qt's offscreen platform so it runs on a build machine without a display:
it starts the real window, feeds it simulated readings through the real worker
thread, and checks that a frame renders.
"""

from __future__ import annotations

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
        finally:
            window.shutdown()

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
