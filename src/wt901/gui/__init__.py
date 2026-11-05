"""Prototype GUI for needle angle guidance.

Run it with::

    python -m wt901.gui --sim
    python -m wt901.gui --address AA:BB:CC:DD:EE:FF

The GUI is intentionally dependency-light: PySide6 only, no OpenGL/VTK.
"""

from __future__ import annotations

import sys
from typing import Optional


def run_gui(
    *,
    address: Optional[str] = None,
    name: str = "WT901",
    sim: bool = False,
    screenshot: Optional[str] = None,
    screenshot_delay: float = 2.5,
    seconds: float = 0.0,
    sim_amp: float = 18.0,
    sim_period: float = 6.0,
) -> int:
    from PySide6.QtCore import QTimer
    from PySide6.QtWidgets import QApplication

    from ..sensor import WT901Config
    from . import theme
    from .window import MainWindow

    app = QApplication.instance() or QApplication(sys.argv[:1])
    app.setStyle("Fusion")
    _prepare_fonts()
    app.setStyleSheet(theme.stylesheet())

    if sim:
        from ..simulator import SimulatedTransport

        config = WT901Config(
            transport=SimulatedTransport(hz=60.0, pitch_amp=sim_amp, period_s=sim_period)
        )
    else:
        from ..ble import WITMOTION_BLE
        from ..devices import load_last_device

        # Remember the last working address: direct connect is much faster than
        # scanning every launch; the name is kept as the fallback.
        cached = load_last_device() if address is None else None
        config = WT901Config(
            address=address or (cached.address if cached else None),
            name_contains=name,
            profile=WITMOTION_BLE,  # FFE5/FFE4/FFE9, verified on WT901BLE67
            scan_timeout_s=6.0,
        )

    window = MainWindow(config, simulated=sim)
    window.show()

    if screenshot:
        def shoot() -> None:
            ok = window.capture_screenshot(screenshot)
            print(f"screenshot -> {screenshot} ({'ok' if ok else 'failed'})", flush=True)
            app.quit()

        QTimer.singleShot(int(screenshot_delay * 1000), shoot)

    if seconds > 0:
        QTimer.singleShot(int(seconds * 1000), app.quit)

    try:
        return app.exec()
    finally:
        window.shutdown()


__all__ = ["run_gui"]


def _prepare_fonts() -> None:
    """Register system CJK fonts and pick the first available family.

    Needed for the offscreen platform used by the screenshot self-test, where
    Qt ships no fonts at all; on a normal Windows desktop this is a no-op that
    simply resolves "Microsoft YaHei UI".
    """

    import os

    from PySide6.QtGui import QFontDatabase

    from . import theme

    candidates = [
        r"C:\Windows\Fonts\msyh.ttc",
        r"C:\Windows\Fonts\msyhbd.ttc",
        r"C:\Windows\Fonts\simhei.ttf",
        r"C:\Windows\Fonts\segoeui.ttf",
        r"C:\Windows\Fonts\consola.ttf",
        "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf",
        "/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc",
    ]
    for path in candidates:
        if os.path.exists(path):
            QFontDatabase.addApplicationFont(path)

    families = set(QFontDatabase.families())
    for family in ("Microsoft YaHei UI", "Microsoft YaHei", "SimHei", "Segoe UI", "DejaVu Sans"):
        if family in families:
            theme.FONT_UI = family
            break
    for family in ("Consolas", "Cascadia Mono", "DejaVu Sans Mono", "Courier New"):
        if family in families:
            theme.FONT_MONO = family
            break
