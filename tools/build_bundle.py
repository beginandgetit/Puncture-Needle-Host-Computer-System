"""Assemble the delivery folder: source zip + README files.

    python tools/build_bundle.py
"""

from __future__ import annotations

import zipfile
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
DIST = ROOT / "dist"
VERSION = "0.5.1"
PREFIX = f"wt901-control-{VERSION}"

INCLUDE_GLOBS = [
    "src/**/*.py",
    "src/wt901/py.typed",
    "examples/*.py",
    "examples/*.stl",
    "examples/*.md",
    "config/*.json",
    "examples/*.glb",
    "tools/*.py",
    "tests/*.py",
    "docs/**/*.md",
    "docs/**/*.png",
    "docs/*.bin",
    "README.md",
    "pyproject.toml",
    ".gitignore",
    "*.cmd",
]

EXCLUDE_PARTS = {"__pycache__", ".venv", ".venv_pkgtest", "dist", ".pytest_cache"}


def _iter_files():
    seen = set()
    for pattern in INCLUDE_GLOBS:
        for path in ROOT.glob(pattern):
            if not path.is_file():
                continue
            if any(part in EXCLUDE_PARTS for part in path.parts):
                continue
            if path.suffix == ".pyc":
                continue
            if path in seen:
                continue
            seen.add(path)
            yield path


def build_zip(target: Path) -> int:
    count = 0
    with zipfile.ZipFile(target, "w", compression=zipfile.ZIP_DEFLATED) as zf:
        for path in sorted(_iter_files()):
            zf.write(path, f"{PREFIX}/{path.relative_to(ROOT).as_posix()}")
            count += 1
    return count


DELIVERY_README = f"""# 交付说明 · wt901-control {VERSION}

日期：2026-11-05
适用设备：维特智能 **WT901BLE67**（WT9011DCL-BT50 系列 BLE IMU）

---

## 一、包内容

| 文件 | 说明 |
| --- | --- |
| `wt901_control-{VERSION}-py3-none-any.whl` | **SDK 安装包**（推荐），含采集、解析、诊断、GUI |
| `wt901_control-{VERSION}.tar.gz` | 源码分发包（sdist） |
| `wt901-sdk-{VERSION}-source.zip` | 完整工程：源码 + 示例 + 工具 + 测试 + 文档 + 一键启动脚本 |
| `数据参数说明书.docx` | 通信协议与数据参数（实测），可直接给同事/客户 |
| `测试数据/` | 真实抓包 + 合成数据 + 黄金帧 + 统计 |

## 二、安装与验证

### 从 wheel 安装（需要 BLE 时加 `[ble]`，需要界面时加 `[gui]`）

```powershell
python -m pip install "wt901_control-{VERSION}-py3-none-any.whl[ble]"
python -m pip install PySide6          # 仅在需要图形界面时
python -m pip install cascadio         # 仅在需要导入 STEP/STP 模型时
python -m pip install trimesh          # 仅在需要导入 glTF/GLB 模型时
```

已验证：干净虚拟环境安装后 `wt901 --version` 输出 `wt901-control {VERSION}`，
`wt901 selftest` 五项全 PASS。

### 源码工程（推荐现场调试）

解压 `wt901-sdk-{VERSION}-source.zip`，双击 `start_real.cmd`（连设备）或
`start_gui.cmd`（模拟）。启动器自动设置 `PYTHONPATH`，无需安装。

## 三、五分钟自检

```powershell
wt901 selftest
wt901 analyze --file 测试数据\\real_static_6s.bin
wt901 gui --sim
```

## 四、主要接口

```python
from wt901 import WT901, WT901Config
from wt901.ble import WITMOTION_BLE

cfg = WT901Config(name_contains="WT901BLE67", profile=WITMOTION_BLE)
with WT901(cfg) as imu:
    reading = imu.read(timeout=10)
    print(reading.roll, reading.pitch, reading.yaw, reading.ax, reading.gy)
```

| 对象 | 用途 |
| --- | --- |
| `WT901` / `AsyncWT901` | 阻塞 / asyncio 采集句柄（自动重连、校准） |
| `WT901Config` | 设备、profile、超时、量程等配置 |
| `ImuReading` | 单帧物理量（角度 / 加速度 / 角速度 / 温度） |
| `BleProfile` / `WITMOTION_BLE` | 固化 GATT 布局（服务 ffe5 / 通知 ffe4 / 写入 ffe9） |
| `wit_frame()` | 构造 FF AA 寄存器命令 |
| `NeedleTracker` | 姿态 → 针轴方向 → 倾角/偏差（角度引导） |
| `analyze_capture()` | 离线解析抓包 |
| `wt901 gui` | PySide6 角度引导界面 |

## 五、已知限制

- 传感器同一时刻只能被一个中心设备连接；维特上位机或手机 App 连接时，本程序连不上（会持续重试，关闭后自动恢复）。
- 出厂默认 10 Hz；更高频率需改设备输出速率并复测。
- `0x55 0x61` 帧无校验和，只能做量程合理性检查。

详见 `数据参数说明书.docx` 与工程内 `docs/` 目录。
"""

TESTDATA_README = """# 测试数据说明

全部数据由 `tools/make_testdata.py` 生成，可复现。

| 文件 | 内容 | 规模 |
| --- | --- | --- |
| `real_static_6s.bin` | 真实设备静态抓包（原始 BLE 通知字节） | 1200 B / 60 帧 / 6 s |
| `real_static_6s.csv` | 解码后的物理量（角度/加速度/角速度/模长） | 60 行 |
| `real_static_6s_stats.json` | 统计结果（均值、标准差、坏帧） | — |
| `synthetic_motion_30s.bin` | 合成摆动数据：roll ±6°（8 s）、pitch ±12°（12 s） | 60 KB / 3000 帧 / 30 s |
| `synthetic_motion_30s.csv` | 同上，解码结果 | 3000 行 |
| `golden_frames.json` | 5 组黄金帧 + 期望值（容差 0.05），用于协议回归 | — |

## 真实数据统计

- 60 帧 / 6.0 s / 10 Hz，坏帧 0
- Roll：-0.496° ± 0.381°
- Pitch：0.285° ± 0.150°
- |accel|：1.0029 g ± 0.0007 g（静置，验证量程 ±16 g）

## CSV 列

`index, t_est_s, roll_deg, pitch_deg, yaw_deg, ax_g, ay_g, az_g, gx_dps, gy_dps, gz_dps, accel_norm_g`

> `t_est_s` 是按 10 Hz 由帧序号推算的时间（抓包文件本身不含时间戳）。

## 复现/扩展

```powershell
python tools\\make_testdata.py --real docs\\capture_wt901ble67.bin --out 测试数据
```

采集新的真实数据：

```powershell
wt901 run --name WT901BLE67 --capture new.bin --duration 60 --quiet
```
"""


def main() -> int:
    DIST.mkdir(parents=True, exist_ok=True)
    target = DIST / f"wt901-sdk-{VERSION}-source.zip"
    count = build_zip(target)
    (DIST / "README_交付说明.md").write_text(DELIVERY_README, encoding="utf-8")
    (DIST / "测试数据" / "README.md").write_text(TESTDATA_README, encoding="utf-8")
    print(f"zip: {target.name} ({count} files, {target.stat().st_size} bytes)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
