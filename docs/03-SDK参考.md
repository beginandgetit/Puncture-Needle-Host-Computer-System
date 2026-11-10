# SDK 参考

面向接入方：一个 `WT901Config` + 一个 `with` 语句就能拿数据；需要什么再往下翻。

```python
from wt901 import WT901, WT901Config

cfg = WT901Config(address="AA:BB:CC:DD:EE:FF")   # 见 `python -m wt901 scan`
with WT901(cfg) as imu:
    r = imu.read(timeout=2.0)
    print(r.roll, r.pitch, r.yaw, r.ax, r.gy)
```

---

## 1. 安装

```powershell
cd wt901_control
python -m venv .venv
.\.venv\Scripts\python.exe -m pip install -e ".[ble]"
```

如果只是把代码放进现有工程，把 `src/wt901` 复制过去、把 `src` 加入 `sys.path` 即可；
BLE 那部分才需要 `bleak`，解析、模拟、离线回放都不需要。

## 2. 两种用法

| | `WT901`（同步） | `AsyncWT901`（异步） |
| --- | --- | --- |
| 适用 | 普通产品主循环、Qt/MFC 工作线程、脚本 | 已有 asyncio 服务、需要并发多设备 |
| 阻塞 | `read()` 阻塞调用线程 | `await read()` |
| 实现 | 内部起一个线程跑事件循环，包装 `AsyncWT901` | 直接使用 |
| 能力 | 完全一致（重连、校准、状态、监听器） | 完全一致 |

## 3. 配置：`WT901Config`

| 字段 | 默认 | 说明 |
| --- | --- | --- |
| `address` | `None` | BLE 地址，**推荐**。与 `name_contains` 二选一 |
| `name_contains` | `"WT901"` | 按名称发现；同时给 address 时作为兜底 |
| `notify_uuid` | `None` | 固定数据特征值，`probe` 会告诉你 |
| `write_uuid` | `None` | 写入特征值，只有要发寄存器命令时才需要 |
| `profile` | `None` | 已知 GATT 布局，见 `wt901.ble.WITMOTION_BLE` |
| `emit_on` | `"attitude"` | `attitude`=每帧姿态角或四元数产出一条；`any`=任意帧都产出 |
| `scales` | `Scales(16, 2000, 180)` | 量程，必须与设备一致 |
| `combined_checksum` | `False` | 20 字节合并帧若实际 21 字节则打开 |
| `reconnect` | `True` | 断线自动重连 |
| `reconnect_delay_s` | `2.0` | 首次重连等待 |
| `reconnect_max_s` | `15.0` | 退避上限（每次 ×1.6） |
| `connect_timeout_s` | `25.0` | `with`/`start` 等待首帧的时间（扫描+连接在 Windows 约 10 s） |
| `scan_timeout_s` | `8.0` | 扫描时长；GUI 用 6 s |
| `queue_size` | `256` | 读取队列；满了丢最旧 |
| `transport` | `None` | 注入自定义数据源（测试/回放），生产留空 |

### 2.1 设备 profile 与缓存

`wt901.ble.WITMOTION_BLE` 固化了本机实测的 GATT 布局（服务 `ffe5`／通知 `ffe4`／写 `ffe9`），
把它传给 `profile=` 后，特征值选择不再靠猜：

```python
from wt901.ble import WITMOTION_BLE
from wt901 import WT901, WT901Config

cfg = WT901Config(name_contains="WT901BLE67", profile=WITMOTION_BLE)
```

设备缓存 `wt901.devices`（`%LOCALAPPDATA%\wt901-control\last_device.json`）记录上次成功的
地址，下次启动直接连、失败再按名字扫描。删除该文件即可强制重新发现。


## 4. 数据：`ImuReading`

冻结 dataclass，单位固定：角度 `deg`、加速度 `g`、角速度 `deg/s`、温度 `degC`。

| 字段 | 说明 |
| --- | --- |
| `t_mono` | 采样时刻（`time.monotonic()`，用于超时判断） |
| `t_wall` | 墙钟时间（用于与其他系统对时/落盘） |
| `seq` | 递增序号，可用来判断"有没有新数据" |
| `roll / pitch / yaw` | 姿态角，已应用角度零点 |
| `qw / qx / qy / qz` | 设备四元数（传感器系，Hamilton，`w/x/y/z`）；设备只发欧拉角时为 `None` |
| `ax / ay / az` | 加速度 |
| `gx / gy / gz` | 角速度，已应用零偏 |
| `temp_c` | 温度，设备不提供时为 `None` |
| `has_attitude` | 设备是否真的输出了姿态角（`emit_on="any"` 且只有加速度帧时为 `False`） |

属性/方法：`has_quaternion`、`quaternion`（返回 `(w,x,y,z)` 或 `None`）、
`as_dict()`、`to_json()`、`str()`。

## 5. 方法与状态

### `WT901` / `AsyncWT901` 共有能力

| 方法 | 说明 |
| --- | --- |
| `start(wait=True)`（同步） | 启动；`wait=True` 时等到首帧，失败抛 `ConnectionTimeout` 并自动停止 |
| `await start()`（异步） | 立即返回；用 `await wait_connected(t)` 或 `async with` 等首帧 |
| `stop()` / `await stop()` | 停止、取消重连、关闭连接；可重复调用 |
| `read(timeout=None)` | 取下一条；超时抛 `NoDataError`；`None` 表示一直等（`stop()` 会唤醒） |
| `latest` | 最近一条，不阻塞；无数据时 `None` |
| `stream()` | 迭代器；`stop()` 后结束 |
| `status` | `SensorStatus` 快照：状态、帧数、坏帧、速率、数据年龄、重连次数 |
| `state` | `LinkState` 枚举 |
| `add_listener(cb)` / `remove_listener(cb)` | 每帧回调（在采集线程执行） |
| `on_error(cb)` | 链路错误/监听器异常回调 |
| `calibrate_gyro_bias(duration)` | 静止时求零偏 |
| `zero_here(include_yaw=False)` | 当前姿态作为零点 |
| `set_gyro_bias` / `set_angle_offset` / `reset_calibration` | 手工设定 |
| `save_calibration(path)` / `load_calibration(path)` | JSON 持久化 |

### `SensorStatus`

```python
status = imu.status
status.state          # LinkState.CONNECTED
status.frames         # 累计帧数
status.rate_hz        # 最近 1 秒的速率
status.last_reading_age_s
status.reconnects
status.bad_checksum / bad_header   # 链路质量
```

### 错误

| 异常 | 触发场景 | 建议处理 |
| --- | --- | --- |
| `ConfigError` | 配置非法 | 启动前修配置 |
| `LinkError` | 连接失败 | 提示用户检查设备 |
| `ConnectionTimeout` | 等了 `connect_timeout_s` 没有首帧 | 见第 10 节排查 |
| `NoDataError` | `read()` 超时 | 进入安全状态 |
| `NotStartedError` | 未 `start()` 就调用 | 先启动 |
| `CalibrationError` | 采样不足/设备在动 | 静止后重试 |
| `WT901Error` | 以上所有异常的基类 | 统一兜底 |

## 6. 线程模型（重要）

- `WT901` 内部有一个线程和一个事件循环；`read()`、`start()`、`stop()` 可以从任意线程调用。
- **监听器回调运行在采集线程**，不要在里面做耗时操作，也不要调用 `stop()`（会死锁，会直接抛 `WT901Error`）。
- 需要更新 UI 时，把回调改成投递到 UI 队列（Qt 的 `signal.emit`、`queue.Queue` 都可以）。
- `AsyncWT901` 只在创建它的事件循环里使用。

```python
with WT901(cfg) as imu:
    imu.add_listener(lambda r: ui_queue.put(r.as_dict()))   # 线程安全队列
    ...
```

## 7. 自动重连语义

```
CONNECTING → CONNECTED → (断链) → RECONNECTING → CONNECTED
                                 ↘ stop() → STOPPED
```

- 断链瞬间 `latest` 置空、队列清空，**不会**把旧数据当成新数据。
- 重连间隔 2 s 起，每次 ×1.6，上限 15 s。
- `status.reconnects` 可用来做告警；连续重连失败不抛异常，只有 `on_error` 会收到通知。
- 产品侧仍应自己做"多久没数据就停机"的判断，SDK 只保证数据不会假装是新的。

## 8. 校准

```python
with WT901(cfg) as imu:
    imu.calibrate_gyro_bias(2.0)      # 静止 2 秒
    imu.zero_here(include_yaw=False)  # 把当前姿态当 0°
    imu.save_calibration("cal.json")  # 下次 load_calibration 直接复用
```

- 零偏必须**静止**采集；有振动会污染结果。
- `zero_here` 只改角度显示，不改原始数据。
- 校准值同时影响 `read()` / `latest` / `stream()`，不影响 `status` 里的帧计数。

## 9. 三种典型集成形态

**A. 固定周期轮询**（最常见，见 `examples/product_loop.py`）

```python
while running:
    try:
        r = imu.read(timeout=0.2)
        do_work(r)
    except NoDataError:
        safe_state()
    time.sleep(max(0, period - elapsed))
```

**B. 事件驱动**（UI、报警器）

```python
imu.add_listener(on_reading)   # 每帧触发；内部要快
```

**C. asyncio 服务**（多设备/网络）

```python
async with AsyncWT901(cfg) as imu:
    async for r in imu.stream():
        await publish(r)
```

**与控制逻辑组合**（见 `examples/control_demo.py`）：

```python
from wt901 import AttitudeController, ControlConfig
from wt901.state import Snapshot

ctrl = AttitudeController(ControlConfig(axis="pitch", threshold_deg=15, hysteresis_deg=3))
cmd = ctrl.step(Snapshot.from_reading(reading), time.monotonic())
```

## 10. 连不上 / 没数据怎么查

先跑 `python -m wt901 probe`，它会打印服务列表、每个特征值收到多少字节、帧格式和速率。

| 现象 | 原因 | 处理 |
| --- | --- | --- |
| `scan` 找不到 | 未广播/已连接别处 | 关手机 App、重新上电 |
| 扫描列表里没有、但 Windows 已配对 | 厂商软件可能把它切换成了 SPP/虚拟串口模式 | 关掉厂商软件、重新上电；用 `probe --address` 直接尝试连接；若仍不行，按串口方案处理 |
| 连上 0 字节 | 数据不在这个特征值 / 设备需使能 | `probe` 看哪个特征值有流量；仍为 0 则用厂商上位机确认输出配置 |
| `bad_checksum` 涨 | 帧长或校验假设错 | `analyze` 看 `frames by id`；试 `combined_checksum=True` |
| `az` 不是 1 g | 量程不符 | 改 `Scales(accel_g=...)` |
| 频繁断连 | 距离/干扰/USB 省电 | 缩短距离、关 USB 选择性暂停 |

## 11. 性能与限制

- 控制/读取线程不要做重活；`read()` 间隔建议 ≥ 10 ms。
- Windows 定时器粒度约 15.6 ms，`asyncio.sleep` 的实际步进会大于 1 ms。
- 单实例一个设备；多设备请每台一个 `WT901`（各自线程），或在一个 asyncio 循环里多个 `AsyncWT901`。
- SDK 不做姿态融合：优先使用设备 0x59 四元数，只有欧拉角帧时退回设备角度
  （默认 ZYX 假设）。针体界面再通过 `mapping.py` 做传感器↔模型映射。
- 设备配置写入（改 ODR、改回传内容）尚未实现，目前依赖设备当前配置。

## 12. 版本

`0.2.0`：新增 SDK（`WT901` / `AsyncWT901`）、`probe`、校准、`py.typed`。
后续若要改 API，按语义化版本走，`0.x` 期间以本文档为准。
