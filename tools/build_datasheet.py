"""Build the datasheet: WT901BLE67 通信协议与数据参数说明书 (DOCX).

Run with the bundled runtime:
    <dsh python> tools/build_datasheet.py --stats dist/测试数据/real_static_6s_stats.json --out dist/数据参数说明书.docx
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path

from docx import Document
from docx.enum.table import WD_TABLE_ALIGNMENT
from docx.enum.text import WD_ALIGN_PARAGRAPH
from docx.oxml.ns import qn
from docx.shared import Cm, Pt, RGBColor

CN_FONT = "微软雅黑"
MONO = "Consolas"
ACCENT = RGBColor(0x1F, 0x4E, 0x79)
MUTED = RGBColor(0x59, 0x59, 0x59)


def _font(run, name=CN_FONT, size=None, bold=None, color=None):
    run.font.name = name
    rpr = run._element.get_or_add_rPr()
    rfonts = rpr.find(qn("w:rFonts"))
    if rfonts is None:
        rfonts = rpr.makeelement(qn("w:rFonts"), {})
        rpr.append(rfonts)
    rfonts.set(qn("w:eastAsia"), name)
    rfonts.set(qn("w:ascii"), name)
    rfonts.set(qn("w:hAnsi"), name)
    if size is not None:
        run.font.size = Pt(size)
    if bold is not None:
        run.bold = bold
    if color is not None:
        run.font.color.rgb = color
    return run


def body(doc, text, *, size=10.5, bold=False, space_after=6, color=None, align=None):
    p = doc.add_paragraph()
    p.paragraph_format.space_after = Pt(space_after)
    p.paragraph_format.line_spacing = 1.25
    if align is not None:
        p.alignment = align
    _font(p.add_run(text), size=size, bold=bold, color=color)
    return p


def bullet(doc, text, *, size=10.5):
    p = doc.add_paragraph(style="List Bullet")
    p.paragraph_format.space_after = Pt(3)
    _font(p.add_run(text), size=size)
    return p


def code(doc, text):
    p = doc.add_paragraph()
    p.paragraph_format.left_indent = Cm(0.4)
    p.paragraph_format.space_after = Pt(6)
    run = p.add_run(text)
    run.font.name = MONO
    rpr = run._element.get_or_add_rPr()
    rfonts = rpr.find(qn("w:rFonts"))
    if rfonts is None:
        rfonts = rpr.makeelement(qn("w:rFonts"), {})
        rpr.append(rfonts)
    rfonts.set(qn("w:eastAsia"), MONO)
    run.font.size = Pt(9.5)
    return p


def heading(doc, text, level=1):
    h = doc.add_heading(level=level)
    h.paragraph_format.space_before = Pt(14 if level == 1 else 10)
    h.paragraph_format.space_after = Pt(6)
    _font(h.add_run(text), size=15 if level == 1 else 12.5, bold=True, color=ACCENT)
    return h


def table(doc, headers, rows, widths=None):
    t = doc.add_table(rows=1, cols=len(headers))
    t.style = "Table Grid"
    t.alignment = WD_TABLE_ALIGNMENT.CENTER
    hdr = t.rows[0].cells
    for i, h in enumerate(headers):
        hdr[i].text = ""
        p = hdr[i].paragraphs[0]
        _font(p.add_run(h), size=9.5, bold=True)
    for row in rows:
        cells = t.add_row().cells
        for i, value in enumerate(row):
            cells[i].text = ""
            p = cells[i].paragraphs[0]
            _font(p.add_run(str(value)), size=9)
    if widths:
        # A4 with 2.4 cm margins leaves ~16.2 cm; scale so tables never overflow.
        total = sum(widths)
        scale = 15.8 / total if total > 15.8 else 1.0
        scaled = [w * scale for w in widths]
        for row in t.rows:
            for i, w in enumerate(scaled):
                row.cells[i].width = Cm(w)
    doc.add_paragraph().paragraph_format.space_after = Pt(2)
    return t


def build(stats: dict, out_path: Path) -> None:
    doc = Document()

    normal = doc.styles["Normal"]
    normal.font.name = CN_FONT
    normal.font.size = Pt(10.5)
    normal.element.rPr.rFonts.set(qn("w:eastAsia"), CN_FONT)

    section = doc.sections[0]
    section.top_margin = Cm(2.2)
    section.bottom_margin = Cm(2.2)
    section.left_margin = Cm(2.4)
    section.right_margin = Cm(2.4)

    title = doc.add_paragraph()
    title.alignment = WD_ALIGN_PARAGRAPH.CENTER
    _font(title.add_run("WT901BLE67 通信协议与数据参数说明书"), size=20, bold=True, color=ACCENT)

    sub = doc.add_paragraph()
    sub.alignment = WD_ALIGN_PARAGRAPH.CENTER
    _font(sub.add_run("维特智能 BLE IMU · 实测参数 · 测试数据说明"), size=10.5, color=MUTED)

    meta = doc.add_paragraph()
    meta.alignment = WD_ALIGN_PARAGRAPH.CENTER
    _font(meta.add_run("版本 1.0    2026-11-05    适用型号：WT901BLE67（WT9011DCL-BT50 系列）"), size=9, color=MUTED)

    heading(doc, "1. 概述")
    body(
        doc,
        "本文档记录 WT901BLE67 蓝牙姿态传感器在本机 Windows 环境下的实测通信参数，"
        "包括 GATT 结构、数据帧格式、采样特性、连接时序与写入协议，并说明随附测试数据的组织方式。"
        "文中所有数值均来自 2026-11-05 的实测，而非厂商手册的转述；与手册不一致处，以实测为准并在文中标注。",
    )

    heading(doc, "2. 连接与配对")
    bullet(doc, "配对方式：Windows「设置 → 蓝牙和其他设备 → 添加设备」，配对一次即可，之后程序通过 BLE GATT 连接。")
    bullet(doc, "同一时刻只允许一个中心设备连接。维特上位机、手机 App 连接期间，传感器会停止广播，Windows 甚至无法枚举到该设备。")
    bullet(doc, "出现「一直连接中」时，先关闭 WitMotion.exe 或手机 App；本程序会自动重连，无需重启。")
    code(doc, "Get-PnpDevice -Class Bluetooth | Where-Object { $_.FriendlyName -match 'WT901' }")
    body(doc, "实测占用排查：在上位机运行期间，bleak 直接报 BleakDeviceNotFoundError，扫描列表中也找不到该设备；关闭上位机后约 10–30 秒恢复。", size=9.5, color=MUTED)

    heading(doc, "3. GATT 服务与特征值")
    table(
        doc,
        ["角色", "UUID", "属性", "说明"],
        [
            ["服务", "0000ffe5-0000-1000-8000-00805f9a34fb", "—", "维特私有数据服务"],
            ["数据通知", "0000ffe4-0000-1000-8000-00805f9a34fb", "notify", "唯一的传感器数据源"],
            ["命令写入", "0000ffe9-0000-1000-8000-00805f9a34fb", "write / write-without-response", "FF AA 寄存器命令"],
            ["设备名", "00002a00", "read / write", "Generic Access"],
            ["首选连接参数", "00002a04", "read", "Generic Access"],
        ],
        widths=[2.4, 8.6, 3.4, 3.6],
    )
    body(
        doc,
        "注意：UUID 基址为 ...9a34fb（维特自定义），不是标准的 ...9b34fb。"
        "程序按 16 位短 UUID（ffe4）匹配，两种基址都能识别。",
        size=9.5,
        color=MUTED,
    )

    heading(doc, "4. 数据帧格式")
    body(doc, "当前固件输出 0x55 0x61 合并帧，固定 20 字节，9 个 int16 小端，无校验字节。")
    table(
        doc,
        ["偏移", "长度", "内容", "换算"],
        [
            ["0", "1", "0x55 帧头", "—"],
            ["1", "1", "0x61 类型", "—"],
            ["2–7", "6", "AX / AY / AZ", "raw / 32768 × 16  （g）"],
            ["8–13", "6", "GX / GY / GZ", "raw / 32768 × 2000（°/s）"],
            ["14–19", "6", "Roll / Pitch / Yaw", "raw / 32768 × 180（°）"],
        ],
        widths=[2.0, 1.8, 5.4, 8.8],
    )
    body(doc, "因为 2 + 9×2 = 20 正好占满，没有校验字节，只能做量程合理性检查，无法逐帧校验。")
    body(doc, "示例帧（偏航 -170°、俯仰 18°）：", size=9.5, color=MUTED)
    code(doc, "55 61 CD 00 66 FE C3 07 10 00 21 00 31 00 D8 02 CD 0C 1C 87")

    heading(doc, "5. 采样与量程（实测）")
    table(
        doc,
        ["参数", "实测值", "备注"],
        [
            ["输出速率", "10 Hz", "出厂默认；6 s 收到 60 帧"],
            ["单帧长度", "20 字节", "带宽约 200 B/s"],
            ["加速度量程", "±16 g", "静置 |a| = 1.003 g"],
            ["角速度量程", "±2000 °/s", "分辨率 0.061 °/s/LSB"],
            ["角度量程", "±180°", "欧拉角，存在 ±180 绕回"],
            ["坏帧", "0 / 60", "6 s 抓包内无坏帧"],
        ],
        widths=[3.6, 4.0, 9.0],
    )

    heading(doc, "6. 连接时序（Windows 实测）")
    table(
        doc,
        ["阶段", "耗时", "说明"],
        [
            ["过滤扫描", "2.5–6 s", "设备广播间隔决定"],
            ["连接（设备对象）", "≈5 s", "首选路径"],
            ["连接（仅地址）", "≈11 s", "兜底路径，WinRT 需重新解析"],
            ["首次启动到出数据", "10–13 s", "扫描 + 连接"],
            ["断线重连", "2 s 起，×1.6，上限 15 s", "自动重试"],
        ],
        widths=[4.6, 5.2, 6.4],
    )

    heading(doc, "7. 写入协议（FF AA）")
    body(doc, "寄存器写入采用维特标准帧，用于修改输出速率、输出内容等。当前版本只提供帧构造，不自动写设备。")
    code(doc, "FF AA <reg> <data_l> <data_h> <sum>        sum = (FF+AA+reg+data_l+data_h) & 0xFF")
    table(
        doc,
        ["用途", "帧", "说明"],
        [
            ["写寄存器", "FF AA reg lo hi sum", "reg 与取值含义以官方手册为准"],
            ["保存设置", "FF AA 00 00 00 00", "写入后需保存"],
            ["解锁/唤醒", "FF AA 69 88 B5", "部分模块需要，本机不需要"],
        ],
        widths=[3.4, 6.2, 8.4],
    )
    body(doc, "建议：先用厂商上位机修改并观察效果，再用 probe 复测，确认语义后再由程序写入。", size=9.5, color=MUTED)

    heading(doc, "8. 随附测试数据")
    table(
        doc,
        ["文件", "内容", "规模"],
        [
            ["real_static_6s.bin", "真实设备静态抓包（原始字节）", "1200 B / 60 帧 / 6 s"],
            ["real_static_6s.csv", "同上，解码后的物理量", "60 行"],
            ["real_static_6s_stats.json", "统计结果", "见下表"],
            ["synthetic_motion_30s.bin", "合成摆动数据（roll ±6°、pitch ±12°）", "60 KB / 3000 帧 / 30 s"],
            ["synthetic_motion_30s.csv", "同上，解码结果", "3000 行"],
            ["golden_frames.json", "5 组黄金帧与期望值（容差 0.05）", "—"],
        ],
        widths=[6.2, 8.0, 5.0],
    )
    body(doc, "真实静态数据统计：", size=9.5, color=MUTED)
    table(
        doc,
        ["指标", "数值"],
        [
            ["帧数 / 时长 / 速率", f'{stats.get("frames")} 帧 / {stats.get("duration_s")} s / {stats.get("rate_hz")} Hz'],
            ["Roll 均值 ± 标准差", f'{stats.get("roll_mean_deg")}° ± {stats.get("roll_std_deg")}°'],
            ["Pitch 均值 ± 标准差", f'{stats.get("pitch_mean_deg")}° ± {stats.get("pitch_std_deg")}°'],
            ["加速度模长", f'{stats.get("accel_norm_mean_g")} g ± {stats.get("accel_norm_std_g")} g'],
            ["坏帧", f'{stats.get("bad_checksum")} 校验错 / {stats.get("bad_header")} 帧头错'],
        ],
        widths=[6.0, 9.0],
    )
    body(doc, "复现命令：python -m wt901 analyze --file dist/测试数据/real_static_6s.bin", size=9.5, color=MUTED)

    heading(doc, "9. 已知限制")
    bullet(doc, "单连接：上位机/手机 App 连接时，本设备无法同时接入，且停止广播。")
    bullet(doc, "10 Hz 偏低：界面依赖平滑；如需更高频率，需修改设备输出速率并复测。")
    bullet(doc, "地址可能变化：重新配对或更换适配器后地址可能改变，程序会回落到按名称扫描。")
    bullet(doc, "无校验帧：偶发误码只能依赖量程检查和后续滤波。")
    bullet(doc, "Windows 蓝牙省电策略可能导致断连，可在设备管理器中关闭「允许计算机关闭此设备以节约电源」。")

    heading(doc, "附录 A 参数速查")
    table(
        doc,
        ["项", "值"],
        [
            ["设备名 / 地址", "WT901BLE67 / EA:D7:22:40:D3:0A"],
            ["服务 / 通知 / 写入", "ffe5 / ffe4 / ffe9（基址 ...9a34fb）"],
            ["帧格式", "0x55 0x61，20 字节，无校验"],
            ["速率 / 带宽", "10 Hz / ≈200 B/s"],
            ["量程", "±16 g / ±2000 °/s / ±180°"],
            ["是否需要解锁", "不需要"],
        ],
        widths=[5.0, 10.0],
    )

    heading(doc, "附录 B 常用命令")
    code(
        doc,
        "wt901.cmd scan                              列出附近设备\n"
        "wt901.cmd probe --name WT901BLE67 --duration 6   GATT + 帧格式 + 速率\n"
        "wt901.cmd analyze --file capture.bin        离线解析抓包\n"
        "start_real.cmd                             启动角度引导界面",
    )

    doc.core_properties.title = "WT901BLE67 通信协议与数据参数说明书"
    doc.core_properties.author = "wt901-control"
    doc.core_properties.subject = "BLE IMU 通信参数与测试数据"
    out_path.parent.mkdir(parents=True, exist_ok=True)
    doc.save(out_path)


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--stats", default="dist/测试数据/real_static_6s_stats.json")
    ap.add_argument("--out", default="dist/数据参数说明书.docx")
    args = ap.parse_args()

    stats = {}
    path = Path(args.stats)
    if path.exists():
        stats = json.loads(path.read_text(encoding="utf-8"))

    build(stats, Path(args.out))
    print(f"written: {args.out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
