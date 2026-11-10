# 演示模型来源与许可

## skull_demo.stl（头骨）

- **来源**：BodyParts3D（日本 DBCLS / 生命科学数据库中心），经 Wikimedia Commons 分发
  - 页面：https://commons.wikimedia.org/wiki/File:BodyParts3D_Skull.stl
  - 文件：https://upload.wikimedia.org/wikipedia/commons/7/7b/BodyParts3D_Skull.stl
- **原始规模**：171,248 面，8.5 MB，外形 151 × 209 × 215 mm（成人头骨，含下颌）
- **处理**：`tools/decimate_stl.py --faces 8000` 二次误差简化到 8,000 面（400 KB），尺寸不变
- **许可**：**CC BY-SA 2.1 Japan**（署名—相同方式共享）
  - 用于演示可以；对外发布/商业产品里使用需保留署名，且衍生模型需同许可
  - 若产品不能接受 ShareAlike，请改用 NIH 3D（多为公有领域）或自行扫描数据
- **注意**：原始坐标不在原点（Z≈1421–1636），导入后程序会自动居中；单位按 mm

## needle_demo.stl / .glb

150 mm 针，由 `tools/make_demo_stl.py` 程序生成，无第三方版权。

## base_demo.stl / base_box.stl / .glb

底座方块，同上，程序生成。

## body_demo.stl

占位躯干（椭球），程序生成，**不是解剖模型**，仅用于测试视角/导入。

## 重新生成

```powershell
python tools/make_demo_stl.py --out examples     # 针 / 底座 / 占位躯干
python tools/make_box.py --size 120 80 10 --glb  # 长方体基座
python tools/decimate_stl.py 原始头骨.stl --faces 8000 --out examples\skull_demo.stl
```
