# 测试用三维模型

## material.stp

- 来源：`https://github.com/trimesh/cascadio` 的 `tests/models/material.stp`
- 用途：仅用于验证 STEP 导入通道（cascadio → OBJ → 三角面）
- 规模：8.9 KB，转换后 24 顶点 / 12 三角面，外形约 60–70 mm 的方块

## examples/ 下的演示模型

- `needle_demo.stl`：150 mm 针（180 面），由 `tools/make_demo_stl.py` 生成
- `base_demo.stl`：90×90×8 mm 底座（12 面），同上

`tools/make_demo_stl.py` 可随时重新生成，不依赖任何第三方库。
