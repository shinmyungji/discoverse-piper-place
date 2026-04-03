# XML Editor V1.0 已完成功能详解

本文梳理当前 XML Editor V1.0 中与四个核心需求相关的实现逻辑。每个需求包含：执行流程、关键代码位置、参数/数据处理细节以及一个可操作的使用示例。文末列出在复核过程中发现并修复的问题。

---

## 需求 1：属性面板支持多位小数输入并最终四舍五入显示三位小数

### 核心实现
- 自定义 `SmartDecimalsSpinBox` 派生自 `QDoubleSpinBox`，通过 `textFromValue` 在非编辑态格式化为 3 位小数，编辑态展示完整精度，避免输入过程中被截断（`xml_editor/view/property_view.py:17`, `xml_editor/view/property_view.py:56`）。
- 所有属性面板的浮点输入（位置、旋转、尺寸、Mesh 缩放、Joint 属性等）统一由 `_create_double_spinbox` 创建，直接复用上述控件，并允许配置最大 15 位小数的输入精度（`xml_editor/view/property_view.py:458`, `xml_editor/view/property_view.py:477`）。
- 控制面板中用于调整全局 Gizmo 尺寸及高斯背景变换的输入也采用同一控件，保证界面行为一致（`xml_editor/view/control_panel.py:74`, `xml_editor/view/control_panel.py:105`，`xml_editor/view/control_panel.py:445`, `xml_editor/view/control_panel.py:452`）。

### 数据流与事件
1. 用户在 SpinBox 中输入数值，控件处于编辑态，实时保留全部精度。
2. `editingFinished` 触发 `_on_editing_finished`，内部调用 `setValue(self.value())`，触发 `valueChanged`。
3. `valueChanged` 信号通过 `_create_double_spinbox` 内部绑定，直接分发到属性视图模型 `PropertyViewModel.set_property`（`xml_editor/viewmodel/property_viewmodel.py:200`, `xml_editor/viewmodel/property_viewmodel.py:360`），更新几何体数据并向场景模型广播变更。
4. 控件刷新显示文本时走 `textFromValue`，最终以 3 位小数格式化展示，满足“四舍五入到三位”的需求。

### 操作示例
1. 选中任意几何体，在属性面板的“位置 X”栏输入 `1.23456789`。
2. 按回车或切换焦点后，控件离开编辑态，界面显示 `1.235`。
3. 若再次进入编辑态，文本栏会展示完整的 `1.23456789`，便于继续微调。
4. 保存场景时，对应的几何体位置会使用高精度浮点写入，再由需求 2 的角度转换逻辑统一处理输出。

### 位置编辑的事件链路
1. **UI 捕获输入**：位置 SpinBox 属于 `SmartDecimalsSpinBox`，在 `valueChanged` 时立即将最新数值通过信号发出（`xml_editor/view/property_view.py:457`）。
2. **面板转发**：`PropertyPanel` 把 `propertyChanged` 信号直接连接到视图模型的 `set_property`，形成 UI→ViewModel 的通道（`xml_editor/view/property_panel.py:40`）。
3. **数据更新**：`PropertyViewModel.set_property` 根据字段名更新几何体的 `position`，仅改动被编辑的分量后写回对象实例（`xml_editor/viewmodel/property_viewmodel.py:210`）。
4. **广播变更**：同一方法随后调用 `SceneViewModel.notify_object_changed`，统一刷新层级缓存并发射对象变化信号（`xml_editor/viewmodel/scene_viewmodel.py:747`）。
5. **视图重绘**：OpenGL 视图订阅 `objectChanged`，收到信号后更新操控器射线并请求重绘，于是场景中的几何体即时移动到新位置（`xml_editor/view/opengl_view.py:1476`）。

### 属性展示链路：单文件 vs 多文件
1. **加载阶段**：`MainWindow._open_file` 调用 `SceneViewModel.load_scene(path, append=False)`，单文件场景会清空 `_geometries`，随后 `XMLParser.load` 解析 MJCF 并返回 `Geometry` 列表，`SceneViewModel._wrap_loaded_geometries` 以文件名创建 `GeometryGroup` 并设置 `_active_source_file`（`xml_editor/main.py:270`, `xml_editor/viewmodel/scene_viewmodel.py:547`, `xml_editor/model/xml_parser.py:68`）。（0930）
2. **多文件追加**：再次打开文件时，`load_scene(path, append=True)` 保留旧场景并为新文件添加额外的 `GeometryGroup`，每个对象都附带 `source_file`，因此属性读取时能精确定位所属来源（`xml_editor/viewmodel/scene_viewmodel.py:300`, `xml_editor/viewmodel/scene_viewmodel.py:318`）。
3. **选择同步**：用户在层级树或视图中选中对象会触发 `SceneViewModel.selected_geometry`，内部设置 `_selected_geo`、刷新 `_active_source_file` 并发射 `selectionChanged`（`xml_editor/viewmodel/scene_viewmodel.py:102`, `xml_editor/viewmodel/scene_viewmodel.py:129`）。
4. **属性映射**：`PropertyViewModel` 监听 `selectionChanged`，把最新 `Geometry` 缓存在 `_selected_object` 并发射 `propertiesChanged`。界面随后通过 `get_property` 读取位置、旋转、物理、mesh、joint 等字段；多文件场景下相同逻辑同样适用（`xml_editor/viewmodel/property_viewmodel.py:17`, `xml_editor/viewmodel/property_viewmodel.py:58`, `xml_editor/view/property_view.py:602`）。
5. **上下文切换**：对于多文件情况，`SceneViewModel` 会在发射 `selectionChanged` 前激活对应解析上下文，确保属性面板读写的都是选中对象所属文件的原始属性（`xml_editor/model/xml_parser.py:152`, `xml_editor/viewmodel/scene_viewmodel.py:120`）。

这样一来，无论当前打开的是单个 XML 还是多份 MJCF，属性面板都会即时反映各对象在原文件中的定义，为后续编辑和保存提供一致的数据源。

---

## 需求 2：加载 XML 时弧度→角度、编辑时使用角度、保存时再转弧度

### 关键数据与变量
- `_last_loaded_angle_mode`：记录最近一次载入文件的角度单位，供后续导出或另存为时恢复上下文（`xml_editor/model/xml_parser.py:26`, `xml_editor/model/xml_parser.py:95`）。
- `angle_mode/is_radian`：`_load_mujoco_format` 中依据 `<compiler angle>`, 默认 `radian`；无声明时视为弧度（`xml_editor/model/xml_parser.py:347`）。
- `_to_deg_scalar/_to_deg_vec`：将 MJCF 中的弧度转成编辑器内部使用的角度数组，保证 UI 显示统一（`xml_editor/model/xml_parser.py:382`）。
- `_convert_joint_angles_to_radian`：保存时兜底处理所有 joint `range/ref` 字段，防止角度制残留（`xml_editor/model/xml_parser.py:1425`）。

### 时间链路一览（radian 源文件）
1. **读取**：`XMLParser.load` 解析 `<compiler angle="radian">`，缓存单位信息并传入 `_load_mujoco_format`（`xml_editor/model/xml_parser.py:95`）。
2. **转换**：`_load_mujoco_format` 中 `is_radian=True`，所有包含 `euler`/`quat`/`axisangle` 的 body 与 geom 会调用 `_to_deg_*` 或 `_quat_to_euler` 变为角度，存入 `Geometry.rotation` 与 `GeometryGroup.rotation`（`xml_editor/model/xml_parser.py:382`, `xml_editor/model/xml_parser.py:1290`）。
3. **展示**：属性面板直接读取 `Geometry.rotation`，配合需求 1 的高精 SpinBox 显示角度值（`xml_editor/viewmodel/property_viewmodel.py:225`）。
4. **编辑**：用户修改角度后，`PropertyViewModel.set_property` 以角度更新几何体实例，场景视图模型广播变更供视图刷新（`xml_editor/viewmodel/property_viewmodel.py:225`, `xml_editor/viewmodel/scene_viewmodel.py:747`）。
5. **保存**：`export_mujoco_xml` 以原始 MJCF 为底稿，将 `<compiler angle>` 统一写成 `radian`；`_add_object_to_mujoco` 把当前角度用 `np.radians` 转回弧度写入 `euler`（`xml_editor/model/xml_parser.py:957`, `xml_editor/model/xml_parser.py:1102`）。
6. **Joint 兜底**：若原文件为 degree，但后续另存仍沿用旧上下文，`_convert_joint_angles_to_radian` 会批量把 joint `range/ref` 转成弧度，slide 关节例外保持线性单位（`xml_editor/model/xml_parser.py:1355`）。

### 时间链路二览（degree 源文件）
1. `<compiler angle="degree">` 或手动省略 angle 时（是不是这里不应该这么说，没有就是默认设置为弧度radian？？？）：`_last_loaded_angle_mode='degree'`，`is_radian=False`（`xml_editor/model/xml_parser.py:388`）。
2. 加载过程中 `_normalize_joint_angle_dict` 会把 hinge 关节的 `range/ref` 统一转换为弧度，同时给 `Geometry` 标记 `joint_angle_mode='radian'`，因此内部总是以弧度存储，而 UI 仍显示角度（`xml_editor/model/xml_parser.py:167`, `xml_editor/model/xml_parser.py:516`, `xml_editor/viewmodel/property_viewmodel.py:425`）。
3. 保存时会先把所有 `<compiler>` 的 `angle` 归一化为 `radian`，`_add_object_to_mujoco` 依据 `joint_angle_mode` 检测是否仍有度数并即时转弧度；只有在上下文尚未归一（历史缓存）时才触发兜底的 `_convert_joint_angles_to_radian`（`xml_editor/model/xml_parser.py:1023`, `xml_editor/model/xml_parser.py:1225`, `xml_editor/model/xml_parser.py:1057`, `xml_editor/model/xml_parser.py:1425`）。
4. 如果只修改了部分几何体，其余从原文件继承的 `mjcf_attrs`（例如 `euler`、`quat`）也会在 `_add_object_to_mujoco` 中被更新为弧度表示，保证输出文件完全统一（`xml_editor/model/xml_parser.py:1204`）。
5. UI 若将 hinge `range/ref` 输入角度值，`PropertyViewModel.set_property` 会先 `np.radians` 写回 `joint_attrs`；导出阶段 `_add_object_to_mujoco` 将 `joint_attrs` 与 `mjcf_attrs` 合并后写入 XML，因此文件始终保存最新弧度值，`mjcf_attrs` 无需单独同步（`xml_editor/viewmodel/property_viewmodel.py:326`, `xml_editor/model/xml_parser.py:1217`）。

### Joint 属性处理链路
1. 加载：若 joint 使用 degree（`angle="degree"`），`PropertyViewModel.get_joint_attributes` 在读取弧度值后转换为角度文本显示，界面与内部单位保持一致（`xml_editor/viewmodel/property_viewmodel.py:425`）。
2. 编辑：用户输入角度时，`set_property` 在写回 `joint_attrs` 前转为弧度字符串，并把 `joint_angle_mode` 标记成 `'radian'`，确保后续保存不会重复转换（`xml_editor/viewmodel/property_viewmodel.py:326`）。
3. 保存：`_add_object_to_mujoco` 优先读取 `joint_angle_mode` 判断是否需要把 `range/ref` 从度数转换为弧度；仍保留大于 `2π` 的兜底检测，最终统一输出弧度制（`xml_editor/model/xml_parser.py:1225`）。

### 操作示例
1. 打开含 `compiler angle="radian"` 的 MJCF，`euler="1.5707963 0 0"` 会在 UI 中显示为 `90°`。
2. 用户将 X 轴角度改为 `45` 并保存，新文件里的 `euler` 写成 `0.7853981633974483 0.0 0.0`，`<compiler>` 保证为 `radian`。
3. 打开 `angle="degree"` 的文件时即便不改动也会在保存后统一转弧度；joint `range="-90 90"` 会输出为 `-1.5708 1.5708`。

---

## 需求 3：坐标系(Gizmo)尺寸随物体自动缩放，并支持手动调整

### 自动缩放事件链路
1. **选中对象**：`SceneViewModel.selected_geometry` setter 在高亮新物体时调用 `_auto_update_gizmo_size`（`xml_editor/viewmodel/scene_viewmodel.py:102`）。
2. **量测尺寸**：`_auto_update_gizmo_size` 优先读取 mesh 三角面包围盒最大边长；如果是原生 geom，则取 `size` 的最大半径×2，过小结果兜底为 1.0（`xml_editor/viewmodel/scene_viewmodel.py:421`）。
3. **更新全局尺度**：测得的 `max_dim` 传入 `set_global_gizmo_size_world`，该方法缓存到 `global_gizmo_size_world` 并通过 `gizmoSizeChanged` 广播（`xml_editor/viewmodel/scene_viewmodel.py:1442`）。
4. **视图响应**：OpenGL 视图监听 `objectChanged` 和 `gizmoSizeChanged`，在下一帧绘制前读取新的世界尺度；控制面板也同步刷新 SpinBox 显示（`xml_editor/view/opengl_view.py:1476`, `xml_editor/main.py:61`）。

### 手动调节事件链路
1. 用户在控制面板的 “全局大小” SpinBox 输入新值，控件直接调用 `SceneViewModel.set_global_gizmo_size_world`（`xml_editor/view/control_panel.py:74`, `xml_editor/main.py:58`）。
2. 视图模型写入同一全局变量并广播，覆盖自动推导的结果；下次选择对象时若仍启用自动流程，会在步骤 1 重新测量更新。
3. OpenGL 视图与控制面板通过信号保持同步，防止出现 UI 与实际 Gizmo 尺度不一致的情况（`xml_editor/main.py:92`）。

### OpenGL 呈现与交互
- `_draw_transform_controller` 在渲染平移/旋转/缩放 Gizmo 前读取 `global_gizmo_size_world`，所有轴线、圆环、方块把手按该尺度缩放，保证不同尺寸对象拥有合理的控件体积（`xml_editor/view/opengl_view.py:2373`, `xml_editor/view/opengl_view.py:2382`）。
- 拾取时同样基于该尺度推导轴长度、拾取半径与内外间距，从而在大物体与小物体上拥有一致的交互灵敏度（`xml_editor/view/opengl_view.py:1856`）。
- **鼠标命中判定流程**：
  1. 鼠标按下后，`OpenGLView.mousePressEvent` 先根据当前变换模式挑选对应的 `_dragAxis` 类型（平移/旋转/缩放），并把屏幕坐标转换成 OpenGL 视口坐标（`xml_editor/view/opengl_view.py:1202`, `xml_editor/view/opengl_view.py:1294`）。
  2. `_pick_translate_axis` / `_pick_rotate_axis` / `_pick_scale_axis` 会把鼠标射线投射到世界空间，结合当前 Gizmo 大小构造出每个轴/圆环的粗细和半径；例如平移把手会在 `_hit_capsule` 中把 “轴线 + 半径” 构成一个圆柱形包络体，用于测试射线是否命中（`xml_editor/view/opengl_view.py:1782`, `xml_editor/view/opengl_view.py:1826`）。
  3. 命中后记录 `_activeAxis` 与起始射线参数，为后续 `mouseMoveEvent` 中的拖动换算做准备，并触发 `scene_viewmodel` 更新选中状态。若没有任何命中，则回退到常规的几何体选择逻辑，让用户继续拾取场景对象（`xml_editor/view/opengl_view.py:1334`, `xml_editor/view/opengl_view.py:1880`）。

### 操作示例
1. 选中一个尺寸为 `10x10x10` 的 Box，自动 Gizmo 轴长变为 10，方便拖拽；切换到半径 0.2 的 Sphere 时同样缩小。
2. 在控制面板把“全局大小”改成 `5.0` 后，任何选中对象都会立即显示相同大小的 Gizmo，适合对多对象进行统一操作。
3. 再次选择新的 mesh 时，若未手动修改，系统会重新根据 mesh 三角面重新估算尺度，实现自适应效果。

---

## 需求 4：平移/旋转/缩放三种模式的坐标轴样式区分

### 模式切换事件链路
1. **用户操作**：控制面板单选按钮或快捷键更新 `ControlViewModel.operation_mode`，随后写入 `SceneViewModel.operation_mode`（`xml_editor/view/control_panel.py:204`, `xml_editor/view/control_panel.py:333`）。
2. **状态保存**：`SceneViewModel.operation_mode` setter 发出 `operationModeChanged`，OpenGL 视图在槽函数中刷新控制器射线缓存并触发重绘（`xml_editor/viewmodel/scene_viewmodel.py:80`, `xml_editor/view/opengl_view.py:1482`）。
3. **渲染分支**：每次绘制 (`_draw_transform_controller`) 都依据当前模式调用平移/旋转/缩放三种绘制函数，确保界面立即响应模式切换（`xml_editor/view/opengl_view.py:2379`）。

### 绘制与 OpenGL 处理
- **平移 Gizmo**：关闭光照以使用纯色线条，绘制三轴线段并在端点添加圆锥箭头；当前选中轴使用浅色高亮（`xml_editor/view/opengl_view.py:865`）。
- **旋转 Gizmo**：同样关闭光照，并在绘制前暂时禁用 `GL_BLEND` 避免球状端点被半透明混合；端点使用 `glutSolidSphere` 表示旋转把手（`xml_editor/view/opengl_view.py:936`）。
- **缩放 Gizmo**：以线段连接方块把手，方块通过 `glScalef` 缩放保证与世界尺度一致，同样使用高亮提示当前轴（`xml_editor/view/opengl_view.py:1020`）。
- 三类函数均读取 `global_gizmo_size_world` 与 `_controller_axis`，前者控制整体大小，后者决定哪条轴需要高亮（`xml_editor/view/opengl_view.py:885`, `xml_editor/view/opengl_view.py:1856`, `xml_editor/view/opengl_view.py:2373`）。

- 鼠标按下若当前已有 Gizmo，可直接拾取对应轴控制器；否则左键选中几何体会触发 `SceneViewModel.selected_geometry`，其 setter 自动切换到平移模式并重新估算 Gizmo 尺度，从而让坐标轴随选中物体立即出现在场景中央（`xml_editor/view/opengl_view.py:1266`, `xml_editor/viewmodel/scene_viewmodel.py:110`）。
- 拾取流程： `_pick_controller` 先依据世界尺度 `_pick_axis_by_distance` 判断最近的轴线段；若未命中则回退到 `_controllor_raycaster.raycast` 做光线检测，兼容自定义控件（`xml_editor/view/opengl_view.py:1853`, `xml_editor/view/opengl_view.py:1884`）。
- 若成功命中，记录 `_controller_axis` 与 `_drag_operation`；随后的鼠标拖动会根据当前模式写回 `position`/`rotation`/`size` 并通过 `SceneViewModel.notify_object_changed` 广播，使视图和属性面板保持同步（`xml_editor/view/opengl_view.py:1873`, `xml_editor/viewmodel/property_viewmodel.py:210`, `xml_editor/viewmodel/scene_viewmodel.py:747`）。

### 操作示例
1. 进入平移模式并拖动红轴顶部箭头，对象沿 X 轴移动；坐标轴实时高亮并更新位置。
2. 切换到旋转模式，拖动绿色端点，控件在蓝色平面内绘制旋转圆弧，位置保持不变。
3. 切换到缩放模式，拖动方块把手即可沿对应轴缩放；所有操作均继承自动/手动设定的 Gizmo 世界尺度。

---

## 复核中发现并修复的问题

- **问题**：当原始 MJCF 使用 `quat`（或其他姿态字段）描述姿态时，导出流程会保留旧属性，同时新增 `euler`。MuJoCo 会优先读取 `quat`，导致编辑器中对角度的修改在导出后不生效。
- **原因定位**：`_add_object_to_mujoco` 在写回原始属性快照后，没有移除 `quat` / `axisangle` / `xyaxes` / `zaxis` 等互斥字段（原实现位于 `xml_editor/model/xml_parser.py:1012`, `xml_editor/model/xml_parser.py:1140`）。
- **修复**：在更新 Body 与 Geom 姿态时，统一删除这些冲突字段，仅保留更新后的 `euler`（`xml_editor/model/xml_parser.py:1017`, `xml_editor/model/xml_parser.py:1044`, `xml_editor/model/xml_parser.py:1123`）。
- **成效**：现在即使原始 XML 由四元数描述，重新导出也会使用最新的角度（弧度制）数据，满足需求 2 关于角度统一处理的约束。

- **问题**：当源文件声明 `<compiler angle="degree">` 时，保存后的 hinge `range` 可能缩小 57.3 倍，例如 `90` 被写成 `0.0274156`。
- **原因定位**：加载阶段保留了度数字符串，保存时 `_add_object_to_mujoco` 首先根据阈值将其乘以 `π/180`，随后兜底的 `_convert_joint_angles_to_radian` 再次执行 `np.radians`，造成双重转换（`xml_editor/model/xml_parser.py:1225`, `xml_editor/model/xml_parser.py:1057`）。
- **修复**：加载时通过 `_normalize_joint_angle_dict` 将 hinge `range/ref` 统一转弧度并记录 `joint_angle_mode`；导出前把所有 `<compiler>` 的 `angle` 改写为 `radian`，保存时仅在标记为度数或检测到异常范围时才转换，避免重复兜底（`xml_editor/model/xml_parser.py:167`, `xml_editor/model/xml_parser.py:516`, `xml_editor/model/xml_parser.py:1023`, `xml_editor/model/xml_parser.py:1057`, `xml_editor/viewmodel/property_viewmodel.py:326`）。
- **成效**：`range="90 0"` 现在稳定输出为 `1.57079632679 0`，并且结果 XML 不再同时存在 `angle="degree"` 与 `angle="radian"` 的重复声明。

- **问题**：Gizmo 的 X 轴在平移/旋转/缩放模式下拖动方向与鼠标相反，且 X 轴缩放会向反方向收缩。
- **原因定位**：`_calculate_drag_amount` 通过 `np.cross(camera_forward, world_up)` 求取摄像机右向量，导致右向量与世界坐标系 X 方向相反；同时 `_handle_local_scale` 对 X/Y 轴使用除法更新尺寸（`xml_editor/view/opengl_view.py:2392`, `xml_editor/view/opengl_view.py:2345`）。
- **修复**：改为以 `world_up × camera_forward` 生成右向量并重新规范化上向量，确保右手坐标系不变；缩放时统一乘以正向缩放因子并设置最小阈值，杜绝反向收缩（`xml_editor/view/opengl_view.py:2392`, `xml_editor/view/opengl_view.py:2353`）。
- **成效**：鼠标沿 X 轴拖动现在与视觉预期一致，三种模式在三轴上的反馈保持统一，属性面板与场景变换同步更新。

- **问题**：`SceneViewModel.notify_object_changed` 在类内被重复定义，后一份覆盖前一份后不再刷新父子层级的变换矩阵，离开渲染循环调用时子节点仍持有旧的 `transform_matrix`。
- **原因定位**：后定义的函数仅发射 `objectChanged/geometryChanged` 信号，没有调用 `update_all_transform_matrices` 和 `selectionChanged`（`xml_editor/viewmodel/scene_viewmodel.py:1730`）。
- **修复**：合并为单一实现：先更新全局变换缓存，再统一发射 `objectChanged/geometryChanged`，若对象仍被选中则补发 `selectionChanged`；同时移除覆盖版并取消重复 emit（`xml_editor/viewmodel/scene_viewmodel.py:1039`, `xml_editor/viewmodel/scene_viewmodel.py:1206`）。
- **成效**：通过属性面板或脚本修改父节点后，子节点立刻获得最新局部/全局矩阵，射线拾取、局部 Gizmo 与导出缓存保持一致。

- **问题**：局部平移 `_handle_local_translation` 混用了加减号，导致 X/Y 轴拖拽反向、Z 轴又保持正确；全局平移还额外手动反转了 X/Y 轴向量，三轴方向逻辑不一致。
- **原因定位**：本地位置更新时写成 `position - translation_vector[:2]` 导致符号颠倒，同时全局分支为了“补偿”又硬编码翻转 X/Y（`xml_editor/view/opengl_view.py:2071`, `xml_editor/view/opengl_view.py:2107`）。
- **修复**：统一改为 `position += translation_vector`，并移除全局分支的手工反号，拖拽结果完全由 `_calculate_drag_amount` 与选中轴决定（`xml_editor/view/opengl_view.py:2076`, `xml_editor/view/opengl_view.py:2110`）。
- **成效**：平移 Gizmo 三轴方向与鼠标移动保持一致，不论局部/全局坐标系；属性面板数值与场景运动同步，避免对不上号。

- **问题**：右侧属性面板默认宽度过窄，打开程序后控件经常被折行或截断，只能手动拖动分隔条。
- **修复**：取消左右 dock（层级、控制、属性）以及其内容控件的最小宽度限制，并为层级树/控制面板增加滚动容器，三栏初始宽度固定但现在都能自由缩放（`xml_editor/view/property_view.py:105`, `xml_editor/view/control_panel.py:48`, `xml_editor/view/hierarchy_tree.py:24`, `xml_editor/main.py:226`）。
- **成效**：初次启动即可看到完整的属性表单，必要时仍可手动缩放面板宽度。

---

## 需求 5：导出时保留未修改的 MJCF 结构

### 关键变量速查
| 名称 | 定义位置 | 作用 |
| --- | --- | --- |
| `_last_loaded_mjcf_root` | `xml_editor/model/xml_parser.py:20` | 深拷贝的原始 MJCF 根节点，作为导出模板 |
| `_context_by_file` | `xml_editor/model/xml_parser.py:27` | 以绝对路径为键的上下文快照（根节点/angle/meshdir/资产） |
| `_active_source_key` | `xml_editor/model/xml_parser.py:28` | 当前激活的源文件路径，配合 `activate_context` 使用 |
| `_source_groups` | `xml_editor/viewmodel/scene_viewmodel.py:53` | 每个源文件对应的 `GeometryGroup`，保存时遍历其子节点 |
| `_collect_export_objects()` | `xml_editor/viewmodel/scene_viewmodel.py:375` | 展开 source group，过滤掉仅用于分组的节点 |
| `_sync_mesh_assets()` | `xml_editor/model/xml_parser.py:1061` | 合并资产列表，保留未修改的 `<mesh>`、`<material>` |

### 单文件工作流
1. **载入**：`XMLParser.load` 在解析完字符串树后调用 `_snapshot_state`，缓存 `root_snapshot`、`angle_mode`、`meshdir` 等信息并存入 `_context_by_file`，随即激活该上下文（`xml_editor/model/xml_parser.py:68`, `xml_editor/model/xml_parser.py:122`）。
2. **编辑**：用户在场景中增删改几何体，`SceneViewModel` 始终保留顶层 `GeometryGroup`（`_is_source_root=True`），并把新几何体归属到该组下（`xml_editor/viewmodel/scene_viewmodel.py:300`, `xml_editor/viewmodel/scene_viewmodel.py:215`）。
3. **保存/另存**：`SceneViewModel.save_scene` 在收集导出对象前先计算上下文：若目标文件属于当前来源列表，则激活其上下文；若是“单文件另存为”则复用唯一来源的上下文；若仅导出未归属对象则显式清空上下文（`xml_editor/viewmodel/scene_viewmodel.py:654`, `xml_editor/viewmodel/scene_viewmodel.py:675`）。随后调用 `XMLParser.export_mujoco_xml(..., preserve_auxiliary=not include_unsourced_only)`，从而在常规保存时复用原 MJCF 模板、在“未归属对象另存”时创建全新根节点（`xml_editor/viewmodel/scene_viewmodel.py:681`）。
   - 若“另存为”指向一个已经存在、但并非当前来源列表中的 XML，会在用户确认覆盖后先删除该目标文件，再以当前来源的上下文为模板写入新内容，从而既不会混入旧文件结构，也能保留源 XML 中的 `<camera>/<light>` 等段落（`xml_editor/viewmodel/scene_viewmodel.py:681`）。
4. **批量保存**：多文件场景触发“保存全部”时，仍由 `SceneViewModel.save_loaded_sources` 逐个激活上下文并写回对应文件，流程与单文件保持一致（`xml_editor/viewmodel/scene_viewmodel.py:375`）。
5. **导出**：`export_mujoco_xml` 在 `preserve_auxiliary=True` 时会优先使用缓存根节点；若缓存缺失但目标文件存在，则回退解析磁盘上的旧文件以复用 `<visual>`、`<equality>` 等 worldbody 之外的段落（`xml_editor/model/xml_parser.py:952`, `xml_editor/model/xml_parser.py:965`, `xml_editor/model/xml_parser.py:975`）。
   - 若 `duplicate_source_file` 的目标路径已存在且与源文件不同，也会先删除目标文件，再使用源文件的上下文重建，从而得到与“新文件”相同的覆盖效果（`xml_editor/viewmodel/scene_viewmodel.py:500`）。
6. **资产合并**：保存完成后 `_sync_mesh_assets` 会把新增 mesh 合并进原 `<asset>`，原有条目保持不动；成功写盘后 `_update_active_context` 更新缓存，保证下一次编辑仍基于最新内容（`xml_editor/model/xml_parser.py:1061`, `xml_editor/model/xml_parser.py:1029`）。

### 多文件工作流
1. 每次加载新文件都会生成专属的 `GeometryGroup` 并缓存上下文；选中不同对象时，`SceneViewModel.selected_geometry` 调用 `XMLParser.activate_context`，确保属性面板读取的上下文与当前文件一致（`xml_editor/viewmodel/scene_viewmodel.py:102`）。
2. `save_loaded_sources` 逐文件循环：对每个 `(path, group)`，激活上下文 → 收集子节点 → 导出 → 成功后刷新缓存。若任意文件写盘失败会记录在 `failures`，便于调用方提示用户（`xml_editor/viewmodel/scene_viewmodel.py:375`）。
3. 另存为场景时 `SceneViewModel.save_scene` 也复用同一流水线：未归属任何源文件的对象会集中导出到新文件，其余源文件依旧按原路径写回，避免混合内容（`xml_editor/viewmodel/scene_viewmodel.py:580`）。
4. 解析层使用 `_context_by_file` 维护“文件 → 解析快照”的映射，每次加载或写盘都会通过 `_snapshot_state` 把根节点、角度模式、`meshdir`、资产表等打包缓存；`XMLParser.activate_context` 根据当前活跃文件恢复这些状态。单文件流程中，加载后立即 `_store_context_for_file`，保存或属性编辑时直接在同一快照上工作；多文件流程会为每个来源维护独立快照，`SceneViewModel` 在切换选中对象或批量保存时调用 `activate_context`，确保不会串写 mesh、compiler 或资产片段（`xml_editor/model/xml_parser.py:122`, `xml_editor/model/xml_parser.py:138`, `xml_editor/model/xml_parser.py:205`, `xml_editor/viewmodel/scene_viewmodel.py:103`, `xml_editor/viewmodel/scene_viewmodel.py:375`）。

### 导出细节与兜底
- **结构保留**：当原文件缺失 `<worldbody>` 或 `<asset>` 时自动补齐；若存在，则只清除 body/geom/joint 后重建，其他节点（camera、light、custom）按原顺序插回（`xml_editor/model/xml_parser.py:987`）。
- **其他段落**：通过根快照机制保留 `<visual>、<equality>` 等 worldbody 外的段落，确保额外配置不会在保存时丢失。
- **角度模式**：无论源文件是 `radian` 还是 `degree`，导出时都会写 `<compiler angle="radian">`，并在 degree 模式下仅在必要时触发 `_convert_joint_angles_to_radian` 批量换算（`xml_editor/model/xml_parser.py:1023`, `xml_editor/model/xml_parser.py:1425`）。若原始 `<compiler>` 已带有 `meshdir`、`texturedir` 等属性，代码会保留原字段仅补写 `angle="radian"`；若根本不存在 `<compiler>`，则在根节点追加新的 `<compiler angle="radian"/>`，确保导出的 MJCF 与编辑器内部单位始终一致（`xml_editor/model/xml_parser.py:1023-1036`）。
- **资产表**：`_sync_mesh_assets` 会合并多个 `<asset>` 节点，保留未修改的材质/纹理，仅更新编辑器新增的 mesh，同时同步 `_mesh_assets` 的 `_orig_file_attr`，防止后续再次重复追加（`xml_editor/model/xml_parser.py:1042`）。
- **包裹组不写出**：导入时 `SceneViewModel._wrap_loaded_geometries` 会生成以文件名命名的 `GeometryGroup` 并标记 `_is_source_root=True`，保存前通过 `_collect_export_objects` 展开真实子节点、跳过带该标记的占位组（`xml_editor/viewmodel/scene_viewmodel.py:368`, `xml_editor/viewmodel/scene_viewmodel.py:386`, `xml_editor/viewmodel/scene_viewmodel.py:401`），因此导出的 XML 不会出现额外的 group 层级。
- **错误恢复**：若保存过程中抛出异常，`failures` 列表会返回对应路径，用户可根据提示选择重试或另存（`xml_editor/viewmodel/scene_viewmodel.py:401`）。

### Bug 修复回顾
- V1.0 前版本仅维护全局 `_last_loaded_mjcf_root`，导致多文件保存时后加载的文件会覆盖前一个文件的 `<asset>/<actuator>` 等段。现在通过 `_context_by_file` + `activate_context` 机制，在切换文件和写盘前都恢复原上下文，彻底避免串写（`xml_editor/model/xml_parser.py:128`, `xml_editor/model/xml_parser.py:188`, `xml_editor/viewmodel/scene_viewmodel.py:104`）。
- 复测时发现当上下文意外丢失或执行单文件“保存”而未先激活上下文时，`export_mujoco_xml` 会退化到全新根节点，导致 `<visual>`、`<equality>` 等根级区块丢失。现已在保存前强制根据目标文件恢复上下文，并在缺省时回读旧文件再导出，仅在另存未归属对象时才跳过该流程（`xml_editor/viewmodel/scene_viewmodel.py:654`, `xml_editor/viewmodel/scene_viewmodel.py:681`, `xml_editor/model/xml_parser.py:965`）。
- 修复删除源 XML 或执行“另存为”后窗口标题、层级树仍指向旧文件的问题：新增 `sourcesChanged` 信号并在保存后通过 `_remap_source_group` 更新根节点归属，同时让主窗口订阅该信号刷新 `current_file`，确保后续新增几何体始终落在正确的来源文件下（`xml_editor/viewmodel/scene_viewmodel.py:38`, `xml_editor/viewmodel/scene_viewmodel.py:672`, `xml_editor/viewmodel/scene_viewmodel.py:728`, `xml_editor/main.py:79`, `xml_editor/main.py:115`, `xml_editor/main.py:442`）。
- 解除“无来源几何体无法保存”限制：当场景不再包含任何已关联的源文件时，允许直接保存/另存为，并在保存成功后自动将未关联的几何体挂载到新文件生成的根组，避免用户在删除旧 XML 后无法为新几何体建立来源的情况（`xml_editor/viewmodel/scene_viewmodel.py:430`, `xml_editor/viewmodel/scene_viewmodel.py:734`）。
- 新增“保存副本”操作，保留传统“另存为”继续编辑新文件的行业规范，同时提供保存副本后仍留在原文件的选项。副本保存通过 `save_scene(..., finalize=False)` 避免重绑来源，仅写出目标 XML（`xml_editor/main.py:233`, `xml_editor/main.py:474`, `xml_editor/viewmodel/scene_viewmodel.py:764`）。
- 优化层级拖放后的归属校验：当几何体从某 XML 根节点拖出后，会即时清空 `source_file` 并递归同步子节点，保存时统一提示“未关联到任何 XML”的警告，避免误把脱离的对象再次写回原文件（`xml_editor/viewmodel/hierarchy_viewmodel.py:466`, `xml_editor/viewmodel/scene_viewmodel.py:333`, `xml_editor/viewmodel/scene_viewmodel.py:430`）。

### 操作示例
1. 打开带有摄像机和自定义 `default` 的 MJCF，修改某个 geom 的颜色后保存；输出文件中仅目标 geom 发生变化，摄像机、`default`、`custom` 块保持原样。
2. 同时打开 `A.xml` 与 `B.xml`，分别修改不同对象并保存；两份文件各自写回，`A.xml` 不再包含 `B.xml` 的 `<asset>` 或 `<actuator>` 段。
3. 另存为新文件时，原有来源文件照常写回，新文件只包含未归属源的几何体，便于拆分场景。

---

## 需求 6：修复 geom name 在导入/导出之间重复叠加的问题

### 核心实现
- 解析阶段记录每个 geom 是否在原始 XML 中显式带有 `name` 属性，缺省时仍为内部显示生成的 `bodyName_geom`，但 `_mjcf_had_name` 标志为 False（`xml_editor/model/xml_parser.py:404`, `xml_editor/model/xml_parser.py:520`, `xml_editor/model/xml_parser.py:572`）。
- 导出时仅在原文件本来就有 `name` 时才把当前名称写回；若原先缺省则从导出的 XML 中移除，防止额外追加（`xml_editor/model/xml_parser.py:1183`, `xml_editor/model/xml_parser.py:1186`, `xml_editor/model/xml_parser.py:1188`）。
- 新建几何体默认会写入名称，方便后续定位；但对已有对象不会重复前缀，因此多次导入/导出后名称保持稳定。
- 如果用户在 UI 中为原本无 `name` 的 geom 修改了名称，编辑器会保留该值用于界面展示与识别，但导出仍遵循 `_mjcf_had_name=False` 的判断，不会把名称写入 XML，以避免破坏原始文件的“匿名”约定（`xml_editor/model/xml_parser.py:1183`）。

### 操作示例
1. 导入历史版本中名称为 `door_handle` 的 geom。
2. 修改材质后导出，再导入检查：名称仍为 `door_handle`，不再变成 `body_door_handle` 或多重前缀。
3. 对于原本没有 `name` 的 geom，导出文件继续保持无 `name` 属性。

---

## 需求 7：导入高斯背景时防止几何体变换导致背景变暗

### 修复要点
- `_draw_gs_background` 获取高斯渲染帧后，绘制背景前暂时关闭 GL_LIGHTING、GL_DEPTH_TEST 和 GL_BLEND，并在绘制完成后恢复原状态，避免前一帧中物体带来的渲染状态影响背景亮度（`xml_editor/view/opengl_view.py:328`, `xml_editor/view/opengl_view.py:339`, `xml_editor/view/opengl_view.py:395`）。
- 渲染结果使用纹理方式全屏贴图，不再与场景几何共用着色 pipeline，从而避免局部坐标缩放或旋转造成的光照干扰（`xml_editor/view/opengl_view.py:352`, `xml_editor/view/opengl_view.py:381`, `xml_editor/view/opengl_view.py:388`）。

### 操作示例
1. 导入高斯背景并选择某个物体，将其切换到缩放模式并放大数倍。
2. 背景保持原亮度和色调，不再因对象的局部变换而发暗。
3. 切换回旋转/平移模式，背景同样稳定显示。

---

## 需求 8：保存结果完整性验证

### 核心逻辑
- 保存入口遍历每个来源文件的根组，递归收集所有可导出的几何体，再逐个调用 `XMLParser.export_mujoco_xml` 写盘（`xml_editor/viewmodel/scene_viewmodel.py:375`, `xml_editor/viewmodel/scene_viewmodel.py:389`, `xml_editor/viewmodel/scene_viewmodel.py:392`）。
- `export_mujoco_xml` 在成功写入文件后返回 True，并捕获异常作为失败项报告，便于外层检测保存是否完整（`xml_editor/model/xml_parser.py:858`, `xml_editor/model/xml_parser.py:915`, `xml_editor/model/xml_parser.py:920`）。

### 检查结论
- 针对多来源场景逐个保存验证：每个源文件都能生成完整 MJCF，无遗漏或截断。
- 若导出过程中出现异常，界面会在 `failures` 列表中反馈文件路径，便于用户立即重试或排查。

---

以上内容覆盖了八个已实现需求的完整链路及关键代码位点，可作为后续维护和升级的参考资料。

---

## 需求 9：复制物体时自动生成 `_copy_n` 命名

### 核心实现
- 粘贴/复制操作统一委托给层级视图模型，先统计场景中已占用的名称，再逐个生成不冲突的新名（`xml_editor/viewmodel/hierarchy_viewmodel.py:162`, `xml_editor/viewmodel/hierarchy_viewmodel.py:178`）。
- `_generate_copy_name` 会剥离原名尾部已有的 `_copy` 或 `_copy_x` 再继续编号，计数从 1 递增确保形如 `car_copy_1`, `car_copy_2`（`xml_editor/viewmodel/hierarchy_viewmodel.py:307`, `xml_editor/viewmodel/hierarchy_viewmodel.py:323`）。
- 递归复制子节点时沿用同一 `used_names` 集合，保证嵌套结构中也不会重复（`xml_editor/viewmodel/hierarchy_viewmodel.py:240`, `xml_editor/viewmodel/hierarchy_viewmodel.py:300`）。
- 新建几何体/分组同样通过 `_generate_unique_name` 去重，避免因删除后重建导致名称复用进而破坏 MJCF 唯一性（`xml_editor/viewmodel/scene_viewmodel.py:181`, `xml_editor/viewmodel/scene_viewmodel.py:448`）。

### 操作示例
1. 在层级树中复制 `car` 并粘贴，会得到 `car_copy_1`；再次粘贴自动变成 `car_copy_2`。
2. 若源对象本身已经是 `car_copy_5`，复制后会回退到基名 `car` 继续新增 `car_copy_6`。

---

## 需求 10：同时加载并展示多个 3DGS 场景

### 核心实现
- 主窗口的 “打开 PLY” 支持多选，把路径数组交给 `OpenGLView.set_gs_backgrounds` 统一加载（`xml_editor/main.py:311`, `xml_editor/main.py:318`）。
- `OpenGLView.set_gs_backgrounds` 为每个 PLY 生成稳定的键值，缓存键到文件的映射并初始化 `GSRenderer`（`xml_editor/view/opengl_view.py:204`, `xml_editor/view/opengl_view.py:236`, `xml_editor/view/opengl_view.py:252`）。重复文件名会自动追加序号以避免覆盖。
- 视图层变更后，通过 `SceneViewModel.set_gs_background_state` 将 `[key, path]` 列表和当前激活键同步到模型层，便于状态持久化与界面联动（`xml_editor/view/opengl_view.py:272`, `xml_editor/viewmodel/scene_viewmodel.py:957`）。
- 控制面板监听 `gsBackgroundsChanged`，刷新下拉列表并保持键值一致，这样用户可以在 UI 中任意切换背景（`xml_editor/main.py:79`, `xml_editor/main.py:95`, `xml_editor/view/control_panel.py:81`）。
- 重新加载 XML 时保留已加载的 PLY 背景，同时控制面板提供常驻的“清除 PLY”按钮，点击后重置参数并调用视图模型清空背景列表（`xml_editor/main.py:340`, `xml_editor/view/control_panel.py:105`, `xml_editor/view/control_panel.py:452`, `xml_editor/viewmodel/control_viewmodel.py:98`, `xml_editor/viewmodel/scene_viewmodel.py:1559`）。
- 为避免参数累积，每次执行 “加载修改参数” 后都会自动把平移/旋转重置为 0、缩放恢复为 1.0，方便继续精调（`xml_editor/view/control_panel.py:446`）。

### 场景切换流程
1. 用户在控制面板中选择某个键，触发 `gsPlySelectionChanged` 信号（`xml_editor/view/control_panel.py:182`）。
2. `OpenGLView.set_active_gs_background` 根据键更新当前操作的 PLY 路径，并按需回写到 SceneViewModel（`xml_editor/view/opengl_view.py:287`, `xml_editor/view/opengl_view.py:300`）。
3. SceneViewModel 再次广播 `gsBackgroundsChanged` 时，OpenGL 视图和控制面板同步刷新，实现多背景的一致显示。

### 操作示例
1. 一次选中多个 PLY 文件导入，渲染器会在视图中逐个绘制；控制面板下拉框显示对应键值。
2. 切换下拉框条目即可在各背景间快速切换查看，不影响其它已加载的场景。

---

## 需求 11：对单个或多个 3DGS 场景进行平移/旋转/缩放

### 核心实现
- 控制面板提供平移/旋转/缩放输入并在“加载修改参数”按钮点击时发出 `applyGsEditRequested`，包含目标平移、欧拉角转四元数及 scale 值（`xml_editor/view/control_panel.py:70`, `xml_editor/view/control_panel.py:95`, `xml_editor/view/control_panel.py:437`）。
- 缺失法线的 mesh 会在加载/缩放时预先计算并缓存面法线，避免渲染阶段反复做叉乘运算导致拖拽卡顿（`xml_editor/viewmodel/scene_viewmodel.py:307`, `xml_editor/viewmodel/scene_viewmodel.py:1236`, `xml_editor/view/opengl_view.py:840`）。
- `MainWindow` 把信号路由到 `OpenGLView.apply_gsply_transform`，后者使用 `gsply_edit.py` 对激活的 PLY 做就地变换，并重新加载所有背景保持画面一致（`xml_editor/main.py:81`, `xml_editor/view/opengl_view.py:3026`, `xml_editor/view/opengl_view.py:3079`）。
- 变换命令由 `QProcess` 异步运行，避免 `subprocess.run` 阻塞 UI；OpenGL 视图会在任务执行期间显示状态、拦截重复请求，并在完成或失败后统一刷新背景/回滚备份（`xml_editor/view/opengl_view.py:3134`, `xml_editor/view/opengl_view.py:3231`）。
- 状态栏新增常驻 PLY 指示器，空闲/完成为绿色，执行中为黄色，失败为红色，便于用户确认细微变动是否已经应用完毕（`xml_editor/main.py:96`, `xml_editor/view/opengl_view.py:3199`）。
- 为防止破坏原始 PLY，首次对某个背景执行变换时会弹出“另存为”对话框拷贝出可编辑副本，后续所有命令都作用在该副本上，原文件保留不变；副本默认存放在 `save/gs_edits/` 并自动刷新 SceneViewModel 的路径映射（`xml_editor/view/opengl_view.py:334`, `xml_editor/view/opengl_view.py:351`, `xml_editor/view/opengl_view.py:3097`）。
- 在修改前会创建备份并托管给 SceneViewModel，支持撤销及历史回滚（`xml_editor/view/opengl_view.py:3041`, `xml_editor/viewmodel/scene_viewmodel.py:1010`）。
- 为避免多 PLY 场景中“操作完跳回上一背景”，刷新 PLY 后仅在本地恢复激活项并保留当前 key，从而保持用户焦点一致（`xml_editor/view/opengl_view.py:3085`）。
- 生成的命令形如 `python gsply_edit.py foo.ply -t tx ty tz -r qx qy qz qw -s scale -o foo.ply`，缩放默认 1.0，可用于等比调整点云尺寸。

### 多 PLY 控制流程
1. 控制面板下拉框通过 `gsPlySelectionChanged` 通知视图改变激活项；OpenGL 视图记住当前 key/路径并回写 SceneViewModel（`xml_editor/view/control_panel.py:182`, `xml_editor/view/opengl_view.py:287`）。
2. SceneViewModel 维护 key→路径映射，并在任何列表变化时广播 `gsBackgroundsChanged`；主窗口把最新列表回写给控制面板，保持三者一致（`xml_editor/viewmodel/scene_viewmodel.py:1085`, `xml_editor/main.py:61`）。
3. 应用平移/旋转/缩放时，只处理当前激活 PLY；执行完命令后刷新所有条目并恢复同一个 key，使多文件场景不会错位（`xml_editor/view/opengl_view.py:3065`）。（这里的多ply另存为的逻辑还没有测回退对不对，并且没有可视化过edited的ply文件？？？）

### 回退与存档目录
- `apply_gsply_transform` 在执行脚本前调用 `_create_gs_backup`，把原 PLY 拷贝到 `save/gs_backups/<timestamp>_xxx.ply`，目录在 OpenGL 视图初始化时确保存在（`xml_editor/view/opengl_view.py:135`, `xml_editor/view/opengl_view.py:3041`）。
- 备份路径通过 `SceneViewModel.record_gs_edit` 追加到 `_gs_edit_history`，便于撤销时按栈顺序恢复；`restore_gs_history_to` 会逐条复制备份回原文件并删除已回滚的备份，清理掉多余文件（`xml_editor/viewmodel/scene_viewmodel.py:1076`, `xml_editor/viewmodel/scene_viewmodel.py:1085`）。
- 当重新加载 PLY 或主动重置历史时 `_clear_gs_edit_history(remove_files=True)` 会删除所有备份文件，防止堆积；若只在内存中清空，则传 `False` 保留备份以便后续手动回退（`xml_editor/viewmodel/scene_viewmodel.py:1101`）。
- 关闭应用时 `MainWindow.closeEvent` 会调用 `SceneViewModel.clear_gs_backups()`，统一清除 `save/gs_backups` 里的临时 ply，避免备份在退出后继续占用磁盘（`xml_editor/main.py:512`, `xml_editor/viewmodel/scene_viewmodel.py:1157`）。
- 另外，控制面板的“创建存档点”功能使用 `save` 目录下的 JSON 存档，与 `gs_backups` 区分：前者保留场景几何体快照，后者保留高斯背景的 PLY 备份。

### 操作示例
1. 在下拉框选择某个背景，输入平移/旋转/缩放参数后点击“加载修改参数”，相应 PLY 会被重新渲染到新的位置和尺寸。
2. 对多个背景重复上述流程，系统会始终针对当前激活的键写入改动，其他背景保持原状。

---

## 需求 12：OBJ/STL 网格渲染（含直接导入与 XML 引用）
- 支持直接选择 `.mtl` 文件，若同目录存在同名 OBJ 会自动定位并加载，流程与直接导入 OBJ 相同，方便把材质与主体模型放在一起（`xml_editor/utils/mesh_loader.py:23`, `xml_editor/viewmodel/scene_viewmodel.py:242`）。

### 核心实现
- 直接导入：主窗口提供 “打开 OBJ/STL” 菜单，选择文件后交给 `OpenGLView.load_mesh`，最终调用 `SceneViewModel.create_mesh_from_path` 构建 `Geometry` 实例（`xml_editor/main.py:311`, `xml_editor/view/opengl_view.py:3090`, `xml_editor/viewmodel/scene_viewmodel.py:254`）。
- XML 引用：`XMLParser._load_mujoco_format` 读取 `<asset><mesh>` 与 `<geom type="mesh">`，组合 `meshdir` 与 `file` 计算绝对路径，调用 `load_mesh_file` 解析 OBJ/STL 数据并填充 `Geometry`（`xml_editor/model/xml_parser.py:552`, `xml_editor/model/xml_parser.py:590`）。
- 保存时统一通过 `XMLParser.register_mesh_asset` 保持资产表，`_sync_mesh_assets` 写回 `<mesh name="..." file="..." scale="...">`，并依据 `_mjcf_had_name` 决定是否输出 `name`（`xml_editor/model/xml_parser.py:37`, `xml_editor/model/xml_parser.py:1113`, `xml_editor/model/xml_parser.py:1330`）。
- 若导入的新 mesh 未绑定资产记录，导出前会自动补注册，确保生成的 `<asset><mesh file="..."/></asset>` 不会缺失路径（`xml_editor/model/xml_parser.py:126`, `xml_editor/model/xml_parser.py:1309`）。
- 无论直接导入还是从 MJCF 解析，都会先走 `utils.mesh_loader.load_mesh_file` 把 OBJ/STL 面片与法线读入 numpy 数组，填充 `Geometry.mesh_model_triangles` 供 OpenGL 实时绘制，并据此计算包围盒与默认 Gizmo 尺寸（`xml_editor/utils/mesh_loader.py:15`, `xml_editor/viewmodel/scene_viewmodel.py:256`, `xml_editor/view/opengl_view.py:3214`）。
- 解析后的三角数据在 `register_mesh_asset` 中登记资源路径；若跳过 utils 解析直接依赖 `<mesh file="...">`，编辑器将缺乏可渲染的面片数据，因此当前流程必须先解析 OBJ/STL 后才能驱动可视化与拾取。
- 缩放与中心更新由 `SceneViewModel.update_mesh_asset_scale` 与 `_rebuild_mesh_geometry` 负责，保证同一资产的所有实例同步变换（`xml_editor/viewmodel/scene_viewmodel.py:869`, `xml_editor/viewmodel/scene_viewmodel.py:913`）。
- 新增的 `.mtl` 入口允许直接选择 MTL 文件，内部会寻找同名 OBJ 并复用 OBJ 解析逻辑，方便将材质库与主体模型放在一起（`xml_editor/utils/mesh_loader.py:15`, `xml_editor/utils/mesh_loader.py:23`）。
- 属性面板永远允许编辑 mesh 资产缩放，后台会自动维护资产表并实时更新所有实例（`xml_editor/viewmodel/scene_viewmodel.py:856`, `xml_editor/viewmodel/scene_viewmodel.py:869`）。
- 为避免保存前恢复旧上下文导致缩放丢失，更新 mesh 资产缩放后会立即刷新 `_context_by_file` 中的快照；因此无论是新建 XML 还是加载旧 XML 后修改缩放，导出的 `<asset><mesh scale="...">` 都会准确写入最新值（`xml_editor/viewmodel/scene_viewmodel.py:309`, `xml_editor/viewmodel/scene_viewmodel.py:1100`, `xml_editor/model/xml_parser.py:158`）。
- 另存为/覆盖旧文件时若当前场景未使用旧的 mesh 资产，会在导出前依据实际几何体收集资产名称并裁剪 `_mesh_assets`，并在 `_sync_mesh_assets` 内移除不再使用的 `<mesh>` 节点，防止先前文件的资产残留（`xml_editor/model/xml_parser.py:107`, `xml_editor/model/xml_parser.py:1003`, `xml_editor/model/xml_parser.py:1070`）。

### Mesh 拾取逻辑
- 光线投射器为 `mesh` 类型单独分支，优先走 `_intersect_mesh` 而非默认 AABB 检测，彻底对齐渲染时的几何形状（`xml_editor/model/raycaster.py:201`, `xml_editor/model/raycaster.py:612`）。
- `_intersect_mesh` 会先调用 `transform_ray_to_local` 将视口射线带入 mesh 的局部坐标空间，局部三角形坐标已经在载入时依据资产中心与缩放进行了归一化处理，因此父节点的平移、旋转、缩放全部准确参与碰撞判定（`xml_editor/model/raycaster.py:630`, `xml_editor/model/raycaster.py:657`）。
- 进入局部空间后先执行 `_ray_intersects_aabb` 做快速裁剪，再对每个三角面使用 Möller–Trumbore 算法求交点与法线，最终选取距离最近的命中结果并转换回世界坐标（`xml_editor/model/raycaster.py:637`, `xml_editor/model/raycaster.py:670`）。
- 拾取结果包含世界坐标命中点与单位法线，交互层仍然复用统一的 `RaycastResult`，因此 Gizmo 吸附、属性面板刷新与其它几何体完全一致（`xml_editor/model/raycaster.py:657`, `xml_editor/model/raycaster.py:665`）。

### 桥梁场景拾取问题复盘
- 旧版本在未命中专用分支时会退回 `_intersect_aabb`，直接使用 `Geometry.aabb_min/max` 作为世界包围盒。然而该 AABB 仅由 `position ± size` 计算，并未折算父节点的姿态；当 mesh 挂在带旋转的 body 下时，AABB 与真实 mesh 会出现错位（`xml_editor/model/geometry.py:182`）。
- 在示例 `block_bridge_place.xml` 中，`bridge1`/`bridge2` 的父 body 分别带有 ±90° 的 Euler 角，因此它们的局部 AABB 被错误地投射到 worldbody 平面外，导致光线检测直接返回未命中，看起来像“点不到”。
- `_intersect_mesh` 改为基于局部三角面检测后，拾取流程不再依赖局部 AABB 的世界投影；即便父级存在任意组合的旋转/缩放，也能精准命中对应 mesh，在该桥梁场景中实测两个 geom 均可正常选中，不会再意外点到旁边的实例（`xml_editor/model/raycaster.py:612`, `xml_editor/model/raycaster.py:657`）。

### 操作示例
1. 从菜单导入 `chair.obj`，系统会自动计算包围盒及初始位置，并在资产表中登记 `chair`。
2. 若仅选中 `chair.mtl`，加载器会查找同目录下 `chair.obj` 并完成渲染。
3. 在 XML 中引用 `<mesh file="models/chair.stl" scale="2 2 1">`，解析器会加载并应用缩放，同时允许在属性面板继续调整资产缩放；渲染视图中点击任意 mesh 面片都会立即选中该几何体，即使它挂在带旋转的父节点之下。

---

## 需求 13：多 XML 文件的并行导入与保存

### 加载流程
- `MainWindow._open_file` 支持多选并依次调用 `SceneViewModel.load_scene(path, append=index>0)`，首个文件清空场景，其余文件以追加方式载入（`xml_editor/main.py:270`, `xml_editor/main.py:282`, `xml_editor/viewmodel/scene_viewmodel.py:547`）。
- `SceneViewModel._wrap_loaded_geometries` 用文件名创建顶层 `GeometryGroup`，标记 `_is_source_root` 与 `source_file`，并递归写入子节点，确保层级树能够按来源划分（`xml_editor/viewmodel/scene_viewmodel.py:300`, `xml_editor/viewmodel/scene_viewmodel.py:318`）。
- 解析器为每个文件缓存独立的上下文（根节点、angle 模式、meshdir、资产表等），存入 `_context_by_file` 并在选择或导出时通过 `XMLParser.activate_context` 恢复该文件的原始状态，避免串改（`xml_editor/model/xml_parser.py:25`, `xml_editor/model/xml_parser.py:128`, `xml_editor/model/xml_parser.py:161`）。
- 切换选中对象时激活对应来源的解析上下文，保证属性编辑及后续导出都针对当前文件（`xml_editor/viewmodel/scene_viewmodel.py:102`, `xml_editor/viewmodel/scene_viewmodel.py:120`）。
- 新建几何体会优先归属当前选择的来源（层级中选中的 XML 或其子节点），若未选中任何来源则保持游离状态以便明确另存为（`xml_editor/viewmodel/scene_viewmodel.py:181`, `xml_editor/viewmodel/scene_viewmodel.py:215`）。

### 导入示例
- **单文件 `humanoid.xml`**：
  1. 打开文件时 `MainWindow._open_file` 清空场景并将 `current_file` 设为绝对路径，同时 `loaded_xml_files` 只含该文件（`xml_editor/main.py:353`）。
  2. `SceneViewModel.load_scene` 解析后生成一个名为 `humanoid` 的 `GeometryGroup`，设置 `_is_source_root=True` 与 `source_file='.../humanoid.xml'`，并把所有 body/geom 挂在该组下（`xml_editor/viewmodel/scene_viewmodel.py:370`）。
  3. `_source_groups` 记录 `{abs_path: file_group}`，`_active_source_file` 指向该绝对路径，随即在 `_context_by_file` 中存入包含根节点、`angle`、`meshdir`、资产表等内容的快照，供后续编辑/保存复用（`xml_editor/viewmodel/scene_viewmodel.py:612`, `xml_editor/model/xml_parser.py:128`）。
- **多文件 `humanoid.xml` + `arena.xml`**：
  1. 第一轮与单文件相同，第二个文件以 `append=True` 加载，`SceneViewModel.load_scene` 再生成 `arena` 根组并追加到 `_geometries` 列表（`xml_editor/viewmodel/scene_viewmodel.py:609`）。
  2. `_source_groups` 变为 `{humanoid_abs: humanoid_group, arena_abs: arena_group}`；`loaded_xml_files` 列表拥有两个绝对路径，窗口标题显示“2 个文件”。
  3. `XMLParser._context_by_file` 中新增 `arena` 的解析快照；当用户在层级树选中 `arena` 下的 geom 时，`SceneViewModel.selected_geometry` 会把 `_active_source_file` 切换为 `arena_abs` 并调用 `activate_context(arena_abs)`，同步恢复其 `<compiler>`、资产等派生数据（`xml_editor/viewmodel/scene_viewmodel.py:120`, `xml_editor/model/xml_parser.py:198`）。
  4. 随后新建几何体或编辑属性都会写入 `arena_group`，保存时 `save_loaded_sources` 逐一激活上下文并写回到原始 `humanoid.xml` / `arena.xml`，彼此独立（`xml_editor/viewmodel/scene_viewmodel.py:375`）。

### 保存与清理
- `SceneViewModel.save_loaded_sources` 在批量保存前逐文件调用 `XMLParser.activate_context(path)`，然后使用 `_collect_export_objects` 仅导出该文件下的真实几何体，未修改的节点继续沿用原上下文（`xml_editor/viewmodel/scene_viewmodel.py:375`, `xml_editor/viewmodel/scene_viewmodel.py:394`, `xml_editor/viewmodel/scene_viewmodel.py:386`）。
- `SceneViewModel.clear_scene` 会清理场景并重置解析上下文，防止旧文件残留（`xml_editor/viewmodel/scene_viewmodel.py:1335`）。
- `SceneViewModel.save_scene` 针对“另存为”场景时也复用 `_collect_export_objects`，避免多文件情形下重复包装根组（`xml_editor/viewmodel/scene_viewmodel.py:580`）。
- 任何时候保存成功后都会刷新当前上下文至缓存，确保下一次编辑仍基于最新数据（`xml_editor/model/xml_parser.py:913`）。
- 另存为场景时若存在未关联几何体，会仅导出这些对象并自动创建新的来源分组及资产上下文，其它已绑定文件仍按原路径保存，随后立即批量保存所有已打开的 XML，确保修改同步落盘（`xml_editor/viewmodel/scene_viewmodel.py:104`, `xml_editor/viewmodel/scene_viewmodel.py:580`, `xml_editor/viewmodel/scene_viewmodel.py:1010`, `xml_editor/main.py:372`）。
- 具体行为：`保存` 会逐来源文件写回原 XML；`另存为` 若所有对象都来自既有来源，则把全部几何体扁平化合并到新文件；若存在未归属对象，则新文件仅包含这些孤立几何体，随后 `save_loaded_sources` 仍会把其它来源的改动写回各自 XML（`xml_editor/main.py:512`, `xml_editor/viewmodel/scene_viewmodel.py:653`, `xml_editor/viewmodel/scene_viewmodel.py:666`）。
- 若场景中存在未归属或来源失效的几何体（`source_file` 为空或不在当前来源列表里），保存/另存为都会被禁止并提示用户先将对象拖入目标 XML 节点后再重试（`xml_editor/viewmodel/scene_viewmodel.py:366`, `xml_editor/main.py:308`）。
- 多文件场景下，保存/另存为必须先在层级结构中选择目标 XML 根节点；对应按钮会只影响该文件，并通过 `SceneViewModel.save_source_file` / `duplicate_source_file` 执行实际写盘，未选中根节点时会给出提示阻止操作（`xml_editor/main.py:435`, `xml_editor/main.py:463`, `xml_editor/viewmodel/scene_viewmodel.py:408`, `xml_editor/viewmodel/scene_viewmodel.py:432`）。
- 无论单文件还是多文件，来源根节点（`GeometryGroup`）会始终保留在层级树中，即便其子节点被删除，这样可以继续针对该 XML 执行保存或追加操作，直到用户明确删除该根节点（`xml_editor/viewmodel/scene_viewmodel.py:300`, `xml_editor/viewmodel/scene_viewmodel.py:523`）。

### 情况覆盖
- 支持同时加载多个 MJCF，且在保存其中任意文件时，其它文件的 `<asset>`、`<actuator>` 等段均保持原貌，不会被最近一个文件覆盖。
- 当用户删除或新增几何体时，`source_file` 信息被保留下来，保存时仍能回写到对应 XML；若有未关联来源的几何体，则提示用户先另存或指定目标。
- 当用户选择另存为时，新文件只包含新建对象，原有多个来源文件则继续按原路径各自保存，避免重复内容。
- 将未归属的几何体拖入某个来源组（或其子节点）时，会立即根据新父节点的 `source_file` 更新该对象及其子树的来源，从而可以直接保存到目标 XML，无需再遇到“未关联对象需另存”的提示（`xml_editor/viewmodel/hierarchy_viewmodel.py:454`）。
- 删除某个来源组会同步清理其解析上下文并重置当前活动来源，避免后续新建几何体继续指向已移除的文件（`xml_editor/viewmodel/scene_viewmodel.py:507`）。

### 操作示例
1. 同时打开 `A.xml` 与 `B.xml`，修改 `A` 中的一个 geom，保存后 `B.xml` 的自定义段仍保持不变。
2. 删除场景并重新加载另一组文件时，旧上下文会被清空，避免历史路径污染新会话。

---

## 需求 14：几何体物理属性的查看与编辑

### 核心实现
- 解析阶段：`XMLParser._load_mujoco_format` 将 `<geom>` 中的物理字段（如 `friction`, `solimp` 等）完整保存在 `Geometry.mjcf_attrs`，未出现的字段保持缺省，便于区分原始值与新增值（`xml_editor/model/xml_parser.py:552`, `xml_editor/model/xml_parser.py:632`）。
- 属性面板：`PropertyView` 预先为 `PHYSICS_ATTRS` 中的所有字段构建输入框，并在选中任意非关节几何体时全部展示；若某字段在 MJCF 中缺失，则以空白表示，仍可在 UI 中补录（`xml_editor/view/property_view.py:403`, `xml_editor/view/property_view.py:609`）。
- 数据流：`PropertyViewModel.get_physics_attributes` 从 `mjcf_attrs` 里筛选 `PHYSICS_ATTRS`，返回给界面；用户编辑后通过 `set_property("physics_attr_*", value)` 写回，空字符串代表删除该项（`xml_editor/viewmodel/property_viewmodel.py:145`, `xml_editor/viewmodel/property_viewmodel.py:351`）。
- Joint 属性沿用原有逻辑：无论 MJCF 是否声明 `range/ref` 等字段，界面都会显示输入框并以空值表示缺省，方便与物理属性保持一致的使用体验（`xml_editor/view/property_view.py:423`, `xml_editor/viewmodel/property_viewmodel.py:326`）。
- 新建几何体：`SceneViewModel._apply_default_physics_attrs` 仅负责保证 `mjcf_attrs` 是字典，不再填充密度、摩擦等默认值，便于与 MuJoCo 原生行为保持一致（`xml_editor/viewmodel/scene_viewmodel.py:332`）。
- 导出阶段：`XMLParser._add_object_to_mujoco` 在写回物理字段时会跳过空值（`str(val).strip()` 为空直接忽略），从而满足“空属性不落盘”的约束；既有字段保持原格式，新字段则依据当前输入写入（`xml_editor/model/xml_parser.py:1259`）。

### 情况覆盖
- 对于从 XML 导入的几何体，物理属性初始值完全来源于原文件；若用户不改动，导出时会原样保留。
- 在编辑过程中，将某个字段清空即可从 `mjcf_attrs` 中移除，导出时不再生成该属性。
- 新建几何体不会再自动补齐任何物理参数，所有字段保持空白，用户需要时再手动填入。

### 操作示例
1. 打开包含 `friction="1 0.5 0.5"` 的 XML，属性面板显示相同值；修改为 `0.8 0.4 0.4` 后保存，导出的 XML 中只更新 friction 这一项。
2. 创建新的 box，默认带有密度/摩擦等参数；把某个字段清空后保存，输出 XML 将不再包含该字段。

---

以上内容现已覆盖 14 项需求的实现链路与关键代码，可作为调试和扩展的权威参考。

---

## 需求 15：Joint 的创建、显示与编辑

### 载入与可视化
- `_load_mujoco_format` 在解析 `<body>` 时同步解析其 `<joint>` 子节点，生成 `Geometry(type="joint")` 并挂载到对应的 `GeometryGroup`；若 joint 位于 `worldbody` 顶层，则作为独立对象加入场景（`xml_editor/model/xml_parser.py:449`, `xml_editor/model/xml_parser.py:861`）。
- 加载阶段会将 `axis` 归一化并调用 `_axis_to_euler` 计算局部旋转，`length`（若存在）用于推导可视半长度并写入 `joint_length`，便于后续调整（`xml_editor/model/xml_parser.py:876`）。
- OpenGL 层通过 `_draw_joint` 将关节绘制为加粗轴线和端点小球，并关闭光照以保持类型颜色（hinge 为黄、slide 为蓝），确保与几何体渲染隔离（`xml_editor/view/opengl_view.py:783`）。

### 创建流程
- 控制面板提供“关节·铰链 / 关节·滑动”按钮，拖拽到场景时 `_create_geometry_from_drag` 会实例化 joint 几何体、写入默认 `joint_attrs['type']` 并设置配色（`xml_editor/view/control_panel.py:262`, `xml_editor/view/opengl_view.py:2644`）。
- 新建关节会立即被选中；`SceneViewModel.selected_geometry` setter 自动切换到平移模式并调用 `_auto_update_gizmo_size`，使 Gizmo 与关节长度匹配（`xml_editor/viewmodel/scene_viewmodel.py:110`）。
- 在拖拽过程中，`_draw_joint_preview` 以半透明圆柱展示即将创建的关节，用于确认长度与方向（`xml_editor/view/opengl_view.py:268`, `xml_editor/view/opengl_view.py:2737`）。

### 属性编辑与交互
- `PropertyView` 针对 joint 展示类型、轴向、长度及扩展属性，仍使用高精度 SpinBox；`propertyChanged` 信号连到 `PropertyViewModel.set_property`（`xml_editor/view/property_view.py:423`, `xml_editor/view/property_view.py:432`）。
- `PropertyViewModel` 对 joint 的处理包括：
  * 依据 `joint_length` 更新可视尺寸及 `joint_attrs['length']`（若原 XML 带该字段）；
  * 正规化轴向并刷新 `joint_attrs['axis']`，确保导出值始终为单位向量；
  * 切换 joint 类型时更新颜色、`joint_attrs['type']` 并广播变化（`xml_editor/viewmodel/property_viewmodel.py:256`, `xml_editor/viewmodel/property_viewmodel.py:281`, `xml_editor/viewmodel/property_viewmodel.py:339`）。
- 轴向编辑现已与旋转解耦：调整 `joint_axis` 仅更新 UI 与 `joint_attrs['axis']`，不会再同步修改 `rotation`，反之旋转也不再回写轴向，避免两者互相覆盖（`xml_editor/viewmodel/property_viewmodel.py:282`, `xml_editor/model/geometry.py:145`）。
- 轴向输入改为单行文本（如 `0 1 0`），互不影响；若清空字段，界面保持空值并导出时省略 `<axis>`，新建 Joint 也不再写入默认 `1 0 0`。关节在视图中仅以十字标记显示位置，不再根据 axis 旋转，避免引入额外歧义（`xml_editor/view/property_view.py:436`, `xml_editor/viewmodel/property_viewmodel.py:272`, `xml_editor/model/xml_parser.py:930`, `xml_editor/view/opengl_view.py:880`）。
- 鼠标点击 joint 或其 Gizmo 会通过 `OpenGLView.mousePressEvent` 触发 selection；Gizmo 拾取优先调用 `_pick_controller`，结合世界尺度推导拾取半径，未命中时退回射线检测。拖动成功后根据当前模式更新 `position/rotation/size` 并通过 `SceneViewModel.notify_object_changed` 同步至 UI（`xml_editor/view/opengl_view.py:1266`, `xml_editor/view/opengl_view.py:1853`, `xml_editor/viewmodel/scene_viewmodel.py:747`）。

### 保存流程与兼容性
- 导出时会先清空原 body 中旧的 `<joint>` 节点，再基于当前场景生成新的 joint 结构，避免旧版本未考虑 joint 导致的重复或丢失（`xml_editor/model/xml_parser.py:1118`）。
- `_add_object_to_mujoco` 为 joint 写回 `type/pos/axis/name` 等属性，`axis` 始终使用单位向量；同时保留 `joint_attrs` 中的拓展字段（`range/ref` 等），并在 hinge 类型下自动把角度字段转换为弧度，slide 类型仍保持线性单位（`xml_editor/model/xml_parser.py:1144`, `xml_editor/model/xml_parser.py:1182`）。
- 若原 `<compiler angle>` 为 degree，保存阶段兜底的 `_convert_joint_angles_to_radian` 会在仍有残留度数时批量转弧度，保证旧场景与 V1.0 的统一弧度制兼容（`xml_editor/model/xml_parser.py:1425`）。

### 验证结论
- 经测试：导入旧 MJCF（只含 hinge/slide）、混合多个 joint 并编辑属性、以及从控制面板新建 joint 再另存，输出文件均能保持正确的 `<joint>` 元素序列，关节可视化与属性面板状态同步，未发现新的兼容性或保存问题。


### 属性
几何（geom / 盒子 Box）
字段	MuJoCo 对应/含义	对止摆是否有用
类型 / 名称 / 可见 / 颜色 / 透明度	纯可视化	否
condim	接触维度（1=法向，3=法向+切向等）。影响接触求解模型	基本无关（除非它在与别的物体持续摩擦）
friction	接触摩擦系数三元组：slide spin roll（滑动/自旋/滚动）	只有当它与别的物体接触时才影响阻尼；单体悬空不影响摆动
solimp / solref	接触（或限制）求解器参数：软硬度/时间常数等	与简单止摆无关
solmix	多接触混合参数（可选）	无关
margin	接触提前量（距离小于 margin 就生成接触）	无关
gap	允许穿透的间隙（软约束）	无关
density / mass	密度/质量（二选一；留空用默认密度）	质量越大，角速度衰减慢一点（动量大），但不是治本手段
stiffness / damping（geom）	流体/场阻尼相关的旧字段；一般不在 geom 上用	不建议用来止摆
viscosity	流体阻力系数（空气/水阻的简化）	可以让运动慢慢衰减，但常用于“空气阻力”场景

关键点：几何参数大多影响接触和质量。你的摆动来自“重力+关节自由度+几何与铰点不重合”，因此该改关节，不是改 geom。

关节（joint / Hinge）
字段	MuJoCo 对应/含义	对止摆是否有用
类型（Hinge）	type="hinge" 单自由度转动	—
轴 axis	转轴方向（如 0 1 0）	不正确会导致奇怪转动；无直接止摆效果
长度 length	仅用于渲染关节轴向“柄”的长度（可视化）	否
range	角度限制（若 limited="true"）	有时能把摆幅限制住，但不让其停止
damping	速度阻尼（粘滞摩擦，∝ 角速度）	最直接有效；设大一些会很快停
stiffness	关节弹簧刚度（与 ref 或 springref 形成扭簧）	有效；能把角度拉回参考位姿
springref / ref	弹簧的参考角度：关节偏离该角会产生回复力矩	配合 stiffness 一起用
frictionloss	库仑摩擦阈值（静摩擦/干摩擦，克服它才动）	很有用；加大后小幅摆会被“卡住”
armature	转子惯量（增加相当于给关节轴加惯性）	只改变动态“厚重感”，不直接止摆
margin	关节限位的缓冲（limit 软化）	无关（除非用 range）

关键点：damping（粘滞）、frictionloss（干摩擦）、stiffness+ref（弹簧）是“止摆三件套”。
