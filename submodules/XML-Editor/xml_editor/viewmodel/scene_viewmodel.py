"""
场景视图模型

作为场景数据和视图之间的桥梁，处理场景操作的业务逻辑
"""

from PyQt5.QtCore import QObject, pyqtSignal
import numpy as np
import os
import shutil

from ..model.geometry import (
    Geometry, GeometryGroup, GeometryType, 
    Material, OperationMode
)
from ..utils.mesh_loader import load_mesh as load_mesh_file
from ..model.xml_parser import XMLParser
from ..model.raycaster import GeometryRaycaster, RaycastResult

class SceneViewModel(QObject):
    """
    场景视图模型类
    
    处理场景数据的加载、修改、保存，并通知视图更新
    """
    # 信号定义
    geometriesChanged = pyqtSignal()  # 几何体列表发生变化
    selectionChanged = pyqtSignal(object)  # 选中对象变化
    operationModeChanged = pyqtSignal(object)  # 操作模式变化
    objectChanged = pyqtSignal(object)  # 对象属性变化
    coordinateSystemChanged = pyqtSignal(bool)  # 坐标系变化信号
    geometryAdded = pyqtSignal(object)  # 几何体添加信号
    geometryDeleted = pyqtSignal(object)  # 几何体删除信号
    geometryChanged = pyqtSignal(object)  # 几何体变化信号
    positionChanged = pyqtSignal(object)  # 位置变化信号
    rotationChanged = pyqtSignal(object)  # 旋转变化信号
    scaleChanged = pyqtSignal(object)     # 缩放变化信号
    sourcesChanged = pyqtSignal(list)     # 已加载源XML路径列表发生变化

    gizmoSizeChanged = pyqtSignal(float)   # 全局 gizmo 大小变化
    gsBackgroundsChanged = pyqtSignal(list, object)  # 高斯背景变化 (entries, active_key)
    
    def __init__(self):
        super().__init__()
        self._geometries = []  # 场景中的几何体列表
        self._selected_geo = None  # 当前选中的几何体
        self._operation_mode = OperationMode.OBSERVE  # 当前操作模式
        self._raycaster = None  # 射线投射器
        self._camera_config = {
            'position': np.array([0, 0, 10]),
            'target': np.array([0, 0, 0]),
            'up': np.array([0, 1, 0]),
            'view_matrix': np.eye(4),
            'projection_matrix': np.eye(4)
        }
        self._use_local_coords = True
        self.hierarchyViewModel = None  # 添加 hierarchyViewModel 属性


        self.global_gizmo_size_world = 1.0
        self._gs_backgrounds = []
        self._gs_active_background = None
        self._gs_edit_history = []
        self.control_viewmodel = None

        self._source_groups = {}
        self._active_source_file = None
    
    @property
    def geometries(self):
        """获取所有几何体"""
        return self._geometries
    
    @geometries.setter
    def geometries(self, value):
        """设置几何体列表并发出通知"""
        self._geometries = value
        self._source_groups = {}
        for obj in self._geometries:
            source = getattr(obj, 'source_file', None)
            if getattr(obj, '_is_source_root', False) and source:
                self._source_groups[os.path.abspath(source)] = obj
        self._emit_sources_changed()
        self._update_raycaster()
        self.geometriesChanged.emit()
    
    @property
    def selected_geometry(self):
        """获取当前选中的几何体"""
        return self._selected_geo
    
    @selected_geometry.setter
    def selected_geometry(self, value):
        """设置选中的几何体并发出通知"""
        # 先取消之前选中的几何体
        if self._selected_geo:
            self._selected_geo.selected = False

        self._selected_geo = value

        # 标记新选中的几何体
        if self._selected_geo:
            self._selected_geo.selected = True
            # 选中时先切到平移模式，方便立即操作，同时刷新 Gizmo 尺度
            if self._operation_mode == OperationMode.OBSERVE:
                self.operation_mode = OperationMode.TRANSLATE
            self._auto_update_gizmo_size(self._selected_geo)

        source_candidate = None
        if self._selected_geo:
            source_candidate = getattr(self._selected_geo, 'source_file', None)
            if source_candidate is None and getattr(self._selected_geo, 'parent', None) is not None:
                source_candidate = getattr(self._selected_geo.parent, 'source_file', None)
        self._active_source_file = source_candidate

        if self._active_source_file:
            XMLParser.activate_context(self._active_source_file)
        else:
            XMLParser.activate_context(None)

        # 发出选择变化信号 - 但不触发几何体变化信号
        self.selectionChanged.emit(self._selected_geo)
    
    @property
    def transform_mode(self):
        """获取当前变换模式"""
        return self._operation_mode
    
    @transform_mode.setter
    def transform_mode(self, value):
        """设置变换模式并发出通知"""
        self._operation_mode = value
        self.operationModeChanged.emit(value)
    
    @property
    def operation_mode(self):
        """获取当前操作模式"""
        return self._operation_mode
    
    @operation_mode.setter
    def operation_mode(self, value):
        """设置操作模式并发出变换模式变化通知"""
        old_value = self._operation_mode
        self._operation_mode = value
        # 如果操作模式变化，也发出变换模式变化信号
        if old_value != value:
            self.operationModeChanged.emit(value)
    
    @property
    def use_local_coords(self):
        """获取当前坐标系模式，True表示局部坐标系，False表示全局坐标系"""
        return self._use_local_coords
    
    @use_local_coords.setter
    def use_local_coords(self, value):
        """设置坐标系模式"""
        if self._use_local_coords != value:
            self._use_local_coords = value
            # 通知OpenGL视图更新坐标系模式
            self.coordinateSystemChanged.emit(value)
    
    def set_camera_config(self, config):
        """设置摄像机配置"""
        self._camera_config.update(config)
        self._update_raycaster()
    
    def _update_raycaster(self):
        """更新射线投射器"""
        if self._raycaster:
            self._raycaster.update_camera(self._camera_config)
            self._raycaster.update_geometries(self._geometries)
        else:
            self._raycaster = GeometryRaycaster(self._camera_config, self._geometries)

    def _emit_sources_changed(self):
        """广播当前已加载的源XML列表"""
        try:
            paths = [os.path.abspath(path) for path in self._source_groups.keys()]
        except Exception:
            paths = list(self._source_groups.keys())
        self.sourcesChanged.emit(paths)
    
    def create_geometry(self, geo_type, name=None, position=(0, 0, 0), size=(1, 1, 1), rotation=(0, 0, 0), parent=None, source_file=None):
        """
        创建新的几何体
        
        参数:
            geo_type: 几何体类型
            name: 几何体名称（如果为None则自动生成）
            position: 位置坐标
            size: 尺寸
            rotation: 旋转角度
            parent: 父对象
        
        返回:
            创建的几何体对象
        """
        # 自动生成名称
        existing_names = {geo.name for geo in self.get_all_geometries()}
        if name is None:
            base_name = geo_type.name.capitalize()
            name = self._generate_unique_name(base_name)
        elif name in existing_names:
            name = self._generate_unique_name(name)
        
        # 创建几何体
        geometry = Geometry(
            geo_type=geo_type.value,
            name=name,
            position=position,
            size=size,
            rotation=rotation,
            parent=parent
        )

        # 标记为编辑器新建对象，导出时保留 name 属性
        geometry._mjcf_had_name = True
        if geo_type == GeometryType.JOINT:
            geometry.joint_angle_mode = 'radian'

        if source_file is None:
            if parent is not None:
                source_file = getattr(parent, 'source_file', None)
            if source_file is None:
                source_file = self._active_source_file

        geometry.source_file = source_file

        self._apply_default_physics_attrs(geometry)
        
        # 添加到场景中
        if parent:
            parent.add_child(geometry)
        else:
            target_group = None
            if source_file and source_file in self._source_groups:
                target_group = self._source_groups[source_file]
            if target_group:
                target_group.add_child(geometry)
            else:
                self._geometries.append(geometry)

        # 触发更新
        self._update_raycaster()
        self.geometriesChanged.emit()

        return geometry

    def _generate_unique_name(self, base_name: str) -> str:
        """生成在当前场景中唯一的名称"""
        existing = {geo.name for geo in self.get_all_geometries()}
        if base_name not in existing:
            return base_name

        index = 1
        while f"{base_name}_{index}" in existing:
            index += 1
        return f"{base_name}_{index}"

    def create_mesh_from_path(self, path: str, name=None, parent=None):
        """
        从外部 OBJ/STL 文件构建 mesh 几何体：
        - 读取三角面与法线
        - 计算包围盒，设置初始位置/尺寸
        - 注册 mesh 资产，方便保存与后续缩放
        """
        try:
            triangles, normals = load_mesh_file(path)
        except Exception as exc:
            print(f"[Mesh] 加载失败: {path} -> {exc}")
            return None

        if triangles is None or len(triangles) == 0:
            print(f"[Mesh] 文件无有效三角形: {path}")
            return None

        tris_np = np.asarray(triangles, dtype=np.float32)
        norms_np = None if normals is None else np.asarray(normals, dtype=np.float32)

        # 基于三角面顶点计算包围盒，确定初始中心和尺寸
        pts = tris_np.reshape(-1, 3)
        mins = pts.min(axis=0)
        maxs = pts.max(axis=0)
        center = (mins + maxs) * 0.5
        half_extents = np.maximum((maxs - mins) * 0.5, 1e-5)

        # 将局部几何体平移到以原点为中心，便于后续变换
        tris_local = (tris_np - center.reshape(1, 1, 3)).astype(np.float32)

        base = name or os.path.splitext(os.path.basename(path))[0] or "mesh"
        unique_name = self._generate_unique_name(base)

        geometry = Geometry(
            geo_type=GeometryType.MESH.value,
            name=unique_name,
            position=tuple(center.tolist()),
            size=tuple(half_extents.tolist()),
            rotation=(0.0, 0.0, 0.0),
            parent=parent
        )

        geometry.material.color = np.array([0.8, 0.8, 0.8, 1.0], dtype=np.float32)
        geometry.mesh_model_triangles = tris_local
        geometry.mesh_model_normals = None if norms_np is None or norms_np.size == 0 else norms_np.astype(np.float32)
        geometry.mesh_path = path
        geometry.mesh_asset_scale = [1.0, 1.0, 1.0]
        geometry.mesh_geom_scale = [1.0, 1.0, 1.0]
        geometry.mesh_name = unique_name
        geometry.mjcf_attrs = {"mesh": unique_name}
        geometry.mesh_origin_offset = center.astype(np.float32).tolist()
        geometry._mjcf_had_name = True

        geometry.source_file = self._active_source_file
        self._apply_default_physics_attrs(geometry)
        if norms_np is None or norms_np.size == 0:
            self._ensure_mesh_normals(geometry)

        # 向 XMLParser 注册 mesh 资产，保存/导出时可以引用同一资源
        XMLParser.register_mesh_asset(unique_name, path, [1.0, 1.0, 1.0], has_scale_attr=False)

        if parent:
            parent.add_child(geometry)
        else:
            target_group = None
            if geometry.source_file and geometry.source_file in self._source_groups:
                target_group = self._source_groups[geometry.source_file]
            if target_group:
                target_group.add_child(geometry)
            else:
                self._geometries.append(geometry)

        if hasattr(geometry, "update_transform_matrix"):
            geometry.update_transform_matrix()

        self._update_raycaster()
        self.geometriesChanged.emit()
        self.geometryAdded.emit(geometry)
        return geometry

    def _apply_default_physics_attrs(self, geometry):
        try:
            if geometry is None:
                return
            geo_type = getattr(geometry, 'type', None)
            if geo_type in ('group', GeometryType.JOINT.value):
                return

            attrs = getattr(geometry, 'mjcf_attrs', None)
            if not isinstance(attrs, dict):
                geometry.mjcf_attrs = {}
        except Exception as exc:
            print(f"[PhysicsAttrs] 设置默认物理属性失败: {exc}")

    @staticmethod
    def _compute_face_normals(tris):
        if tris is None:
            return None
        tris_arr = np.asarray(tris, dtype=np.float32)
        if tris_arr.ndim != 3 or tris_arr.shape[1] != 3 or tris_arr.shape[2] != 3:
            return None
        count = tris_arr.shape[0]
        if count == 0:
            return np.empty((0, 3), dtype=np.float32)
        v0 = tris_arr[:, 0, :]
        v1 = tris_arr[:, 1, :]
        v2 = tris_arr[:, 2, :]
        normals = np.cross(v1 - v0, v2 - v0)
        lengths = np.linalg.norm(normals, axis=1, keepdims=True)
        lengths[lengths < 1e-12] = 1.0
        normals = normals / lengths
        return normals.astype(np.float32)

    def _ensure_mesh_normals(self, geometry):
        if geometry is None:
            return
        geo_type = getattr(geometry, 'type', None)
        if geo_type == GeometryType.MESH.value:
            tris = getattr(geometry, 'mesh_model_triangles', None)
            normals = getattr(geometry, 'mesh_model_normals', None)
            needs = False
            if tris is None or len(tris) == 0:
                needs = False
            else:
                if normals is None:
                    needs = True
                else:
                    try:
                        needs = len(normals) != len(tris)
                    except Exception:
                        needs = True
            if needs:
                computed = self._compute_face_normals(tris)
                if computed is not None:
                    geometry.mesh_model_normals = computed
        children = getattr(geometry, 'children', None)
        if children:
            for child in children:
                self._ensure_mesh_normals(child)

    def _wrap_loaded_geometries(self, filename, geometries):
        group_name = os.path.splitext(os.path.basename(filename))[0]
        file_group = GeometryGroup(name=group_name, position=(0, 0, 0), rotation=(0, 0, 0))
        file_group.source_file = filename
        file_group._is_source_root = True
        self._set_source_recursive(file_group, filename)

        for geo in geometries:
            file_group.add_child(geo)
            self._set_source_recursive(geo, filename)
            self._ensure_mesh_normals(geo)

        if hasattr(file_group, 'update_transform_matrix'):
            file_group.update_transform_matrix()

        return file_group

    def _set_source_recursive(self, geometry, source_file):
        try:
            geometry.source_file = source_file
            if hasattr(geometry, '_is_source_root') and not source_file:
                geometry._is_source_root = False
        except Exception:
            pass
        if hasattr(geometry, 'children') and geometry.children:
            for child in geometry.children:
                self._set_source_recursive(child, source_file)

    def _collect_export_objects(self, objects):
        # 递归展开 source group 下的真实对象，忽略仅用于分组的根节点
        export_list = []
        for obj in objects:
            if getattr(obj, '_is_source_root', False):
                export_list.extend(self._collect_export_objects(obj.children))
            else:
                export_list.append(obj)
        return export_list

    def get_loaded_sources(self):
        return list(self._source_groups.keys())

    def save_loaded_sources(self):
        failures = []
        for path, group in self._source_groups.items():
            export_objects = self._collect_export_objects(group.children)
            try:
                # 保存前激活对应文件的解析上下文，避免跨文件串改
                XMLParser.activate_context(path)
                ok = XMLParser.export_mujoco_xml(path, export_objects)
            except Exception:
                ok = False
            if not ok:
                failures.append(path)
        return failures

    def save_source_file(self, source_path):
        if not source_path:
            return False
        try:
            abs_path = os.path.abspath(source_path)
        except Exception:
            abs_path = source_path

        group = self._source_groups.get(abs_path)
        if group is None:
            return False

        export_objects = self._collect_export_objects(group.children)
        try:
            XMLParser.activate_context(abs_path)
            ok = XMLParser.export_mujoco_xml(abs_path, export_objects)
        except Exception:
            ok = False
        if ok:
            try:
                XMLParser._store_context_for_file(abs_path)
            except Exception:
                XMLParser.activate_context(abs_path)
        return ok

    def duplicate_source_file(self, source_path, target_path, *, adopt=False):
        if not source_path or not target_path:
            return False

        try:
            src_abs = os.path.abspath(source_path)
        except Exception:
            src_abs = source_path

        group = self._source_groups.get(src_abs)
        if group is None:
            return False

        export_objects = self._collect_export_objects(group.children)

        try:
            XMLParser.activate_context(src_abs)
        except Exception:
            XMLParser.activate_context(None)

        try:
            target_abs = os.path.abspath(target_path)
        except Exception:
            target_abs = target_path

        preserve_aux = True
        if os.path.exists(target_abs) and target_abs != src_abs:
            try:
                os.remove(target_abs)
            except Exception:
                preserve_aux = False

        ok = XMLParser.export_mujoco_xml(
            target_path,
            export_objects,
            preserve_auxiliary=preserve_aux
        )
        if not ok:
            try:
                XMLParser.activate_context(src_abs)
            except Exception:
                pass
            return False

        try:
            XMLParser._store_context_for_file(os.path.abspath(target_path))
        except Exception:
            XMLParser.activate_context(target_path)

        if adopt:
            self._remap_source_group(src_abs, target_path)
        else:
            try:
                XMLParser.activate_context(src_abs)
            except Exception:
                pass
        return True

    def get_active_source_file(self):
        return self._active_source_file

    def get_source_group(self, source_file):
        if not source_file:
            return None
        return self._source_groups.get(os.path.abspath(source_file))

    def has_unsourced_geometry(self):
        # 若当前场景尚无已关联的来源文件，则允许用户通过保存/另存为建立新来源
        if not self._source_groups:
            return False
        valid_sources = {os.path.abspath(path) for path in self._source_groups.keys()}
        for geo in self.get_all_geometries():
            source = getattr(geo, 'source_file', None)
            if not source:
                return True
            try:
                abs_source = os.path.abspath(source)
            except Exception:
                return True
            if abs_source not in valid_sources:
                return True
        return False

    def _auto_update_gizmo_size(self, geometry):
        """
        根据当前对象的包围尺寸自动调整 Gizmo 的世界尺度
        1）优先用网格三角数据的包围盒（最精准）；
        2）无网格或过小时，用原生几何的 size（半尺寸）推直径；
        3）再不行，默认 1.0。
        这保证任何对象都能得到一个合理的 Gizmo 尺度。
        
        """
        if geometry is None or getattr(geometry, "type", "") == "group":
            return

        max_dim = None

        tris = getattr(geometry, "mesh_model_triangles", None)
        if tris is not None and len(tris) > 0:
            pts = np.asarray(tris, dtype=np.float32).reshape(-1, 3)
            bounds = np.max(pts, axis=0) - np.min(pts, axis=0)
            # mesh 以三角面包围盒的最大边长作为尺度参考
            max_dim = float(np.max(np.abs(bounds)))

        if max_dim is None or max_dim <= 1e-6:
            size_attr = getattr(geometry, "size", None)
            if size_attr is not None:
                size_arr = np.abs(np.asarray(size_attr, dtype=float))
                if size_arr.size > 0:
                    # 原生 geom 以 size（半尺寸）推导直径，避免过小 gizmo
                    max_dim = float(np.max(size_arr) * 2.0)

        if max_dim is None or max_dim <= 1e-6:
            # 兜底：给出一个可见的默认尺度
            max_dim = 1.0

        self.set_global_gizmo_size_world(max_dim)

    def create_group(self, name=None, position=(0, 0, 0), rotation=(0, 0, 0), parent=None, source_file=None):
        """
        创建新的几何体组
        
        参数:
            name: 组名称（如果为None则自动生成）
            position: 位置坐标
            rotation: 旋转角度
            parent: 父对象
        
        返回:
            创建的几何体组对象
        """
        # 自动生成名称
        existing_names = {geo.name for geo in self.get_all_geometries()}
        if name is None:
            name = self._generate_unique_name("Group")
        elif name in existing_names:
            name = self._generate_unique_name(name)
        
        # 创建几何体组
        group = GeometryGroup(
            name=name,
            position=position,
            rotation=rotation,
            parent=parent
        )
        if source_file is None:
            if parent is not None:
                source_file = getattr(parent, 'source_file', None)
            else:
                source_file = self._active_source_file
        group.source_file = source_file
        if source_file:
            self._set_source_recursive(group, source_file)
        
        # 添加到场景中
        if parent:
            parent.add_child(group)
        else:
            target_group = None
            if source_file and source_file in self._source_groups:
                target_group = self._source_groups[source_file]
            if target_group:
                target_group.add_child(group)
            else:
                self._geometries.append(group)
        
        # 触发更新
        self.geometriesChanged.emit()
        
        return group
    
    def remove_geometry(self, geometry):
        """
        从场景中移除几何体
        
        参数:
            geometry: 要移除的几何体
        """
        # 如果是当前选中的几何体，先取消选中
        if self._selected_geo == geometry:
            self.selected_geometry = None
        
        # 从父对象中移除
        if geometry.parent:
            geometry.parent.remove_child(geometry)
        # 从顶层列表中移除
        elif geometry in self._geometries:
            self._geometries.remove(geometry)

        sources_changed = False
        if getattr(geometry, '_is_source_root', False):
            source = getattr(geometry, 'source_file', None)
            if source:
                abs_source = os.path.abspath(source)
                if abs_source in self._source_groups:
                    del self._source_groups[abs_source]
                    sources_changed = True
                XMLParser._context_by_file.pop(abs_source, None)
                if self._active_source_file and os.path.abspath(self._active_source_file) == abs_source:
                    self._active_source_file = None
                    XMLParser.activate_context(None)

        if sources_changed:
            self._emit_sources_changed()

        # 触发更新
        self.geometriesChanged.emit()
        self.geometryDeleted.emit(geometry)
        print(f"发射了 geometryDeleted 信号: {geometry.name}")
    
    def select_at(self, screen_x, screen_y, viewport_width, viewport_height):
        """
        在指定屏幕坐标选择几何体
        
        参数:
            screen_x: 屏幕X坐标
            screen_y: 屏幕Y坐标
            viewport_width: 视口宽度
            viewport_height: 视口高度
        
        返回:
            bool: 是否选中了几何体
        """
        result = self._raycaster.raycast(screen_x, screen_y, viewport_width, viewport_height)

        if result.is_hit():
            self.selected_geometry = result.geometry
            return True
        else:
            return False
    
    def clear_selection(self):
        """清除当前选择"""
        self.selected_geometry = None
    
    def load_scene(self, filename, *, append=False):
        """
        从文件加载场景
        
        参数:
            filename: 要加载的文件路径
            
        返回:
            bool: 是否成功加载
        """
        try:
            loaded_geos = XMLParser.load(filename)
            abs_path = os.path.abspath(filename)

            if not append:
                self._geometries = []
                self._source_groups = {}
                self._active_source_file = None

            file_group = self._wrap_loaded_geometries(abs_path, loaded_geos)
            self._geometries.append(file_group)
            self._source_groups[abs_path] = file_group
            self._active_source_file = abs_path
            self._emit_sources_changed()
            self._update_raycaster()
            self.geometriesChanged.emit()
            return True
        except Exception as e:
            print(f"加载场景失败: {e}")
            return False
    
    def _collect_unsourced_roots(self):
        """
        在场景里找出所有“没有 source_file 标记”的根节点。

        根节点的判定：自己没 source_file，且父节点要么不存在，要么已经有 source_file。

        产出：一组“无来源标记的最高层节点”。
        
        """
        roots = []
        seen = set()
        for geo in self.get_all_geometries():
            if getattr(geo, 'source_file', None) is not None:
                continue
            parent = getattr(geo, 'parent', None)
            if parent is not None and getattr(parent, 'source_file', None) is None:
                continue
            if id(geo) not in seen:
                roots.append(geo)
                seen.add(id(geo))
        return roots

    def _attach_unsourced_to_file(self, filename, unsourced_roots):
        """
        把这些节点统一挂到一个“以文件命名的根分组”下，并给它们整棵子树打上统一的 source_file，随后同步系统状态。

        把这些“散落的无来源根”统一**装进一个新建的 GeometryGroup（文件分组）**里：

        分组名来自文件名（避免重名会自动改名）；

        给这个分组打上 source_file=文件绝对路径，并设为 _is_source_root=True；

        把所有无来源根移出旧位置（从父里摘除、从顶层列表剔除），挂到这个分组下面；

        递归给它们及其子树都打上相同的 source_file；

        最后登记“这个文件路径 ↔ 分组”的映射，设为当前激活文件，上下文切换（XMLParser），并发出“来源变更/几何变更”的事件、更新拾取器。
        
        把散落的无来源对象变成有清晰来源、结构规整的一棵大树，便于管理与后续操作。
        
        """
        if not unsourced_roots:
            return
        abs_path = os.path.abspath(filename)

        base_name = os.path.splitext(os.path.basename(abs_path))[0] or "Scene"
        existing = {geo.name for geo in self.get_all_geometries()}
        if base_name in existing:
            group_name = self._generate_unique_name(base_name)
        else:
            group_name = base_name
        file_group = GeometryGroup(name=group_name, position=(0, 0, 0), rotation=(0, 0, 0))
        file_group.source_file = abs_path
        file_group._is_source_root = True

        self._set_source_recursive(file_group, abs_path)

        for obj in unsourced_roots:
            parent = getattr(obj, 'parent', None)
            if parent is not None and hasattr(parent, 'remove_child'):
                try:
                    parent.remove_child(obj)
                except Exception:
                    pass
            if obj in self._geometries:
                self._geometries.remove(obj)
            file_group.add_child(obj)
            self._set_source_recursive(obj, abs_path)

        self._geometries.append(file_group)
        self._source_groups[abs_path] = file_group
        self._active_source_file = abs_path
        XMLParser._store_context_for_file(abs_path)
        XMLParser.activate_context(abs_path)
        self._emit_sources_changed()
        self._update_raycaster()
        self.geometriesChanged.emit()

    def _remap_source_group(self, old_path, new_path):
        """将现有来源文件的根组迁移到新的文件路径"""
        if not new_path:
            return

        try:
            new_abs = os.path.abspath(new_path)
        except Exception:
            new_abs = new_path

        old_abs = None
        if old_path:
            try:
                old_abs = os.path.abspath(old_path)
            except Exception:
                old_abs = old_path

        if old_abs == new_abs and old_abs in self._source_groups:
            group = self._source_groups[old_abs]
            group_name = os.path.splitext(os.path.basename(new_abs))[0] or group.name
            if group.name != group_name:
                group.name = group_name
                self.geometriesChanged.emit()
            return

        group = None
        if old_abs and old_abs in self._source_groups:
            group = self._source_groups.pop(old_abs)
        elif new_abs in self._source_groups:
            group = self._source_groups[new_abs]

        if group is None:
            return

        group.source_file = new_abs
        group._is_source_root = True
        group_name = os.path.splitext(os.path.basename(new_abs))[0] or group.name
        if group.name != group_name:
            group.name = group_name

        self._set_source_recursive(group, new_abs)
        self._source_groups[new_abs] = group

        if self._active_source_file and old_abs and os.path.abspath(self._active_source_file) == old_abs:
            self._active_source_file = new_abs

        try:
            if old_abs:
                XMLParser._context_by_file.pop(old_abs, None)
            XMLParser._store_context_for_file(new_abs)
        except Exception:
            XMLParser.activate_context(new_abs)

        self._emit_sources_changed()
        self.geometriesChanged.emit()

    def _finalize_save_context(self, target_path, context_path):
        """保存完成后刷新上下文与来源映射"""
        if not target_path:
            return

        try:
            target_abs = os.path.abspath(target_path)
        except Exception:
            target_abs = target_path

        unsourced_roots = self._collect_unsourced_roots()
        if unsourced_roots:
            self._attach_unsourced_to_file(target_abs, unsourced_roots)
            return

        context_abs = None
        if context_path:
            try:
                context_abs = os.path.abspath(context_path)
            except Exception:
                context_abs = context_path

        if context_abs and context_abs != target_abs and len(self._source_groups) == 1:
            self._remap_source_group(context_abs, target_abs)
            return

        try:
            XMLParser._store_context_for_file(target_abs)
        except Exception:
            XMLParser.activate_context(target_abs)

        self._emit_sources_changed()

    def save_scene(self, filename, *, include_unsourced_only=False):
        """
        保存场景到文件
        
        参数:
            filename: 保存的文件路径
            
        返回:
            bool: 是否成功保存
        """
        try:
            # 根据文件扩展名决定保存格式
            _, ext = os.path.splitext(filename)
            
            if ext.lower() == '.xml':
                unsourced_roots = []
                if include_unsourced_only:
                    unsourced_roots = self._collect_unsourced_roots()
                    if not unsourced_roots:
                        export_objects = self._collect_export_objects(self._geometries)
                    else:
                        export_objects = self._collect_export_objects(unsourced_roots)
                else:
                    export_objects = self._collect_export_objects(self._geometries)

                context_path = None
                if not include_unsourced_only:
                    abs_target = os.path.abspath(filename)
                    if abs_target in self._source_groups:
                        context_path = abs_target
                    elif self._active_source_file and self._active_source_file in self._source_groups:
                        context_path = self._active_source_file
                    elif len(self._source_groups) == 1:
                        # 单文件场景另存为新路径时仍复用唯一来源的上下文，以保留 worldbody 之外的附加节点
                        context_path = next(iter(self._source_groups.keys()))

                abs_target = None
                try:
                    abs_target = os.path.abspath(filename)
                except Exception:
                    abs_target = filename

                if context_path:
                    XMLParser.activate_context(context_path)
                else:
                    # 另存为仅包含未关联对象，或当前无匹配上下文时清空解析状态，回落到全新 MJCF 根
                    XMLParser.activate_context(None)

                context_abs = None
                if context_path:
                    try:
                        context_abs = os.path.abspath(context_path)
                    except Exception:
                        context_abs = context_path

                preserve_aux = not include_unsourced_only
                if preserve_aux and abs_target and os.path.exists(abs_target):
                    if context_abs and context_abs != abs_target:
                        try:
                            os.remove(abs_target)
                        except Exception:
                            preserve_aux = False
                    elif context_abs is None:
                        preserve_aux = False

                ok = XMLParser.export_mujoco_xml(
                    filename,
                    export_objects,
                    preserve_auxiliary=preserve_aux
                )
                if ok:
                    if include_unsourced_only:
                        self._attach_unsourced_to_file(filename, unsourced_roots)
                    else:
                        self._finalize_save_context(filename, context_path)
                return ok
            else:
                return XMLParser.export_enhanced_xml(filename, self._geometries)
        except Exception as e:
            print(f"保存场景失败: {e}")
            return False
    
    def get_all_geometries(self):
        """
        获取场景中的所有几何体（包括嵌套在组中的）
        
        返回:
            list: 所有几何体的列表
        """
        result = []
        
        for geo in self._geometries:
            result.append(geo)
            if hasattr(geo, 'children') and geo.children:
                result.extend(self._get_children_recursive(geo))
        
        return result
    
    def _get_children_recursive(self, parent):
        """递归获取所有子对象"""
        result = []
        
        for child in parent.children:
            result.append(child)
            if hasattr(child, 'children') and child.children:
                result.extend(self._get_children_recursive(child))
        
        return result
    
    def update_geometry_property(self, geometry, property_name, value):
        """
        更新几何体的属性
        
        参数:
            geometry: 要更新的几何体
            property_name: 属性名称
            value: 新的属性值
            
        返回:
            bool: 是否成功更新
        """
        if not geometry:
            return False
        
        try:
            if property_name == 'name':
                geometry.name = value
            elif property_name == 'position':
                geometry.position = value
            elif property_name == 'size':
                geometry.size = value
            elif property_name == 'rotation':
                geometry.rotation = value
            elif property_name == 'color':
                geometry.material.color = value
            else:
                return False
                
            # 如果更新了变换相关属性，更新变换矩阵
            if property_name in ('position', 'size', 'rotation'):
                geometry.update_transform_matrix()
                
                # 递归更新所有子对象的变换矩阵
                if hasattr(geometry, 'children') and geometry.children:
                    for child in geometry.children:
                        self._update_transform_recursive(child)
            
            # 触发更新
            self.geometriesChanged.emit()
                
            return True
        except Exception as e:
            print(f"更新几何体属性失败: {e}")
            return False
    
    def _update_transform_recursive(self, geometry):
        """递归更新几何体及其子对象的变换矩阵"""
        if hasattr(geometry, 'update_transform_matrix'):
            geometry.update_transform_matrix()
        
        if hasattr(geometry, 'children') and geometry.children:
            for child in geometry.children:
                self._update_transform_recursive(child)
    
    def notify_object_changed(self, obj, *, emit_geometry=True):
        """
        通知对象属性已更改
        
        参数:
            obj: 被修改的对象
        """
        if obj is None:
            return

        try:
            # 更新所有变换矩阵，确保父子层次的缓存与最新属性一致
            self.update_all_transform_matrices()
        except Exception as exc:
            print(f"更新变换矩阵失败: {exc}")

        # 发出对象变化信号
        self.objectChanged.emit(obj)

        if emit_geometry:
            self.geometryChanged.emit(obj)

        # 如果对象在选择列表中，同时更新选择
        if obj == self._selected_geo:
            self.selectionChanged.emit(obj)
    
    def screen_to_world_ray(self, screen_x, screen_y, viewport_width, viewport_height):
        """
        从屏幕坐标计算世界空间射线
        
        参数:
            screen_x, screen_y: 屏幕坐标
            viewport_width, viewport_height: 视口尺寸
            
        返回:
            (ray_origin, ray_direction): 射线起点和方向
        """
        # 转换为归一化设备坐标(NDC)
        ndc_x = (2.0 * screen_x / viewport_width) - 1.0
        ndc_y = 1.0 - (2.0 * screen_y / viewport_height)  # Y轴方向翻转
        
        # 获取近平面和远平面上的点
        near_point = self.unproject_point(ndc_x, ndc_y, 0.0)
        far_point = self.unproject_point(ndc_x, ndc_y, 1.0)
        
        # 射线起点(相机位置)
        ray_origin = np.array(self._camera_config['position'])
        
        # 射线方向
        ray_direction = np.array(far_point) - np.array(near_point)
        ray_direction = ray_direction / np.linalg.norm(ray_direction)  # 归一化
        
        return (ray_origin, ray_direction)
    
    def unproject_point(self, ndc_x, ndc_y, ndc_z):
        """
        将归一化设备坐标转换为世界坐标
        
        参数:
            ndc_x, ndc_y, ndc_z: 归一化设备坐标(范围[-1,1])
            
        返回:
            世界坐标(x, y, z)
        """
        # 获取投影和视图矩阵
        proj_matrix = self._camera_config['projection_matrix']
        view_matrix = self._camera_config['view_matrix']
        
        # 计算视图投影矩阵的逆矩阵
        view_proj = np.matmul(proj_matrix, view_matrix)
        inv_view_proj = np.linalg.inv(view_proj)
        
        # 将NDC坐标转换为齐次裁剪空间坐标
        clip_coords = np.array([ndc_x, ndc_y, ndc_z, 1.0])
        
        # 应用逆矩阵转换为世界坐标
        world_coords = np.matmul(inv_view_proj, clip_coords)
        
        # 透视除法
        if world_coords[3] != 0:
            world_coords = world_coords / world_coords[3]
        
        return world_coords[:3]
    
    def get_geometry_at(self, screen_x, screen_y, viewport_width, viewport_height):
        """
        获取指定屏幕坐标处的几何体，但不改变选择状态
        
        参数:
            screen_x, screen_y: 屏幕坐标
            viewport_width, viewport_height: 视口尺寸
            
        返回:
            几何体对象或None
        """
        # 使用射线投射器进行检测
        result = self._raycaster.raycast(screen_x, screen_y, viewport_width, viewport_height)
        
        # 检查是否击中几何体
        if result and result.is_hit():
            return result.geometry
        
        # 没有击中任何几何体
        return None
    
    def update_all_transform_matrices(self):
        """
        更新场景中所有几何体的变换矩阵
        
        从根节点开始递归更新层次结构中的所有变换矩阵
        """
        # 获取场景中的顶层节点
        top_level_objects = self.get_all_geometries()
        
        # 递归更新所有节点的变换矩阵
        for obj in top_level_objects:
            self._update_object_transform_recursive(obj)
    
    def _update_object_transform_recursive(self, obj):
        """
        递归更新对象及其子对象的变换矩阵
        
        参数:
            obj: 要更新的对象
        """
        # 确保对象有update_transform_matrix方法
        if hasattr(obj, 'update_transform_matrix'):
            obj.update_transform_matrix()
        
        # 如果对象有子对象，递归更新它们
        if hasattr(obj, 'children'):
            for child in obj.children:
                self._update_object_transform_recursive(child)
    
    def add_geometry(self, geometry_type, name=None, position=None, size=None, rotation=None, parent=None):
        """
        添加几何体到场景
        
        参数:
            geometry_type: 几何体类型
            name: 几何体名称（可选）
            position: 位置（可选）
            size: 尺寸（可选）
            rotation: 旋转角度（可选）
            parent: 父对象（可选）
            
        返回:
            新创建的几何体
        """
        # 创建几何体
        geometry = self.create_geometry(geometry_type, name, position, size, rotation, parent)
        
        # 更新所有变换矩阵
        self.update_all_transform_matrices()
        
        # 发出场景变化信号
        self.geometriesChanged.emit()
        self.geometryAdded.emit(geometry)
        print(f"发射了 geometryAdded 信号: {geometry.name}")
        
        return geometry
    
    def move_geometry(self, geometry, new_position):
        """
        移动几何体到新位置

        参数:
            geometry: 要移动的几何体
            new_position: 新位置坐标
        """
        if geometry:
            geometry.position = new_position
            
            # 更新所有变换矩阵
            self.update_all_transform_matrices()
            
            # 通知对象已更改
            self.notify_object_changed(geometry)

    def is_mesh_scale_editable(self, geometry):
        """判断 mesh 资产是否允许在属性面板调节缩放"""
        if geometry is None or getattr(geometry, 'mesh_name', None) is None:
            return False
        asset = XMLParser._mesh_assets.get(geometry.mesh_name)
        if not asset:
            return True
        return True

    def update_mesh_asset_scale(self, geometry, new_scale):
        """修改 mesh 资产的统一缩放，并同步刷新所有引用该资产的几何体"""
        if geometry is None or getattr(geometry, 'mesh_name', None) is None:
            return False

        try:
            scale_arr = np.array(new_scale, dtype=float)
        except Exception:
            return False

        if scale_arr.shape[0] != 3:
            return False

        scale_list = scale_arr.tolist()
        mesh_name = geometry.mesh_name
        asset_entry = XMLParser._mesh_assets.get(mesh_name)

        if asset_entry is not None:
            asset_entry['scale'] = scale_list
            asset_entry['has_scale_attr'] = True
            asset_entry['_is_new'] = False

        # 更新所有引用的几何体
        base_tris = None
        base_norms = None
        base_path = getattr(geometry, 'mesh_path', None)
        if base_path and os.path.exists(base_path):
            try:
                base_tris, base_norms = load_mesh_file(base_path)
            except Exception as exc:
                print(f"[Mesh] 重新加载失败: {base_path} -> {exc}")
                base_tris, base_norms = None, None

        for geo in self.get_all_geometries():
            if getattr(geo, 'mesh_name', None) == mesh_name:
                # 对每个实例更新资产缩放，并根据最新几何数据重建缓存
                geo.mesh_asset_scale = scale_list
                self._rebuild_mesh_geometry(geo, tris=base_tris, norms=base_norms)

        try:
            XMLParser._update_active_context()
        except Exception:
            pass

        if hasattr(self, 'control_viewmodel') and self.control_viewmodel:
            self.control_viewmodel._on_geometry_modified()

        return True

    def _rebuild_mesh_geometry(self, geometry, tris=None, norms=None):
        """按最新资产缩放/文件内容重建 mesh 的局部数据"""
        path = getattr(geometry, 'mesh_path', None)
        if tris is None or norms is None:
            if not path or not os.path.exists(path):
                return
            try:
                tris, norms = load_mesh_file(path)
            except Exception as exc:
                print(f"[Mesh] 重新加载失败: {path} -> {exc}")
                return

        asset_scale = np.array(getattr(geometry, 'mesh_asset_scale', [1.0, 1.0, 1.0]), dtype=np.float32)
        geom_scale = np.array(getattr(geometry, 'mesh_geom_scale', [1.0, 1.0, 1.0]), dtype=np.float32)
        total_scale = asset_scale * geom_scale
        tris_array = np.asarray(tris, dtype=np.float32)
        tris_scaled = tris_array * total_scale.reshape(1, 1, 3)

        # 基于缩放后的网格重新计算包围盒与中心
        flat_pts = tris_scaled.reshape(-1, 3)
        mins = flat_pts.min(axis=0)
        maxs = flat_pts.max(axis=0)
        center = (mins + maxs) * 0.5
        half_extents = np.maximum((maxs - mins) * 0.5, 1e-5)

        prev_offset = np.array(getattr(geometry, 'mesh_origin_offset', [0.0, 0.0, 0.0]), dtype=float)
        geometry.mesh_origin_offset = center.astype(float).tolist()

        geometry.mesh_model_triangles = (tris_scaled - center.reshape(1, 1, 3)).astype(np.float32)
        if norms is None:
            geometry.mesh_model_normals = None
        else:
            geometry.mesh_model_normals = np.asarray(norms, dtype=np.float32)
        if geometry.mesh_model_normals is None:
            self._ensure_mesh_normals(geometry)
        geometry.size = tuple(half_extents.tolist())

        # 中心变化需要同步补偿到几何体世界位置
        delta = center - prev_offset
        geometry.position = (np.array(geometry.position, dtype=float) + delta).tolist()

        if hasattr(geometry, 'update_transform_matrix'):
            geometry.update_transform_matrix()

        self.notify_object_changed(geometry)

    def set_gs_background_state(self, entries, active_key, reset_history=False, emit=True):
        """
        同步高斯背景列表与当前激活项：
        - entries: [{key, path}] 或 (key, path) 结构
        - active_key: 当前激活的GS背景键值
        - reset_history: 是否清空编辑历史及备份
        - emit: 是否向外发射变化信号
        """
        normalized = []
        for entry in entries or []:
            # 归一化输入格式，统一为 {key, path} 字典
            key = None
            path = None
            if isinstance(entry, dict):
                key = entry.get('key')
                path = entry.get('path')
            elif isinstance(entry, (list, tuple)):
                if len(entry) > 0:
                    key = entry[0]
                if len(entry) > 1:
                    path = entry[1]
            else:
                continue
            if not key:
                continue
            normalized.append({'key': key, 'path': path})

        changed = (normalized != self._gs_backgrounds) or (active_key != self._gs_active_background)
        # 仅当列表或激活项发生变化时才进一步处理

        if reset_history:
            # 需要重置时同时清理历史备份文件
            self._clear_gs_edit_history(remove_files=True)

        self._gs_backgrounds = normalized
        self._gs_active_background = active_key

        if emit and (changed or reset_history):
            payload = [dict(item) for item in self._gs_backgrounds]
            # 将最新状态广播给所有订阅方（如OpenGL视图、控制面板）
            self.gsBackgroundsChanged.emit(payload, self._gs_active_background)

        if reset_history and hasattr(self, 'control_viewmodel') and self.control_viewmodel:
            try:
                self.control_viewmodel.clear_history()
                self.control_viewmodel.record_current_state()
            except Exception as exc:
                print(f"同步撤销栈失败: {exc}")

    def get_gs_background_state(self):
        """返回当前缓存的高斯背景列表与激活键"""
        return list(self._gs_backgrounds), self._gs_active_background

    def record_gs_edit(self, path, backup_path):
        """记录一次对GS文件的修改及其备份，供撤销恢复使用"""
        if not path or not backup_path:
            return
        # 记录元组 {原路径, 备份文件}，供撤销时恢复并清理
        self._gs_edit_history.append({'path': os.path.abspath(path), 'backup': os.path.abspath(backup_path)})

    def restore_gs_history_to(self, length):
        """根据目标长度回滚高斯编辑历史，同时恢复备份文件"""
        if length is None:
            return
        try:
            target = max(0, int(length))
        except Exception:
            target = 0

        refreshed = False
        while len(self._gs_edit_history) > target:
            record = self._gs_edit_history.pop()
            backup = record.get('backup')
            path = record.get('path')
            refreshed = True
            # 逐条回滚，必要时复制备份文件覆盖原PLY
            if backup and path and os.path.exists(backup):
                try:
                    shutil.copy2(backup, path)
                    refreshed = True
                except Exception as e:
                    print(f"恢复 PLY 失败: {e}")
            if backup and os.path.exists(backup):
                try:
                    os.remove(backup)
                except Exception:
                    pass

        if refreshed or target < len(self._gs_edit_history):
            payload = [dict(item) for item in self._gs_backgrounds]
            # 回滚后通知外部刷新界面
            self.gsBackgroundsChanged.emit(payload, self._gs_active_background)

    def _clear_gs_edit_history(self, remove_files=False):
        """清理GS编辑历史，可选顺带删除备份文件"""
        if remove_files:
            for record in self._gs_edit_history:
                backup = record.get('backup')
                if backup and os.path.exists(backup):
                    try:
                        os.remove(backup)
                    except Exception:
                        pass
        self._gs_edit_history = []

    def get_gs_history_length(self):
        """返回当前保存的GS编辑历史长度"""
        return len(self._gs_edit_history)

    def clear_gs_backups(self):
        """外部调用入口：清理全部高斯背景备份并重置历史记录"""
        self._clear_gs_edit_history(remove_files=True)

    def _restore_mesh_geometry(self, geometry, mesh_info):
        """根据存档信息还原 mesh 的资产/几何缓存，用于撤销或文件恢复"""
        if geometry is None or not mesh_info:
            return

        mesh_path = mesh_info.get('path')
        if mesh_path:
            geometry.mesh_path = mesh_path
        mesh_name = mesh_info.get('mesh_name')
        if mesh_name:
            geometry.mesh_name = mesh_name
        if 'asset_scale' in mesh_info:
            geometry.mesh_asset_scale = mesh_info.get('asset_scale') or [1.0, 1.0, 1.0]
        if 'geom_scale' in mesh_info:
            geometry.mesh_geom_scale = mesh_info.get('geom_scale') or [1.0, 1.0, 1.0]
        if 'origin_offset' in mesh_info:
            geometry.mesh_origin_offset = mesh_info.get('origin_offset') or [0.0, 0.0, 0.0]

        # 优先使用存档自带的三角面数据，否则回退到重新读取文件
        tris = mesh_info.get('triangles')
        norms = mesh_info.get('normals')

        if tris is not None:
            geometry.mesh_model_triangles = np.array(tris, dtype=np.float32)
        elif mesh_path and os.path.exists(mesh_path):
            try:
                loaded_tris, loaded_norms = load_mesh_file(mesh_path)
                geometry.mesh_model_triangles = loaded_tris.astype(np.float32)
                if norms is None:
                    norms = loaded_norms
            except Exception as exc:
                print(f"[Mesh Restore] 加载失败: {mesh_path} -> {exc}")
                geometry.mesh_model_triangles = np.empty((0, 3, 3), dtype=np.float32)
        else:
            geometry.mesh_model_triangles = np.empty((0, 3, 3), dtype=np.float32)

        if norms is not None:
            geometry.mesh_model_normals = np.array(norms, dtype=np.float32)
        else:
            geometry.mesh_model_normals = None

        if hasattr(geometry, 'update_transform_matrix'):
            geometry.update_transform_matrix()

    def get_serializable_geometries(self):
        """
        获取可序列化的几何体数据，包括所有嵌套的子对象

        返回:
            dict: 包含所有几何体数据的字典
        """
        geometries_data = []
        
        def serialize_geometry(geo, parent_id=None):
            """递归序列化几何体及其子对象"""
            # 创建当前几何体的数据对象
            geo_id = id(geo)  # 使用对象ID作为唯一标识
            geo_data = {
                'id': geo_id,
                'parent_id': parent_id,
                'type': geo.type if isinstance(geo.type, str) else (geo.type.name if hasattr(geo.type, 'name') else str(geo.type)),
                'position': geo.position.tolist(),  # 位置转为列表
                'rotation': geo.rotation.tolist(),  # 旋转转为列表
                'scale': geo.size.tolist(),  # 缩放转为列表
                'name': geo.name if hasattr(geo, 'name') else f"Object_{id(geo)}",
            }
            
            # 添加颜色属性
            if hasattr(geo, 'material') and hasattr(geo.material, 'color'):
                geo_data['color'] = geo.material.color.tolist()
            else:
                geo_data['color'] = [1.0, 1.0, 1.0, 1.0]
            
            # 添加特有属性（如果存在）
            if hasattr(geo, 'get_specific_properties'):
                geo_data['properties'] = geo.get_specific_properties()
            else:
                geo_data['properties'] = {}

            # 记录 mesh 专属信息
            mesh_info = {}
            if getattr(geo, 'type', None) in (GeometryType.MESH.value, 'mesh'):
                # 序列化资产路径、缩放与缓存的三角面，便于保存/撤销恢复
                mesh_path = getattr(geo, 'mesh_path', None)
                mesh_info['path'] = mesh_path
                mesh_info['mesh_name'] = getattr(geo, 'mesh_name', None)
                if hasattr(geo, 'mesh_asset_scale'):
                    mesh_info['asset_scale'] = list(getattr(geo, 'mesh_asset_scale', []))
                if hasattr(geo, 'mesh_geom_scale'):
                    mesh_info['geom_scale'] = list(getattr(geo, 'mesh_geom_scale', []))
                if hasattr(geo, 'mesh_origin_offset'):
                    mesh_info['origin_offset'] = list(getattr(geo, 'mesh_origin_offset', []))

                tris = getattr(geo, 'mesh_model_triangles', None)
                if isinstance(tris, np.ndarray):
                    mesh_info['triangles'] = tris.tolist()
                elif isinstance(tris, list):
                    mesh_info['triangles'] = tris

                norms = getattr(geo, 'mesh_model_normals', None)
                if isinstance(norms, np.ndarray):
                    mesh_info['normals'] = norms.tolist()
                elif isinstance(norms, list):
                    mesh_info['normals'] = norms

                geo_data['mesh_info'] = mesh_info
            
            # 添加当前几何体数据
            geometries_data.append(geo_data)
            
            # 递归处理子对象
            if hasattr(geo, 'children'):
                for child in geo.children:
                    serialize_geometry(child, geo_id)
        
        # 处理所有顶层几何体
        for geo in self._geometries:
            serialize_geometry(geo)
        
        # 返回包含场景信息和几何体数据的字典
        return {
            'version': '1.0',
            'geometries': geometries_data,
            'gs_backgrounds': [dict(item) for item in self._gs_backgrounds],
            'gs_active_background': self._gs_active_background,
            'gs_history_length': self.get_gs_history_length()
        }
    
    def load_geometries_from_data(self, data):
        """
        从数据加载几何体，包括层次结构
        
        参数:
            data: 包含几何体数据的字典
        
        返回:
            bool: 加载是否成功
        """
        try:
            print("开始加载几何体数据...")
            
            # 检查数据版本兼容性
            if 'version' not in data:
                print("无法识别的数据格式")
                return False
            
            # 清除当前场景中的所有几何体
            self._geometries = []
            
            # 创建ID到几何体的映射，用于处理父子关系
            id_to_geo = {}
            
            # 记录加载的几何体数量
            loaded_count = 0
            
            # 首先创建所有几何体
            for geo_data in data.get('geometries', []):
                # 获取ID和父ID
                geo_id = geo_data.get('id')
                parent_id = geo_data.get('parent_id')
                
                # 从数据中获取几何体类型
                geo_type_name = geo_data.get('type')
                if not geo_type_name:
                    continue
                
                # 从数据中获取属性
                name = geo_data.get('name', None)
                position = geo_data.get('position', [0, 0, 0])
                rotation = geo_data.get('rotation', [0, 0, 0])
                size = geo_data.get('scale', [1, 1, 1])
                color = geo_data.get('color', [1, 1, 1, 1])
                
                print(f"正在加载: {name}, 类型: {geo_type_name}, ID: {geo_id}, 父ID: {parent_id}")
                
                # 查找父对象
                parent_geo = id_to_geo.get(parent_id) if parent_id else None
                
                geo = None  # 初始化几何体变量
                
                # 检查是否是几何体组
                if geo_type_name == 'group':
                    try:
                        # 创建几何体组
                        geo = self.create_group(
                            name=name,
                            position=position,
                            rotation=rotation,
                            parent=parent_geo
                        )
                        loaded_count += 1
                    except Exception as e:
                        print(f"创建几何体组失败: {str(e)}")
                        continue
                else:
                    # 处理普通几何体类型
                    try:
                        # 尝试直接匹配枚举或值
                        found = False
                        for gt in GeometryType:
                            if (gt.name == geo_type_name or 
                                gt.value == geo_type_name or 
                                str(gt.name).lower() == str(geo_type_name).lower() or 
                                str(gt.value).lower() == str(geo_type_name).lower()):
                                found = True
                                try:
                                    # 创建几何体
                                    geo = self.create_geometry(
                                        geo_type=gt,
                                        name=name,
                                        position=position,
                                        size=size,
                                        rotation=rotation,
                                        parent=parent_geo
                                    )
                                    # Mesh 需要额外恢复网格数据
                                    if gt == GeometryType.MESH and isinstance(geo_data.get('mesh_info'), dict):
                                        mesh_info = geo_data.get('mesh_info')
                                        self._restore_mesh_geometry(geo, mesh_info)
                                    
                                    # 设置颜色
                                    if hasattr(geo, 'material') and hasattr(geo.material, 'color'):
                                        geo.material.color = color
                                    
                                    loaded_count += 1
                                    break
                                except Exception as e:
                                    print(f"创建几何体失败: {str(e)}")
                                    continue
                        
                        if not found:
                            print(f"未找到匹配的几何体类型: {geo_type_name}")
                    except Exception as e:
                        print(f"处理几何体时出错: {str(e)}")
                        continue
                
                # 如果成功创建了几何体，将其添加到ID映射中
                if geo and geo_id:
                    id_to_geo[geo_id] = geo
            
            # 更新所有变换矩阵
            self.update_all_transform_matrices()
            
            # 更新射线投射器
            self._update_raycaster()

            # 清除当前选择
            self.clear_selection()

            # 恢复高斯背景状态
            entries = data.get('gs_backgrounds', [])
            active_key = data.get('gs_active_background')
            history_len = data.get('gs_history_length')
            if hasattr(self, 'set_gs_background_state'):
                self.set_gs_background_state(entries, active_key, reset_history=False, emit=True)
            if history_len is not None:
                self.restore_gs_history_to(history_len)

            # 通知视图更新
            self.geometriesChanged.emit()

            print(f"成功加载 {loaded_count} 个几何体")
            
            return True
        except Exception as e:
            print(f"加载几何体数据失败: {str(e)}")
            import traceback
            traceback.print_exc()
            return False
    
    def clear_scene(self, *, reset_backgrounds=True):
        """
        清除场景中的所有几何体
        """
        self._geometries = []
        self._source_groups = {}
        self._active_source_file = None
        self._emit_sources_changed()
        self.geometriesChanged.emit()
        if reset_backgrounds:
            self.set_gs_background_state([], None, reset_history=True, emit=True)
        try:
            XMLParser._context_by_file.clear()
        except AttributeError:
            pass
        XMLParser.activate_context(None)
    
    def notifyPositionChanged(self, geometry):
        """通知几何体位置变化"""
        self.positionChanged.emit(geometry)
        self.objectChanged.emit(geometry)
    
    def notifyRotationChanged(self, geometry):
        """通知几何体旋转变化"""
        self.rotationChanged.emit(geometry)
        self.objectChanged.emit(geometry)
    
    def notifyScaleChanged(self, geometry):
        """通知几何体缩放变化"""
        self.scaleChanged.emit(geometry)
        self.objectChanged.emit(geometry)
    
    def set_hierarchy_viewmodel(self, hierarchy_viewmodel):
        """设置层级视图模型的引用"""
        self.hierarchyViewModel = hierarchy_viewmodel

    def refresh_hierarchy_tree(self):
        """请求刷新层级树"""
        if hasattr(self, 'hierarchyViewModel') and self.hierarchyViewModel:
            # 只发出信号，让信号的接收者决定如何刷新
            self.geometriesChanged.emit()

            self.hierarchyViewModel.hierarchyChanged.emit()

    def set_global_gizmo_size_world(self, value: float):
        try:
            v = max(0.01, float(value))  # 不让它为 0 或负
        except Exception:
            return
        if getattr(self, "global_gizmo_size_world", None) != v:
            self.global_gizmo_size_world = v
            # 向界面与 OpenGL 视图广播新的世界尺度
            self.gizmoSizeChanged.emit(v)
            # 通知重绘：任选一种你已有的刷新方式
            # try:
            #     self.objectChanged.emit(self.selected_geometry)  # 触发一次重绘
            # except Exception:
            #     pass
        
    
    # 然后在初始化代码中连接它们
    # hierarchy_viewmodel = HierarchyViewModel(scene_viewmodel)
    # scene_viewmodel.set_hierarchy_viewmodel(hierarchy_viewmodel) 
