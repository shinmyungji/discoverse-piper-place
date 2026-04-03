"""
XML解析器

处理MJCF文件的加载、解析和保存功能。
"""

import xml.etree.ElementTree as ET
from xml.etree.ElementTree import tostring, fromstring
from xml.dom import minidom
import numpy as np
from .geometry import Geometry, GeometryGroup, GeometryType
from ..utils.mesh_loader import load_mesh as load_mesh_file
import os
import copy
from scipy.spatial.transform import Rotation as R  # 新增

class XMLParser:
    """
    XML文件解析和生成工具
    
    用于处理MJCF场景的加载和保存，支持两种格式：
    1. 增强XML格式（自定义格式，更适合编辑器内部使用）
    2. MuJoCo XML格式（标准MJCF格式）
    """
    _last_loaded_mjcf_root = None        # 深拷贝root节点
    _last_loaded_angle_mode = "radian"   # 度解析使用的单位默认弧度
    _last_xml_dir = ""                   # XML文件的目录（不包含文件）
    _last_meshdir = ""                   # Meshdir的相对路径
    _mesh_assets = {}                    # mesh的资产{name, {file...}}(assets)
    _mesh_instances = []
    _new_mesh_assets = set()
    _context_by_file = {}           # 每个源文件对应的解析上下文快照
    _active_source_key = None       # 当前激活的源文件路径（用于导出/属性编辑）
    _angles_normalized = True       # 当前上下文中的关节角度是否已统一为弧度

    @staticmethod
    def register_mesh_asset(name, file_path, scale=None, has_scale_attr=False):
        """
        注册网格资产
        
        """
        if not name:
            return

        if scale is None:
            scale = [1.0, 1.0, 1.0]

        try:
            scale = [float(s) for s in scale]
        except Exception:
            scale = [1.0, 1.0, 1.0]

        abs_file = os.path.abspath(file_path) if file_path else ""
        XMLParser._mesh_assets[name] = {
            "file": file_path or "",
            "abs_file": abs_file,
            "scale": scale,
            "_is_new": True,
            "has_scale_attr": bool(has_scale_attr),
        }
        XMLParser._new_mesh_assets.add(name)
        XMLParser._update_active_context()

    @staticmethod
    def load(filename):
        """
        从XML文件导入几何体和组层级结构
        参数:
            filename: 要加载的XML文件路径  
        返回:
            几何体对象列表
        """
        try:
            abs_file = os.path.abspath(filename)
            XMLParser._last_xml_dir = os.path.abspath(os.path.dirname(filename))
            XMLParser._mesh_assets = {}
            XMLParser._mesh_instances = []
            XMLParser._last_meshdir = ""
            XMLParser._new_mesh_assets = set()
            XMLParser._angles_normalized = True
            tree = ET.parse(filename)
            root = tree.getroot()
            
            # 检查文件格式类型
            is_mujoco_format = root.tag == "mujoco"
            # 缓存“原始 MJCF 根节点”的深拷贝
            if is_mujoco_format:
                # 深拷贝，避免后续修改影响原节点
                try:
                    XMLParser._last_loaded_mjcf_root = fromstring(tostring(root))
                except Exception:
                    XMLParser._last_loaded_mjcf_root = None
            else:
                XMLParser._last_loaded_mjcf_root = None
            is_enhanced_format = root.tag == "Scene"
            
            # 解析 angle_mode：MuJoCo 若未显式声明则默认为弧度
            if is_mujoco_format:
                comp = root.find("compiler")
                if comp is not None and comp.get("angle"):
                    XMLParser._last_loaded_angle_mode = comp.get("angle").strip().lower()
                else:
                    XMLParser._last_loaded_angle_mode = "radian"
            XMLParser._angles_normalized = (XMLParser._last_loaded_angle_mode != "degree")

            if is_enhanced_format:
                geometries = XMLParser._load_enhanced_format(root)
                XMLParser._store_context_for_file(abs_file)
                XMLParser.activate_context(abs_file)
                return geometries
            elif is_mujoco_format:
                geometries = XMLParser._load_mujoco_format(root)
                XMLParser._store_context_for_file(abs_file)
                XMLParser.activate_context(abs_file)
                return geometries
            else:
                raise ValueError(f"不支持的XML格式：{root.tag}")
        except Exception as e:
            print(f"加载XML文件时出错: {e}")
            return []
    
    @staticmethod
    def _clone_element(elem):
        if isinstance(elem, ET.Element):
            try:
                return fromstring(tostring(elem))
            except Exception:
                return None
        return None

    @staticmethod
    def _snapshot_state():
        # 打包当前解析上下文，供多文件场景在切换/保存时恢复
        return {
            'root_snapshot': XMLParser._clone_element(XMLParser._last_loaded_mjcf_root),
            'angle_mode': XMLParser._last_loaded_angle_mode,
            'meshdir': XMLParser._last_meshdir,
            'xml_dir': XMLParser._last_xml_dir,
            'mesh_assets': copy.deepcopy(XMLParser._mesh_assets),
            'mesh_instances': copy.deepcopy(XMLParser._mesh_instances),
            'new_mesh_assets': set(XMLParser._new_mesh_assets),
            'angles_normalized': XMLParser._angles_normalized,
        }

    @staticmethod
    def _store_context_for_file(filename):
        if not filename:
            return
        try:
            path = os.path.abspath(filename)
        except Exception:
            return
        # 以绝对路径作为 key 缓存当前快照
        XMLParser._context_by_file[path] = XMLParser._snapshot_state()
        XMLParser._active_source_key = path

    @staticmethod
    def _update_active_context():
        key = XMLParser._active_source_key
        if not key:
            return
        if key not in XMLParser._context_by_file:
            return
        XMLParser._context_by_file[key] = XMLParser._snapshot_state()

    @staticmethod
    def _collect_mesh_names(objects):
        names = set()
        stack = list(objects or [])
        while stack:
            obj = stack.pop()
            if obj is None:
                continue
            obj_type = getattr(obj, 'type', None)
            if obj_type in (GeometryType.MESH.value, 'mesh'):
                mesh_name = getattr(obj, 'mesh_name', None)
                if mesh_name:
                    names.add(mesh_name)
            children = getattr(obj, 'children', None)
            if children:
                stack.extend(children)
        return names

    @staticmethod
    def _normalize_joint_angle_dict(attrs, joint_type, angle_mode):
        if joint_type != 'hinge':
            return False
        if angle_mode != 'degree':
            return False
        if not isinstance(attrs, dict):
            return False
        changed = False
        for key in ("range", "ref"):
            text = attrs.get(key)
            if not text:
                continue
            try:
                parts = [float(p) for p in str(text).split()]
            except Exception:
                continue
            if not parts:
                continue
            rad_vals = [float(np.radians(val)) for val in parts]
            attrs[key] = " ".join(f"{val:.12g}" for val in rad_vals)
            changed = True
        return changed

    @staticmethod
    def activate_context(source_file):
        if not source_file:
            XMLParser._active_source_key = None
            XMLParser._last_loaded_mjcf_root = None
            XMLParser._last_loaded_angle_mode = "radian"
            XMLParser._last_meshdir = ""
            XMLParser._last_xml_dir = ""
            XMLParser._mesh_assets = {}
            XMLParser._mesh_instances = []
            XMLParser._new_mesh_assets = set()
            XMLParser._angles_normalized = True
            return False
        try:
            path = os.path.abspath(source_file)
        except Exception:
            return False
        # 尝试恢复之前缓存的上下文；若缺失则退回默认状态
        context = XMLParser._context_by_file.get(path)
        if not context:
            XMLParser._active_source_key = None
            XMLParser._last_loaded_mjcf_root = None
            XMLParser._last_loaded_angle_mode = "radian"
            XMLParser._last_meshdir = ""
            XMLParser._last_xml_dir = ""
            XMLParser._mesh_assets = {}
            XMLParser._mesh_instances = []
            XMLParser._new_mesh_assets = set()
            XMLParser._angles_normalized = True
            return False

        XMLParser._last_loaded_mjcf_root = XMLParser._clone_element(context.get('root_snapshot'))
        XMLParser._last_loaded_angle_mode = context.get('angle_mode', 'radian')
        XMLParser._last_meshdir = context.get('meshdir', '')
        XMLParser._last_xml_dir = context.get('xml_dir', XMLParser._last_xml_dir)
        XMLParser._mesh_assets = copy.deepcopy(context.get('mesh_assets', {}))
        XMLParser._mesh_instances = copy.deepcopy(context.get('mesh_instances', []))
        XMLParser._new_mesh_assets = set(context.get('new_mesh_assets', set()))
        XMLParser._angles_normalized = context.get('angles_normalized', XMLParser._angles_normalized)
        XMLParser._active_source_key = path
        return True

    @staticmethod
    def _load_enhanced_format(root):
        """
        处理增强XML格式（自定义格式）
        
        参数:
            root: XML根元素
            
        返回:
            几何体对象列表
        """
        geometries = []
        objects_node = root.find("Objects")
        
        if objects_node is not None:
            # 递归处理对象树
            def process_node(node, parent=None):
                results = []
                
                for child in node:
                    if child.tag == "Group":
                        # 创建组
                        name = child.get("name", "Group")
                        
                        # 解析位置
                        pos_elem = child.find("Position")
                        position = [0, 0, 0]
                        if pos_elem is not None:
                            position = [
                                float(pos_elem.get("x", 0)),
                                float(pos_elem.get("y", 0)),
                                float(pos_elem.get("z", 0))
                            ]
                        
                        # 解析旋转
                        rot_elem = child.find("Rotation")
                        rotation = [0, 0, 0]
                        if rot_elem is not None:
                            rotation = [
                                float(rot_elem.get("x", 0)),
                                float(rot_elem.get("y", 0)),
                                float(rot_elem.get("z", 0))
                            ]
                        
                        # 创建组对象
                        group = GeometryGroup(name=name, position=position, rotation=rotation, parent=parent)
                        
                        # 确保变换矩阵被更新
                        if hasattr(group, "update_transform_matrix"):
                            group.update_transform_matrix()
                        
                        # 处理子节点
                        children_elem = child.find("Children")
                        if children_elem is not None:
                            child_objects = process_node(children_elem, group)
                            for child_obj in child_objects:
                                if parent is None:  # 顶层对象
                                    group.add_child(child_obj)
                        
                        if parent is None:
                            results.append(group)
                        else:
                            parent.add_child(group)
                        
                    elif child.tag == "Geometry":
                        # 处理几何体
                        name = child.get("name", "Object")
                        geo_type = child.get("type", "box")
                        
                        # 解析位置
                        pos_elem = child.find("Position")
                        position = [0, 0, 0]
                        if pos_elem is not None:
                            position = [
                                float(pos_elem.get("x", 0)),
                                float(pos_elem.get("y", 0)),
                                float(pos_elem.get("z", 0))
                            ]
                            
                        # 解析尺寸
                        size_elem = child.find("Size")
                        size = [1, 1, 1]
                        if size_elem is not None:
                            size = [
                                float(size_elem.get("x", 1)),
                                float(size_elem.get("y", 1)),
                                float(size_elem.get("z", 1))
                            ]
                            
                        # 解析旋转
                        rot_elem = child.find("Rotation")
                        rotation = [0, 0, 0]
                        if rot_elem is not None:
                            rotation = [
                                float(rot_elem.get("x", 0)),
                                float(rot_elem.get("y", 0)),
                                float(rot_elem.get("z", 0))
                            ]
                        
                        # 创建几何体
                        geo = Geometry(
                            geo_type=geo_type, 
                            name=name,
                            position=position,
                            size=size,
                            rotation=rotation,
                            parent=parent
                        )
                        
                        # 确保变换矩阵被更新
                        if hasattr(geo, "update_transform_matrix"):
                            geo.update_transform_matrix()
                        
                        # 处理材质
                        material_elem = child.find("Material")
                        if material_elem is not None:
                            color_elem = material_elem.find("Color")
                            if color_elem is not None:
                                color = [
                                    float(color_elem.get("r", 1.0)),
                                    float(color_elem.get("g", 1.0)),
                                    float(color_elem.get("b", 1.0)),
                                    float(color_elem.get("a", 1.0))
                                ]
                                geo.material.color = color
                        
                        if parent is None:
                            results.append(geo)
                        else:
                            parent.add_child(geo)
                
                return results
            
            # 开始处理对象节点
            geometries = process_node(objects_node)
        
        return geometries
    
    @staticmethod
    def _load_mujoco_format(root):
        """
        处理MuJoCo XML格式
        
        参数:
            root: XML根元素
            
        返回:
            几何体对象列表
        """
        # 解析 compiler 标签，记录原始 angle 单位及 meshdir 等全局信息
        compiler = root.find("compiler")
        angle_mode = "radian"
        if compiler is not None:
            val = compiler.get("angle")
            if val:
                angle_mode = val.strip().lower()
        is_radian = (angle_mode == "radian")

        XMLParser._last_meshdir = compiler.get("meshdir", "") if compiler is not None else ""

        # mesh资产遍历
        assets = {}
        asset_node = root.find("asset")
        if asset_node is not None:
            for m in asset_node.findall("mesh"):
                name = m.get("name")
                file = m.get("file", "")
                sstr = m.get("scale", "")
                if sstr:
                    s = list(map(float, sstr.split()))
                else:
                    s = [1.0, 1.0, 1.0]
                if len(s) == 1: s = [s[0], s[0], s[0]]
                if len(s) == 2: s = [s[0], s[1], 1.0]
                if name:
                    assets[name] = {
                        "file": file,
                        "_orig_file_attr": file,
                        "scale": s,
                        "_is_new": False,
                        "has_scale_attr": bool(sstr)
                    }
        XMLParser._mesh_assets = assets

        # 弧度->度
        def _to_deg_scalar(x):
            x = float(x)
            # MuJoCo 输入若标记为弧度，则转换成角度供编辑器使用
            return float(np.degrees(x)) if is_radian else x

        def _to_deg_vec(seq):
            arr = np.array(list(map(float, seq)), dtype=float)
            return np.degrees(arr).tolist() if is_radian else arr.tolist()
        
        geometries = [] # 存放顶层组(gemo和group)
        
        # 创建一个字典来跟踪body和对应的几何体组
        body_groups = {} # 所有body对象
        parent_map = {}  # 用于跟踪父子关系
        
        # 构建父子关系映射
        for body in root.findall(".//body"):
            body_name = body.get('name', 'Unnamed')
            # 寻找直接父body
            parent_body = None
            for parent in root.findall(".//body"):
                if body in parent.findall("./body"):
                    parent_body = parent
                    break
            
            if parent_body is not None:
                parent_map[body_name] = parent_body.get('name', 'Unnamed')#{"child_name": "parent_name"}
        
        # 处理所有body
        for body in root.findall(".//body"):
            body_name = body.get('name', 'Unnamed')
            if body_name in body_groups:
                continue  # 跳过已处理的body
            
            body_pos = list(map(float, body.get('pos', '0 0 0').split()))

            # 统一把 euler 转成“度”
            if 'euler' in body.attrib:
                body_euler = _to_deg_vec(body.get('euler').split())
            else:
                body_euler = [0, 0, 0]

            if 'quat' in body.attrib:
                quat = list(map(float, body.get('quat').split()))
                if len(quat) == 4:
                    body_euler = XMLParser._quat_to_euler(quat)
            
            # 创建组对象，一个body对应一个组对象
            group = GeometryGroup(
                name=body_name,
                position=body_pos,
                rotation=body_euler
            )

            try:
                group._mjcf_body_snapshot = ET.fromstring(ET.tostring(body)) #body深拷贝
            except Exception:
                group._mjcf_body_snapshot = None

            # 确保变换矩阵被更新，转换到世界坐标系
            if hasattr(group, "update_transform_matrix"):
                group.update_transform_matrix()
            
            body_groups[body_name] = group
            
            # 先处理关节（只处理 hinge 和 slide）
            joint_elems = body.findall("joint")
            for j_idx, joint_elem in enumerate(joint_elems):
                joint_type = (joint_elem.get('type') or 'hinge').lower()
                if joint_type not in ('hinge', 'slide'):
                    continue

                joint_name = joint_elem.get('name', f"{body_name}_joint{j_idx + 1}")
                joint_pos = list(map(float, joint_elem.get('pos', '0 0 0').split())) if 'pos' in joint_elem.attrib else [0.0, 0.0, 0.0]
                axis_raw = joint_elem.get('axis')
                axis_vec = None
                if axis_raw:
                    parts = axis_raw.replace(',', ' ').split()
                    if len(parts) >= 3:
                        try:
                            axis_vec = [float(parts[i]) for i in range(3)]
                        except Exception:
                            axis_vec = None
                if axis_vec is None:
                    axis_vec = [1.0, 0.0, 0.0]

                joint_rotation = [0.0, 0.0, 0.0]
                joint_length = float(joint_elem.attrib.get('length', 0.4))
                half_len = max(joint_length * 0.5, 0.05)

                joint_geo = Geometry(
                    geo_type=GeometryType.JOINT.value,
                    name=joint_name,
                    position=joint_pos,
                    size=(half_len, 0.015, 0.015),
                    rotation=joint_rotation,
                    parent=group
                )

                joint_color = (1.0, 0.9, 0.2, 1.0) if joint_type == 'hinge' else (0.4, 0.8, 1.0, 1.0)
                joint_geo.material.color = joint_color
                joint_geo.mjcf_attrs = dict(joint_elem.attrib)
                joint_geo.joint_attrs = dict(joint_elem.attrib)
                joint_geo.joint_angle_mode = 'radian'
                if XMLParser._normalize_joint_angle_dict(joint_geo.joint_attrs, joint_type, XMLParser._last_loaded_angle_mode):
                    XMLParser._angles_normalized = True
                if XMLParser._normalize_joint_angle_dict(joint_geo.mjcf_attrs, joint_type, XMLParser._last_loaded_angle_mode):
                    XMLParser._angles_normalized = True
                joint_geo.joint_axis = axis_vec
                joint_geo.joint_length = joint_length
                joint_geo.joint_type = joint_type
                joint_geo._mjcf_had_name = ('name' in joint_elem.attrib)

                if hasattr(joint_geo, "update_transform_matrix"):
                    joint_geo.update_transform_matrix()

                group.add_child(joint_geo)

            # 添加所有geom子对象
            for geom in body.findall("geom"):
                geo_type = geom.get('type', 'box')
                geom_name = geom.get('name', f"{body_name}_geom") # gemo的名字前加上body的名字
                
                # 解析尺寸
                size_str = geom.get('size', '1 1 1')
                size = list(map(float, size_str.split()))
                
                # 适当地处理尺寸格式
                if geo_type == 'sphere':
                    if len(size) == 1:
                        size = [size[0], size[0], size[0]]  # 保持三个相同的半径值
                elif geo_type == 'ellipsoid':
                    # 确保有三个尺寸
                    if len(size) < 3:
                        size.extend([size[0]] * (3 - len(size)))
                elif geo_type in ['capsule', 'cylinder']:
                    # 确保有两个尺寸
                    if len(size) < 2:
                        size.append(1.0)  # 默认半高
                    if len(size) < 3:
                        size.append(0)  # 补充第三个参数
                
                # 解析位置（相对于body的局部坐标）
                local_pos = list(map(float, geom.get('pos', '0 0 0').split())) if 'pos' in geom.attrib else [0, 0, 0]
                
                # 解析旋转
                local_euler = [0, 0, 0]
                if 'euler' in geom.attrib:
                    local_euler = _to_deg_vec(geom.get('euler').split())
                elif 'quat' in geom.attrib:
                    quat = list(map(float, geom.get('quat').split()))
                    if len(quat) == 4:
                        local_euler = XMLParser._quat_to_euler(quat)
                
                # 解析颜色
                color = [0.8, 0.8, 0.8, 1.0]  # 默认灰色
                
                # 优先使用rgba属性
                if 'rgba' in geom.attrib:
                    rgba_str = geom.get('rgba')
                    rgba_values = list(map(float, rgba_str.split()))
                    # 确保有四个值
                    if len(rgba_values) == 3:
                        rgba_values.append(1.0)  # 添加alpha默认值
                    elif len(rgba_values) < 3:
                        rgba_values = [0.8, 0.8, 0.8, 1.0]  # 默认灰色
                    color = rgba_values
                
                # 检查是否引用了material
                elif 'material' in geom.attrib:
                    material_name = geom.get('material')
                    # 尝试在asset下找到对应的material
                    material_elem = root.find(f".//asset/material[@name='{material_name}']")
                    if material_elem is not None and 'rgba' in material_elem.attrib:
                        rgba_str = material_elem.get('rgba')
                        color = list(map(float, rgba_str.split()))
                        if len(color) == 3:
                            color.append(1.0)  # 添加默认透明度
                
                # Mesh几何体：创建真实Geometry并缓存三角面
                if geo_type == 'mesh':
                    mesh_ref = geom.get('mesh', '')
                    asset = assets.get(mesh_ref) # 从assets（提前收集）中找到对应的asset中的mesh资产属性

                    if not asset:
                        print(f"[Mesh] 未找到 asset: {mesh_ref}")
                        continue

                    # 解析 geom 层的 scale（gscale），用于与资产内 scale 组合出总缩放
                    gscale = geom.get('scale', '')
                    if gscale:
                        gs = list(map(float, gscale.split()))
                        if len(gs) == 1:
                            gs = [gs[0], gs[0], gs[0]]
                        if len(gs) == 2:
                            gs = [gs[0], gs[1], 1.0]
                    else:
                        gs = [1.0, 1.0, 1.0]

                    asset_scale = np.array(asset.get('scale', [1.0, 1.0, 1.0]), dtype=np.float32)
                    geom_scale = np.array(gs, dtype=np.float32)
                    total_scale = asset_scale * geom_scale # 总缩放

                    meshdir = XMLParser._last_meshdir or ""
                    base_dir = XMLParser._last_xml_dir or ""
                    mesh_path = asset.get('abs_file')
                    if not mesh_path:
                        mesh_file_attr = asset.get('_orig_file_attr') or asset.get('file', '')
                        mesh_path = mesh_file_attr
                        if mesh_file_attr and not os.path.isabs(mesh_file_attr):
                            # 如果路径以 .. 或 / 开头，说明是相对于XML文件的路径，不使用meshdir
                            if mesh_file_attr.startswith('../') or mesh_file_attr.startswith('/'):
                                mesh_path = os.path.normpath(os.path.join(base_dir, mesh_file_attr))
                            else:
                                # 否则，使用meshdir作为前缀
                                mesh_path = os.path.normpath(os.path.join(base_dir, meshdir, mesh_file_attr))
                    if mesh_path:
                        asset['abs_file'] = mesh_path

                    tris = None
                    norms = None
                    if mesh_path and os.path.exists(mesh_path):
                        try:
                            tris, norms = load_mesh_file(mesh_path)
                        except Exception as exc:
                            print(f"[Mesh] 加载失败: {mesh_path} -> {exc}")
                    else:
                        print(f"[Mesh] 找不到文件: {mesh_path}")

                    if tris is None:
                        tris = np.empty((0, 3, 3), dtype=np.float32)
                    tris = tris.astype(np.float32)
                    tris = tris * total_scale.reshape(1, 1, 3)

                    if norms is not None:
                        norms = norms.astype(np.float32)
                        # 非均匀缩放会破坏原有法线，统一在渲染阶段使用叉积补法线
                        norms = None

                    #AABB
                    center = np.zeros(3, dtype=np.float32)
                    if len(tris) > 0:
                        flat_pts = tris.reshape(-1, 3)
                        mins = flat_pts.min(axis=0)
                        maxs = flat_pts.max(axis=0)
                        center = (mins + maxs) * 0.5
                        half_extents = (maxs - mins) * 0.5
                        half_extents = np.maximum(half_extents, 1e-5)
                    else:
                        half_extents = np.array([0.5, 0.5, 0.5], dtype=np.float32)

                    # 坐标系转换
                    tris_local = tris - center.reshape(1, 1, 3) #方便计算aabb
                    local_pos_np = np.asarray(local_pos, dtype=np.float32)
                    geo_position = (local_pos_np + center).tolist() # 区分gemo和trid的坐标

                    geo = Geometry(
                        geo_type=GeometryType.MESH.value,
                        name=geom_name,
                        position=geo_position,
                        size=tuple(half_extents.tolist()),
                        rotation=local_euler,
                        parent=group
                    )

                    geo.mjcf_attrs = dict(geom.attrib)
                    geo._mjcf_had_name = ('name' in geom.attrib) # 根据true/false来判断是否有name属性，是否后续要补name
                    geo.material.color = color

                    geo.mesh_name = mesh_ref
                    geo.mesh_path = mesh_path
                    geo.mesh_asset_scale = asset_scale.tolist()
                    geo.mesh_geom_scale = geom_scale.tolist()
                    geo.mesh_model_triangles = tris_local
                    geo.mesh_model_normals = norms

                    if hasattr(geo, "update_transform_matrix"):
                        geo.update_transform_matrix() # 世界坐标

                    geo.mesh_origin_offset = center.astype(np.float32).tolist()

                    group.add_child(geo) # gemo是对应group的child
                    continue
                
                # 创建几何体
                geo = Geometry(
                    geo_type=geo_type,
                    name=geom_name,
                    position=local_pos,
                    size=size,
                    rotation=local_euler,
                    parent=group
                )
                
                # 保存原始 MJCF 属性（用于导出直通）
                geo.mjcf_attrs = dict(geom.attrib)
                geo._mjcf_had_name = ('name' in geom.attrib)

                # 设置颜色
                geo.material.color = color
                
                # 确保变换矩阵被更新
                if hasattr(geo, "update_transform_matrix"):
                    geo.update_transform_matrix()
                
                # 添加到组中
                group.add_child(geo)
        
        # 在返回前进行一次全面的变换矩阵更新
        # 先确保所有父子关系已经建立
        for body_name, parent_name in parent_map.items():
            if body_name in body_groups and parent_name in body_groups:
                child_group = body_groups[body_name]
                parent_group = body_groups[parent_name]
                
                # 避免重复添加
                if child_group not in parent_group.children:
                    parent_group.add_child(child_group)
        
        # 收集所有顶层组（没有父组的组）
        top_level_groups = []
        for name, group in body_groups.items():
            if name not in parent_map:  # 没有父组
                top_level_groups.append(group)
        
        # 如果找到了顶层组，将它们添加到geometries
        if top_level_groups:
            geometries.extend(top_level_groups)
        
        # 处理worldbody下的直接geom
        # 处理worldbody下的直接geom —— 直接作为顶层对象，不再包装到 "World" 组
        world_body = root.find(".//worldbody")
        if world_body is not None:
            for geom in world_body.findall("geom"):
                # 可按需排除参考平面与坐标轴
                geom_name = geom.get('name', '')
                if geom_name in ["ground", "x_axis", "y_axis", "z_axis"]:
                    continue

                geo_type = geom.get('type', 'box')

                # pos / size
                pos = list(map(float, geom.get('pos', '0 0 0').split()))
                size_str = geom.get('size', '1 1 1')
                size = list(map(float, size_str.split()))

                # 尺寸格式修正
                if geo_type == 'sphere':
                    if len(size) == 1:
                        size = [size[0], size[0], size[0]]
                elif geo_type in ['capsule', 'cylinder']:
                    if len(size) < 2:
                        size.append(1.0)  # 半高默认
                    if len(size) < 3:
                        size.append(0)
                elif len(size) < 3:
                    size.extend([1.0] * (3 - len(size)))

                # 旋转（统一转成“度”存内存）
                euler = [0, 0, 0]
                if 'euler' in geom.attrib:
                    euler = _to_deg_vec(geom.get('euler').split())
                elif 'quat' in geom.attrib:
                    quat = list(map(float, geom.get('quat').split()))
                    if len(quat) == 4:
                        euler = XMLParser._quat_to_euler(quat)

                # 颜色
                color = [0.8, 0.8, 0.8, 1.0]
                if 'rgba' in geom.attrib:
                    rgba_values = list(map(float, geom.get('rgba').split()))
                    if len(rgba_values) >= 3:
                        color = rgba_values
                        if len(color) == 3:
                            color.append(1.0)

                if geo_type == 'mesh':
                    mesh_ref = geom.get('mesh', '')
                    asset = assets.get(mesh_ref)
                    if not asset:
                        print(f"[Mesh] 未找到 asset: {mesh_ref}")
                        continue

                    gscale = geom.get('scale', '')
                    if gscale:
                        gs = list(map(float, gscale.split()))
                        if len(gs) == 1:
                            gs = [gs[0], gs[0], gs[0]]
                        if len(gs) == 2:
                            gs = [gs[0], gs[1], 1.0]
                    else:
                        gs = [1.0, 1.0, 1.0]

                    asset_scale = np.array(asset.get('scale', [1.0, 1.0, 1.0]), dtype=np.float32)
                    geom_scale = np.array(gs, dtype=np.float32)
                    total_scale = asset_scale * geom_scale

                    meshdir = XMLParser._last_meshdir or ""
                    base_dir = XMLParser._last_xml_dir or ""
                    mesh_path = asset.get('abs_file')
                    if not mesh_path:
                        mesh_file_attr = asset.get('_orig_file_attr') or asset.get('file', '')
                        mesh_path = mesh_file_attr
                        if mesh_file_attr and not os.path.isabs(mesh_file_attr):
                            # 如果路径以 .. 或 / 开头，说明是相对于XML文件的路径，不使用meshdir
                            if mesh_file_attr.startswith('../') or mesh_file_attr.startswith('/'):
                                mesh_path = os.path.normpath(os.path.join(base_dir, mesh_file_attr))
                            else:
                                # 否则，使用meshdir作为前缀
                                mesh_path = os.path.normpath(os.path.join(base_dir, meshdir, mesh_file_attr))
                    if mesh_path:
                        asset['abs_file'] = mesh_path

                    tris = None
                    norms = None
                    if mesh_path and os.path.exists(mesh_path):
                        try:
                            tris, norms = load_mesh_file(mesh_path)
                        except Exception as exc:
                            print(f"[Mesh] 加载失败: {mesh_path} -> {exc}")
                    else:
                        print(f"[Mesh] 找不到文件: {mesh_path}")

                    if tris is None:
                        tris = np.empty((0, 3, 3), dtype=np.float32)
                    tris = tris.astype(np.float32)
                    tris = tris * total_scale.reshape(1, 1, 3)

                    if norms is not None:
                        norms = norms.astype(np.float32)
                        norms = None

                    center = np.zeros(3, dtype=np.float32)
                    if len(tris) > 0:
                        flat_pts = tris.reshape(-1, 3)
                        mins = flat_pts.min(axis=0)
                        maxs = flat_pts.max(axis=0)
                        center = (mins + maxs) * 0.5
                        half_extents = (maxs - mins) * 0.5
                        half_extents = np.maximum(half_extents, 1e-5)
                    else:
                        half_extents = np.array([0.5, 0.5, 0.5], dtype=np.float32)

                    tris_local = tris - center.reshape(1, 1, 3)
                    pos_np = np.asarray(pos, dtype=np.float32)
                    geo_position = (pos_np + center).tolist()

                    geo = Geometry(
                        geo_type=GeometryType.MESH.value,
                        name=geom_name or "Object",
                        position=geo_position,
                        size=tuple(half_extents.tolist()),
                        rotation=euler,
                        parent=None
                    )

                    geo.mjcf_attrs = dict(geom.attrib)
                    geo._mjcf_had_name = ('name' in geom.attrib)
                    geo.material.color = color
                    geo.mesh_name = mesh_ref
                    geo.mesh_path = mesh_path
                    geo.mesh_asset_scale = asset_scale.tolist()
                    geo.mesh_geom_scale = geom_scale.tolist()
                    geo.mesh_model_triangles = tris_local
                    geo.mesh_model_normals = norms
                    geo.mesh_origin_offset = center.astype(np.float32).tolist()

                    if hasattr(geo, "update_transform_matrix"):
                        geo.update_transform_matrix()

                    geometries.append(geo)
                    continue

                geo = Geometry(
                    geo_type=geo_type,
                    name=geom_name or "Object",
                    position=pos,
                    size=size,
                    rotation=euler,
                    parent=None
                )

                # 保留原始 MJCF 属性（导出直通）
                geo.mjcf_attrs = dict(geom.attrib)
                geo._mjcf_had_name = ('name' in geom.attrib)

                # 设置颜色
                geo.material.color = color

                # 更新矩阵
                if hasattr(geo, "update_transform_matrix"):
                    geo.update_transform_matrix()

                # 直接作为“顶层对象”加入场景
                geometries.append(geo)

            # 处理 worldbody 下的直接 joint（无 body 包裹的情况）
            for joint_elem in world_body.findall("joint"):
                joint_type = (joint_elem.get('type') or 'hinge').lower()
                if joint_type not in ('hinge', 'slide'):
                    continue

                joint_name = joint_elem.get('name', 'world_joint')
                joint_pos = list(map(float, joint_elem.get('pos', '0 0 0').split())) if 'pos' in joint_elem.attrib else [0.0, 0.0, 0.0]

                axis_attr = joint_elem.get('axis')
                axis_vec = None
                if axis_attr:
                    parts = axis_attr.replace(',', ' ').split()
                    if len(parts) >= 3:
                        try:
                            axis_vec = [float(parts[i]) for i in range(3)]
                        except Exception:
                            axis_vec = None
                if axis_vec is None:
                    axis_vec = [1.0, 0.0, 0.0]

                joint_rotation = [0.0, 0.0, 0.0]
                joint_length = float(joint_elem.attrib.get('length', 0.4))
                half_len = max(joint_length * 0.5, 0.05)

                joint_geo = Geometry(
                    geo_type=GeometryType.JOINT.value,
                    name=joint_name,
                    position=joint_pos,
                    size=(half_len, 0.015, 0.015),
                    rotation=joint_rotation,
                    parent=None
                )

                joint_color = (1.0, 0.9, 0.2, 1.0) if joint_type == 'hinge' else (0.4, 0.8, 1.0, 1.0)
                joint_geo.material.color = joint_color
                joint_geo.mjcf_attrs = dict(joint_elem.attrib)
                joint_geo.joint_attrs = dict(joint_elem.attrib)
                joint_geo.joint_angle_mode = 'radian'
                if XMLParser._normalize_joint_angle_dict(joint_geo.joint_attrs, joint_type, XMLParser._last_loaded_angle_mode):
                    XMLParser._angles_normalized = True
                if XMLParser._normalize_joint_angle_dict(joint_geo.mjcf_attrs, joint_type, XMLParser._last_loaded_angle_mode):
                    XMLParser._angles_normalized = True
                joint_geo.joint_axis = axis_vec
                joint_geo.joint_length = joint_length
                joint_geo.joint_type = joint_type
                joint_geo._mjcf_had_name = ('name' in joint_elem.attrib)

                if hasattr(joint_geo, "update_transform_matrix"):
                    joint_geo.update_transform_matrix()

                geometries.append(joint_geo)

        
        # 最后，对所有对象进行两遍更新以确保变换正确传播
        # 第一遍：更新所有对象的本地变换
        XMLParser._update_transforms_recursive(geometries)
        
        # 第二遍：确保世界变换正确传播
        XMLParser._update_world_transforms_recursive(geometries)
        XMLParser._angles_normalized = True
        
        return geometries
    
    @staticmethod
    def _update_transforms_recursive(objects):
        """递归更新所有几何体的变换矩阵"""
        if isinstance(objects, list):
            for obj in objects:
                XMLParser._update_transforms_recursive(obj)
        else:
            # 更新当前对象的变换矩阵
            if hasattr(objects, "update_transform_matrix"):
                objects.update_transform_matrix()
            
            # 如果是组，递归更新子对象
            if hasattr(objects, "children") and objects.children:
                for child in objects.children:
                    XMLParser._update_transforms_recursive(child)
    
    @staticmethod
    def _update_world_transforms_recursive(objects):
        """递归更新所有几何体的世界变换矩阵"""
        if isinstance(objects, list):
            for obj in objects:
                XMLParser._update_world_transforms_recursive(obj)
        else:
            # 更新当前对象的全局变换
            if hasattr(objects, "update_global_transform"):
                objects.update_global_transform()
            elif hasattr(objects, "update_transform_matrix"):
                # 如果没有专门的全局变换更新方法，使用常规更新
                objects.update_transform_matrix()
            
            # 如果是组，递归更新子对象
            if hasattr(objects, "children") and objects.children:
                for child in objects.children:
                    XMLParser._update_world_transforms_recursive(child)
    
    @staticmethod
    def export_mujoco_xml(filename, geometries, *, preserve_auxiliary=True):
        try:
            use_cached = isinstance(getattr(XMLParser, "_last_loaded_mjcf_root", None), ET.Element)

            used_mesh_names = XMLParser._collect_mesh_names(geometries)
            if used_mesh_names or XMLParser._mesh_assets:
                XMLParser._mesh_assets = {
                    name: info for name, info in XMLParser._mesh_assets.items()
                    if name in used_mesh_names
                }
                if XMLParser._mesh_instances:
                    filtered_instances = []
                    for inst in XMLParser._mesh_instances:
                        mesh_key = inst.get("mesh") or inst.get("name")
                        if mesh_key in used_mesh_names:
                            filtered_instances.append(inst)
                    XMLParser._mesh_instances = filtered_instances

            base_root = None
            if use_cached:
                base_root = XMLParser._last_loaded_mjcf_root
            elif XMLParser._active_source_key:
                ctx = XMLParser._context_by_file.get(XMLParser._active_source_key)
                if ctx and isinstance(ctx.get('root_snapshot'), ET.Element):
                    base_root = ctx.get('root_snapshot')
                    use_cached = True

            if preserve_auxiliary and (not use_cached or base_root is None) and filename and os.path.exists(filename):
                try:
                    parsed_root = ET.parse(filename).getroot()
                    if parsed_root.tag == "mujoco":
                        # 当缓存上下文缺失时，直接读取磁盘上的旧文件，保留 worldbody 之外的结构
                        base_root = fromstring(ET.tostring(parsed_root))
                        use_cached = True
                except Exception:
                    base_root = None

            if preserve_auxiliary and use_cached and base_root is not None:
                # 以原始根为底稿，仅替换 body/geom，保持其余结构不变
                root = fromstring(ET.tostring(base_root))

                # 一律导出为弧度
                # 不重复添加 <compiler angle="radian"/>：只要已经存在，就不再新增
                compiler_elems = root.findall("compiler")
                has_radian = False
                for comp in compiler_elems:
                    angle_attr = (comp.get("angle") or "").strip().lower()
                    if angle_attr != "radian":
                        comp.set("angle", "radian")
                        angle_attr = "radian"
                    if angle_attr == "radian":
                        has_radian = True
                if not compiler_elems:
                    comp = ET.SubElement(root, "compiler")
                    comp.set("angle", "radian")
                elif not has_radian:
                    compiler_elems[0].set("angle", "radian")

                # worldbody：只替换 body/geom，其它孩子（camera/light/...）保留
                worldbody = root.find("worldbody") or ET.SubElement(root, "worldbody")

                # 收集需要保留的节点（camera/light/...），joint 也应由当前场景重建
                keep_nodes = [ch for ch in list(worldbody) if ch.tag not in ("body", "geom", "joint")]

                # 清空 worldbody 的所有子节点
                for ch in list(worldbody):
                    worldbody.remove(ch)

                # 先把保留节点按原顺序放回去
                for node in keep_nodes:
                    worldbody.append(node)

                # 再重建当前场景的 body/geom
                for obj in geometries:
                    XMLParser._add_object_to_mujoco(worldbody, obj)

                # 若原文件 angle=degree，则把所有 joint 的角度字段（range/ref）按“度→弧度”批量转换
                if (getattr(XMLParser, "_last_loaded_angle_mode", "radian") == "degree"
                        and not getattr(XMLParser, "_angles_normalized", True)):
                    XMLParser._convert_joint_angles_to_radian(root)

            else:
                # 原逻辑（新建场景）
                root = ET.Element("mujoco")
                root.set("model", "MJCFScene")
                compiler = ET.SubElement(root, "compiler")
                compiler.set("angle", "radian")
                asset = ET.SubElement(root, "asset")
                worldbody = ET.SubElement(root, "worldbody")
                for obj in geometries:
                    XMLParser._add_object_to_mujoco(worldbody, obj)

            export_dir = os.path.abspath(os.path.dirname(filename)) if filename else XMLParser._last_xml_dir
            XMLParser._sync_mesh_assets(root, export_dir)

            rough = ET.tostring(root, "utf-8")
            pretty = minidom.parseString(rough).toprettyxml(indent="  ", newl="\n")
            # 去掉空白行
            pretty = "\n".join(line for line in pretty.splitlines() if line.strip())
            with open(filename, "w", encoding="utf-8") as f:
                f.write(pretty + "\n")
            XMLParser._update_active_context()
            return True


        except Exception as e:
            print(f"导出MuJoCo XML时出错: {e}")
            return False

    
    @staticmethod
    def _normalize_mesh_path(path: str, export_dir: str, meshdir_attr: str = "") -> str:
        if not path:
            return ""
        normalized = os.path.abspath(path)
        if meshdir_attr:
            try:
                meshdir_full = os.path.normpath(os.path.join(export_dir, meshdir_attr))
                rel_meshdir = os.path.relpath(normalized, meshdir_full)
                if not rel_meshdir.startswith('..'):
                    rel_meshdir = rel_meshdir.replace('\\', '/')
                    if rel_meshdir == '.':
                        return ''
                    return rel_meshdir
            except Exception:
                pass
        try:
            rel = os.path.relpath(normalized, export_dir) if export_dir else normalized
        except Exception:
            rel = normalized
        return rel.replace('\\', '/')

    @staticmethod
    def _sync_mesh_assets(root, export_dir):
        asset_elems = root.findall("asset")
        asset_elem = asset_elems[0] if asset_elems else None

        if asset_elem is None:
            asset_elem = ET.SubElement(root, "asset")
        else:
            # ElementTree treats empty elements as falsy in boolean context, so avoid
            # duplicating <asset> by consolidating any additional siblings into the first.
            for extra in asset_elems[1:]:
                for child in list(extra):
                    asset_elem.append(child)
                root.remove(extra)
        existing = {mesh.get("name"): mesh for mesh in asset_elem.findall("mesh")}

        # 清理已不存在于当前资产表中的旧 mesh
        for name, elem in list(existing.items()):
            if not name or name not in XMLParser._mesh_assets:
                asset_elem.remove(elem)
                existing.pop(name, None)

        meshdir_attr = ""
        compiler = root.find("compiler")
        if compiler is not None:
            meshdir_attr = compiler.get("meshdir", "") or ""

        for name, info in XMLParser._mesh_assets.items():
            if not name:
                continue

            mesh_elem = existing.get(name)
            if mesh_elem is None:
                mesh_elem = ET.SubElement(asset_elem, "mesh")
                mesh_elem.set("name", name)

            orig_attr = info.get('_orig_file_attr')
            if orig_attr and not info.get('_is_new', False):
                mesh_elem.set("file", orig_attr)
            else:
                file_path = info.get('abs_file') or info.get('file', "")
                normalized = XMLParser._normalize_mesh_path(file_path, export_dir, meshdir_attr)
                mesh_elem.set("file", normalized)
                info['_orig_file_attr'] = normalized
                info['file'] = normalized
                info['_is_new'] = False

            scale_vals = info.get("scale", [1.0, 1.0, 1.0])
            try:
                scale_vals = [float(s) for s in scale_vals]
            except Exception:
                scale_vals = [1.0, 1.0, 1.0]

            if info.get('has_scale_attr', False):
                mesh_elem.set("scale", " ".join(f"{s:g}" for s in scale_vals))
            elif any(abs(s - 1.0) > 1e-6 for s in scale_vals):
                mesh_elem.set("scale", " ".join(f"{s:g}" for s in scale_vals))
            else:
                mesh_elem.attrib.pop("scale", None)

        XMLParser._new_mesh_assets.clear()


    @staticmethod
    def _add_object_to_mujoco(parent_elem, obj, prefix=""):
        if obj.type == "group":
            # 有原始 <body> 快照,用它为底稿
            if hasattr(obj, "_mjcf_body_snapshot") and isinstance(obj._mjcf_body_snapshot, ET.Element):
                body_elem = ET.fromstring(ET.tostring(obj._mjcf_body_snapshot))
                parent_elem.append(body_elem)

                # 更新 name/pos/euler
                body_elem.set("name", obj.name)
                body_elem.set("pos", f"{obj.position[0]} {obj.position[1]} {obj.position[2]}")
                rot = np.asarray(obj.rotation, dtype=float)
                r = np.radians(rot)  # 编辑器内部存的是角度，这里再转回弧度
                body_elem.set("euler", f"{r[0]} {r[1]} {r[2]}")
                # 始终使用欧拉角导出，移除与其冲突的姿态属性
                for orient_key in ("quat", "axisangle", "xyaxes", "zaxis"):
                    if orient_key in body_elem.attrib:
                        body_elem.attrib.pop(orient_key, None)

                # 清掉旧 geom，准备写入当前几何体
                for g in list(body_elem.findall("geom")):
                    body_elem.remove(g) 

                # 也清旧的 joint/body，避免重复添加
                for j in list(body_elem.findall("joint")):
                    body_elem.remove(j)
                for b in list(body_elem.findall("body")):
                    body_elem.remove(b)

                # 递归写入子对象
                for child in obj.children:
                    XMLParser._add_object_to_mujoco(body_elem, child, prefix=prefix) # 递归地写入子body和gemo

            else:
                # 没快照(新建场景):新建 body，并递归写入
                body_elem = ET.SubElement(parent_elem, "body")
                body_elem.set("name", obj.name)
                body_elem.set("pos", f"{obj.position[0]} {obj.position[1]} {obj.position[2]}")
                rot = np.asarray(obj.rotation, dtype=float)
                r = np.radians(rot)  # 统一以弧度写回 MJCF
                body_elem.set("euler", f"{r[0]} {r[1]} {r[2]}")
                for orient_key in ("quat", "axisangle", "xyaxes", "zaxis"):
                    body_elem.attrib.pop(orient_key, None)

                for child in obj.children:
                    XMLParser._add_object_to_mujoco(body_elem, child, prefix=prefix)

        else:
            if obj.type == GeometryType.JOINT.value or obj.type == "joint":
                joint_elem = ET.SubElement(parent_elem, "joint")
                attrs = {}
                if hasattr(obj, "mjcf_attrs") and isinstance(obj.mjcf_attrs, dict):
                    attrs.update(obj.mjcf_attrs)
                if hasattr(obj, "joint_attrs") and isinstance(obj.joint_attrs, dict):
                    attrs.update(obj.joint_attrs)

                joint_type = (attrs.get("type") or "hinge")
                joint_elem.set("type", joint_type)

                pos_vec = np.asarray(obj.position, dtype=float)
                joint_elem.set("pos", f"{pos_vec[0]} {pos_vec[1]} {pos_vec[2]}")

                axis_text = None
                if isinstance(attrs, dict):
                    axis_text = attrs.get('axis')
                if axis_text:
                    joint_elem.set("axis", axis_text)
                else:
                    joint_elem.attrib.pop('axis', None)

                name_text = (attrs.get("name") or "").strip()
                if name_text:
                    joint_elem.set("name", name_text)
                elif getattr(obj, "_mjcf_had_name", False):
                    joint_elem.set("name", obj.name)

                for key, val in attrs.items():
                    if key in ("type", "pos", "axis", "name"):
                        continue
                    if val is None:
                        continue
                    text = str(val).strip()
                    if not text:
                        continue
                    if joint_type == 'hinge' and key in ('range', 'ref'):
                        parts = text.split()
                        try:
                            nums = [float(p) for p in parts]
                        except Exception:
                            nums = []
                        if nums:
                            threshold = 2.0 * np.pi + 1e-6
                            angle_mode = getattr(obj, 'joint_angle_mode', 'radian')
                            need_convert = angle_mode == 'degree'
                            if not need_convert:
                                need_convert = any(abs(n) > threshold for n in nums)
                            if need_convert:
                                nums = [float(np.radians(n)) for n in nums]
                                text = " ".join(f"{num:.12g}" for num in nums)
                                if hasattr(obj, 'joint_attrs') and isinstance(obj.joint_attrs, dict):
                                    obj.joint_attrs[key] = text
                                setattr(obj, 'joint_angle_mode', 'radian')
                                XMLParser._angles_normalized = True
                    joint_elem.set(key, text)
                return

            # 处理几何体geom
            geom_elem = ET.SubElement(parent_elem, "geom")

            # 是否具备“从 MJCF 读取时带来的原始属性快照”
            has_orig = hasattr(obj, "mjcf_attrs") and isinstance(getattr(obj, "mjcf_attrs", None), dict)

            # 先把原始属性全部写回
            if has_orig:
                for k, v in obj.mjcf_attrs.items():
                    if v is None:
                        continue
                    text = str(v).strip()
                    if not text:
                        continue
                    try:
                        geom_elem.set(k, text)
                    except Exception:
                        pass
                # 统一转用欧拉角后，去掉原始四元数等姿态字段
                for orient_key in ("quat", "axisangle", "xyaxes", "zaxis"):
                    geom_elem.attrib.pop(orient_key, None)

            # 类型：以当前编辑器中的类型为准（可能你在 UI 改了类型）
            geom_elem.set("type", obj.type)

            if (obj.type == "mesh" or obj.type == GeometryType.MESH.value):
                mesh_name = getattr(obj, 'mesh_name', None)
                mesh_path = getattr(obj, 'mesh_path', None)
                if mesh_name and mesh_path and mesh_name not in XMLParser._mesh_assets:
                    scale = getattr(obj, 'mesh_asset_scale', [1.0, 1.0, 1.0])
                    XMLParser.register_mesh_asset(mesh_name, mesh_path, scale, has_scale_attr=True)

            # 位置：总是用当前值覆盖（mesh 需要扣除原始局部偏移:从上述AABB那里得到的）
            pos_vec = np.asarray(obj.position, dtype=float)
            if (obj.type == GeometryType.MESH.value or obj.type == "mesh"):
                origin_offset = np.asarray(getattr(obj, "mesh_origin_offset", [0.0, 0.0, 0.0]), dtype=float)
                if origin_offset.shape[0] >= 3:
                    pos_vec = pos_vec - origin_offset[:3]
            geom_elem.set("pos", f"{pos_vec[0]} {pos_vec[1]} {pos_vec[2]}")

            # 旋转：内部用“度”，导出写“弧度”
            rot = np.asarray(obj.rotation, dtype=float)
            r = np.radians(rot)  # 0 0 0 也会得到 0 0 0
            geom_elem.set("euler", f"{r[0]} {r[1]} {r[2]}")

            # 尺寸：
            # primitive 一律写 size（按 MuJoCo 规则）
            # mesh：仅当“原始就带 size”或该对象是新建对象（无 mjcf_attrs）时才写 size
            if obj.type == "mesh":
                if not has_orig:
                    # 编辑器里新建的 mesh：保持你原本的写法
                    geom_elem.set("size", f"{obj.size[0]} {obj.size[1]} {obj.size[2]}")
                else:
                    if "size" in obj.mjcf_attrs:
                        # 原始就有 size：按当前值更新
                        geom_elem.set("size", f"{obj.size[0]} {obj.size[1]} {obj.size[2]}")
                    else:
                        # 原始没有 size：不要凭空写入
                        geom_elem.attrib.pop("size", None)
            elif obj.type == GeometryType.SPHERE.value:
                geom_elem.set("size", f"{obj.size[0]}")
            elif obj.type in [GeometryType.CYLINDER.value, GeometryType.CAPSULE.value]:
                geom_elem.set("size", f"{obj.size[0]} {obj.size[2]}")
            elif obj.type == "plane":
                geom_elem.set("size", "0 0 0.01")
            else:
                geom_elem.set("size", f"{obj.size[0]} {obj.size[1]} {obj.size[2]}")

            # 颜色：总是用当前值覆盖
            geom_elem.set("rgba", f"{obj.material.color[0]} {obj.material.color[1]} {obj.material.color[2]} {obj.material.color[3]}")

            extra_attrs = obj.mjcf_attrs if isinstance(getattr(obj, "mjcf_attrs", None), dict) else {}
            if extra_attrs:
                for key, val in extra_attrs.items():
                    if key in ("type", "pos", "quat", "euler", "rgba", "size", "name"):
                        continue
                    if val is None:
                        continue
                    text = str(val).strip()
                    if not text:
                        continue
                    geom_elem.set(key, text)

            # name：
            # 若是从文件来的对象：只有“原文件里本来就有 name”才写 name
            # 若是编辑器新建对象：照旧写 name
            if has_orig:
                had_name = getattr(obj, "_mjcf_had_name", False)
                if had_name:
                    geom_elem.set("name", obj.name)
                else:
                    geom_elem.attrib.pop("name", None)
            else:
                geom_elem.set("name", obj.name)

    @staticmethod
    def _quat_to_euler(quat):
        """四元数转欧拉角（ZYX顺序）"""
        # 实现四元数到欧拉角的转换
        # 这里使用简化的计算方法
        w, x, y, z = quat
        
        # 计算姿态角
        t0 = 2.0 * (w * x + y * z)
        t1 = 1.0 - 2.0 * (x * x + y * y)
        roll = np.degrees(np.arctan2(t0, t1))
        
        t2 = 2.0 * (w * y - z * x)
        t2 = np.clip(t2, -1.0, 1.0)
        pitch = np.degrees(np.arcsin(t2))
        
        t3 = 2.0 * (w * z + x * y)
        t4 = 1.0 - 2.0 * (y * y + z * z)
        yaw = np.degrees(np.arctan2(t3, t4))
        
        return [roll, pitch, yaw]

    @staticmethod
    def _axis_to_euler(axis):
        axis = np.array(axis, dtype=float)
        if axis.size < 3:
            axis = np.pad(axis, (0, 3-axis.size), constant_values=0.0)
        norm = np.linalg.norm(axis)
        if norm < 1e-8:
            return [0.0, 0.0, 0.0]
        axis = axis / norm
        default = np.array([1.0, 0.0, 0.0])
        if np.allclose(axis, default):
            rot_matrix = np.eye(3)
        elif np.allclose(axis, -default):
            rot_matrix = np.array([[-1, 0, 0], [0, -1, 0], [0, 0, 1]], dtype=float)
        else:
            v = np.cross(default, axis)
            s = np.linalg.norm(v)
            c = float(np.dot(default, axis))
            vx = np.array([[0, -v[2], v[1]],
                           [v[2], 0, -v[0]],
                           [-v[1], v[0], 0]], dtype=float)
            rot_matrix = np.eye(3) + vx + vx @ vx * ((1 - c) / (s ** 2))
        return R.from_matrix(rot_matrix).as_euler('XYZ', degrees=True).tolist()
    
    
    # 保存方法别名，使用增强XML格式
    save = export_mujoco_xml

    @staticmethod
    def _convert_joint_angles_to_radian(root):
        """
        把所有 joint 的角度字段（range/ref）从“度”转成“弧度”。slide 关节不转换。
        """
        for je in root.findall(".//joint"):
            jt = (je.get("type") or "hinge").lower()

            # range: 两个数
            rng = je.get("range")
            if rng:
                parts = rng.split()
                if len(parts) >= 2:
                    try:
                        lo, hi = float(parts[0]), float(parts[1])
                        if jt != "slide":      # slide 的 range 表示位移，保留原值
                            lo, hi = np.radians([lo, hi])
                        je.set("range", f"{lo:g} {hi:g}")
                    except Exception:
                        pass

            # ref: 一个数
            rv = je.get("ref")
            if rv:
                try:
                    ref = float(rv)
                    if jt != "slide":   # hinge 的参考姿态是角度，slide 仍保持线性单位
                        ref = float(np.radians(ref))
                    je.set("ref", f"{ref:g}")
                except Exception:
                    pass

        XMLParser._angles_normalized = True

    @staticmethod
    def get_mesh_instances():
        """
        返回最近一次 load() 解析出的 mesh 实例列表：
        每项: {"path": 绝对路径, "scale": [sx,sy,sz], "transform": 4x4 列表, "color": [r,g,b,a]}
        """
        out = []
        base = XMLParser._last_xml_dir or ""
        meshdir = XMLParser._last_meshdir or ""
        for inst in XMLParser._mesh_instances:
            file = inst.get("file", "")
            # 解析绝对路径：<compiler meshdir> + 相对 file
            path = file
            if not os.path.isabs(path):
                path = os.path.normpath(os.path.join(base, meshdir, file))

            s_asset = np.array(inst.get("asset_scale", [1,1,1]), dtype=float)
            s_geom  = np.array(inst.get("geom_scale",  [1,1,1]), dtype=float)
            s_total = (s_asset * s_geom).tolist()

            out.append({
                "path": path,
                "scale": s_total,
                "transform": np.array(inst["transform"]).tolist(),
                "color": inst.get("color", [0.8,0.8,0.8,1.0]),
            })
        return out
