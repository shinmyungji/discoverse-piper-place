"""
属性视图模型

作为属性视图和模型层之间的桥梁，处理属性相关的业务逻辑。
"""

import numpy as np
from PyQt5.QtCore import QObject, pyqtSignal

from ..model.xml_parser import XMLParser
from ..viewmodel.scene_viewmodel import SceneViewModel
class PropertyViewModel(QObject):
    """
    属性视图模型
    
    管理对象属性的业务逻辑，包括获取、设置属性值，并在属性更改时发出信号
    """
    # 信号：属性变化时触发，视图将更新显示
    propertiesChanged = pyqtSignal()
    
    # 添加新的信号
    propertyChanged = pyqtSignal(object, str, object)  # 属性变化信号（对象、属性名、新值）
    PHYSICS_ATTRS = [
        "condim", "friction", "solimp", "solref", "solmix", "margin", "gap",
        "density", "mass", "stiffness", "damping", "viscosity", "diaginertia",
        "contact", "priority"
    ]
    JOINT_EXTRA_ATTRS = [
        "range", "damping", "stiffness", "springref", "springstiff",
        "frictionloss", "armature", "ref", "margin"
    ]
    
    def __init__(self, scene_model:SceneViewModel):
        """
        初始化属性视图模型
        
        参数:
            scene_model: 场景模型的引用
        """
        super().__init__()
        self._scene_model = scene_model
        
        # 保存当前选中的对象
        self._selected_object = None
        
        # 连接场景模型的选择变化信号
        self._scene_model.selectionChanged.connect(self._on_selection_changed)
        
        # 连接对象属性变化信号
        self._scene_model.objectChanged.connect(self._on_object_changed)
    
    @property
    def selected_object(self):
        """获取当前选中的对象"""
        return self._selected_object
    
    def _on_selection_changed(self, selected_object):
        """
        处理场景中对象选择变化
        
        参数:
            selected_object: 新选中的对象
        """
        self._selected_object = selected_object
        self.propertiesChanged.emit()
    
    def _on_object_changed(self, obj):
        """
        处理对象属性变化
        
        参数:
            obj: 被修改的对象
        """
        if obj == self._selected_object:
            self.propertiesChanged.emit()
    
    def get_property(self, property_name):
        """
        获取对象的属性值
        
        参数:
            property_name: 属性名称
            
        返回:
            属性值或None（如果属性不存在或无选中对象）
        """
        if self._selected_object is None:
            return None
        
        # 基本属性
        if property_name == "name":
            return self._selected_object.name
        elif property_name == "type":
            if hasattr(self._selected_object.type, 'value'):
                return self._selected_object.type.value
            else:
                return self._selected_object.type  # 处理group类型，它的类型是字符串
        elif property_name == "visible":
            return getattr(self._selected_object, "visible", True)
        
        # 变换属性
        elif property_name == "position":
            return self._selected_object.position
        elif property_name == "position_x":
            return self._selected_object.position[0]
        elif property_name == "position_y":
            return self._selected_object.position[1]
        elif property_name == "position_z":
            return self._selected_object.position[2]
        
        elif property_name == "rotation":
            return self._selected_object.rotation
        elif property_name == "rotation_x":
            return self._selected_object.rotation[0]
        elif property_name == "rotation_y":
            return self._selected_object.rotation[1]
        elif property_name == "rotation_z":
            return self._selected_object.rotation[2]
        
        elif property_name == "scale":
            return self._selected_object.size
        elif property_name == "scale_x":
            return self._selected_object.size[0]
        elif property_name == "scale_y":
            return self._selected_object.size[1]
        elif property_name == "scale_z":
            return self._selected_object.size[2]
        
        # 材质属性
        elif property_name == "material_color":
            return self._selected_object.material.color
        elif property_name == "mesh_scale":
            if self.is_mesh_scale_editable():
                return getattr(self._selected_object, "mesh_asset_scale", None)
            return None
        elif property_name == "mesh_scale_x":
            scale = self.get_property("mesh_scale")
            return scale[0] if scale is not None else None
        elif property_name == "mesh_scale_y":
            scale = self.get_property("mesh_scale")
            return scale[1] if scale is not None else None
        elif property_name == "mesh_scale_z":
            scale = self.get_property("mesh_scale")
            return scale[2] if scale is not None else None
        elif property_name == "physics_attrs":
            return self.get_physics_attributes()
        elif property_name.startswith("physics_attr_"):
            attrs = self.get_physics_attributes()
            key = property_name[len("physics_attr_"):]
            return attrs.get(key)
        elif property_name == "joint_type":
            joint_type = getattr(self._selected_object, 'joint_type', None)
            if not joint_type:
                attrs = getattr(self._selected_object, 'joint_attrs', {}) or {}
                joint_type = attrs.get('type')
            return (joint_type or "hinge").lower()
        elif property_name == "joint_length":
            length = getattr(self._selected_object, 'joint_length', None)
            if length is not None:
                return length
            size = getattr(self._selected_object, 'size', None)
            if size is None or len(size) == 0:
                return None
            return float(size[0]) * 2.0
        elif property_name == "joint_axis":
            attrs = getattr(self._selected_object, 'joint_attrs', {}) or {}
            return attrs.get('axis', "")
        elif property_name == "joint_attrs":
            return self.get_joint_attributes()
        elif property_name.startswith("joint_attr_"):
            attrs = self.get_joint_attributes()
            key = property_name[len("joint_attr_"):]
            return attrs.get(key)

        return None
    
    def set_property(self, property_name, value):
        """
        设置对象的属性值
        
        参数:
            property_name: 属性名称
            value: 新的属性值
            
        返回:
            是否设置成功
        """
        if self._selected_object is None:
            return False
        
        # 类型属性不允许修改
        if property_name == "type":
            return False
            
        # 基本属性
        if property_name == "name":
            self._selected_object.name = value
        elif property_name == "visible":
            self._selected_object.visible = value
        
        # 变换属性
        elif property_name.startswith("position"):
            # 从当前坐标复制一份，按需更新单个分量，避免覆盖未修改的轴
            position = list(self._selected_object.position)
            
            if property_name == "position":
                position = value
            elif property_name == "position_x":
                position[0] = value
            elif property_name == "position_y":
                position[1] = value
            elif property_name == "position_z":
                position[2] = value
            
            self._selected_object.position = position
        
        elif property_name.startswith("rotation"):
            rotation = list(self._selected_object.rotation)
            
            if property_name == "rotation":
                rotation = value
            elif property_name == "rotation_x":
                rotation[0] = value
            elif property_name == "rotation_y":
                rotation[1] = value
            elif property_name == "rotation_z":
                rotation[2] = value
            
            self._selected_object.rotation = rotation
        
        elif property_name == "joint_type":
            if not isinstance(value, str):
                return False
            jt = value.strip().lower()
            if jt not in ("hinge", "slide"):
                return False
            self._selected_object.joint_type = jt
            if not hasattr(self._selected_object, 'joint_attrs') or not isinstance(self._selected_object.joint_attrs, dict):
                self._selected_object.joint_attrs = {}
            self._selected_object.joint_attrs['type'] = jt
            # 依据类型切换默认渲染颜色，便于快速区分
            if hasattr(self._selected_object, 'material') and hasattr(self._selected_object.material, 'color'):
                self._selected_object.material.color = (1.0, 0.9, 0.2, 1.0) if jt == 'hinge' else (0.4, 0.8, 1.0, 1.0)
            self.propertyChanged.emit(self._selected_object, property_name, jt)
            self._scene_model.notify_object_changed(self._selected_object)
            return True

        elif property_name == "joint_length":
            try:
                length = float(value)
            except Exception:
                return False
            length = max(length, 1e-5)
            size = list(getattr(self._selected_object, 'size', [length * 0.5, 0.015, 0.015]))
            if len(size) < 3:
                size += [0.015] * (3 - len(size))
            size[0] = length * 0.5
            # 保证厚度非零，避免可视化与拾取失效
            if abs(size[1]) < 1e-6:
                size[1] = 0.015
            if abs(size[2]) < 1e-6:
                size[2] = 0.015
            self._selected_object.size = size
            self._selected_object.joint_length = length
            # 若原始XML携带自定义长度字段则同步更新
            if hasattr(self._selected_object, 'joint_attrs') and isinstance(self._selected_object.joint_attrs, dict):
                if 'length' in self._selected_object.joint_attrs:
                    self._selected_object.joint_attrs['length'] = f"{length:g}"
            self.propertyChanged.emit(self._selected_object, property_name, length)
            self._scene_model.notify_object_changed(self._selected_object)
            return True

        elif property_name == "joint_axis":
            text = "" if value is None else str(value).strip()
            if not hasattr(self._selected_object, 'joint_attrs') or not isinstance(self._selected_object.joint_attrs, dict):
                self._selected_object.joint_attrs = {}

            if not text:
                self._selected_object.joint_attrs.pop('axis', None)
                self.propertyChanged.emit(self._selected_object, "joint_axis", "")
                self._scene_model.notify_object_changed(self._selected_object)
                return True

            self._selected_object.joint_attrs['axis'] = text
            self.propertyChanged.emit(self._selected_object, "joint_axis", text)
            self._scene_model.notify_object_changed(self._selected_object)
            return True

        elif property_name.startswith("joint_attr_"):
            attr_name = property_name[len("joint_attr_"):]
            if not hasattr(self._selected_object, 'joint_attrs') or not isinstance(self._selected_object.joint_attrs, dict):
                self._selected_object.joint_attrs = {}
            attrs = self._selected_object.joint_attrs
            text = "" if value is None else str(value).strip()

            store_text = text
            if attr_name in ("range", "ref") and text:
                joint_type = getattr(self._selected_object, 'joint_type', '').lower()
                if joint_type == 'hinge':
                    parts = text.split()
                    try:
                        nums = [float(p) for p in parts]
                        rad_nums = [np.radians(n) for n in nums]  # UI 输入角度，内部存弧度
                        store_text = " ".join(f"{num:.12g}" for num in rad_nums)
                        setattr(self._selected_object, 'joint_angle_mode', 'radian')
                    except Exception:
                        store_text = text

            attrs[attr_name] = store_text
            self.propertyChanged.emit(self._selected_object, property_name, value)
            self._scene_model.notify_object_changed(self._selected_object)
            return True

        elif property_name.startswith("mesh_scale"):
            if not self.is_mesh_scale_editable():
                return False
            scale = list(getattr(self._selected_object, 'mesh_asset_scale', [1.0, 1.0, 1.0]))
            if property_name == "mesh_scale":
                scale = value
            elif property_name == "mesh_scale_x":
                scale[0] = value
            elif property_name == "mesh_scale_y":
                scale[1] = value
            elif property_name == "mesh_scale_z":
                scale[2] = value
            self._scene_model.update_mesh_asset_scale(self._selected_object, scale)
            self.propertyChanged.emit(self._selected_object, property_name, value)
            return True

        elif property_name.startswith("physics_attr_"):
            attr_name = property_name[len("physics_attr_"):]
            if not hasattr(self._selected_object, 'mjcf_attrs') or not isinstance(self._selected_object.mjcf_attrs, dict):
                self._selected_object.mjcf_attrs = {}
            attrs = self._selected_object.mjcf_attrs
            text = "" if value is None else str(value).strip()
            attrs[attr_name] = text
            self.propertyChanged.emit(self._selected_object, property_name, value)
            self._scene_model.notify_object_changed(self._selected_object)
            return True

        elif property_name.startswith("scale"):
            size = list(self._selected_object.size)
            
            if property_name == "scale":
                size = value
            elif property_name == "scale_x":
                size[0] = value
            elif property_name == "scale_y":
                size[1] = value
            elif property_name == "scale_z":
                size[2] = value
            
            self._selected_object.size = size
        
        # 材质属性
        elif property_name == "material_color":
            self._selected_object.material.color = value
        
        else:
            return False
        
        # 发射属性变化信号
        self.propertyChanged.emit(self._selected_object, property_name, value)
        
        # 通知场景视图模型对象已更改
        self._scene_model.notify_object_changed(self._selected_object)
        
        # 属性更新后，如果是名称变更，通过场景视图模型触发层级树刷新
        if property_name == "name":
            # 触发场景模型的几何体变化和删除信号
            self._scene_model.geometriesChanged.emit()
            # 注意：通常geometryDeleted信号用于通知几何体被删除，这里使用可能不合适
            # 而且当修改名称时，几何体并没有被删除
            # self._scene_model.geometryDeleted.emit(self._selected_object)
            
            # 如果有层级视图模型引用，触发其层级变化信号
            if hasattr(self._scene_model, 'hierarchyViewModel') and self._scene_model.hierarchyViewModel:
                self._scene_model.hierarchyViewModel.hierarchyChanged.emit()
        
        return True

    def is_mesh_scale_editable(self):
        """透传给 SceneViewModel，判断当前选中 mesh 是否支持资产缩放编辑"""
        if self._selected_object is None:
            return False
        if hasattr(self._scene_model, 'is_mesh_scale_editable'):
            return self._scene_model.is_mesh_scale_editable(self._selected_object)
        return False

    def get_physics_attributes(self):
        if self._selected_object is None:
            return {}
        attrs = getattr(self._selected_object, 'mjcf_attrs', {}) or {}
        if not isinstance(attrs, dict):
            return {}
        result = {}
        for key in self.PHYSICS_ATTRS:
            if key in attrs:
                result[key] = attrs[key]
        return result

    def get_joint_attributes(self):
        if self._selected_object is None:
            return {}
        attrs = getattr(self._selected_object, 'joint_attrs', {}) or {}
        if not isinstance(attrs, dict):
            return {}
        result = {}
        for key in self.JOINT_EXTRA_ATTRS:
            if key not in attrs or attrs[key] is None:
                continue
            text = str(attrs[key]).strip()
            if not text:
                result[key] = ""
                continue
            if key in ("range", "ref"):
                joint_type = getattr(self._selected_object, 'joint_type', '').lower()
                if joint_type == 'hinge':
                    parts = text.split()
                    try:
                        nums = [float(p) for p in parts]
                        deg_nums = [np.degrees(n) for n in nums]  # 存储用弧度，界面展示角度
                        text = " ".join(f"{num:g}" for num in deg_nums)
                    except Exception:
                        pass
            result[key] = text
        return result
    
    def reset_properties(self):
        """
        重置属性面板
        
        清空当前选择并刷新属性面板
        """
        # 清除当前选择
        self._selected_object = None
        
        # 通知视图刷新
        self.propertiesChanged.emit() 
