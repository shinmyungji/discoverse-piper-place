"""Utility helpers for loading triangle meshes from OBJ/STL files."""

from __future__ import annotations

import os
import struct
from typing import Optional, Tuple

import numpy as np


MeshData = Tuple[np.ndarray, Optional[np.ndarray]]


def load_mesh(path: str) -> MeshData:
    """Load an OBJ or STL mesh and return triangles plus optional normals."""
    # 统一入口：根据扩展名选择 OBJ 或 STL 解析器，返回三角面与可选法线

    ext = os.path.splitext(path)[1].lower()
    if ext == ".obj":
        return load_obj(path)
    if ext == ".mtl":
        # 允许直接选择 MTL 文件，默认寻找同名 OBJ 并载入
        candidate = os.path.splitext(path)[0] + ".obj"
        if os.path.exists(candidate):
            return load_obj(candidate)
        raise ValueError(f"MTL 文件需要同名 OBJ 支持: {path}")
    if ext == ".stl":
        return load_stl(path)
    raise ValueError(f"Unsupported mesh extension: {ext}")


def load_obj(path: str) -> MeshData:
    """Parse a minimal subset of OBJ (v, vn, f).

    Returns a tuple ``(triangles, normals)`` where ``triangles`` has shape
    ``(N, 3, 3)``. ``normals`` may be ``None`` or match the triangles shape.
    """

    vs = []
    vns = []
    triangles = []
    tri_normals = []

    def parse_f_token(token: str):
        parts = token.split("/")
        vi = int(parts[0]) if parts[0] else 0
        vti = int(parts[1]) if len(parts) > 1 and parts[1] else 0
        vni = int(parts[2]) if len(parts) > 2 and parts[2] else 0
        return vi, vti, vni

    with open(path, "r", encoding="utf-8", errors="ignore") as f:
        for raw in f:
            line = raw.strip()
            if not line or line.startswith("#"):
                continue
            if line.startswith("v "):
                _, x, y, z = line.split()[:4]
                vs.append([float(x), float(y), float(z)])
            elif line.startswith("vn "):
                _, x, y, z = line.split()[:4]
                vns.append([float(x), float(y), float(z)])
            elif line.startswith("f "):
                tokens = line.split()[1:]
                if len(tokens) < 3:
                    continue

                fv = [parse_f_token(tok) for tok in tokens]
                for i in range(1, len(fv) - 1):
                    tri_idx = [fv[0], fv[i], fv[i + 1]]
                    tri_vertices = []
                    tri_norm = []
                    for (vi, _, vni) in tri_idx:
                        vi = vi - 1 if vi > 0 else len(vs) + vi
                        tri_vertices.append(vs[vi])
                        if vni != 0:
                            vni = vni - 1 if vni > 0 else len(vns) + vni
                            tri_norm.append(vns[vni])
                    triangles.append(tri_vertices)
                    tri_normals.append(tri_norm if len(tri_norm) == 3 else None)

    triangles_np = np.asarray(triangles, dtype=np.float32)
    if len(triangles_np) == 0:
        return triangles_np, None

    if not tri_normals or all(n is None for n in tri_normals):
        normals_np = None
    else:
        packed = []
        for tri, nrm in zip(triangles_np, tri_normals):
            if nrm is None:
                v0, v1, v2 = tri
                n = np.cross(v1 - v0, v2 - v0)
                ln = np.linalg.norm(n) + 1e-12
                packed.append([*(n / ln)] * 3)
            else:
                packed.append(nrm)
        normals_np = np.asarray(packed, dtype=np.float32)

    return triangles_np, normals_np


def load_stl(path: str) -> MeshData:
    """Parse binary or ASCII STL into triangles plus optional normals."""

    with open(path, "rb") as fb:
        header = fb.read(80)
        try:
            tri_count = struct.unpack("<I", fb.read(4))[0]
            expected_len = 80 + 4 + tri_count * 50
            fb.seek(0, os.SEEK_END)
            actual_len = fb.tell()
            if actual_len == expected_len:
                fb.seek(84)
                tris = []
                norms = []
                for _ in range(tri_count):
                    n = struct.unpack("<fff", fb.read(12))
                    v0 = struct.unpack("<fff", fb.read(12))
                    v1 = struct.unpack("<fff", fb.read(12))
                    v2 = struct.unpack("<fff", fb.read(12))
                    fb.read(2)
                    tris.append([v0, v1, v2])
                    norms.append(n)
                triangles_np = np.asarray(tris, dtype=np.float32)
                normals_np = np.asarray(norms, dtype=np.float32)
                return triangles_np, normals_np
        except Exception:
            pass

    tris_accum = []
    norms_accum = []
    all_vertices = []
    cur_vertices = []
    cur_normal = None

    def compute_normal(v0, v1, v2):
        v0 = np.asarray(v0, dtype=np.float64)
        v1 = np.asarray(v1, dtype=np.float64)
        v2 = np.asarray(v2, dtype=np.float64)
        n = np.cross(v1 - v0, v2 - v0)
        ln = float(np.linalg.norm(n))
        if ln == 0.0:
            return (0.0, 0.0, 1.0)
        return (float(n[0] / ln), float(n[1] / ln), float(n[2] / ln))

    with open(path, "r", encoding="utf-8", errors="ignore") as f:
        for raw in f:
            s = raw.strip()
            if not s:
                continue
            lower = s.lower()

            if lower.startswith("facet normal"):
                parts = s.split()
                cur_normal = None
                if len(parts) >= 5:
                    try:
                        cur_normal = (
                            float(parts[2]),
                            float(parts[3]),
                            float(parts[4]),
                        )
                    except Exception:
                        cur_normal = None
                cur_vertices = []
            elif lower.startswith("vertex"):
                parts = s.split()
                if len(parts) >= 4:
                    try:
                        x, y, z = float(parts[1]), float(parts[2]), float(parts[3])
                        vertex = [x, y, z]
                        cur_vertices.append(vertex)
                        all_vertices.append(vertex)
                    except Exception:
                        pass
            elif lower.startswith("endfacet"):
                if len(cur_vertices) >= 3:
                    v0 = cur_vertices[0]
                    for i in range(1, len(cur_vertices) - 1):
                        tri = [v0, cur_vertices[i], cur_vertices[i + 1]]
                        tris_accum.append(tri)
                        n = cur_normal
                        if not n or n == (0.0, 0.0, 0.0):
                            n = compute_normal(*tri)
                        norms_accum.append(n)
                cur_vertices = []
                cur_normal = None

    if tris_accum:
        triangles_np = np.asarray(tris_accum, dtype=np.float32)
        normals_np = (
            np.asarray(norms_accum, dtype=np.float32)
            if len(norms_accum) == len(tris_accum)
            else None
        )
        return triangles_np, normals_np

    if len(all_vertices) >= 3:
        usable = (len(all_vertices) // 3) * 3
        triangles_np = np.asarray(all_vertices[:usable], dtype=np.float32).reshape(-1, 3, 3)
        return triangles_np, None

    return np.empty((0, 3, 3), dtype=np.float32), None
