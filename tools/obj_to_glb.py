#!/usr/bin/env python3
"""
Convert a Wavefront OBJ + MTL file to a GLB (binary glTF 2.0) file.
Handles multi-material meshes and per-vertex normals.

Usage:
    python obj_to_glb.py <input.obj> <output.glb> [scale]

The optional scale factor uniformly scales all vertex positions.
Normals are automatically recalculated per-face (flat shading), which gives
the spacecraft a clean low-poly look consistent with the original Java3D render.
"""

import struct
import json
import math
import sys
import os


# ---------------------------------------------------------------------------
# OBJ / MTL parsing
# ---------------------------------------------------------------------------

def parse_mtl(path):
    """Returns {name: {'baseColor': [r,g,b,a], 'metallic': f, 'roughness': f}}"""
    mats = {}
    cur = None
    try:
        with open(path) as f:
            for line in f:
                p = line.strip().split()
                if not p:
                    continue
                if p[0] == 'newmtl':
                    cur = p[1]
                    mats[cur] = {'Kd': [0.8, 0.8, 0.8], 'Ka': [0.0, 0.0, 0.0]}
                elif cur:
                    if p[0] == 'Kd':
                        mats[cur]['Kd'] = [float(x) for x in p[1:4]]
                    elif p[0] == 'Ka':
                        mats[cur]['Ka'] = [float(x) for x in p[1:4]]
    except FileNotFoundError:
        pass
    return mats


def parse_obj(path):
    """
    Returns:
      positions: list of [x, y, z]
      normals:   list of [nx, ny, nz]
      groups:    list of {'name': str, 'material': str, 'faces': [[(vi,ni)…]]}
    Face indices are 0-based.  Each face is a triangle (3 (vi,ni) tuples).
    """
    positions, normals = [], []
    groups = []
    cur = None

    with open(path) as f:
        for line in f:
            p = line.strip().split()
            if not p:
                continue
            tok = p[0]
            if tok == 'v':
                positions.append([float(x) for x in p[1:4]])
            elif tok == 'vn':
                normals.append([float(x) for x in p[1:4]])
            elif tok == 'g':
                cur = {'name': p[1] if len(p) > 1 else 'default',
                       'material': None, 'faces': []}
                groups.append(cur)
            elif tok == 'usemtl' and cur is not None:
                cur['material'] = p[1]
            elif tok == 'f' and cur is not None:
                verts = []
                for ref in p[1:]:
                    parts = ref.split('/')
                    vi = int(parts[0]) - 1
                    ni = int(parts[2]) - 1 if len(parts) > 2 and parts[2] else 0
                    verts.append((vi, ni))
                # Fan-triangulate polygons (most faces here are already triangles)
                for i in range(1, len(verts) - 1):
                    groups[-1]['faces'].append([verts[0], verts[i], verts[i + 1]])

    return positions, normals, groups


# ---------------------------------------------------------------------------
# Geometry helpers
# ---------------------------------------------------------------------------

def cross(a, b):
    return [
        a[1]*b[2] - a[2]*b[1],
        a[2]*b[0] - a[0]*b[2],
        a[0]*b[1] - a[1]*b[0],
    ]


def norm(v):
    mag = math.sqrt(sum(x*x for x in v))
    return [x / mag for x in v] if mag > 1e-12 else [0.0, 0.0, 1.0]


def build_primitive(positions, normals_src, faces, scale, use_flat_normals=True):
    """
    Build flat vertex + index buffers for one material group.
    Returns (pos_floats, nrm_floats, idx_shorts_or_ints, min_pos, max_pos).
    If use_flat_normals=True the OBJ normals are ignored and we compute
    per-face normals (gives a crisp low-poly look).
    """
    pos_data = []   # flat list of floats: x,y,z per vertex
    nrm_data = []   # flat list of floats: nx,ny,nz per vertex
    idx_data = []   # triangle index list
    key_to_idx = {}
    next_idx = 0

    for face in faces:
        if use_flat_normals:
            # Compute face normal from cross product (ignore OBJ normals)
            p0 = [x * scale for x in positions[face[0][0]]]
            p1 = [x * scale for x in positions[face[1][0]]]
            p2 = [x * scale for x in positions[face[2][0]]]
            e1 = [p1[i] - p0[i] for i in range(3)]
            e2 = [p2[i] - p0[i] for i in range(3)]
            fn = norm(cross(e1, e2))
        tri_indices = []
        for (vi, ni) in face:
            if use_flat_normals:
                # Key on vertex position only (all verts in face share the face normal)
                face_id = id(face)
                key = (vi, face_id)
                fn_use = fn
            else:
                key = (vi, ni)
                fn_use = norm(normals_src[ni]) if normals_src else [0.0, 1.0, 0.0]

            if key not in key_to_idx:
                key_to_idx[key] = next_idx
                next_idx += 1
                p = [x * scale for x in positions[vi]]
                pos_data.extend(p)
                nrm_data.extend(fn_use)
            tri_indices.append(key_to_idx[key])
        idx_data.extend(tri_indices)

    # Bounding box
    px = pos_data[0::3]
    py = pos_data[1::3]
    pz = pos_data[2::3]
    min_pos = [min(px), min(py), min(pz)]
    max_pos = [max(px), max(py), max(pz)]

    return pos_data, nrm_data, idx_data, min_pos, max_pos


# ---------------------------------------------------------------------------
# glTF 2.0 / GLB writer
# ---------------------------------------------------------------------------

COMPONENT_FLOAT = 5126
COMPONENT_USHORT = 5123
COMPONENT_UINT = 5125
TARGET_ARRAY_BUFFER = 34962
TARGET_ELEMENT_ARRAY_BUFFER = 34963


def pack_floats(data):
    return struct.pack(f'{len(data)}f', *data)


def pack_indices(data, use_uint32):
    if use_uint32:
        return struct.pack(f'{len(data)}I', *data)
    return struct.pack(f'{len(data)}H', *data)


def align4(n):
    return (n + 3) & ~3


def write_glb(out_path, gltf_json, bin_data):
    json_bytes = json.dumps(gltf_json, separators=(',', ':')).encode('utf-8')
    # Pad to 4-byte boundary with spaces
    json_pad = (-len(json_bytes)) % 4
    json_bytes += b' ' * json_pad

    bin_pad = (-len(bin_data)) % 4
    bin_data += b'\x00' * bin_pad

    json_chunk_len = len(json_bytes)
    bin_chunk_len = len(bin_data)
    total_len = 12 + 8 + json_chunk_len + 8 + bin_chunk_len

    with open(out_path, 'wb') as f:
        # GLB header
        f.write(struct.pack('<III', 0x46546C67, 2, total_len))  # magic, version, length
        # JSON chunk
        f.write(struct.pack('<II', json_chunk_len, 0x4E4F534A))  # length, type=JSON
        f.write(json_bytes)
        # BIN chunk
        f.write(struct.pack('<II', bin_chunk_len, 0x004E4942))   # length, type=BIN
        f.write(bin_data)


# ---------------------------------------------------------------------------
# Main conversion
# ---------------------------------------------------------------------------

def mtl_to_gltf_material(name, mat_props):
    """Convert MTL Kd/Ka to a glTF 2.0 PBR material."""
    kd = mat_props.get('Kd', [0.8, 0.8, 0.8])
    ka = mat_props.get('Ka', [0.0, 0.0, 0.0])

    # If Ka has a negative component it was used as a tint flag in the legacy code
    # (body material has Ka = -1.0, 0.13, 1.0).  Clamp to 0+.
    ka = [max(0.0, x) for x in ka]

    # Blend Kd with Ka to get a representative base colour
    base = [min(1.0, kd[i] + ka[i]) for i in range(3)] + [1.0]

    # Heuristic: if Ka had any saturation, treat as slightly metallic (gold/foil)
    ka_sat = max(abs(x) for x in mat_props.get('Ka', [0]*3))
    metallic = 0.8 if ka_sat > 0.1 else 0.0
    roughness = 0.3 if metallic > 0 else 0.6

    return {
        'name': name,
        'pbrMetallicRoughness': {
            'baseColorFactor': base,
            'metallicFactor': metallic,
            'roughnessFactor': roughness,
        },
        'doubleSided': True,
    }


def convert(obj_path, glb_path, scale=1.0):
    obj_dir = os.path.dirname(obj_path)
    mtl_path = os.path.join(obj_dir, 'sparkymat.mtl')

    print(f"Parsing MTL: {mtl_path}")
    mat_props = parse_mtl(mtl_path)

    print(f"Parsing OBJ: {obj_path}")
    positions, normals_src, groups = parse_obj(obj_path)
    print(f"  {len(positions)} vertices, {len(normals_src)} normals, "
          f"{sum(len(g['faces']) for g in groups)} faces, "
          f"{len(groups)} groups")

    # Compute scale so the longest axis fits in target_size ER
    target_size = 0.18   # ER — visually prominent but not absurdly large
    all_x = [v[0] for v in positions]
    all_y = [v[1] for v in positions]
    all_z = [v[2] for v in positions]
    span = max(
        max(all_x) - min(all_x),
        max(all_y) - min(all_y),
        max(all_z) - min(all_z),
    )
    if scale == 1.0 and span > 0:
        scale = target_size / span
        print(f"  Auto-scale: span={span:.2f} OBJ units -> scale={scale:.6f} (target {target_size} ER)")

    # Build per-group geometry
    bin_parts = []
    buffer_views = []
    accessors = []
    primitives = []
    gltf_materials = []

    byte_offset = 0

    for g in groups:
        mat_name = g['material'] or 'default'
        mat_idx = len(gltf_materials)
        gltf_materials.append(
            mtl_to_gltf_material(mat_name, mat_props.get(mat_name, {}))
        )

        pos_data, nrm_data, idx_data, min_pos, max_pos = \
            build_primitive(positions, normals_src, g['faces'], scale,
                            use_flat_normals=True)

        use_uint32 = max(idx_data) > 65535
        idx_type = COMPONENT_UINT if use_uint32 else COMPONENT_USHORT
        n_verts = len(pos_data) // 3
        n_tris  = len(idx_data) // 3

        pos_bytes = pack_floats(pos_data)
        nrm_bytes = pack_floats(nrm_data)
        idx_bytes = pack_indices(idx_data, use_uint32)
        # Pad index buffer to 4-byte boundary
        idx_bytes += b'\x00' * ((-len(idx_bytes)) % 4)

        # BufferView: positions
        bv_pos = len(buffer_views)
        buffer_views.append({
            'buffer': 0,
            'byteOffset': byte_offset,
            'byteLength': len(pos_bytes),
            'target': TARGET_ARRAY_BUFFER,
        })
        acc_pos = len(accessors)
        accessors.append({
            'bufferView': bv_pos,
            'byteOffset': 0,
            'componentType': COMPONENT_FLOAT,
            'count': n_verts,
            'type': 'VEC3',
            'min': min_pos,
            'max': max_pos,
        })
        byte_offset += len(pos_bytes)
        bin_parts.append(pos_bytes)

        # BufferView: normals
        bv_nrm = len(buffer_views)
        buffer_views.append({
            'buffer': 0,
            'byteOffset': byte_offset,
            'byteLength': len(nrm_bytes),
            'target': TARGET_ARRAY_BUFFER,
        })
        acc_nrm = len(accessors)
        accessors.append({
            'bufferView': bv_nrm,
            'byteOffset': 0,
            'componentType': COMPONENT_FLOAT,
            'count': n_verts,
            'type': 'VEC3',
        })
        byte_offset += len(nrm_bytes)
        bin_parts.append(nrm_bytes)

        # BufferView: indices
        bv_idx = len(buffer_views)
        buffer_views.append({
            'buffer': 0,
            'byteOffset': byte_offset,
            'byteLength': len(idx_bytes),
            'target': TARGET_ELEMENT_ARRAY_BUFFER,
        })
        acc_idx = len(accessors)
        accessors.append({
            'bufferView': bv_idx,
            'byteOffset': 0,
            'componentType': idx_type,
            'count': len(idx_data),
            'type': 'SCALAR',
        })
        byte_offset += len(idx_bytes)
        bin_parts.append(idx_bytes)

        primitives.append({
            'attributes': {'POSITION': acc_pos, 'NORMAL': acc_nrm},
            'indices': acc_idx,
            'material': mat_idx,
            'mode': 4,  # TRIANGLES
        })

        print(f"  Group '{g['name']}' ({mat_name}): "
              f"{n_verts} verts, {n_tris} triangles")

    bin_data = b''.join(bin_parts)

    gltf = {
        'asset': {'version': '2.0', 'generator': 'obj_to_glb.py'},
        'scene': 0,
        'scenes': [{'nodes': [0]}],
        'nodes': [{'mesh': 0, 'name': 'Sparky'}],
        'meshes': [{'name': 'Sparky', 'primitives': primitives}],
        'materials': gltf_materials,
        'accessors': accessors,
        'bufferViews': buffer_views,
        'buffers': [{'byteLength': len(bin_data)}],
    }

    os.makedirs(os.path.dirname(glb_path), exist_ok=True)
    write_glb(glb_path, gltf, bin_data)
    size_kb = os.path.getsize(glb_path) / 1024
    print(f"Written: {glb_path} ({size_kb:.1f} KB)")


if __name__ == '__main__':
    if len(sys.argv) < 3:
        print("Usage: obj_to_glb.py <input.obj> <output.glb> [scale]")
        sys.exit(1)
    scale = float(sys.argv[3]) if len(sys.argv) > 3 else 1.0
    convert(sys.argv[1], sys.argv[2], scale)
