import omni

try:
    import open3d as o3d
except:
    omni.kit.pipapi.install("open3d")
    omni.kit.pipapi.install("rtree")

try:
    import trimesh
except:
    omni.kit.pipapi.install("trimesh")

try:
    omni.kit.pipapi.install("transforms3d")
except:
    import transforms3d.quaternions as quaternions

import numpy as np
from math import atan2

class O3dMesh():
    def __init__(self, vertices, triangles) -> None:
        self.vertices = vertices
        self.triangles = triangles

def merge_mesh(meshes):
    # Merge without color and normal
    vertices = np.zeros((0, 3))
    triangles = np.zeros((0, 3))

    for mesh in meshes:
        vertices_i = np.asarray(mesh.vertices)
        triangles_i = np.asarray(mesh.triangles)
        triangles_i += vertices.shape[0]
        vertices = np.append(vertices, vertices_i, axis=0)
        triangles = np.append(triangles, triangles_i, axis=0)

    vertices = o3d.utility.Vector3dVector(vertices)
    triangles = o3d.utility.Vector3iVector(triangles)
    # print(vertices, triangles)
    # exit(0)
    mesh = o3d.geometry.TriangleMesh(vertices, triangles)
    mesh.compute_vertex_normals(normalized=True)
    mesh.compute_triangle_normals(normalized=True)
    return mesh

def o3d_to_trimesh(x):
    vertices = np.asarray(x.vertices)
    faces = np.asarray(x.triangles)
    return trimesh.Trimesh(vertices=vertices, faces=faces)
def trimesh_to_o3d(x):
    return o3d.geometry.TriangleMesh(o3d.utility.Vector3dVector(x.vertices), o3d.utility.Vector3iVector(x.faces))


    
def normalize_and_clip_in_interval(x, min_x, max_x=None):
    if max_x is None:
        min_x = -abs(min_x)
        max_x = abs(min_x)
    len_x = max_x - min_x
    return (min(max(x, min_x), max_x) - min_x) / len_x


def clip(x, min_x, max_x):
    return min(max(min_x, x), max_x)


def normalize_reward(x, norm_x):
    return x / norm_x


def norm(x, keepdims=False):
    return np.sqrt((x ** 2).sum(axis=-1, keepdims=keepdims))



# def angle_distance(q0, q1):
#     qd =  euler_angles_to_quat(matrix_to_euler_angles((inv(q0) * q1)))
#     return 2 * atan2(norm(qd[1:]), qd[0]) / pi

def rotation_diff(q0, q1):
    
    qd = quaternions.qmult( quaternions.qconjugate(q0),  q1)
    return 2 * atan2(norm(qd[1:]), qd[0]) / pi
    # angle = math.acos(abs(qd[0])) * 2

    # return math.degrees(angle)


def to_normal(x):
    if x.shape[-1] == 3:
        return x
    assert x.shape[-1] == 4
    return x[..., :3] / x[..., 3:]
    
def to_generalized(x):
    if x.shape[-1] == 4:
        return x
    assert x.shape[-1] == 3
    output_shape = list(x.shape)
    output_shape[-1] = 4
    ret = np.ones(output_shape)
    ret[..., :3] = x
    return ret
def apply_pose_to_points(x, pose):
#pose is the transformation matrix

    return to_normal(to_generalized(x) @ pose.T)
    # return to_normal(to_generalized(x) @ pose.to_transformation_matrix().T)

