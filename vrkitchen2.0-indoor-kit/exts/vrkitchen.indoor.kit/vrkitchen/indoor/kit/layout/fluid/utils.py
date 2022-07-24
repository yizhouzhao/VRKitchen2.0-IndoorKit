import math
from pxr import Gf
import numpy as np
import copy

def point_sphere(samples, scale):
    indices = [x + 0.5 for x in range(0, samples)]

    phi = [math.acos(1 - 2 * x / samples) for x in indices]
    theta = [math.pi * (1 + 5**0.5) * x for x in indices]

    x = [math.cos(th) * math.sin(ph) * scale for (th, ph) in zip(theta, phi)]
    y = [math.sin(th) * math.sin(ph) * scale for (th, ph) in zip(theta, phi)]
    z = [math.cos(ph) * scale for ph in phi]
    points = [Gf.Vec3f(x, y, z) for (x, y, z) in zip(x, y, z)]
    return points

#generate inside mesh
def swapPositions(list, pos1, pos2):
     
    list[pos1], list[pos2] = list[pos2], list[pos1]
    return list
    

def generate_inside_mesh(lowerCenter: Gf.Vec3f, h: float, radius: float, sphereDiameter: float, mesh, scale):
    # print("bounds: ", mesh.bounds)
    # samples = generate_hcp_samples(Gf.Vec3f(-radius, 0, -radius), Gf.Vec3f(radius, h, radius), sphereDiameter)
    min_bound = list(mesh.bounds[0])
    max_bound = list(mesh.bounds[1])
    min_bound = [min_bound[0], min_bound[2], min_bound[1]]
    max_bound = [max_bound[0], max_bound[2], max_bound[1]]

    min_bound = (item * scale for item in min_bound)
    max_bound = (item * scale for item in max_bound)

    samples = generate_hcp_samples(Gf.Vec3f(*min_bound), Gf.Vec3f(*max_bound), sphereDiameter*2)
    finalSamples = []
    import copy
    import trimesh

    samples_copy = copy.deepcopy(samples)

    samples_copy = [ [ sample_copy[0]/scale, sample_copy[1]/scale, sample_copy[2]/scale ] for sample_copy in samples_copy ]
    samples_copy = [ [ sample_copy[0], sample_copy[2], sample_copy[1] ] for sample_copy in samples_copy ]
    # print("num particles: ", len(samples_copy))
    print("eva contains:")
    contains =  mesh.contains(samples_copy)
    
    
    # signed_distance = trimesh.proximity.ProximityQuery(mesh).signed_distance(samples_copy)
    # contains =  signed_distance >= 0
    print("eva done:")

    for contain, sample in zip(contains, samples):
        if contain:
            finalSamples.append(sample)
            
    print("length: ", len(finalSamples) )
    return finalSamples

def in_hull(p, hull):
    """
    Test if points in `p` are in `hull`

    `p` should be a `NxK` coordinates of `N` points in `K` dimensions
    `hull` is either a scipy.spatial.Delaunay object or the `MxK` array of the 
    coordinates of `M` points in `K`dimensions for which Delaunay triangulation
    will be computed
    """
    try:
        from scipy.spatial import Delaunay
    except:
        import omni
        omni.kit.pipapi.install("scipy")
        from scipy.spatial import Delaunay
        
    if not isinstance(hull,Delaunay):
        hull = Delaunay(hull)

    return hull.find_simplex(p)>=0

def generate_inside_point_cloud(sphereDiameter, cloud_points, scale = 1):
    """
    Generate sphere packs inside a point cloud
    """
    offset = 2
    min_x = np.min(cloud_points[:, 0]) + offset
    min_y = np.min(cloud_points[:, 1]) + offset
    min_z = np.min(cloud_points[:, 2]) + offset

    max_x = np.max(cloud_points[:, 0]) 
    max_y = np.max(cloud_points[:, 1]) 
    max_z = np.max(cloud_points[:, 2]) 

    
    min_bound = [min_x, min_y, min_z]
    max_bound = [max_x, max_y, max_z]
    
    min_bound = [item * scale for item in min_bound]
    max_bound = [item * scale for item in max_bound]

    samples = generate_hcp_samples(Gf.Vec3f(*min_bound), Gf.Vec3f(*max_bound), sphereDiameter)
    
    samples_copy = np.array(copy.deepcopy(samples))

    print("samples_copy", samples_copy.shape)
    
    finalSamples = []
    contains = in_hull(samples, cloud_points)
    max_particles = 2000

    for contain, sample in zip(contains, samples):
        if contain and len(finalSamples) < max_particles:
            finalSamples.append(sample)

    
    print("length: ", len(finalSamples) )
    return finalSamples
    

# generate cylinder points
def generate_cylinder_y(lowerCenter: Gf.Vec3f, h: float, radius: float, sphereDiameter: float):
    samples = generate_hcp_samples(Gf.Vec3f(-radius, 0, -radius), Gf.Vec3f(radius, h, radius), sphereDiameter)

    finalSamples = []

    for p in samples:
        r2 = p[0] * p[0] + p[2] * p[2]
        if r2 <= radius * radius:
            finalSamples.append(p + lowerCenter)

    return finalSamples

 # Generates hexagonal close packed samples inside an axis aligned bounding box
def generate_hcp_samples(boxMin: Gf.Vec3f, boxMax: Gf.Vec3f, sphereDiameter: float):

    layerDistance = math.sqrt(2.0 / 3.0) * sphereDiameter
    rowShift = math.sqrt(3.0) / 2.0 * sphereDiameter

    result = []
    layer1Offset = (1.0 / 3.0) * (
        Gf.Vec2f(0, 0) + Gf.Vec2f(0.5 * sphereDiameter, rowShift) + Gf.Vec2f(sphereDiameter, 0)
    )

    zIndex = 0
    while True:

        z = boxMin[2] + zIndex * layerDistance
        if z > boxMax[2]:
            break

        yOffset = layer1Offset[1] if zIndex % 2 == 1 else 0

        yIndex = 0
        while True:
            y = boxMin[1] + yIndex * rowShift + yOffset
            if y > boxMax[1]:
                break

            xOffset = 0
            if zIndex % 2 == 1:
                xOffset += layer1Offset[0]
                if yIndex % 2 == 1:
                    xOffset -= 0.5 * sphereDiameter
            elif yIndex % 2 == 1:
                xOffset += 0.5 * sphereDiameter

            xIndex = 0
            while True:
                x = boxMin[0] + xIndex * sphereDiameter + xOffset
                if x > boxMax[0]:
                    break

                result.append(Gf.Vec3f(x, y, z))
                xIndex += 1
            yIndex += 1
        zIndex += 1

    return result

def get_quat_from_extrinsic_xyz_rotation(angleXrad: float = 0.0, angleYrad: float = 0.0, angleZrad: float = 0.0):
    
    def rotate_around_axis(x, y, z, angle):
        s = math.sin(0.5 * angle)
        return Gf.Quatf(math.cos(0.5 * angle), s * x, s * y, s * z)

    # angles are in radians
    rotX = rotate_around_axis(1, 0, 0, angleXrad)
    rotY = rotate_around_axis(0, 1, 0, angleYrad)
    rotZ = rotate_around_axis(0, 0, 1, angleZrad)
    return rotZ * rotY * rotX