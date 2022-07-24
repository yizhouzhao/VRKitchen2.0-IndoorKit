import carb
import math
from pathlib import Path
from pxr import Usd, UsdLux, UsdGeom, Sdf, Gf, Vt, UsdPhysics, PhysxSchema
import sys
#put schemaHelpers.py into path
from  omni.kitchen.asset.layout.fluid.schemaHelpers import PhysxParticleInstancePrototype, addPhysxParticleSystem
import omni.timeline

from typing import List
from omni.kitchen.asset.task_check.newJointCheck import JointCheck
import math


ASYNC_SIMULATION = "/persistent/physics/asyncSimRender"

def setGridFilteringPass(gridFilteringFlags: int, passIndex: int, operation: int, numRepetitions: int = 1):
    numRepetitions = max(0, numRepetitions - 1)
    shift = passIndex * 4
    gridFilteringFlags &= ~(3 << shift)
    gridFilteringFlags |= (((operation) << 2) | numRepetitions) << shift
    return gridFilteringFlags

def norm(a):
    square_sum = 0
    for item in a:
        square_sum += item * item
    
    return math.sqrt(square_sum) 



# https://math.stackexchange.com/questions/2346982/slerp-inverse-given-3-quaternions-find-t
def quarternion_slerp_inverse(q0, q1, q):
    q1_inv =  q1.GetInverse()
    q0_inv =  q0.GetInverse()
    q_inv =  q.GetInverse()

    tmp_1 = (q0_inv * q).GetNormalized()
    real = tmp_1.GetReal()
    img = [ tmp_1.GetImaginary()[0], tmp_1.GetImaginary()[1], tmp_1.GetImaginary()[2] ]

    # print("1: ", real)
    # print("term 1 cos: ", math.acos(real))
    term21 =   [ math.acos(real) / norm(img) * item for item in img]

    log_tmp1 =  [0, term21[0], term21[1], term21[2]]

    tmp_2 = (q0_inv * q1).GetNormalized()
    real = tmp_2.GetReal()
    img = [ tmp_2.GetImaginary()[0], tmp_2.GetImaginary()[1], tmp_2.GetImaginary()[2] ]

    # print("2: ", real)
    # print("term 2 cos: ", math.acos(real))
    term22 = [ math.acos(real) / norm(img) * item for item in img ]

    log_tmp2 =  [0, term22[0], term22[1], term22[2]]
    rates = []

    if abs(term21[0]) < 0.0001 and abs(term22[0]) < 0.0001:
        rates.append(None)
    else:
        t1 = (term21[0] / term22[0])
        rates.append(t1)

    if abs(term21[1]) < 0.0001 and abs(term22[1]) < 0.0001:
        rates.append(None)
    else:
        t2 = (term21[1] / term22[1])
        rates.append(t2)

    if abs(term21[2]) < 0.0001 and abs(term22[2]) < 0.0001:
        rates.append(None)
    else:
        t3 = (term21[2] / term22[2])
        rates.append(t3)
        
    # print("rates pre: ", rates)
    rates = list(filter(lambda x: x is not None, rates))
    # print("rates post: ", rates)
    
    # length = len(rates)
    # for i in range(length):
    #     for j in range(i+1, length):
    #         if not abs(rates[i] - rates[j]) <= 0.001:
    #             raise Exception("not the same")


    # print("rates: ", rates)
    return  max(rates)

# https://math.stackexchange.com/questions/167827/compute-angle-between-quaternions-in-matlab
def rotation_diff(q0, q1):
    z = q0.GetNormalized() * q1.GetNormalized().GetConjugate()
    z_real = abs(z.GetReal())

    if z_real > 1:
        z_real = 1
    elif z_real < -1:
        z_real = -1

    angle = math.acos(abs(z_real)) * 2

    return math.degrees(angle)


class Faucet():
    def __init__(self, particle_params = None, iso_surface_params = None, 
        liquid_material_path = "/World/Looks/OmniSurface_ClearWater", inflow_path:str = "/World/faucet/inflow", 
        link_paths:List[str] = ["/World/faucet/link_0"]
         ):
        """! Faucet class
         @param particle_params : parameters for particles
         @param iso_surface_params: parameters for iso_surface
         @param liquid_material_path: parameters for liquid materials
         @param inflow_path: used to compute the location of water drops
         @param link_paths: used to compute the rotation of faucet handle and determine the speed and size of water drops
         @param particle_params: parameters related to particle systems
         @return an instance of Faucet class
        """
        # particle Instance path
        self.particleInstanceStr_tmp = "/particlesInstance"

        self.particle_params = particle_params
        self.iso_surface_params = iso_surface_params
        self.liquid_material_path = liquid_material_path

        #Not sure if the isregistry thing works
        isregistry = carb.settings.acquire_settings_interface()
        self._async_simulation = carb.settings.get_settings().get_as_bool(ASYNC_SIMULATION)
        isregistry.set_bool(ASYNC_SIMULATION, True)
        isregistry.set_int("persistent/simulation/minFrameRate", 30)

        self.stage = omni.usd.get_context().get_stage()
        self.inflow_path = inflow_path
        self.link_paths = link_paths
        self.list_of_point_instancers = []
        self.active_indexes_for_point_instancers = []
        self.rate_checkers = []
        for link in link_paths:
            path = Path(link)
            self.rate_checkers.append(JointCheck( str(path.parent), str(path.name) ))



        self.create()

    def point_sphere(self, samples, scale):
        """! create locations for each particles
        @param samples: the number of particles per sphere
        @param scale: the scale(radius) of the water drop 
        """
        indices = [x + 0.5 for x in range(0, samples)]

        phi = [math.acos(1 - 2 * x / samples) for x in indices]
        theta = [math.pi * (1 + 5**0.5) * x for x in indices]

        x = [math.cos(th) * math.sin(ph) * scale for (th, ph) in zip(theta, phi)]
        y = [math.sin(th) * math.sin(ph) * scale for (th, ph) in zip(theta, phi)]
        z = [math.cos(ph) * scale for ph in phi]
        points = [Gf.Vec3f(x, y, z) for (x, y, z) in zip(x, y, z)]
        return points

    def create_ball(self, pos, rate = 1):
        """! create a water drop
        @param pos: the center of the water drop
        @param rate: the number of particles for each water drop
        """
        # create sphere on points
       
        points = self.point_sphere( 10+int(90 * rate), 1)

        # basePos = Gf.Vec3f(11.0, 12.0, 35.0) + pos
        basePos = pos
        positions = [Gf.Vec3f(x) + Gf.Vec3f(basePos) for x in points]

        radius = 0.2
        # particleSpacing = 2.0 * radius * 0.6
        particleSpacing = 2.0 * radius * 0.6

        positions_list = positions
        velocities_list = [Gf.Vec3f(0.0, 0.0, 0.0)] * len(positions)
        protoIndices_list = [0] * len(positions)

        protoIndices = Vt.IntArray(protoIndices_list)
        positions = Vt.Vec3fArray(positions_list)
        velocities = Vt.Vec3fArray(velocities_list)

        # particleInstanceStr = "/particlesInstance" + str(self.it)
        particleInstanceStr = omni.usd.get_stage_next_free_path(self.stage, self.particleInstanceStr_tmp, False)
        particleInstancePath = Sdf.Path(particleInstanceStr)

        # Create point instancer
        pointInstancer = UsdGeom.PointInstancer.Define(self.stage, particleInstancePath)
        prototypeRel = pointInstancer.GetPrototypesRel()

        # Create particle instance prototypes
        particlePrototype = PhysxParticleInstancePrototype()
        particlePrototype.selfCollision = True
        particlePrototype.fluid = True
        particlePrototype.collisionGroup = 0
        particlePrototype.mass = 0.5e-5

        prototypePath = particleInstancePath.pathString + "/particlePrototype"
        

        sphere = UsdGeom.Sphere.Define(self.stage, Sdf.Path(prototypePath))
        spherePrim = sphere.GetPrim()
        
        
        sphere.GetRadiusAttr().Set(particleSpacing)
        spherePrim = sphere.GetPrim()
        spherePrim.GetAttribute('visibility').Set('invisible')

        spherePrim.CreateAttribute("enableAnisotropy", Sdf.ValueTypeNames.Bool, True).Set(True)

        particleInstanceApi = PhysxSchema.PhysxParticleAPI.Apply(spherePrim)

        particleInstanceApi.CreateSelfCollisionAttr().Set(particlePrototype.selfCollision)
        particleInstanceApi.CreateFluidAttr().Set(particlePrototype.fluid)
        particleInstanceApi.CreateParticleGroupAttr().Set(particlePrototype.collisionGroup)
        particleInstanceApi.CreateMassAttr().Set(particlePrototype.mass)

        # Reference simulation owner using PhysxPhysicsAPI
        physicsApi = PhysxSchema.PhysxPhysicsAPI.Apply(spherePrim)
        physicsApi.CreateSimulationOwnerRel().SetTargets([self.particleSystemPath])

        # add prototype references to point instancer
        prototypeRel.AddTarget(Sdf.Path(prototypePath))

        # Set active particle indices
        activeIndices = []
        for i in range(len(positions)):
            activeIndices.append(protoIndices[i])

        orientations = [Gf.Quath(1.0, Gf.Vec3h(0.0, 0.0, 0.0))] * len(positions)

        angular_velocities = [Gf.Vec3f(0.0, 0.0, 0.0)] * len(positions)

        pointInstancer.GetProtoIndicesAttr().Set(activeIndices)
        pointInstancer.GetPositionsAttr().Set(positions)
        pointInstancer.GetOrientationsAttr().Set(orientations)
        pointInstancer.GetVelocitiesAttr().Set(velocities)
        pointInstancer.GetAngularVelocitiesAttr().Set(angular_velocities)

        self.list_of_point_instancers.append(pointInstancer)
        self.active_indexes_for_point_instancers.append(activeIndices)

    def create(self):
        """! initialize the related parameters for faucet
        create physics scenes
        create particle systems
        create isosurface
        """
        self._setup_callbacks()
        
        self.it = 0
        self.counter = 10

        # Physics scene
        scenePath = Sdf.Path("/physicsScene")
     
        # Particle System
        self.particleSystemPath = omni.usd.get_stage_next_free_path(self.stage, "/particleSystem", False)
        # particleSystemPath = Sdf.Path("/particleSystem0")
        self.particleSystemPath = self.particleSystemPath

        _fluidSphereDiameter = 0.24
        _solverPositionIterations = 10
        _solverVelocityIterations = 1
        _particleSystemSchemaParameters = {
            "contact_offset": 0.3,
            "particle_contact_offset": 0.25,
            "rest_offset": 0.25,
            "solid_rest_offset": 0,
            "fluid_rest_offset": 0.5 * _fluidSphereDiameter + 0.03,
            "solver_position_iterations": _solverPositionIterations,
            "solver_velocity_iterations": _solverVelocityIterations,
            "wind": Gf.Vec3f(0, 0, 0),
        }

        addPhysxParticleSystem(
            self.stage,
            self.particleSystemPath,
           **_particleSystemSchemaParameters,
            scenePath = scenePath
        )

        particleSystem = self.stage.GetPrimAtPath(self.particleSystemPath)
        # particle system settings
        if self.particle_params is not None:
            for key,value in self.particle_params.items():
                if isinstance(value, list):
                    particleSystem.CreateAttribute(key, value[0], value[1]).Set(value[2])
                else:
                    particleSystem.GetAttribute(key).Set(value)

        # apply isoSurface params        
        if self.iso_surface_params is not None:
            particleSystem.CreateAttribute("enableIsosurface", Sdf.ValueTypeNames.Bool, True).Set(True)
            for key,value in self.iso_surface_params.items():
                if isinstance(value, list):
                    particleSystem.CreateAttribute(key, value[0], value[1]).Set(value[2])
                else:
                    particleSystem.GetAttribute(key).Set(value)

        self.stage.SetInterpolationType(Usd.InterpolationTypeHeld)   


    def _setup_callbacks(self):
        """! callbacks registered with timeline and physics steps to drop water 
        """
        # callbacks
        self._timeline = omni.timeline.get_timeline_interface()
        stream = self._timeline.get_timeline_event_stream()
        self._timeline_subscription = stream.create_subscription_to_pop(self._on_timeline_event)
        # subscribe to Physics updates:
        self._physics_update_subscription = omni.physx.get_physx_interface().subscribe_physics_step_events(
            self.on_physics_step
        )

        # events = omni.physx.get_physx_interface().get_simulation_event_stream()
        # self._simulation_event_sub = events.create_subscription_to_pop(self._on_simulation_event)

    def _on_timeline_event(self, e):
        if e.type == int(omni.timeline.TimelineEventType.STOP):
            self.it = 0
            self._physics_update_subscription = None
            self._timeline_subscription = None

    def on_physics_step(self, dt):
        xformCache = UsdGeom.XformCache()

        # compute location to dispense water
        pose =  xformCache.GetLocalToWorldTransform(self.stage.GetPrimAtPath(self.inflow_path))
        pos_faucet = Gf.Vec3f(pose.ExtractTranslation())

        ##TODO hangle multiple faucet handles
        
        rate = self.rate_checkers[0].compute_distance()/100.0
        
        if rate > 1:
            rate = 1
      
        if self.it == 0:
            iso2Prim = self.stage.GetPrimAtPath(self.particleSystemPath+"/Isosurface")
            rel = iso2Prim.CreateRelationship("material:binding", False)
            # rel.SetTargets([Sdf.Path(self.liquid_material_path)])
            # rel.SetTargets([Sdf.Path("/World/Looks/OmniSurface_OrangeJuice")])

        #TODO we can have the water keep running, but we should delete some particles that are too old and not in containers.
        #this implementation will stop after 300 balls

        if self.it > 300:
            return
        
        if rate < 0.1:
            return

         # emit a ball based on rate
        if (self.counter < 20 - rate):
            self.counter = self.counter + 1
            return
     
        self.counter = 0
        self.it = self.it + 1


        self.create_ball( pos_faucet, rate)
    def __del__(self):
        self._physics_update_subscription = None
        self._timeline_subscription = None
        #TODO not sure if this works
        isregistry = carb.settings.acquire_settings_interface()
        isregistry.set_bool(ASYNC_SIMULATION, self._async_simulation)


# if __name__ == '__main__':
from omni.physx import  acquire_physx_interface
physx = acquire_physx_interface()
physx.overwrite_gpu_setting(1)
physx.reset_simulation()
particle_params = {
            "cohesion": 0.02,
            "smoothing": 0.8,
            "anisotropyScale": 1.0,
            "anisotropyMin": 0.2,
            "anisotropyMax": 2.0,
            "viscosity": 0.0091,
            "surfaceTension": 0.0074,
            "particleFriction": 0.1,
            "maxParticleNeighborhood": [ Sdf.ValueTypeNames.Int, True, 64],
            "maxParticles": 20000
        }

filterSmooth = 1
filtering = 0
passIndex = 0
filtering = setGridFilteringPass(filtering, passIndex, filterSmooth)
passIndex = passIndex + 1
filtering = setGridFilteringPass(filtering, passIndex, filterSmooth)
passIndex = passIndex + 1
iso_surface_params = {
            "maxIsosurfaceVertices": [Sdf.ValueTypeNames.Int, True,  1024 * 1024],
            "maxIsosurfaceTriangles": [Sdf.ValueTypeNames.Int, True, 2 * 1024 * 1024],
            "maxNumIsosurfaceSubgrids": [Sdf.ValueTypeNames.Int, True,  1024 * 4],
            "isosurfaceGridSpacing": [Sdf.ValueTypeNames.Float, True, 0.2],
            "isosurfaceKernelRadius": [Sdf.ValueTypeNames.Float, True,  0.5 ], 
            "isosurfaceLevel": [ Sdf.ValueTypeNames.Float, True, -0.3 ],
            "isosurfaceGridFilteringFlags": [Sdf.ValueTypeNames.Int, True, filtering ],
            "isosurfaceGridSmoothingRadiusRelativeToCellSize": [Sdf.ValueTypeNames.Float, True, 0.3 ],
            "isosurfaceEnableAnisotropy": [Sdf.ValueTypeNames.Bool, True, False ],
            "isosurfaceAnisotropyMin": [ Sdf.ValueTypeNames.Float, True, 0.1 ],
            "isosurfaceAnisotropyMax": [ Sdf.ValueTypeNames.Float, True, 2.0 ],
            "isosurfaceAnisotropyRadius": [ Sdf.ValueTypeNames.Float, True, 0.5 ],
            "numIsosurfaceMeshSmoothingPasses": [ Sdf.ValueTypeNames.Int,  True, 5 ],
            "numIsosurfaceMeshNormalSmoothingPasses": [ Sdf.ValueTypeNames.Int, True, 5 ],
            "isosurfaceDoNotCastShadows": [Sdf.ValueTypeNames.Bool, True, True ]
        }

# fluid_fill = Faucet(particle_params=particle_params, iso_surface_params=iso_surface_params,
#         liquid_material_path = "/World/Looks/OmniSurface_ClearWater", 
#         inflow_path  = "/World/faucet/inflow", 
#         link_paths = ["/World/faucet/link_1/joint_0"])
# fluid_fill = Faucet(particle_params=particle_params, iso_surface_params=iso_surface_params,
#          liquid_material_path = "/World/Looks/OmniSurface_ClearWater", 
#          inflow_path  = "/World/mobility/inflow", 
#          link_paths = ["/World/mobility/link_1/joint_0"])
