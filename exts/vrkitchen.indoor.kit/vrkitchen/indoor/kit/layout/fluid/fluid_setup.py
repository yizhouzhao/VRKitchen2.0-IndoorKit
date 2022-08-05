import carb
import math
from pxr import Usd, UsdGeom, Sdf, Gf, Vt, UsdPhysics, PhysxSchema
import omni.timeline

import omni.physxdemos as demo

from .schemaHelpers import PhysxParticleInstancePrototype, addPhysxParticleSystem

ASYNC_SIMULATION = "/persistent/physics/asyncSimRender"

def setGridFilteringPass(gridFilteringFlags: int, passIndex: int, operation: int, numRepetitions: int = 1):
    numRepetitions = max(0, numRepetitions - 1)
    shift = passIndex * 4
    gridFilteringFlags &= ~(3 << shift)
    gridFilteringFlags |= (((operation) << 2) | numRepetitions) << shift
    return gridFilteringFlags

class FluidFill(demo.Base):
    def __init__(self, pos = Gf.Vec3f(0 , 20, 0.0)):
        self.stage = omni.usd.get_context().get_stage()
        self.pos = pos
        xformCache = UsdGeom.XformCache()
        pose =  xformCache.GetLocalToWorldTransform(self.stage.GetPrimAtPath("/World/mobility/link_0"))
        pos_link = Gf.Vec3f(pose.ExtractTranslation())
        self.rot_link_init = Gf.Quatf(pose.ExtractRotationQuat())

        # print("attributes: ", self.stage.GetPrimAtPath("/World/faucet/link_0").GetAttributes())
        
        self.init_orient = self.stage.GetPrimAtPath("/World/mobility/link_0").GetAttribute("xformOp:orient").Get()
        
    def point_sphere(self, samples, scale):
        indices = [x + 0.5 for x in range(0, samples)]

        phi = [math.acos(1 - 2 * x / samples) for x in indices]
        theta = [math.pi * (1 + 5**0.5) * x for x in indices]

        x = [math.cos(th) * math.sin(ph) * scale for (th, ph) in zip(theta, phi)]
        y = [math.sin(th) * math.sin(ph) * scale for (th, ph) in zip(theta, phi)]
        z = [math.cos(ph) * scale for ph in phi]
        points = [Gf.Vec3f(x, y, z) for (x, y, z) in zip(x, y, z)]
        return points

    def create_ball(self, stage, pos, rate = 1):

        # create sphere on points
        # print("scale: ", rate)
        points = self.point_sphere( 10+int(90 * rate), 1)
        # points = self.point_sphere( int(80 * rate), 1)

        # basePos = Gf.Vec3f(11.0, 12.0, 35.0) + pos
        basePos = pos
        positions = [Gf.Vec3f(x) + Gf.Vec3f(basePos) for x in points]

        radius = 0.1
        # particleSpacing = 2.0 * radius * 0.6
        particleSpacing = 2.0 * radius * 0.6

        positions_list = positions
        velocities_list = [Gf.Vec3f(0.0, 0.0, 0.0)] * len(positions)
        protoIndices_list = [0] * len(positions)

        protoIndices = Vt.IntArray(protoIndices_list)
        positions = Vt.Vec3fArray(positions_list)
        velocities = Vt.Vec3fArray(velocities_list)

        particleInstanceStr = "/particlesInstance" + str(self.it)
        particleInstancePath = Sdf.Path(particleInstanceStr)

        # Create point instancer
        pointInstancer = UsdGeom.PointInstancer.Define(stage, particleInstancePath)
        prototypeRel = pointInstancer.GetPrototypesRel()

        # Create particle instance prototypes
        particlePrototype = PhysxParticleInstancePrototype()
        particlePrototype.selfCollision = True
        particlePrototype.fluid = True
        particlePrototype.collisionGroup = 0
        particlePrototype.mass = 0.001

        prototypePath = particleInstancePath.pathString + "/particlePrototype"
        
        sphere = UsdGeom.Sphere.Define(stage, Sdf.Path(prototypePath))
        spherePrim = sphere.GetPrim()
        
        
        sphere.GetRadiusAttr().Set(particleSpacing)
        # color_rgb = [0.0, 0.08, 0.30]
        
        # color = Vt.Vec3fArray([Gf.Vec3f(color_rgb[0], color_rgb[1], color_rgb[2])])
        # sphere.CreateDisplayColorAttr(color)
        spherePrim = sphere.GetPrim()
        spherePrim.GetAttribute('visibility').Set('invisible')
        # spherePrim.GetVisibilityAttr().Set(False)

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

    def create(self, stage):
        
        self._setup_callbacks()
        self.stage = stage

        self.it = 0
        self.counter = 10

        # set up axis to z
        UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.y)
        UsdGeom.SetStageMetersPerUnit(stage, 0.01)

        # light
        # sphereLight = UsdLux.SphereLight.Define(stage, Sdf.Path("/SphereLight"))
        # sphereLight.CreateRadiusAttr(150)
        # sphereLight.CreateIntensityAttr(30000)
        # sphereLight.AddTranslateOp().Set(Gf.Vec3f(650.0, 0.0, 1150.0))

        # Physics scene
        scenePath = Sdf.Path("/physicsScene")
        scene = UsdPhysics.Scene.Define(stage, scenePath)
        scene.CreateGravityDirectionAttr().Set(Gf.Vec3f(0.0, -1.0, 0.0))
        scene.CreateGravityMagnitudeAttr().Set(9.81)

        # Particle System
        particleSystemPath = Sdf.Path("/particleSystem0")
        self.particleSystemPath = particleSystemPath

        particleSpacing = 0.2
        restOffset = particleSpacing * 0.9
        solidRestOffset = restOffset
        fluidRestOffset = restOffset * 0.6
        particleContactOffset = max(solidRestOffset + 0.001, fluidRestOffset / 0.6)
        contactOffset = restOffset + 0.001

        addPhysxParticleSystem(
            stage,
            particleSystemPath,
            contactOffset,
            restOffset,
            particleContactOffset,
            solidRestOffset,
            fluidRestOffset,
            4,
            1,
            Gf.Vec3f(0, 0, 0),
            scenePath
        )

        particleSystem = stage.GetPrimAtPath(particleSystemPath)
        # particle system settings
        particleSystem.GetAttribute("cohesion").Set(0.002)
        particleSystem.GetAttribute("smoothing").Set(0.8)
        particleSystem.GetAttribute("anisotropyScale").Set(1.0)
        particleSystem.GetAttribute("anisotropyMin").Set(0.2)
        particleSystem.GetAttribute("anisotropyMax").Set(2.0)
        particleSystem.GetAttribute("viscosity").Set(0.0091)
        particleSystem.GetAttribute("surfaceTension").Set(0.0074)
        particleSystem.GetAttribute("particleFriction").Set(0.1)
        particleSystem.CreateAttribute("maxParticleNeighborhood", Sdf.ValueTypeNames.Int, True).Set(64)
        particleSystem.GetAttribute("maxParticles").Set(20000)

        # apply isoSurface params
        particleSystem.CreateAttribute("enableIsosurface", Sdf.ValueTypeNames.Bool, True).Set(True)
        particleSystem.CreateAttribute("maxIsosurfaceVertices", Sdf.ValueTypeNames.Int, True).Set(1024 * 1024)
        particleSystem.CreateAttribute("maxIsosurfaceTriangles", Sdf.ValueTypeNames.Int, True).Set(2 * 1024 * 1024)
        particleSystem.CreateAttribute("maxNumIsosurfaceSubgrids", Sdf.ValueTypeNames.Int, True).Set(1024 * 4)
        particleSystem.CreateAttribute("isosurfaceGridSpacing", Sdf.ValueTypeNames.Float, True).Set(0.2)

        filterSmooth = 1

        filtering = 0
        passIndex = 0
        filtering = setGridFilteringPass(filtering, passIndex, filterSmooth)
        passIndex = passIndex + 1
        filtering = setGridFilteringPass(filtering, passIndex, filterSmooth)
        passIndex = passIndex + 1

        particleSystem.CreateAttribute("isosurfaceKernelRadius", Sdf.ValueTypeNames.Float, True).Set(0.5)
        particleSystem.CreateAttribute("isosurfaceLevel", Sdf.ValueTypeNames.Float, True).Set(-0.3)
        particleSystem.CreateAttribute("isosurfaceGridFilteringFlags", Sdf.ValueTypeNames.Int, True).Set(filtering)
        particleSystem.CreateAttribute(
            "isosurfaceGridSmoothingRadiusRelativeToCellSize", Sdf.ValueTypeNames.Float, True
        ).Set(0.3)

        particleSystem.CreateAttribute("isosurfaceEnableAnisotropy", Sdf.ValueTypeNames.Bool, True).Set(False)
        particleSystem.CreateAttribute("isosurfaceAnisotropyMin", Sdf.ValueTypeNames.Float, True).Set(0.1)
        particleSystem.CreateAttribute("isosurfaceAnisotropyMax", Sdf.ValueTypeNames.Float, True).Set(2.0)
        particleSystem.CreateAttribute("isosurfaceAnisotropyRadius", Sdf.ValueTypeNames.Float, True).Set(0.5)

        particleSystem.CreateAttribute("numIsosurfaceMeshSmoothingPasses", Sdf.ValueTypeNames.Int, True).Set(5)
        particleSystem.CreateAttribute("numIsosurfaceMeshNormalSmoothingPasses", Sdf.ValueTypeNames.Int, True).Set(5)

        particleSystem.CreateAttribute("isosurfaceDoNotCastShadows", Sdf.ValueTypeNames.Bool, True).Set(True)

        stage.SetInterpolationType(Usd.InterpolationTypeHeld)

        

    def _setup_callbacks(self):
        # callbacks
        self._timeline = omni.timeline.get_timeline_interface()
        stream = self._timeline.get_timeline_event_stream()
        self._timeline_subscription = stream.create_subscription_to_pop(self._on_timeline_event)
        # subscribe to Physics updates:
        self._physics_update_subscription = omni.physx.get_physx_interface().subscribe_physics_step_events(
            self.on_physics_step
        )

    def _on_timeline_event(self, e):
        if e.type == int(omni.timeline.TimelineEventType.STOP):
            self.it = 0
            self.on_shutdown()
    
    def step(self):
        self.on_physics_step(None)

    def on_physics_step(self, dt):
        # import transforms3d
        import math
        xformCache = UsdGeom.XformCache()
        
        # stop after 80 balls
        # if (self.it > 80):
        #     return

        pose =  xformCache.GetLocalToWorldTransform(self.stage.GetPrimAtPath("/World/faucet/inflow"))
        pos_faucet = Gf.Vec3f(pose.ExtractTranslation())
        rot_faucet = Gf.Quatf(pose.ExtractRotationQuat())

        pose =  xformCache.GetLocalToWorldTransform(self.stage.GetPrimAtPath("/World/faucet/link_0"))
        pos_link = Gf.Vec3f(pose.ExtractTranslation())
        rot_link = Gf.Quatf(pose.ExtractRotationQuat()) 
        
        
        diff = rot_link * self.rot_link_init.GetInverse()
        real = diff.GetReal()
        img = [diff.GetImaginary()[0],diff.GetImaginary()[1], diff.GetImaginary()[2] ]
        
        #angle = transforms3d.euler.quat2euler([real, img[0], img[1], img[2]], axes='sxyz')
        #sum_angle = abs(math.degrees(angle[0])) + abs(math.degrees(angle[1])) + abs(math.degrees(angle[2]))

        rate = 1 #(sum_angle/30.0)
        # print("pre rate:", rate)
        if rate > 1:
            rate = 1
        # print("rate: ", rate)
        # print("sum_angle", sum_angle)
        
        
        if self.it == 0:
            iso2Prim = self.stage.GetPrimAtPath("/particleSystem0/Isosurface")
            rel = iso2Prim.CreateRelationship("material:binding", False)
            rel.SetTargets([Sdf.Path("/World/Looks/OmniSurface_ClearWater")])
            # rel.SetTargets([Sdf.Path("/World/Looks/OmniSurface_OrangeJuice")])

        if self.it > 200:
            return

        # emit a ball every 10 physics steps
        if (self.counter < 20 - rate):
            self.counter = self.counter + 1
            return
     
        self.counter = 0
        self.it = self.it + 1
       
        # print(faucet_prim.GetAttribute('xformOp:translate'))
       
        # openness = 0.6 + 0.5 * rate
        
        # print("openess", openness)
        if rate < 0.1:
            return
        self.create_ball(self.stage, pos_faucet, rate)

    def on_shutdown(self):
        self._physics_update_subscription = None
        self._timeline_subscription = None
        # restore settings
        # isregistry = carb.settings.acquire_settings_interface()
        # isregistry.set_bool(ASYNC_SIMULATION, self._async_simulation)

    def on_startup(self):
        isregistry = carb.settings.acquire_settings_interface()
        self._async_simulation = carb.settings.get_settings().get_as_bool(ASYNC_SIMULATION)
        isregistry.set_bool(ASYNC_SIMULATION, True)
        isregistry.set_int("persistent/simulation/minFrameRate", 60)

from omni.physx import  acquire_physx_interface
physx = acquire_physx_interface()
physx.overwrite_gpu_setting(1)
physx.reset_simulation()

fluid_fill = FluidFill()
stage = omni.usd.get_context().get_stage()
fluid_fill.create(stage)
_timeline = omni.timeline.get_timeline_interface()
stream = _timeline.get_timeline_event_stream()


def _on_timeline_event(e):
    if e.type == int(omni.timeline.TimelineEventType.STOP):
        fluid_fill.on_shutdown()

_timeline_subscription = stream.create_subscription_to_pop(_on_timeline_event)
# for i in range(10):
#     fluid_fill.step()



