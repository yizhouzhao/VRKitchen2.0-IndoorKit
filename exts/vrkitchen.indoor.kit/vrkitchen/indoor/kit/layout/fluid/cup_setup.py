import math
import os
from ..param import ROOT as root
from ...param import IS_IN_ISAAC_SIM, APP_VERION
import carb
import omni
import pxr
from pxr import Gf, UsdPhysics, Sdf, Usd, UsdGeom, PhysxSchema
from omni.physx.scripts import utils, physicsUtils

if APP_VERION.startswith("2022"):
    from omni.physx.scripts import particleUtils

import numpy as np
from .constants import PARTICLE_PROPERTY
# from omni.isaac.core.utils.stage import add_reference_to_stage

from .schemaHelpers import addPhysxParticleSystem, addPhysxParticlesSimple, PhysxParticleInstancePrototype
from .utils import generate_cylinder_y, generate_inside_point_cloud, get_quat_from_extrinsic_xyz_rotation
from .cup_data import CUP_PARTICLE_INFO



def setGridFilteringPass(gridFilteringFlags: int, passIndex: int, operation: int, numRepetitions: int = 1):
    numRepetitions = max(0, numRepetitions - 1)
    shift = passIndex * 4
    gridFilteringFlags &= ~(3 << shift)
    gridFilteringFlags |= (((operation) << 2) | numRepetitions) << shift
    return gridFilteringFlags

class CupFluidHelper():
    def __init__(self, cup_id = 0, r = 0.1, g  = 0.4,b =0.6, material = None, height = None) -> None:
        self.stage = omni.usd.get_context().get_stage()

        self.cup_id = cup_id
        self.rgb =[r,g,b]
        self.material = material
        self.height = height
    
    def create(self):
        # needs to be called first: set_up_fluid_physical_scene
        self.set_up_fluid_physical_scene()
        self.set_cup()

        self.set_up_particle_system()
        self.set_color()
        self.set_particle_offset() 

    def modify_cup_scene(self, cup_prim, add_liquid = True, set_physics=True):
        """
        Modify cup scene given the cup_prim,
        1. setup physical scene and fluid scene
        2. add particles
        :param::
            : cup_prim
        """        
        print("modify cup at path: ", cup_prim.GetPath().pathString)
        game_prim = cup_prim.GetParent()

        # set up physical
        self.set_up_fluid_physical_scene()

        carb.log_warn("APP_VERION 1: " + APP_VERION)

        # modify particleSystemStr
        if add_liquid:
            particleSystemStr = game_prim.GetPath().AppendPath("Fluid").pathString
            self.particleSystemPath = pxr.Sdf.Path(particleSystemStr)
            self.particleInstanceStr = game_prim.GetPath().AppendPath("Particles").pathString

        # modify cup
        cup_shape_prim_path = cup_prim.GetPath().AppendPath("cupShape").pathString
        cup_shape_prim = self.stage.GetPrimAtPath(cup_shape_prim_path)
        cup_volume_prim_path = cup_prim.GetPath().AppendPath("cup_volume").pathString
        cup_volume_prim = self.stage.GetPrimAtPath(cup_volume_prim_path)
        if not cup_shape_prim:
            raise Exception(f"Cup shape must exist at path {cup_shape_prim_path}")

        if IS_IN_ISAAC_SIM :
            from omni.isaac.core.utils.semantics import add_update_semantics
            add_update_semantics(cup_shape_prim, "Cup")
        
        # utils.setPhysics(prim=cup_shape_prim, kinematic=False)    
        # utils.setCollider(prim=cup_shape_prim, approximationShape="convexDecomposition")

        # if not set_physics:
        #     physicsAPI = UsdPhysics.RigidBodyAPI.Apply(cup_shape_prim)
        #     physicsAPI.CreateRigidBodyEnabledAttr(False)

        physxCollisionAPI = pxr.PhysxSchema.PhysxCollisionAPI.Get(self.stage, cup_shape_prim.GetPath())
        if not physxCollisionAPI:
            physxCollisionAPI = pxr.PhysxSchema.PhysxCollisionAPI.Apply(cup_shape_prim)
        self._setup_physics_material(cup_shape_prim.GetPath())

         # Mug parameters
        restOffset = PARTICLE_PROPERTY._cup_rest_offset
        contactOffset = PARTICLE_PROPERTY._cup_contact_offset

        assert physxCollisionAPI.GetRestOffsetAttr().Set(restOffset)
        assert physxCollisionAPI.GetContactOffsetAttr().Set(contactOffset)
        assert cup_shape_prim.CreateAttribute("physxMeshCollision:minThickness", pxr.Sdf.ValueTypeNames.Float).Set(0.001)


        self._fluidPositionOffset = Gf.Vec3f(0,0,0)
     
  
        massAPI = UsdPhysics.MassAPI.Apply(cup_shape_prim)
        massAPI.GetMassAttr().Set(PARTICLE_PROPERTY._cup_mass)
        
        # utils.setPhysics(prim=cup_prim, kinematic=False)
        utils.removeRigidBody(cup_shape_prim)
        utils.setRigidBody(cup_prim, "convexDecomposition", False)
        utils.removeCollider(cup_volume_prim)



        if add_liquid:
            self.volume_mesh = pxr.UsdGeom.Mesh.Get(self.stage, cup_prim.GetPath().AppendPath(f"cup_volume"))    

            self.set_up_particle_system()

            carb.log_warn("APP_VERION 1: " + APP_VERION)

            self.set_color()

            from omni.physx import  acquire_physx_interface
            physx = acquire_physx_interface()
            physx.overwrite_gpu_setting(1)
            physx.reset_simulation()

    def set_up_fluid_physical_scene(self, gravityMagnitude = PARTICLE_PROPERTY._gravityMagnitude):
        """
        Fluid / PhysicsScene
        """
        

        default_prim_path = self.stage.GetDefaultPrim().GetPath()
        if default_prim_path.pathString == '':
            # default_prim_path = pxr.Sdf.Path('/World')
            root = UsdGeom.Xform.Define(self.stage, "/World").GetPrim()
            self.stage.SetDefaultPrim(root)
            default_prim_path = self.stage.GetDefaultPrim().GetPath()

        # if self.stage.GetPrimAtPath("/World/physicsScene"):
        #     self.physicsScenePath = default_prim_path.AppendChild("physicsScene")
        #     return

        particleSystemStr = default_prim_path.AppendPath("Fluid").pathString
        self.physicsScenePath = default_prim_path.AppendChild("physicsScene")
        self.particleSystemPath = pxr.Sdf.Path(particleSystemStr)
        self.particleInstanceStr = default_prim_path.AppendPath("Particles").pathString
        

        # Physics scene
        self._gravityMagnitude = gravityMagnitude  # IN CM/s2 - use a lower gravity to avoid fluid compression at 60 FPS
        self._gravityDirection = Gf.Vec3f(0.0, -1.0, 0.0)
        physicsScenePath = default_prim_path.AppendChild("physicsScene")
        if self.stage.GetPrimAtPath("/World/physicsScene"):
            scene = UsdPhysics.Scene.Get(self.stage, physicsScenePath)
        else:
            scene = UsdPhysics.Scene.Define(self.stage, physicsScenePath)
        scene.CreateGravityDirectionAttr().Set(self._gravityDirection)
        scene.CreateGravityMagnitudeAttr().Set(self._gravityMagnitude)
        physxSceneAPI = PhysxSchema.PhysxSceneAPI.Apply(scene.GetPrim())
        physxSceneAPI.CreateEnableCCDAttr().Set(True)
        physxSceneAPI.GetTimeStepsPerSecondAttr().Set(120)
        physxSceneAPI.CreateEnableGPUDynamicsAttr().Set(True)
        physxSceneAPI.CreateEnableEnhancedDeterminismAttr().Set(True)

        

    def set_up_particle_system(self):
        self._fluidSphereDiameter = PARTICLE_PROPERTY._fluidSphereDiameter
        
        self._particleSystemSchemaParameters = PARTICLE_PROPERTY._particleSystemSchemaParameters

        self._particleSystemAttributes = PARTICLE_PROPERTY._particleSystemAttributes
        
        if APP_VERION.startswith("2022"):
            self._particleSystem = particleUtils.add_physx_particle_system(
                self.stage, self.particleSystemPath, **self._particleSystemSchemaParameters, simulation_owner=Sdf.Path(self.physicsScenePath.pathString)
            )

            # materialPathStr = "/World/Looks/OmniGlass"

            # particleUtils.add_pbd_particle_material(self.stage, materialPathStr, **PARTICLE_PROPERTY._particleMaterialAttributes)
            # physicsUtils.add_physics_material_to_prim(self.stage, self._particleSystem.GetPrim(), materialPathStr)

        else:
            addPhysxParticleSystem(
                self.stage, self.particleSystemPath, **self._particleSystemSchemaParameters, \
                    scenePath=pxr.Sdf.Path(self.physicsScenePath.pathString)
            )

        particleSystem = self.stage.GetPrimAtPath(self.particleSystemPath)

        if APP_VERION.startswith("2022"):
            pass
        else:
            for key, value in self._particleSystemAttributes.items():
                particleSystem.GetAttribute(key).Set(value)

        particleInstancePath = pxr.Sdf.Path(self.particleInstanceStr)
        proto = PhysxParticleInstancePrototype()
        proto.selfCollision = True
        proto.fluid = True
        proto.collisionGroup = 0
        proto.mass = PARTICLE_PROPERTY._particle_mass
        protoArray = [proto]

        positions_list = []
        velocities_list = []
        protoIndices_list = []

        lowerCenter = pxr.Gf.Vec3f(0, 0, 0)

        particle_rest_offset = self._particleSystemSchemaParameters["fluid_rest_offset"]
        
            ####################################
        if not hasattr(self, "volume_mesh") or self.volume_mesh is None: # not "volume_container" in CUP_PARTICLE_INFO[self.cup_id]:
              ################DATA####################
            if self.height is None:
                cylinder_height = CUP_PARTICLE_INFO[self.cup_id]["cylinder_height"]
            else:
                cylinder_height = self.height
            cylinder_radius = CUP_PARTICLE_INFO[self.cup_id]["cylinder_radius"]
    
            positions_list = generate_cylinder_y(lowerCenter, h=cylinder_height, radius=cylinder_radius, sphereDiameter=particle_rest_offset * 2.0)
        # positions_list  = generate_inside_mesh(lowerCenter, h=cylinder_height, radius=cylinder_radius, 
        # sphereDiameter=particle_rest_offset * 2.0, mesh= self.mesh, scale=self.scale)
        else:
            self.cloud_points = np.array(self.volume_mesh.GetPointsAttr().Get())
            # two crowded, add 0.08
            positions_list  = generate_inside_point_cloud(sphereDiameter=particle_rest_offset * (2.0 + 0.08), cloud_points = self.cloud_points, scale=1.0)

        for _ in range(len(positions_list)):
            velocities_list.append(pxr.Gf.Vec3f(0, 0, 0))
            protoIndices_list.append(0)

        # print("positions_list", len(positions_list))
        
        # positions_list -= np.array([228, 0, -231])
        # positions_list = positions_list.tolist()

        self.positions_list = positions_list
        protoIndices = pxr.Vt.IntArray(protoIndices_list)
        positions = pxr.Vt.Vec3fArray(positions_list)
        velocities = pxr.Vt.Vec3fArray(velocities_list)

        if APP_VERION.startswith("2022"):
            particleUtils.add_physx_particleset_pointinstancer(
                self.stage,
                particleInstancePath,
                positions,
                velocities,
                self.particleSystemPath,
                self_collision=True,
                fluid=True,
                particle_group=0,
                particle_mass=PARTICLE_PROPERTY._particle_mass,
                density=0.0,
            )

            # mtl_created = []
            # omni.kit.commands.execute(
            #     "CreateAndBindMdlMaterialFromLibrary",
            #     mdl_name="OmniSurfacePresets.mdl",
            #     mtl_name="OmniSurface_ClearWater",
            #     mtl_created_list=mtl_created,
            # )
            # pbd_particle_material_path = mtl_created[0]
            # omni.kit.commands.execute(
            #     "BindMaterial", prim_path=self.particleSystemPath, material_path=pbd_particle_material_path
            # )

            # # Create a pbd particle material and set it on the particle system
            # particleUtils.add_pbd_particle_material(
            #     self.stage,
            #     pbd_particle_material_path,
            #     cohesion=0.01,
            #     viscosity=0.0091,
            #     surface_tension=0.0074,
            #     friction=0.1,
            # )
        else:
            addPhysxParticlesSimple(
                self.stage, particleInstancePath, protoArray, protoIndices, positions, velocities, self.particleSystemPath
            )
        
        # isosurfaceAPI = PhysxSchema.PhysxParticleIsosurfaceAPI.Apply(self._particleSystem.GetPrim())
        # isosurfaceAPI.CreateIsosurfaceEnabledAttr().Set(True)
        # isosurfaceAPI.CreateMaxVerticesAttr().Set(1024 * 1024)
        # isosurfaceAPI.CreateMaxTrianglesAttr().Set(2 * 1024 * 1024)
        # isosurfaceAPI.CreateMaxSubgridsAttr().Set(1024 * 4)
        # isosurfaceAPI.CreateGridSpacingAttr().Set(particle_rest_offset * 1.5)
        # isosurfaceAPI.CreateSurfaceDistanceAttr().Set(particle_rest_offset * 1.6)
        # isosurfaceAPI.CreateGridFilteringPassesAttr().Set("")
        # isosurfaceAPI.CreateGridSmoothingRadiusAttr().Set(particle_rest_offset * 2)

        return
        # FIXME:DEBUG
        # make sure scales and orientations are authored
        pprim = self.stage.GetPrimAtPath(self.particleInstanceStr)
        pprim.GetAttribute("scales").Set([PARTICLE_PROPERTY._particle_scale for i in range(len(positions_list))])
        pprim.GetAttribute("orientations").Set([Gf.Quath(1.0, 0.0, 0.0, 0.0) for i in range(len(positions_list))])

         # Enable Flow rendering, plus shadows, reflections, and translucency
        flowRenderSettings = {
            "rtx:flow:enabled": True,
            "rtx:flow:rayTracedTranslucencyEnabled": True,
            "rtx:flow:rayTracedReflectionsEnabled": True,
            "rtx:flow:rayTracedShadowsEnabled": True,
            "rtx:flow:compositeEnabled": False,  # use translucency instead
            "rtx:flow:useFlowLibraryComposite": False,  # we want to use native rendering
            "rtx:flow:useFlowLibrarySelfShadow": False,  # not implemented for isosurfaces yet
        }

        metadata = self.stage.GetMetadata("customLayerData")
        if "renderSettings" in metadata:
            renderSettings = metadata["renderSettings"]
        else:
            renderSettings = {}
        metadata["renderSettings"] = {**renderSettings, **flowRenderSettings}
        self.stage.SetMetadata("customLayerData", metadata)

        def create_isoSurface():
            # isosurface render params
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
            radius = 0.2
            particleSystem.CreateAttribute("enableIsosurface", Sdf.ValueTypeNames.Bool, True).Set(True)
            isopath = self.particleInstanceStr + "/flowIsosurface"
            isoprim = self.stage.DefinePrim(isopath, "FlowIsosurface")
            isoprim.CreateAttribute("densityCellSize", Sdf.ValueTypeNames.Float, False).Set(radius / 2.0)
            isoprim.CreateAttribute("layer", Sdf.ValueTypeNames.Int, False).Set(0)
            isoprim.CreateAttribute("positionValues", Sdf.ValueTypeNames.Float4Array, False).Set([])
            
            elprim = self.stage.DefinePrim(isopath + "/ellipsoidRaster", "FlowEllipsoidRasterParams")
            elprim.CreateAttribute("allocationScale", Sdf.ValueTypeNames.Float, False).Set(2.5)
            elprim.CreateAttribute("scale", Sdf.ValueTypeNames.Float, False).Set(1)
            elprim.CreateAttribute("smoothIterations", Sdf.ValueTypeNames.UInt, False).Set(3)

            for key,value in iso_surface_params.items():
                    if isinstance(value, list):
                        particleSystem.CreateAttribute(key, value[0], value[1]).Set(value[2])
                    else:
                        particleSystem.GetAttribute(key).Set(value)

            self.stage.SetInterpolationType(Usd.InterpolationTypeHeld)  
            pt, aE1t, aE2t, aE3t = [], [], [], []
            for v in self.positions_list:
                pt.append((v[0], v[1], v[2], 0.0))
                aE1t.append((1.0, 0.0, 0.0, 0.5))
                aE2t.append((0.0, 1.0, 0.0, 0.5))
                aE3t.append((0.0, 0.0, 1.0, 0.5))

            elprim.CreateAttribute("positions", Sdf.ValueTypeNames.Float4Array, False).Set(pt)
            elprim.CreateAttribute("anisotropyE1s", Sdf.ValueTypeNames.Float4Array, False).Set(aE1t)
            elprim.CreateAttribute("anisotropyE2s", Sdf.ValueTypeNames.Float4Array, False).Set(aE2t)
            elprim.CreateAttribute("anisotropyE3s", Sdf.ValueTypeNames.Float4Array, False).Set(aE3t)

            marchprim = self.stage.DefinePrim(isopath + "/rayMarchIsosurface", "FlowRayMarchIsosurfaceParams")
            marchprim.CreateAttribute("densityThreshold", Sdf.ValueTypeNames.Float, False).Set(0.1)
            marchprim.CreateAttribute("stepSizeScale", Sdf.ValueTypeNames.Float, False).Set(0.75)
            marchprim.CreateAttribute("visualizeNormals", Sdf.ValueTypeNames.Bool, False).Set(False)
            marchprim.CreateAttribute("fluidColor", Sdf.ValueTypeNames.Float3, False).Set(Gf.Vec3f(1, 0, 0))
            marchprim.CreateAttribute("fluidDiffuseReflectance", Sdf.ValueTypeNames.Float3, False).Set(Gf.Vec3f(0.6, 0, 0))

        print("creating Isosurface")
        from ...param import USE_ISO_SURFACE
        if USE_ISO_SURFACE:
            print("creating Isosurface")
            create_isoSurface()

    def set_color(self):
        # Set color
        color_rgb = self.rgb#[0.1, 0.4, 0.6]
        color = pxr.Vt.Vec3fArray([pxr.Gf.Vec3f(color_rgb[0], color_rgb[1], color_rgb[2])])
        colorPathStr = self.particleInstanceStr + "/particlePrototype0"
        gprim = pxr.UsdGeom.Sphere.Define(self.stage, pxr.Sdf.Path(colorPathStr))
        gprim.CreateDisplayColorAttr(color)
        # prototypePathStr = particleInstanceStr + "/particlePrototype0"
        # gprim = UsdGeom.Sphere.Define(stage, Sdf.Path(prototypePathStr))
        # gprim.CreateVisibilityAttr("invisible")
        
        # TODO: debug transperency 
        gprim.CreateDisplayOpacityAttr([float(0.1)])
        from ...param import USE_ISO_SURFACE
        if USE_ISO_SURFACE:
            gprim.GetPrim().GetAttribute('visibility').Set('invisible')

        # usdPrim = stage.GetPrimAtPath(particleInstancePath)
        usdPrim = self.stage.GetPrimAtPath(colorPathStr)
        usdPrim.CreateAttribute("enableAnisotropy", pxr.Sdf.ValueTypeNames.Bool, True).Set(True)
        usdPrim.CreateAttribute("radius", pxr.Sdf.ValueTypeNames.Double, True).Set(0.3)

        gprim.GetRadiusAttr().Set(self._fluidSphereDiameter)


       
    def set_cup(self):
        # get cup info from data
        abspath = CUP_PARTICLE_INFO[self.cup_id]["usd_path"]
        mesh_name = CUP_PARTICLE_INFO[self.cup_id]["mesh_name"]
        scale = CUP_PARTICLE_INFO[self.cup_id]["scale"]
        particle_offset = CUP_PARTICLE_INFO[self.cup_id]["particle_offset"]
        cup_offset = CUP_PARTICLE_INFO[self.cup_id]["cup_offset"]

        self.scale = scale

        default_prim_path = self.stage.GetDefaultPrim().GetPath()
        self.stage.DefinePrim(default_prim_path.AppendPath(f"Cup")).GetReferences().AddReference(abspath)
        mug = pxr.UsdGeom.Mesh.Get(self.stage, default_prim_path.AppendPath(f"Cup/{mesh_name}"))
        utils.setPhysics(prim=mug.GetPrim(), kinematic=False)
        utils.setCollider(prim=mug.GetPrim(), approximationShape="convexDecomposition")
        
        if "volume_container" in CUP_PARTICLE_INFO[self.cup_id]:
            volume_container = CUP_PARTICLE_INFO[self.cup_id]["volume_container"]
            self.volume_mesh = pxr.UsdGeom.Mesh.Get(self.stage, default_prim_path.AppendPath(f"Cup/{volume_container}"))
        
        prim = mug.GetPrim()
        self.mug = mug
        # self._setup_rb_collision_parameters(mug.GetPrim(), restOffset=self._mugRestOffset, contactOffset=self._mugContactOffset)

        physxCollisionAPI = pxr.PhysxSchema.PhysxCollisionAPI.Get(self.stage, prim.GetPath())
        if not physxCollisionAPI:
            physxCollisionAPI = pxr.PhysxSchema.PhysxCollisionAPI.Apply(prim)
        self._setup_physics_material(prim.GetPath())

        # Mug parameters
        restOffset = 0.0
        contactOffset = 1.0

        assert physxCollisionAPI.GetRestOffsetAttr().Set(restOffset)
        assert physxCollisionAPI.GetContactOffsetAttr().Set(contactOffset)
        assert prim.CreateAttribute("physxMeshCollision:minThickness", pxr.Sdf.ValueTypeNames.Float).Set(0.001)

        # assert (
        #     mug.GetPrim().CreateAttribute("physxMeshCollision:maxConvexHulls", Sdf.ValueTypeNames.Float).Set(32)
        # )

        self._mugInitPos = Gf.Vec3f(cup_offset[0], cup_offset[1], cup_offset[2]) * scale

        self._mugInitRot = get_quat_from_extrinsic_xyz_rotation(angleYrad=-0.7 * math.pi)

        self._fluidPositionOffset = Gf.Vec3f(particle_offset[0], particle_offset[1], particle_offset[2])
     
        self._mugScale = Gf.Vec3f(scale)
        self._mugOffset = Gf.Vec3f(0, 0, 0) * scale

        self.transform_mesh(mug, self._mugInitPos + self._mugOffset * 0, self._mugInitRot, self._mugScale)
        massAPI = UsdPhysics.MassAPI.Apply(prim)
        massAPI.GetMassAttr().Set(PARTICLE_PROPERTY._cup_mass)

        
    def transform_mesh(self, mesh, loc, orient=pxr.Gf.Quatf(1.0), scale=pxr.Gf.Vec3d(1.0, 1.0, 1.0)):
        for op in mesh.GetOrderedXformOps():
            if op.GetOpType() == pxr.UsdGeom.XformOp.TypeTranslate:
                op.Set(loc)
            if op.GetOpType() == pxr.UsdGeom.XformOp.TypeOrient:
                op.Set(orient)
            if op.GetOpType() == pxr.UsdGeom.XformOp.TypeScale:
                op.Set(scale)

    def _setup_physics_material(self, path: pxr.Sdf.Path):

        # and ground plane
        self._material_static_friction = 10.0
        self._material_dynamic_friction = 10.0
        self._material_restitution = 0.0
        
        self._physicsMaterialPath = None

        if self._physicsMaterialPath is None:
            self._physicsMaterialPath = self.stage.GetDefaultPrim().GetPath().AppendChild("physicsMaterial")
            pxr.UsdShade.Material.Define(self.stage, self._physicsMaterialPath)
            material = pxr.UsdPhysics.MaterialAPI.Apply(self.stage.GetPrimAtPath(self._physicsMaterialPath))
            material.CreateStaticFrictionAttr().Set(self._material_static_friction)
            material.CreateDynamicFrictionAttr().Set(self._material_dynamic_friction)
            material.CreateRestitutionAttr().Set(self._material_restitution)

        collisionAPI = pxr.UsdPhysics.CollisionAPI.Get(self.stage, path)
        prim = self.stage.GetPrimAtPath(path)
        if not collisionAPI:
            collisionAPI = pxr.UsdPhysics.CollisionAPI.Apply(prim)
        # apply material
        physicsUtils.add_physics_material_to_prim(self.stage, prim, self._physicsMaterialPath)

    def set_particle_offset(self):
        # set partile up offset
        default_prim_path = self.stage.GetDefaultPrim().GetPath()
        particles = pxr.UsdGeom.Xformable(self.stage.GetPrimAtPath(default_prim_path.AppendPath("Particles")))
        particles.AddTranslateOp().Set(self._mugInitPos + self._fluidPositionOffset)