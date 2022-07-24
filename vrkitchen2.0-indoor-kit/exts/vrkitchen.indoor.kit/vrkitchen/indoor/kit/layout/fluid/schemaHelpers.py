from pxr import Usd, UsdGeom, Sdf, Gf, Vt, PhysxSchema


class PhysxParticleInstancePrototype:
    def __init__(self, mass=0.0, phase=0):
        self.mass = mass
        self.phase = phase


def addPhysxParticleSystem(
    stage,
    particle_system_path,
    contact_offset,
    rest_offset,
    particle_contact_offset,
    solid_rest_offset,
    fluid_rest_offset,
    solver_position_iterations,
    solver_velocity_iterations,
    wind,
    scenePath,
):

    particle_system = PhysxSchema.PhysxParticleSystem.Define(stage, particle_system_path)
    if particle_system:
        particle_system.CreateContactOffsetAttr().Set(contact_offset)
        particle_system.CreateRestOffsetAttr().Set(rest_offset)
        particle_system.CreateParticleContactOffsetAttr().Set(particle_contact_offset)
        particle_system.CreateSolidRestOffsetAttr().Set(solid_rest_offset)
        particle_system.CreateFluidRestOffsetAttr().Set(fluid_rest_offset)
        particle_system.CreateSolverPositionIterationCountAttr().Set(solver_position_iterations)
        particle_system.CreateSolverVelocityIterationCountAttr().Set(solver_velocity_iterations)
        particle_system.CreateWindAttr().Set(wind)

        # Reference simulation owner using PhysxPhysicsAPI
        physics_api = PhysxSchema.PhysxPhysicsAPI.Apply(particle_system.GetPrim())
        physics_api.CreateSimulationOwnerRel().SetTargets([scenePath])

        return particle_system
    else:
        return None


def addPhysxParticlesSimple(stage, path, prototypes, prototype_indices, positions, velocities, particle_system_path):

    prototype_base_path = path.pathString + "/particlePrototype"

    # Create point instancer
    shape_list = UsdGeom.PointInstancer.Define(stage, path)
    mesh_list = shape_list.GetPrototypesRel()

    # Create particle instance prototypes
    for i in range(len(prototypes)):
        prototype_path = prototype_base_path + str(i)
        geom_sphere = UsdGeom.Sphere.Define(stage, Sdf.Path(prototype_path))
        particle_instance_api = PhysxSchema.PhysxParticleAPI.Apply(geom_sphere.GetPrim())

        particle_instance_api.CreateSelfCollisionAttr().Set(prototypes[i].selfCollision)
        particle_instance_api.CreateFluidAttr().Set(prototypes[i].fluid)
        particle_instance_api.CreateParticleGroupAttr().Set(prototypes[i].collisionGroup)

        particle_instance_api.CreateMassAttr().Set(prototypes[i].mass)

        # Reference simulation owner using PhysxPhysicsAPI
        physics_api = PhysxSchema.PhysxPhysicsAPI.Apply(geom_sphere.GetPrim())
        physics_api.CreateSimulationOwnerRel().SetTargets([particle_system_path])

        # add mesh references to point instancer
        mesh_list.AddTarget(Sdf.Path(prototype_path))

    # Set particle instance data
    mesh_indices = []
    for i in range(len(positions)):
        mesh_indices.append(prototype_indices[i])

    orientations = [Gf.Quath(1.0, Gf.Vec3h(0.0, 0.0, 0.0))] * len(positions)

    angular_velocities = [Gf.Vec3f(0.0, 0.0, 0.0)] * len(positions)

    shape_list.GetProtoIndicesAttr().Set(mesh_indices)
    shape_list.GetPositionsAttr().Set(positions)
    shape_list.GetOrientationsAttr().Set(orientations)
    shape_list.GetVelocitiesAttr().Set(velocities)
    shape_list.GetAngularVelocitiesAttr().Set(angular_velocities)


def addPhysxClothWithConstraints(
    stage,
    path,
    positions,
    normals,
    rest_positions,
    velocities,
    inv_masses,
    triangle_indices,
    spring_connections,
    spring_stiffnesses,
    spring_dampings,
    spring_rest_lengths,
    self_collision,
    self_collision_filter,
    inv_gravity,
    particle_group,
    particle_system_path,
):

    mesh = UsdGeom.Mesh.Define(stage, path)
    prim = mesh.GetPrim()

    mesh.CreateDoubleSidedAttr().Set(True)

    vertex_count_attr = mesh.CreateFaceVertexCountsAttr()
    vertex_indices_attr = mesh.CreateFaceVertexIndicesAttr()
    norm_attr = mesh.CreateNormalsAttr()
    point_attr = mesh.CreatePointsAttr()

    # Triangle array's vertex count per face is always 3
    vertex_count = 3
    array_size = int(len(triangle_indices) / 3)
    index_array = Vt.IntArray(array_size, vertex_count)
    vertex_count_attr.Set(index_array)

    vertex_indices_attr.Set(triangle_indices)
    norm_attr.Set(normals)
    point_attr.Set(positions)

    cloth_api = PhysxSchema.PhysxClothAPI.Apply(prim)

    cloth_api.CreateSelfCollisionAttr().Set(self_collision)
    cloth_api.CreateSelfCollisionFilterAttr().Set(self_collision_filter)
    cloth_api.CreateParticleGroupAttr().Set(particle_group)

    # Reference simulation owner using PhysxPhysicsAPI
    physics_api = PhysxSchema.PhysxPhysicsAPI.Apply(prim)
    physics_api.CreateSimulationOwnerRel().SetTargets([particle_system_path])

    # Custom attributes
    prim.CreateAttribute("invGravity", Sdf.ValueTypeNames.Bool).Set(inv_gravity)

    prim.CreateAttribute("springConnections", Sdf.ValueTypeNames.Int2Array).Set(spring_connections)
    prim.CreateAttribute("springStiffnesses", Sdf.ValueTypeNames.FloatArray).Set(spring_stiffnesses)
    prim.CreateAttribute("springDampings", Sdf.ValueTypeNames.FloatArray).Set(spring_dampings)
    prim.CreateAttribute("springRestLengths", Sdf.ValueTypeNames.FloatArray).Set(spring_rest_lengths)

    prim.CreateAttribute("restPositions", Sdf.ValueTypeNames.Point3fArray).Set(rest_positions)
    prim.CreateAttribute("velocities", Sdf.ValueTypeNames.Point3fArray).Set(velocities)

    prim.CreateAttribute("inverseMasses", Sdf.ValueTypeNames.FloatArray).Set(inv_masses)


def addPhysxCloth(
    stage,
    path,
    dynamic_mesh_path,
    initial_velocity,
    initial_mass,
    stretch_stiffness,
    bend_stiffness,
    shear_stiffness,
    self_collision,
    self_collision_filter,
    inv_gravity,
    particle_group,
    particle_system_path,
):

    mesh = UsdGeom.Mesh.Define(stage, path)
    prim = mesh.GetPrim()

    if dynamic_mesh_path:
        prim.GetReferences().AddReference(dynamic_mesh_path)

    cloth_api = PhysxSchema.PhysxClothAPI.Apply(prim)
    cloth_api.CreateDefaultParticleVelocityAttr().Set(initial_velocity)
    cloth_api.CreateDefaultParticleMassAttr().Set(initial_mass)

    cloth_api.CreateStretchStiffnessAttr().Set(stretch_stiffness)
    cloth_api.CreateBendStiffnessAttr().Set(bend_stiffness)
    cloth_api.CreateShearStiffnessAttr().Set(shear_stiffness)

    cloth_api.CreateSelfCollisionAttr().Set(self_collision)
    cloth_api.CreateSelfCollisionFilterAttr().Set(self_collision_filter)
    cloth_api.CreateParticleGroupAttr().Set(particle_group)

    # Reference simulation owner using PhysxPhysicsAPI
    physics_api = PhysxSchema.PhysxPhysicsAPI.Apply(prim)
    physics_api.CreateSimulationOwnerRel().SetTargets([particle_system_path])

    # Custom attributes
    prim.CreateAttribute("invGravity", Sdf.ValueTypeNames.Bool).Set(inv_gravity)


def applyInflatableApi(stage, path, pressure):

    prim = stage.GetPrimAtPath(path)
    # TODO: Add more checks here
    if prim.IsValid():
        inflatable_api = PhysxSchema.PhysxInflatableAPI.Apply(prim)
        inflatable_api.CreatePressureAttr().Set(pressure)


def _get_rigid_attachments(stage, prim: Usd.Prim):
    attachments = []
    rigidAttachmentRel = prim.CreateRelationship("physxRigidAttachments")
    for attachment_path in rigidAttachmentRel.GetTargets():
        attachment = PhysxSchema.PhysxRigidAttachment.Get(stage, attachment_path)
        if attachment:
            attachments.append(attachment)
    return attachments


# def _get_rigid_attachment_target(attachment: PhysxSchema.PhysxRigidAttachment):
#     targets = attachment.GetRigidRel().GetTargets()
#     if len(targets) <= 0:
#         return Sdf.Path()
#     else:
#         return targets[0]


# def _create_rigid_attachment(
#     stage, attachment_path: Sdf.Path, rigidbody_path: Sdf.Path, deformable_path: Sdf.Path
# ) -> PhysxSchema.PhysxRigidAttachment:
#     attachment = PhysxSchema.PhysxRigidAttachment.Define(stage, attachment_path)
#     attachment.GetRigidRel().SetTargets([rigidbody_path])
#     attachment.GetDeformableRel().SetTargets([deformable_path])
#     return attachment


# def add_deformable_to_rigid_body_attachment(
#     stage, target_attachment_path: Sdf.Path, deformable_path: Sdf.Path, rigid_path: Sdf.Path
# ):
#     deformable_prim = stage.GetPrimAtPath(deformable_path)
#     softbody_xformable = UsdGeom.Xformable(deformable_prim)
#     rigidbody_prim = stage.GetPrimAtPath(rigid_path)
#     rigidbody_xformable = UsdGeom.Xformable(rigidbody_prim)

#     attachments = _get_rigid_attachments(stage, deformable_prim)
#     if any(_get_rigid_attachment_target(attachment) == rigid_path for attachment in attachments):
#         return False

#     # Create new attachment
#     attachment = _create_rigid_attachment(stage, target_attachment_path, rigid_path, deformable_path)

#     attachment_prim = attachment.GetPrim()
#     attachment_prim.CreateAttribute("physxEnableHaloParticleFiltering", Sdf.ValueTypeNames.Bool).Set(True)
#     attachment_prim.CreateAttribute("physxEnableVolumeParticleAttachments", Sdf.ValueTypeNames.Bool).Set(True)
#     attachment_prim.CreateAttribute("physxEnableSurfaceTetraAttachments", Sdf.ValueTypeNames.Bool).Set(False)

#     sb_bound = softbody_xformable.ComputeLocalBound(
#         Usd.TimeCode.Default(), purpose1=softbody_xformable.GetPurposeAttr().Get()
#     )
#     sb_size = sb_bound.ComputeAlignedBox().GetSize()
#     avg_dim = (sb_size[0] + sb_size[1] + sb_size[2]) / 3.0
#     default_rad = avg_dim * 0.05
#     attachment_prim.CreateAttribute("physxHaloParticleFilteringRadius", Sdf.ValueTypeNames.Float).Set(default_rad * 4)
#     attachment_prim.CreateAttribute("physxVolumeParticleAttachmentRadius", Sdf.ValueTypeNames.Float).Set(default_rad)
#     attachment_prim.CreateAttribute("physxSurfaceSamplingRadius", Sdf.ValueTypeNames.Float).Set(default_rad)

#     # Update soft body relationship
#     attachments.append(attachment)
#     attachment_paths = [attachment.GetPath() for attachment in attachments]
#     deformable_prim.CreateRelationship("physxRigidAttachments").SetTargets(attachment_paths)

#     # Store the global xforms
#     globalPose = rigidbody_xformable.ComputeLocalToWorldTransform(Usd.TimeCode.Default())
#     attachment_prim.CreateAttribute("physxRigidBodyXform", Sdf.ValueTypeNames.Matrix4d).Set(globalPose)

#     globalPose = softbody_xformable.ComputeLocalToWorldTransform(Usd.TimeCode.Default())
#     attachment_prim.CreateAttribute("physxDeformableXform", Sdf.ValueTypeNames.Matrix4d).Set(globalPose)
