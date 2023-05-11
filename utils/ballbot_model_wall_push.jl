using RigidBodyDynamics
using LinearAlgebra
using StaticArrays
using MeshCat
using MeshCatMechanisms
using ForwardDiff

# Define urdf file
srcdir = "ballbot_arm_description"
urdf = joinpath(srcdir, "robots", "urdf", "ballbot_no_arms.urdf")
urdf_arms = joinpath(srcdir, "robots", "urdf", "ballbot_plus_fixedbase_two_link_arm.urdf")

function build_ballbot()
    # Define urdf file
    srcdir = "ballbot_arm_description"
    urdf = joinpath(srcdir, "robots", "urdf", "ballbot_no_arms.urdf")
    ballbot = parse_urdf(urdf)
    return ballbot
end

function build_reduced_ballbot_arms()
    # Define urdf file
    srcdir = "ballbot_arm_description"
    urdf_arms = joinpath(srcdir, "robots", "urdf", "ballbot_plus_fixedbase_two_link_arm.urdf")
    # ballbot = parse_urdf(urdf_arms)
    ballbot = parse_urdf(urdf_arms; remove_fixed_tree_joints=true)
    return ballbot
end

function build_ballbot_arms()
    # Define urdf file
    srcdir = "ballbot_arm_description"
    urdf_arms = joinpath(srcdir, "robots", "urdf", "ballbot_plus.urdf")
    ballbot = parse_urdf(urdf_arms)
    return ballbot
end

function build_ballbot_arms_against_wall(left_end_coord::AbstractVector{T}, right_end_coord::AbstractVector{T}) where {T}
    # Define urdf file
    srcdir = "ballbot_arm_description"
    urdf_arms = joinpath(srcdir, "robots", "urdf", "ballbot_plus_fixedbase_two_link_arm.urdf")
    ballbot = parse_urdf(urdf_arms; remove_fixed_tree_joints=true)

    ## attach loop joints fixed to world
    world = RigidBodyDynamics.findbody(ballbot, "world")
    r_end_eff_body = RigidBodyDynamics.findbody(ballbot, "RArm4")
    joint_r_wall = Joint("JRA8", RigidBodyDynamics.QuaternionSpherical{Float64}())
    r_wall_tf = Transform3D(default_frame(world), frame_after(joint_r_wall), SVector(-right_end_coord[1], -right_end_coord[2], -right_end_coord[3]))
    r_end_eff_tf = RigidBodyDynamics.frame_definitions(r_end_eff_body)[end]
    r_end_tf = Transform3D(frame_before(joint_r_wall), r_end_eff_tf.to, r_end_eff_tf.mat)
    attach!(ballbot, r_end_eff_body, world, joint_r_wall; joint_pose = r_end_tf, successor_pose = r_wall_tf)

    l_end_eff_body = RigidBodyDynamics.findbody(ballbot, "LArm4")
    joint_l_wall = Joint("JLA8", RigidBodyDynamics.QuaternionSpherical{Float64}())
    l_wall_tf = Transform3D(default_frame(world), frame_after(joint_l_wall), SVector(-left_end_coord[1], -left_end_coord[2], -left_end_coord[3]))
    l_end_eff_tf = RigidBodyDynamics.frame_definitions(l_end_eff_body)[end]
    l_end_tf = Transform3D(frame_before(joint_l_wall), l_end_eff_tf.to, l_end_eff_tf.mat)
    attach!(ballbot, l_end_eff_body, world, joint_l_wall; joint_pose = l_end_tf, successor_pose = l_wall_tf)

    return ballbot  
end

struct Ballbot{C}
    mech::Mechanism{Float64}
    statecache::C
    dyncache::DynamicsResultCache{Float64}
    xdot::Vector{Float64}
    function Ballbot(mech::Mechanism)
        N = num_positions(mech) + num_velocities(mech)
        statecache = StateCache(mech)
        rescache = DynamicsResultCache(mech)
        xdot = zeros(N)
        new{typeof(statecache)}(mech, statecache, rescache, xdot)
    end
end

function Ballbot()
    Ballbot(build_ballbot())
end

function Ballbot_arms()
    Ballbot(build_ballbot_arms())
end

function Ballbot_arms_reduced()
    Ballbot(build_reduced_ballbot_arms())
end

function Ballbot_wall(left_end_coord::AbstractVector{T}, right_end_coord::AbstractVector{T}) where {T}
    Ballbot(build_ballbot_arms_against_wall(left_end_coord, right_end_coord))
end

function m_dynamics(model::Ballbot, x::AbstractVector{T1}, u::AbstractVector{T2}) where {T1,T2} 
    T = promote_type(T1,T2)
    state = model.statecache[T]
    res = model.dyncache[T]

    # Convert from state ordering to the ordering of the mechanism
    copyto!(state, x)
    τ = [u[1:2]; zeros(2); u[3:end]]
    dynamics!(res, state, τ)
    q̇ = res.q̇
    v̇ = res.v̇
    return [q̇; v̇]
end

function rk4(model::Ballbot, x::Vector, u, dt::Float64)
    # Vanilla RK4
    k1 = dt*m_dynamics(model, x, u)
    k2 = dt*m_dynamics(model, x + k1/2, u)
    k3 = dt*m_dynamics(model, x + k2/2, u)
    k4 = dt*m_dynamics(model, x + k3, u)
    x + (1/6)*(k1 + 2*k2 + 2*k3 + k4)
end