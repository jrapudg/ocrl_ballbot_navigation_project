using RigidBodyDynamics
using LinearAlgebra
using StaticArrays
using MeshCat
using MeshCatMechanisms
using ForwardDiff

# Define urdf file
srcdir = "ballbot_arm_description"
urdf = joinpath(srcdir, "robots", "urdf", "ballbot_no_arms.urdf")
urdf_arms = joinpath(srcdir, "robots", "urdf", "ballbot_plus_fixedbase.urdf")

function build_ballbot()
    # Define urdf file
    srcdir = "ballbot_arm_description"
    urdf = joinpath(srcdir, "robots", "urdf", "ballbot_no_arms.urdf")
    ballbot = parse_urdf(urdf)
    return ballbot
end

function build_ballbot_arms()
    # Define urdf file
    srcdir = "ballbot_arm_description"
    urdf_arms = joinpath(srcdir, "robots", "urdf", "ballbot_plus_fixedbase.urdf")
    ballbot = parse_urdf(urdf_arms)
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