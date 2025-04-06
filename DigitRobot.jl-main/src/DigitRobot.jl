 module DigitRobot

using RigidBodyDynamics 
using MeshCat
using StaticArrays
using Colors

using MeshCatMechanisms
using CoordinateTransformations
using Rotations 

include("kinematics.jl")
include("utils.jl")
packagepath() = joinpath(@__DIR__,"..")
urdfpath() = joinpath(packagepath(), "urdf", "digit_model.urdf")
 
export mechanism, 
       setnominal!, 
       urdfpath, 
       default_background!

#kinematics
export  solve_left_leg_fk, 
        solve_left_leg_ik, 
        solve_right_leg_fk, 
        solve_right_leg_ik,
        left_leg_neutral_conf,
        right_leg_neutral_conf

#utils
export load_digit, 
        set_limb_configuration!

function mechanism(::Type{T} = Float64;
        floating = true, 
        remove_fixed_tree_joints = true, 
        add_flat_ground=true) where {T}

    mechanism = RigidBodyDynamics.parse_urdf(urdfpath(); scalar_type=T, floating=floating, remove_fixed_tree_joints=remove_fixed_tree_joints)

    # if contactmodel != nothing
    #     for side in (:left, :right)
    #         foot = findbody(mechanism, "$(string(side))_toe_roll")
    #         frame = default_frame(foot)

    #         #point of contact of foot
    #         x = 0.000559
    #         y = -0.050998
    #         z = -0.030691

    #         add_contact_point!(foot, ContactPoint(Point3D(frame, x,y,z), contactmodel))
    #     end
    # end

    if add_flat_ground
        frame = root_frame(mechanism)
        ground = RigidBodyDynamics.HalfSpace3D(Point3D(frame, 0.,0.,0.), FreeVector3D(frame, 0.,0.,1.))
        add_environment_primitive!(mechanism, ground)
    end

    remove_fixed_tree_joints && remove_fixed_tree_joints!(mechanism)

    return mechanism
end


function setnominal!(digitstate::MechanismState)
    mechanism = digitstate.mechanism
    zero!(digitstate)

    hip_abduction_left = findjoint(mechanism, "hip_abduction_left")
    hip_abduction_right = findjoint(mechanism, "hip_abduction_right")
    toe_pitch_joint_left = findjoint(mechanism, "toe_pitch_joint_left")
    toe_pitch_joint_right = findjoint(mechanism, "toe_pitch_joint_right")
    shoulder_pitch_left = findjoint(mechanism, "shoulder_pitch_joint_left")
    shoulder_pitch_right = findjoint(mechanism, "shoulder_pitch_joint_right")

    set_configuration!(digitstate, hip_abduction_left, 0.337)
    set_configuration!(digitstate, hip_abduction_right, -0.337)
    set_configuration!(digitstate, toe_pitch_joint_left, -0.126)
    set_configuration!(digitstate, toe_pitch_joint_right, 0.126)
    set_configuration!(digitstate, shoulder_pitch_left, 0.463)
    set_configuration!(digitstate, shoulder_pitch_right, -0.463)

    floatingjoint = first(out_joints(root_body(mechanism), mechanism))
    set_configuration!(digitstate, floatingjoint, [1; 0; 0; 0; 0; 0; 0.91])
    digitstate
end

function default_background!(mvis)
    vis = mvis.visualizer
    setvisible!(vis["/Background"], true)
    setprop!(vis["/Background"], "top_color", RGBA(1.0, 1.0, 1.0, 1.0))
    setprop!(vis["/Background"], "bottom_color", RGBA(1.0, 1.0, 1.0, 1.0))
    setvisible!(vis["/Axes"], false)
end

function __init__()
    if !isfile(urdfpath())
        error("Could not find $(urdfpath())")
    end
end

end #module
