function get_limb_joint_names(limb::String)
    if limb == "right_leg"
        return ["hip_abduction_right", "hip_rotation_right", "hip_flexion_right", "knee_joint_right", "shin_to_tarsus_right", "toe_pitch_joint_right", "toe_roll_joint_right"]
    elseif limb == "left_leg"
        return ["hip_abduction_left", "hip_rotation_left", "hip_flexion_left", "knee_joint_left", "shin_to_tarsus_left", "toe_pitch_joint_left", "toe_roll_joint_left"]
    else
        return nothing 
    end


end
function load_digit()
    mech  = mechanism(add_flat_ground=true)
    mvis = MechanismVisualizer(mech, URDFVisuals(DigitRobot.urdfpath(), 
            package_path=[DigitRobot.packagepath()])) 
    state = MechanismState(mech)
    setnominal!(state)
    set_configuration!(mvis, configuration(state))
    default_background!(mvis)
    settransform!(mvis.visualizer["/Cameras/default"],
            compose(Translation(0.0, 0.0, -1.0), LinearMap(RotZ(-pi / 2.0))))

    return mvis, mech, state
end 

function set_limb_configuration!(qs::Vector, limb::String, mvis, mech) 
    jns = get_limb_joint_names(limb)
    for (i, jn) in enumerate(jns)
        set_configuration!(mvis, findjoint(mech, jn), qs[i])
    end
    if limb == "left_leg"
        set_configuration!(mvis, findjoint(mech, jns[6]), -0.115)
    elseif limb == "right_leg"
        set_configuration!(mvis, findjoint(mech, jns[6]), 0.115)
    end
end