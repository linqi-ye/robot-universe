using DigitRobot 
using MeshCat  
using CoordinateTransformations

include("gaits.jl")
mvis, mech, state = load_digit()
init_position = [0.0, 0.0, 0.0]
goal_position = [5.0, 0.0, 0.0] 

sl = 0.25 
N = Int(goal_position[1]/sl)

anim = MeshCat.Animation()  

lgaits = get_gaits(N, "left_leg")
rgaits = get_gaits(N, "right_leg")
f = get_com_poses(N; sl)

# lgaits = get_nomid_gaits(N, "left_leg")
# rgaits = get_nomid_gaits(N, "right_leg")
# f = get_com_no_mid_pose(N; sl)


xb,yb,zb = f,  0.0 .* ones(length(f)), 0.0 .* ones(length(f)) 
min_ = min(length(lgaits), length(rgaits),length(xb))

for i=1:min_
    atframe(anim, i*20) do
        settransform!(mvis.visualizer["world"], Translation(xb[i], yb[i], zb[i]))

        lqs = lgaits[i]
        set_limb_configuration!(lqs, "left_leg", mvis, mech) 

        rqs = rgaits[i]
        set_limb_configuration!(rqs, "right_leg", mvis, mech) 
    end
end
setanimation!(mvis.visualizer, anim)
 
render(mvis)

