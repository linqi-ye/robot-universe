using DigitRobot 
using MeshCat 
using lipm
using Plots
using CoordinateTransformations

include("gaits.jl")
mvis, mech, state = load_digit()
init_position = [0.0, 0.0, 0.0]
goal_position = [5.0, 0.0, 0.0]
# x = move_to_position(init_position; goal_position=goal_position, stride_length_x=0.25, stride_length_y=0.1, delta_t=0.1)

# f = [i for i=0:0.05:5]
# fl = []; fr = []
# for (i,ef) in enumerate(f)
#     if i%5. == 0.
#         push!(fl, ef+0.25)
#     else
#         push!(fl, ef)
#     end 
# end

# for (i,ef) in enumerate(f)
#     if i%5. == 0.
#         push!(fr, ef)
#     else
#         push!(fr, ef+0.25)
#     end 
# end
# ff = [i+0.25 for i in f]

 
N = 41
num_steps = N/2 -1
sl = 0.25
lid = [i for i=0:N if i%2 == 0]
rid = [i for i=0:N if i%2 == 1]
cid = [i for i=0:N]
fl = []; fr = [0.]; f = [0.]
for i in lid 
    r = sl*i 
    push!(fl, r)
    push!(fl, r)
end
for i in rid 
    r = sl*i 
    push!(fr, r)
    push!(fr, r)
end
for i in cid 
    r = sl*i 
    push!(f, r)
    push!(f, r)
end
# f = [sl*i for i in cid]
min_ = min(length(fl), length(fr), length(f))
pushfirst!(f, 0.)
f = f[1:min_]
fl = fl[1:min_]
fr = fr[1:min_]
# fl = [0., 0., 0.5, 0.5, 1.0, 1.0]
# f =  [0.0, 0.0, 0.0, 0.0, 0.25, 0.25, 0.5, 0.5, 0.75, 0.75, 1.0, 1.0]
# fr = [0., 0.25, 0.25, 0.75, 0.75, 1.25]
xb,yb,zb = f,  0.0 .* ones(length(f)), 0.0 .* ones(length(f))
xl,yl,zl = fl, 0.123 .* ones(length(fl)), -0.851 .* ones(length(fl))
xr,yr,zr =fr, -0.123 .* ones(length(fr)), -0.851 .* ones(length(fr))

plot(xb, yb, seriestype = :scatter, label="com")
plot!(xl,yl, seriestype = :scatter,label="left")
plot!(xr, yr, seriestype = :scatter, label="right")

anim = MeshCat.Animation()  
lgaits = get_gaits(N, "left_leg")
rgaits = get_gaits(N, "right_leg")
min_ = min(length(lgaits), length(rgaits),length(xb))
for i=1:min_
    atframe(anim, i*20) do
        settransform!(mvis.visualizer["world"], Translation(xb[i], yb[i], zb[i]))

        # lqs = solve_left_leg_ik(xl[i]-xb[i], yl[i], zl[i]) 
        # push!(lgait, lqs)
        lqs = lgaits[i]
        set_limb_configuration!(lqs, "left_leg", mvis, mech) 

        # rqs = solve_right_leg_ik(xr[i]-xb[i], yr[i], zr[i]) 
        # push!(rgait, rqs)
        rqs = rgaits[i]
        set_limb_configuration!(rqs, "right_leg", mvis, mech) 
    end
end
setanimation!(mvis.visualizer, anim)
 
render(mvis)

