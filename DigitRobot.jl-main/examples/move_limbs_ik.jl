using Revise 
using DigitRobot 

# mvis, mech, state = load_digit()
# open(mvis)
# qs = solve_left_leg_ik(0.25, 0.123, -0.851) 
set_limb_configuration!(qs, "left_leg", mvis, mech) 
println("\nfoot position: ",solve_left_leg_fk(qs))