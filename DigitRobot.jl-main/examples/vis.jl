using MuJoCo, LyceumMuJoCo, LyceumMuJoCoViz
import Distributions:Uniform

mj_activate("/home/alphonsus/keys/mjkey.txt")
T=100
m = jlModel("models/cartpole.xml")
d = jlData(m) 
sim = MJSim(m,d)
states = Array(undef, statespace(sim), T)
for t = 1:T
    step!(sim)
    states[:, t] .= getstate(sim)
end

visualize(sim, trajectories=[states], 
          controller = sim->setaction!(sim, [rand(Uniform(-1.,1.))]))