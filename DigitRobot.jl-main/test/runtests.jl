using Test
using DigitRobot
using RigidBodyDynamics
using RigidBodyDynamics.Contact: location

@testset "digit" begin
    mechanism = DigitRobot.mechanism()
    @test num_velocities(mechanism) == 28
    @test num_positions(mechanism) == 29
end


@testset "setnominal!" begin
    mechanism = DigitRobot.mechanism()
    state = MechanismState(mechanism)
    DigitRobot.setnominal!(state)
 
    @test center_of_mass(state).v[3] > 0.8
end
