using KalmanFilter: Measurements, get_index, number_of_measurements, approxfloor
using Test 

@testset "approxfloor" begin
    @test (0.6 / 0.2) ≈ 3
    @test floor(Int, 0.6 / 0.2) == 2
    @test approxfloor(0.6 / 0.2) == 3
end 

@testset "get index" begin 
    # 4 mediciones cada 2 segundos 
    ms = Measurements([1. 2.; 2. 5.; 1. 2.; 0. 1.], 0.2)
    @test number_of_measurements(ms) == 4 
    @test get_index(ms, 0.0) == 1 
    @test get_index(ms, 0.1) == 1 
    @test get_index(ms, 0.2) == 2 
    @test get_index(ms, 0.6) == 4 
    @test get_index(ms, 0.9) == 4     
end 