#=
Test para los métodos básicos de filtro de Kalman
=#

using Test
# Autito
dt = 0.1
M = [1. dt; 0. 1.]
B = [dt^2/2, dt]

wn = 0.2
F = [dt*wn/2, wn]

updater = LinearUpdater(M, B, F)

X = StochasticState([0.,0.], 1.)

@test updater(X, 1., 0.) ≈ [0.005, 0.1]

updater(X, 1., 0.01)
