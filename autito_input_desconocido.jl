#= Qué pasa si no conozco el control? Es como un input desconocido... =#
using KalmanFilter
#include("kalman.jl")

### Sistema básico
dt = 0.1
M = [1. dt; 0. 1.]
B = [dt^2/2, dt]
H = [1. 0.]

wn = 0.2
F = [dt*wn/2, wn]
G = [10.]
x0 = [0., 0.]

T = 60
N = Int(T/dt)

### Sistema generalizado, para agregar los inputs como estado
tildeM = [M B; 0. 0. 1.]
tildeH = [H 0.]

tildeP = [F * F' zeros(2); 0. 0. 1.]

tildex0 = [x0; 1.]

tildeF = [F; 0.]

tildeF * tildeF'
tildeG = G
begin
  observations2 = Vector{Float64}(undef, N)
  real_states2 = Array{Float64, 2}(undef, N, 3)
  predictions2 = Array{Float64, 2}(undef, N, 3)
  errors = Array{Float64,2}(undef, N, 3)

  updater2 = LinearUpdater(tildeM, zeros(3), tildeF)
  observer2 = LinearObserver(tildeH, zeros(1), G)
  X2 = StochasticState(tildex0, 0.)
  hatX2 = ObservedState(tildex0, tildeP)
  iterator2 = LinearKalmanIterator(X2, hatX2, updater2, observer2, tildeP, Normal())

  for i in 1:N
    control = 1.
    observation = observe_state_system(iterator2)
    previous_step!(iterator2, control, observation)

    # Save states
    observations2[i] = observation[1]
    real_states2[i,:] = iterator2.X.x
    predictions2[i,:] = iterator2.hatX.hatx
    errors[i,:] = [iterator2.hatX.hatP[j,j] for j in 1:3]
  end
end


# Estado real, observaciones y predicciones

using Plots
ts = 0.0:dt:(T-dt)

### Velocidad
plot(ts, real_states2[:,2], title = "Velocidad", label = "Real")
plot!(ts, predictions2[:,2], label = "Kalman", ribbon = sqrt.(errors[:,2]))

### Posición
plot(ts, real_states2[:,1], title = "Posición", label = "Real")
plot!(ts, observations2, label = "Observada")
plot!(ts, predictions2[:,1], label = "Kalman", ribbon = ribbon = sqrt.(errors[:,1]))

### Control
plot(ts, real_states2[:,3], title = "Control", label = "Real")
plot!(ts, predictions2[:,3], label = "Kalman", ribbon = sqrt.(errors[:,3]))
