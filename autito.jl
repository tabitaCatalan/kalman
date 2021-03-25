include("kalman.jl")


## Caso autito moviéndose en línea recta

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
begin
  observations = Vector{Float64}(undef, N)
  real_states = Array{Float64, 2}(undef, N, 2)

  updater = LinearUpdater(M, B, F)
  observer = LinearObserver(H, zeros(1), G)
  X = StochasticState(x0, 1.)
  hatX = ObservedState(x0, F * F')
  iterator = LinearKalmanIterator(X, hatX, updater, observer, F * F', Normal())

  for i in 1:N
    control = 1.
    observation = observe_state_system(iterator)
    previous_step!(iterator, control, observation)

    # Save states
    observations[i] = observation[1]
    real_states[i,:] = iterator.X.x
  end
end

using Plots

ts = 0.0:dt:(T-dt)

plot(ts, real_states[:,2], title = "Velocidad")

plot(ts, real_states[:,1], title = "Posición", label = "Real")

plot!(ts, observations, label = "Observada")
