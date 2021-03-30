#= Modelo SEIIR, pero selo SE (no lineal)
Modo simple, sin input desconocido =#

using KalmanFilter

####################

# Modelo
#=
α = 1.
γₑ = 1/5
γᵢ = 1/5
φ = 0.4
=#
α = 1.881556778828361e-8
γₑ = 0.14
γᵢ = 0.14
φ = 0.3


p = (gammae = γₑ, gammai = γᵢ, phi = φ)

dt = 0.2

function episystem_full(x, α, p)
  γᵢ = p.gammai; γₑ = p.gammae; φ = p.phi;
  β = 1e-7
  [-α * β * x[1] * (x[2] + x[3]),
  α * β * x[1] * (x[2] + x[3]) - γₑ * x[2],
  (1 - φ) * γₑ * x[2] - γᵢ * x[3],
  φ * γₑ * x[2] - γᵢ * x[4],
  φ * γₑ * x[2]]
end
function epijacobian_full_x(x, α, p)
  γᵢ = p.gammai; γₑ = p.gammae; φ = p.phi;
  β = 1e-7
  [-α*β*(x[2] + x[3])  -α*β*x[1]     -α*β*x[1]  0.  0.;
  α*β*(x[2] + x[3])  (α*β*x[1]-γₑ)  α*β*x[1]  0.  0.;
  0.                 (1-φ)*γₑ       -γᵢ       0.  0.;
  0.                 φ*γₑ           0.        -γᵢ 0.;
  0.                 φ*γₑ           0.        0.  0.]
end


#####################################
x0 = [100., 10., 0.1, 0.1, 0.1]
x0 = [7.112808e6, 1046.8508799517147, 0.0, 521.963080420307, 0.0]
F = 100. * ones(5)
G = [50000.]
H = [0. 0. 0. 0. 1.]
dimensions = 5


#=

episystem(x, α, γ) = [-α * x[1] * x[2],
α * x[1] * x[2] - γₑ * x[2]]
epijacobian_x(x, α, γ) = [-α*x[2]   -α*x[1];
α*x[2]   α*x[1]-γₑ]


x0 = [100., 10]
F = 0.0001 * ones(2)
G = [0.5]
H = [1. 0.]
dimensions = 2
=#

rk = KalmanFilter.RK4(episystem_full, epijacobian_full_x, p, dt)
eu = KalmanFilter.Euler(episystem_full, epijacobian_full_x, p, dt)

####################

T = 40
N = Int(T/dt)

nlupdater = NLUpdater(rk,F,x0,1.)
observer = KalmanFilter.LinearObserver(H, zeros(1), G)


iterator = KalmanFilter.LinearKalmanIterator(x0,F*F',nlupdater,observer)



#Juno.@enter KalmanFilter.full_iteration(iterator, N)

observations, real_states, analysis, forecast, errors_analysis, errors_forecast = KalmanFilter.full_iteration(iterator, N)
using Plots


function plotstate(state_index, title)
  i = state_index
  nans = NaN * ones(dimensions)'
  errors_forecast_correction = [nans; errors_analysis[1:end-1,:]]
  forecast_correction = [nans; forecast[1:end-1,:]]

  a_plot = plot(title = title)
  plot!(a_plot, ts[rango], real_states[rango,i], label = "Real")
  plot!(a_plot, ts[rango], analysis[rango,i], label = "Kalman analysed", ribbon = sqrt.(abs.(errors_analysis[rango,i])))
  plot!(a_plot, ts[rango], forecast_correction[rango,i], label = "Kalman forecast", ribbon = sqrt.(abs.(errors_forecast_correction[rango,i])))
  a_plot
end

ts = 0.0:dt:(T-dt)
rango = 1:floor(Int,length(ts))
plotstate(1, "Susceptibles")
plotstate(2, "Expuestos")
plotstate(3, "Mild")
plotstate(4, "Infectados")
plotstate(5, "Acumulados")

plot!(ts[rango], observations[rango], label = "Observaciones", legend =:bottomright)

function plot_error(state_index, state_name)
  i = state_index
  a_plot = plot(title = "Error " * state_name)
  plot!(a_plot, ts[1:end-1], sqrt.(errors_analysis[2:end,i]), label = "Analysis")
  plot!(a_plot, ts, sqrt.(errors_forecast[:,i]), label = "Forecast")
end

plot_error(1, "Susceptibles")
plot_error(2, "Expuestos")
plot_error(3, "Mild")

#################################################################
### Modelo SEIIR con input α desconocido
#################################################################

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
  end
end
# una interfaz que permita trabajar con un sistema


# Verificar los resultados del discretizador
using Plots
x0 = [100., 10., 0., 0., 0.]
results = Array{Float64, 2}(undef, N, 5)
xn = x0
for n in 1:N
  results[n,:] = xn
  xn = rk(xn, 1.)
  if xn[2] < 0
    print(n)
    error("x[2] our of domain")
  end
end

results[67,:]

plot(results[:,1])
plot!(results[:,5])

plot(results[:,2])
plot!(results[:,3])
plot!(results[:,4])


Juno.@enter rk(x0,1.)
