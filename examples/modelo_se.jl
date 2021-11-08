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
#α = 1.881556778828361e-8
γₑ = 0.14
γᵢ = 0.14
φ = 0.3

include("KalmanFilter\\src\\seii_model.jl")


p = (gammae = γₑ, gammai = γᵢ, phi = φ)

dt = 0.2

#####################################
x0 = [100., 10., 0.1, 0.1, 0.1]
x0 = [7.112808e6, 1046.8508799517147, 0.0, 521.963080420307, 0.0]
F = 0.1 * ones(5)
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

rk = KalmanFilter.RK4Dx(episystem_full, epijacobian_full_x, p, dt)
#eu = KalmanFilter.Euler(episystem_full, epijacobian_full_x, p, dt)

####################

T = 40
N = Int(T/dt)

nlupdater = NLUpdater(rk,F,x0,1.)
observer = KalmanFilter.LinearObserver(H, zeros(1), G)

control(t, limit) = t <= limit ? 1. : 0.5

control(0.1, 12)
#Juno.@enter KalmanFilter.full_iteration(iterator, N)
iterator = KalmanFilter.SimpleKalmanIterator(x0,F*F',nlupdater,observer)
observations, real_states, analysis, forecast, errors_analysis, errors_forecast = KalmanFilter.full_iteration(iterator, dt, N, t -> control(t, 50.))
using Plots


function plotstate!(a_plot, state_index, ts, rango = 1:length(ts); labels = ["Real", "Kalman analysed", "Kalman forecast"])
  i = state_index
  dimensions = size(real_states)[2]
  nans = NaN * ones(dimensions)'
  errors_forecast_correction = [nans; errors_forecast[1:end-1,:]]
  forecast_correction = [nans; forecast[1:end-1,:]]
  if i ≠ 6
    plot!(a_plot, ts[rango], real_states[rango,i], label = labels[1])
  end
  #plot!(a_plot, ts[rango], analysis[rango,i], label = labels[2])

  plot!(a_plot, ts[rango], analysis[rango,i], label = labels[2], ribbon = 2 * sqrt.(abs.(errors_analysis[rango,i])))
  plot!(a_plot, ts[rango], analysis[rango,i], label = labels[2], ribbon = sqrt.(abs.(errors_analysis[rango,i])))
  #plot!(a_plot, ts[rango], forecast_correction[rango,i], label = labels[3], ribbon = sqrt.(abs.(errors_forecast_correction[rango,i])))
  a_plot
end

ts = 0.0:dt:(T-dt)
rango = 1:floor(Int,length(ts))
a_plot = plot(title = "Susceptibles")
plotstate!(a_plot, 1, ts)
plotstate(2, "Expuestos", ts)
plotstate(3, "Mild", ts)
plotstate(4, "Infectados", ts)
plotstate(5, "Acumulados", ts)

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
