# # Filtro de Kalman Extendido con input desconocido

using KalmanFilter
using Plots

include("seii_model.jl")

# Definimos un vector de condiciones iniciales ``x_0``.

x0 = [7.112808e6, 1046.8508799517147, 0.0, 521.963080420307, 0.0]
F = 1000. * ones(5)

# Solo consideraremos conocida la cantidad total de infectados ``\mathbf{x}_5 = cI``.
H = [0. 0. 0. 0. 1.]

# Usaremos un error de unas 50.000 personas en las mediciones.
G = [50000.]
dimensions = 5

# Definimos las matrices ``\tilde{H}`` para nuestro sistema auxiliar, que incluirá al input.
tildeH = [H 0.]

tildeP = [F * F' zeros(5); zeros(5)' 1.]

# EL nuevo vector de estados será ``\tilde{\mathbf{x}} = \begin{pmatrix} \mathbf{x}_0 \\ 1. \end{pmatrix}``,
# donde estamos considerando una suposición inicial para el estado $\alpha = 1$.
tildex0 = [x0; 1.]

dt = 0.2
T = 100.
N = Int(T/dt)

# Definimos los parámetros
γₑ = 0.14
γᵢ = 1/14
φ = 0.3

# Que agrupamos en una tupla `p`.
p = (gammae = γₑ, gammai = γᵢ, phi = φ)


# Usaremos un `Discretizer` RungeKutta de orden 4, diferenciable tanto en el estado
# ``x`` como en el control ``u``. Lo construimos a partir de un discretizador solo
# diferenciable en ``x`` por comodidad.
rkx = KalmanFilter.RK4Dx(episystem_full, epijacobian_full_x, p, dt)
rk = KalmanFilter.RK4Du(rkx, epijacobian_full_u)


# El control para el sistema interno (que supondremos desconocido y es el que intentaremos
# averiguar) será la función constante por pedazos `control_pieces`.

function control_pieces(t)
    if t < 5.
        0.5
    elseif t >= 5. && t < 10.
        0.5
    elseif t >= 10. && t < 15.
        1.1
    elseif t >= 15. && t < 20.
        1.5
    elseif t >= 20.
        2.5
    end
end

# Definimos las estructuras necesarias para crear un `LinearKalmanIterator`.
nlupdater = NLUpdater(rk,F,x0,1.)
nlaugmented = KalmanFilter.NLUpdaterUnknowInput(nlupdater, control_pieces)
observer = KalmanFilter.LinearObserver(tildeH, zeros(1), G)
iterator = KalmanFilter.LinearKalmanIterator(tildex0, tildeP, nlaugmented, observer)

# Y realizamos un total de `N` iteraciones, guardando los estamos intermedios
# en las variables que aparecen abajo.
observations, real_states, analysis, forecast, errors_analysis, errors_forecast = KalmanFilter.full_iteration(iterator, dt, N, t -> 0.)

# ## Resultados
# Graficaremos los estados internos considerados, y los resultados obtenidos.
# Definimos un vector con los tiempos
ts = 0.0:dt:(T-dt)
rango = 1:floor(Int,length(ts))

# Definimos unas funciones horribles para acortar el proceso
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

function plot_error(state_index, state_name)
  i = state_index
  a_plot = plot(title = "Error " * state_name)
  plot!(a_plot, ts[1:end-1], sqrt.(errors_analysis[2:end,i]), label = "Analysis")
  plot!(a_plot, ts, sqrt.(errors_forecast[:,i]), label = "Forecast")
end

# Y procedemos a graficar

a_plot = plot(title = "Susceptibles")
plotstate!(a_plot, 1, ts)

a_plot = plot(title = "Infectado")
plotstate!(a_plot, 4, ts)

#plot_error(1, "Susceptibles")

a_plot = plot(title = "Expuestos")
plotstate!(a_plot, 2, ts)

a_plot = plot(title = "Mild")
plotstate!(a_plot, 3, ts)



# Observemos el control obtenido
a_plot = plot(title = "Control")
plot!(ts, control_pieces.(ts), label = "Control real")
plotstate!(a_plot, 6, ts)

# Notamos que tras una cierta cantidad de tiempo es posible averiguarlo con bastante certeza.

# Podemos ver el estado ``cI``, y las observaciones que hicimos de él.
a_plot = plot(title = "Acumulados")
plotstate!(a_plot, 5, ts)
plot!(ts[rango], observations[rango], label = "Observaciones", legend =:bottomright)

# Salió bien :3
