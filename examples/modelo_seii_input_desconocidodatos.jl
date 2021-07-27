# # Filtro de Kalman Extendido con input desconocido

using KalmanFilter
using Plots
#using CSV

#total_RM = CSV.File("..\\data\\totales_RM.csv").Metropolitana

include("seii_model.jl")
include("common_defs_enkf.jl")

# Veamos un gráfico del control usado, para lo que definimos un vector con los tiempos `ts`.

#plot(ts, control_pieces.(ts), label = "Control real")

# Definimos las estructuras necesarias para crear un `LinearKalmanIterator`.


episystem_a(x, α, p) = [episystem_full(x[1:5], x[6],p); 0.]
epijacobian_a_x(x, α, p) = [epijacobian_full_x(x[1:5], x[6], p) epijacobian_full_u(x[1:5], x[6],p); zeros(6)']

#nlaugmented = KalmanFilter.NLUpdaterUnknowInput(nlupdater, control_pieces, 1.)
#==================================# 

odeupdater = KalmanFilter.ODEForecaster(dt, 10, tildex0, tildeP0, tildeF, episystem_a, epijacobian_a_x, p, (x) -> state_integrity(x, sum(x0)))
observer = KalmanFilter.LinearObserver(tildeH, zeros(1), G, observation_integrity)
#system = KalmanFilter.InnerState(tildex0)
#system = KalmanFilter.Measurements(total_RM, 1.)
iterator = KalmanFilter.LinearKalmanIterator(tildex0, tildeP0, odeupdater, observer, system, dt)
#enkf = KalmanFilter.EnKF(states, updater, observer, system, dt)
#==================================# 
# Y realizamos un total de `N` iteraciones, guardando los estamos intermedios
# en las variables que aparecen abajo.
results, ensamble = KalmanFilter.full_iteration(iterator, dt, N, t -> 0., 1)
#results2, ensamble = KalmanFilter.full_iteration(iterator2, dt, N, t -> 0., 1)

# ## Resultados
# Graficaremos los estados internos considerados, y los resultados obtenidos.

# Ahora podemos graficar los diferentes estados de nuestro sistema, así como las
# aproximaciones obtenidas con filtro de Kalman.
using Plots 
# Susceptibles ``S``
a = plot(results, ts, 1)
plot!(ts, sdesol'[1:end-1, 1])

# Expuestos ``E``
a = plot(results, ts, 2)
plot!(ts, sdesol'[1:end-1, 2])
#plot!(results2, ts, 2)
display(a)

# Infectados asintomáticos *mild* ``I^m``
a = plot(results, ts, 3)
#plot!(results2, ts, 3)
display(a)

# Infectados ``I``
a = plot(results, ts, 4)
#plot!(results2, ts, 4)
display(a)

# Infectados acumulados ``cI``, y las observaciones que hicimos de él.
a = plot(results, ts, 5)
#plot!(results2, ts, 5)
display(a)
# Finalmente, veamos el control usado y el aproximado
#plot(ts, control_pieces.(ts), label = "Control real")
plot(results, ts, 6, ylims = (-2,2))
plot!(ts, control_pieces.(ts), label = "Control real")

# Notamos que tras una cierta cantidad de tiempo es posible averiguar el control
# Aunque hay bastante incerteza de los resultados.
