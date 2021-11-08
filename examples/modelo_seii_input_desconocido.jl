# # Filtro de Kalman Extendido con input desconocido
using Random
using KalmanFilter
using Plots
using LinearAlgebra
include("examples\\seii_model.jl")

include("common_defs.jl")

# Definimos las matrices ``\tilde{H}`` para nuestro sistema auxiliar, que incluirá al input.
tildeH = [H 0.]

# EL nuevo vector de estados será ``\tilde{\mathbf{x}} = \begin{pmatrix} \mathbf{x}_0 \\ 1. \end{pmatrix}``,
# donde estamos considerando una suposición inicial para el estado $\alpha = 1$.

# Veamos un gráfico del control usado, para lo que definimos un vector con los tiempos `ts`.
plot(ts, control_pieces.(ts), label = "Control real")


a₀ = 1.; Fₐ = 1.5
tildeP = [F * F' zeros(5); zeros(5)' Fₐ]

# Definimos las estructuras necesarias para crear un `SimpleKalmanIterator`.

Q = Diagonal(sqrt(dt) * ones(5))
nlupdater = NLUpdater(rk,F, Q, x0, a₀, (x) -> state_integrity(x, sum(x0)))
nlaugmented = KalmanFilter.NLUpdaterUnknowInput(nlupdater, control_pieces, a₀)
observer = KalmanFilter.LinearObserver(tildeH, zeros(1), G)
system = KalmanFilter.InnerState(tildex0)
iterator = KalmanFilter.SimpleKalmanIterator(tildex0, tildeP, nlaugmented, observer, system, dt)

# Y realizamos un total de `N` iteraciones, guardando los estamos intermedios
# en las variables que aparecen abajo.
results, ensamble = KalmanFilter.full_iteration(iterator, dt, N, t -> 0., 1)

# ## Resultados
# Graficaremos los estados internos considerados, y los resultados obtenidos.

# Ahora podemos graficar los diferentes estados de nuestro sistema, así como las
# aproximaciones obtenidas con filtro de Kalman.

# Susceptibles ``S``
plot(results, ts, 1)

# Expuestos ``E``
plot(results, ts, 2)

# Infectados asintomáticos *mild* ``I^m``
plot(results, ts, 3)

# Infectados ``I``
plot(results, ts, 4)

# Infectados acumulados ``cI``, y las observaciones que hicimos de él.
plot(results, ts, 5)

# Finalmente, veamos el control usado y el aproximado
plot(ts, control_pieces.(ts), label = "Control real")
plot!(results, ts, 6)

# Notamos que tras una cierta cantidad de tiempo es posible averiguar el control
# Aunque hay bastante incerteza de los resultados.
