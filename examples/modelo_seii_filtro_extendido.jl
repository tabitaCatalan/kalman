# # Filtro de Kalman Extendido

using KalmanFilter
using Plots
using LinearAlgebra

include("examples\\seii_model.jl")

include("common_defs.jl")

# Definimos las matrices ``\tilde{H}`` para nuestro sistema auxiliar, que incluirá al input.

# EL nuevo vector de estados será ``\tilde{\mathbf{x}} = \begin{pmatrix} \mathbf{x}_0 \\ 1. \end{pmatrix}``,
# donde estamos considerando una suposición inicial para el estado $\alpha = 1$.


# Usaremos un `Discretizer` RungeKutta de orden 4, diferenciable tanto en el estado
# ``x`` como en el control ``u``. Lo construimos a partir de un discretizador solo
# diferenciable en ``x`` por comodidad.
rkx = KalmanFilter.RK4Dx(episystem_full, epijacobian_full_x, p, dt)
rk = KalmanFilter.RK4Du(rkx, epijacobian_full_u)

plot(ts, control_pieces.(ts), label = "Control real")

Q = Diagonal(sqrt(dt) * ones(5))
# Definimos las estructuras necesarias para crear un `SimpleKalmanIterator`.
nlupdater = NLUpdater(rk,F, Q, x0, a₀, (x) -> state_integrity(x, sum(x0)))
#nlaugmented = KalmanFilter.NLUpdaterUnknowInput(nlupdater, control_pieces, 1.)
#observer = KalmanFilter.LinearObserver(tildeH, zeros(1), G, tildex0)
observer = KalmanFilter.LinearObserver(H, zeros(1), G, observation_integrity)
system = KalmanFilter.InnerState(x0)
P0 = F(x0) * Q * F(x0)' 
isposdef(P0) # debería ser true
iterator = KalmanFilter.SimpleKalmanIterator(x0, P0, nlupdater, observer, system, dt)

# Y realizamos un total de `N` iteraciones, guardando los estamos intermedios
# en las variables que aparecen abajo.
results, ensamble = KalmanFilter.full_iteration(iterator, dt, N, control_pieces, 1)

# ## Resultados
# Graficaremos los estados internos considerados, y los resultados obtenidos.

# Ahora podemos graficar los diferentes estados de nuestro sistema, así como las
# aproximaciones obtenidas con filtro de Kalman.

using Plots

# Susceptibles ``S``
plot(results, ts, 5)


# Expuestos ``E``
plot(results, ts, 2)


# Infectados asintomáticos *mild* ``I^m``
plot(results, ts, 3)

# Infectados ``I``
plot(results, ts, 4)

# Infectados acumulados ``cI``, y las observaciones que hicimos de él.
plot(results, ts, 5)



# Notamos que tras una cierta cantidad de tiempo es posible averiguar el control
# Aunque hay bastante incerteza de los resultados.
