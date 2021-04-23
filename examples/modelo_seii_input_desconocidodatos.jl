# # Filtro de Kalman Extendido con input desconocido

using KalmanFilter
using Plots
using CSV

total_RM = CSV.File("..\\data\\totales_RM.csv").Metropolitana

include("..\\seii_model.jl")

# Definimos un vector de condiciones iniciales ``x_0``.

x0 = [7.112808e6, 10, 0.0, 0.0, 0.0]
F = 100. * ones(5)

# Solo consideraremos conocida la cantidad total de infectados ``\mathbf{x}_5 = cI``.
H = [0. 0. 0. 0. 1.]

# Usaremos un error de unas 50.000 personas en las mediciones.
G = [5.]
dimensions = 5

# Definimos las matrices ``\tilde{H}`` para nuestro sistema auxiliar, que incluirá al input.
tildeH = [H 0.]

tildeP = [F * F' zeros(5); zeros(5)' 1.]

# EL nuevo vector de estados será ``\tilde{\mathbf{x}} = \begin{pmatrix} \mathbf{x}_0 \\ 1. \end{pmatrix}``,
# donde estamos considerando una suposición inicial para el estado $\alpha = 1$.
a₀ = 1.; Fₐ = 1.5
tildex0 = [x0; a₀]


dt = 1.
T = 400.
N = Int(T/dt)

# Definimos los parámetros
γₑ = 0.14
γᵢ = 1/14
φ = 0.5

# Que agrupamos en una tupla `p`.
p = (gammae = γₑ, gammai = γᵢ, phi = φ)

# El control para el sistema interno (que supondremos desconocido y es el que intentaremos
# averiguar) será la función constante por pedazos `control_pieces`.
#=
function control_pieces(t)
    if t < 5.
        1.
    elseif t >= 5. && t < 12.
        0.8
    elseif t >= 12. && t < 25.
        0.5
    elseif t >= 25. && t < 40.
        1.5
    elseif t >= 40.
        1.
    end
end
=#

function state_integrity(x, total)
    x = x * total/sum(x)
    x = max.(x, 0.)
    x
end


observation_integrity(x) = max.(x,0.)

# Veamos un gráfico del control usado, para lo que definimos un vector con los tiempos `ts`.
ts = 0.0:dt:(T-dt)
#plot(ts, control_pieces.(ts), label = "Control real")

# Definimos las estructuras necesarias para crear un `LinearKalmanIterator`.


episystem_a(x, α, p) = [episystem_full(x,α,p); 0.]
epijacobian_a_x(x, α, p) = [epijacobian_full_x(x, α, p) epijacobian_full_u(x,α,p); zeros(6)']


#nlaugmented = KalmanFilter.NLUpdaterUnknowInput(nlupdater, control_pieces, 1.)
#==================================# 
updater = KalmanFilter.ODEForecaster(dt, 10, tildex0, [F; Fₐ], episystem_a, epijacobian_a_x, p, (x) -> state_integrity(x, sum(x0)))
observer = KalmanFilter.LinearObserver(tildeH, zeros(1), G, observation_integrity)
#system = KalmanFilter.InnerState(tildex0)
system = KalmanFilter.Measurements(total_RM, 1.)
iterator = KalmanFilter.LinearKalmanIterator(tildex0, tildeP, updater, observer, system, dt)
#==================================# 
# Y realizamos un total de `N` iteraciones, guardando los estamos intermedios
# en las variables que aparecen abajo.
results, ensamble = KalmanFilter.full_iteration(iterator, dt, N, t -> 0., 1)
#results2, ensamble = KalmanFilter.full_iteration(iterator2, dt, N, t -> 0., 1)
isposdef(iterator.hatX.hatP)
results
# ## Resultados
# Graficaremos los estados internos considerados, y los resultados obtenidos.

# Ahora podemos graficar los diferentes estados de nuestro sistema, así como las
# aproximaciones obtenidas con filtro de Kalman.

# Susceptibles ``S``
a = plot(results, ts, 1)
#plot!(results2, ts, 1)
display(a)
# Expuestos ``E``
a = plot(results, ts, 2)
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
plot(results, ts, 6)

# Notamos que tras una cierta cantidad de tiempo es posible averiguar el control
# Aunque hay bastante incerteza de los resultados.
