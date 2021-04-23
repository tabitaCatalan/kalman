# # Filtro de Kalman Extendido con input desconocido
using Random
using KalmanFilter
using Plots

include("seii_model.jl")

# Definimos un vector de condiciones iniciales ``x_0``.

x0 = [7.112808e6, 1046.8508799517147, 0.0, 521.963080420307, 0.0]
F = 100. * ones(5)

# Solo consideraremos conocida la cantidad total de infectados ``\mathbf{x}_5 = cI``.
H = [0. 0. 0. 0. 1.]

# Usaremos un error de unas 50.000 personas en las mediciones.
G = [500.]
dimensions = 5

# Definimos las matrices ``\tilde{H}`` para nuestro sistema auxiliar, que incluirá al input.
tildeH = [H 0.]

tildeP = [F * F' zeros(5); zeros(5)' 1.]

# EL nuevo vector de estados será ``\tilde{\mathbf{x}} = \begin{pmatrix} \mathbf{x}_0 \\ 1. \end{pmatrix}``,
# donde estamos considerando una suposición inicial para el estado $\alpha = 1$.
tildex0 = [x0; 0.4]

dt = 0.05
T = 30.
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


# Veamos un gráfico del control usado, para lo que definimos un vector con los tiempos `ts`.
ts = 0.0:dt:(T-dt)
plot(ts, control_pieces.(ts), label = "Control real")

# Definimos las estructuras necesarias para crear un `LinearKalmanIterator`.
nlupdater = NLUpdater(rk,F,x0,0.4)
nlaugmented = KalmanFilter.NLUpdaterUnknowInput(nlupdater, control_pieces, 1.)
observer = KalmanFilter.LinearObserver(tildeH, zeros(1), G)
system = KalmanFilter.InnerState(tildex0)
iterator = KalmanFilter.LinearKalmanIterator(tildex0, tildeP, nlaugmented, observer, system, dt)

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
