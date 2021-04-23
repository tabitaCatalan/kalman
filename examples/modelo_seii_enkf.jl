# # Filtro de Kalman por ensambles
using Random, Distributions
using KalmanFilter
using Plots


include("seii_model.jl")


"""
Permite generar una muestra de condiciones iniciales ``x_{0}^{(1)}, \\dots, x_{0}^{(N)} ``

Genera una muestra a partir de un vector ``x0 = (S_0, E_0, I^m_0, I_0, cI_0)``. Los
elementos de la muestra suman la misma cantidad de personas. La muestra se obtiene
generando valores al azar que distribuyen ``\\mathcal{N}(E_0, \\beta E_0)``.
- `x0`: ``x_0``
- `factor`: ``\\beta``
- `N`: Cantidad de muestras ``N``.
"""
function get_initial_states(x0, factor, N, a0,unknowninput = false)
    total = sum(x0)
    expuestos_base = x0[2]
    expuestos = rand(Normal(expuestos_base, expuestos_base * factor), N)
    susceptibles = total .- expuestos
    if unknowninput
        states = [[susceptibles[i], expuestos[i], 0., 0., 0., rand(Normal(a0, factor * a₀ ))] for i in 1:N]
    else
        states = [[susceptibles[i], expuestos[i], 0., 0., 0.] for i in 1:N]
    end
    states
end

# Definimos un vector de condiciones iniciales ``x_0``.

x0 = [7.112808e6, 1046.8508799517147, 0.0, 521.963080420307, 0.0]
F = 50000. * ones(5)

# Solo consideraremos conocida la cantidad total de infectados ``\mathbf{x}_5 = cI``.
H = [0. 0. 0. 0. 1.]

# Usaremos un error de unas 50.000 personas en las mediciones.
G = [5e6]
dimensions = 5

# Definimos las matrices ``\tilde{H}`` para nuestro sistema auxiliar, que incluirá al input.
tildeH = [H 0.]

tildeP = [F * F' zeros(5); zeros(5)' 1.]

dt = 1.
T = 50.
N = Int(T/dt)

# Definimos los parámetros
γₑ = 0.14
γᵢ = 1/14
φ = 0.3

# Que agrupamos en una tupla `p`.
p = (gammae = γₑ, gammai = γᵢ, phi = φ)


a₀ = 1.; Fₐ = 1.5

# EL nuevo vector de estados será ``\tilde{\mathbf{x}} = \begin{pmatrix} \mathbf{x}_0 \\ 1. \end{pmatrix}``,
# donde estamos considerando una suposición inicial para el estado $\alpha = 1$.

tildex0 = [x0; a₀]


rkx = KalmanFilter.RK4Dx(episystem_full, epijacobian_full_x, p, dt)
rk = KalmanFilter.RK4Du(rkx, epijacobian_full_u)

nlupdater = NLUpdater(rk,F,x0,a₀)


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


#nlaugmented = KalmanFilter.NLUpdaterUnknowInput(nlupdater, control_pieces, Fₐ)
observer = KalmanFilter.LinearObserver(H, zeros(1), G)
#observer = KalmanFilter.LinearObserver(tildeH, zeros(1), G, tildex0)

ensemble_size = 50
states = get_initial_states(x0, 1.5, ensemble_size, a₀, false)
system = KalmanFilter.InnerState(x0)
enkf = KalmanFilter.EnKF(states, nlupdater, observer, system, dt)

results, ensamble = KalmanFilter.full_iteration(enkf, dt, N, control_pieces, ensemble_size)
#Juno.@enter KalmanFilter.full_iteration(enkf, dt, N, control_pieces)

# Susceptibles ``S``
plot(ensamble, ts, 1, ylims = (0, 7.5e6))
plot!(results, ts, 1)


# Expuestos ``E``
plot(ensamble, ts, 2)
plot!(results, ts, 2, ylims = (0, 2.8e6))

# Infectados asintomáticos *mild* ``I^m``
plot(ensamble, ts, 3)
plot!(results, ts, 3)

# Infectados ``I``
plot(ensamble, ts, 4)
plot!(results, ts, 4)

# Infectados acumulados ``cI``, y las observaciones que hicimos de él.
plot(ensamble, ts, 5)
plot!(results, ts, 5)


# Control
#=
plot(ts, control_pieces.(ts), label = "Control real")
plot!(results, ts, 6)
plot!(ensamble, ts, 6)=#