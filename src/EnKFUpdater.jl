# Ensemble Kalman Filter Updater
using Statistics, Distributions, Random, LinearAlgebra


"""
`KalmanIterator` que permite aproximar un estado mediante la técnica Ensemble
Kalman Filter.

$(TYPEDFIELDS)
"""
mutable struct EnKF <: KalmanFilter.KalmanIterator
    """Número de iteración actual ``n``. Se inicializa en 0."""
    n::Int
    """Tamaño del ensamble ``N``, cantidad de estimaciones que se trabajarán."""
    N::Int
    """Control ``u_{n-1}`` usado para llegar al estado actual"""
    u::Real
    """Sistema"""
    system::ObservableSystem
    """Array de aproximaciones ``\\{ \\hat{x}^{(i)}_{n,n} \\}_i`` del estado real ``x_n``."""
    states_hatx # debería ser un vector de estados
    """
    Array de estimaciones ``\\{ \\hat{x}^{(i)}_{n,n-1} \\}_i`` del estado real ``x_n``,
    antes de conocer la observación ``y_n`` (solo contando con información
    hasta ``y_{n-1}``).
    """
    states_next_hatx
    """Matriz de ganancia de Kalman ``K_n``"""
    kalman_gain::AbstractArray
    """
    `KalmanUpdater` que permite actualizar tanto el estado interno como las
    aproximaciones. No es necesario usar un updater linealizable, aunque sea NL.
    """
    updater::KalmanUpdater
    """
    Un observador lineal que entrega observaciones ``y_n`` del estado real.
    """
    observer::LinearObserver
    """Una distribución que permite agregar ruido al sistema. Por defecto es
    una normal ``\\mathcal{N}(0,1)``."""
    noiser::UnivariateDistribution
end


function EnKF(states, updater, observer::KalmanFilter.KalmanObserver, system::ObservableSystem, dt = 1.)
    N = length(states)
    kf, kc = KalmanFilter.kalman_size(observer)
    K_initialize = Array{Float64,2}(undef, kf, kc)
    EnKF(0, N, 1., system, copy(states), similar(states), K_initialize, updater, observer, Normal(0.,sqrt(dt)))
end

function update_inner_state!(enkf::EnKF, control)
    update_real_state!(enkf.system, enkf.updater, control)
    enkf.u = control
end

# Debería haber una función inflated cov que use un parametro a (=10) 
# en este caso, que sea guardado en enkf.
mean_cov_from_sample(sample) = mean(sample), inflated_cov(sample)
inflated_cov(sample, a::Number) = cov(sample) + a^2 * I # a = 1e4
inflated_cov(sample, a) = cov(sample) + Diagonal(a)
inflated_cov(sample) = inflated_cov(sample, [1e5, 1e2, 1e2, 1e2, 1e2, 1e2, 0.15])

hatx(enkf::EnKF) = mean(enkf.states_hatx)
hatP(enkf::EnKF) = inflated_cov(enkf.states_hatx)

function forecast_hatX(enkf::EnKF, control)
    hatP = inflated_cov(enkf.states_hatx)
    [forecast_with_error(enkf.updater, enkf.states_hatx[i], hatP, control).x for i in 1:enkf.N]
end

function forecast_observed_state!(enkf::EnKF, control)
    enkf.states_next_hatx .= forecast_hatX(enkf, control)
    hatPₙ₊₁ₙ = inflated_cov(enkf.states_next_hatx)
    enkf.kalman_gain .= KalmanFilter.KalmanGain(enkf, hatPₙ₊₁ₙ)
end

function observe_inner_system(enkf::EnKF)
    observe_real_state(enkf.system, enkf.observer, enkf.u, rand(enkf.noiser))
end

function observe_forecasted_system(enkf::EnKF)
  enkf.observer.(enkf.states_next_hatx, enkf.u, 0.)
end

function analyse!(enkf::EnKF, observation)
    for (index, observed_state) in enumerate(observe_forecasted_system(enkf))
        enkf.states_hatx[index] = enkf.states_next_hatx[index] + enkf.kalman_gain * (observation .- observed_state)
    end
end

function update_updater!(enkf::EnKF)
    update!(enkf.updater, hatx(enkf), hatP(enkf), enkf.u)
end



forecast(enkf::EnKF, control) = mean_cov_from_sample(forecast_hatX(enkf, control))

#===#


struct EnsamblesStoring
    ensambles::Array{Float64, 3}
    """
    # Argumentos 
    - `N`: número de ensambles en cada paso 
    - `dimensions`: dimensión de cada ensamble
    - `Nt`: número de pasos temporales a guardar 
    """
    function EnsamblesStoring(N, dimensions, Nt)
        new(Array{Float64, 3}(undef,N, dimensions, Nt))
    end
end

function add_ensamble!(ensemble::EnsamblesStoring, i, iterator::KalmanIterator) end

function add_ensamble!(ensemble::EnsamblesStoring, n, enkf::EnKF)
    for (i,member) in enumerate(enkf.states_hatx)
        ensemble.ensambles[i,:,n] .= member
    end
end

using RecipesBase

@recipe function f(en::EnsamblesStoring, ts, index, rango = 1:length(ts))
    i = index
    titles = ["Susceptibles", "Expuestos", "Infectados mild", "Infectados", "Recuperados", "Infectados acumulados", "Control"]
    title --> titles[i]
    xguide --> "Tiempos t (días)"
    yguide --> "Personas"
    @series begin
        primary := false
        seriestype := :path
        label --> :none
        linewidth --> 0.1
        seriescolor := :gray
        seriesalpha := 0.8
        ts[rango], en.ensambles[:,i,:]'
    end
end
