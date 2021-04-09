# Ensemble Kalman Filter Updater
using Statistics, Distributions, Random


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
    updater::KalmanFilter.KalmanUpdater
    """
    Un observador lineal que entrega observaciones ``y_n`` del estado real.
    """
    observer::KalmanFilter.LinearObserver
    """Una distribución que permite agregar ruido al sistema. Por defecto es
    una normal ``\\mathcal{N}(0,1)``."""
    noiser::UnivariateDistribution
end


function EnKF(states, updater, observer::KalmanFilter.KalmanObserver)
    N = length(states)
    kf, kc = KalmanFilter.kalman_size(observer)
    K_initialize = Array{Float64,2}(undef, kf, kc)
    EnKF(0, N, 1., states, similar(states), K_initialize, updater, observer, Normal(0,1))
end

function update_inner_state!(enkf::EnKF, control)
    noise = rand(enkf.noiser)
    update_real_state!(enkf.observer, enkf.updater, control, noise)
    enkf.u = control
end

mean_cov_from_sample(sample) = mean(sample), cov(sample)

function forecast_hatX(enkf::EnKF, control)
    [enkf.updater(enkf.states_hatx[i], control, rand(enkf.noiser)) for i in 1:enkf.N]
end

function forecast_observed_state!(enkf::EnKF, control)
    enkf.states_next_hatx .= forecast_hatX(enkf, control)
    hatxₙ₊₁ₙ, hatPₙ₊₁ₙ = mean_cov_from_sample(enkf.states_next_hatx)
    enkf.kalman_gain .= KalmanFilter.KalmanGain(enkf, hatPₙ₊₁ₙ)
end

observe_inner_system(enkf::EnKF) = observe_real_state(enkf.observer, enkf.u, rand(enkf.noiser))

function observe_forecasted_system(enkf::EnKF)
  enkf.observer.(enkf.states_next_hatx, enkf.u, 0.)
end

function analyse!(enkf::EnKF, observation)
    for (index, observed_state) in enumerate(observe_forecasted_system(enkf))
        enkf.states_hatx[index] = enkf.states_next_hatx[index] + enkf.kalman_gain * (observation .- observed_state)
    end
end

function update_updater!(enkf::EnKF) end

hatx(enkf::EnKF) = mean(enkf.states_hatx)
hatP(enkf::EnKF) = cov(enkf.states_hatx)

forecast(enkf::EnKF, control) = mean_cov_from_sample(forecast_hatX(enkf, control))
