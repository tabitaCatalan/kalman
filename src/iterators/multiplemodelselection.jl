#=
Necesito traquear varios filtros simultáneamente, y una probabilidad para cada uno.
- EL sistema si inicializa con probabilidades a priori para cada set de parámetros.
La idea es que a partide un set de parámetros puedo calcular una iteración del filtro. 
- Discretizer común que pueda usar con varios parámetros. Cada filtro lo usa.
- Un updater único que pueda usar un discretizer general. Similar a NLUpdater, pero 
    pero no puede guardar el estado, necesita calcularlo para cada uno.
- número de iteración o tiempo 
Qué información necesita cada filtro? 
    - Parámetros varios de estos se necesitan en el discretizer.
        Creo que necesito un discretizer que no guarde los parámetros, 
        no tienen sentido tener un montón de funciones iguales para calcular lo mismo. 
        Así que este filtro tiene dinámica única, y parámetros variable. 
        Cada filtro sabe cómo usar sus parámetros para generar una estimación. 
    - análisis y forecast de estado y covarianza 
        hatX::ComponentArray
        next_hatX::ComponentArray
Una forma de usar las probabilidades para mezclar los estados y todo eso. 
Una forma de actualizar las probabilidades. 
=# 

using KalmanFilter
using StaticArrays
using LinearAlgebra: Diagonal, I
using Distributions: MvNormal, Normal

# No es exactamente un iterador, no tiene updater ni nada 
struct SimpleKalmanEstimation{P,CA1 <: ComponentArray, CA2 <: ComponentArray} 
    """Parameters. Have 2 components: filter_p (filter) and p (dynamics)."""
    params::P
    """State and covariance estimation (analysis)"""
    hatX::CA1
    """State and covarianza (forecast)"""
    next_hatX::CA2 
end 
# Versiones generales los discretizadores comunes RK4, Euler, etc
# GeneralDiscretizer(x, a, p, dt) # tiene la misma idea de discretizer pero es más general. Creo que voy a tener que generalizar los que tengo para que funcione.

hatx(estimation::SimpleKalmanEstimation) = estimation.hatX.x
hatP(estimation::SimpleKalmanEstimation) = estimation.hatX.P

next_hatx(estimation::SimpleKalmanEstimation) = estimation.next_hatX.x
next_hatP(estimation::SimpleKalmanEstimation) = estimation.next_hatX.P

function set_hatX!(estimator::SimpleKalmanEstimation, hatx, hatP) 
    estimator.hatX.x = hatx
    estimator.hatX.P = hatP
end 

function set_next_hatX!(estimator::SimpleKalmanEstimation, hatX::ComponentArray) 
    estimator.next_hatX.x = hatX.x
    estimator.next_hatX.P = hatX.P
end 

dynamic_params(estimation::SimpleKalmanEstimation) = estimation.params.p
filter_params(estimation::SimpleKalmanEstimation) = estimation.params.filter_p

# make_lowpassalpha(filter_p) esta función debe estar en estimation,
# o debe recibirse o estar definida en algun lado 
make_lowpassalpha(estimator::SimpleKalmanEstimation) = make_lowpassalpha(filter_params(estimator))

function set_hatX_with_lowpass!(estimator::SimpleKalmanEstimation, hatx, hatP)
    newhatx = KalmanFilter.lowpass(hatx(estimator), hatx, make_lowpassalpha(estimator))
    set_hatX!(estimator, newhatx, hatP)
end 

#========================================================
MultipleModelKalman:
Contiene toda la información de las distintas estimaciones, 
los parámetros que usa cada una y cómo mezclarlas para
hacer una estimación.
========================================================#

"""
$(TYPEDEF)

Contains all the necessary information to run `N` kalman filter estimations, 
each with different parameters, and combine the estimations in a bayesian way.

``\\hat{x}_{kj}`` is the ``j``-th estimation at ``k``-th iteration. Uses ``p_j`` 
parameters. This parameters can be of the dynamics (e.g. rates of transmision 
in an epidemiological model) or of the filter (initial convariance, noise matrix, 
etc). Every iteration updater the priors and the estimations when receiving new 
observations ``y_k``.
"""
struct MultipleModelKalman{T <: AbstractFloat,
                           Pr <:AbstractVector{T}, 
                           Ms <: AbstractVector{M} where M <: SimpleKalmanEstimation,
                           CU <: CommonUpdater,
                           CO <: CommonObserver,
                           Sys <: KalmanFilter.Measurements} # <: KalmanIterator
    """`prior[j]` correspond to ``\\mathbb{P}(p_{j}| y_{k})``. Inicialization 
    ``\\mathbb{P}(p_{j}| y_{0})`` correspond to an *a priori* probability, setted before
    seing any observation."""
    priors::Pr 
    """Vector of `SimpleKalmanEstimation`s."""
    models::Ms 
    """A `CommonUpdater`. The same is used when updating and forecasting every estimation, 
    so it needs to receive the parameters."""
    updater::CU
    """A `CommonObserver`. The same is used when observing every estimation, 
    so it needs to receive the parameters."""
    observer::CO
    """A `Measurements` element with all the observations."""
    system::Sys
    """`k` correspond to the ``k``-th iteration."""
    k::int 
end 

#===== Getters y setters =====#
how_many_models(mmkf::MultipleModelKalmanFilter) = length(mmkf.priors)
enumerate_models(mmkf::MultipleModelKalman) = 1:how_many_models(mmkf)

get_prior(mmkf::MultipleModelKalman, model_index) = mmkf.priors[model_index]
get_model(mmkf::MultipleModelKalman, model_index) = mmkf.models[model_index]


function set_hatX_with_lowpass!(mmkf::MultipleModelKalman, model_index, hatx, hatP)
    set_hatX_with_lowpass!(get_model(mmkf, model_index), hatx, hatP)
end 

next_hatx(mmkf::MultipleModelKalman) = mix_estimation(mmkf, next_hatx)
next_hatP(mmkf::MultipleModelKalman) = mix_estimation(mmkf, next_hatP)
hatx(mmkf::MultipleModelKalman) = mix_estimation(mmkf, hatx)
hatP(mmkf::MultipleModelKalman) = mix_estimation(mmkf, hatP)

tn(iterator) = iterator.k * dt(iterator.updater)


"""
- `estimator::Function`: que actúa en `SimpleKalmanEstimation`
"""
function mix_estimation(mmkf::MultipleModelKalman, estimator::Function)
    sum(get_prior(mmkf,i) * estimator(get_model(mmkf, i)) for i in enumerate_models(mmkf))
end 

function update_priors!(mmkf::MultipleModelKalman, new_priors)
    if length(new_priors) == how_many_models(mmkf)
        mmkf.priors .= new_priors
    else 
        print("Trying to assign incorrect ammount of priors to MultipleModelKalman")
    end
end 

gaussian_pdf(x, P) 

function probability_observation_given_p(
    observer::KalmanFilter.LinearizableObserver,
    estimation::SimpleKalmanEstimation,
    observation)
    r = observation - KalmanFilter.observe_without_error(observer, hatx(estimation))
    S = Hn(observer) * hatP(estimation) * Hn(observer)' + Rn(observer)
    gaussian_pdf(r, S)
end 

function probability_observation_given_p(
    mmkf::MultipleModelKalman,
    observation, model)
    probability_observation_given_p(mmkf.observer, mmkf.model[model], observation)
end 

function probability_p_given_observation(mmkf::MultipleModelKalman, observation)
    auxvec = [probability_observation_given_p(mmkf, observation, model) * get_prior(mmfk, model) for model in enumerate_models(mmkf)]
    auxvec ./ sum(auxvec)
end 

"""
- `method`: las opciones son `:maxprob`, `:weighted`
"""
function parameter_estimation(mmkf::MultipleModelKalman, method)
    if method == :weighted 
        sum(get_prior(mmkf, i) * mmkf.model[i].p for i in enumerate_models(mmkf))
    elseif method == :maxprob 
        maxval, maxindx = findmax(get_prior(mmkf, i) for i in enumerate_models(mmkf))
        mmkf.model[maxindx].p
    end 
end 
#=
function forecast(iterator::SimpleKalmanIterator, control)
    forecast(iterator.updater, hatx(iterator), hatP(iterator), control, tn(iterator))
end
=#

#===================================================
KalmanIterator interface 
===================================================#
function update_inner_state!(mmkf::MultipleModelKalman, control)

end 

function forecast_observed_state!(mmkf::MultipleModelKalman, control)
    # si guardara una estimación de next_hatX este seria el momento de actualizarla,
    # pero no lo hago, solo actualizo las de los modelos internos
    updater = mmkf.updater 
    for model in enumerate_models(mmkf)
        forecast_observed_state!(updater, get_model(mmkf, model), tn(mmkf))
    end 
    # update priors 
end

function observe_inner_system(mmkf::MultipleModelKalman)
    observe_real_state(mmkf.system)
end   

function analyse!(mmkf::MultipleModelKalman, yₙ₊₁)
    update_priors!(mmkf,  probability_p_given_observation(mmkf, yₙ₊₁))
    for model_index in enumerate_models(mmkf)
        hatPₙ₊₁ₙ₊₁ = analyse_hatP(mmkf, model_index)
        hatxₙ₊₁ₙ₊₁ = analyse_hatx(mmkf, model_index, yₙ₊₁)
        set_hatX_with_lowpass!(mmkf, model_index, hatxₙ₊₁ₙ₊₁, hatPₙ₊₁ₙ₊₁)
    end
end 

function update_updater!(mmkf::MultipleModelKalman) end 

function advance_counter!(mmkf::MultipleModelKalman) mmkf.n += 1 end 

analyse_hatP(mmkf::MultipleModelKalman, model_index) = analyse_hatP(mmkf.observer, get_model(mmkf, model_index))
analyse_hatx(mmkf::MultipleModelKalman, model_index, observation) = analyse_hatx(mmkf.observer, get_model(mmkf, model_index), observation)



#=
Qué necesita un Updater 
- necesita un discretizer general 
- necesita recibir el parametro 
=#

abstract type GeneralDiscretizer <: KalmanFilter.Discretizer end

function (::GeneralDiscretizer)(x,α,p,t) error("No se ha definido un método para este discretizador.") end
function jacobian_x(ds::GeneralDiscretizer, x, α, p, t) error("No se ha definido método jacobian_x para este discretizador.")  end
function dt(ds::GeneralDiscretizer) error("No se ha definido método dt para este discretizador.") end

"""No almacena parámmetros, así que debe dársele uno al pedir que discretize."""
struct GeneralRK4{F1 <: Function, F2 <: Function, T} <: GeneralDiscretizer
    """
    La función tal que ``x' = f(x,\\alpha, p, t)``, donde ``x`` es el estado,
    ``\\alpha `` un control, ``p`` son parámetros extra y ``t`` es el tiempo.
    """
    f::F1
    """``D_x f (x,\\alpha, p, t)``, el diferencial de ``f`` con respecto a ``x``."""
    Dxf::F2
    """Tamaño del paso temporal ``\\Delta t``, tal que ``t_{n+1} = t_n + \\Delta t``."""
    dt::T
  end

(rk::GeneralRK4)(x, α, p, t) = KalmanFilter.rungekutta_predict(rk.f, x, α, p, t, rk.dt)
jacobian_x(rk::GeneralRK4, x, α, p, t) = KalmanFilter.rungekutta_jacobian_x(rk.f, rk.Dxf, x, α, p, t, rk.dt)


abstract type KalmanUpdaterWithGeneralDiscretizer end

#= necesito hacer algo con Updater,
tal vez tenga que hacer algo más general.
Este updater será diferente; Mn por ejemplo no se puede calcular sin 
el parámetro p. Puede que incluso necesite recibir el tiempo, ya que 
Updater no lo maneja internamente y no tendrá la versión linealizada.
También va a necesitar una función de integridad más que seguro 
Mn(updater::KalmanUpdaterWithGeneralDiscretizer, x, α, p, t)
=#

#indepnoisematrix(dt, dims) = Diagonal(sqrt(dt) * SVector{dims}(ones(dims)))
indepnoisematrix(dt) = sqrt(dt) * I


Mn(updater::CommonUpdater, x, p, t) = jacobian_x(updater.discretizer, x, 0., p, t) 
#Bn(updater::CommonUpdater, x, α, params, t) = jacobian_x(updater.discretizer, x, α, p, t) 
Fn(updater::CommonUpdater, filter_p) = make_F(updater)(filter_p) 
Qn(updater::CommonUpdater) = indepnoisematrix(dt(updater))



"""
Actualizador de la forma 
```math
x_{n+1} = \\mathcal{M}(x_n, p, t) + F w_n 
```
donde ``\\mathcal{M}`` es una dinámica posiblemente no lineal discretizada, ``p`` son parámetros 
de la dinámica, ``t`` es tiempo, ``F`` una matriz constante de dispersión de ruido y 
``w_n \\sim \\mathcal{N}(0, Q_n)``. Para simplificar, suponemos que ``Q_n = \\sqrt{\\Delta t} I``, 
con ``I`` la matriz identidad.
# Argumentos 
- `makeF`: que recibe un argumento `filter_p` y retorna la matriz ``F`` de dispersión de ruido del modelo.
- `discretizer`: guarda la dinámica de un problema continuo, discretizada por medio de algún método 
numérico como Euler Progresivo, RungeKutta de orden 4, etc. 
- `make_F`: función que recibe un argumento `filter_p` y produce una matriz de ruido.
- `integrity`: luego de linealizar, se usa para mantener los estados en valores razonables. Es una función 
de los estados ``x``.
# Comentarios
La diferencia con un `LinearizableUpdater` está en los parámetros, tanto de la dinámica como del filtro de 
Kalman en sí. `CommonUpdater` está pensado como un actualizador para ser usado con varios filtros distintos, 
cambiando ya sea los parámetros de la dinámica, o la matriz de dispersión de ruido del modelo del filtro de
Kalman. Un `LinearizableUpdater`, en cambio, funciona con set de parámetros fijos. Es por ese mismo motivo
que `CommonUpdater` no usa los `Discretizer`s normales, sino que usa `GeneralDiscretizer`.

"""
struct CommonUpdater{D <: GeneralDiscretizer, F1 <: Function, F2 <: Function} <: KalmanUpdaterWithGeneralDiscretizer
    dims::Int
    discretizer::D
    makeF::F1
    integrity::F2
end 

make_F(updater::CommonUpdater) = updater.makeF
dimensions(updater::CommonUpdater) = updater.dims 
dt(updater::CommonUpdater) = dt(updater.discretizer)

make_noise(updater::CommonUpdater, filter_p) = Fn(updater, filter_p) * rand(Normal(0., sqrt(dt(updater))), dimensions(updater))

function update_approximation(updater::CommonUpdater, hatx, p, t, filter_p)
    updater.discretizer(hatx, 0., p, t) + make_noise(updater, filter_p)
end 

#==================================================
Updater and estimation interactions 
==================================================# 


function forecast_state(updater::CommonUpdater, estimation::SimpleKalmanEstimation, t)
    update_approximation(updater, hatx(estimation), dynamic_params(estimation), t, filter_params(estimation))
end 

function forecast_hatP(updater::CommonUpdater, estimation::SimpleKalmanEstimation, t)
    M = Mn(updater, hatx(estimation), dynamic_params(estimation), t)
    F = Fn(updater, filter_params(estimation))
    M * hatP(estimation) * M' + F * Qn(updater) * F' 
end 

function forecast(updater::CommonUpdater, estimation::SimpleKalmanEstimation, t)
    hatPnp1 = forecast_hatP(updater, estimation, t)
    hatxnp1 = forecast_state(updater, estimation, t)
    ComponentArray(x = hatxnp1, P = hatPnp1)
end

function forecast_observed_state!(updater::CommonUpdater, estimation::SimpleKalmanEstimation, t)
    Xₙ₊₁ₙ = forecast(updater, estimation, t)
    set_next_hatX!(estimation, Xₙ₊₁ₙ)
end  


#==================================================
 General Observer 
==================================================#

"""
Representa un observador lineal de la forma 
"""
struct CommonObserver{T <: AbstractFloat,M <: AbstractArray{T, 2} D <: GeneralDiscretizer, F1 <: Function, F2 <: Function} 
    """Observation matrix"""
    H::M
    discretizer::D
    """Función que recibe un argumento `filter_p` y produce una matriz de ruido."""
    makeG::F1
    integrity::F2
    dt::T
end 
make_G(observer::CommonObserver) = observer.makeG
#indepnoisematrix(dt) = sqrt(dt) * I

Hn(observer::CommonObserver) = observer.H 
#Dn(observer::CommonObserver, x, α, params, t) = jacobian_x(updater.discretizer, x, α, p, t) 
Gn(observer::CommonObserver, filter_p) = make_G(observer)(filter_p) 
Rn(observer::CommonObserver) = indepnoisematrix(observer.dt)

state_dimension(observer::CommonObserver) = size(Hn(observer))[2]
observation_dimension(observer::CommonObserver) = size(Hn(observer))[1]

#make_noise(observer, filter_p) = Gn(observer, filter_p) * rand(zeros(state_dimension(observer)), Rn(observer))
make_noise(observer::CommonObserver, filter_p) = Gn(observer, filter_p) * rand(Normal(0., sqrt(observer.dt)), state_dimension(observer))

function observe_without_error(observer::CommonObserver, x)
    Hn(observer) * x 
end 

function observe_with_error(observer::CommonObserver, x, filter_p)
    observe_without_error(observer, x) + make_noise(observer, filter_p)
end 

En(observer::CommonObserver, hatPnp1n, filter_p) = Hn(observer) * hatPnp1n * Hn(observer)' + Gn(observer, filter_p) * Rn(observer) * Gn(observer, filter_p)' 
KalmanGain(observer::CommonObserver, hatPnp1n, filter_p) = hatPnp1n * Hn(observer)' * inv(En(observer, hatPnp1n, filter_p))

#==================================================
Observer and estimation interactions 
==================================================# 

function observe_with_error(observer::CommonObserver, estimation::SimpleKalmanEstimation)
    observe_with_error(observer, hatx(estimation), filter_params(estimation))
end 

function observe_forecasted_system(observer::CommonObserver, estimation::SimpleKalmanEstimation)
    observe_without_error(observer, next_hatx(estimation))
end

function KalmanGain(observer::CommonObserver, estimation::SimpleKalmanEstimation)
    KalmanGain(observer, next_hatP(estimation), filter_params(estimation)) 
end

function analyse_hatP(observer::CommonObserver, estimation::SimpleKalmanEstimation)
    K = KalmanGain(observer, estimation)
    (I - K * Hn(observer)) * next_hatP(estimation) * (I - K * Hn(iterator))'
        + K * Gn(observer, filter_p) * Rn(observer) * Gn(observer, filter_p)' * K'
end 

function analyse_hatx(observer::CommonObserver, estimation::SimpleKalmanEstimation, observation)
    next_hatx(estimation) + KalmanGain(observer, estimation) * (observation - observe_forecasted_system(observer, estimation))
end


#=
Tengo dos tipos distintos de parámetros 
Los que son parte de la dinámica `p``: γₑ, γᵢ, ect 
Y los que son propios del filtro `filter_p`; lowpass_alpha, max_values, etc. 
Necesito optimizar sobre ambos tipos...
Podrían ser distintos para cada filtro! Cada uno corre sus propias covarianzas y sus propios modelos.
Y así hay que encontrar sentido en medio de toda es confusión xDDD 
Es un desastre.

Updater necesita saber construir un Rn, Bn, etc, a partir de los parámetros del filtro.
Voy a dejar de lado Bn, no lo voy a usar (== 0 siempre). 
Supondré que Rn es dado (de hecho, con saber el dt debería suponer ruido independiente sqrt(dt)
y el dt viene del discretizador.)

Fn sí que depende de filter_p. Necesito una función tipo: 
make_F(filter_p). Esa función debe estar dentro de updater.
=#

#=
Observer tiene una matriz de observación H fija para cada filtro de kalman 
El ruido de observación depende de filter_p
=#