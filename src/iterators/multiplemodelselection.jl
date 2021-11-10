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

# No es exactamente un iterador, no tiene updater ni nada 
struct SimpleKalmanEstimation{P,CA1 <: ComponentArray, CA2 <: ComponentArray} 
    """Parameters"""
    p::P
    """State and covariance estimation (analysis)"""
    hatX::CA1
    """State and covarianza (forecast)"""
    next_hatX::CA2 
end 
# Versiones generales los discretizadores comunes RK4, Euler, etc
# GeneralDiscretizer(x, a, p, dt) # tiene la misma idea de discretizer pero es más general. Creo que voy a tener que generalizar los que tengo para que funcione.

function set_hatX!(estimator::SimpleKalmanEstimation, hatX::ComponentArray) 
    estimator.hatX.x = hatX.x
    estimator.hatX.P = hatX.P
end 

function set_next_hatX!(estimator::SimpleKalmanEstimation, hatX::ComponentArray) 
    estimator.next_hatX.x = hatX.x
    estimator.next_hatX.P = hatX.P
end 


#=
usando un kalman discretizer es posible calcular next_hatX a partir de next_hatX
=# 

struct MultipleModelKalman
    # lista de probabilidades
    priors # array 
    # lista de SimpleKalmanEstimation s  
    # discretizer, updater, observer, system 
end 

how_many_models(mmkf::MultipleModelKalmanFilter) = length(mmkf.priors)
enumerate_models(mmkf::MultipleModelKalman) = 1:how_many_models(mmkf)

get_prior(mmfk::MultipleModelKalman, model) = mmkf.priors[model]

function mix_estimation(mmkf::MultipleModelKalman)
    sum(get_prior(mmkf,i) * hatx(mmkf.model[i]) for i in enumerate_models(mmkf))
end 
function mix_covariances(mmkf::MultipleModelKalman)
    sum(get_prior(mmkf, i) * hatP(mmkf.model[i]) for i in enumerate_models(mmkf))
end 

function update_priors!(mmkf::MultipleModelKalman)end 

gaussian_pdf(x, P) 

function probability_observation_given_p(
    observer::KalmanFilter.LinearizableObserver,
    estimation::SimpleKalmanEstimation,
    observation)
    r = observation - KalmanFilter.observe_without_error(observer, hatx(estimation))
    S = Hn(observer) * hatP(estimation) * Hn(observer)' + Rn(observer)
    pdfyp = gaussian_pdf(r, S)
    pdfyp
end 

function probability_observation_given_p(
    mmkf::MultipleModelKalman,
    observation, model)
    probability_observation_given_p(mmkf.observer, mmkf.model[model], observation)
end 

function probability_p_given_observation(mmkf::MultipleModelKalman, observation, model)
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
function forecast(mmkf::MultipleModelKalman, control)
    # esto es muy largo, pero incluye hacer forecast a cada 
    # uno de los filtros y luego combinarlo
end

function forecast!(updater::KalmanFilter.LinearizableUpdater, estimation::SimpleKalmanEstimation, control, t)
    hatXₙ = hatX(estimation)
    hatXₙ₊₁ = KalmanFilter.forecast(updater::KalmanFilter.LinearizableUpdater, hatXₙ.x, hatXₙ.P, control, estimation.p, t)
    set_hatX!(estimator, hatXₙ₊₁)
end 

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

Mn(updater::KalmanUpdaterWithGeneralDiscretizer, x, params, t) = jacobian_x(updater.discretizer, x, 0., params.p, t) 
#Bn(updater::KalmanUpdaterWithGeneralDiscretizer, x, α, params, t) = jacobian_x(updater.discretizer, x, α, p, t) 
Fn(updater::KalmanUpdaterWithGeneralDiscretizer, x, params, t) = make_F(updater)(params.filter_p) 
Qn(updater::KalmanUpdaterWithGeneralDiscretizer, x, α, params, t) = indepnoisematrix(dt(updater.discretizer))



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


#==================================================
 General Observer 
==================================================#

"""
Representa un observador lineal de la forma 
"""
struct CommonObserver{D <: GeneralDiscretizer, F1 <: Function, F2 <: Function} <: KalmanUpdaterWithGeneralDiscretizer
    """Observation matrix"""
    H
    discretizer::D
    makeF::F1
    integrity::F2
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