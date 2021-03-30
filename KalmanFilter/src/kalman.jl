using Random, Distributions, DocStringExtensions

#abstract type State end

struct StochasticState{T}# < State
  x::AbstractVector{T}
  u::T
end

struct ObservedState{T} #< State
  hatx::AbstractVector{T}
  hatP::AbstractMatrix{T}
end

################################################################################

# Interfaz que debe ser definida por las estructuras tipo `KalmanUpdater`.
abstract type KalmanUpdater end
function update!(L::KalmanUpdater, hatx, control) error("Updating method not defined") end
function (updater::KalmanUpdater)(x::AbstractArray, u::Real, error) error("Evaluation method not defined") end
function Mn(::KalmanUpdater) error("Mn no definida") end
function Bn(::KalmanUpdater) error("Bn no definida") end
function Fn(::KalmanUpdater) error("Fn no definida") end
################################################################################

function (updater::KalmanUpdater)(state::StochasticState, error)
  updater(state.x, state.u, error)
end


#= LinearUpdater define la interfaz de KalmanUpdater =#

"""
$(TYPEDEF)
Define un actualizador lineal constante.

Define un actualizador lineal del sistema, que permite calcular el estado ``x_{n+1}``
a partir del estado ``x_{n}`` y el control ``u_n`` según
```math
x_{n+1} = M x_n + B u_n + F N_n
```
donde ``N_n`` es un número aleatorio (dado por una variable aleatorio normal
``\\mathcal{N}(0,1)``).
# Campos
$(TYPEDFIELDS)
"""
struct SimpleLinearUpdater{T} <: KalmanUpdater
  """Matriz ``M``"""
  M::AbstractMatrix{T}
  """Matriz ``B``"""
  B::AbstractVector{T}
  """Matriz ``F``"""
  F::AbstractVector{T}
end



function (updater::SimpleLinearUpdater)(x::AbstractArray, u::Real, error)
  updater.M * x + updater.B * u + updater.F * error
end

function update!(L::SimpleLinearUpdater, hatx, control) end # no necesita actualizarse

Mn(updater::SimpleLinearUpdater) = updater.M
Bn(updater::SimpleLinearUpdater) = updater.B
Fn(updater::SimpleLinearUpdater) = updater.F

################################################################################
#= Observadores
El tipo abstracto `KalmanObserver` está pensado como una interfaz a las estructuras
que permitan observar el sistema en estado ``x_{n}``, entregando una observación
``y_{n}``. Se espera que sean, o bien lineales de la forma
``
y_{n} = H_n x_n + D_n u_n + G_n N_n
``
o bien, en caso de ser no lineales ``y_{n} = \mathcal{H}(x_n, u_n)``, que
puedan ser linealizados a la forma anterior.
Se espera que tengan la siguiente interfaz:
`update!(L::KalmanUpdater, hatx, control)`: un método que permita actualizar al
iterador y dejarlo listo para la siguiente iteración. Cuando se usan matrices
``H_n := M, D_n := D, G_n:= G`` contantes se puede dejar en blanco, pero debería
usarse, por ejemplo, para linearlizar en torno a ``\hat{x}_n`` cuando se usa
un `KalmanObserver` no lineal.
- `Hn`, `Dn`, `Gn` de la linearización en el estado actual
- Debe poder ser evaluado en la siguiente firma: `(x::AbstractArray, u::Real, error)`
=#
################################################################################

abstract type KalmanObserver end

struct LinearObserver{T} <: KalmanObserver
  H::AbstractMatrix{T}
  D::AbstractVector{T}
  G::AbstractVector{T}
end


function (observer::LinearObserver)(state, error)
  observer.H * state.x + observer.D * state.u + observer.G * error
end

function (observer::LinearObserver)(x,u, error)
  observer.H * x + observer.D * u + observer.G * error
end

Hn(observer::LinearObserver) = observer.H
Dn(observer::LinearObserver) = observer.D
Gn(observer::LinearObserver) = observer.G

################################################################################
#= Iterador de Kalman
La estructura `LinearKalmanIterator` contiene la información tanto del sistema
estocástico como del sistema determinista dado por las formulas del filtro de
Kalman.
Puede ser observado y actualizado.
=#
################################################################################
# No creo que esto sea necesario... o tal vez sí, podría ser útil para definir
# iteradores que no conozcan el sistema (lo que será útil después, al incorporar
# la info de los casos reales)
# abstract type KalmanIterator end

mutable struct LinearKalmanIterator{T} #<: KalmanIterator
  n::Int
  X::StochasticState{T}
  hatX::ObservedState{T}
  next_hatX::ObservedState{T}
  updater::KalmanUpdater
  observer::LinearObserver{T}
  #Pn::AbstractMatrix{T}
  noiser::UnivariateDistribution
  function LinearKalmanIterator(x0::AbstractVector{T}, P0::AbstractMatrix{T},
      updater::KalmanUpdater,
      observer::KalmanObserver) where T <: Real
    n = 0
    X = StochasticState(x0, 0.)
    hatX = ObservedState(x0, P0)
    next_hatX = ObservedState(x0, P0)
    noiser = Normal()
    new{T}(n, X, hatX, next_hatX, updater, observer, noiser)
  end
end



function next_iteration!(iterator, control)
  # advance real system
  xₙ₊₁ = next_state_system(iterator, control); uₙ = control
  iterator.X = Xₙ₊₁ = StochasticState(xₙ₊₁, uₙ)

  forecast_observed_state!(iterator, control) #esto cambia next_hatX

  # observe system
  yₙ₊₁ = observe_state_system(iterator) # hay que actualizar iterator.X primero...

  # analysis
  hatPₙ₊₁ₙ₊₁ = analyse_hatP(iterator)
  hatxₙ₊₁ₙ₊₁ = analyse_hatx(iterator, yₙ₊₁)

  iterator.hatX = ObservedState(hatxₙ₊₁ₙ₊₁, hatPₙ₊₁ₙ₊₁)

  update_updater!(iterator)

  iterator.n = iterator.n + 1
  yₙ₊₁
end

function observe_forecasted_system(iterator)
  iterator.observer(next_hatx(iterator), un(iterator), 0.)
end

# Getters de matrices y fórmulas para actualizar y todo eso

Mn(iterator::LinearKalmanIterator) = Mn(iterator.updater)
Bn(iterator::LinearKalmanIterator) = Bn(iterator.updater)
Fn(iterator::LinearKalmanIterator) = Fn(iterator.updater)
Hn(iterator::LinearKalmanIterator) = Hn(iterator.observer)
Dn(Iterator::LinearKalmanIterator) = Dn(Iterator.observer)
Gn(Iterator::LinearKalmanIterator) = Gn(Iterator.observer)

Sn(iterator::LinearKalmanIterator) = Fn(iterator) * Gn(iterator)'
Rn(iterator::LinearKalmanIterator) = Gn(iterator) * Gn(iterator)'
Qn(iterator::LinearKalmanIterator) = Fn(iterator) * Fn(iterator)'

xn(iterator::LinearKalmanIterator) = iterator.X.x
un(iterator::LinearKalmanIterator) = iterator.X.u

hatx(iterator) = iterator.hatX.hatx
hatP(iterator) = iterator.hatX.hatP

next_hatP(iterator) = iterator.next_hatX.hatP
next_hatx(iterator) = iterator.next_hatX.hatx


En(iterator, hatPnp1n) = Rn(iterator) + Hn(iterator) * hatPnp1n * Hn(iterator)'
En(iterator) = En(iterator, next_hatP(iterator))

KalmanGain(iterator, hatPnp1n) = hatPnp1n * Hn(iterator)' * inv(En(iterator))
KalmanGain(iterator) = KalmanGain(iterator, next_hatP(iterator))


#function analysed_state(iterator, observation)
#  hatxn(iterator) + KalmanGain(iterator) * (observation - observe_observed_system(iterator))
#end


# Funciones más generales para observar y actualizar el sistema

# Observar
observe_state_system(iterator) = iterator.observer(iterator.X, rand(iterator.noiser))

function observe_observed_system(iterator)
  tempX = StochasticState(next_hatx(iterator), un(iterator))
  iterator.observer(tempX, 0.)
end

# Actualizar
next_state_system(iterator, control) = iterator.updater(xn(iterator), control, rand(iterator.noiser))
#next_observed_system(iterator, control) = iterator.updater(hatx(iterator), control, 0.)

function update_state!(iterator, xnext, control)
  iterator.X = StochasticState(xnext, control)
end

function update_state_system!(iterator, control)
  next = next_state_system(iterator, control)
  update_state!(iterator, next, control)
end

#function update_observer_system!(iterator, updater, control)
#  next = next_observed_system(iterator, control)
#  update_state!(iterator, next, control)
#  # iterator.updater = updater (para matrices variables en el tiempo)
#end

function update_updater!(iterator)
  update!(iterator.updater, hatx(iterator), un(iterator))
end



function forecast(iterator, control)
  hatPₙₙ = hatP(iterator)
  K = KalmanGain(iterator, hatPₙₙ)
  E = En(iterator, hatPₙₙ)
  hatPₙ₊₁ₙ = forecast_hatP(iterator, K, E, hatPₙₙ, control) # idem
  hatxₙ₊₁ₙ = forecast_hatx(iterator, control) # aun no debe haber cambiado el analizado
  hatxₙ₊₁ₙ, hatPₙ₊₁ₙ
end

function forecast_hatx(updater, hatx, control)
  updater(hatx, control, 0.)
end

function forecast_hatx(iterator::LinearKalmanIterator, control)
  forecast_hatx(iterator.updater, hatx(iterator), control)
end

function forecast_hatP(iterator::LinearKalmanIterator, K, E, hatP, control)
  Mn(iterator) * hatP * Mn(iterator)' + Qn(iterator)
    - Sn(iterator) * inv(E) * Sn(iterator)'
    - Sn(iterator) * K' * Mn(iterator)'
    - Mn(iterator) * K * Sn(iterator)'
end

function forecast_hatP(iterator::LinearKalmanIterator, control)
  forecast_hatP(iterator::LinearKalmanIterator, KalmanGain(iterator), En(iterator), hatP(iterator), control)
end

function forecast_observed_state!(iterator, control)
  # forecast P para calcular K y E
  hatxₙ₊₁ₙ, hatPₙ₊₁ₙ = forecast(iterator, control) # idem
  iterator.next_hatX = ObservedState(hatxₙ₊₁ₙ, hatPₙ₊₁ₙ)
end


#= Analizar solucion incorporando observación=#

function analyse_hatx(iterator, observation)
  next_hatx(iterator) + KalmanGain(iterator) * (observation - observe_forecasted_system(iterator))
end

function analyse_hatP(iterator)
  K = KalmanGain(iterator)
  next_hatP(iterator) - K * En(iterator) * K'
end



function full_iteration(iterator, N)

  dimensions = length(xn(iterator))

  observations = Vector{Float64}(undef, N)
  real_states = Array{Float64, 2}(undef, N, dimensions)
  analysis = Array{Float64, 2}(undef, N, dimensions)
  forecast = Array{Float64, 2}(undef, N, dimensions)

  errors_analysis = Array{Float64, 2}(undef, N, dimensions)
  errors_forecast = Array{Float64, 2}(undef, N, dimensions)

  for i in 1:N
    control = 1.
    #observation = KalmanFilter.observe_state_system(iterator)

    real_states[i,:] = KalmanFilter.xn(iterator)
    analysis[i,:] = KalmanFilter.hatx(iterator)

    forecastx, forecastP = KalmanFilter.forecast(iterator, control)
    forecast[i,:] = forecastx
    errors_analysis[i,:] = [KalmanFilter.hatP(iterator)[j,j] for j in 1:dimensions]
    errors_forecast[i,:] = [forecastP[j,j] for j in 1:dimensions]

    #iterator.X.x ≈ iterator.hatX.hatx ? print("!") :
    #predictions2[i,:] = KalmanFilter.analysed_state(iterator, observation)

    observations[i] = KalmanFilter.next_iteration!(iterator, control)[1]
    # Save states

  end
  observations, real_states, analysis, forecast, errors_analysis, errors_forecast
end
