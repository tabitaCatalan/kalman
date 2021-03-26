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
  X::StochasticState{T}
  hatX::ObservedState{T}
  updater::KalmanUpdater
  observer::LinearObserver{T}
  Pn::AbstractMatrix{T}
  noiser::UnivariateDistribution
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

hatxn(iterator) = iterator.hatX.hatx
hatPn(iterator) = iterator.hatX.hatP
En(iterator) = Rn(iterator) + Hn(iterator) * hatPn(iterator) * Hn(iterator)'
KalmanGain(iterator) = hatPn(iterator) * Hn(iterator)' * inv(En(iterator))


function analysed_state(iterator, observation)
  hatxn(iterator) + KalmanGain(iterator) * (observation - observe_observed_system(iterator))
end


function P_n(iterator)
  K = KalmanGain(iterator)
  hatPn(iterator) - K * En(iterator) * K'
end

# Funciones más generales para observar y actualizar el sistema

# Observar
observe_state_system(iterator) = iterator.observer(iterator.X, rand(iterator.noiser))

function observe_observed_system(iterator)
  tempX = StochasticState(iterator.hatX.hatx, iterator.X.u)
  iterator.observer(tempX, 0.)
end

# Actualizar
next_state_system(iterator, control) = iterator.updater(iterator.X.x, control, rand(iterator.noiser))
next_state_system(iterator) = iterator.updater(iterator.X, rand(iterator.noiser))
next_observed_system(iterator) = iterator.updater(iterator.X, 0.)

function update_state!(iterator, xnext, control)
  iterator.X = StochasticState(xnext, control)
end

function update_state_system!(iterator, control)
  next = next_state_system(iterator, control)
  update_state!(iterator, next, control)
  # iterator.updater = updater (para matrices variables en el tiempo)
end

function update_observer_system!(iterator, updater, control)
  next = next_observed_system(iterator, control)
  update_state!(iterator, next, control)
  # iterator.updater = updater (para matrices variables en el tiempo)
end

function update_updater!(iterator)
  update!(iterator.updater, hatxn(iterator), un(iterator))
end

function forecast_observed_state!(iterator, observation)
  hatx, hatP = forecast(iterator, observation)
  iterator.hatX = ObservedState(hatx, hatP)
  update_updater!(iterator)
end

function forecast(iterator::LinearKalmanIterator, observation)
  hatxnp1 = iterator.updater(analysed_state(iterator, observation), un(iterator), 0.)
  hatPnp1 = Mn(iterator) * P_n(iterator) * Mn(iterator)' + Qn(iterator)
    - Sn(iterator) * inv(En(iterator)) * Sn(iterator)'
    - Sn(iterator) * KalmanGain(iterator)' * Mn(iterator)'
    - Mn(iterator) * KalmanGain(iterator) * Sn(iterator)'
  hatxnp1, hatPnp1
end

function previous_step!(iterator, control, observation)
  forecast_observed_state!(iterator, observation)
  update_state_system!(iterator, control)
end
