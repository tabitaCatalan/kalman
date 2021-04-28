# Updater

################################################################################

# Interfaz que debe ser definida por las estructuras tipo `KalmanUpdater`.
abstract type KalmanUpdater end
function update!(L::KalmanUpdater, hatx, hatP, control) error("Updating method not defined") end
#function (updater::KalmanUpdater)(x::AbstractArray, u::Real, noise) error("Evaluation method not defined") end
function update_inner_system(updater::KalmanUpdater, x::AbstractArray, u::Real, noise) error("Implement update_inner_system") end
function update_aproximation(updater::KalmanUpdater, x::AbstractArray, u::Real, noise) error("Implement update_aproximation") end

abstract type LinearizableUpdater <: KalmanUpdater end

function Mn(::LinearizableUpdater) error("Por favor defina Mn para LinearizableUpdater") end
function Bn(::LinearizableUpdater) error("Por favor defina Bn para LinearizableUpdater") end
function Fn(::LinearizableUpdater) error("Por favor defina Fn para LinearizableUpdater") end
function Qn(::LinearizableUpdater) error("Por favor defina Qn para LinearizableUpdater") end

################################################################################

function (updater::KalmanUpdater)(state::StochasticState, error)
  updater(state.x, state.u, error)
end

function forecast(updater::LinearizableUpdater, hatx, hatP, control)
  hatPnp1 = forecast_hatP(updater, hatP)
  hatxnp1 = update_aproximation(updater, hatx, control, 0.)
  hatxnp1, hatPnp1
end

function forecast_hatP(updater::LinearizableUpdater, hatP)
  Mn(updater) * hatP * Mn(updater)' + Qn(updater)
  #  - Sn(iterator) * inv(E) * Sn(iterator)'
  #  - Sn(iterator) * K' * Mn(iterator)'
  #  - Mn(iterator) * K * Sn(iterator)'
end

dimensions(updater::LinearizableUpdater) = size(Mn(updater))[1]
noiser(updater::LinearizableUpdater) = MvNormal(zeros(dimensions(updater)), Qn(updater))



#= LinearUpdater define la interfaz de KalmanUpdater =#

"""
$(TYPEDEF)
Define un actualizador lineal constante.

Define un actualizador lineal del sistema, que permite calcular el estado ``x_{n+1}``
a partir del estado ``x_{n}`` y un control escalar ``u_n`` según
```math
x_{n+1} = M x_n + B u_n + F N_n
```
donde ``N_n`` es un número aleatorio (dado por una variable aleatorio normal
``\\mathcal{N}(0,1)``).
# Campos
$(TYPEDFIELDS)
"""
struct SimpleLinearUpdater{T} <: LinearizableUpdater
  """Matriz ``M``"""
  M::AbstractMatrix{T}
  """Vector ``B``"""
  B::AbstractVector{T}
  """Vector ``F``"""
  F::AbstractVector{T}
  """Función que corrige `x` para dejarlo dentro de un dominio."""
  integrity
end


function (updater::SimpleLinearUpdater)(x::AbstractArray, u::Real, error)
  updater.integrity(updater.M * x + updater.B * u + updater.F * error)
end

update_inner_system(updater::SimpleLinearUpdater, x::AbstractArray, u::Real, noise) = updater(x, u, noise)
update_aproximation(updater::SimpleLinearUpdater, x::AbstractArray, u::Real, noise) = updater(x, u, noise)


function update!(L::SimpleLinearUpdater, hatx, hatP, control) end # no necesita actualizarse

Mn(updater::SimpleLinearUpdater) = updater.M
Bn(updater::SimpleLinearUpdater) = updater.B
Fn(updater::SimpleLinearUpdater) = updater.F
