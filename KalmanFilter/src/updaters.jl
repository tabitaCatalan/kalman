# Updater

################################################################################

# Interfaz que debe ser definida por las estructuras tipo `KalmanUpdater`.
abstract type KalmanUpdater end
function update!(L::KalmanUpdater, hatx, control) error("Updating method not defined") end
#function (updater::KalmanUpdater)(x::AbstractArray, u::Real, noise) error("Evaluation method not defined") end
function update_inner_system(updater::KalmanUpdater, x::AbstractArray, u::Real, noise) error("Implement update_inner_system") end
function update_aproximation(updater::KalmanUpdater, x::AbstractArray, u::Real, noise) error("Implement update_aproximation") end

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
a partir del estado ``x_{n}`` y un control escalar ``u_n`` según
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
  """Vector ``B``"""
  B::AbstractVector{T}
  """Vector ``F``"""
  F::AbstractVector{T}
end


function (updater::SimpleLinearUpdater)(x::AbstractArray, u::Real, error)
  updater.M * x + updater.B * u + updater.F * error
end

update_inner_system(updater::SimpleLinearUpdater, x::AbstractArray, u::Real, noise) = updater(x, u, noise)
update_aproximation(updater::SimpleLinearUpdater, x::AbstractArray, u::Real, noise) = updater(x, u, noise)


function update!(L::SimpleLinearUpdater, hatx, control) end # no necesita actualizarse

Mn(updater::SimpleLinearUpdater) = updater.M
Bn(updater::SimpleLinearUpdater) = updater.B
Fn(updater::SimpleLinearUpdater) = updater.F
