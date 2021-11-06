# Updater
using Distributions, Random, ComponentArrays

################################################################################

# Interfaz que debe ser definida por las estructuras tipo `KalmanUpdater`.
abstract type KalmanUpdater end
function update!(L::KalmanUpdater, hatx, hatP, control, t) error("Updating method not defined") end
#function (updater::KalmanUpdater)(x::AbstractArray, u::Real, noise) error("Evaluation method not defined") end
function update_inner_system(updater::KalmanUpdater, x::AbstractArray, u::Real, t) error("Implement update_inner_system") end
function update_aproximation(updater::KalmanUpdater, x::AbstractArray, u::Real, t) error("Implement update_aproximation") end

function dt(updater::KalmanUpdater) error("dt method not defined for updater") end

abstract type LinearizableUpdater <: KalmanUpdater end

function Mn(::LinearizableUpdater) error("Por favor defina Mn para LinearizableUpdater") end
function Bn(::LinearizableUpdater) error("Por favor defina Bn para LinearizableUpdater") end
function Fn(::LinearizableUpdater) error("Por favor defina Fn para LinearizableUpdater") end
function Qn(::LinearizableUpdater) error("Por favor defina Qn para LinearizableUpdater") end

################################################################################

function (updater::KalmanUpdater)(state::StochasticState, t, error)
  updater(state.x, state.u, t, error)
end
#=================================================================
Las funciones que siguen solo sirven para el caso en que updater 
es linealizable, es decir, que tiene definidas las funciones 
`Mn`, `Bn`, `Fn`, `Qn`.
De no ser así, será obligatorio definir
`forecast(updater::KalmanUpdater, hatx, hatP, control, t)` 
=================================================================#

function forecast(updater::LinearizableUpdater, hatx, hatP, control, t)
  hatPnp1 = forecast_hatP(updater, hatP)
  hatxnp1 = update_aproximation(updater, hatx, control, t)
  ComponentArray(x = hatxnp1, P = hatPnp1)
end
# esta funcion es para EnKF, que necesita agregarle ruido a las aproximaciones
# a diferencia de los demas metodos
function forecast_with_error(updater::LinearizableUpdater, hatx, hatP, control, t)
  hatPnp1 = forecast_hatP(updater, hatP)
  hatxnp1 = update_inner_system(updater, hatx, control, t)
  ComponentArray(x = hatxnp1, P = hatPnp1)
end

function forecast_hatP(updater::LinearizableUpdater, hatP)
  Mn(updater) * hatP * Mn(updater)' + Fn(updater) * Qn(updater) * Fn(updater)'
  #  - Sn(iterator) * inv(E) * Sn(iterator)'
  #  - Sn(iterator) * K' * Mn(iterator)'
  #  - Mn(iterator) * K * Sn(iterator)'
end

"""
$(TYPEDSIGNATURES)

Devuelve el número de dimensiones que puede soportar el `LinearizableUpdater`
en el estado. 
"""
dimensions(updater::LinearizableUpdater) = size(Mn(updater))[1]

"""
$(TYPEDSIGNATURES)

Devuelve una distribución normal multivariada ``\\mathcal{N}(0, Q)``, 
donde ``Q`` es la matriz de covarianzas del `LinearizableUpdater`.
"""
noiser(updater::LinearizableUpdater) = MvNormal(zeros(dimensions(updater)), Qn(updater))



#= LinearUpdater define la interfaz de KalmanUpdater =#

abstract type Integrity end
# An example of how to use integrity class
struct MaxIntegrity <: Integrity end
(f::MaxIntegrity)(x) = max.(x,0.)

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
struct SimpleLinearUpdater{T, 
                          A <:AbstractArray{T,2},
                          V <: AbstractVector{T},
                          I <: Integrity
                          } <: LinearizableUpdater
  """Matriz ``M``"""
  M::A
  """Vector ``B``"""
  B::V
  """Matriz ``F``"""
  F::A
  """Matriz ``Q`` de covarianzas del error"""
  Q::A
  """``\\Delta t``"""
  dt::T
  """Función que corrige `x` para dejarlo dentro de un dominio."""
  integrity::I
end
Mn(updater::SimpleLinearUpdater) = updater.M
Bn(updater::SimpleLinearUpdater) = updater.B
Fn(updater::SimpleLinearUpdater) = updater.F
Qn(updater::SimpleLinearUpdater) = updater.Q
dt(updater::SimpleLinearUpdater) = updater.dt 
integrity(up::SimpleLinearUpdater) = up.integrity

function (up::SimpleLinearUpdater)(x::AbstractArray, u::Real, t, noise)
  integrity(up)(Mn(up) * x + Bn(up) * u + Fn(up) * noise)
end

update_inner_system(updater::SimpleLinearUpdater, x::AbstractArray, u::Real, t) = updater(x, u, t, rand(noiser(updater)))
update_aproximation(updater::SimpleLinearUpdater, x::AbstractArray, u::Real, t) = updater(x, u, t, zeros(dimensions(updater)))

function update!(L::SimpleLinearUpdater, hatx, hatP, control, t) end # no necesita actualizarse