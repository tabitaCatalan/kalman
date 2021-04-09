
################################################################################
#= Observadores: interfaz a implementar =#
################################################################################

abstract type KalmanObserver end


function (observer::KalmanObserver)(x::AbstractArray, u::Real, error) error("Evaluation method not defined") end
function Hn(::KalmanObserver) error("Hn no definida") end
function Dn(::KalmanObserver) error("Dn no definida") end
function Gn(::KalmanObserver) error("Gn no definida") end
function observe_real_state(observer::KalmanObserver) error("observe_real_state no definida para este KalmanObserver") end

# Opcionales
function get_inner_state(observer::KalmanObserver) error("inner state not defined") end
# Esta función debería ser opcional, de quienes tengan inner state. O dejarla en  blanco
function update_real_state!(observer::KalmanObserver, updater::KalmanUpdater, control, error)
  new_state = updater(get_inner_state(observer), control, error)
  set_inner_state!(observer, new_state)
  #error("Updating method not defined")
end

#==================================================================
Implementación sencilla de la interfaz: LinearObserver
===================================================================#
"""
$(TYPEDEF)
Representa un observador lineal simple de la forma
```math
y_n = H x_n + D u_n + G_n N_n
```
de un estado interno ``x``.
# Campos
$(TYPEDFIELDS)
"""
struct LinearObserver{T} <: KalmanObserver
  H::AbstractMatrix{T}
  D::AbstractVector{T}
  G::AbstractVector{T}
  x::AbstractVector{T}
end
function get_inner_state(observer::LinearObserver) observer.x end
function set_inner_state!(observer::LinearObserver, x) observer.x .= x end
#function (observer::LinearObserver)(state, error)
#  observer.H * state.x + observer.D * state.u + observer.G * error
#end

function (observer::LinearObserver)(x::AbstractArray, u::Real, error)
  observer.H * x + observer.D * u + observer.G * error
end

Hn(observer::LinearObserver) = observer.H
Dn(observer::LinearObserver) = observer.D
Gn(observer::LinearObserver) = observer.G

function kalman_size(observer::LinearObserver)
  H = Hn(observer)
  size(H')
end

function observe_real_state(observer::LinearObserver, control, error)
  observer(get_inner_state(observer), control, error)
end
