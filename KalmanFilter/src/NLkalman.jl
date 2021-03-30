#=
Código que extiende a kalman.jl, para incorporar Updaters no lineales
=#

using LinearAlgebra

abstract type Discretizer end

function (::Discretizer)(x,α) error("No se ha definido un método para este discretizador.") end
function jacobian_x(ds::Discretizer, x, α) error("No se ha definido un método para este discretizador.")  end

"""
$(TYPEDEF)
Define un `Discretizer` con el método de Euler progresivo.
# Campos
$(FIELDS)
"""
struct Euler <: Discretizer
  """
  La función tal que ``x' = f(x,\\alpha, p)``, donde ``x`` es el estado,
  ``\\alpha `` un control, y ``p`` son parámetros extra.
  """
  f
  """``D_x f (x,\\alpha, p)``, el diferencial de ``f`` con respecto a ``x``."""
  Dxf
  """Parámetros extra ``p``."""
  p
  """Tamaño del paso temporal ``\\Delta t``, tal que ``t_{n+1} = t_n + \\Delta t``."""
  dt
end

(eu::Euler)(x, α) = x + eu.dt * eu.f(x, α , eu.p)
jacobian_x(eu::Euler, x, α) = I + eu.dt * eu.Dxf(x, α, eu.p)



"""
$(TYPEDEF)
Define un `Discretizer` con el método Runge-Kutta de orden 4.
# Campos
$(FIELDS)
"""
struct RK4 <: Discretizer
  """
  La función tal que ``x' = f(x,\\alpha, p)``, donde ``x`` es el estado,
  ``\\alpha `` un control, y ``p`` son parámetros extra.
  """
  f
  """``D_x f (x,\\alpha, p)``, el diferencial de ``f`` con respecto a ``x``."""
  Dxf
  """Parámetros extra ``p``."""
  p
  """Tamaño del paso temporal ``\\Delta t``, tal que ``t_{n+1} = t_n + \\Delta t``."""
  dt
end

function get_ks(rk::RK4, x, α)
  p = rk.p; dt = rk.dt

  x1 = x
  k1 = rk.f(x1, α, p)

  x2 = x1 + dt * k1/2
  k2 = rk.f(x2, α, p)

  x3 = x1 + dt * k2/2
  k3 = rk.f(x3, α, p)

  x4 = x1 + dt * k3
  k4 = rk.f(x4, α, p)
  (x1, x2, x3, x4), (k1, k2, k3, k4)
end

function (rk::RK4)(x,α)

  xs, ks = get_ks(rk, x, α)

  x + (rk.dt/6) * (ks[1] + 2 * ks[2] + 2 * ks[3] + ks[4])
end

function jacobian_x(rk::RK4, x, α)
  dt = rk.dt; p = rk.p
  xs, ks = get_ks(rk, x, α)

  I + (dt/6) * (rk.Dxf(xs[1], α, p) + 2 * rk.Dxf(xs[2], α, p) + 2 * rk.Dxf(xs[3], α, p)+ rk.Dxf(xs[4], α, p))
end

#########################

"""
$(TYPEDEF)
Define un actualizador dado por la discretización de una ODE.

Define un actualizador posiblemente no lineal del sistema, a partir de la
discretización de una Ecuación Diferencial Ordinaria. Esto permite calcular el
estado ``x_{n+1}`` a partir del estado ``x_{n}`` y el control ``u_n`` según
```math
x_{n+1} = \\mathcal{M}(x_n, u_n) + F N_n
```
donde ``N_n`` es un número aleatorio (dado por una variable aleatorio normal
``\\mathcal{N}(0,1)``) y ``\\mathcal{M}`` está dada por un `Discretizer`.
# Argumentos
$(TYPEDSIGNATURES)
"""
mutable struct NLUpdater <: KalmanUpdater
  discretizer::Discretizer
  F
  linear::SimpleLinearUpdater
  function NLUpdater(discretizer, F, x0, α)
    linear = linearize_x(discretizer, F, x0, α)
    new(discretizer, F, linear)
  end
end


function update!(updater::NLUpdater, x, α) #NLupdater = NonLinearUpdater(fancyM, jacobian_x)
  linear = linearize_x(updater, x, α)
  updater.linear = linear
end

Mn(updater::NLUpdater) = Mn(updater.linear)
Bn(updater::NLUpdater) = Bn(updater.linear)
Fn(updater::NLUpdater) = Fn(updater.linear)



#function (updater::NLUpdater)(state, control, error)
#  updater.discretizer(state.x, control) + updater.F * error
#end


function (updater::NLUpdater)(x::AbstractArray, u::Real, error)
  updater.discretizer(x, u) + updater.F * error
end
#### podría hacer algo que retorne un linear updater...

function linearize_x(NLup::NLUpdater, x, α)
  discretizer = NLup.discretizer
  linearize_x(discretizer, NLup.F, x, α)
end

function linearize_x(discretizer::Discretizer, F, x, α)
  M = jacobian_x(discretizer, x, α)
  B = discretizer(x, α) - M * x
  SimpleLinearUpdater(M, B, F)
end
