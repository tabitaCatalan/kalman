
using LinearAlgebra: I 

abstract type Discretizer end

function (::Discretizer)(x,α,t) error("No se ha definido un método para este discretizador.") end
function jacobian_x(ds::Discretizer, x, α, t) error("No se ha definido un método para este discretizador.")  end
dt(ds::Discretizer) = ds.dt

"""
$(TYPEDEF)
Define un `Discretizer` con el método de Euler progresivo.
# Campos
$(FIELDS)
"""
struct Euler{F1 <: Function, F2 <: Function, P, T} <: Discretizer
  """
  La función tal que ``x' = f(x,\\alpha, p, t)``, donde ``x`` es el estado,
  ``\\alpha `` un control, ``p`` son parámetros extra y ``t`` es el tiempo.
  """
  f::F1
  """``D_x f (x,\\alpha, p, t)``, el diferencial de ``f`` con respecto a ``x``."""
  Dxf::F2
  """Parámetros extra ``p``."""
  p::P
  """Tamaño del paso temporal ``\\Delta t``, tal que ``t_{n+1} = t_n + \\Delta t``."""
  dt::T
end

(eu::Euler)(x, α, t) = x + eu.dt * eu.f(x, α , eu.p, t)
jacobian_x(eu::Euler, x, α, t) = I + eu.dt * eu.Dxf(x, α, eu.p, t)

abstract type RK4 <: Discretizer end

"""
$(TYPEDEF)
Define un `Discretizer` con el método Runge-Kutta de orden 4.
# Campos
$(FIELDS)
"""
struct SimpleRK4{F1 <: Function, P, T} <: RK4
  """
  La función tal que ``x' = f(x,\\alpha, p, t)``, donde ``x`` es el estado,
  ``\\alpha `` un control, ``p`` son parámetros extra y ``t`` es el tiempo.
  """
  f::F1
  """Parámetros extra ``p``."""
  p::P
  """Tamaño del paso temporal ``\\Delta t``, tal que ``t_{n+1} = t_n + \\Delta t``."""
  dt::T
end

"""
$(TYPEDEF)
Define un `Discretizer` con el método Runge-Kutta de orden 4, que permite ser
diferenciado con respecto a la variable ``x``.
# Campos
$(FIELDS)
"""
struct RK4Dx{F1 <: Function, F2 <: Function, P, T} <: RK4
  """
  La función tal que ``x' = f(x,\\alpha, p, t)``, donde ``x`` es el estado,
  ``\\alpha `` un control, ``p`` son parámetros extra y ``t`` es el tiempo.
  """
  f::F1
  """``D_x f (x,\\alpha, p, t)``, el diferencial de ``f`` con respecto a ``x``."""
  Dxf::F2
  """Parámetros extra ``p``."""
  p::P
  """Tamaño del paso temporal ``\\Delta t``, tal que ``t_{n+1} = t_n + \\Delta t``."""
  dt::T
end

function rungekutta_points(f, x, α, p, t, dt)
  x1 = x
  k1 = f(x1, α, p, t)

  x2 = x1 + dt * k1/2
  k2 = f(x2, α, p, t)

  x3 = x1 + dt * k2/2
  k3 = f(x3, α, p, t)

  x4 = x1 + dt * k3
  k4 = f(x4, α, p, t)
  (x1, x2, x3, x4), (k1, k2, k3, k4)
end

function rungekutta_predict(f, x, α, p, t, dt)
  xs, ks = rungekutta_points(f, x, α, p, t, dt)

  x + (dt/6) * (ks[1] + 2 * ks[2] + 2 * ks[3] + ks[4])
end 

function rungekutta_jacobian_x(f, Dxf, x, α, p, t, dt)
  xs, ks = rungekutta_points(f, x, α, p, t, dt)

  Dₓk₁ = Dxf(xs[1], α, p, t)
  Dₓx₂ = I + (dt/2) * Dₓk₁
  Dₓk₂ = Dxf(xs[2], α, p, t) * Dₓx₂
  Dₓx₃ = I + (dt/2) * Dₓk₂
  Dₓk₃ = Dxf(xs[3], α, p, t) * Dₓx₃
  Dₓx₄ = I + dt * Dₓk₃
  Dₓk₄ = Dxf(xs[4], α, p, t) * Dₓx₄

  I + (dt/6) * (Dₓk₁ + 2 * Dₓk₂ + 2 * Dₓk₃ + Dₓk₄)
end 


(rk::RK4)(x,α,t) = rungekutta_predict(rk.f, x, α, rk.p, t, rk.dt)
jacobian_x(rk::RK4, x, α, t) = rungekutta_jacobian_x(rk.f, rk.Dxf, x, α, rk.p, t, rk.dt)
