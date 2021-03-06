
using LinearAlgebra

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
struct Euler <: Discretizer
  """
  La función tal que ``x' = f(x,\\alpha, p, t)``, donde ``x`` es el estado,
  ``\\alpha `` un control, ``p`` son parámetros extra y ``t`` es el tiempo.
  """
  f
  """``D_x f (x,\\alpha, p, t)``, el diferencial de ``f`` con respecto a ``x``."""
  Dxf
  """Parámetros extra ``p``."""
  p
  """Tamaño del paso temporal ``\\Delta t``, tal que ``t_{n+1} = t_n + \\Delta t``."""
  dt
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
struct SimpleRK4 <: RK4
  """
  La función tal que ``x' = f(x,\\alpha, p, t)``, donde ``x`` es el estado,
  ``\\alpha `` un control, ``p`` son parámetros extra y ``t`` es el tiempo.
  """
  f
  """Parámetros extra ``p``."""
  p
  """Tamaño del paso temporal ``\\Delta t``, tal que ``t_{n+1} = t_n + \\Delta t``."""
  dt
end

"""
$(TYPEDEF)
Define un `Discretizer` con el método Runge-Kutta de orden 4, que permite ser
diferenciado con respecto a la variable ``x``.
# Campos
$(FIELDS)
"""
struct RK4Dx <: RK4
  """
  La función tal que ``x' = f(x,\\alpha, p, t)``, donde ``x`` es el estado,
  ``\\alpha `` un control, ``p`` son parámetros extra y ``t`` es el tiempo.
  """
  f
  """``D_x f (x,\\alpha, p, t)``, el diferencial de ``f`` con respecto a ``x``."""
  Dxf
  """Parámetros extra ``p``."""
  p
  """Tamaño del paso temporal ``\\Delta t``, tal que ``t_{n+1} = t_n + \\Delta t``."""
  dt
end

function get_ks(rk::RK4, x, α, t)
  p = rk.p; dt = rk.dt

  x1 = x
  k1 = rk.f(x1, α, p, t)

  x2 = x1 + dt * k1/2
  k2 = rk.f(x2, α, p, t)

  x3 = x1 + dt * k2/2
  k3 = rk.f(x3, α, p, t)

  x4 = x1 + dt * k3
  k4 = rk.f(x4, α, p, t)
  (x1, x2, x3, x4), (k1, k2, k3, k4)
end

function (rk::RK4)(x,α,t)

  xs, ks = get_ks(rk, x, α, t)

  x + (rk.dt/6) * (ks[1] + 2 * ks[2] + 2 * ks[3] + ks[4])
end

function jacobian_x(rk::RK4, x, α, t)
  dt = rk.dt; p = rk.p
  xs, ks = get_ks(rk, x, α, t)

  Dₓk₁ = rk.Dxf(xs[1], α, p, t)
  Dₓx₂ = I + (dt/2) * Dₓk₁
  Dₓk₂ = rk.Dxf(xs[2], α, p, t) * Dₓx₂
  Dₓx₃ = I + (dt/2) * Dₓk₂
  Dₓk₃ = rk.Dxf(xs[3], α, p, t) * Dₓx₃
  Dₓx₄ = I + dt * Dₓk₃
  Dₓk₄ = rk.Dxf(xs[4], α, p, t) * Dₓx₄

  I + (dt/6) * (Dₓk₁ + 2 * Dₓk₂ + 2 * Dₓk₃ + Dₓk₄)
end
