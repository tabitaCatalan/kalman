#################################################################
### Modelo SEIIR con input α desconocido
#################################################################

### Esta es mi oportunidad de usar un truquito del libro de patrones
# de diseño!! Algo de los atributos mágicos o no sé qué

using LinearAlgebra


"""
$(TYPEDEF)
Define un `Discretizer` con el método Runge-Kutta de orden 4, que permite ser
diferenciado con respecto a la variable ``x``.
# Campos
$(FIELDS)
"""
struct RK4Du <: RK4
  """
  La función tal que ``x' = f(x,u,p)``, donde ``x`` es el estado,
  ``\\alpha `` un control, y ``p`` son parámetros extra.
  """
  f
  """``D_x f (x,u,p)``, el diferencial de ``f`` con respecto a ``x``."""
  Dxf
  """``D_u f (x,u,p)``, el diferencial de ``f`` con respecto a ``u``."""
  Duf
  """Parámetros extra ``p``."""
  p
  """Tamaño del paso temporal ``\\Delta t``, tal que ``t_{n+1} = t_n + \\Delta t``."""
  dt
end

RK4Du(rkx::RK4Dx, Duf) = RK4Du(rkx.f, rkx.Dxf, Duf, rkx.p, rkx.dt)

function linearize_x(discretizer::RK4Du, F, x, α)
  M = jacobian_x(discretizer, x, α)
  B = jacobian_u(discretizer, x, α)
  SimpleLinearUpdater(M, B, F)
end


function jacobian_u(rk::RK4Du, x, α)
  dt = rk.dt; p = rk.p
  xs, ks = get_ks(rk, x, α)

  Dᵤk₁ = rk.Duf(xs[1], α, p)
  Dᵤx₂ = (dt/2) * Dᵤk₁
  Dᵤk₂ = rk.Dxf(xs[2], α, p) * Dᵤx₂ + rk.Duf(xs[2], α, p)
  Dᵤx₃ = (dt/2) * Dᵤk₂
  Dᵤk₃ = rk.Dxf(xs[3], α, p) * Dᵤx₃ + rk.Duf(xs[3], α, p)
  Dᵤx₄ = dt * Dᵤk₃
  Dᵤk₄ = rk.Dxf(xs[4], α, p) * Dᵤx₄ + rk.Duf(xs[4], α, p)

  (dt/6) * (Dᵤk₁ + 2 * Dᵤk₂ + 2 * Dᵤk₃ + Dᵤk₄)

end

"""
$(TYPEDEF)
Define un actualizador para trabajar con un estado dado por la discretización
de una ODE no lineal, aumentado para incorporar un input desconocido.

Esto permite calcular el estado aumentado
``\\begin{pmatrix}x_{n} \\ u_n\\end{pmatrix}``,estado ``x_{n+1}``,  a partir del
estado ``x_{n}``
```math
x_{n+1} = \\mathcal{M}(x_n, u_n) + F N_n
```
y un control ``u_n`` desconocido, que se intentará descubrir. ``N_n`` es un
número aleatorio (dado por una variable aleatorio normal ``\\mathcal{N}(0,1)``)
y ``\\mathcal{M}`` está dada por un `Discretizer`.
"""
mutable struct NLUpdaterUnknowInput <: KalmanUpdater
  state_updater::NLUpdater
  linear::SimpleLinearUpdater
  control
  n::Int
  Fa::Float64
  """
  # Argumentos
  - `nlupdater`: Debe ser ``x`` y ``u``-diferenciable
  - `control`: una función evaluable en tiempo
  - `Fa`: Error esperado en el input
  """
  function NLUpdaterUnknowInput(nlupdater::NLUpdater, control, Fa)
    # update nlupdater en x0
    # Para que funcione el Bn() hay que definir el linearize_u en el discretizador
    linear = linearize_augmented_state(nlupdater, Fa)
    n = 0
    new(nlupdater, linear, control, n, Fa)
  end
end

function linearize_augmented_state(nlupdater, Fa)
  M = Mn(nlupdater)
  filas = size(M)[1]; columnas = size(M)[1]
  tildeM = [M Bn(nlupdater); zeros(filas)' 1.]

  #tildeP = [F * F' zeros(2); zeros(filas)' 1.]

  #tildex0 = [x0; 1.]

  tildeF = [nlupdater.F; Fa]
  linear = SimpleLinearUpdater(tildeM, zeros(columnas + 1), tildeF)
  linear
end

Mn(nl::NLUpdaterUnknowInput) = Mn(nl.linear)
Bn(nl::NLUpdaterUnknowInput) = Bn(nl.linear)
Fn(nl::NLUpdaterUnknowInput) = Fn(nl.linear)

function update!(nl::NLUpdaterUnknowInput, hatX, control)
  hatx = hatX[1:end-1]
  hatu = hatX[end]
  update!(nl.state_updater, hatx, hatu)
  nl.linear = linearize_augmented_state(nl.state_updater, nl.Fa)
  nl.n += 1
end

function (nl::NLUpdaterUnknowInput)(X::AbstractArray, u::Real, error)
  X_next = similar(X)
  x = X[1:end-1]
  X_next[1:end-1] = update_inner_system(nl.state_updater, x, u, error)
  X_next[end] = u
  X_next
end

function update_inner_system(nl::NLUpdaterUnknowInput, X::AbstractArray, u::Real, noise)
  u = next_control(nl)
  nl(X,u,noise)
end
function update_aproximation(nl::NLUpdaterUnknowInput, X::AbstractArray, u::Real, noise)
  u = X[end]
  nl(X,u,noise)
end





next_control(nl::NLUpdaterUnknowInput) = nl.control(nl.n * nl.state_updater.discretizer.dt)
