
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