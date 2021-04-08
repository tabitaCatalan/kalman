
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

function (observer::LinearObserver)(x,u, error)
  observer.H * x + observer.D * u + observer.G * error
end

Hn(observer::LinearObserver) = observer.H
Dn(observer::LinearObserver) = observer.D
Gn(observer::LinearObserver) = observer.G
