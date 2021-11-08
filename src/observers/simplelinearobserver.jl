
"""
$(TYPEDEF)
Representa un observador lineal simple de la forma. Las matrices son constantes.
```math
y_n = H x_n + D u_n + G_n N_n
```
de un estado interno ``x``.
# Campos
$(TYPEDFIELDS)
"""
struct SimpleLinearObserver{T,
                                M1 <:AbstractArray{T,2},
                                V <: AbstractVector{T},
                                M2 <:AbstractArray{T,2}, 
                                M3 <:AbstractArray{T,2}
                                } <: LinearObserver
  H::M1
  D::V
  G::M2
  R::M3
  #=
  Función que recibe una observación `y` y la corrige para dar valores razonables.
  Por ejemplo, para el caso de observar un sistema epidemiológico, no tiene sentido 
  que una variable sea negativa. Se podría definir `integrity(y) = max.(y,0.)`.
  =#
  #integrity::I
end 

Hn(observer::SimpleLinearObserver) = observer.H
Dn(observer::SimpleLinearObserver) = observer.D
Gn(observer::SimpleLinearObserver) = observer.G
Rn(observer::SimpleLinearObserver) = observer.R

function kalman_size(observer::KalmanObserver)
  H = Hn(observer)
  n,m = size(H)
  m,n
end