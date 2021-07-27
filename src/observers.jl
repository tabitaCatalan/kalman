
################################################################################
#= Observadores: interfaz a implementar =#
################################################################################

abstract type KalmanObserver end


function (observer::KalmanObserver)(x::AbstractArray, u::Real, error) error("Evaluation method not defined") end
function Hn(::KalmanObserver) error("Hn no definida") end
function Dn(::KalmanObserver) error("Dn no definida") end
function Gn(::KalmanObserver) error("Gn no definida") end

#=
Forma interna de agregar el ruido a las observaciones 
=#
"""
$(TYPEDSIGNATURES)

Devuelve el número de dimensiones que puede soportar el `LinearizableUpdater`
en el estado. 
"""
dimensions(observer::KalmanObserver) = size(Hn(observer))[1]

#="""
$(TYPEDSIGNATURES)

Devuelve una distribución normal multivariada ``\\mathcal{N}(0, Q)``, 
donde ``Q`` es la matriz de covarianzas del `LinearizableUpdater`.
"""=#
#noiser(observer::NLObserver) = MvNormal(zeros(dimensions(observer)), Rn(observer))

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
struct LinearObserver<: KalmanObserver
  H::AbstractMatrix
  D::AbstractVector
  G::AbstractVector
  """Función que recibe una observación `y` y la corrige para dar valores razonables.
  Por ejemplo, para el caso de observar un sistema epidemiológico, no tiene sentido 
  que una variable sea negativa. Se podría definir `integrity(y) = max.(y,0.)`.
  """
  integrity
end

function (observer::LinearObserver)(x::AbstractArray, u::Real, error)
  observer.integrity(observer.H * x + observer.D * u + observer.G * error)
end

Hn(observer::LinearObserver) = observer.H
Dn(observer::LinearObserver) = observer.D
Gn(observer::LinearObserver) = observer.G

function kalman_size(observer::KalmanObserver)
  H = Hn(observer)
  size(H')
end
