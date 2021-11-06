
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

state_dimension(obs::KalmanObserver) = size(Hn(obs))[2]
observation_dimension(obs::KalmanObserver) = size(Hn(obs))[1]


#="""
$(TYPEDSIGNATURES)

Devuelve una distribución normal multivariada ``\\mathcal{N}(0, Q)``, 
donde ``Q`` es la matriz de covarianzas del `LinearizableUpdater`.
"""=#
#noiser(observer::NLObserver) = MvNormal(zeros(dimensions(observer)), Rn(observer))

#==================================================================
Implementación sencilla de la interfaz: LinearObserver
===================================================================#

abstract type LinearObserver <: KalmanObserver end

#noise(obs::LinearObserver, dt) = rand(Normal(dt), observation_dimension(obs))
(obs::LinearObserver)(x, u, error) = Hn(obs) * x + Dn(obs) * u + Gn(obs) * error #noise(obs, dt)

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
                                M<:AbstractArray{T,2},
                                V <: AbstractVector{T},
                                } <: LinearObserver
  H::M
  D::V
  G::M
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

function kalman_size(observer::KalmanObserver)
  H = Hn(observer)
  n,m = size(H)
  m,n
end
