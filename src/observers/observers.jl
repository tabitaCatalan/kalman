
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