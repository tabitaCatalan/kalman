
################################################################################
#= Observadores: interfaz a implementar =#
################################################################################

abstract type KalmanObserver end

isLinearizableObserver(::KalmanUpdater) = false

function (observer::KalmanObserver)(x::AbstractArray, u::Real, error) error("Evaluation method not defined for observer") end

function noiser(observer::KalmanObserver) error("noiser not defined for observer") end

function observe_with_error(observer::KalmanObserver, x, u)
    observer(x, u, rand(noiser(observer)))
end 

function observe_without_error(observer::KalmanObserver, x, u)
    observer(x, u, zeros(observation_dimension(observer)))
end 

abstract type LinearizableObserver <: KalmanObserver end

isLinearizableObserver(::LinearizableObserver) = true

function Hn(::LinearizableObserver) error("Defin a Hn para LinearizableObserver") end
function Dn(::LinearizableObserver) error("Defin a Dn para LinearizableObserver") end
function Gn(::LinearizableObserver) error("Defin a Gn para LinearizableObserver") end
function Rn(::LinearizableObserver) error("Defin a Rn para LinearizableObserver") end

#=
Forma interna de agregar el ruido a las observaciones 
=#
"""
$(TYPEDSIGNATURES)

Devuelve el número de dimensiones que puede soportar el `LinearizableUpdater`
en el estado. 
"""
dimensions(observer::LinearizableObserver) = size(Hn(observer))[1]

state_dimension(obs::LinearizableObserver) = size(Hn(obs))[2]
observation_dimension(obs::LinearizableObserver) = size(Hn(obs))[1]

noiser(observer::LinearizableObserver) = noiser(Rn(observer))
(obs::LinearizableObserver)(x::AbstractArray, u::Real, error)= Hn(obs) * x + Dn(obs) * u + Gn(obs) * error #noise(obs, dt)
#="""
$(TYPEDSIGNATURES)

Devuelve una distribución normal multivariada ``\\mathcal{N}(0, Q)``, 
donde ``Q`` es la matriz de covarianzas del `LinearizableUpdater`.
"""=#
#noiser(observer::NLObserver) = MvNormal(zeros(dimensions(observer)), Rn(observer))

#==================================================================
Implementación sencilla de la interfaz: LinearObserver
===================================================================#

abstract type LinearObserver <: LinearizableObserver end

#noise(obs::LinearObserver, dt) = rand(Normal(dt), observation_dimension(obs))
