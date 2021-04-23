module KalmanFilter

export SimpleLinearUpdater, NLUpdater

using DocStringExtensions

include("states.jl")
include("updaters.jl")
include("observers.jl")
include("observablesystem.jl")

include("kalman.jl")

include("discretizers.jl")
include("NLkalman.jl")
include("UnknownInput.jl")
include("EnKFUpdater.jl")
include("ODEUpdater.jl")
include("filteredseries.jl")
end # module
