module KalmanFilter

export SimpleLinearUpdater, NLUpdater

using DocStringExtensions

include("states.jl")
include(joinpath("updaters","updaters.jl"))
include(joinpath("updaters","simplelinearupdater.jl"))
include("observers.jl")
include("observablesystem.jl")

include(joinpath("iterators","iterators.jl"))
include(joinpath("iterators","simpleiterator.jl"))

include("discretizers.jl")
include(joinpath("updaters","nlupdater.jl"))
include("NLObserver.jl")
include(joinpath("iterators","enkfiterator.jl"))
include(joinpath("updaters","contdiscretmomentum.jl"))
include(joinpath("updaters","odeupdater.jl"))
include("filteredseries.jl")
end # module
