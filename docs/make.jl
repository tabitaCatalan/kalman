using Documenter
using KalmanFilter

makedocs(modules=[KalmanFilter],
         doctest=false, clean=true,
         #format = Documenter.HTML(canonical="https://tabitaCatalan/kalman/stable"),
         format = Documenter.HTML(prettyurls=false),
         sitename="KalmanFilter.jl",
         authors="Tabita Catalán",
         pages = Any[
         "KalmanFilter.jl" => "index.md",
         "Iterators" => "iterators.md",
         "Ejemplos" => Any["ejemplos/NLFilterUnknownInput.md"],
         "Updaters" => "updaters.md", 
         "Observers" => "observers.md",
         "Discretizers" => "discretizers.md"
         ])
