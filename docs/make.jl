using Documenter
using KalmanFilter

makedocs(modules=[KalmanFilter],
         doctest=false, clean=true,
         #format = Documenter.HTML(canonical="https://tabitaCatalan/kalman/stable"),
         format = Documenter.HTML(),
         sitename="KalmanFilter.jl",
         authors="Tabita Catalán",
         pages = Any[
         "KalmanFilter.jl" => "index.md",
         #"Tutorials" => Any["tutorials/ode_example.md"],
         "Updaters" => "updaters.md", 
         "Discretizers" => "discretizers.md"
         ])
