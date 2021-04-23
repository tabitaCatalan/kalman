using Documenter
using KalmanFilter

deployconfig = Documenter.auto_detect_deploy_system()

makedocs(modules=[KalmanFilter],
         doctest=false, clean=true,
         #format = Documenter.HTML(canonical="https://tabitaCatalan/kalman/stable"),
         format = Documenter.HTML(prettyurls=false),
         sitename="KalmanFilter.jl",
         authors="Tabita Catalán",
         pages = Any[
         "KalmanFilter.jl" => "index.md",
         "Iterators" => "iterators.md",
         "Updaters" => "updaters.md", 
         "Observers" => "observers.md",
         "Systems" => "systems.md",
         "Discretizers" => "discretizers.md", 
         "Ejemplos" => Any["ejemplos/NLFilterUnknownInput.md", 
                            "ejemplos/NLFilter.md", 
                            "ejemplos/NLFilterENKF.md"],
         ])

deploydocs(
    repo = "github.com/tabitaCatalan/kalman.git",
    deploy_config = deployconfig,
)
