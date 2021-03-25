#= Modelo SEIIR, pero selo SE (no lineal)
Modo simple, sin input desconocido =#

include("kalman.jl")
include("NLKalman.jl")
####################

# Modelo
#=
α = 1.
γₑ = 1/5
γᵢ = 1/5
φ = 0.4
=#
α = 1.881556778828361e-8
γₑ = 0.14
γᵢ = 0.14
φ = 0.3


p = (gammae = γₑ, gammai = γᵢ, phi = φ)

dt = 0.2

function episystem_full(x, α, p)
  γᵢ = p.gammai; γₑ = p.gammae; φ = p.phi;
  β = 1e-7
  [-α * β * x[1] * (x[2] + x[3]),
  α * β * x[1] * (x[2] + x[3]) - γₑ * x[2],
  (1 - φ) * γₑ * x[2] - γᵢ * x[3],
  φ * γₑ * x[2] - γᵢ * x[4],
  φ * γₑ * x[2]]
end
function epijacobian_full_x(x, α, p)
  γᵢ = p.gammai; γₑ = p.gammae; φ = p.phi;
  β = 1e-7
  [-α*β*(x[2] + x[3])  -α*β*x[1]     -α*β*x[1]  0.  0.;
  α*β*(x[2] + x[3])  (α*β*x[1]-γₑ)  α*β*x[1]  0.  0.;
  0.                 (1-φ)*γₑ       -γᵢ       0.  0.;
  0.                 φ*γₑ           0.        -γᵢ 0.;
  0.                 φ*γₑ           0.        0.  0.]
end


#####################################
#x0 = [100., 10., 0.1, 0.1, 0.1]
x0 = [7.112808e6, 1046.8508799517147, 0.0, 521.963080420307, 0.0]
F = 1000. * ones(5)
G = [50000.]
H = [0. 0. 0. 0. 1.]
dimensions = 5

#=

episystem(x, α, γ) = [-α * x[1] * x[2],
α * x[1] * x[2] - γₑ * x[2]]
epijacobian_x(x, α, γ) = [-α*x[2]   -α*x[1];
α*x[2]   α*x[1]-γₑ]


x0 = [100., 10]
F = 0.0001 * ones(2)
G = [0.5]
H = [1. 0.]
dimensions = 2
=#

rk = RK4(p, dt, episystem_full, epijacobian_full_x)
eu = Euler(p, dt, episystem_full, epijacobian_full_x)

####################


T = 40
N = Int(T/dt)


observations = Vector{Float64}(undef, N)
real_states = Array{Float64, 2}(undef, N, dimensions)
predictions = Array{Float64, 2}(undef, N, dimensions)
errors = Array{Float64, 2}(undef, N, dimensions)


nlupdater = NLUpdater(eu, F, x0, α)
observer = LinearObserver(H, zeros(1), G)
X = StochasticState(x0, α)
hatX = ObservedState(x0, F * F')
iterator = LinearKalmanIterator(X, hatX, nlupdater, observer, F * F', Normal())


for i in 1:N
  control = 1.
  observation = observe_state_system(iterator)
  observations[i] = observation[1]
  real_states[i,:] = iterator.X.x
  predictions[i,:] = iterator.hatX.hatx
  errors[i,:] = [iterator.hatX.hatP[j,j] for j in 1:dimensions]

  #iterator.X.x ≈ iterator.hatX.hatx ? print("!") :
  hatxn = analysed_state(iterator, observation)


  # Save states

end

using Plots


ts = 0.0:dt:(T-dt)
rango = 1:length(ts)
plot(ts[rango], real_states[rango,1], title = "Susceptibles", label = "Real")
plot!(ts[rango], predictions[rango,1], label = "Kalman", ribbon = sqrt.(errors[rango,1]))

plot(ts[rango], real_states[rango,2], title = "Expuestos", label = "Real")
plot!(ts[rango], predictions[rango,2], label = "Kalman",  ribbon = sqrt.(errors[rango,2]))

plot(ts[rango], real_states[rango,3], title = "Mild", label = "Real")
plot!(ts[rango], predictions[rango,3], label = "Kalman",  ribbon = sqrt.(errors[rango,3]))

plot(ts[rango], real_states[rango,4], title = "Infectados", label = "Real")
plot!(ts[rango], predictions[rango,4], label = "Kalman",  ribbon = sqrt.(errors[rango,4]))


plot(ts[rango], real_states[rango,5], title = "Acumulados", label = "Real")
plot!(ts[rango], predictions[rango,5], label = "Kalman",  ribbon = sqrt.(errors[rango,5]))
plot!(ts[rango], observations[rango], label = "Observaciones", legend =:bottomright)


#################################################################
### Modelo SEIIR con input α desconocido
#################################################################

tildeM = [M B; 0. 0. 1.]
tildeH = [H 0.]

tildeP = [F * F' zeros(2); 0. 0. 1.]

tildex0 = [x0; 1.]

tildeF = [F; 0.]

tildeF * tildeF'
tildeG = G
begin
  observations2 = Vector{Float64}(undef, N)
  real_states2 = Array{Float64, 2}(undef, N, 3)
  predictions2 = Array{Float64, 2}(undef, N, 3)

  updater2 = LinearUpdater(tildeM, zeros(3), tildeF)
  observer2 = LinearObserver(tildeH, zeros(1), G)
  X2 = StochasticState(tildex0, 0.)
  hatX2 = ObservedState(tildex0, tildeP)
  iterator2 = LinearKalmanIterator(X2, hatX2, updater2, observer2, tildeP, Normal())

  for i in 1:N
    control = 1.
    observation = observe_state_system(iterator2)
    previous_step!(iterator2, control, observation)

    # Save states
    observations2[i] = observation[1]
    real_states2[i,:] = iterator2.X.x
    predictions2[i,:] = iterator2.hatX.hatx
  end
end
# una interfaz que permita trabajar con un sistema
