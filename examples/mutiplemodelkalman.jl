# include("src\\iterators\\mutiplemodelkalman.jl")

# # Ejemplo 10.3 del Simon 

# dependencies 
using ComponentArrays
using LinearAlgebra: I
# El sistema es de la forma 
# $$
# \begin{aligned}  
# \dot{x} &= Ax + Bw_1 \\
# &= \begin{bmatrix}
# 0 & 1 \\ 
# - w_n^2 & -2 \zeta w_n
# \end{bmatrix} x + 
# \begin{bmatrix}
# 0 \\ 
# - w_n^2
# \end{bmatrix} w_1 \\
# &= \begin{bmatrix}
# 1 & 0 \\ 
# 0 & 1 
# \end{bmatrix} x_k + v_k \\ 
# w_1 & \sim N(0, Q_c) \\
# v_k & \sim N(0, R)
# \end{aligned}  
# $$
# Funcion que darle al discretizador 

dT = 0.1
Afunc(wₙ, ζ = 0.1 ) = [0. 1.; - wₙ^2 -2*ζ*wₙ]
Bfunc(wₙ) = [0. 0.; 0. wₙ^2]
Qc = 1000. 


# Necesito una función que entregue la dinámica 
# debe ser de la forma f(x1, α, p, t) 

dynamics(x, α, p, t) = Afunc(p.wn) * x 
jacobianxdynamics(x, α, p, t) = Afunc(p.wn)

rk4 = KalmanFilter.GeneralRK4(dynamics, jacobianxdynamics, dT)

# Ahora un CommonUpdater 
makeF(filter_p) = Bfunc(filter_p.wn) * sqrt(Qc)

updater = KalmanFilter.CommonUpdater(2, rk4, makeF, x -> x)

# Un CommonObserver 
H = [1. 0.; 0. 1.]
makeG(filter_p) = sqrt(10.) * I
observer = KalmanFilter.CommonObserver(H, makeG, x -> x, 1.)

priors = [0.1, 0.6, 0.3] 

param1 = ComponentArray(p = ComponentArray(wn = sqrt(4.)), filter_p = ComponentArray(wn = sqrt(4.)))
param2 = ComponentArray(p = ComponentArray(wn = sqrt(4.4)), filter_p = ComponentArray(wn = sqrt(4.4)))
param3 = ComponentArray(p = ComponentArray(wn = sqrt(4.8)), filter_p = ComponentArray(wn = sqrt(4.8)))

x0 = [0., 0.] 
P0 = [0.001 0.; 0. 0.001]

X0 = ComponentArray(x = x0, P = P0)

models = [KalmanFilter.SimpleKalmanEstimation(param1, copy(X0), copy(X0)), 
    KalmanFilter.SimpleKalmanEstimation(param2, copy(X0), copy(X0)), 
    KalmanFilter.SimpleKalmanEstimation(param3, copy(X0), copy(X0))] 

# Generate observations 
T = 60
observations = Array{Float64, 2}(undef, T, 2)

observations[1,:] = x0
x = x0 
real_wn = sqrt(3.8)
for time in 1:T
    x = rk4(x, 0, ComponentArray(wn = real_wn), 0.) + KalmanFilter.make_noise(updater, ComponentArray(wn = real_wn))
    y = KalmanFilter.observe_with_error(observer, x, ComponentArray(wn = real_wn)) 
    observations[time,:] = y 
end 

system = KalmanFilter.Measurements(observations, dT)

mmkf = KalmanFilter.MultipleModelKalman(copy(priors), models, updater, observer, system, 0);

priors_in_time = Array{Float64, 2}(undef, T, 3)
estimations_in_time = Array{Float64, 2}(undef, T, 6)
for time in 1:T
    priors_in_time[time, :] = mmkf.priors
    estimations_in_time[time, 1:2] = KalmanFilter.hatx(KalmanFilter.get_model(mmkf, 1))
    estimations_in_time[time, 3:4] = KalmanFilter.hatx(KalmanFilter.get_model(mmkf, 2))
    estimations_in_time[time, 5:6] = KalmanFilter.hatx(KalmanFilter.get_model(mmkf, 3))
    KalmanFilter.next_iteration!(mmkf, 0.)
end 

#using Plots: plot, plot! 
#plot(observations)

display(plot(priors_in_time, title = "wₙ real $real_wn"))
#plot(estimations_in_time)
#plot!(observations)

#KalmanFilter.observe_with_error(observer, x0, ComponentArray(wn = real_wn)) 

#KalmanFilter.parameter_estimation(mmkf, :maxprob)