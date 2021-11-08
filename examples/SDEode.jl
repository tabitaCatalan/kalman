# # Filtro de Kalman Extendido con input desconocido
#using Random
using KalmanFilter
using Plots
using CSV
using StaticArrays
#using LinearAlgebra

total_RM = reshape(CSV.File("data\\totales_RM.csv").Metropolitana,:,1)
RtRMData = CSV.File("data\\RtRM.csv", delim=';', decimal=',') 
Rt_RM = RtRMData.Metropolitana
Rt_inf = RtRMData.Inf
Rt_sup = RtRMData.Sup

include("examples\\seii_model_v2.jl")
include("examples\\common_defs.jl")
include("examples\\common_defs_enkf.jl")

#include("src\\NLObserver.jl")

#==================================# 


episystem_a(x, α, p, t) = [episystem_full(x[1:6], x[7],p); 0.]
epijacobian_a_x(x, α, p, t) = [epijacobian_full_x(x[1:6], x[7], p) epijacobian_full_u(x[1:6], x[7],p); zeros(7)']

tildeP = (x) -> tildeF(x) * tildeF(x)'
p1 = (gammae = 1/6.2, gammai = 1/7.8, phi = 0.48, pe = 0.08, pin = 0.6, pim = 0.3)
rkx = KalmanFilter.RK4Dx(episystem_a, epijacobian_a_x, p, dt)

x0prima = initial_states[4] # depende mucho de la condición inicial 
x0prima = copy(tildex0) #initial_states[3]
dims = length(tildex0)
#Q = Diagonal(sqrt(dt) * ones(length(tildex0)))
Q = Diagonal(sqrt(dt) * SVector{dims}(ones(dims)))
#struct Integrity <: KalmanFilter.Integrity end 
#(in::Integrity)(x) = [state_integrity(x[1:6], sum(x0)); min(max(x[7], 0.), 5e-7)]

function integrity(x)
  [state_integrity(x[1:6], sum(x0)); min(max(x[7], 0.), 5e-7)]
end
nlupdater = NLUpdater(rkx, tildeF(tildex0), Q, x0prima, 0., 0., integrity);

#=
Observer 
=#
#=
cum_Rt_obs(x, α, p) = [x[6]; Rt_value(x, x[7], p, sum(x0))] 
function Jx_cum_Rt_obs(x, α, p)
  RtDs, RtDa = Rt_DsDa(x, x[7], p, sum(x0)) 
  [
    0     0 0 0 0 1 0;
    RtDs  0 0 0 0 0 RtDa
  ]
end

# (H, DxH, G, x0, α, integrity)
nlobserver = KalmanFilter.NLObserver( (x, a) -> cum_Rt_obs(x, a, p),
            (x, a) -> Jx_cum_Rt_obs(x, a, p),
            [10., 0.5], tildex0, 0., y -> max.(y,0)
            )

=#

#using Distributions, Random
#rand(MvNormal(x0, tildeP(x0)))

#anon_func = (x) -> [state_integrity(x[1:6], sum(x0)); min(max(x[7], 0.), 5e-7)]
#anon_func(tildex0)
#Q = Diagonal(sqrt(dt) * ones(7))
n_steps = 4 
small_dt = dt/n_steps
Q = Diagonal(sqrt(small_dt) * ones(length(tildex0)))

#UM = KalmanFilter.UMomentum(episystem_a, Q, tildeF)

EM = KalmanFilter.ExtendedMomentum(episystem_a, epijacobian_a_x, x -> tildeF(tildex0), Q)


odeupdater = KalmanFilter.ODEForecaster(dt, n_steps, x0prima,
                                        tildeP(x0prima),
                                        p, 
                                        (x)-> state_integrity(x, sum(x0[1:5])), 
                                        EM)

                                        
#Hnew = [0. 1. 0. 0. 0. 0. 0.; 
#        0. 0. 0. 0. 0. 1. 0.]
obsdim = 1
R = Diagonal(sqrt(dt) * SVector{obsdim}(ones(obsdim)))
observer = KalmanFilter.SimpleLinearObserver(tildeH, zeros(1), R, R)
#observer = KalmanFilter.LinearObserver(Hnew, zeros(2), [1., 1.], observation_integrity)
#system = KalmanFilter.InnerState(tildex0)



observaciones = [0.1, 1.0580159464948373, 3.5870700420603026, 10.3766056520616, 29.17765883784156, 80.22001702779332, 224.27199734412486, 626.5825061023203, 1734.0539438178014, 4583.524171315316, 11043.81713021596, 23757.964092967894, 45436.47746837652, 77875.62270140811, 122505.23133524648, 172157.4270582024, 225865.5269395236, 281268.9431934942, 332694.0270276407, 387167.9566060224, 440843.9688405235, 499953.8196180029, 566144.4666627902, 631638.9546653195, 690335.5185676186, 750682.508424226, 820196.2893580594, 888015.0498463825, 948020.9938491018, 1.0010306979752984e6, 1.074091135219417e6, 1.136362253437184e6, 1.2024847136589722e6, 1.2539402085526502e6, 1.3112748657537634e6, 1.360811881626533e6, 1.4303943722394796e6, 1.473959832120512e6, 1.5254611712247003e6, 1.580979125946655e6, 1.612619543985415e6, 1.6633505418988662e6, 1.6816800792628673e6, 1.7057508750396862e6, 1.748898524993519e6, 1.7872539824598283e6, 1.8009864434713195e6, 1.8257023860288577e6, 1.8419866882036848e6, 1.8969305353374959e6, 1.9125969175881029e6, 1.9447125418109596e6, 1.9658459012092007e6, 1.972976151251052e6, 2.0208175111555695e6, 2.0748122488082221e6, 2.12318755832934e6, 2.1819635728725814e6, 2.193164987962152e6, 2.228209781763545e6, 2.2336874601433217e6, 2.2339609271844598e6, 2.2714142611998953e6, 2.3150464271635716e6, 2.324547290068156e6, 2.351044640428341e6, 2.4190158342167963e6, 2.474957420204883e6, 2.5394755859631943e6, 2.6116085230244766e6, 2.613702335387401e6];
observaciones = reshape(observaciones, (length(observaciones),1))

#observaciones = [total_RM[1:N] Rt_RM[1:N]]

system = KalmanFilter.Measurements(observaciones, dt)
# system = KalmanFilter.Measurements(total_RM, dt)

α = 0.7 * ones(size(x0prima))
α[end] = 0.3
iterator = KalmanFilter.SimpleKalmanIterator(x0prima, tildeP(x0prima), nlupdater, observer, system, dt, α);
#iterator = KalmanFilter.SimpleKalmanIterator(x0prima, tildeP(x0prima), odeupdater, observer, system, dt, α)
#==================================# 
# Y realizamos un total de `N` iteraciones, guardando los estamos intermedios
# en las variables que aparecen abajo.
#results, ensamble = KalmanFilter.full_iteration(enkf, dt, N, t -> 0., length(initial_states))
#results, ensamble = KalmanFilter.full_iteration(iterator, dt, size(total_RM)[1]-1, t -> 0., 1)
results, ensamble = KalmanFilter.full_iteration(iterator, dt, N, t -> 0., 1)

results, ensamble = KalmanFilter.full_iteration(iterator, dt, N, t -> 0., 1)

# La ecuación original respeta mas o menos la integridad...? No 
#plot([sum(sdesol[1:5,i]) for i in 1:length(ts)])

#results2, ensamble = KalmanFilter.full_iteration(iterator2, dt, N, t -> 0., 1)

# ## Resultados
# Graficaremos los estados internos considerados, y los resultados obtenidos.

# Ahora podemos graficar los diferentes estados de nuestro sistema, así como las
# aproximaciones obtenidas con filtro de Kalman.


r0vec, qtls = KalmanFilter.apply_to_ensemble(R_0, ensamble, 0.4)

spectral_radius(A) = maximum(abs.(eigvals(A)))

R_0(x) = R_0(x, x[7], p1)
function R_0(x, α, p)
  NGM = next_generation_matrix(x, α, p)
  spectral_radius(NGM) * x[1]/sum(x0)
end

R0vec = [R_0(results.analysis[i,:]) for i in 1:size(results.analysis)[1]]
#R0real = [R_0(sdesol[:, i], control_pieces(ts[i])) for i in 1:(size(sdesol)[2]-1)]

# KalmanFilter.apply_to_ensemble(x->x[7], ensamble, 0.05)


# plot(ts,R0real, title = "Rₜ", label = "Real")
#plot(ylim = (0,6), title = "Rₜ estimado en RM")
#plot!(ts, R0vec, label = "Filtro de Kalman con p")
plot!(ts, R0vec, label = "Filtro de Kalman con p₂")
plot!(0:length(Rt_RM)-1, Rt_RM, label = "CMM Cori", ribbon = (Rt_RM - Rt_inf, Rt_sup-Rt_RM))
hspan!([1., 0.], alpha = 0.2, label = "Rₜ ≤ 1")

#= Comentarios 
- No estoy usando ni de lejos buenos parámetros, elegi unos casi al azar xD 
- No estoy corrigiendo el problema al inicio de la cuestión, 
  el salto en los casos acumulados. 
- Solo le estoy dando de input los casos acumulados, nada más.
- Me falta ponerle una barra de error 
=# 


isposdef(KalmanFilter.hatP(iterator))

################################################################
# Gráficos 
################################################################

using Plots 

plot(ts, r0vec, ribbon = qtls)
# Susceptibles ``S``
plot(results, ts, 1, legend = :topleft)
#plot(results, ensamble, ts, 1, p = 0.05)
#plot!(ts, sdesol'[1:end-1, 1], label = "Sol real")

plot(results, ts, 1, legend = :bottomleft, ylims = (0,8e6))
plot!(ts, sdesol'[1:end-1, 1], label = "Sol real")
plot!(ensamble, ts, 1)
#plot!(ts, xs[:,1], label = "RTS smoother")

display(a)

# Expuestos ``E``
a = plot(results, ts, 2, legend = :topleft)
#plot(results, ensamble, ts, 2, p = 0.8)
plot!(ts, sdesol'[1:end-1, 2], label = "Sol real")
display(a)

# Infectados asintomáticos *mild* ``I^m``
a = plot(results, ts, 3, legend = :topleft)
plot!(ts, sdesol'[1:end-1, 3], label = "Sol real")
#plot!(results2, ts, 3)
display(a)

# Infectados ``I``
a = plot(results, ts, 4)
plot!(ts, sdesol'[1:end-1,4], label = "Sol real")
#plot!(results2, ts, 4)
display(a)

# Infectados acumulados ``cI``, y las observaciones que hicimos de él.
a = plot(results, ts, 5)
#plot!(ensamble, ts, 5)
plot!(ts, sdesol'[1:end-1, 5], label = "Sol real")
plot!(legend =:bottomright)
#plot!(results2, ts, 5)
display(a)
# Finalmente, veamos el control usado y el aproximado
#plot(ts, control_pieces.(ts), label = "Control real")
plot(results, ts, 6)
plot!(ts, sdesol'[1:end-1, 6], label = "Sol real")

plot(results, ts, 7, legend = :topleft)
plot(results, ts, 7, ylims = (0,4e-7), legend = :topleft)
plot!(ts, control_pieces.(ts), label = "Control real")
plot!(ensamble, ts, 7)

plot(results, ensamble, ts, 7)
using Statistics
plot(ts, results.analysis[:,2])
plot!(ts[1:end-2], moving_average(results.analysis[:,2], 3))
# Notamos que tras una cierta cantidad de tiempo es posible averiguar el control
# Aunque hay bastante incerteza de los resultados.
# Casos nuevos 

res, qt = KalmanFilter.apply_to_ensemble(x -> x[7], ensamble, 0.05)
plot(ts, res, ribbon = qt)
moving_average(vs,n) = [sum(@view vs[i:(i+n-1)])/n for i in 1:(length(vs)-(n-1))]
