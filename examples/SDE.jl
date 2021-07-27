using Random
using DifferentialEquations
using Plots
using KalmanFilter

#cd("KalmanFilter\\examples")

include("examples\\seii_model_v2.jl")

include("examples\\common_defs.jl")

#plot(ts, control_pieces.(ts))

function forcingF(dx,x,p,t)
    dx .= episystem_full(x, control_pieces(t), p)
    #dx .= episystem_full(x, 0., p)
end

function noise(dx,x,p,t)
    dx .= 0.0001 * x 
end



sdeprob = SDEProblem(forcingF, noise, x0, (0.,T))
sdesol = solve(sdeprob, p = p, saveat = dt);
#plot(sdesol, vars=(0,2:5)) 

img_folder = pwd() * "\\img\\presentacion-investigadores\\"
#caso = "sintetico-controlgrande-gammae0_7"
caso = "datosreales-gammae0_14"
formato = ".svg"

plot(sdesol, vars = (0,1), label = "S(t)", ylims = (0,7.5e6))
savefig(img_folder * caso * "S-real" * formato)
plot(sdesol, vars = (0,2:4), label = ["E(t)" "Iᵐ(t)" "I(t)"], legend = :topleft)
savefig(img_folder * caso * "EImI-real" * formato)
plot(sdesol, vars = (0,[5,6]), label = ["R(t)" "C(t)"])
savefig(img_folder * caso * "RC-real" * formato )

plot(ts, control_pieces.(ts), label = "Tasa de contagio α(t)", ylims = (0.,2.5e-7),)
savefig(img_folder * caso * "alpha-real" * formato )

R0real = [R_0(sdesol[:, i], control_pieces(ts[i]), p) for i in 1:(size(sdesol)[2]-1)];
plot(ts,R0real, title = "Rₜ", ylims = (0.,2.5),label =:none)
hspan!([1., 0.], alpha = 0.2, label = "Rₜ ≤ 1", c = :green)
savefig(img_folder * caso * "Rt-real" * formato )

observaciones = sdesol'[:, [2,6]] 
observaciones = sdesol'[:, 6:6]  
observaciones = [sdesol'[1:end-1,6] R0real]

