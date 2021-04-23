# Test discretizador

using Test

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


rk = KalmanFilter.RK4(episystem_full, epijacobian_full_x, p, dt)


x0 = [100., 10., 0., 0., 0.]

xp0 = rk.f(x0, 1., p)
x1 = rk(x0, 1.)


@test xp0[1] <= 0
@test xp0[2] <= 0
@test xp0[3] >= 0
@test xp0[4] >= 0
@test xp0[5] >= 0

@test x1[1] <= x0[1]
@test x1[2] <= x0[2]
@test x1[3] >= x0[3]
@test x1[4] >= x0[4]
@test x1[5] >= x0[5]
