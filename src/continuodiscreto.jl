"""
Este archivo contiene códigos para trabajar con sistemas 
filtro continuo-discreto.
"""

using ComponentArrays
using LinearAlgebra


"""
Para trabajar con un sistema de la forma 
```math 
dx = f(x,u,p)dt + g(x)dw
```

donde ``w_t`` es un browniano (``w_t - w_s \\sim \\mathcal{N}(0, t-s)``, etc). 

# Interfaz 

Deben estar definido un método de la forma 
```julia 
function (CD::ContinuousDiscretMomentum)(X::ComponentArray, u, p, t) 
    ...
end 
```
donde `X` tiene los campos `x`, `P`. Debería retornar un `ComponentArray` de la misma forma.

Este método debe retornar la aproximación de dinámica de las ecuaciones de momentum
evaluadas en `X`, `p`, `t`.
    (CD::ContinuousDiscretMomentum)(X::ComponentArray, u, p, t) 
Las ecuaciones de momentum son 

```math 
\\begin{aligned}
\frac{dm}{dt} &= \\mathbb{E}\\big[ f(x,t) \\big] \\
\frac{dP}{dt} &= \\mathbb{E}\\big[ (x-m)f^{\\top}(x,t) \\big] + \\mathbb{E}\\big[f(x,t)(x-m)^{\\top} \\big] + \\mathbb{E}\\big[ g(x) Q g(x)^\\top \\big] 
\\end{aligned}
```

Adicionalemente, la ecuación 
```math 
\frac{dC_k}{dt} &= C_k P^{-1}\\mathbb{E}\\big[f(x,t)(x-m)^{\\top} \\big] 
```
"""
abstract type ContinuousDiscretMomentum end 

"""
The `UnscentedMomentum` approximates the momentum equation by using the Unscented Transform.
Los parámetros 
`alpha`, `beta`, `lambda`: corresponden a parámetros ``\\alpha, \\beta, \\gamma``
    de la transformación *unscented*. 
"""
struct UnscentedMomentum <: ContinuousDiscretMomentum
    f
    Q::AbstractMatrix{Float64}
    g
    alpha::Float64
    beta ::Float64
    lambda::Float64
end

function UMomentum(f, Q, g = I, alpha = 1., beta = 0., lambda = 2.)
    UnscentedMomentum(f, Q, g, alpha, beta, lambda)
end

function (UM::UnscentedMomentum)(X::ComponentArray, u, p, t) 
    points, w_μ, w_Σ, sqrt_P = unscented_transform_sqrtP(UM, X)
    N = length(w_μ) # 2n+1
    dmdt = sum(w_μ[i] * UM.f(points[:,i], u, p, t) for i in 1:N)
    dPdt = sum(covariance_approx(sqrt_P, points[:,i], w_Σ[i], UM.f, u, p, t, (x) -> UM.g(x) * UM.Q * UM.g(x)') for i in 1:N)
    Ef = sum(Efx_mt(sqrt_P, points[:,i], w_Σ[i], UM.f, u, p, t) for i in 1:N)
    ComponentArray(x = dmdt, P = dPdt, Ck = X.Ck * inv(X.P) * Ef)
end

function Efx_mt(sqrt_P, point, w_Σ_i, f, u, p, t)
    aux = sqrt_P * point 
    fx =  f(point, u, p, t)
    w_Σ_i * fx * aux'
end 

function covariance_approx(sqrt_P, point, w_Σ_i, f, u, p, t, noise)
    aux = sqrt_P * point 
    fx =  f(point, u, p, t)
    w_Σ_i * (aux * fx' + fx * aux' + noise(point))
end


# definir f, noise. noise(x,t) = G(x, t)Q(t)G(x,t)'
#discret_moment_ukf = KalmanFilter.SimpleRK4((X,u,p) -> moment_eq_ukf(X, f, u,p, noise))

function unscented_transform_sqrtP(UM::UnscentedMomentum, X::ComponentArray)
    unscented_transform_sqrtP(X.x, X.P, UM.lambda, UM.alpha, UM.beta)
end

#= Copiado desde 
https://github.com/sisl/GaussianFilters.jl/blob/master/src/ukf.jl
con algunas modificaciones
=# 
"""
    unscented_transform_sqrtP(μ, Σ, λ, α, β)
The unscented transform is an approximation of the result of applying a given
nonlinear transformation to a probability distribution that is characterized
only in terms of its mean `μ` and covariance matrix `Σ`.
# Arguments 
- `μ`: mean 
- `Σ`: covariance matrix, must be semidefinite positive.
- `λ`, `α`: related with the spread of the `σ`-points
- `β`: encodes prior info of the distribution 
Falta agregar algo que diga qué es cada parámetro 
# Returns 
A 4-Tuple with the following components 
- `points`: `σ`-points 
- `w_μ`: weights 
- `w_Σ`: weights 
- `s`: 
"""
function unscented_transform_sqrtP(μ, Σ, λ, α, β)
    n = length(μ)

    # compute weights
    w_μ = 1/(2.0*(n+λ))*ones(2*n+1)
    w_μ[1] = λ/(λ+n)
    w_Σ = copy(w_μ)
    w_Σ[1] += (1 - α^2 + β) # Per ProbRob formulation

    # sqrt of P 
    s = cholesky(Σ).L

    points = Array{Float64,2}(undef, n, 2n+1)
    points[:,1] .= μ
    for i in 1:n
        points[:,2i].= μ + sqrt(n+λ)*s[:,i] 
        points[:,2i+1] .= μ - sqrt(n+λ)*s[:,i] 
        #push!(points, μ + sqrt(n+λ)*s[:,i])
        #push!(points, μ - sqrt(n+λ)*s[:,i])
    end

    return points, w_μ, w_Σ, s
end


"""
Para trabajar con un sistema continuo discreto, usando filtro extendido 
"""
struct ExtendedMomentum <: ContinuousDiscretMomentum
    """Función ``f(x, u, p, t)``"""
    f 
    """Función ``D_xf(x, u, p, t)``"""
    Dxf 
    g
    """Función que puede ser evaluada en `x`""" 
    Q
end

function (EM::ExtendedMomentum)(X::ComponentArray, u, p, t)
    dx = EM.f(X.x, u, p, t)
    dxFP = EM.Dxf(X.x, u, p, t) * X.P
    gx = EM.g(X.x)
    ComponentArray(x = dx, P = dxFP + dxFP' + gx * EM.Q * gx', Ck = X.Ck * inv(X.P) * dxFP')
end