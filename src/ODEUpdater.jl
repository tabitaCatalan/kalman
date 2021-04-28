#using KalmanFilter
using ComponentArrays
using LinearAlgebra

# Usar misma estructura que differential equations... preservar interfaz
mutable struct ODEForecaster <: KalmanFilter.KalmanUpdater
    dt 
    n_steps::Int 
    Xn::ComponentArray # creo que es innececsario
    discret_system::KalmanFilter.Discretizer
    discret_momentum::KalmanFilter.Discretizer
    """Función que corrige un estado `x` para dejarlo dentro de cierto dominio."""
    integrity
end

"""
Outer constructor
"""
function ODEForecaster(dt, n_steps, x0, P0, F, f, Dxf, p, integrity)
    small_dt = dt/n_steps
    discret_system = KalmanFilter.RK4Dx(f, Dxf, p, small_dt)
    Q = Diagonal(sqrt(small_dt) * ones(length(x0)))
    X0 = ComponentArray(x=x0, P= P0)
    momentum_eq = (X, a, p) -> momentum_equation(discret_system, X.x, X.P, a, p, F(X.x), Q)
    discret_momentum = KalmanFilter.SimpleRK4(momentum_eq, p, small_dt)
    ODEForecaster(dt, n_steps, X0, discret_system, discret_momentum, integrity)
end 

# tengo que asegurar que el discretizer usa un dt más chico... 

#small_dt(odefor::ODEForecaster) = odefor.dt / odefor.n_steps 

function next_Xn(Xn, odefor::ODEForecaster, control)
    odefor.discret_momentum(Xn, control)
end 

function set_Xn!(odefor::ODEForecaster, Xn)
    set_Xn!(odefor.Xn, Xn)
end 

function set_Xn!(Xn::ComponentArray, Xnew)
    Xn.x .= Xnew.x
    Xn.P .= Xnew.P    
end

## Para darle la interfaz de KalmanUpdater 
function update!(odefor::ODEForecaster, hatx, hatP, control) 
    set_Xn!(odefor, ComponentArray(x = hatx, P = hatP))
end

function forecast(odefor::ODEForecaster, hatx, hatP, control)
    X_actual = ComponentArray(x = hatx, P = hatP)
    for n in 1:odefor.n_steps
        set_Xn!(X_actual, next_Xn(X_actual, odefor, control))
    end 
    odefor.integrity(X_actual.x), X_actual.P
end 
# confío en que no la van a llamar... solo debería usarse con measurements,
# NO con InnerState  
function (updater::ODEForecaster)(x::AbstractArray, u::Real, error)end

# Necesito agregar una ODE para el sistema de la matriz P 
# ```math
# dP/dt = \frac{\partial F(t, \hat{x})}{\partial \hat{x}} P + P \left( \frac{\partial F(t, \hat{x})}{\partial \hat{x}} \right)^{T} + G(t) Q(t) G^{T}(t)
# ```

# Necesito un discretizador con la edo de P(t)
# Lo bueno que cada dicretizador derivable tiene esa infoooooo 
# (ds::Discretizer, x, α)

#=
"""
` tildep`. tupla que puede descomponerse en `(ds.p, F, Q)`, donde: 
    - `F`: evaluada en `x` da la dispersión
    - `Q`: matriz de covarianzas del ruido 
"""
function momentum(ds::KalmanFilter.Discretizer, F, Q)
    function f(X, a, tildep)
        p, F, Q = tildep
        dx = ds.f(X.x, a, p)
        dxFP = ds.Dxf(X.x, a, p) * X.P
        ComponentArray(x = dx, P = dxFP + dxFP' + F(x) * Q * F(x)')
    end 
    tildep = (ds.p, F, Q)
    KalmanFilter.SimpleRK4(f, tildep, ds.dt)
end 
=#

function momentum_equation(ds::KalmanFilter.Discretizer, x, P, a, p, Fx, Q)
    dx = ds.f(x, a, p)
    dxFP = ds.Dxf(x, a, p) * P
    ComponentArray(x = dx, P = dxFP + dxFP' + Fx * Q * Fx')
end 




