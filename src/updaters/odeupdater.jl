#using KalmanFilter
using ComponentArrays
using LinearAlgebra

# Usar misma estructura que differential equations... preservar interfaz
mutable struct ODEForecaster <: KalmanUpdater
    dt 
    n_steps::Int 
    #Xn::ComponentArray # creo que es innececsario
    discret_momentum::Discretizer
    """Función que corrige un estado `x` para dejarlo dentro de cierto dominio."""
    integrity
end

"""
Outer constructor
"""
function ODEForecaster(dt, n_steps, x0, P0, p, integrity, momentum::ContinuousDiscretMomentum)
    small_dt = dt/n_steps
    #discret_system = KalmanFilter.RK4Dx(f, Dxf, p, small_dt)
    #Q = Diagonal(sqrt(small_dt) * ones(length(x0)))
    #X0 = ComponentArray(x=x0, P= P0)  (x,\\alpha, p, t)
    discret_momentum = KalmanFilter.SimpleRK4(momentum, p, small_dt)
    ODEForecaster(dt, n_steps, discret_momentum, integrity)
end 

dt(updater::ODEForecaster) = updater.dt 

# tengo que asegurar que el discretizer usa un dt más chico... 

#small_dt(odefor::ODEForecaster) = odefor.dt / odefor.n_steps 

function next_Xn(Xn, odefor::ODEForecaster, control, t)
    odefor.discret_momentum(Xn, control, t)
end 
#=
function set_Xn!(odefor::ODEForecaster, Xn)
    set_Xn!(odefor.Xn, Xn)
end 

function set_Xn!(Xn::ComponentArray, Xnew)
    Xn.x .= Xnew.x
    Xn.P .= Xnew.P    
end
=#

#= Para darle la interfaz de KalmanUpdater 
function update!(odefor::ODEForecaster, hatx, hatP, control) 
    set_Xn!(odefor, ComponentArray(x = hatx, P = hatP))
end =#
function update!(odefor::ODEForecaster, hatx, hatP, control, t) end

function apply_integrity!(X, odefor)
    X.x .= odefor.integrity(X.x)
end

function forecast(odefor::ODEForecaster, hatx, hatP, control, t)
    X_actual = ComponentArray(x = hatx, P = hatP, Ck = hatP)
    for n in 1:odefor.n_steps
        X_actual = next_Xn(X_actual, odefor, control, t) 
    end 
    apply_integrity!(X_actual, odefor)
    X_actual
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





