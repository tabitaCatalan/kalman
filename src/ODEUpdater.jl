#using KalmanFilter
using ComponentArrays

# Usar misma estructura que differential equations... preservar interfaz
mutable struct ODEForecaster <: KalmanFilter.KalmanUpdater
    dt 
    n_steps::Int 
    Xn::ComponentArray 
    discret_system::KalmanFilter.Discretizer
    discret_momentum::KalmanFilter.Discretizer
    """Función que corrige un estado `x` para dejarlo dentro de cierto dominio."""
    integrity
end






function ODEForecaster(dt, n_steps, x0, F, f, Dxf, p, integrity)
    P0 = F * F'
    discret_system = KalmanFilter.RK4Dx(f, Dxf, p, dt/n_steps)
    discret_momentum = momentum(discret_system, F)
    X0 = ComponentArray(x=x0, P= F* F')
    ODEForecaster(dt, n_steps, X0, discret_system, discret_momentum, integrity)
end 

# tengo que asegurar que el discretizer usa un dt más chico... 

#small_dt(odefor::ODEForecaster) = odefor.dt / odefor.n_steps 

function next_Xn!(Xn, odefor::ODEForecaster, control)
    set_Xn!(odefor, odefor.discret_momentum(Xn, control))
end 

function set_Xn!(odefor::ODEForecaster, Xn)
    odefor.Xn.x .= Xn.x
    odefor.Xn.P .= Xn.P 
end 

## Para darle la interfaz de KalmanUpdater 
function update!(odefor::ODEForecaster, hatx, hatP, control) 
    set_Xn!(odefor, ComponentArray(x = hatx, P = hatP))
    (p, F) = odefor.discret_momentum.p 
    odefor.discret_momentum = momentum(odefor.discret_system, F)
end

function forecast(odefor::ODEForecaster, hatx, hatP, control)
    X_actual = ComponentArray(x = hatx, P = hatP)
    for n in 1:odefor.n_steps
        next_Xn!(X_actual, odefor, control)
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

# cuento con ds::Discretizer 
function momentum(ds::KalmanFilter.Discretizer, F)    
    function f(X, a, tildep)
        p, F = tildep
        dx = ds.f(X.x, a, p)
        dxFP = ds.Dxf(X.x, a, p) * X.P
        ComponentArray(x = dx, P = dxFP + dxFP' + F * F')
    end 
    tildep = (ds.p, F)
    KalmanFilter.SimpleRK4(f, tildep, ds.dt)
end 




