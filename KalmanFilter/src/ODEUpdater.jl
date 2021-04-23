#using KalmanFilter

# Usar misma estructura que differential equations... preservar interfaz
mutable struct ODEForecaster <: KalmanFilter.KalmanUpdater
    dt 
    n_steps::Int 
    xn
    Pn 
    discret_hatx::KalmanFilter.Discretizer
    discret_hatP::KalmanFilter.Discretizer
    """Función que corrige un estado `x` para dejarlo dentro de cierto dominio."""
    integrity
end

function ODEForecaster(dt, n_steps, x0, F, f, Dxf, p, integrity)
    P0 = F * F'
    discret_hatx = KalmanFilter.RK4Dx(f, Dxf, p, dt/n_steps)
    discret_hatP = momentumP(discret_hatx, n_steps, P0, F)
    ODEForecaster(dt, n_steps, copy(x0), P0, discret_hatx, discret_hatP, integrity)
end 

# tengo que asegurar que el discretizer usa un dt más chico... 

#small_dt(odefor::ODEForecaster) = odefor.dt / odefor.n_steps 

function next_xn!(xn, odefor::ODEForecaster, control)
    xn .= odefor.discret_hatx(xn, control)
end 
function next_Pn!(Pn, odefor::ODEForecaster, control)
    Pn .= odefor.discret_hatP(Pn, control)
end 

function set_xn!(odefor::ODEForecaster, xn)
    odefor.xn .= xn 
end 
function set_Pn!(odefor::ODEForecaster, Pn)
    odefor.Pn .= Pn 
end 

## Para darle la interfaz de KalmanUpdater 
function update!(odefor::ODEForecaster, hatx, hatP, control) 
    set_xn!(odefor, hatx)
    set_Pn!(odefor, hatP)
    (p, P, F) = odefor.discret_hatP.p 
    odefor.discret_hatP = momentumP(odefor.discret_hatx, odefor.n_steps, hatP, F)
end

function forecast(odefor::ODEForecaster, hatx, hatP, control)
    x_actual = copy(odefor.xn); P_actual = copy(odefor.Pn)
    for n in 1:odefor.n_steps
        next_xn!(x_actual, odefor, control)
        next_Pn!(P_actual, odefor, control)
    end 
    odefor.integrity(x_actual), P_actual
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
function momentumP(ds::KalmanFilter.Discretizer, n_steps, P, F)    
    function f(x, a, tildep)
        p, P, F = tildep
        dxFP = ds.Dxf(x, a, p) * P
        dxFP + dxFP' + F * F'
    end 
    tildep = (ds.p, P, F)
    KalmanFilter.SimpleRK4(f, tildep, ds.dt)
end 




