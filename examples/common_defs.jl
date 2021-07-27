using LinearAlgebra
# Definimos un vector de condiciones iniciales ``x_0``.

x0 = [7e6, 10., 0.1, 0.1, 0.1, 0.1]

dispersion(ν, x) = Diagonal(ν .* x)

F = (x) -> dispersion(0.001 * ones(length(x)), x)
#F = (x) -> dispersion([1e3, 1e1, 1e1, 1e1, 1e1, 1e1], ones(6))
P0 = F(x0) * F(x0)
# Solo consideraremos conocida la cantidad total de infectados ``\mathbf{x}_5 = cI``.
H = [0. 0. 0. 0. 0. 1.]

# Usaremos un error de unas 50.000 personas en las mediciones.
G = [10.]
dimensions = 6

# Definimos el intervalo y los tiempos de medición 

dt = 1.#0.1 # Intervalo de sampleo de observaciones 
T = 450. # Se resolverá el problema en el intervalo [0,T]
N = Int(T/dt) # número de mediciones 
ts = 0.:dt:(T-dt) # grilla de tiempos 

# Definimos los parámetros que usaremos para resolver la EDO 
#γₑ = 1/14 # tasa de paso de expuesto a infectado 
γₑ = 1/7 # tasa de paso de expuesto a infectado 
γᵢ = 1/7 # tasa de recuperación 
φ = 0.4 # fracción de infectados que son detectados (sintomáticos)
pₑ = 0.1
pᵢ = 0.5
pᵢₘ = 0.2
# Que agrupamos en una tupla `p`
p = (gammae = γₑ, gammai = γᵢ, phi = φ, pe = pₑ, pin = pᵢ, pim = pᵢₘ)

# Por comodidad definimos una función para hacer una interpolación lineal 
"""
    linear_interpolol(t, t₁, t₂, v₁, v₂)
Interpola el valor de una recta en `t`, dado que en el punto `t₁` toma 
el valor `v₁` y que en `t₂` toma el valor `v₂`.
"""
linear_interpol = (t, t₁, t₂, v₁, v₂) -> (v₂ - v₁)*(t - t₁)/(t₂ - t₁) + v₁

# Definimos un control lineal por pedazos 
function control_pieces(t)
    # control grande γₑ = 1/7 (con 1/14 funciona feo)
    t₁ = 5.; t₂ = 30.; t₃ = 150.; t₄ = 180.; t₅ = 250.; t₆ = 400.; t₇ = 400.
    v₁ = 0.2e-6; v₂ = 0.08e-6; v₃ = 0.05e-6; v₄ = 0.12e-6;  v₅ = v₄
    # control chico γₑ = 1/7
    #t₁ = 5.; t₂ = 30.; t₃ = 120.; t₄ = 180.; t₅ = 230.; t₆ = 320.; t₇ = 400.
    #v₁ = 0.2e-6; v₂ = 0.075e-6; v₃ = 0.04e-6; v₄ = 0.08e-6; v₅ = 0.04e-6
    # control grande γₑ = 1/14 aun es feo xD 
    #t₁ = 5.; t₂ = 30.; t₃ = 150.; t₄ = 180.; t₅ = 250.; t₆ = 400.; t₇ = 400.
    #v₁ = 0.2e-6; v₂ = 0.08e-6; v₃ = 0.05e-6; v₄ = 0.12e-6;  v₅ = v₄
    # control chico γₑ = 1/14
    #t₁ = 5.; t₂ = 30.; t₃ = 120.; t₄ = 180.; t₅ = 230.; t₆ = 320.; t₇ = 400.
    #v₁ = 0.2e-6; v₂ = 0.075e-6; v₃ = 0.04e-6; v₄ = 0.057e-6; v₅ = 0.04e-6
    
    if t < t₁
        v₁
    elseif t >= t₁ && t < t₂
        linear_interpol(t, t₁, t₂, v₁, v₂)
    elseif t >= t₂ && t < t₃
        v₂
    elseif t >= t₃ && t < t₄
        linear_interpol(t, t₃, t₄, v₂, v₃)
    elseif t >= t₄ && t < t₅
        v₃
    elseif t >= t₅ && t < t₆
        linear_interpol(t, t₅, t₆, v₃, v₄)
    elseif t >= t₆ && t < t₇
        linear_interpol(t, t₆, t₇, v₄, v₅)
    elseif t >= t₇ 
        v₅
    end
end

# Definimos también una función de integridad para los estados 
function state_integrity(x, total)
    x[1:5] = x[1:5] * total/sum(x[1:5])
    x = max.(x, 0.)
    #x[7] = min(x[7], 5.)
    x
end

# Y otra para las observaciones
observation_integrity(x) = max.(x,0.)
