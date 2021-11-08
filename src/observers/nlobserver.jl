#=
Observador con una función no lineal del estado y = h(x,u,v)
=#

"""
$(TYPEDEF)
Representa un observador no lineal de la forma
```math
y_n = h(x_n, u_n) + G_n v_n
```
de un estado interno ``x``.
# Campos
$(TYPEDFIELDS)
"""
struct NLObserver<: LinearizableObserver
    """Función ``h`` posiblemente no lineal de tres entradas, que recibe un estado ``x``
    y devuelve la observación ``y``, ``y = h(x,u,v)``. ``v`` corresponde a un ruido y
    ``u`` a un control."""
    H # funcion no lineal H(x,α)
    DxH # D_x H ... eventualmente hay que agregar DaH 
    G # matriz que pondera al ruido
    R # matriz de covarianzas de ruido 
    linear::LinearObserver
    """Función que recibe una observación `y` y la corrige para dar valores razonables.
    Por ejemplo, para el caso de observar un sistema epidemiológico, no tiene sentido 
    que una variable sea negativa. Se podría definir `integrity(y) = max.(y,0.)`.
    """
    integrity
    """
    $(TYPEDSIGNATURES)
    Constructor de un observador no lineal `NLObserver`.
    Es de la forma  
    ```math
    y = \\mathcal{H}(x, \\alpha) + Gv_t 
    ```
    # Argumentos
    - `H`: función ``\\mathcal{H}``, debe poder evaluarse en `x` y `\\alpha`.
    - `DxH`: función ``D_x \\mathcal{H}``, debe poder evaluarse en `x` y `\\alpha`.
    - `x0`: valor inicial del estado (estimación)
    - `α`: valor inicial del control (de la estimación)
    - `G`: matriz de dispersión del ruido 
    - `integrity`: función que transforma un vector `y` para que cumple ciertas 
    restricciones de integridad (ser positivo, etc).
    """
    function NLObserver(H, DxH, G, R, x0, α, integrity)
        linear = linearize_x(H, DxH, x0, α, G, R)
        new(H, DxH, G, R, linear, integrity)
    end
end

function (observer::NLObserver)(x::AbstractArray, u::Real, error)
    observer.integrity(observer.H(x, u) + Gn(observer) * error)
end

# Necesito la versión linealizada de esto... tengo que definir Hn, Dn, Gn 
function linearize_x(NLob::NLObserver, x, α)
    linearize_x(NLob.H, NLOb.DxH, x, α, Gn(NLob), Rn(NLob))
end

"""
$(TYPEDEF)
# Argumentos 
- `funcH`: función ``\\mathcal{H}(x, \\alpha)`` posiblemente no lineal para observar el estado.
- `funcJxH`: ``D_x\\mathcal{H}`` jacobiano con respecto al estado.
- `x`: Vector en torno al que se va a linealizar 
- `α`: Valor del control en torno al que se va a linealizar 
- `G`: matriz de dispersión del ruido. 
- `integrity`: función de `x`, que conserva el valor dentro de un dominio.
"""
function linearize_x(funcH, funcJxH, x, α, G, R)
    H = funcJxH(x, α)
    D = funcH(x, α) - H * x 
    SimpleLinearObserver(H, D, G, R)
end
  
Hn(observer::NLObserver) = Hn(observer.linear)
Dn(observer::NLObserver) = Dn(observer.linear)
Gn(observer::NLObserver) = observer.G
Rn(observer::NLObserver) = observer.R
