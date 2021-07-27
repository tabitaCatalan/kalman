using LinearAlgebra
using Random, Distributions

# Definimos una función que permita general fácilmente condiciones iniciales 
# que se desvían de manera gausiana de la condición inicial dada.
"""
Permite generar una muestra de condiciones iniciales ``x_{0}^{(1)}, \\dots, x_{0}^{(N)} ``

Genera una muestra a partir de un vector ``x0 = (S_0, E_0, I^m_0, I_0, cI_0)``. Los
elementos de la muestra suman la misma cantidad de personas. La muestra se obtiene
generando valores al azar que distribuyen ``\\mathcal{N}(E_0, \\beta E_0)``.
- `x0`: ``x_0``
- `factor`: ``\\beta``
- `N`: Cantidad de muestras ``N``.
"""
function get_initial_states(x0, factor, N, a0,unknowninput = false)
    total = sum(x0[1:5])
    pos_norm_vector(index) = max.(rand(Normal(x0[index], x0[index] * factor), N),0.)
    expuestos = pos_norm_vector(2)
    infect_mild = pos_norm_vector(3)
    infect = pos_norm_vector(4)
    recup = pos_norm_vector(5)
    acu = pos_norm_vector(6)
    susceptibles = total .- (expuestos + infect + infect_mild + recup + acu)
    if unknowninput
        states = [[susceptibles[i], expuestos[i], infect_mild[i], infect[i], recup[i], acu[i], max(rand(Normal(a0, a0 * factor)),0.)] for i in 1:N]
    else
        states = [[susceptibles[i], expuestos[i], infect_mild[i], infect[i], recup[i], acu[i]] for i in 1:N]
    end
    states
end

# Definimos las matrices ``\tilde{H}`` para nuestro sistema auxiliar, que incluirá al input.
tildeH = [H 0.]

# Trabajaremos con un estado aumentado ``\tilde{x} = \begin{pmatrix}x \\ \alpha \end{pmatrix}``,
# que consistirá simplemente en el estado ``x`` al que le agregaremos la tasa de contagio ``\alpha``
# como si fuera una nueva variable de estado. Su dinámica será ``\alpha' = 0``. 
# Definimos también una condición inicial `a₀` y valor de incerteza para esa condición, que llamaremos 
# `Fₐ`.
a₀ = 2.5e-7; Fₐ = 1e-8
tildex0 = [x0; a₀]

# Trabajar con un estado aumentado nos obliga a usar matrices más grandes para la dinámica.
tildeF = (x) -> [F(x[1:6]) zeros(dimensions); zeros(dimensions)' Fₐ]
tildeP0 = tildeF(x0) * tildeF(x0)'
isposdef(tildeP0) # true 

ensemble_size = 50
initial_states = get_initial_states(x0, 1.5, ensemble_size, a₀, true)
