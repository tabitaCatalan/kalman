function episystem_full(x, α, p)
    γᵢ = p.gammai; γₑ = p.gammae; φ = p.phi;
    pᵢₘ = p.pim; pᵢ = p.pin; pₑ = p.pe;
    [-x[1]*α*(pᵢ*x[4] + pᵢₘ*x[3] + pₑ*x[2]),
    x[1]*α*(pᵢ*x[4] + pᵢₘ*x[3] + pₑ*x[2]) - (x[2]*γₑ),
                    x[2]*γₑ*(1 - φ) - (x[3]*γᵢ),
                          x[2]*γₑ*φ - (x[4]*γᵢ),
                               γᵢ*(x[3] + x[4]),
                                    x[2]*γₑ*φ]
end

function epijacobian_full_x(x, α, p)
    γᵢ = p.gammai; γₑ = p.gammae; φ = p.phi;
    pᵢₘ = p.pim; pᵢ = p.pin; pₑ = p.pe;
    [-α*(pᵢ*x[4] + pᵢₘ*x[3] + pₑ*x[2])      -pₑ*x[1]*α      -pᵢₘ*x[1]*α     -pᵢ*x[1]*α  0   0;
    α*(pᵢ*x[4] + pᵢₘ*x[3] + pₑ*x[2])        (pₑ*x[1]*α-γₑ)  pᵢₘ*x[1]*α      pᵢ*x[1]*α   0   0;
                             0              γₑ*(1 - φ)      -γᵢ             0           0   0;
                             0              γₑ*φ            0               -γᵢ         0   0;
                             0              0               γᵢ              γᵢ          0   0;
                             0              γₑ*φ            0               0           0   0]

end

function epijacobian_full_u(x, α, p)
    pᵢₘ = p.pim; pᵢ = p.pin; pₑ = p.pe;
    [-x[1]*(pᵢ*x[4] + pᵢₘ*x[3] + pₑ*x[2]),
    x[1]*(pᵢ*x[4] + pᵢₘ*x[3] + pₑ*x[2]),
                              0,
                              0,
                              0,
                              0]
    
end

function next_generation_matrix(x, α, p)
    S = x[1]
    γᵢ = p.gammai; γₑ = p.gammae; φ = p.phi;
    pᵢₘ = p.pim; pᵢ = p.pin; pₑ = p.pe;
    [S*pₑ*α/γₑ  S*pᵢₘ*α/γᵢ  S*pᵢ*α/γᵢ;
          1 - φ 0           0;
              φ 0           0]
end  


"""
    caract_poli_coefs(x, α, p)

Devuelve los coeficientes del polinomio característico simplificado
de la matriz de próxima generación.

Se sabe que ``\\lambda = 0`` es valor propio. 
El polinomio caracteristico (simplificando por ``\\lambda``) es 
```math 
-λ^2 + (α λ p_1 S)/γ_1 + (α p_3 S ϕ)/γ_3 + (1 - ϕ)(α p_2 S)/γ_2
```
donde `S = x[1]`, y los demás son parámetros que vienen dentro de `p`.
lo dejo de la forma aλ² + b λ + c  
Devuelve `b, c`.
"""
function caract_poli_coefs(x, α, p)
    γᵢₘ = γᵢ = p.gammai; γₑ = p.gammae; φ = p.phi;
    pᵢₘ = p.pim; pᵢ = p.pin; pₑ = p.pe;
    S = x[1];

    # El polinomio caracteristico (simplificando por λ) es 
    # -λ^2 + (α λ p_1 S)/γ_1 + (α p_3 S ϕ)/γ_3 + (1 - ϕ)(α p_2 S)/γ_2
    # lo dejo de la forma aλ² + b λ + c 
    # a = -1 
    b = α * pₑ * S / γₑ 
    c = φ * α * pᵢ * S / γᵢ + (1-φ) * α * pᵢₘ * S / γᵢₘ 
    b, c 
end 

function Rt_value(x, α, p, total) 
    b, c = caract_poli_coefs(x, α, p)
    # la solución es (-b ± √(b^2 + 4c))/(2a)
    S = x[1]
    Δ = b^2 + 4 * c 

    # λ₂ = (b - √Δ)/2
    # λ₃ = (b + √Δ)/2

    if Δ > 0  
        r = (b + √Δ)/2 * (S/total)
        #println("(b + √Δ)/2 = ", (b + √Δ)/2)
    else # Δ ≤ 0
        r = sqrt(b^2/4 + - Δ/4) * (S/total) 
        #println("((b ± √Δ)/ 2) = ", b/2, " ± ", sqrt(-Δ)/2, "i" )
    end
    r 
end

function Rt_DsDa(x, α, p, total)
    b, c = caract_poli_coefs(x, α, p)
    # la solución es (-b ± √(b^2 + 4c))/(2a)
    S = x[1]
    Δ = b^2 + 4 * c 

    γᵢₘ = γᵢ = p.gammai; γₑ = p.gammae; φ = p.phi;
    pᵢₘ = p.pim; pᵢ = p.pin; pₑ = p.pe;

    if Δ > 0  
        Ds = S*(0.25(2S*(pₑ^2)*(α^2)*(γₑ^-2) + 4pᵢ*α*φ*(γᵢ^-1) + 4pᵢₘ*α*(γᵢₘ^-1)*(1 - φ))*(sqrt((S^2)*(pₑ^2)*(α^2)*(γₑ^-2) + 4S*pᵢ*α*φ*(γᵢ^-1) + 4S*pᵢₘ*α*(γᵢₘ^-1)*(1 - φ))^-1) + 0.5pₑ*α*(γₑ^-1)) + 0.5sqrt((S^2)*(pₑ^2)*(α^2)*(γₑ^-2) + 4S*pᵢ*α*φ*(γᵢ^-1) + 4S*pᵢₘ*α*(γᵢₘ^-1)*(1 - φ)) + 0.5S*pₑ*α*(γₑ^-1)
        Da = S*(0.25(4S*pᵢ*φ*(γᵢ^-1) + 4S*pᵢₘ*(γᵢₘ^-1)*(1 - φ) + 2α*(S^2)*(pₑ^2)*(γₑ^-2))*(sqrt((S^2)*(pₑ^2)*(α^2)*(γₑ^-2) + 4S*pᵢ*α*φ*(γᵢ^-1) + 4S*pᵢₘ*α*(γᵢₘ^-1)*(1 - φ))^-1) + 0.5S*pₑ*(γₑ^-1))
    else 
        Ds = (1//2)*S*(-pᵢ*α*φ*(γᵢ^-1) - (pᵢₘ*α*(γᵢₘ^-1)*(1 - φ)))*(sqrt(-S*pᵢ*α*φ*(γᵢ^-1) - (S*pᵢₘ*α*(γᵢₘ^-1)*(1 - φ)))^-1) + sqrt(-S*pᵢ*α*φ*(γᵢ^-1) - (S*pᵢₘ*α*(γᵢₘ^-1)*(1 - φ)))
        Da = (1//2)*S*(-S*pᵢ*φ*(γᵢ^-1) - (S*pᵢₘ*(γᵢₘ^-1)*(1 - φ)))*(sqrt(-S*pᵢ*α*φ*(γᵢ^-1) - (S*pᵢₘ*α*(γᵢₘ^-1)*(1 - φ)))^-1)
    end
    Ds/total , Da/total
end