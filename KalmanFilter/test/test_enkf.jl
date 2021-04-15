# Necesito testear que se está calculando bien la covarianza. 
# Pero parece ser que no viene de ahí el problema 

using Statistics
using Test 
states = [
    [2., 3, 5., 6., 1.], 
    [2.2, 3.4, 5.1, 6.7, 1.2], 
    [2.5, 3.1, 5.6, 6.4, 1.6]
]

N = length(states)

mean(states) 

get_value(i,j) = states[i][j] 
get_coor(j) = get_value.(1:N, j)

@test mean(states) == [sum(get_coor(j))/N for j in 1:5]

dot(a, b) = sum(a .* b)


@test dot([1, 2, -1], [0, 1, 3]) == -1


function covar(i,j)
    dot(get_coor(i) .- mean(states)[i], get_coor(j) .- mean(states)[j])/(N-1)
end

@test covar(1,3) == covar(3, 1)
@test cov(states) == [covar(i,j) for i in 1:5, j in 1:5]


