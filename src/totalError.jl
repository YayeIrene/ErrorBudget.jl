function μ(gun::Gun, target::AbstractTarget)
    μx = paralax(gun,target)[1]*target.position
    μy = paralax(gun,target)[2]*target.position
    return μx,μy
end
