function layError(proj::projectile, target::AbstractTarget)
    range = euclidean(proj.position, target.position)
    lay = 300/range+0.05
end

function randomError()
end
