function layError(proj::projectile, target::AbstractTarget)
    range = euclidean(proj.position, target.position)
    lay = 300/range+0.05
end



function randomError(σdisp::Error,σaiming::Error)
    σrandom=Error(0.0,0.0)
    σrandom.horizontal = sqrt(σdisp.horizontal^2+σaiming.horizontal^2)

    σrandom.vertical = sqrt(σdisp.vertical^2+σaiming.vertical^2)

    return σrandom

end
