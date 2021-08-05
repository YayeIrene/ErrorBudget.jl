function cant(gun::Gun)
    h = gun.QE*sind(gun.Φ)
    v =gun.QE*(1-cosd(gun.Φ))
    return h,v
end

function windEffect(target::AbstractTarget, proj::AbstractPenetrator, gun::Gun,wind::Array{Float64,1})
    gun.QE,gun.AZ = QEfinderMPMM!(target, proj, gun)
    u0 = iniCond(gun, proj.calibre)
    proj.position = [u0[1],u0[2],u0[3]]
    proj.velocity = [u0[4],u0[5],u0[6]]
    impactBaseline=trajectoryMPMM(proj, target, gun)[2]
    impactWind=trajectoryMPMM(proj, target, gun, wind)[2]
    return impactBaseline-impactWind

end
function jump(proj::Projectile, target::AbstractTarget,jumpTable::Array{Float64,1})
    range = euclidean(proj.position, target.position)
    LinearInterpolation(range, jumpTable, extrapolation_bc=Flat())
end
function fc(proj::Projectile,target::AbstractTarget,fcTable::Array{Float64,1})
    range = euclidean(proj.position, target.position)
    LinearInterpolation(range, fcTable, extrapolation_bc=Flat())
end

function variableBias(gun::Gun,target::AbstractTarget,proj::AbstractPenetrator)
    QE,AZ = QEfinderMPMM!(target, proj, gun)
    gun.QE = QE
    gun.AZ = AZ
    cantH,cantV=cant(gun, target, proj)
    wind=windEffect(target, proj, gun, wind)
    jumpError = jump(proj,target,jumpTable)
    fcError=fc(proj,target,fcTable)
    #boresight error is input and is constant

end
