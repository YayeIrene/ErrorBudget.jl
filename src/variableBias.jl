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

"""
    windCrossError(σ,wind,tank, weapon,proj, target, aero;args ...)

Returns the normal distribution for the impact points (horizontal coordinate).
    The function performs a Monte Carlo simulation by sampling the wind (cross field).
        It is assumes that the cross wind variation is a gaussian distribution with
        μ = wind.cross and standard deviation = σ.
        * tank
        * weapon : the launcher of type Gun
        * proj : the projectile
        * target
        * aero : the aerodynamic coefficients of the projectile
        One optional argument is N (the number of Monte Carlo simulations). Default = 100

"""
function windCrossError(σ::Float64,wind::Wind,tank::Tank, weapon::Gun,proj::AbstractPenetrator, target::AbstractTarget, aero::DataFrame;N=100)
    d = Normal(wind.cross,σ)
#drange = Normal(w.range,σrange)

    x = rand(d, N)
#xrange = rand(drange, 100)

    res = zeros(N)#Array{Float64}#[]
    for i=1:N

        wind.cross = x[i]
#w.range = xrange

        windIn = Cuas.wind(tank, wind)
        res[i]=trajectoryMPMM(proj, target, weapon,aero,w_bar=windIn)[1][3]
#push!(res,tmp)
    end

    σcrossₜ=fit(Normal, res)
    return σcrossₜ
end

"""
    windRangeError(σ,wind,tank, weapon,proj, target, aero;args ...)

Returns the normal distribution for the impact points (vertical coordinate).
    The function performs a Monte Carlo simulation by sampling the wind (range field).
        It is assumes that the range wind variation is a gaussian distribution with
        μ = wind.range and standard deviation = σ.
        * tank
        * weapon : the launcher of type Gun
        * proj : the projectile
        * target
        * aero : the aerodynamic coefficients of the projectile
        One optional argument is N (the number of Monte Carlo simulations). Default = 100

"""
function windRangeError(σ::Float64,wind::Wind,tank::Tank, weapon::Gun,proj::AbstractPenetrator, target::AbstractTarget, aero::DataFrame;N=100)

    d = Normal(wind.range,σ)
    x = rand(d, N)
    res = zeros(N)#Array{Float64}#[]
    for i=1:N

    #w.cross = xcross[i]
    wind.range = x[i]

    windIn = Cuas.wind(tank, wind)
    res[i]=trajectoryMPMM(proj, target, weapon,aero,w_bar=windIn)[1][2]
    #push!(res,tmp)
    end

    σrangeₜ =fit(Normal, res)

    return σrangeₜ
end

"""
    interTable(range,jumpTable, value)

Returns the 'value' from an interpolation table 'jumpTable' over the 'range'

"""
function interTable(range::Array{Float64,1},jumpTable::Array{Float64,1}, value::Float64)
    inter=LinearInterpolation(range, jumpTable, extrapolation_bc=Flat())
    inter(value)
end

"""
    rangeError(target, σ, tank, weapon, proj, aero; args)

Returns the normal distribution for elevation and azimuth angles.
    The function performs a Monte Carlo simulation by sampling the target range.
        It is assumes that the range variation is a gaussian distribution with
        μ = target.ρ and standard deviation = σ. For each sampled range, a elevation and azimuth angle is computed.
        * tank
        * weapon : the launcher of type Gun
        * proj : the projectile
        * target
        * aero : the aerodynamic coefficients of the projectile
        One optional argument is N (the number of Monte Carlo simulations). Default = 100

"""
function rangeError(target::AbstractTarget, σ::Float64, tank::Tank, weapon::Gun, proj::AbstractPenetrator, aero::DataFrame; N=100)

    d = Normal(target.ρ,σ)
    x = rand(d, N)
    QE = zeros(N)#Array{Float64}#[]
    AZ = zeros(N)
    for i=1:N
        target.ρ = x[i]
        target.position = targetPos(target, tank)

    QE[i],AZ[i]=QEfinderMPMM!(target, proj, weapon,aero)
    #println("QE", " ", QE[i])
    #println("AZ", " ", AZ[i])
    end
    σQE =fit(Normal, QE)
    σAZ = fit(Normal, AZ)


    return σQE,σAZ
end

"""
    elevationError(σ, target,proj,weapon,tank,aero;args )

Returns the normal distribution for the impact points (vertical and horizontal distribution).
    The function performs a Monte Carlo simulation by sampling the elevation angle.
        It is assumes that the elevation angle variation is a gaussian distribution with
        μ = the compute elevation angle and standard deviation = σ.
        * tank
        * weapon : the launcher of type Gun
        * proj : the projectile
        * target
        * aero : the aerodynamic coefficients of the projectile
        One optional argument is N (the number of Monte Carlo simulations). Default = 100

"""
function elevationError(σ::Float64, target::AbstractTarget,proj::AbstractPenetrator,weapon::Gun,tank::Tank,aero::DataFrame;N=100 )
    QE,AZ=QEfinderMPMM!(target, proj, weapon,aero)
    weapon.AZ = rad2deg(AZ)
    d = Normal(QE,σ)
    x = rand(d, N)
    resV = zeros(N)#Arra
    resH = zeros(N)#Arra

    for i=1:N
        weapon.QE=rad2deg(x[i])
        target.position= targetPos(target, tank)
        proj.position=muzzlePos(tank)
        proj.velocity=muzzleVel(tank)


    resV[i], resH[i]=trajectoryMPMM(proj, target, weapon,aero)[1][2:3]
    #resH[i]=trajectoryMPMM(proj, target, weapon,aero)[1][3]
    #println("QE", " ", QE[i])
    #println("AZ", " ", AZ[i])
    end
    σθv =fit(Normal, resV)
    σθh =fit(Normal, resH)


    return σθv ,σθh

end

"""
    azimuthError(σ, target,proj,weapon,tank,aero;args )

Returns the normal distribution for the impact points (vertical and horizontal distribution).
    The function performs a Monte Carlo simulation by sampling the azimuth angle.
        It is assumes that the elevation angle variation is a gaussian distribution with
        μ = the computed azimuth angle and standard deviation = σ.
        * tank
        * weapon : the launcher of type Gun
        * proj : the projectile
        * target
        * aero : the aerodynamic coefficients of the projectile
        One optional argument is N (the number of Monte Carlo simulations). Default = 100

"""
function azimuthError(σ::Float64, target::AbstractTarget,proj::AbstractPenetrator,weapon::Gun,tank::Tank,aero::DataFrame;N=100 )
    QE,AZ=QEfinderMPMM!(target, proj, weapon,aero)
    weapon.QE = rad2deg(QE)
    d = Normal(AZ,σ)
    x = rand(d, N)
    resV = zeros(N)#Arra
    resH = zeros(N)#Arra

    for i=1:N
        weapon.AZ=rad2deg(x[i])
        target.position= targetPos(target, tank)
        proj.position=muzzlePos(tank)
        proj.velocity=muzzleVel(tank)


        resV[i], resH[i]=trajectoryMPMM(proj, target, weapon,aero)[1][2:3]
        #resH[i]=trajectoryMPMM(proj, target, weapon,aero)[1][3]
    #println("QE", " ", QE[i])
    #println("AZ", " ", AZ[i])
    end
    σξv =fit(Normal, resV)
    σξh =fit(Normal, resH)


    return σξv ,σξh

end

"""
    muzzleVelError(σ, target,proj,weapon,tank,aero;args )

Returns the normal distribution for the impact points (vertical and horizontal distribution).
    The function performs a Monte Carlo simulation by sampling the muzzle velocity.
        It is assumes that the muzzle velocity variation is a gaussian distribution with
        μ = weapon.u₀ and standard deviation = σ.
        * tank
        * weapon : the launcher of type Gun
        * proj : the projectile
        * target
        * aero : the aerodynamic coefficients of the projectile
        One optional argument is N (the number of Monte Carlo simulations). Default = 100

"""
function muzzleVelError(σ::Float64, target::AbstractTarget,proj::AbstractPenetrator,weapon::Gun,tank::Tank,aero::DataFrame;N=100)
    QE,AZ=QEfinderMPMM!(target, proj, weapon,aero)
    weapon.QE = rad2deg(QE)
    weapon.AZ = rad2deg(AZ)
    target.position= targetPos(target, tank)
    proj.position=muzzlePos(tank)
    d = Normal(weapon.u₀,σ)
    x = rand(d, N)
    resV = zeros(N)#Arra
    resH = zeros(N)#Arra

    for i=1:N

        weapon.u₀ = x[i]
        proj.velocity=muzzleVel(tank)

        resV[i], resH[i]=trajectoryMPMM(proj, target, weapon,aero)[1][2:3]
        #resH[i]=trajectoryMPMM(proj, target, weapon,aero)[1][3]
    #println("QE", " ", QE[i])
    #println("AZ", " ", AZ[i])
    end
    σv =fit(Normal, resV)
    σh =fit(Normal, resH)


    return σv ,σh
end

"""
    temperatureError(σ, target,proj,weapon,tank,aero, atmosphere;args )

Returns the normal distribution for the impact points (vertical and horizontal distribution).
    The function performs a Monte Carlo simulation by sampling the air temperature.
        It is assumes that the temperature variation is a gaussian distribution with
        μ = atmosphere.t and standard deviation = σ.
        * tank
        * weapon : the launcher of type Gun
        * proj : the projectile
        * target
        * aero : the aerodynamic coefficients of the projectile
        * atmosphere: the atmospheric characteristics
        One optional argument is N (the number of Monte Carlo simulations). Default = 100

"""
function temperatureError(σ::Float64, target::AbstractTarget,proj::AbstractPenetrator,weapon::Gun,tank::Tank,aero::DataFrame,atmosphere::Air;N=100)
    QE,AZ=QEfinderMPMM!(target, proj, weapon,aero,atmosphere=atmosphere)
    weapon.QE = rad2deg(QE)
    weapon.AZ = rad2deg(AZ)
    proj.velocity=muzzleVel(tank)
    target.position= targetPos(target, tank)
    proj.position=muzzlePos(tank)
    d = Normal(atmosphere.t,σ)
    x = rand(d, N)
    resV = zeros(N)#Arra
    resH = zeros(N)#Arra

    for i=1:N

        atmosphere.t = x[i]

        resV[i], resH[i]=trajectoryMPMM(proj, target, weapon,aero,atm=atmosphere)[1][2:3]
        #resH[i]=trajectoryMPMM(proj, target, weapon,aero)[1][3]
    #println("QE", " ", QE[i])
    #println("AZ", " ", AZ[i])
    end
    σv =fit(Normal, resV)
    σh =fit(Normal, resH)


    return σv ,σh

end

function pressureError(σ::Float64, target::AbstractTarget,proj::AbstractPenetrator,weapon::Gun,tank::Tank,aero::DataFrame,atmosphere::Air;N=100)
    QE,AZ=QEfinderMPMM!(target, proj, weapon,aero,atmosphere=atmosphere)
    weapon.QE = rad2deg(QE)
    weapon.AZ = rad2deg(AZ)
    proj.velocity=muzzleVel(tank)
    target.position= targetPos(target, tank)
    proj.position=muzzlePos(tank)
    d = Normal(atmosphere.p,σ)
    x = rand(d, N)
    resV = zeros(N)#Arra
    resH = zeros(N)#Arra

    for i=1:N

        atmosphere.p = x[i]

        resV[i], resH[i]=trajectoryMPMM(proj, target, weapon,aero,atm=atmosphere)[1][2:3]
        #resH[i]=trajectoryMPMM(proj, target, weapon,aero)[1][3]
    #println("QE", " ", QE[i])
    #println("AZ", " ", AZ[i])
    end
    σv =fit(Normal, resV)
    σh =fit(Normal, resH)


    return σv ,σh

end

function gunPositionError(σ::Float64, target::AbstractTarget,proj::AbstractPenetrator,weapon::Gun,tank::Tank,aero::DataFrame,atmosphere::Air;N=100)
    QE,AZ=QEfinderMPMM!(target, proj, weapon,aero,atmosphere=atmosphere)
    weapon.QE = rad2deg(QE)
    weapon.AZ = rad2deg(AZ)
    proj.velocity=muzzleVel(tank)
    target.position= targetPos(target, tank)
    proj.position=muzzlePos(tank)
    dx = Normal(proj.position[1],σ)
    dy = Normal(proj.position[2],σ)
    dz = Normal(proj.position[3],σ)
    x = rand(dx, N)
    y = rand(dy, N)
    z = rand(dz, N)
    resV = zeros(N)#Arra
    resH = zeros(N)#Arra

    for i=1:N

        proj.position = [x[i],y[i],z[i]]

        resV[i], resH[i]=trajectoryMPMM(proj, target, weapon,aero,atm=atmosphere)[1][2:3]
        #resH[i]=trajectoryMPMM(proj, target, weapon,aero)[1][3]
    #println("QE", " ", QE[i])
    #println("AZ", " ", AZ[i])
    end
    σv =fit(Normal, resV)
    σh =fit(Normal, resH)


    return σv ,σh

end

"""
    cantError(σ, σmeas, target,proj,weapon,tank,aero, atmosphere;args )

Returns the normal distribution for the impact points (vertical and horizontal distribution).
    The function performs a Monte Carlo simulation by sampling the cant angle.
        It is assumes that the cant variation is a gaussian distribution with
        μ = tank.hull.Φ and standard deviation = σ. On this value of cant we perform another sampling to
        take into account the measurement error. We assume also a normal distribution with μ = sampled cant and
        the standard deviation = σmeas
        * tank
        * weapon : the launcher of type Gun
        * proj : the projectile
        * target
        * aero : the aerodynamic coefficients of the projectile

        One optional argument is N (the number of Monte Carlo simulations). Default = 100

"""
function cantError(σ::Float64, σmeas::Float64, target::AbstractTarget,proj::AbstractPenetrator,weapon::Gun,tank::Tank,aero::DataFrame;N=100)
    QE,AZ=QEfinderMPMM!(target, proj, weapon,aero)
    weapon.QE = rad2deg(QE)
    weapon.AZ = rad2deg(AZ)
    tank.canon.θ = rad2deg(QE)
    tank.turret.ξ = rad2deg(AZ)

    target.position = targetPos(target, tank)
    proj.position=muzzlePos(tank)
    proj.velocity=muzzleVel(tank)

    d1 = Normal(tank.hull.Φ,σ)

    x1 = rand(d1, N)

    resV1 = zeros(N)#Arra
    resH1 = zeros(N)#Arra

    for i=1:N

        #tank.hull.Φ = x[i]

        d2 = Normal(x1[i],σmeas)
        resV2 = zeros(N)#Arra
        resH2 = zeros(N)#Arra

        x2 = rand(d2, N)
        for j=1:N

            tank.hull.Φ = x2[j]

            target.position = targetPos(target, tank)
            proj.position=muzzlePos(tank)
            proj.velocity=muzzleVel(tank)

            resV2[j], resH2[j]=trajectoryMPMM(proj, target, weapon,aero)[1][2:3]
        end
        resV1[i] =fit(Normal, resV2).μ
        resH1[i] =fit(Normal, resH2).μ
        #resH[i]=trajectoryMPMM(proj, target, weapon,aero)[1][3]
    #println("QE", " ", QE[i])
    #println("AZ", " ", AZ[i])
    end
    σv =fit(Normal, resV1)
    σh =fit(Normal, resH1)



    return σv ,σh

end



"""
variableBias(target,tank, weapon, proj, aero, wind, atmosphere;args ...)


Returns the variable bias error as an objet "Error".
optional arguments are all the standard deviation of error sources: σrange, σcross_wind,σrange_wind,
σjump,σfc,σzeroing,σboresight,σMv, σtemp, σpress, σopt, σgun, σcant, σcant_meas
default values for optional parameters are nothing or nul.

        * tank
        * weapon : the launcher of type Gun
        * proj : the projectile
        * target
        * aero : the aerodynamic coefficients of the projectile
        * wind
        * atmosphere

"""
function variableBias(target::AbstractTarget,tank::Tank, weapon::Gun, proj::AbstractPenetrator, aero::DataFrame, wind::Wind, atmosphere::Air;
    σrange=nothing, σcross_wind=nothing,σrange_wind=nothing,σjump=Error(0.0,0.0),σfc=Error(0.0,0.0),σzeroing=Error(0.0,0.0),σboresight=Error(0.0,0.0),
    σMv=nothing, σtemp=nothing, σpress=nothing, σopt=Error(0.0,0.0), σgun=nothing, σcant =nothing, σcant_meas=nothing)

σvariable =Error(0.0,0.0)
σR=Error(0.0,0.0)
σcross_value = 0.0
σrange_value = 0.0
σMVv_value= 0.0
σMVh_value= 0.0
σtempV_value = 0.0
σtempH_value = 0.0
σpressV_value =0.0
σpressH_value =0.0
σgunV_value = 0.0
σgunH_value = 0.0
σcantV_value = 0.0
σcantH_value = 0.0


if isnothing(σrange)==false

dQE,dAZ=rangeError(target,σrange,tank, weapon, proj, aero)

σvEl, σhEl = elevationError(dQE.σ, target,proj,weapon,tank,aero)
σvAz, σhAz = azimuthError(dAZ.σ, target,proj,weapon,tank,aero)


#σR.horizontal = sqrt((rand(σhEl))^2+(rand(σhAz))^2)
#σR.vertical = sqrt((rand(σvEl))^2+(rand(σvAz))^2)

σR.horizontal = sqrt((σhEl.σ)^2+(σhAz.σ)^2)
σR.vertical = sqrt((σvEl.σ)^2+(σvAz.σ)^2)

end

if isnothing(σcross_wind)==false
σcross_dist= windCrossError(σcross_wind,wind,tank,weapon, proj, target, aero) #m
#σcross_value =rand(σcross_dist)
σcross_value =σcross_dist.σ
end

if isnothing(σrange_wind)==false
σrange_dist=windRangeError(σrange_wind,wind,tank,weapon, proj, target, aero) #m
#σrange_value =rand(σrange_dist)
σrange_value =σrange_dist.σ
end

if isnothing(σMv)==false
σMVvₜ, σMVhₜ = muzzleVelError(σMv, target,proj,weapon,tank,aero)

#σMVv_value = rand(σMVvₜ)
#σMVh_value = rand(σMVhₜ)
σMVv_value = σMVvₜ.σ
σMVh_value = σMVhₜ.σ
end

if isnothing(σtemp)==false
σtempV, σtempH = temperatureError(σtemp, target,proj,weapon,tank,aero,atmosphere)

#σtempV_value = rand(σtempV)
#σtempH_value = rand(σtempH)

σtempV_value = σtempV.σ
σtempH_value = σtempH.σ

end

if isnothing(σpress)==false
    σpressV, σpressH = pressureError(σpress, target,proj,weapon,tank,aero,atmosphere)

#    σpressV_value = rand(σpressV)
#    σpressH_value = rand(σpressH)

    σpressV_value = σpressV.σ
    σpressH_value = σpressH.σ
end

if isnothing(σgun)==false
    σgunV, σgunH = gunPositionError(σgun, target,proj,weapon,tank,aero,atmosphere)

#    σgunV_value = rand(σgunV)
#    σgunH_value = rand(σgunH)

    σgunV_value = σgunV.σ
    σgunH_value = σgunH.σ

end

if isnothing(σcant)==false
    σcantV, σcantH = cantError(σcant,σcant_meas, target,proj,weapon,tank,aero)

#    σcantV_value = rand(σcantV)
#    σcantH_value = rand(σcantH)

    σcantV_value = σcantV.σ
    σcantH_value = σcantH.σ
end

σvariable.horizontal = sqrt(σR.horizontal^2+σcross_value^2+σjump.horizontal^2+σfc.horizontal^2+σzeroing.horizontal^2+
σboresight.horizontal^2+σMVh_value^2+σtempH_value^2+σpressH_value^2+σopt.horizontal^2+σgunH_value^2+σcantH_value^2)

σvariable.vertical = sqrt(σR.vertical^2+σrange_value^2+σjump.vertical^2+σfc.vertical^2+σzeroing.vertical^2+σboresight.vertical^2+
σMVv_value^2+σtempV_value^2+σpressV_value^2+σopt.vertical^2+σgunV_value^2+σcantV_value^2)

return σvariable

end
