function μ(gun::Gun, target::AbstractTarget)
    μx = paralax(gun,target)[1]*target.position
    μy = paralax(gun,target)[2]*target.position
    return μx,μy
end

"""
    aiming_error(aiming)

Returns the alignment error (horizontal and vertical) for the error sources defined in 'aiming'
"""
function aiming_error(aiming::AimingError)
    σ_x =sqrt(aiming.point_def[1]^2+aiming.human[1]^2+aiming.res[1]^2+aiming.opt_path[1]^2)
    σ_y =sqrt(aiming.point_def[2]^2+aiming.human[2]^2+aiming.res[2]^2+aiming.opt_path[2]^2)
    return σ_x, σ_y
end
"""
    alignment_error(align)

Returns the alignment error (horizontal and vertical) for the error sources defined in 'align'
"""
function alignment_error(align::AlignmentError)
    σ_x=sqrt(align.boresight_acc[1]^2+align.boresight_ret[1]^2+align.sync[1]^2+align.stab[1]^2+align.gun_pos[1]^2)
    σ_y=sqrt(align.boresight_acc[2]^2+align.boresight_ret[2]^2+align.sync[2]^2+align.stab[2]^2+align.gun_pos[2]^2)
    return σ_x, σ_y
end

"""
    lof_error(lof)

Returns the line of fire error (horizontal and vertical) for the error sources defined in 'lof'
"""
function lof_error(lof::LofError)
    σ_x=sqrt(lof.fcs[1]^2+lof.lead[1]^2+lof.cant[1]^2+lof.earth_rate[1]^2+lof.muzzle_vel[1]^2+lof.temp[1]^2+lof.press[1]^2+lof.cross_wind[1]^2+lof.range_wind[1]^2+lof.range[1]^2+lof.jump[1]^2+lof.zeroing[1]^2)
    σ_y=sqrt(lof.fcs[2]^2+lof.lead[2]^2+lof.cant[2]^2+lof.earth_rate[2]^2+lof.muzzle_vel[2]^2+lof.temp[2]^2+lof.press[2]^2+lof.cross_wind[2]^2+lof.range_wind[2]^2+lof.range[2]^2+lof.jump[2]^2+lof.zeroing[2]^2)
    return σ_x, σ_y
end

"""
    flight_error(flight)

Returns the projectile dispersion (horizontal and vertical)
"""
function flight_error(flight::FlightError)
    σ_x=sqrt(flight.aero[1]^2)
    σ_y=sqrt(flight.aero[2]^2)
    return σ_x, σ_y
end

"""
    σ(aiming,align,lof,flight)

Returns the total error (horizontal and vertical) with error budget defined by error sources:
* aiming: aiming error
* align: alignment error
* lof: line of fire error
* flight: projectile dispersion
"""
function σ(aiming::AimingError,align::AlignmentError,lof::LofError,flight::FlightError)
    σ_x =sqrt(aiming_error(aiming)[1]^2+alignment_error(align)[1]^2+lof_error(lof)[1]^2+flight_error(flight)[1]^2)
    σ_y =sqrt(aiming_error(aiming)[2]^2+alignment_error(align)[2]^2+lof_error(lof)[2]^2+flight_error(flight)[2]^2)
    return σ_x, σ_y
end

"""
    σ()

Returns the total error of type 'Error'. The error budget is defined by type of errors:
* σfixed : fixed bias errors
* σvariable: varible bias errors
* σrandom: random errors
"""

function σ(;σfixed=Error(0.0,0.0), σvariable=Error(0.0,0.0), σrandom=Error(0.0,0.0))
    σv = σfixed.vertical + sqrt(σvariable.vertical^2 + σrandom.vertical^2)
    σh = σfixed.horizontal + sqrt(σvariable.horizontal^2 + σrandom.horizontal^2)
    σtot = Error(σh,σv)

    return σtot
end
