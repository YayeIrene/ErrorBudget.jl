var documenterSearchIndex = {"docs":
[{"location":"#ErrorBudget.jl-Documentation","page":"Home","title":"ErrorBudget.jl Documentation","text":"","category":"section"},{"location":"","page":"Home","title":"Home","text":"Error budget computation.","category":"page"},{"location":"#Table-of-contents","page":"Home","title":"Table of contents","text":"","category":"section"},{"location":"","page":"Home","title":"Home","text":"Pages = [\"index.md\"]","category":"page"},{"location":"#Introduction","page":"Home","title":"Introduction","text":"","category":"section"},{"location":"","page":"Home","title":"Home","text":"using ErrorBudget","category":"page"},{"location":"#Package-Features","page":"Home","title":"Package Features","text":"","category":"section"},{"location":"","page":"Home","title":"Home","text":"Tank accuracy computation","category":"page"},{"location":"#Function-documentation","page":"Home","title":"Function documentation","text":"","category":"section"},{"location":"","page":"Home","title":"Home","text":"Modules = [ErrorBudget]\r\nOrder = [:function]","category":"page"},{"location":"#ErrorBudget.aiming_error-Tuple{AimingError}","page":"Home","title":"ErrorBudget.aiming_error","text":"aiming_error(aiming)\n\nReturns the alignment error (horizontal and vertical) for the error sources defined in 'aiming'\n\n\n\n\n\n","category":"method"},{"location":"#ErrorBudget.alignment_error-Tuple{AlignmentError}","page":"Home","title":"ErrorBudget.alignment_error","text":"alignment_error(align)\n\nReturns the alignment error (horizontal and vertical) for the error sources defined in 'align'\n\n\n\n\n\n","category":"method"},{"location":"#ErrorBudget.azimuthError-Tuple{Float64, ExternalBallistics.AbstractTarget, ExternalBallistics.AbstractPenetrator, ExternalBallistics.Gun, Tank, DataFrames.DataFrame}","page":"Home","title":"ErrorBudget.azimuthError","text":"azimuthError(σ, target,proj,weapon,tank,aero;args )\n\nReturns the normal distribution for the impact points (vertical and horizontal distribution). The function performs a Monte Carlo simulation by sampling the azimuth angle. It is assumes that the elevation angle variation is a gaussian distribution with μ = the computed azimuth angle and standard deviation = σ:\n\ntank\nweapon : the launcher of type Gun\nproj : the projectile\ntarget\naero : the aerodynamic coefficients of the projectile\n\nOne optional argument is N (the number of Monte Carlo simulations). Default = 100\n\n\n\n\n\n","category":"method"},{"location":"#ErrorBudget.cantError-Tuple{Float64, Float64, ExternalBallistics.AbstractTarget, ExternalBallistics.AbstractPenetrator, ExternalBallistics.Gun, Tank, DataFrames.DataFrame}","page":"Home","title":"ErrorBudget.cantError","text":"cantError(σ, σmeas, target,proj,weapon,tank,aero, atmosphere;args )\n\nReturns the normal distribution for the impact points (vertical and horizontal distribution). The function performs a Monte Carlo simulation by sampling the cant angle. It is assumes that the cant variation is a gaussian distribution with μ = tank.hull.Φ and standard deviation = σ. On this value of cant we perform another sampling to take into account the measurement error. We assume also a normal distribution with μ = sampled cant and the standard deviation = σmeas:\n\ntank\nweapon : the launcher of type Gun\nproj : the projectile\ntarget\naero : the aerodynamic coefficients of the projectile\n\nOne optional argument is N (the number of Monte Carlo simulations). Default = 100\n\n\n\n\n\n","category":"method"},{"location":"#ErrorBudget.createTargetCirc-Tuple{Float64, Float64}","page":"Home","title":"ErrorBudget.createTargetCirc","text":"createTargetCirc(size, position)\n\nReturns a circular target by specifying the radius (size) and the position (range)\n\n\n\n\n\n","category":"method"},{"location":"#ErrorBudget.createTargetRect-Tuple{Float64, Float64, Float64}","page":"Home","title":"ErrorBudget.createTargetRect","text":"createTargetRect(a,b, position)\n\nReturn a rectangular target by specifying the length 'a', the height 'b' and the 'position' (range)\n\n\n\n\n\n","category":"method"},{"location":"#ErrorBudget.elevationError-Tuple{Float64, ExternalBallistics.AbstractTarget, ExternalBallistics.AbstractPenetrator, ExternalBallistics.Gun, Tank, DataFrames.DataFrame}","page":"Home","title":"ErrorBudget.elevationError","text":"elevationError(σ, target,proj,weapon,tank,aero;args )\n\nReturns the normal distribution for the impact points (vertical and horizontal distribution). The function performs a Monte Carlo simulation by sampling the elevation angle. It is assumes that the elevation angle variation is a gaussian distribution with μ = the compute elevation angle and standard deviation = σ:\n\ntank\nweapon : the launcher of type Gun\nproj : the projectile\ntarget\naero : the aerodynamic coefficients of the projectile\n\nOne optional argument is N (the number of Monte Carlo simulations). Default = 100\n\n\n\n\n\n","category":"method"},{"location":"#ErrorBudget.flight_error-Tuple{FlightError}","page":"Home","title":"ErrorBudget.flight_error","text":"flight_error(flight)\n\nReturns the projectile dispersion (horizontal and vertical)\n\n\n\n\n\n","category":"method"},{"location":"#ErrorBudget.interTable-Tuple{Vector{Float64}, Vector{Float64}, Float64}","page":"Home","title":"ErrorBudget.interTable","text":"interTable(range,jumpTable, value)\n\nReturns the 'value' from an interpolation table 'jumpTable' over the 'range'\n\n\n\n\n\n","category":"method"},{"location":"#ErrorBudget.lof_error-Tuple{LofError}","page":"Home","title":"ErrorBudget.lof_error","text":"lof_error(lof)\n\nReturns the line of fire error (horizontal and vertical) for the error sources defined in 'lof'\n\n\n\n\n\n","category":"method"},{"location":"#ErrorBudget.muzzleVelError-Tuple{Float64, ExternalBallistics.AbstractTarget, ExternalBallistics.AbstractPenetrator, ExternalBallistics.Gun, Tank, DataFrames.DataFrame}","page":"Home","title":"ErrorBudget.muzzleVelError","text":"muzzleVelError(σ, target,proj,weapon,tank,aero;args )\n\nReturns the normal distribution for the impact points (vertical and horizontal distribution). The function performs a Monte Carlo simulation by sampling the muzzle velocity. It is assumes that the muzzle velocity variation is a gaussian distribution with μ = weapon.u₀ and standard deviation = σ:\n\ntank\nweapon : the launcher of type Gun\nproj : the projectile\ntarget\naero : the aerodynamic coefficients of the projectile\n\nOne optional argument is N (the number of Monte Carlo simulations). Default = 100\n\n\n\n\n\n","category":"method"},{"location":"#ErrorBudget.rangeError-Tuple{ExternalBallistics.AbstractTarget, Float64, Tank, ExternalBallistics.Gun, ExternalBallistics.AbstractPenetrator, DataFrames.DataFrame}","page":"Home","title":"ErrorBudget.rangeError","text":"rangeError(target, σ, tank, weapon, proj, aero; args)\n\nReturns the normal distribution for elevation and azimuth angles. The function performs a Monte Carlo simulation by sampling the target range. It is assumes that the range variation is a gaussian distribution with μ = target.ρ and standard deviation = σ. For each sampled range, a elevation and azimuth angle is computed:\n\ntank\nweapon : the launcher of type Gun\nproj : the projectile\ntarget\naero : the aerodynamic coefficients of the projectile\n\nOne optional argument is N (the number of Monte Carlo simulations). Default = 100\n\n\n\n\n\n","category":"method"},{"location":"#ErrorBudget.temperatureError-Tuple{Float64, ExternalBallistics.AbstractTarget, ExternalBallistics.AbstractPenetrator, ExternalBallistics.Gun, Tank, DataFrames.DataFrame, ExternalBallistics.Air}","page":"Home","title":"ErrorBudget.temperatureError","text":"temperatureError(σ, target,proj,weapon,tank,aero, atmosphere;args )\n\nReturns the normal distribution for the impact points (vertical and horizontal distribution). The function performs a Monte Carlo simulation by sampling the air temperature. It is assumes that the temperature variation is a gaussian distribution with μ = atmosphere.t and standard deviation = σ:\n\ntank\nweapon : the launcher of type Gun\nproj : the projectile\ntarget\naero : the aerodynamic coefficients of the projectile\natmosphere: the atmospheric characteristics\n\nOne optional argument is N (the number of Monte Carlo simulations). Default = 100\n\n\n\n\n\n","category":"method"},{"location":"#ErrorBudget.variableBias-Tuple{ExternalBallistics.AbstractTarget, Tank, ExternalBallistics.Gun, ExternalBallistics.AbstractPenetrator, DataFrames.DataFrame, ExternalBallistics.Wind, ExternalBallistics.Air}","page":"Home","title":"ErrorBudget.variableBias","text":"variableBias(target,tank, weapon, proj, aero, wind, atmosphere;args ...)\n\nReturns the variable bias error as an objet \"Error\". optional arguments are all the standard deviation of error sources: σrange, σcrosswind,σrangewind, σjump,σfc,σzeroing,σboresight,σMv, σtemp, σpress, σopt, σgun, σcant, σcant_meas default values for optional parameters are nothing or nul:\n\ntank\nweapon : the launcher of type Gun\nproj : the projectile\ntarget\naero : the aerodynamic coefficients of the projectile\nwind\natmosphere\n\n\n\n\n\n","category":"method"},{"location":"#ErrorBudget.windCrossError-Tuple{Float64, ExternalBallistics.Wind, Tank, ExternalBallistics.Gun, ExternalBallistics.AbstractPenetrator, ExternalBallistics.AbstractTarget, DataFrames.DataFrame}","page":"Home","title":"ErrorBudget.windCrossError","text":"windCrossError(σ,wind,tank, weapon,proj, target, aero;args ...)\n\nReturns the normal distribution for the impact points (horizontal coordinate). The function performs a Monte Carlo simulation by sampling the wind (cross field). It is assumes that the cross wind variation is a gaussian distribution with μ = wind.cross and standard deviation = σ:\n\ntank\nweapon : the launcher of type Gun\nproj : the projectile\ntarget\naero : the aerodynamic coefficients of the projectile\n\nOne optional argument is N (the number of Monte Carlo simulations). Default = 100\n\n\n\n\n\n","category":"method"},{"location":"#ErrorBudget.windRangeError-Tuple{Float64, ExternalBallistics.Wind, Tank, ExternalBallistics.Gun, ExternalBallistics.AbstractPenetrator, ExternalBallistics.AbstractTarget, DataFrames.DataFrame}","page":"Home","title":"ErrorBudget.windRangeError","text":"windRangeError(σ,wind,tank, weapon,proj, target, aero;args ...)\n\nReturns the normal distribution for the impact points (vertical coordinate). The function performs a Monte Carlo simulation by sampling the wind (range field). It is assumes that the range wind variation is a gaussian distribution with μ = wind.range and standard deviation = σ:\n\ntank\nweapon : the launcher of type Gun\nproj : the projectile\ntarget\naero : the aerodynamic coefficients of the projectile\n\nOne optional argument is N (the number of Monte Carlo simulations). Default = 100\n\n\n\n\n\n","category":"method"},{"location":"#ErrorBudget.σ-Tuple{AimingError, AlignmentError, LofError, FlightError}","page":"Home","title":"ErrorBudget.σ","text":"σ(aiming,align,lof,flight)\n\nReturns the total error (horizontal and vertical) with error budget defined by error sources:\n\naiming: aiming error\nalign: alignment error\nlof: line of fire error\nflight: projectile dispersion\n\n\n\n\n\n","category":"method"},{"location":"#Type-documentation","page":"Home","title":"Type documentation","text":"","category":"section"},{"location":"","page":"Home","title":"Home","text":"Modules = [ErrorBudget]\r\nOrder = [:type]","category":"page"},{"location":"#ErrorBudget.AimingError","page":"Home","title":"ErrorBudget.AimingError","text":"AimingError(point_def,human,res,opt_path)\n\ndefines an objet 'AimingError' with fields containing the aiming error sources:\n\npoint_def: error (standard deviatin) on the aimpoint definition expressed as a tupple (horizontal and vertical).\nhuman: human error (standard deviatin) on the aimpoint definition expressed as a tupple (horizontal and vertical).\nres: resolution error (standard deviatin) on the aimpoint definition expressed as a tupple (horizontal and vertical).\nopt_path: optical path bending error (standard deviatin) on the aimpoint definition expressed as a tupple (horizontal and vertical).\n\n\n\n\n\n","category":"type"},{"location":"#ErrorBudget.AlignmentError","page":"Home","title":"ErrorBudget.AlignmentError","text":"AlignmentError(boresight_acc,boresight_ret,sync,stab, gun_pos)\n\ndefines an objet 'AlignmentError' with fields containing the alignment error sources:\n\nboresight_acc: boresight accuracy (standard deviatin) expressed as a tupple (horizontal and vertical).\nboresight_ret: boresight retention (standard deviatin) expressed as a tupple (horizontal and vertical).\nsync: synchronism/parallelism error (standard deviatin) expressed as a tupple (horizontal and vertical).\nstab: stabilisation error (standard deviation) expressed as a tupple (horizontal and vertical).\ngun_pos: gun position measurement error (standard deviation) expressed as a tupple (horizontal and vertical).\n\n\n\n\n\n","category":"type"},{"location":"#ErrorBudget.Error","page":"Home","title":"ErrorBudget.Error","text":"Error(horizontal, vertical)\n\nReturns the Error object with field: 'horizontal' and 'vertical'\n\n\n\n\n\n","category":"type"},{"location":"#ErrorBudget.FlightError","page":"Home","title":"ErrorBudget.FlightError","text":"FlightError(aero)\n\ndefines an objet 'FlightError' which contains the projectile dispersion as an error (standard deviation) expressed as a tupple (horizontal and vertical).\n\n\n\n\n\n","category":"type"},{"location":"#ErrorBudget.LofError","page":"Home","title":"ErrorBudget.LofError","text":"LofError(fcs,lead,cant,earth_rate, muzzle_vel, temp, press, cross_wind, range_wind, range, jump, zeroing)\n\ndefines an objet 'LofError' with fields containing the line of fire error sources:\n\nfcs: fire control error (standard deviation) expressed as a tupple (horizontal and vertical).\nlead: lead angle (standard deviation) expressed as a tupple (horizontal and vertical).\ncant: cant angle error (standard deviation) expressed as a tupple (horizontal and vertical).\nearth_rate: earth rate error (standard deviation) expressed as a tupple (horizontal and vertical).\nmuzzle_vel: muzzle velocity error (standard deviation) expressed as a tupple (horizontal and vertical).\ntemp: temperature error (standard deviation) expressed as a tupple (horizontal and vertical).\npress: pressure error (standard deviation) expressed as a tupple (horizontal and vertical).\ncross_wind: cross wind error (standard deviation) expressed as a tupple (horizontal and vertical).\nrange_wind: range wind error (standard deviation) expressed as a tupple (horizontal and vertical).\njump: fleet jump error (standard deviation) expressed as a tupple (horizontal and vertical).\nzeroing: calibration error (standard deviation) expressed as a tupple (horizontal and vertical).\n\n\n\n\n\n","category":"type"},{"location":"#ErrorBudget.SpheError","page":"Home","title":"ErrorBudget.SpheError","text":"SpheError(σx,σy,σz,μx,μy,μz)\n\nReturns a spherical error object with standard deviation and center of impact points\n\n\n\n\n\n","category":"type"},{"location":"#ErrorBudget.UnitEffect","page":"Home","title":"ErrorBudget.UnitEffect","text":"UnitEffect(velocity,temp,density, windR,windC)\n\nReturns the UnitEffects for different ranges. The parameters considered are:     * velocity     * temp: temperature     * density     * windR: wind range     * windC: cross wind\n\n\n\n\n\n","category":"type"},{"location":"","page":"Home","title":"Home","text":"","category":"page"}]
}
