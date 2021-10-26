# ErrorBudget.jl Documentation

*Error budget computation.*
## Introduction

```@repl
using ErrorBudget
```
## Table of contents


```@contents
Pages = ["index.md"]
```

## Package Features
- Tank accuracy computation

## Functions Documentation

```@docs
σ
```
```@docs
windCrossError
```
!!! info "'info'"
    The trajectory is computed using MPMM model
!!! warning "'warning'"
    tank here is only used to compute the vector wind (in the future we will probably use another approach)
```@docs
windRangeError
```
```@docs
interTable
```
```@docs
rangeError
```
```@docs
elevationError
```
```@docs
azimuthError
```
```@docs
muzzleVelError
```
```@docs
temperatureError
```
```@docs
cantError
```
```@docs
variableBias
```
### Objects generation

###  computation


## Types Documentation
```@docs
AimingError
```
```@docs
AlignmentError
```
```@docs
LofError
```
```@docs
FlightError
```
```@docs
UnitEffect
```
```@docs
Error
```
