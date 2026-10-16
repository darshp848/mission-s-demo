# Compatibility entry point for the revised, shared-layout pipeline.
param(
    [switch]$SkipMonteCarlo,
    [switch]$SkipRender,
    [string]$MatlabExecutable,
    [string]$BlenderExecutable
)
& (Join-Path $PSScriptRoot 'run_revised.ps1') @PSBoundParameters
