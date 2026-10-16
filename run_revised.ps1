param(
    [switch]$SkipMonteCarlo,
    [switch]$SkipRender,
    [string]$MatlabExecutable = 'C:\Program Files\MATLAB\R2026a\bin\matlab.exe',
    [string]$BlenderExecutable = 'C:\Program Files\Blender Foundation\Blender 5.2\blender.exe'
)
$ErrorActionPreference = 'Stop'
$missionRoot = $PSScriptRoot
$missionOut = Join-Path $missionRoot 'out\revised'
New-Item -ItemType Directory -Path $missionOut -Force | Out-Null
$missionLog = Join-Path $missionOut 'pipeline.log'
$missionMatlabRoot = $missionRoot.Replace('\','/').Replace("'","''")
function Invoke-MissionMatlab([string]$Code) {
    & $MatlabExecutable -batch $Code *>> $missionLog
    if ($LASTEXITCODE -ne 0) { throw "MATLAB failed; see $missionLog" }
}
"Validation $(Get-Date -Format o)" | Add-Content $missionLog
Invoke-MissionMatlab "cd('$missionMatlabRoot/simulink'); ms_validate_plant; ms_validate_allocator; build_mission_s_model;"
"Nominal mission $(Get-Date -Format o)" | Add-Content $missionLog
Invoke-MissionMatlab "cd('$missionMatlabRoot/simulink'); S=run_mission_s(); assert(S.success,'Nominal mission failed; inspect scene.json');"
if (-not $SkipMonteCarlo) {
    "Monte Carlo $(Get-Date -Format o)" | Add-Content $missionLog
    Invoke-MissionMatlab "cd('$missionMatlabRoot/simulink'); R=ms_robustness(50); assert(R.full_stunt_pass==R.N,'Dispersed missions failed; inspect monte_carlo.csv');"
}
if (-not $SkipRender) {
    "Blender GPU render $(Get-Date -Format o)" | Add-Content $missionLog
    & $BlenderExecutable -b -P (Join-Path $missionRoot 'blender\build_scene.py') -- --csv (Join-Path $missionOut 'trajectory.csv') --scene (Join-Path $missionOut 'scene.json') --out $missionOut --render final --res 70 --samples 64 --frames_dir (Join-Path $missionOut 'frames') --save (Join-Path $missionOut 'mission_s.blend') *>> $missionLog
    if ($LASTEXITCODE -ne 0) { throw "Blender failed; see $missionLog" }
    "Telemetry video $(Get-Date -Format o)" | Add-Content $missionLog
    Invoke-MissionMatlab "addpath('$missionMatlabRoot/matlab'); telemetry_video('$missionMatlabRoot/out/revised');"
}
"Complete $(Get-Date -Format o)" | Add-Content $missionLog
Write-Host "Results: $missionOut"
