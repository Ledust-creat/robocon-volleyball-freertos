$ErrorActionPreference = 'Stop'
$projectRoot = (Resolve-Path (Join-Path $PSScriptRoot '../..')).Path
$outputDir = Join-Path $projectRoot 'build/host-control-regression'
New-Item -ItemType Directory -Path $outputDir -Force | Out-Null
$compilerArgs = @('-std=c11', '-O0', '-g', '-Wall', '-Wextra', '-Wno-unused-variable',
    '-Wno-unused-function', '-Werror=implicit-function-declaration',
    '-Werror=incompatible-pointer-types', '-Werror=return-type')
foreach ($dir in @('tests/host/stubs', 'BSP', 'DVE', 'Math', 'Task', 'USER/Inc')) {
    $compilerArgs += '-I' + (Join-Path $projectRoot $dir)
}
$executable = Join-Path $outputDir 'control_regression.exe'
$compilerArgs += @((Join-Path $PSScriptRoot 'control_regression.c'), '-o', $executable)
& gcc @compilerArgs
if ($LASTEXITCODE -ne 0) { throw 'Host regression build failed.' }
& $executable | Tee-Object -FilePath (Join-Path $outputDir 'results.log')
if ($LASTEXITCODE -ne 0) { throw 'Host regression failed.' }
