$ErrorActionPreference = 'Stop'
$root = Split-Path -Parent $MyInvocation.MyCommand.Path
$source = @(
    (Join-Path $root 'PointCloudPlayer.cs'),
    (Join-Path $root 'LivoxRealtime.cs')
)
$output = Join-Path $root 'LivoxPointCloudPlayer-latest.exe'

if (Test-Path -LiteralPath $output) {
    Remove-Item -LiteralPath $output -Force
}

Add-Type `
    -Path $source `
    -ReferencedAssemblies 'System.dll','System.Core.dll','System.Drawing.dll','System.Windows.Forms.dll' `
    -OutputAssembly $output `
    -OutputType WindowsApplication

Write-Host "Built: $output"
