# pros upload with a choice of brain: V5Upload.cmd runs this.
#
#   V5Upload [path] [pros upload options...]
#
# Asks which V5 device when there are several (scripts/v5ports.ps1) and runs
#   pros upload <path or .> <its Communications port> [options...]
# pros-cli's own "Multiple V5 Brains" prompt crashes on Windows (it reads
# pyserial's `product`, which Windows never fills in), so this picks the port.
param([Parameter(ValueFromRemainingArguments = $true)][string[]]$Rest)
. "$PSScriptRoot\v5ports.ps1"

$path = '.'
if ($Rest -and $Rest.Count -gt 0 -and -not $Rest[0].StartsWith('-')) {
  $path = $Rest[0]
  $Rest = @($Rest | Select-Object -Skip 1)
}
if (-not $Rest) { $Rest = @() }

$pros = Find-Pros
$pick = Select-V5Device @(Get-V5Devices) 'Upload to' 'Port' 'pros upload <path> COMn'

Write-Host "pros upload $path $($pick.Port) $Rest   ($($pick.Label))"
& $pros upload $path $pick.Port @Rest
exit $LASTEXITCODE
