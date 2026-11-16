# pros terminal on the right brain: V5Term.cmd runs this.
#
#   V5Term [pros terminal options...]
#
# Asks which V5 device when there are several (scripts/v5ports.ps1) and runs
#   pros terminal <its User port> [options...]
# Plain `pros terminal` finds no port at all when more than one V5 device is
# on USB under Windows, and then prints nothing.
param([Parameter(ValueFromRemainingArguments = $true)][string[]]$Rest)
. "$PSScriptRoot\v5ports.ps1"
if (-not $Rest) { $Rest = @() }

$pros = Find-Pros
$found = @(Get-V5Devices | Where-Object { $_.UserPort })
$pick = Select-V5Device $found 'Open the terminal of' 'UserPort' 'pros terminal COMn'

Write-Host "pros terminal $($pick.UserPort) $Rest   ($($pick.Label))"
& $pros terminal $pick.UserPort @Rest
exit $LASTEXITCODE
