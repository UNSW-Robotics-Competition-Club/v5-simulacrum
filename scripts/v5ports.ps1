# V5 devices on this PC's USB, for V5Upload and V5Term (dot-source this).
#
# Lists the V5 brains and controllers (simulators by instance, from the USB
# serial number V5SIM<N> that v5usb_usbip.py reports) with both their ports:
# interface 0 is Communications (system), interface 2 the User port. pros-cli
# can't tell these apart on Windows (they are all "USB Serial Device", and it
# only guesses when there are exactly two ports), so these scripts pick them.

function Find-Pros {
  $pros = (Get-Command pros -ErrorAction SilentlyContinue).Source
  if ($pros) { return $pros }
  $pros = Join-Path $env:APPDATA 'Code\User\globalStorage\sigbots.pros\install\pros-cli-windows\pros.exe'
  if (Test-Path $pros) { return $pros }
  Write-Error 'pros not found: install the PROS VS Code extension or put pros on PATH'
  exit 2
}

# The COM port of interface $mi of a V5 device, keyed by the device's
# instance ID (the parent of each interface).
function Get-V5Interfaces([string]$mi) {
  $byParent = @{}
  foreach ($e in Get-CimInstance Win32_PnPEntity -Filter "PNPDeviceID LIKE 'USB\\VID_2888&PID_%&MI_$mi%'") {
    if ($e.Name -notmatch '\((COM\d+)\)') { continue }
    $com = $Matches[1]
    $parent = (Get-PnpDeviceProperty -InstanceId $e.PNPDeviceID -KeyName DEVPKEY_Device_Parent -ErrorAction SilentlyContinue).Data
    if ($parent) { $byParent[$parent] = [pscustomobject]@{ Port = $com; Id = $e.PNPDeviceID } }
  }
  return $byParent
}

function Get-PnpParent([string]$id) {
  (Get-PnpDeviceProperty -InstanceId $id -KeyName DEVPKEY_Device_Parent -ErrorAction SilentlyContinue).Data
}

# A device on usbip-win2's emulated host controller (device -> root hub ->
# controller with service usbip2_ude) is a simulator.
function Test-UsbipDevice([string]$id) {
  $hub = Get-PnpParent $id
  $ctl = if ($hub) { Get-PnpParent $hub }
  if (-not $ctl) { return $false }
  (Get-PnpDeviceProperty -InstanceId $ctl -KeyName DEVPKEY_Device_Service -ErrorAction SilentlyContinue).Data -eq 'usbip2_ude'
}

# Brains and controllers to choose from. A USB/IP device without a V5SIM<N>
# serial is a duplicate attach of a sim (Windows drops a serial it has already
# seen and makes up an ID); those are skipped, with a note.
function Get-V5Devices {
  $system = Get-V5Interfaces '00'
  $user = Get-V5Interfaces '02'
  $found = @()
  $copies = @()
  foreach ($parent in $system.Keys) {
    $serial = ($parent -split '\\')[-1]
    if ($serial -notmatch '^V5SIM\d+$' -and (Test-UsbipDevice $parent)) {
      $copies += $system[$parent].Port
      continue
    }
    if ($serial -match '^V5SIM(\d+)$') {
      $n = [int]$Matches[1]
      $label = "simulator $n (field view http://localhost:$(8765 + $n)/)"
      $order = $n
    } elseif ($system[$parent].Id -match 'PID_0503') {
      $label = 'V5 controller (its paired brain)'
      $order = 1000
    } else {
      $label = 'V5 brain'
      $order = 1000
    }
    $userPort = if ($user.ContainsKey($parent)) { $user[$parent].Port } else { $null }
    $found += [pscustomobject]@{
      Port = $system[$parent].Port; UserPort = $userPort; Label = $label; Order = $order
    }
  }
  if ($copies) {
    Write-Host ("Skipping {0} duplicate USB/IP attach(es) of a simulator ({1}); a Windows restart clears them." -f $copies.Count, ($copies -join ', '))
  }
  return @($found | Sort-Object Order, { [int]($_.Port -replace '\D', '') })
}

# Pick one device: the only one, or ask. $portOf names the port shown and
# typed (Port or UserPort); $what completes "<what> which one?".
function Select-V5Device($found, [string]$what, [string]$portOf, [string]$manual) {
  if ($found.Count -eq 0) {
    Write-Error 'No V5 brain on USB. Is V5Sim running (with usbip-win2 installed) or a brain plugged in?'
    exit 1
  }
  if ($found.Count -eq 1) { return $found[0] }
  Write-Host "$what which one?"
  for ($i = 0; $i -lt $found.Count; $i++) {
    Write-Host ("  {0}) {1}  {2}" -f ($i + 1), $found[$i].$portOf, $found[$i].Label)
  }
  # No one to ask (input redirected, e.g. from a script): don't guess one.
  if ([Console]::IsInputRedirected) {
    Write-Error "Several V5 devices and no console to ask: run  $manual  with one of the ports above"
    exit 1
  }
  while ($true) {
    $answer = Read-Host "Choose 1-$($found.Count) [1]"
    if ($answer -eq '') { $answer = '1' }
    $k = 0
    if ([int]::TryParse($answer, [ref]$k) -and $k -ge 1 -and $k -le $found.Count) {
      return $found[$k - 1]
    }
    $m = @($found | Where-Object { $_.$portOf -ieq $answer })
    if ($m.Count -eq 1) { return $m[0] }
  }
}
