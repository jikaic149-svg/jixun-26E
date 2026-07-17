[CmdletBinding()]
param(
    [Parameter(Mandatory = $true)]
    [ValidateSet("Doctor", "Configure", "Build", "Clean", "Purge", "Generate", "Flash", "SysConfig")]
    [string]$Action,

    [ValidateSet("MSPM0G3507", "MSPM0G3519")]
    [string]$Device = "MSPM0G3519",

    [ValidateSet("Debug", "Release")]
    [string]$Configuration = "Debug",

    [ValidateSet("Current", "Minimal")]
    [string]$Profile = "Current",

    [ValidateSet("DAPLink", "XDS110")]
    [string]$Probe
)

Set-StrictMode -Version Latest
$ErrorActionPreference = "Stop"

$ExpectedProbe = if ($Device -eq "MSPM0G3507") { "XDS110" } else { "DAPLink" }
if ([string]::IsNullOrWhiteSpace($Probe)) {
    $Probe = $ExpectedProbe
}
if ($Probe -ne $ExpectedProbe) {
    throw "Unsupported probe combination: $Device via $Probe. MSPM0G3507 uses XDS110; MSPM0G3519 uses DAPLink."
}

$WorkspaceRoot = Split-Path -Parent $PSScriptRoot
$TiMspm0Root = Split-Path -Parent $WorkspaceRoot
$TiEnvRoot = Join-Path $TiMspm0Root "TI ENV"
$ArmGccRoot = "D:\GCC\ARMGCC\10 2021.10"
$CMakeRoot = "D:\CMAKE"
$NinjaRoot = "D:\Ninja"
$StockOpenOcdRoot = "D:\OPENOCD\xpack-openocd-0.12.0-7"
$G3519OpenOcdRoot = "D:\OPENOCD\openocd-custom-20260615"
$ExpectedStockOpenOcdSha256 = "10567F01D37C66A0975537960D66E61F7B4E2A17EB2C9637E2C2CBC569F70C79"
$ExpectedG3519OpenOcdSha256 = "CB97B5EB254FF359772A4AF0438CF7987E05FDA038DBFDE73B767390A1797D2F"
$ExpectedG3519LibUsbSha256 = "EDDD25DF84B078158F477DFCE80B26500F91ECDB5AF850581D2F5D09D6F21C26"
$ExpectedG3519LibHidApiSha256 = "0133795F06C3BA6C5A21EB1A1B24CFCC80B38F979719D5BD29A5786AE87ED744"
$ExpectedG3519TargetScriptSha256 = "49EFF22E64336FEB787314D26EDFCC6E0930EB858771345E5B4C8E3CDEC28590"
$ExpectedG3519ProjectCfgSha256 = "B4BDCB031A086E154525B82C5E9133ED6E5EB8BCAEF505D0552933A2E605E4A6"
$ExpectedG3507ProjectCfgSha256 = "3BF909DF2BE66F59DFE2712AB59E1EB8D5CBF67C843E30CE978436F531AA0EED"
$Mspm0SdkRoot = Join-Path $TiEnvRoot "mspm0_sdk_2_10_00_04"
$SysConfigRoot = Join-Path $TiEnvRoot "sysconfig_1.27.1"
$ProbeLocalCfg = if ($Probe -eq "DAPLink") {
    Join-Path $TiEnvRoot "local\openocd\daplink.cfg"
}
else {
    Join-Path $TiEnvRoot "local\openocd\xds110.cfg"
}

# The stock xPack build is used only for MSPM0G3507 + XDS110. It predates
# MSPM0G3519 part-table support, so G3519 + DAPLink uses the isolated custom
# build with the narrow FACTORYREGION fallback.
$UseG3519OpenOcd = ($Device -eq "MSPM0G3519")
if ($UseG3519OpenOcd) {
    $OpenOcdRoot = $G3519OpenOcdRoot
    $OpenOcdScripts = Join-Path $OpenOcdRoot "share\openocd\scripts"
    $OpenOcdFlavor = "custom G3519 part support / FACTORYREGION fallback"
}
else {
    $OpenOcdRoot = $StockOpenOcdRoot
    $OpenOcdScripts = Join-Path $OpenOcdRoot "openocd\scripts"
    $OpenOcdFlavor = "stock xPack"
}

$CMake = Join-Path $CMakeRoot "bin\cmake.exe"
$Ninja = Join-Path $NinjaRoot "ninja.exe"
$OpenOcd = Join-Path $OpenOcdRoot "bin\openocd.exe"
$G3519LibUsb = Join-Path $G3519OpenOcdRoot "bin\libusb-1.0.dll"
$G3519LibHidApi = Join-Path $G3519OpenOcdRoot "bin\libhidapi-0.dll"
$G3519TargetScript = Join-Path $G3519OpenOcdRoot "share\openocd\scripts\target\ti\mspm0.cfg"
$OpenOcdConfigRoot = Join-Path $WorkspaceRoot "config\openocd"
$G3519ProjectCfg = Join-Path $OpenOcdConfigRoot "mspm0_daplink_custom.cfg"
$G3507ProjectCfg = Join-Path $OpenOcdConfigRoot "mspm0_xds110.cfg"
$Gcc = Join-Path $ArmGccRoot "bin\arm-none-eabi-gcc.exe"
$Gdb = Join-Path $ArmGccRoot "bin\arm-none-eabi-gdb.exe"
$Size = Join-Path $ArmGccRoot "bin\arm-none-eabi-size.exe"
$SysConfigNode = Join-Path $SysConfigRoot "nodejs\node.exe"
$SysConfigCli = Join-Path $SysConfigRoot "dist\cli.js"
$SysConfigNw = Join-Path $SysConfigRoot "nw\nw.exe"
$SysConfigApp = Join-Path $SysConfigRoot "app"
$SysConfigProduct = Join-Path $Mspm0SdkRoot ".metadata\product.json"

function Assert-File {
    param([Parameter(Mandatory = $true)][string]$Path)
    if (-not (Test-Path -LiteralPath $Path -PathType Leaf)) {
        throw "Required file does not exist: $Path"
    }
}

function Assert-FileHash {
    param(
        [Parameter(Mandatory = $true)][string]$Path,
        [Parameter(Mandatory = $true)][string]$ExpectedSha256,
        [Parameter(Mandatory = $true)][string]$Label
    )

    Assert-File -Path $Path
    $actualSha256 = (Get-FileHash -Algorithm SHA256 -LiteralPath $Path).Hash
    if ($actualSha256 -ne $ExpectedSha256) {
        throw "Refusing unrecognized $Label. Expected SHA-256 $ExpectedSha256, got $actualSha256 ($Path)"
    }
}

function Assert-Directory {
    param([Parameter(Mandatory = $true)][string]$Path)
    if (-not (Test-Path -LiteralPath $Path -PathType Container)) {
        throw "Required directory does not exist: $Path"
    }
}

function Get-ProbeSerial {
    if (-not (Test-Path -LiteralPath $ProbeLocalCfg -PathType Leaf)) {
        $exampleName = if ($Probe -eq "DAPLink") {
            "daplink.local.cfg.example"
        }
        else {
            "xds110.local.cfg.example"
        }
        $examplePath = Join-Path $OpenOcdConfigRoot $exampleName
        throw (("Missing machine-local {0} config: {1}. Create it from {2}; " +
            "Doctor/Flash will not continue without an explicit probe serial.") -f
            $Probe, $ProbeLocalCfg, $examplePath)
    }

    $activeLines = @(
        Get-Content -LiteralPath $ProbeLocalCfg -Encoding UTF8 |
            ForEach-Object { $_.Trim() } |
            Where-Object { $_.Length -ne 0 -and -not $_.StartsWith("#") }
    )
    if ($activeLines.Count -ne 1) {
        throw (("Invalid machine-local {0} config: {1}. Expected exactly one " +
            "active line: adapter serial `"<probe serial>`"") -f
            $Probe, $ProbeLocalCfg)
    }

    $serialMatch = [regex]::Match(
        $activeLines[0],
        '^adapter[ \t]+serial[ \t]+"([^"\r\n]+)"[ \t]*(?:#.*)?$'
    )
    if (-not $serialMatch.Success) {
        throw (("Invalid machine-local {0} config: {1}. Expected exactly one " +
            "active line: adapter serial `"<probe serial>`"") -f
            $Probe, $ProbeLocalCfg)
    }

    $serial = $serialMatch.Groups[1].Value
    if ($serial -notmatch '^[A-Za-z0-9._-]+$') {
        throw (("Unsafe {0} serial in {1}. Only letters, digits, dot, underscore " +
            "and hyphen are allowed.") -f $Probe, $ProbeLocalCfg)
    }
    return $serial
}

function Assert-ProbePresent {
    if ($Probe -eq "DAPLink") {
        $usbId = "VID_0D28&PID_0204"
    }
    else {
        $usbId = "VID_0451&PID_BEF3"
    }
    $serial = Get-ProbeSerial

    # DAPLink and XDS110 are composite USB devices. Their present parent
    # devnode does not consistently expose a volatile registry Control subkey,
    # so query the exact serial-qualified PnP instance instead.
    $instanceId = "USB\$usbId\$serial"
    $device = Get-PnpDevice -InstanceId $instanceId -PresentOnly `
        -ErrorAction SilentlyContinue
    if ($null -eq $device -or $device.Status -ne "OK") {
        throw "$Probe is not present ($usbId, serial $serial). Reconnect the expected probe before $Action."
    }
}

function Write-Stage {
    param([Parameter(Mandatory = $true)][string]$Message)
    Write-Host ("[STEP] {0}" -f $Message) -ForegroundColor Cyan
}

function Write-Ok {
    param([Parameter(Mandatory = $true)][string]$Message)
    Write-Host ("[OK]   {0}" -f $Message) -ForegroundColor Green
}

function Invoke-Native {
    param(
        [Parameter(Mandatory = $true)][string]$FilePath,
        [Parameter()][string[]]$Arguments = @(),
        [Parameter()][string]$DisplayName = "",
        [Parameter()][switch]$QuietOnSuccess,
        [Parameter()][string[]]$KeepOnSuccess = @(),
        [Parameter()][switch]$ShowAllWhenKept
    )

    if ([string]::IsNullOrWhiteSpace($DisplayName)) {
        $DisplayName = Split-Path -Leaf $FilePath
    }
    Write-Verbose ("{0}: {1} {2}" -f $DisplayName, $FilePath, ($Arguments -join " "))

    if ($QuietOnSuccess) {
        $savedErrorActionPreference = $ErrorActionPreference
        try {
            # Windows PowerShell wraps native stderr as ErrorRecord objects.
            # Capture those records and judge success only by the process exit
            # code; otherwise tools such as OpenOCD --version look like a
            # terminating PowerShell error even though they returned zero.
            $ErrorActionPreference = "Continue"
            $output = @(& $FilePath @Arguments 2>&1)
            $exitCode = $LASTEXITCODE
        }
        finally {
            $ErrorActionPreference = $savedErrorActionPreference
        }
        if ($exitCode -eq 0) {
            if ($KeepOnSuccess.Count -ne 0) {
                $keptLines = [System.Collections.Generic.List[string]]::new()
                foreach ($line in $output) {
                    $text = $line.ToString()
                    foreach ($pattern in $KeepOnSuccess) {
                        if ($text -match $pattern) {
                            $keptLines.Add($text)
                            break
                        }
                    }
                }
                if ($ShowAllWhenKept -and $keptLines.Count -ne 0) {
                    foreach ($line in $output) {
                        Write-Host $line.ToString()
                    }
                }
                else {
                    foreach ($line in $keptLines) {
                        Write-Host $line
                    }
                }
            }
            return
        }

        foreach ($line in $output) {
            Write-Host $line.ToString()
        }
        throw "$DisplayName failed with exit code ${exitCode}: $FilePath"
    }

    & $FilePath @Arguments
    $exitCode = $LASTEXITCODE
    if ($exitCode -ne 0) {
        throw "$DisplayName failed with exit code ${exitCode}: $FilePath"
    }
}

function Get-NativeVersionLine {
    param(
        [Parameter(Mandatory = $true)][string]$FilePath,
        [Parameter()][string[]]$Arguments = @(),
        [Parameter(Mandatory = $true)][string]$DisplayName
    )

    Write-Verbose ("{0}: {1} {2}" -f $DisplayName, $FilePath, ($Arguments -join " "))
    $savedErrorActionPreference = $ErrorActionPreference
    try {
        $ErrorActionPreference = "Continue"
        $output = @(& $FilePath @Arguments 2>&1)
        $exitCode = $LASTEXITCODE
    }
    finally {
        $ErrorActionPreference = $savedErrorActionPreference
    }
    if ($exitCode -ne 0) {
        foreach ($line in $output) {
            Write-Host $line.ToString()
        }
        throw "$DisplayName version check failed with exit code ${exitCode}: $FilePath"
    }

    return ($output | ForEach-Object { $_.ToString() } |
        Where-Object { -not [string]::IsNullOrWhiteSpace($_) } |
        Select-Object -First 1)
}

function Assert-MainFlashHex {
    param(
        [Parameter(Mandatory = $true)][string]$Path,
        [Parameter(Mandatory = $true)][uint64]$FlashSize
    )

    [uint64]$upperAddress = 0
    [uint64]$minimumAddress = [uint64]::MaxValue
    [uint64]$maximumAddress = 0
    [uint64]$dataBytes = 0
    [bool]$sawEof = $false

    foreach ($rawLine in [System.IO.File]::ReadLines($Path)) {
        $line = $rawLine.Trim()
        if ($line.Length -eq 0) {
            continue
        }
        if ($sawEof) {
            throw "Unexpected data after Intel HEX EOF record: $line"
        }
        if (-not $line.StartsWith(":")) {
            throw "Invalid Intel HEX record (missing colon): $line"
        }

        $byteCount = [Convert]::ToUInt32($line.Substring(1, 2), 16)
        $expectedLength = 11 + (2 * $byteCount)
        if ($line.Length -ne $expectedLength) {
            throw "Invalid Intel HEX record length: $line"
        }

        $checksum = 0
        for ($index = 1; $index -lt $line.Length; $index += 2) {
            $checksum = ($checksum + [Convert]::ToUInt32($line.Substring($index, 2), 16)) -band 0xFF
        }
        if ($checksum -ne 0) {
            throw "Invalid Intel HEX checksum: $line"
        }

        $recordAddress = [Convert]::ToUInt32($line.Substring(3, 4), 16)
        $recordType = [Convert]::ToUInt32($line.Substring(7, 2), 16)
        switch ($recordType) {
            0 {
                if ($byteCount -eq 0) {
                    throw "Invalid zero-length Intel HEX data record: $line"
                }
                [uint64]$startAddress = $upperAddress + $recordAddress
                [uint64]$endAddress = $startAddress + $byteCount - 1
                if ($startAddress -ge $FlashSize -or $endAddress -ge $FlashSize) {
                    throw ("Refusing to flash data outside main Flash: 0x{0:X8}-0x{1:X8}" -f $startAddress, $endAddress)
                }
                if ($startAddress -lt $minimumAddress) {
                    $minimumAddress = $startAddress
                }
                if ($endAddress -gt $maximumAddress) {
                    $maximumAddress = $endAddress
                }
                $dataBytes += $byteCount
            }
            1 {
                if ($byteCount -ne 0 -or $recordAddress -ne 0) {
                    throw "Invalid Intel HEX EOF record: $line"
                }
                $sawEof = $true
            }
            2 {
                if ($byteCount -ne 2 -or $recordAddress -ne 0) {
                    throw "Invalid Intel HEX extended-segment record: $line"
                }
                $upperAddress = [uint64]([Convert]::ToUInt32($line.Substring(9, 4), 16)) -shl 4
            }
            3 {
                if ($byteCount -ne 4 -or $recordAddress -ne 0) {
                    throw "Invalid Intel HEX start-segment record: $line"
                }
            }
            4 {
                if ($byteCount -ne 2 -or $recordAddress -ne 0) {
                    throw "Invalid Intel HEX extended-linear record: $line"
                }
                $upperAddress = [uint64]([Convert]::ToUInt32($line.Substring(9, 4), 16)) -shl 16
            }
            5 {
                if ($byteCount -ne 4 -or $recordAddress -ne 0) {
                    throw "Invalid Intel HEX start-linear record: $line"
                }
            }
            default {
                throw ("Unsupported Intel HEX record type 0x{0:X2}: {1}" -f $recordType, $line)
            }
        }
    }

    if (-not $sawEof) {
        throw "Intel HEX file has no EOF record: $Path"
    }
    if ($dataBytes -eq 0) {
        throw "Refusing to flash an Intel HEX file with no data: $Path"
    }
    if ($minimumAddress -ne 0) {
        throw ("Refusing to flash an image without a vector table at 0x00000000 (first data is 0x{0:X8})" -f $minimumAddress)
    }

    Write-Ok ("Main-Flash HEX: {0} bytes | 0x{1:X8}-0x{2:X8}" -f $dataBytes, $minimumAddress, $maximumAddress)
}

function Set-IsolatedEnvironment {
    $env:ARM_GCC_ROOT = $ArmGccRoot
    $env:CMAKE_ROOT = $CMakeRoot
    $env:NINJA_ROOT = $NinjaRoot
    $env:OPENOCD_ROOT = $OpenOcdRoot
    $env:OPENOCD_SCRIPTS = $OpenOcdScripts
    $env:MSPM0_SDK_ROOT = $Mspm0SdkRoot
    $env:SYSCONFIG_ROOT = $SysConfigRoot
    $env:HOME = $env:USERPROFILE
    $systemRoot = [Environment]::GetEnvironmentVariable("SystemRoot", "Machine")
    if ([string]::IsNullOrWhiteSpace($systemRoot)) {
        $systemRoot = "C:\Windows"
    }
    $env:Path = "$(Join-Path $ArmGccRoot 'bin');$(Join-Path $CMakeRoot 'bin');$NinjaRoot;$(Join-Path $systemRoot 'System32');$systemRoot"
}

function Get-PresetName {
    $shortDevice = if ($Device -eq "MSPM0G3507") { "g3507" } else { "g3519" }
    $configurationName = $Configuration.ToLowerInvariant()
    if ($Profile -eq "Current") {
        return "$shortDevice-hex25e-$configurationName"
    }
    return "$shortDevice-$configurationName"
}

function Get-BuildDirectory {
    if ($Profile -eq "Current") {
        return Join-Path $WorkspaceRoot "build\Tasks\Hex25E\$Device\$Configuration"
    }
    return Join-Path $WorkspaceRoot "build\$Device\$Configuration"
}

function Write-BuildManifest {
    $compileCommandsPath = Join-Path (Get-BuildDirectory) "compile_commands.json"
    Assert-File -Path $compileCommandsPath
    $commands = Get-Content -LiteralPath $compileCommandsPath -Raw -Encoding UTF8 |
        ConvertFrom-Json
    $files = @($commands | ForEach-Object {
        $_.file.ToString().Replace("\", "/")
    })
    $bspCount = @($files | Where-Object { $_ -match "/BSP/" }).Count
    $componentCount = @($files | Where-Object {
        $_ -match "/APP/Algorithm/"
    }).Count
    $deviceCount = @($files | Where-Object { $_ -match "/Device/" }).Count
    $driverLibCount = @($files | Where-Object {
        $_ -match "/source/ti/driverlib/"
    }).Count
    $freeRtosCount = @($files | Where-Object { $_ -match "/FreeRTOS/" }).Count

    Write-Ok ("Compile units: {0} | BSP {1} | Algorithm {2} | Device {3} | DriverLib {4} | FreeRTOS {5}" -f
        $files.Count, $bspCount, $componentCount, $deviceCount,
        $driverLibCount, $freeRtosCount)
    foreach ($file in $files) {
        Write-Verbose ("Compile unit: {0}" -f $file)
    }
}

function Publish-CurrentArtifacts {
    if ($Profile -ne "Current") {
        return
    }
    $source = Get-BuildDirectory
    $destination = Join-Path $WorkspaceRoot "build\Current\$Device\$Configuration"
    New-Item -ItemType Directory -Path $destination -Force | Out-Null
    foreach ($extension in @("elf", "hex", "bin")) {
        $artifact = Join-Path $source "$Device.$extension"
        if (Test-Path -LiteralPath $artifact -PathType Leaf) {
            Copy-Item -LiteralPath $artifact -Destination $destination -Force
        }
    }
    Write-Ok ("Debug artifacts: {0}" -f $destination)
}

function Invoke-Configure {
    $preset = Get-PresetName
    Write-Stage "Configure CMake ($preset)"
    Invoke-Native -FilePath $CMake -Arguments @("--preset", $preset, "-S", $WorkspaceRoot) `
        -DisplayName "CMake configure" -QuietOnSuccess `
        -KeepOnSuccess @("(?i)warning", "(?i)error") -ShowAllWhenKept
}

function Invoke-Build {
    Invoke-Configure
    $preset = Get-PresetName
    Write-Stage "Compile ($preset)"
    Invoke-Native -FilePath $CMake -Arguments @("--build", "--preset", "build-$preset") `
        -DisplayName "CMake build" -QuietOnSuccess `
        -KeepOnSuccess @("^ninja:", "(?i)warning:", "(?i)error:") `
        -ShowAllWhenKept

    Write-BuildManifest
    $elf = Join-Path (Get-BuildDirectory) "$Device.elf"
    Assert-File -Path $elf
    Write-Stage "Firmware size"
    Invoke-Native -FilePath $Size -Arguments @($elf) -DisplayName "GNU size"

    # Validate after every build, including a no-op incremental build. This
    # ensures Cortex-Debug cannot load a manually modified or stale HEX merely
    # because Ninja decided that the ELF did not need to be relinked.
    $hex = Join-Path (Get-BuildDirectory) "$Device.hex"
    Assert-File -Path $hex
    $mainFlashSize = if ($Device -eq "MSPM0G3507") { 0x20000 } else { 0x80000 }
    Write-Stage "Validate main-Flash HEX"
    Assert-MainFlashHex -Path $hex -FlashSize $mainFlashSize
    Publish-CurrentArtifacts
}

$requiredFiles = switch ($Action) {
    "Doctor"    { @($CMake, $Ninja, $OpenOcd, $Gcc, $Gdb, $SysConfigNode, $SysConfigCli, $SysConfigNw, $SysConfigProduct) }
    "Configure" { @($CMake, $Ninja, $Gcc, $SysConfigNode, $SysConfigCli, $SysConfigProduct) }
    "Build"     { @($CMake, $Ninja, $Gcc, $Size, $SysConfigNode, $SysConfigCli, $SysConfigProduct) }
    "Clean"     { @($CMake, $Ninja) }
    "Purge"     { @() }
    "Generate"  { @($SysConfigNode, $SysConfigCli, $SysConfigProduct) }
    "Flash"     { @($CMake, $Ninja, $OpenOcd, $Gcc, $Gdb, $Size, $SysConfigNode, $SysConfigCli, $SysConfigProduct) }
    "SysConfig" { @($SysConfigNw, $SysConfigProduct) }
}
foreach ($requiredFile in $requiredFiles) {
    Assert-File -Path $requiredFile
}
if ($Action -in @("Doctor", "Flash")) {
    if ($UseG3519OpenOcd) {
        Assert-FileHash -Path $OpenOcd -ExpectedSha256 $ExpectedG3519OpenOcdSha256 -Label "G3519 OpenOCD binary"
        Assert-FileHash -Path $G3519LibUsb -ExpectedSha256 $ExpectedG3519LibUsbSha256 -Label "G3519 OpenOCD libusb runtime"
        Assert-FileHash -Path $G3519LibHidApi -ExpectedSha256 $ExpectedG3519LibHidApiSha256 -Label "G3519 OpenOCD hidapi runtime"
        Assert-FileHash -Path $G3519TargetScript -ExpectedSha256 $ExpectedG3519TargetScriptSha256 -Label "G3519 OpenOCD MSPM0 target script"
        Assert-FileHash -Path $G3519ProjectCfg -ExpectedSha256 $ExpectedG3519ProjectCfgSha256 -Label "G3519 project OpenOCD config"
    }
    else {
        Assert-FileHash -Path $OpenOcd -ExpectedSha256 $ExpectedStockOpenOcdSha256 -Label "G3507 stock OpenOCD binary"
        Assert-FileHash -Path $G3507ProjectCfg -ExpectedSha256 $ExpectedG3507ProjectCfgSha256 -Label "G3507 project OpenOCD config"
    }
}
if ($Action -in @("Doctor", "Flash")) {
    Assert-Directory -Path $OpenOcdScripts
}
if ($Action -in @("Doctor", "SysConfig")) {
    Assert-Directory -Path $SysConfigApp
}
Set-IsolatedEnvironment

$actionTimer = [Diagnostics.Stopwatch]::StartNew()
Write-Host ""
$actionHeader = if ($Action -eq "Doctor") {
    "[MSPM0] {0} | {1} | {2}" -f $Action.ToUpperInvariant(), $Device, $Probe
}
elseif ($Action -eq "SysConfig") {
    "[MSPM0] {0} | {1}" -f $Action.ToUpperInvariant(), $Device
}
else {
    "[MSPM0] {0} | {1} | {2}" -f $Action.ToUpperInvariant(), $Device, $Configuration
}
Write-Host $actionHeader -ForegroundColor White
Write-Host ("-" * 72) -ForegroundColor DarkGray

Push-Location $WorkspaceRoot
try {
    switch ($Action) {
        "Doctor" {
            Write-Stage "Read local $Probe config and check USB presence"
            Assert-ProbePresent
            Write-Ok ("{0} is present" -f $Probe)
            Write-Host "  Probe cfg : $ProbeLocalCfg"
            Write-Stage "Check isolated toolchain"
            Write-Host "  Workspace : $WorkspaceRoot"
            Write-Host "  OpenOCD   : $OpenOcdFlavor"
            Write-Host ("  GCC       : {0}" -f (Get-NativeVersionLine -FilePath $Gcc -Arguments @("--version") -DisplayName "GCC"))
            Write-Host ("  GDB       : {0}" -f (Get-NativeVersionLine -FilePath $Gdb -Arguments @("--version") -DisplayName "GDB"))
            Write-Host ("  CMake     : {0}" -f (Get-NativeVersionLine -FilePath $CMake -Arguments @("--version") -DisplayName "CMake"))
            Write-Host ("  Ninja     : {0}" -f (Get-NativeVersionLine -FilePath $Ninja -Arguments @("--version") -DisplayName "Ninja"))
            Write-Host ("  OpenOCD   : {0}" -f (Get-NativeVersionLine -FilePath $OpenOcd -Arguments @("--version") -DisplayName "OpenOCD"))
            Write-Host ("  SysConfig : {0}" -f (Get-NativeVersionLine -FilePath $SysConfigNode -Arguments @($SysConfigCli, "--version") -DisplayName "SysConfig"))
        }
        "Configure" {
            Invoke-Configure
        }
        "Build" {
            Invoke-Build
        }
        "Clean" {
            $preset = Get-PresetName
            $cache = Join-Path (Get-BuildDirectory) "CMakeCache.txt"
            if (Test-Path -LiteralPath $cache) {
                Write-Stage "Clean ($preset)"
                Invoke-Native -FilePath $CMake -Arguments @("--build", "--preset", "build-$preset", "--target", "clean") `
                    -DisplayName "CMake clean" -QuietOnSuccess `
                    -KeepOnSuccess @("^ninja:", "(?i)warning:", "(?i)error:") `
                    -ShowAllWhenKept
            }
            else {
                Write-Host "No build directory to clean."
            }
        }
        "Purge" {
            $buildRoot = [System.IO.Path]::GetFullPath(
                (Join-Path $WorkspaceRoot "build"))
            $target = [System.IO.Path]::GetFullPath((Get-BuildDirectory))
            if (-not $target.StartsWith(
                    ($buildRoot + [System.IO.Path]::DirectorySeparatorChar),
                    [System.StringComparison]::OrdinalIgnoreCase)) {
                throw "Refusing purge outside build root: $target"
            }
            if (Test-Path -LiteralPath $target) {
                Write-Stage "Purge CMake cache and outputs"
                Remove-Item -LiteralPath $target -Recurse -Force
                Write-Ok ("Purged {0}" -f $target)
            }
            else {
                Write-Host "No build directory to purge."
            }
        }
        "Generate" {
            $syscfg = Join-Path $WorkspaceRoot "BSP\$Device\$Device.syscfg"
            $output = Join-Path (Get-BuildDirectory) "generated"
            Assert-File -Path $syscfg
            New-Item -ItemType Directory -Path $output -Force | Out-Null
            $package = if ($Device -eq "MSPM0G3507") { "LQFP-64(PM)" } else { "LQFP-80(PN)" }
            Write-Stage "Generate SysConfig sources"
            Invoke-Native -FilePath $SysConfigNode -Arguments @(
                $SysConfigCli,
                "--product", $SysConfigProduct,
                "--compiler", "gcc",
                "--device", $Device,
                "--package", $package,
                "--script", $syscfg,
                "--output", $output
            ) -DisplayName "SysConfig generate" -QuietOnSuccess `
                -KeepOnSuccess @("(?i)warning", "(?i)error") `
                -ShowAllWhenKept
        }
        "Flash" {
            Write-Stage "Read local $Probe selection"
            $probeSerial = Get-ProbeSerial
            Write-Ok ("{0} serial: {1}" -f $Probe, $probeSerial)
            Invoke-Build
            $hex = Join-Path (Get-BuildDirectory) "$Device.hex"
            $bin = Join-Path (Get-BuildDirectory) "$Device.bin"
            $readback = Join-Path (Get-BuildDirectory) "$Device.readback.bin"
            Assert-File -Path $bin
            $binLength = (Get-Item -LiteralPath $bin).Length
            if (Test-Path -LiteralPath $readback -PathType Leaf) {
                Remove-Item -LiteralPath $readback -Force
            }

            $cfgName = if ($Device -eq "MSPM0G3519") {
                "mspm0_daplink_custom.cfg"
            }
            else {
                "mspm0_xds110.cfg"
            }
            $cfg = Join-Path $OpenOcdConfigRoot $cfgName
            Assert-File -Path $cfg
            $hexForOpenOcd = $hex.Replace("\", "/")
            $readbackForOpenOcd = $readback.Replace("\", "/")
            Write-Stage "Program and read back via $Probe"
            Invoke-Native -FilePath $OpenOcd -Arguments @(
                "-s", $OpenOcdScripts,
                "-f", $cfg,
                "-c", "init",
                "-c", "reset halt",
                "-c", "flash write_image erase {$hexForOpenOcd}",
                "-c", "reset halt",
                "-c", "dump_image {$readbackForOpenOcd} 0x00000000 $binLength",
                "-c", "shutdown"
            ) -DisplayName "OpenOCD flash" -QuietOnSuccess `
                -KeepOnSuccess @("^Info : .*serial number", "^Info : .*connected to target", "processor detected", "^wrote ", "^dumped ", "^Warn :", "^Error:")

            Assert-File -Path $readback
            $expectedHash = (Get-FileHash -Algorithm SHA256 -LiteralPath $bin).Hash
            $actualHash = (Get-FileHash -Algorithm SHA256 -LiteralPath $readback).Hash
            Remove-Item -LiteralPath $readback -Force
            if ($actualHash -ne $expectedHash) {
                throw "Flash read-back verification failed; target remains halted."
            }
            Write-Ok ("Read-back verify: {0} bytes, SHA-256 match" -f $binLength)

            Write-Stage "Reset target and run"
            Invoke-Native -FilePath $OpenOcd -Arguments @(
                "-s", $OpenOcdScripts,
                "-f", $cfg,
                "-c", "init",
                "-c", "reset run",
                "-c", "shutdown"
            ) -DisplayName "OpenOCD reset" -QuietOnSuccess `
                -KeepOnSuccess @("processor detected", "^Warn :", "^Error:")
        }
        "SysConfig" {
            $syscfg = Join-Path $WorkspaceRoot "BSP\$Device\$Device.syscfg"
            Assert-File -Path $syscfg
            $arguments = @(
                "`"$SysConfigApp`"",
                "--product", "`"$SysConfigProduct`"",
                "--script", "`"$syscfg`""
            )
            Write-Stage "Open SysConfig GUI"
            $process = Start-Process -FilePath $SysConfigNw -ArgumentList $arguments `
                -WorkingDirectory $SysConfigRoot -PassThru
            Start-Sleep -Milliseconds 1000
            if ($process.HasExited) {
                throw "TI SysConfig exited during startup with code $($process.ExitCode)."
            }
            Write-Host "  PID        : $($process.Id)"
            Write-Host "  Config     : $syscfg"
        }
    }

    $actionTimer.Stop()
    Write-Host ("-" * 72) -ForegroundColor DarkGray
    Write-Ok ("{0} completed in {1:N1} s" -f $Action, $actionTimer.Elapsed.TotalSeconds)
}
finally {
    Pop-Location
}
