[CmdletBinding()]
param(
    [string]$Root
)

Set-StrictMode -Version Latest
$ErrorActionPreference = "Stop"

if ([string]::IsNullOrWhiteSpace($Root)) {
    $Root = Split-Path -Parent $PSScriptRoot
}
$Root = (Resolve-Path -LiteralPath $Root).Path
$findings = [System.Collections.Generic.List[string]]::new()

function Get-RelativeTemplatePath {
    param([Parameter(Mandatory = $true)][string]$Path)

    $fullPath = [System.IO.Path]::GetFullPath($Path)
    if ($fullPath.StartsWith(
            ($Root + [System.IO.Path]::DirectorySeparatorChar),
            [System.StringComparison]::OrdinalIgnoreCase)) {
        return $fullPath.Substring($Root.Length + 1)
    }
    return $fullPath
}

function Add-Finding {
    param(
        [Parameter(Mandatory = $true)][string]$Path,
        [Parameter(Mandatory = $true)][int]$Line,
        [Parameter(Mandatory = $true)][string]$Reason
    )

    $relative = Get-RelativeTemplatePath -Path $Path
    $findings.Add(("{0}:{1}: {2}" -f $relative, $Line, $Reason))
}

# Keep generated output, executable configuration, and project records in
# separate roots. This prevents old root-level Config/cmake/debug directories
# from silently returning and becoming competing configuration sources.
$rootDirectories = @(Get-ChildItem -LiteralPath $Root -Directory -Force)
foreach ($legacyName in @("Config", "cmake", "debug", "Profile", "System")) {
    $legacyDirectory = $rootDirectories |
        Where-Object { $_.Name -ceq $legacyName } |
        Select-Object -First 1
    if ($null -ne $legacyDirectory) {
        Add-Finding -Path $legacyDirectory.FullName -Line 1 -Reason (
            "legacy root directory must be migrated to config/ or docs/")
    }
}

$requiredConfigFiles = @(
    "config\README.md",
    "config\cmake\arm-gcc-toolchain.cmake",
    "config\cmake\DriverLibModules.cmake",
    "config\openocd\README.md",
    "config\openocd\daplink.local.cfg.example",
    "config\openocd\xds110.local.cfg.example",
    "config\openocd\mspm0_daplink_custom.cfg",
    "config\openocd\mspm0_xds110.cfg"
)
foreach ($relative in $requiredConfigFiles) {
    $path = Join-Path $Root $relative
    if (-not (Test-Path -LiteralPath $path -PathType Leaf)) {
        Add-Finding -Path $path -Line 1 -Reason "required configuration file is missing"
    }
}

$configRoot = Join-Path $Root "config"
if (Test-Path -LiteralPath $configRoot -PathType Container) {
    $allowedConfigDirectories = @("config\cmake", "config\openocd")
    $configDirectories = Get-ChildItem -LiteralPath $configRoot -Directory -Recurse -Force
    foreach ($directory in $configDirectories) {
        $relative = (Get-RelativeTemplatePath -Path $directory.FullName).Replace("/", "\")
        if ($relative -notin $allowedConfigDirectories) {
            Add-Finding -Path $directory.FullName -Line 1 -Reason (
                "unexpected configuration directory; use config/cmake or config/openocd")
        }
    }

    $configFiles = Get-ChildItem -LiteralPath $configRoot -File -Recurse -Force
    foreach ($file in $configFiles) {
        $relative = (Get-RelativeTemplatePath -Path $file.FullName).Replace("/", "\")
        $allowed = (
            $relative -ceq "config\README.md" -or
            $relative -cmatch '^config\\[^\\]+_config\.h$' -or
            $relative -cmatch '^config\\cmake\\[^\\]+\.cmake$' -or
            $relative -ceq "config\openocd\README.md" -or
            $relative -cmatch '^config\\openocd\\[^\\]+\.cfg(?:\.example)?$'
        )
        if (-not $allowed) {
            Add-Finding -Path $file.FullName -Line 1 -Reason (
                "non-declarative file is not allowed under config")
        }
    }

    $openOcdRoot = Join-Path $configRoot "openocd"
    if (Test-Path -LiteralPath $openOcdRoot -PathType Container) {
        $openOcdCfgFiles = Get-ChildItem -LiteralPath $openOcdRoot -File -Force |
            Where-Object { $_.Name -match '\.cfg(?:\.example)?$' }
        foreach ($file in $openOcdCfgFiles) {
            $serialMatches = @(Select-String -LiteralPath $file.FullName -Pattern (
                '^\s*adapter\s+serial\b'))
            foreach ($serialMatch in $serialMatches) {
                Add-Finding -Path $file.FullName -Line $serialMatch.LineNumber -Reason (
                    "real probe selection must stay in sibling TI ENV/local/openocd")
            }
        }
    }
}

$docsRoot = Join-Path $Root "docs"
if (Test-Path -LiteralPath $docsRoot -PathType Container) {
    $nonMarkdownDocs = Get-ChildItem -LiteralPath $docsRoot -File -Recurse -Force |
        Where-Object { $_.Extension -cne ".md" }
    foreach ($file in $nonMarkdownDocs) {
        Add-Finding -Path $file.FullName -Line 1 -Reason (
            "only project records in Markdown form are allowed under docs")
    }
}

$gitIgnorePath = Join-Path $Root ".gitignore"
$gitIgnoreLines = @(
    Get-Content -LiteralPath $gitIgnorePath -Encoding UTF8 |
        ForEach-Object { $_.Trim() }
)
if ("/build/" -notin $gitIgnoreLines) {
    Add-Finding -Path $gitIgnorePath -Line 1 -Reason (
        "generated build root must remain explicitly ignored")
}

$upperLayerRoots = @(
    "APP",
    "BSP\include",
    "APP\Algorithm",
    "Device",
    "Mode",
    "Task"
)
$forbiddenPatterns = @(
    @{
        Pattern = '#\s*include\s*[<"](?:ti_msp_dl_config|ti/driverlib|driverlib)'
        Reason  = "upper layer includes MCU/DriverLib headers"
    },
    @{
        Pattern = '\bDL_[A-Za-z0-9_]+'
        Reason  = "upper layer calls DriverLib directly"
    },
    @{
        Pattern = '\b(?:PINCM\d+|GPIO[ABC]|TIMA\d+|TIMG\d+|UART\d+|DMA_CH\d+)\b'
        Reason  = "upper layer contains a fixed peripheral or pin token"
    },
    @{
        Pattern = '\b__MSPM0[A-Za-z0-9_]*__\b'
        Reason  = "upper layer branches on an MCU macro"
    }
)

foreach ($relativeRoot in $upperLayerRoots) {
    $directory = Join-Path $Root $relativeRoot
    if (-not (Test-Path -LiteralPath $directory -PathType Container)) {
        continue
    }

    $files = Get-ChildItem -LiteralPath $directory -File -Recurse |
        Where-Object { $_.Extension -in @(".c", ".h") }
    foreach ($file in $files) {
        foreach ($rule in $forbiddenPatterns) {
            $matches = @(Select-String -LiteralPath $file.FullName -Pattern $rule.Pattern -AllMatches)
            foreach ($match in $matches) {
                Add-Finding -Path $file.FullName -Line $match.LineNumber -Reason $rule.Reason
            }
        }
    }
}

# APP is the pure algorithm layer. Product modes and execution concerns must
# not leak back into reusable math and protocol components.
$appDirectory = Join-Path $Root "APP"
if (Test-Path -LiteralPath $appDirectory -PathType Container) {
    $appFiles = Get-ChildItem -LiteralPath $appDirectory -File -Recurse |
        Where-Object { $_.Extension -in @(".c", ".h") }
    $appDependencyPatterns = @(
        @{
            Pattern = '#\s*include\s*[<"](?:bsp_|device_|mode_|FreeRTOS|task\.h)'
            Reason = "APP algorithm includes BSP, Device, Mode, or FreeRTOS"
        },
        @{
            Pattern = '\b(?:xTask|vTask|osThread|TaskHandle|xQueue|xSemaphore)[A-Za-z0-9_]*\b'
            Reason = "APP algorithm contains task/RTOS scheduling code"
        }
    )
    foreach ($file in $appFiles) {
        foreach ($rule in $appDependencyPatterns) {
            $matches = @(Select-String -LiteralPath $file.FullName -Pattern $rule.Pattern -AllMatches)
            foreach ($match in $matches) {
                Add-Finding -Path $file.FullName -Line $match.LineNumber -Reason $rule.Reason
            }
        }
    }
}

# Mode owns business state and safety decisions. It may depend on APP
# algorithms, but not on hardware ownership or FreeRTOS scheduling.
$modeDirectory = Join-Path $Root "Mode"
if (Test-Path -LiteralPath $modeDirectory -PathType Container) {
    $modeFiles = Get-ChildItem -LiteralPath $modeDirectory -File -Recurse |
        Where-Object { $_.Extension -in @(".c", ".h") }
    $modeDependencyPatterns = @(
        @{
            Pattern = '#\s*include\s*[<"](?:bsp_|device_|app_|profile_)'
            Reason = "Mode includes BSP, Device, APP, or Product wrapper"
        },
        @{
            Pattern = '#\s*include\s*[<"](?:FreeRTOS|task|queue|semphr|timers)\.h[>"]'
            Reason = "Mode includes an RTOS/task header"
        },
        @{
            Pattern = '\b(?:xTask|vTask|osThread|TaskHandle|xQueue|xSemaphore)[A-Za-z0-9_]*\b'
            Reason = "Mode contains task/RTOS scheduling code"
        }
    )
    foreach ($file in $modeFiles) {
        foreach ($rule in $modeDependencyPatterns) {
            $matches = @(Select-String -LiteralPath $file.FullName -Pattern $rule.Pattern -AllMatches)
            foreach ($match in $matches) {
                Add-Finding -Path $file.FullName -Line $match.LineNumber -Reason $rule.Reason
            }
        }
    }
}

# Task owns FreeRTOS execution and Device/Mode data movement, but never MCU
# registers or BSP backends directly.
$taskDirectory = Join-Path $Root "Task"
if (Test-Path -LiteralPath $taskDirectory -PathType Container) {
    $taskSources = Get-ChildItem -LiteralPath $taskDirectory -File -Recurse |
        Where-Object { $_.Extension -in @(".c", ".h") }
    $taskHardwarePatterns = @(
        @{
            Pattern = '#\s*include\s*[<"](?:bsp_|ti_msp_dl_config|ti/driverlib|driverlib)'
            Reason = "Task bypasses Device and accesses BSP/MCU directly"
        },
        @{
            Pattern = '\bDL_[A-Za-z0-9_]+'
            Reason = "Task calls DriverLib directly"
        }
    )
    foreach ($file in $taskSources) {
        foreach ($rule in $taskHardwarePatterns) {
            $matches = @(Select-String -LiteralPath $file.FullName -Pattern $rule.Pattern -AllMatches)
            foreach ($match in $matches) {
                Add-Finding -Path $file.FullName -Line $match.LineNumber -Reason $rule.Reason
            }
        }
    }

    $taskEntryPath = Join-Path $taskDirectory "task_app.c"
    if (-not (Test-Path -LiteralPath $taskEntryPath -PathType Leaf)) {
        Add-Finding -Path $taskEntryPath -Line 1 -Reason "FreeRTOS task entry is missing"
    }
    else {
        $taskEntry = Get-Content -LiteralPath $taskEntryPath -Raw -Encoding UTF8
        foreach ($requiredToken in @(
                '#include "FreeRTOS.h"',
                '#include "task.h"',
                'xTaskCreateStatic',
                'vTaskDelayUntil')) {
            if (-not $taskEntry.Contains($requiredToken)) {
                Add-Finding -Path $taskEntryPath -Line 1 -Reason (
                    "FreeRTOS task entry is missing '{0}'" -f $requiredToken)
            }
        }
    }
}

$sourceLikeFiles = Get-ChildItem -LiteralPath $Root -File -Recurse -Force |
    Where-Object {
        $_.FullName -notmatch '[\\/]build[\\/]' -and
        $_.Extension -in @(".c", ".h", ".a", ".lib")
    }
foreach ($file in $sourceLikeFiles) {
    $relative = Get-RelativeTemplatePath -Path $file.FullName
    if ($file.Extension -in @(".a", ".lib")) {
        Add-Finding -Path $file.FullName -Line 1 -Reason "precompiled library is not allowed outside build"
    }
    elseif ($relative -match '(?i)(?:^|[\\/])SeekFree(?:[\\/]|$)' -or
        $file.Name -match '(?i)^zf_') {
        Add-Finding -Path $file.FullName -Line 1 -Reason "forbidden SeekFree/zf source name"
    }
}

$cmakeFiles = Get-ChildItem -LiteralPath $Root -File -Recurse -Force |
    Where-Object {
        $_.FullName -notmatch '[\\/]build[\\/]' -and
        ($_.Name -eq "CMakeLists.txt" -or $_.Extension -eq ".cmake")
    }
foreach ($file in $cmakeFiles) {
    $matches = @(Select-String -LiteralPath $file.FullName -Pattern '^\s*file\s*\(\s*GLOB(?:_RECURSE)?\b')
    foreach ($match in $matches) {
        Add-Finding -Path $file.FullName -Line $match.LineNumber -Reason "source glob would compile files implicitly"
    }
}

$jsonFiles = @(
    "CMakePresets.json",
    ".vscode\c_cpp_properties.json",
    ".vscode\extensions.json",
    ".vscode\launch.json",
    ".vscode\settings.json",
    ".vscode\tasks.json"
)
foreach ($relative in $jsonFiles) {
    $path = Join-Path $Root $relative
    try {
        Get-Content -LiteralPath $path -Raw -Encoding UTF8 |
            ConvertFrom-Json | Out-Null
    }
    catch {
        Add-Finding -Path $path -Line 1 -Reason "invalid JSON"
    }
}

$tasksPath = Join-Path $Root ".vscode\tasks.json"
$launchPath = Join-Path $Root ".vscode\launch.json"
try {
    $tasksDocument = Get-Content -LiteralPath $tasksPath -Raw -Encoding UTF8 |
        ConvertFrom-Json
    $launchDocument = Get-Content -LiteralPath $launchPath -Raw -Encoding UTF8 |
        ConvertFrom-Json
    $taskLabels = @($tasksDocument.tasks | ForEach-Object { $_.label })

    $duplicateTaskLabels = @($taskLabels |
        Group-Object |
        Where-Object { $_.Count -gt 1 } |
        ForEach-Object { $_.Name })
    foreach ($duplicateTaskLabel in $duplicateTaskLabels) {
        Add-Finding -Path $tasksPath -Line 1 -Reason (
            "duplicate task label '{0}'" -f $duplicateTaskLabel)
    }

    foreach ($task in $tasksDocument.tasks) {
        $dependsOnProperty = $task.PSObject.Properties["dependsOn"]
        if ($null -eq $dependsOnProperty) {
            continue
        }

        foreach ($dependency in @($dependsOnProperty.Value)) {
            if ([string]::IsNullOrWhiteSpace($dependency) -or
                $dependency -notin $taskLabels) {
                Add-Finding -Path $tasksPath -Line 1 -Reason (
                    "unknown dependsOn task '{0}'" -f $dependency)
            }
        }
    }

    foreach ($configuration in $launchDocument.configurations) {
        $configFilesProperty =
            $configuration.PSObject.Properties["configFiles"]
        if ($null -ne $configFilesProperty) {
            foreach ($configReference in @($configFilesProperty.Value)) {
                $openOcdPrefix = '${workspaceFolder}/config/openocd/'
                if (-not $configReference.StartsWith(
                        $openOcdPrefix,
                        [System.StringComparison]::Ordinal)) {
                    Add-Finding -Path $launchPath -Line 1 -Reason (
                        "OpenOCD configFiles must resolve under config/openocd")
                    continue
                }

                $workspacePrefix = '${workspaceFolder}/'
                $relativeConfigPath = $configReference.Substring(
                    $workspacePrefix.Length).Replace("/", "\")
                $configPath = Join-Path $Root $relativeConfigPath
                if (-not (Test-Path -LiteralPath $configPath -PathType Leaf)) {
                    Add-Finding -Path $launchPath -Line 1 -Reason (
                        "OpenOCD configFiles target does not exist: {0}" -f
                        $configReference)
                }
            }
        }

        $preLaunchTaskProperty =
            $configuration.PSObject.Properties["preLaunchTask"]
        if ($null -eq $preLaunchTaskProperty) {
            continue
        }

        $preLaunchTask = $preLaunchTaskProperty.Value
        if (-not [string]::IsNullOrWhiteSpace($preLaunchTask) -and
            $preLaunchTask -notin $taskLabels) {
            Add-Finding -Path $launchPath -Line 1 -Reason (
                "unknown preLaunchTask '{0}'" -f $preLaunchTask)
        }
    }
}
catch {
    Add-Finding -Path $launchPath -Line 1 -Reason (
        "cannot validate VS Code task references: {0}" -f $_.Exception.Message)
}

if ($findings.Count -ne 0) {
    Write-Host "[FAIL] Template architecture check" -ForegroundColor Red
    foreach ($finding in $findings) {
        Write-Host ("  {0}" -f $finding)
    }
    exit 1
}

Write-Host "[OK]   Template architecture check" -ForegroundColor Green
Write-Host "       Upper layers do not own MCU resources; VS Code task references are valid."
Write-Host "       APP/Mode stay pure; Task owns FreeRTOS execution without bypassing Device."
Write-Host "       No source glob, forbidden source name, or precompiled library found."
Write-Host "       Config, docs, and generated build output have separate enforced roots."
