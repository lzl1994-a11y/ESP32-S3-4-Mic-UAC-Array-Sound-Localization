[CmdletBinding()]
param(
    [Parameter()]
    [string]$CodexHome = (Join-Path ([Environment]::GetFolderPath('UserProfile')) '.codex'),

    [Parameter()]
    [switch]$DryRun,

    [Parameter()]
    [switch]$AutoApprove,

    [Parameter()]
    [switch]$SkipProcessCheck,

    [Parameter()]
    [string]$SqlitePath
)

Set-StrictMode -Version Latest
$ErrorActionPreference = 'Stop'

$utf8NoBom = New-Object System.Text.UTF8Encoding($false)
$sourceProvider = 'custom'
$targetProvider = 'openai'

function Write-Info {
    param([Parameter(Mandatory)][string]$Message)
    Write-Host "[信息] $Message" -ForegroundColor Cyan
}

function Write-Success {
    param([Parameter(Mandatory)][string]$Message)
    Write-Host "[完成] $Message" -ForegroundColor Green
}

function Resolve-SqliteExecutable {
    param([string]$RequestedPath)

    if (-not [string]::IsNullOrWhiteSpace($RequestedPath)) {
        if (Test-Path -LiteralPath $RequestedPath -PathType Leaf) {
            return [System.IO.Path]::GetFullPath($RequestedPath)
        }
        throw "找不到指定的 sqlite3.exe：$RequestedPath"
    }

    $sqliteCommand = Get-Command 'sqlite3.exe' -ErrorAction SilentlyContinue
    if ($null -ne $sqliteCommand) {
        return $sqliteCommand.Source
    }

    $knownPaths = @(
        'D:\AndroidTool\AndroidSDK\platform-tools\sqlite3.exe'
    )
    foreach ($knownPath in $knownPaths) {
        if (Test-Path -LiteralPath $knownPath -PathType Leaf) {
            return $knownPath
        }
    }

    throw '找不到 sqlite3.exe，无法安全迁移任务数据库。'
}

function Invoke-Sqlite {
    param(
        [Parameter(Mandatory)][string]$Executable,
        [Parameter(Mandatory)][string]$DatabasePath,
        [Parameter(Mandatory)][string]$Query,
        [switch]$ReadOnly
    )

    $arguments = New-Object System.Collections.Generic.List[string]
    if ($ReadOnly) {
        $arguments.Add('-readonly')
    }
    $arguments.Add($DatabasePath)
    $arguments.Add($Query)

    $output = @(& $Executable $arguments.ToArray() 2>&1)
    if ($LASTEXITCODE -ne 0) {
        throw "sqlite3 执行失败（退出码 $LASTEXITCODE）：$($output -join [Environment]::NewLine)"
    }

    return $output
}

function Get-CustomThreadCount {
    param(
        [Parameter(Mandatory)][string]$Executable,
        [Parameter(Mandatory)][string]$DatabasePath
    )

    $output = Invoke-Sqlite -Executable $Executable -DatabasePath $DatabasePath -ReadOnly -Query "SELECT COUNT(*) FROM threads WHERE model_provider = 'custom';"
    $text = ($output -join '').Trim()
    $count = 0
    if (-not [int]::TryParse($text, [ref]$count)) {
        throw "无法解析数据库中的 custom 任务数量：$text"
    }
    return $count
}

function Get-SessionHeader {
    param([Parameter(Mandatory)][string]$Path)

    $stream = [System.IO.File]::Open(
        $Path,
        [System.IO.FileMode]::Open,
        [System.IO.FileAccess]::Read,
        [System.IO.FileShare]::ReadWrite
    )
    $buffer = New-Object System.IO.MemoryStream

    try {
        $hasLineFeed = $false
        while (($value = $stream.ReadByte()) -ne -1) {
            if ($value -eq 10) {
                $hasLineFeed = $true
                break
            }

            $buffer.WriteByte([byte]$value)
            if ($buffer.Length -gt 16777216) {
                throw "聊天元数据首行超过 16 MiB，拒绝自动处理：$Path"
            }
        }

        $headerBytes = $buffer.ToArray()
        $usesCrLf = $headerBytes.Length -gt 0 -and $headerBytes[$headerBytes.Length - 1] -eq 13
        if ($usesCrLf) {
            $headerBytes = $headerBytes[0..($headerBytes.Length - 2)]
        }

        $headerText = $utf8NoBom.GetString($headerBytes)
        $metadata = $headerText | ConvertFrom-Json

        [PSCustomObject]@{
            Metadata    = $metadata
            HasLineFeed = $hasLineFeed
            UsesCrLf    = $usesCrLf
        }
    }
    finally {
        $buffer.Dispose()
        $stream.Dispose()
    }
}

function Test-CustomSession {
    param([Parameter(Mandatory)][string]$Path)

    try {
        $header = Get-SessionHeader -Path $Path
        return (
            $header.Metadata.type -eq 'session_meta' -and
            $null -ne $header.Metadata.payload -and
            $header.Metadata.payload.model_provider -eq $sourceProvider
        )
    }
    catch {
        Write-Warning "跳过无法读取的聊天文件：$Path`n$($_.Exception.Message)"
        return $false
    }
}

function Update-SessionProvider {
    param([Parameter(Mandatory)][string]$Path)

    $resolvedPath = [System.IO.Path]::GetFullPath($Path)
    $operationId = [Guid]::NewGuid().ToString('N')
    $tempPath = "$resolvedPath.migrating-$operationId"
    $lastWriteTimeUtc = [System.IO.File]::GetLastWriteTimeUtc($resolvedPath)
    $creationTimeUtc = [System.IO.File]::GetCreationTimeUtc($resolvedPath)
    $attributes = [System.IO.File]::GetAttributes($resolvedPath)

    $source = $null
    $destination = $null
    $headerBuffer = $null

    try {
        $source = [System.IO.File]::Open(
            $resolvedPath,
            [System.IO.FileMode]::Open,
            [System.IO.FileAccess]::Read,
            [System.IO.FileShare]::Read
        )
        $headerBuffer = New-Object System.IO.MemoryStream
        $hasLineFeed = $false

        while (($value = $source.ReadByte()) -ne -1) {
            if ($value -eq 10) {
                $hasLineFeed = $true
                break
            }

            $headerBuffer.WriteByte([byte]$value)
            if ($headerBuffer.Length -gt 16777216) {
                throw "聊天元数据首行超过 16 MiB，拒绝自动处理：$resolvedPath"
            }
        }

        $headerBytes = $headerBuffer.ToArray()
        $usesCrLf = $headerBytes.Length -gt 0 -and $headerBytes[$headerBytes.Length - 1] -eq 13
        if ($usesCrLf) {
            $headerBytes = $headerBytes[0..($headerBytes.Length - 2)]
        }

        $metadata = ($utf8NoBom.GetString($headerBytes)) | ConvertFrom-Json
        if (
            $metadata.type -ne 'session_meta' -or
            $null -eq $metadata.payload -or
            $metadata.payload.model_provider -ne $sourceProvider
        ) {
            throw "文件在迁移前发生变化，已停止处理：$resolvedPath"
        }

        $metadata.payload.model_provider = $targetProvider
        $newHeader = $metadata | ConvertTo-Json -Depth 100 -Compress
        $newHeaderBytes = $utf8NoBom.GetBytes($newHeader)

        $destination = [System.IO.File]::Open(
            $tempPath,
            [System.IO.FileMode]::CreateNew,
            [System.IO.FileAccess]::Write,
            [System.IO.FileShare]::None
        )
        $destination.Write($newHeaderBytes, 0, $newHeaderBytes.Length)

        if ($hasLineFeed) {
            if ($usesCrLf) {
                $destination.WriteByte(13)
            }
            $destination.WriteByte(10)
        }

        # 原文件首行之后的内容按字节复制，避免改写聊天正文或换行格式。
        $source.CopyTo($destination)
        $destination.Flush()
        $destination.Dispose()
        $destination = $null
        $source.Dispose()
        $source = $null

        [System.IO.File]::Copy($tempPath, $resolvedPath, $true)
        [System.IO.File]::SetCreationTimeUtc($resolvedPath, $creationTimeUtc)
        [System.IO.File]::SetLastWriteTimeUtc($resolvedPath, $lastWriteTimeUtc)
        [System.IO.File]::SetAttributes($resolvedPath, $attributes)
    }
    finally {
        if ($null -ne $headerBuffer) {
            $headerBuffer.Dispose()
        }
        if ($null -ne $destination) {
            $destination.Dispose()
        }
        if ($null -ne $source) {
            $source.Dispose()
        }
        if (Test-Path -LiteralPath $tempPath) {
            Remove-Item -LiteralPath $tempPath -Force
        }
    }
}

function Get-TopLevelProviderLine {
    param(
        [Parameter(Mandatory)]
        [AllowEmptyCollection()]
        [AllowEmptyString()]
        [string[]]$Lines
    )

    for ($index = 0; $index -lt $Lines.Count; $index++) {
        if ($Lines[$index] -match '^\s*\[') {
            break
        }
        if ($Lines[$index] -match '^\s*model_provider\s*=') {
            return $index
        }
    }

    return -1
}

function Update-TopLevelProvider {
    param([Parameter(Mandatory)][string]$ConfigPath)

    $lines = [System.IO.File]::ReadAllLines($ConfigPath, $utf8NoBom)
    $providerLine = Get-TopLevelProviderLine -Lines $lines
    if ($providerLine -lt 0) {
        return $false
    }

    if ($lines[$providerLine] -notmatch '^\s*model_provider\s*=\s*["'']custom["'']\s*(?:#.*)?$') {
        return $false
    }

    $lines[$providerLine] = 'model_provider = "openai"'
    $operationId = [Guid]::NewGuid().ToString('N')
    $tempConfigPath = "$ConfigPath.migrating-$operationId"

    try {
        [System.IO.File]::WriteAllLines($tempConfigPath, $lines, $utf8NoBom)
        [System.IO.File]::Copy($tempConfigPath, $ConfigPath, $true)
    }
    finally {
        if (Test-Path -LiteralPath $tempConfigPath) {
            Remove-Item -LiteralPath $tempConfigPath -Force
        }
    }

    return $true
}

$resolvedCodexHome = [System.IO.Path]::GetFullPath($CodexHome)
$sessionsPath = Join-Path $resolvedCodexHome 'sessions'
$configPath = Join-Path $resolvedCodexHome 'config.toml'
$sessionIndexPath = Join-Path $resolvedCodexHome 'session_index.jsonl'
$stateDbPath = Join-Path $resolvedCodexHome 'state_5.sqlite'

if (-not (Test-Path -LiteralPath $sessionsPath -PathType Container)) {
    throw "找不到聊天目录：$sessionsPath"
}

if (-not (Test-Path -LiteralPath $configPath -PathType Leaf)) {
    throw "找不到配置文件：$configPath"
}

if (-not (Test-Path -LiteralPath $stateDbPath -PathType Leaf)) {
    throw "找不到任务数据库：$stateDbPath"
}

$sqliteExecutable = Resolve-SqliteExecutable -RequestedPath $SqlitePath

if (-not $DryRun -and -not $SkipProcessCheck) {
    $blockingProcesses = @(
        Get-Process -ErrorAction SilentlyContinue |
            Where-Object {
                $_.ProcessName -in @(
                    'ChatGPT',
                    'Codex',
                    'codex-code-mode-host',
                    'codex-computer-use'
                )
            }
    )
    if ($blockingProcesses.Count -gt 0) {
        $processNames = ($blockingProcesses | Select-Object -ExpandProperty ProcessName -Unique) -join ', '
        throw "检测到 Codex/ChatGPT 仍在运行（$processNames）。请从托盘完全退出应用后再执行修复。"
    }
}

Write-Info "正在扫描：$sessionsPath"
$sessionFiles = @(
    Get-ChildItem -LiteralPath $sessionsPath -Recurse -Filter '*.jsonl' -File
)
$candidateFiles = @(
    $sessionFiles | Where-Object { Test-CustomSession -Path $_.FullName }
)

$configLines = [System.IO.File]::ReadAllLines($configPath, $utf8NoBom)
$topLevelProviderLine = Get-TopLevelProviderLine -Lines $configLines
$configNeedsUpdate = (
    $topLevelProviderLine -ge 0 -and
    $configLines[$topLevelProviderLine] -match '^\s*model_provider\s*=\s*["'']custom["'']\s*(?:#.*)?$'
)
$databaseCandidateCount = Get-CustomThreadCount -Executable $sqliteExecutable -DatabasePath $stateDbPath

Write-Host "扫描到聊天文件：$($sessionFiles.Count)"
Write-Host "需要迁移的旧聊天：$($candidateFiles.Count)"
Write-Host "数据库中需要迁移的旧任务：$databaseCandidateCount"
Write-Host "config.toml 需要切换到 openai：$configNeedsUpdate"

if ($DryRun) {
    Write-Success '预演完成，没有修改任何文件。'
    exit 0
}

if ($candidateFiles.Count -eq 0 -and $databaseCandidateCount -eq 0 -and -not $configNeedsUpdate) {
    Write-Success '没有发现需要迁移的旧聊天或配置。'
    exit 0
}

if (-not $AutoApprove) {
    $answer = Read-Host '脚本将先完整备份，再执行迁移。请输入 MIGRATE 继续'
    if ($answer -cne 'MIGRATE') {
        Write-Host '已取消，没有修改任何文件。' -ForegroundColor Yellow
        exit 1
    }
}

$timestamp = Get-Date -Format 'yyyyMMdd-HHmmss'
$backupRoot = Join-Path $resolvedCodexHome "backups\provider-migration-$timestamp"
$backupSessionsPath = Join-Path $backupRoot 'sessions'

Write-Info "正在创建备份：$backupRoot"
New-Item -ItemType Directory -Path $backupRoot -Force | Out-Null
Copy-Item -LiteralPath $sessionsPath -Destination $backupSessionsPath -Recurse -Force
Copy-Item -LiteralPath $configPath -Destination (Join-Path $backupRoot 'config.toml') -Force
if (Test-Path -LiteralPath $sessionIndexPath -PathType Leaf) {
    Copy-Item -LiteralPath $sessionIndexPath -Destination (Join-Path $backupRoot 'session_index.jsonl') -Force
}
Get-ChildItem -LiteralPath $resolvedCodexHome -Filter 'state_5.sqlite*' -File |
    Copy-Item -Destination $backupRoot -Force

$relativeFiles = @(
    $candidateFiles | ForEach-Object {
        $_.FullName.Substring($sessionsPath.Length).TrimStart('\', '/')
    }
)
[System.IO.File]::WriteAllLines(
    (Join-Path $backupRoot 'migrated-files.txt'),
    $relativeFiles,
    $utf8NoBom
)

$migratedCount = 0
$failedFiles = New-Object System.Collections.Generic.List[string]

foreach ($file in $candidateFiles) {
    try {
        Update-SessionProvider -Path $file.FullName
        $migratedCount++
    }
    catch {
        $failedFiles.Add($file.FullName)
        Write-Warning "迁移失败：$($file.FullName)`n$($_.Exception.Message)"
    }
}

if ($failedFiles.Count -gt 0) {
    [System.IO.File]::WriteAllLines(
        (Join-Path $backupRoot 'failed-files.txt'),
        $failedFiles,
        $utf8NoBom
    )
    throw "有 $($failedFiles.Count) 个聊天迁移失败。请不要启动 Codex，先查看 failed-files.txt 或从备份恢复。"
}

$configUpdated = $false
if ($configNeedsUpdate) {
    $configUpdated = Update-TopLevelProvider -ConfigPath $configPath
}

$databaseMigratedCount = 0
if ($databaseCandidateCount -gt 0) {
    Write-Info "正在迁移任务数据库中的 $databaseCandidateCount 条记录"
    Invoke-Sqlite -Executable $sqliteExecutable -DatabasePath $stateDbPath -Query @"
PRAGMA busy_timeout = 5000;
BEGIN IMMEDIATE;
UPDATE threads SET model_provider = 'openai' WHERE model_provider = 'custom';
COMMIT;
"@ | Out-Null

    $remainingDatabaseCount = Get-CustomThreadCount -Executable $sqliteExecutable -DatabasePath $stateDbPath
    $integrityResult = Invoke-Sqlite -Executable $sqliteExecutable -DatabasePath $stateDbPath -ReadOnly -Query 'PRAGMA integrity_check;'
    if ($remainingDatabaseCount -ne 0) {
        throw "数据库迁移后仍有 $remainingDatabaseCount 条 custom 任务。请不要启动 Codex，并从备份恢复。"
    }
    if (($integrityResult -join '').Trim() -ne 'ok') {
        throw "数据库完整性检查失败：$($integrityResult -join ', ')。请不要启动 Codex，并从备份恢复。"
    }
    $databaseMigratedCount = $databaseCandidateCount
}

Write-Host ''
Write-Host "成功迁移聊天：$migratedCount / $($candidateFiles.Count)"
Write-Host "成功迁移数据库任务：$databaseMigratedCount / $databaseCandidateCount"
Write-Host "config.toml 已切换到 openai：$configUpdated"
Write-Host "备份位置：$backupRoot"

Write-Success '迁移完成。现在可以重新打开 Codex，并抽查几个旧聊天。'
