# 生成 compile_commands.json 的辅助脚本
# 使用方法: .\generate_compile_commands.ps1

Write-Host "正在配置 CMake 并生成 compile_commands.json..." -ForegroundColor Green

# 检查是否需要清理并重新配置（如果当前使用的是 Visual Studio 生成器）
$needsRegen = $false
if (Test-Path "build\CMakeCache.txt") {
    $generator = Select-String -Path "build\CMakeCache.txt" -Pattern "CMAKE_GENERATOR:INTERNAL=(.+)" | ForEach-Object { $_.Matches.Groups[1].Value }
    if ($generator -like "*Visual Studio*") {
        Write-Host "检测到 Visual Studio 生成器，该生成器不支持自动生成 compile_commands.json" -ForegroundColor Yellow
        Write-Host "将使用 Ninja 生成器重新配置..." -ForegroundColor Yellow
        $needsRegen = $true
    }
}

# 如果使用 Visual Studio 生成器，询问是否清理重建
if ($needsRegen) {
    $ninjaAvailable = Get-Command ninja -ErrorAction SilentlyContinue
    if (-not $ninjaAvailable) {
        Write-Host "错误: 未找到 Ninja 生成器" -ForegroundColor Red
        Write-Host "解决方案:" -ForegroundColor Yellow
        Write-Host "  1. 安装 Ninja: choco install ninja 或下载 https://github.com/ninja-build/ninja/releases" -ForegroundColor Yellow
        Write-Host "  2. 或者手动配置 .clangd 文件中的包含路径（已完成）" -ForegroundColor Yellow
        exit 1
    }
    Remove-Item -Path "build" -Recurse -Force -ErrorAction SilentlyContinue
    Write-Host "已清理 build 目录" -ForegroundColor Yellow
}

# 确保 build 目录存在
if (-not (Test-Path "build")) {
    New-Item -ItemType Directory -Path "build" | Out-Null
    Write-Host "已创建 build 目录" -ForegroundColor Yellow
}

# 进入 build 目录并运行 CMake
Push-Location build
try {
    # 尝试使用 Ninja 生成器（如果可用），否则使用默认生成器
    $ninjaAvailable = Get-Command ninja -ErrorAction SilentlyContinue

    if ($ninjaAvailable) {
        Write-Host "使用 Ninja 生成器..." -ForegroundColor Yellow
        cmake .. -G Ninja -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
    } else {
        Write-Host "使用默认生成器，尝试生成 compile_commands.json..." -ForegroundColor Yellow
        cmake .. -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
    }

    if ($LASTEXITCODE -eq 0) {
        Write-Host "CMake 配置成功!" -ForegroundColor Green

        # 检查 compile_commands.json 是否存在
        if (Test-Path "compile_commands.json") {
            Copy-Item -Path "compile_commands.json" -Destination "..\compile_commands.json" -Force
            Write-Host "compile_commands.json 已复制到项目根目录" -ForegroundColor Green
        } else {
            Write-Host "警告: compile_commands.json 未在配置阶段生成" -ForegroundColor Yellow
            Write-Host "尝试运行一次构建来生成 compile_commands.json..." -ForegroundColor Yellow

            # 尝试构建来触发生成
            if ($ninjaAvailable) {
                ninja -j 1 2>&1 | Out-Null
            } else {
                cmake --build . --config Debug -j 1 2>&1 | Out-Null
            }

            # 再次检查
            if (Test-Path "compile_commands.json") {
                Copy-Item -Path "compile_commands.json" -Destination "..\compile_commands.json" -Force
                Write-Host "compile_commands.json 已生成并复制到项目根目录" -ForegroundColor Green
            } else {
                Write-Host "错误: 无法生成 compile_commands.json" -ForegroundColor Red
                Write-Host "提示: Visual Studio 生成器可能不支持 compile_commands.json" -ForegroundColor Yellow
                Write-Host "建议: 安装 Ninja 生成器或使用其他方式配置 clangd" -ForegroundColor Yellow
            }
        }
    } else {
        Write-Host "CMake 配置失败，请检查错误信息" -ForegroundColor Red
        exit 1
    }
} finally {
    Pop-Location
}

Write-Host "完成!" -ForegroundColor Green
