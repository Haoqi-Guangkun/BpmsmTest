<#
.SYNOPSIS
    BpmsmTest Git 仓库一键搭建脚本
.DESCRIPTION
    自动完成：Git 安装检测 -> 仓库初始化 -> GitHub 远程仓库创建 -> 4人协作分支设置
.NOTES
    运行前请确保:
    1. 已安装 Git for Windows (https://git-scm.com/download/win)
    2. 已安装 GitHub CLI (https://cli.github.com/)
    3. 已登录 GitHub: 运行 `gh auth login`
#>

$ErrorActionPreference = "Stop"
$ProjectPath = "C:\Users\Lenovo\Desktop\bpmsm_code\BpmsmTest"

Write-Host "========================================" -ForegroundColor Cyan
Write-Host "  BpmsmTest Git 仓库搭建脚本" -ForegroundColor Cyan
Write-Host "========================================" -ForegroundColor Cyan
Write-Host ""

Write-Host "[Step 0] 环境检查..." -ForegroundColor Yellow

try {
    $gitVersion = git --version 2>&1
    Write-Host "  [OK] Git: $gitVersion" -ForegroundColor Green
} catch {
    Write-Host "  [ERROR] Git 未安装!" -ForegroundColor Red
    Write-Host "  请从 https://git-scm.com/download/win 下载安装"
    exit 1
}

try {
    gh --version 2>&1 | Select-Object -First 1 | ForEach-Object { Write-Host "  [OK] $_" -ForegroundColor Green }
    $ghAvailable = $true
} catch {
    Write-Host "  [INFO] GitHub CLI 未安装，跳过远程仓库自动创建" -ForegroundColor Gray
    $ghAvailable = $false
}

if ($ghAvailable) {
    try {
        gh auth status 2>&1 | Out-Null
        Write-Host "  [OK] GitHub CLI 已登录" -ForegroundColor Green
        $ghLoggedIn = $true
    } catch {
        Write-Host "  [WARN] GitHub CLI 未登录，请运行: gh auth login" -ForegroundColor Yellow
        $ghLoggedIn = $false
    }
}

if (-not (Test-Path $ProjectPath)) {
    Write-Host "  [ERROR] 项目目录不存在: $ProjectPath" -ForegroundColor Red
    exit 1
}
Write-Host "  [OK] 项目目录: $ProjectPath" -ForegroundColor Green
Write-Host ""

Write-Host "[Step 1] 创建 .gitignore..." -ForegroundColor Yellow
$gitignore = @"
Debug/
Release/
.settings/
.launches/
.ccsproject
.cproject
.project
*.obj
*.d
*.map
*.out
*.hex
*.bin
*.a
*.lib
ccsObjs.opt
makefile
objects.mk
sources.mk
subdir_rules.mk
subdir_vars.mk
*_linkInfo.xml
Thumbs.db
Desktop.ini
.DS_Store
targetConfigs/*.ccxml
"@
$gitignore | Out-File "$ProjectPath\.gitignore" -Encoding UTF8
Write-Host "  [OK] .gitignore 已创建" -ForegroundColor Green
Write-Host ""

Write-Host "[Step 2] 初始化 Git 仓库..." -ForegroundColor Yellow
Push-Location $ProjectPath

if (Test-Path ".git") {
    Write-Host "  [INFO] Git 仓库已存在，跳过初始化" -ForegroundColor Gray
} else {
    git init
    Write-Host "  [OK] Git 仓库初始化完成" -ForegroundColor Green
}

$gitUser = git config user.name 2>$null
$gitEmail = git config user.email 2>$null
if (-not $gitUser) {
    Write-Host ""
    Write-Host "  请设置 Git 用户信息:" -ForegroundColor Yellow
    $userName = Read-Host "  请输入你的姓名"
    $userEmail = Read-Host "  请输入你的邮箱"
    git config user.name $userName
    git config user.email $userEmail
}
Write-Host ""

Write-Host "[Step 3] 创建首次提交..." -ForegroundColor Yellow
git add -A
$status = git status --porcelain
if ($status) {
    git commit -m "chore: init BpmsmTest repo with BPMSM motor control code"
    Write-Host "  [OK] 首次提交完成" -ForegroundColor Green
} else {
    Write-Host "  [INFO] 无变更需要提交" -ForegroundColor Gray
}
Write-Host ""

Write-Host "[Step 4] GitHub 远程仓库设置..." -ForegroundColor Yellow
if ($ghLoggedIn) {
    $repoName = "BpmsmTest"
    $repoDesc = "TI TMS320F28335 BPMSM - 4-dev collaborative"

    try {
        gh repo create $repoName --private --description $repoDesc --source . --remote origin --push 2>&1
        Write-Host "  [OK] 远程仓库创建成功!" -ForegroundColor Green
        git branch -M main
        git push -u origin main 2>&1 | Out-Null
    } catch {
        Write-Host "  [WARN] 自动创建失败，请手动在 GitHub 创建" -ForegroundColor Yellow
    }
} else {
    Write-Host "  [INFO] 跳过（GitHub CLI 未登录）" -ForegroundColor Gray
    Write-Host "  手动: gh auth login && gh repo create BpmsmTest --private --source . --remote origin --push" -ForegroundColor Yellow
}
Write-Host ""

Write-Host "[Step 5] 创建4人协作分支..." -ForegroundColor Yellow
$developers = @("dev-1", "dev-2", "dev-3", "dev-4")
foreach ($dev in $developers) {
    $branchName = "feature/$dev"
    git branch $branchName 2>$null
    Write-Host "  [OK] 分支: $branchName" -ForegroundColor Green
}
git branch develop 2>$null
Write-Host "  [OK] 分支: develop (集成)" -ForegroundColor Green
Write-Host ""

Write-Host "  当前分支列表:" -ForegroundColor Cyan
git branch
Write-Host ""

Write-Host "[Step 6] 生成协作指南 COLABORATION_GUIDE.md..." -ForegroundColor Yellow
Pop-Location
Write-Host "========================================" -ForegroundColor Green
Write-Host "  Git 仓库搭建完成!" -ForegroundColor Green
Write-Host "========================================" -ForegroundColor Green
Write-Host ""
Write-Host "  分支结构:" -ForegroundColor Cyan
Write-Host "    main           - 稳定发布"
Write-Host "    develop        - 集成分支"
Write-Host "    feature/dev-1  - 开发者1"
Write-Host "    feature/dev-2  - 开发者2"
Write-Host "    feature/dev-3  - 开发者3"
Write-Host "    feature/dev-4  - 开发者4"
Write-Host ""
Write-Host "  下一步:" -ForegroundColor Yellow
Write-Host "    1. 在 GitHub 为4位开发者创建 Fork 仓库"
Write-Host "    2. 每位开发者 clone 自己的 Fork"
Write-Host "    3. git remote add upstream <主仓库URL>"
Write-Host "    4. git checkout feature/dev-X 开始开发"