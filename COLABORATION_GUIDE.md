# BpmsmTest 多人协作开发指南

## 仓库架构

```
upstream/BpmsmTest (主仓库)
  ├── main           # 稳定发布分支
  ├── develop        # 集成分支
  ├── feature/dev-1  # 开发者1 工作分支
  ├── feature/dev-2  # 开发者2 工作分支
  ├── feature/dev-3  # 开发者3 工作分支
  └── feature/dev-4  # 开发者4 工作分支
```

## 4位开发者 Fork 仓库

| 开发者 | Fork 仓库 | 工作分支 |
|--------|----------|---------|
| 开发者1 | your-org/dev-1/BpmsmTest | feature/dev-1 |
| 开发者2 | your-org/dev-2/BpmsmTest | feature/dev-2 |
| 开发者3 | your-org/dev-3/BpmsmTest | feature/dev-3 |
| 开发者4 | your-org/dev-4/BpmsmTest | feature/dev-4 |

## 工作流程

### 1. 克隆个人Fork仓库
```bash
git clone https://github.com/YOUR_USERNAME/BpmsmTest.git
cd BpmsmTest
```

### 2. 添加上游主仓库
```bash
git remote add upstream https://github.com/MAIN_OWNER/BpmsmTest.git
```

### 3. 切换到自己的工作分支
```bash
git checkout feature/dev-X    # X = 1, 2, 3, 4
```

### 4. 日常开发
```bash
git add <modified-files>
git commit -m "feat: 功能描述"
```

### 5. 同步上游更新
```bash
git fetch upstream
git checkout develop
git merge upstream/develop
git checkout feature/dev-X
git merge develop
```

### 6. 提交 Pull Request
```bash
git push origin feature/dev-X
# 然后在 GitHub 上创建 PR: feature/dev-X -> upstream/develop
```

## 分支命名规范

- `feature/xxx`  - 新功能开发
- `fix/xxx`      - Bug 修复
- `refactor/xxx` - 代码重构
- `docs/xxx`     - 文档更新

## 提交信息规范

```
<type>: <简要描述>

feat: 添加速度环PI控制
fix: 修复ADC采样溢出问题
refactor: 重构PWM生成模块
docs: 更新电机参数配置说明
```

## 远程仓库创建步骤

### 主仓库管理员:

1. 在 GitHub 创建 BpmsmTest 仓库（作为 upstream）
2. 推送本地代码到 upstream
3. 在 GitHub Settings -> Collaborators 添加4位开发者
4. 每位开发者在 GitHub 上 Fork 主仓库