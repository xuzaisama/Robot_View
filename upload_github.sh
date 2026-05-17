#!/bin/bash
set -e

cd "$(dirname "$0")"

if git remote get-url origin >/dev/null 2>&1; then
    current_remote="$(git remote get-url origin)"
    if [ "$current_remote" != "git@github.com:xuzaisama/Robot_View.git" ]; then
        git remote set-url origin git@github.com:xuzaisama/Robot_View.git
    fi
else
    git remote add origin git@github.com:xuzaisama/Robot_View.git
fi

current_branch="$(git branch --show-current)"
if [ "$current_branch" != "main" ]; then
    git branch -M main
fi

git add .

msg="${1:-更新项目文件}"
if git diff --cached --quiet; then
    echo "没有新的改动需要提交"
else
    git commit -m "$msg"
fi

git push -u origin main
