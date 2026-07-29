#!/bin/bash
cd ~/ros2_ws

# 변경된 파일이 있는지 확인
if [[ $(git status --porcelain) ]]; then
    git add .
    git commit -m "Auto-sync: $(date +"%Y-%m-%d %H:%M")"
    git push origin main
fi

