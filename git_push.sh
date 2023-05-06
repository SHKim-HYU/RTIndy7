#!/bin/bash
now=$(date +"%Y-%m-%d %H:%M:%S")
now_date=$(date +"%Y-%m-%d %H:%M:%S")

git add *
git commit -m "$now"
git remote add "$now_date" git@github.com:SHKim-HYU/RTIndy7.git
git push "$now_date" smc

