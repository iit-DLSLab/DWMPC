#!/usr/bin/env bash

# ==============================================================================
# Test for lock file
# ==============================================================================
# If the lock file exists, this script has been run in the current copy of the
# repository. Do not run again
project_root=$(git rev-parse --show-toplevel)
if [ -e $project_root/.setup.lock ]; then
	exit 0
fi

# ==============================================================================
# Githooks
# ==============================================================================
echo -- Installing githooks
hooks_root=$(git rev-parse --git-path hooks)

rm -r $hooks_root/*
cp -a $project_root/githooks/. $hooks_root/

# Do not run fast-forward commits on master, but instead create a proper merge
# commit with a commit message
#
# If a fast-forward is performed, then the commit hooks are not run. This will
# prevent auto-increment of version numbers
git config branch.master.mergeoptions "--no-ff"

# ==============================================================================
# Create Lock File
# ==============================================================================
echo "This file tracks whether the setup.sh script has been run" > $project_root/.setup.lock
