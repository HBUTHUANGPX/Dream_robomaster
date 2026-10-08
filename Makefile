.PHONY: configure build test setup doctor navigation duel cube robot stop

# 所有目标使用同一个仓库入口。
# 从其他工作目录使用 make -f /绝对路径/Makefile 时，仍定位到本仓库。
RM_MAKEFILE := $(shell realpath '$(MAKEFILE_LIST)')
RM_ROOT := $(shell dirname '$(RM_MAKEFILE)')

configure:
	@bash "$(RM_ROOT)/rm" doctor
	@mkdir -p "$(RM_ROOT)/.cache/tmp"
	@cd "$(RM_ROOT)" && TMPDIR="$(RM_ROOT)/.cache/tmp" cmake --preset native

build:
	@bash "$(RM_ROOT)/rm" build

test:
	@bash "$(RM_ROOT)/rm" test

setup:
	@bash "$(RM_ROOT)/rm" setup

doctor:
	@bash "$(RM_ROOT)/rm" doctor

stop:
	@bash "$(RM_ROOT)/rm" stop

navigation:
	@bash "$(RM_ROOT)/rm" navigation

duel:
	@bash "$(RM_ROOT)/rm" duel

cube:
	@bash "$(RM_ROOT)/rm" cube

robot:
	@bash "$(RM_ROOT)/rm" robot
