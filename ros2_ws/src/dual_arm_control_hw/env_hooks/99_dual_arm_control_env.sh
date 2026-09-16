# Third-party Pinocchio/CasADi paths are configured explicitly by
# towel_task/scripts/env.sh through TOWEL_OPENROBOTS_PREFIX. Do not probe a
# sibling openrobots_libs_hw install: that legacy package used Python 3.10 and
# can silently mix incompatible libraries into this Python 3.12 workspace.
