# dex-retargeting (vendored)

Third-party dependency, kept at the repo top level next to `ManusSDK_v3.1.1/` so the
teleop service's optimizer is an explicit, offline-reproducible part of the workspace.

- Upstream: https://github.com/dexsuite/dex-retargeting  (pinned to PyPI 0.5.0)
- This folder is the installable package source (`src/dex_retargeting/`), obtained
  from the official `dex_retargeting-0.5.0` wheel because this machine's network
  blocks `git clone`/raw GitHub. It is byte-identical to the released package.
- The runtime uses THIS copy via an editable install:

      conda run -n teleop pip install -e ./dex-retargeting --no-deps

  (`--no-deps` because its `pin` (pinocchio) dependency has no Windows wheel and is
  provided by conda-forge instead — see ../src/teleop/README.md.)

To update later (on a network that allows it), replace `src/dex_retargeting/` with a
fresh `git clone` of the upstream repo's `src/dex_retargeting/`, keeping this layout.
