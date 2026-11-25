# xacro

- 来源：PyPI `xacro` 2.1.1（https://pypi.org/project/xacro/2.1.1/ ，上游 https://github.com/ros/xacro ），wheel 解压后的 `xacro/` 目录，未修改。
- 许可：BSD-3-Clause（`LICENSE`）。
- 用途：M4-08a 模型导入展开 `.xacro`。由 `scripts/import/xacro_expand.py` 调用：`$(find 包名)` 改为查导入时给出的包目录表，
  因此**不需要 ROS**；运行只需要 `python3`（标准库；`xacro.load_yaml` 另需 PyYAML）。
- 校验：`bash scripts/import/check_third_party.sh`（按 `xacro.lock` 核对；`--online` 另下载 wheel 比对）。
