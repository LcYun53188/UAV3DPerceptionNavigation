# 脚本检查与本地回归

[脚本总目录](../README.md)

`test_entrypoints.py`、`test_programs.py`、`test_build_input_moves.py` 随仓库维护，检查两个程序、输入默认值、会话绑定、地图命令与构建路径迁移；也覆盖跨工作目录路径解析及源码摘要。`conftest.py` 配置分类工具的导入路径，不启动设备或仿真。

`local/` 收纳原 `scripts/test_*.py` 的独立回归脚本，由 Git 忽略；迁移保留文件内容和测试能力，新克隆中不保证存在。包内测试仍位于 `src/<package>/test/`。

```bash
# 不启动 ROS 图、Gazebo 或飞控的入口检查
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 SKIP_WS_SETUP=true \
  ./scripts/with_venv.sh python -m pytest -q scripts/tests/test_*.py

# 本地独立回归；按具体测试加载对应 ROS／PX4 环境
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 \
  ./scripts/with_venv.sh python -m pytest -q scripts/tests/local
```

完整独立回归涉及 ROS 消息、OpenCV、原生编译及可选本地证据；使用相应环境和依赖。协议测试、Gazebo、SITL 和硬件验收分别解释。
