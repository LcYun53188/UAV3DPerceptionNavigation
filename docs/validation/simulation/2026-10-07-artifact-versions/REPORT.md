# QGC 与 DM-FC01 固件文件版本核验

日期：2026-10-07。按用户提供的本机安装路径及固件链接进行只读核验，没有启动 QGC GUI、连接飞控或刷写固件。

## QGroundControl

- 本机路径：`/home/nuc/Program/QGC/QGroundControl-x86_64.AppImage`
- 应用版本：**v5.1.5**。来自 AppImage 内 `usr/share/metainfo/org.mavlink.qgroundcontrol.appdata.xml` 的 release，以及内嵌 `usr/bin/QGroundControl` 的 `v5.1.5` 字符串。见 [内嵌 AppStream 元数据](qgc-appdata.xml)。
- 文件 SHA-256：`9a47e4cf269d9e4f897f582f6b19aa7ebf1e9e67c9d68a532a45b541354bc344`
- AppImage runtime 的版本是运行器身份，与 QGC 应用版本分别记录；本次应用版本没有用 `--appimage-version` 推断。

QGC 安装项已从版本锁 pending 移除，其绝对路径和 SHA-256 加入离线 doctor 的文件检查。替换该文件时需更新版本记录。

## 厂商固件

[用户提供的 Gitee 文件页](https://gitee.com/kit-miao/dm-fc01/blob/master/%E5%9B%BA%E4%BB%B6/PX4/damiao_dm-fc01_V1.16.px4)；本次从对应 `/raw/master/` 地址下载并解析 PX4FWv1 JSON，再解压其中 image。master 可变化，以本次 SHA-256 作为文件身份。

| 字段 | 文件中读取的值 |
| --- | --- |
| 文件名 | damiao_dm-fc01_V1.16.px4 |
| 文件 SHA-256 | `2b5cc6ec1012a097bdc6779a6bc23afd05c5d8b53d6cc1561f09d8cb0528bba0` |
| git_identity | `v1.16.0-7-g78a512995e` |
| git_hash | `78a512995e73dad88051707b5bee3df07eed4d78` |
| board_id / board_revision | 7140 / 1 |
| description | Firmware for the DAMIAO DM-FC01 flight controller |
| 包字段 version | `1.0.0`，原样保留，不将它解释为 PX4 发布版本 |
| image_size | 1785596 bytes，已与实际解压大小核对 |
| 解压 image SHA-256 | `071419d812aab50b6c301c206a974833f8f45c68ff7ab8cdd82e5c8fa6b97259` |

完整元数据、大小和来源见 [artifacts.json](artifacts.json) 与 `simulation/px4/versions.lock.yaml`。下载包与解压 image 留在 `.cache/simulation/firmware/`，不将二进制加入 Git；doctor 不要求这个临时缓存永久存在。

这些字段说明所下载固件文件的构建身份。尚未取得并核对对应厂商源码树，也没有新鲜飞控遥测证明当前运行的是同一构建。该文件面向板卡，不是电脑上的 SITL 可执行程序；主机仿真仍需单独冻结 PX4 1.16 SITL tag/commit、Agent 与 px4_msgs 对应关系。

## 本次验证与下一步

离线环境审计见 [environment.json](environment.json)，包含新加入的 QGC 文件 hash 检查。此次修改为版本数据与计划说明更新，没有改动导航、控制或任务执行逻辑。

下一步固定主机 SITL 源码与匹配的 Agent/px4_msgs；厂商固件构建元数据作为后续硬件对照项。继续使用本机仿真主线，Jetson 维持延期。
