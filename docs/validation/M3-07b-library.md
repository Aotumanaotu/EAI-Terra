# M3-07b SQLite 实验记录库

> 状态：**已实现，待基线验证**（2026-11-24）。依据：[M3-07 设计稿](M3-07-record-design.md) 评审问题 5（用户：实验记录要进数据库）；D-112（已确定）。

## 1. 交付

**SQLite 3.54.1（`third_party/sqlite/`）：**
- **来源与校验：**合并版源码 `sqlite3.c` / `sqlite3.h`，取自官方 zip，未做修改。zip 的 SHA3-256 与官方下载页公布的值一致；`sqlite.lock` 另记 zip 与各文件的 SHA-256。
- **许可：**公有领域。
- **复核：**`scripts/sqlite/check_sqlite.sh`，加 `--online` 时另从官方下载比对。
- **构建：**`cmake/TerraSqlite.cmake` 编译为静态库 `terra::sqlite`（C 语言，构建不联网）。选项：
  - `SQLITE_DQS=0`；
  - 默认启用外键；
  - 不加载扩展，因此不需要 libdl；
  - 线程安全模式，需要 pthread。
- **使用范围：**`sqlite3.h` 只允许在 `engine/library/src/` 与测试中出现，由 `boundary.render_isolation` 检查。

**`terra_library`（新模块，`terra/library/library.hpp`）：**
- **原则：**文件是原件，数据库只是可重建的索引。库中存每份记录的摘要，用于列出、筛选与查找；删掉库文件不丢任何东西，重新扫描即可恢复。
  - 摘要字段：路径、文件 SHA-256 与大小、创建时间、场景 / 动作文件及其摘要、起始关键帧、程序与物理后端版本、平台、步数、仿真时间、结束原因（没有摘要时为 `incomplete`）、加入索引的时间。
- **库文件位置：**默认依次取 `$TERRA_LIBRARY`、`$XDG_DATA_HOME/eai-terra/library.sqlite3`、`~/.local/share/eai-terra/library.sqlite3`。默认记录目录为库文件旁的 `records/`。
- **库的版本：**记在 SQLite 的 `user_version` 中，当前为 1；高于本程序支持的版本时拒绝打开。
- **库文件损坏：**改名为 `<文件>.corrupt-<时间>` 并新建，给出警告。
- **接口：**

  | 接口 | 作用 |
  | --- | --- |
  | `addRecord` | 加入记录；同一路径为更新 |
  | `records(scene)` | 按创建时间从新到旧列出，可按场景筛选 |
  | `record(path)` | 按路径查找 |
  | `removeMissing` | 删除文件已不存在的条目 |
  | `rebuild(目录)` | 递归扫描目录；读不出的文件计入错误，不中断扫描 |
- **读取方式：**加入索引时只读记录的第一行与最后一行（新增 `readRecordInfo`），不解析中间的状态，大文件也很快。

**Headless：**
- `--record` 完成后自动加入资源库：库文件用 `--library` 指定，没有指定时用默认位置。加入失败只警告，记录文件本身不受影响。
- `--list-records [--scene 场景] [--library 库]`：每行一个 `{"type":"record",…}`，按创建时间从新到旧。
- 测试与验证脚本都显式指定构建目录中的库文件，不写入用户目录。

## 2. 测试

| 测试 | 内容 |
| --- | --- |
| `library.store`（新增） | 新建库；三份记录的加入与摘要字段；不存在的记录返回错误；按创建时间排序；按场景筛选；按路径查找；同一路径再次加入为更新；没有 summary 的记录标为 `incomplete`；删除文件后清理；按目录重建（含子目录，读不出的文件计入错误）；删掉库文件后重新扫描即可恢复；版本号 = 1，版本过高时拒绝打开；损坏时备份并新建；默认路径（环境变量的优先顺序） |
| `headless.list_records`（新增） | 接在 `headless.record_motion` 之后：按场景筛选，列出刚才的记录（1250 步，结束原因 completed） |
| `headless.cli_parser`（扩充） | `--library`、`--list-records` 的解析与约束 |

**测试总数：**渲染关闭 71 → **73**，渲染开启 99 → **101**。

**变异测试 7 项**（`scripts/verify/m3_07b.sh`，支持 `--mutation-only`）：
- 资源库：列出时不按创建时间排序；不清理文件已不存在的条目；库文件损坏时不备份重建；不检查库的版本；未完成的记录标为 completed；场景筛选失效；
- Headless：不加入索引。

## 3. 验证记录

开发机：未编译、未运行（按约定）。已做的静态检查：
- SQLite zip 的 SHA3-256 与官方公布值一致；`check_sqlite.sh` 通过；
- 变异锚点全部唯一，所有脚本合计 185/185；
- 脚本通过 `bash -n`。

**基线机器：**未执行。
