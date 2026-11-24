# SQLite 3.54.1（合并版源码）

- 来源：https://www.sqlite.org/2026/sqlite-amalgamation-3540100.zip（官方下载页公布的 SHA3-256 与 `sqlite.lock` 一致）
- 文件：`src/sqlite3.c`、`src/sqlite3.h`，取自上述 zip，未做修改；复核：`bash scripts/sqlite/check_sqlite.sh`（加 `--online` 另从官方下载比对）
- 许可：**公有领域**。SQLite 的作者放弃了版权，源码头部以“祝福”代替法律声明（见 `src/sqlite3.c` 开头），可自由使用、修改与分发，无需署名
- 用途：D-112 资源库与实验记录索引（M3-07b 起），编译为静态库 `terra_sqlite`，只在 `engine/library/src/` 中使用
