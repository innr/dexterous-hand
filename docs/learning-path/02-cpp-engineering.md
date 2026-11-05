# S2 — C++17 工程化与质量控制

**入口**：S1 PASSED。**目标**：能够从零创建跨平台可编译、可审查、可回归的机器人软件库。

## 必会知识
CMake targets / include / visibility、静态库和可执行程序、头文件与实现分离、编译警告、`std::optional` 与错误码/异常边界、单元测试夹具、Mock、Git 分支和 PR 审查、Debug/Release、基本性能测量。

## 项目任务
将 S1 的 JointModel 改为独立 `leap_core` 静态库，`leap_demo` 可执行程序；公共头文件置于 `include/leap/`，实现置于 `src/`。引入 CTest 或测试框架，写出跨 Windows/Linux 的构建说明；添加 GitHub Actions，至少编译和测试一种主机环境，条件允许覆盖两种。提供接口错误约定、API 示例与变更记录。

## 验收测试
- S2-T01：从全新 checkout、空构建目录按 README 一次构建成功；不依赖本机绝对路径。
- S2-T02：Debug/Release 构建及全部 CTest 通过，警告策略明示。
- S2-T03：至少 15 个具有明确断言的边界/错误/正常路径测试，故意改坏一个关键判断后测试能失败。
- S2-T04：ASan/UBSan 在支持的平台无错误；不支持时记明限制。
- S2-T05：CI 对失败测试返回非零且阻止通过；保存运行 URL 与 commit SHA。
- S2-T06：执行一次独立评审：从公开接口解释 ownership、异常安全和 ABI/接口兼容的基本取舍。
- S2-T07：本人不参考现成代码完成一个新增 API + 单测 + CI 通过的 PR。

## 通过标准
全部 T01–T07 通过，提供 clean-build 日志、CI 结果、测试清单与评审记录。S2 验收只证明工程能力，不证明线程安全或实时性。
