---
name: offboard-tasks
description: 在 offboard_core_pkg 中创建、修改或审查 Scheduler/ITask/MavrosIface 任务，并同步维护 agent_task.md。适用于 src/offboard_core_pkg/src/tasks 及其对应 include、注册和构建文件。
---

# Offboard Tasks Skill

这是仓库内的 task 开发 skill。处理任何 task 新增或修改时，先在当前工作区定位并读取 `src/offboard_core_pkg/src/tasks/agent_task.md`，再读取实际相关的 `itask.hpp`、`context.hpp`、`mavros_iface.hpp`、`scheduler.hpp` 和现有 task。以仓库当前代码为准，不凭空假设接口。

## 必须遵守

- Task 必须实现 `ITask` 生命周期：用 `onEnter` 初始化，用 `tick` 返回 `RUNNING/SUCCESS/FAILURE`，按需实现退出、暂停、恢复和取消。
- 共享状态只经 `Context`，MAVROS 服务只经 `MavrosIface`；服务请求按异步 callback 处理，不阻塞等待。
- 位置和 yaw setpoint 写入 `Context`，坐标采用 ENU；检查连接、位置有效性、有限数值和参数边界。
- 超时、非法输入或不可行路径要设置 `ctx.fault` 并返回 `FAILURE`；生命周期重入时重置内部状态。
- 同步更新声明头文件、实现文件、`tasks.hpp`、`CMakeLists.txt` 和 scheduler/节点注册；必要时补充测试。

## 文档同步规则

每次写入一个新的 task，必须在 `agent_task.md` 的“现有 task 接口登记”中增加：构造函数、稳定名称、生命周期覆盖、主要功能、副作用、成功/失败条件和公开查询接口。若已有 task 的接口或功能改变，必须修改其登记及“维护检查清单”相关说明。不要只更新代码而留下过时文档。

## 工作方式

1. 以 `agent_task.md` 为开发和审查清单，并核对实现与头文件是否一致。
2. 保持与现有命名、状态机、计时、setpoint 发布和 scheduler 中断语义一致；发现文档与代码冲突时，以代码为事实来源并修正文档。
3. 完成后至少执行针对性的构建/测试或静态检查，并报告未能验证的环境条件。
