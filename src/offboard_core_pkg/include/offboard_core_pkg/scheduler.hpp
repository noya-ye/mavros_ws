#pragma once

#include <memory>
#include <string>
#include <vector>
#include <limits>
#include <iostream>

#include "offboard_core_pkg/itask.hpp"

namespace offboard_core_pkg {
class Scheduler {
public:
  enum class InterruptMode { RESUME_CURRENT, DROP_CURRENT };

  void clear() {
    tasks_.clear();
    paused_stack_.clear();
    idx_ = 0;
    entered_ = false;
    done_ = true;
    failed_ = false;
  }

  void add(std::unique_ptr<ITask> task) {
    tasks_.push_back(TaskSlot{std::move(task), true});
  }
  void addAux(std::unique_ptr<ITask> task) {
    tasks_.push_back(TaskSlot{std::move(task), false});
  }

  void reset() {
    idx_ = firstSequenceIndex();
    entered_ = false;
    done_ = tasks_.empty() || idx_ == npos();
    failed_ = false;
    paused_stack_.clear();
  }

  bool done() const { return done_; }
  bool failed() const { return failed_; }
  bool hasPausedTask() const { return !paused_stack_.empty(); }

  std::size_t total_count() const {
    std::size_t count = 0;
    for (const auto &slot : tasks_) if (slot.in_sequence) ++count;
    return count;
  }
  std::size_t current_index() const {
    const auto total = total_count();
    if (total == 0 || done_) return total == 0 ? 0 : total - 1;
    std::size_t sequence_index = 0;
    for (std::size_t i = 0; i < tasks_.size(); ++i) {
      if (!tasks_[i].in_sequence) continue;
      if (i == idx_) return sequence_index;
      ++sequence_index;
    }
    return total - 1;
  }
  std::string current_name() const {
    return (done_ || idx_ >= tasks_.size()) ? "DONE" : tasks_[idx_].task->name();
  }
  std::string currentName() const { return current_name(); }

  bool interrupt(const std::string &task_name, Context &ctx, MavrosIface &iface,
                 InterruptMode mode = InterruptMode::RESUME_CURRENT) {
    const auto target = findTask(task_name);
    if (target == npos()) return false;
    if (!done_ && idx_ < tasks_.size()) {
      auto *current = tasks_[idx_].task.get();
      if (current->name() == task_name) return true;
      if (entered_) {
        if (!current->canPause(ctx)) return false;
        if (mode == InterruptMode::RESUME_CURRENT) {
          current->onPause(ctx, iface);
          paused_stack_.push_back(Frame{idx_, true});
        } else {
          current->onCancel(ctx, iface);
          logTaskEnd(*current, CancelledStatus{});
        }
      } else if (mode == InterruptMode::RESUME_CURRENT) {
        paused_stack_.push_back(Frame{idx_, false});
      }
    }
    idx_ = target;
    entered_ = false;
    done_ = false;
    failed_ = false;
    return true;
  }

  bool jumpTo(const std::string &task_name, Context &ctx, MavrosIface &iface) {
    const auto target = findTask(task_name);
    if (target == npos()) return false;
    cancelCurrentAndPaused(ctx, iface);
    idx_ = target;
    entered_ = false;
    done_ = false;
    failed_ = false;
    return true;
  }

  void tick(Context &ctx, MavrosIface &iface, double dt_s) {
    if (done_ || failed_) return;
    if (idx_ >= tasks_.size()) { done_ = true; return; }
    auto &task = *tasks_[idx_].task;
    if (!entered_) {
      logTaskStart(task);
      task.onEnter(ctx, iface);
      entered_ = true;
    }
    const auto status = task.tick(ctx, iface, dt_s);
    if (status == ITask::Status::RUNNING) return;
    task.onExit(ctx, iface);
    logTaskEnd(task, status);
    entered_ = false;
    if (status == ITask::Status::FAILURE) { failed_ = true; done_ = true; return; }
    if (!paused_stack_.empty()) {
      const auto frame = paused_stack_.back();
      paused_stack_.pop_back();
      idx_ = frame.idx;
      if (idx_ >= tasks_.size()) { done_ = true; return; }
      if (frame.was_entered) { tasks_[idx_].task->onResume(ctx, iface); entered_ = true; }
      return;
    }
    advanceToNextSequence();
  }

private:
  struct TaskSlot { std::unique_ptr<ITask> task; bool in_sequence{true}; };
  struct Frame { std::size_t idx; bool was_entered; };
  static constexpr std::size_t npos() { return std::numeric_limits<std::size_t>::max(); }
  std::size_t findTask(const std::string &name) const {
    for (std::size_t i = 0; i < tasks_.size(); ++i)
      if (tasks_[i].task && tasks_[i].task->name() == name) return i;
    return npos();
  }
  std::size_t firstSequenceIndex() const {
    for (std::size_t i = 0; i < tasks_.size(); ++i) if (tasks_[i].in_sequence) return i;
    return npos();
  }
  void advanceToNextSequence() {
    auto next = idx_ + 1;
    while (next < tasks_.size() && !tasks_[next].in_sequence) ++next;
    if (next >= tasks_.size()) { done_ = true; return; }
    idx_ = next;
    entered_ = false;
  }
  void cancelCurrentAndPaused(Context &ctx, MavrosIface &iface) {
    if (!done_ && idx_ < tasks_.size() && entered_) {
      tasks_[idx_].task->onCancel(ctx, iface);
      logTaskEnd(*tasks_[idx_].task, CancelledStatus{});
    }
    while (!paused_stack_.empty()) {
      const auto frame = paused_stack_.back();
      paused_stack_.pop_back();
      if (frame.idx < tasks_.size() && frame.was_entered) {
        tasks_[frame.idx].task->onCancel(ctx, iface);
        logTaskEnd(*tasks_[frame.idx].task, CancelledStatus{});
      }
    }
    entered_ = false;
  }

  struct CancelledStatus {};

  void logTaskStart(const ITask &task) const {
    const auto total = total_count();
    const auto position = current_index() + 1;
    std::cout << "[TASK START] " << task.name() << " ("
              << position << "/" << total << ")" << std::endl;
  }

  void logTaskEnd(const ITask &task, ITask::Status status) const {
    const char *label = status == ITask::Status::SUCCESS ? "SUCCESS" : "FAILURE";
    std::cout << "[TASK END] " << task.name() << " - " << label << std::endl;
  }

  void logTaskEnd(const ITask &task, CancelledStatus) const {
    std::cout << "[TASK END] " << task.name() << " - CANCELLED" << std::endl;
  }

  std::vector<TaskSlot> tasks_;
  std::vector<Frame> paused_stack_;
  std::size_t idx_{0};
  bool entered_{false};
  bool done_{false};
  bool failed_{false};
};
}  // namespace offboard_core_pkg
