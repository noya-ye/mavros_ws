#pragma once

#include <string>

namespace offboard_core_pkg {
struct Context;
class MavrosIface;

class ITask {
public:
  enum class Status {
    RUNNING,
    SUCCESS,
    FAILURE,
  };

  virtual ~ITask() = default;
  virtual std::string name() const = 0;

  virtual void onEnter(Context &, MavrosIface &) {}
  virtual Status tick(Context &, MavrosIface &, double dt_s) = 0;
  virtual void onExit(Context &, MavrosIface &) {}

  virtual bool canPause(const Context &) const { return true; }
  virtual void onPause(Context &, MavrosIface &) {}
  virtual void onResume(Context &, MavrosIface &) {}
  virtual void onCancel(Context &, MavrosIface &) {}
};
}  // namespace offboard_core_pkg
