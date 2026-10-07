// Deterministic functional tests; no ROS executor and no claim of closed-loop flight.
#include "ego_2d_planner_pkg/plan_manage/planner_manager_2d.hpp"
#include "ego_2d_planner_pkg/diff_planner/diff_optimizer_2d.hpp"
#include <chrono>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <stdexcept>

using namespace ego_2d_planner_pkg;
using Clock = std::chrono::steady_clock;
int assertions = 0;
void require(bool condition, const std::string& label)
{
  ++assertions;
  if (!condition) throw std::runtime_error(label);
}

namespace ego_2d_planner_pkg
{
struct DiffPlannerTestAccess
{
  static double gradientError(DiffOptimizer2D& optimizer)
  {
    auto x = optimizer.best_x_;
    // Force active feasibility costs as well as obstacle costs, rather than only
    // checking the low-speed smoothness part of the objective.
    for (int i = 0; i < optimizer.pieces_; ++i) x[2 * (optimizer.pieces_ - 1) + i] = 0.0;
    std::vector<double> analytic(x.size()), scratch(x.size());
    optimizer.objective(x.data(), analytic.data());
    double worst = 0.0;
    for (std::size_t i = 0; i < x.size(); ++i) {
      const double original = x[i], h = 1e-5 * std::max(1.0, std::abs(original));
      x[i] = original + h; const double plus = optimizer.objective(x.data(), scratch.data());
      x[i] = original - h; const double minus = optimizer.objective(x.data(), scratch.data());
      x[i] = original;
      const double numeric = (plus - minus) / (2.0 * h);
      worst = std::max(worst, std::abs(numeric - analytic[i]) /
        std::max({1.0, std::abs(numeric), std::abs(analytic[i])}));
    }
    return worst;
  }
};
}

void buildMap(GridMap2D& map, const PlannerParams2D& p, const std::vector<Vec2>& obstacles)
{
  map.configure(p); map.resetAround({0, 0}); map.beginUpdate(1.0);
  for (const auto& obstacle : obstacles) map.setOccupiedWorld(obstacle);
  map.finishUpdate(); map.inflateObstacles(); map.computeDistanceField();
}

void validate(const PlannerManager2D::PlanResult& result, const GridMap2D& map,
              const PlannerParams2D& p, const Vec2& start, const Vec2& goal,
              const Vec2& velocity = {}, const Vec2& acceleration = {})
{
  require(result.success, "planner rejected expected feasible scenario: " + result.message);
  const auto& trajectory = result.trajectory;
  require(trajectory.valid() && trajectory.collisionFree(map), "continuous collision acceptance");
  require(trajectory.feasible(p.max_vel, p.max_acc), "continuous dynamic acceptance");
  require(dist(trajectory.evaluate(0), start) < 1e-8, "start position preserved");
  require(dist(trajectory.evaluate(0, 1), velocity) < 1e-8, "start velocity preserved");
  require(dist(trajectory.evaluate(0, 2), acceleration) < 1e-8, "start acceleration preserved");
  require(dist(trajectory.evaluate(trajectory.duration()), goal) < 1e-7, "requested endpoint preserved");
  require(norm(trajectory.evaluate(trajectory.duration(), 1)) < 1e-7, "terminal velocity zero");
  require(norm(trajectory.evaluate(trajectory.duration(), 2)) < 1e-7, "terminal acceleration zero");
  require(result.optimizer_evaluations <= p.diff_max_evaluations, "shared evaluation budget");
  double peak_v = 0.0, peak_a = 0.0;
  for (double t = 0; t < trajectory.duration(); t += 0.005) {
    require(!map.isOccupiedWorld(trajectory.evaluate(t)), "independent dense collision oracle");
    peak_v = std::max(peak_v, norm(trajectory.evaluate(t, 1)));
    peak_a = std::max(peak_a, norm(trajectory.evaluate(t, 2)));
  }
  require(peak_v <= p.max_vel * (1.0 + 1e-7) && peak_a <= p.max_acc * (1.0 + 1e-7), "dense dynamic oracle");
  for (std::size_t i = 1; i < trajectory.pieces.size(); ++i) {
    PolynomialTrajectory2D left, right;
    left.pieces = {trajectory.pieces[i-1]}; right.pieces = {trajectory.pieces[i]};
    for (int derivative = 0; derivative <= 4; ++derivative)
      require(dist(left.evaluate(left.duration(), derivative), right.evaluate(0, derivative)) < 1e-6,
              "MINCO internal C4 continuity");
  }
  std::cout << "accepted pieces=" << trajectory.pieces.size() << " solve_ms=" << result.planning_ms
            << " peak_v=" << peak_v << " peak_a=" << peak_a << std::endl;
}

int main(int argc, char** argv)
{
  try {
    PlannerParams2D p;
    p.resolution = 0.05; p.map_size_x = p.map_size_y = 10.0; p.inflate_radius = 0.45;
    p.max_vel = 0.35; p.max_acc = 0.8; p.dist0 = 0.25;
    p.optimizer_max_iter = 100; p.lookahead_dist = 0.35;
    PlannerManager2D manager; manager.setParams(p);
    GridMap2D map;
    buildMap(map, p, {});
    validate(manager.plan(map, {-2, -1}, {2, 1}), map, p, {-2, -1}, {2, 1});
    validate(manager.plan(map, {0, 0}, {0.01, 0}), map, p, {0, 0}, {0.01, 0});
    validate(manager.plan(map, {0, 0}, {0, 0}), map, p, {0, 0}, {0, 0});
    const Vec2 moving{0.18, 0.05}, accelerating{0.03, -0.02};
    validate(manager.reboundReplan(map, {-1, 0}, {1, 0.5}, false, moving, accelerating),
             map, p, {-1, 0}, {1, 0.5}, moving, accelerating);
    require(!manager.reboundReplan(map, {0, 0}, {1, 1}, false, {0.36, 0}, {}).success,
            "overspeed initial state rejected");
    require(!manager.plan(map, {0, 0}, {6, 0}).success, "outside rolling map rejected");
    require(!manager.plan(map, {std::numeric_limits<double>::quiet_NaN(), 0}, {1, 0}).success,
            "non-finite start rejected");
    buildMap(map, p, {{0, 0}});
    validate(manager.plan(map, {-2, -0.2}, {2, 0.2}), map, p, {-2, -0.2}, {2, 0.2});
    require(!manager.plan(map, {-2, 0}, {0, 0}).success, "occupied goal retained and rejected");
    buildMap(map, p, {{-0.5, 0.35}, {0.7, -0.35}});
    validate(manager.plan(map, {-2, -0.3}, {2, 0.3}), map, p, {-2, -0.3}, {2, 0.3});
    std::vector<Vec2> u_obstacle;
    for (double y = -1.2; y <= 1.2; y += 0.05) u_obstacle.push_back({0.8, y});
    for (double x = -1.0; x <= 0.8; x += 0.05) {
      u_obstacle.push_back({x, 1.2}); u_obstacle.push_back({x, -1.2});
    }
    buildMap(map, p, u_obstacle);
    validate(manager.plan(map, {-0.2, 0}, {2.2, 0}), map, p, {-0.2, 0}, {2.2, 0});
    std::vector<Vec2> corridor;
    for (double x = -3; x <= 3; x += 0.05) { corridor.push_back({x, 0.85}); corridor.push_back({x, -0.85}); }
    buildMap(map, p, corridor);
    validate(manager.plan(map, {-2, 0}, {2, 0}), map, p, {-2, 0}, {2, 0});
    std::vector<Vec2> blocked;
    for (double y = -5; y < 5; y += 0.04) blocked.push_back({0, y});
    buildMap(map, p, blocked);
    require(!manager.plan(map, {-2, 0}, {2, 0}).success, "impenetrable wall rejected");

    // Endpoints and their chord are free; the curved interior hits an obstacle.
    PlannerParams2D tiny = p; tiny.inflate_radius = 0.05;
    buildMap(map, tiny, {{0, 0}});
    PolynomialTrajectory2D curved;
    PolynomialTrajectory2D::Piece piece;
    piece.duration = 1; piece.coefficients[0] = {-1, -0.5};
    piece.coefficients[1] = {2, 2}; piece.coefficients[2] = {0, -2};
    curved.pieces = {piece};
    require(!map.isOccupiedWorld(curved.evaluate(0)) && !map.isOccupiedWorld(curved.evaluate(1)), "free curve endpoints");
    require(!curved.collisionFree(map), "interior curve collision rejected");
    require(curved.collisionFree(map, 0.9), "executed unsafe prefix excluded from remaining check");
    PolynomialTrajectory2D hidden_speed;
    piece.coefficients = {}; piece.coefficients[2] = {1, 0};
    piece.coefficients[3] = {-2, 0}; piece.coefficients[4] = {1, 0};
    hidden_speed.pieces = {piece};
    require(norm(hidden_speed.evaluate(0, 1)) == 0 && norm(hidden_speed.evaluate(1, 1)) == 0,
            "zero endpoint speed counterexample");
    require(!hidden_speed.feasible(0.1, 100), "interior speed peak rejected");

    // Check the actual adapted adjoint gradient with active obstacle penalties.
    buildMap(map, p, {{0, 0}});
    DiffOptimizer2D optimizer;
    PlannerParams2D gradient_p = p;
    gradient_p.diff_max_evaluations = 1; gradient_p.diff_max_solve_ms = 1000;
    optimizer.optimize(map, {{-1, 0.3}, {-0.3, 0.3}, {0.3, 0.3}, {1, 0.3}}, {}, {}, gradient_p,
      Clock::now() + std::chrono::seconds(2));
    const double error = DiffPlannerTestAccess::gradientError(optimizer);
    require(std::isfinite(error) && error < 2e-4, "analytic waypoint/time gradient matches central differences");
    std::cout << "gradient_relative_error=" << error << " assertions=" << assertions << std::endl;
    if (argc > 1) {
      std::ofstream out(argv[1]);
      out << std::setprecision(12) << "{\"passed\":true,\"assertions\":" << assertions
          << ",\"gradient_relative_error\":" << error << "}\n";
    }
    return 0;
  } catch (const std::exception& error) {
    std::cerr << "FAIL: " << error.what() << " after " << assertions << " assertions\n";
    return 1;
  }
}
