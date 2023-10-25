#ifndef CURVE_NODE_H_
#define CURVE_NODE_H_

#include <string>
#include <vector>

#include "gloo/SceneNode.hpp"
#include "gloo/VertexObject.hpp"
#include "gloo/shaders/ShaderProgram.hpp"

namespace GLOO {

enum class SplineBasis { Bezier, BSpline };

struct CurvePoint {
  glm::vec3 P;
  glm::vec3 T;
};

class CurveNode : public SceneNode {
 public:
  CurveNode(SplineBasis basis, glm::mat4x3 control);
  void Update(double delta_time) override;

 private:
  void ToggleSplineBasis();
  void ConvertGeometry();
  CurvePoint EvalCurve(float t);
  void InitCurve();
  void PlotCurve();
  void PlotControlPoints();
  void PlotTangentLine();

  SplineBasis spline_basis_;

  std::shared_ptr<VertexObject> sphere_mesh_;
  std::shared_ptr<VertexObject> curve_polyline_;
  std::shared_ptr<VertexObject> tangent_line_;

  std::shared_ptr<ShaderProgram> shader_;
  std::shared_ptr<ShaderProgram> polyline_shader_;

  GLOO::SplineBasis basis;
  glm::mat4x3 control;
  glm::mat4 B;
  glm::mat4 dB;
  glm::mat4 BS;
  glm::mat4 dBS;

  std::vector<SceneNode*> point_nodes;

  const int N_SUBDIV_ = 50;
};
}  // namespace GLOO

#endif
