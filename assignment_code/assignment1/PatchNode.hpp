#ifndef PATCH_NODE_H_
#define PATCH_NODE_H_

#include <string>
#include <vector>

#include "gloo/SceneNode.hpp"
#include "gloo/VertexObject.hpp"
#include "gloo/shaders/ShaderProgram.hpp"

#include "CurveNode.hpp"

namespace GLOO {

enum class PatchBasis { Bezier, BSpline };

struct PatchPoint {
  glm::vec3 P;
  glm::vec3 N;
};

class PatchNode : public SceneNode {
 public:
  PatchNode(PatchBasis inputbasis, std::vector<glm::mat4> inputGs);

 private:
  PatchPoint EvalCurve(float u, float v);
  void PlotPatch();

  std::vector<glm::mat4> Gs_;
  PatchBasis patch_basis_;

  std::shared_ptr<VertexObject> patch_mesh_;
  std::shared_ptr<ShaderProgram> shader_;

  glm::mat4 B;
  glm::mat4 BS;
  glm::mat4 basis_M;

  const int N_SUBDIV_ = 50;
};
}  // namespace GLOO

#endif
