#include "PatchNode.hpp"

#include "gloo/components/RenderingComponent.hpp"
#include "gloo/components/ShadingComponent.hpp"
#include "gloo/components/MaterialComponent.hpp"
#include "gloo/shaders/PhongShader.hpp"

namespace GLOO {
PatchNode::PatchNode(PatchBasis inputbasis, std::vector<glm::mat4> inputGs) {
  shader_ = std::make_shared<PhongShader>();
  patch_mesh_ = std::make_shared<VertexObject>();
  patch_basis_ = inputbasis;
  Gs_ = inputGs;

  B = glm::mat4(
        1.0f, 0.0f, 0.0f, 0.0f,
        -3.0f, 3.0f, 0.0f, 0.0f, 
        3.0f, -6.0f, 3.0f, 0.0f,
        -1.0f, 3.0f, -3.0f, 1.0f);
  BS = (1.0f / 6.0f) * glm::mat4(
        1.0f, 4.0f, 1.0f, 0.0f,
        -3.0f, 0.0f, 3.0f, 0.0f,
        3.0f, -6.0f, 3.0f, 0.0f,
        -1.0f, 3.0f, -3.0f, 1.0f);

  if (patch_basis_ == PatchBasis::Bezier) {
    basis_M = B;
  } else {
    basis_M = BS;
  }

  PlotPatch();
  auto patch_node = make_unique<SceneNode>();
  patch_node->CreateComponent<ShadingComponent>(shader_);
  auto& rc = patch_node->CreateComponent<RenderingComponent>(patch_mesh_);
  rc.SetDrawMode(DrawMode::Triangles);

  AddChild(std::move(patch_node));
}

PatchPoint PatchNode::EvalCurve(float u, float v) {
  glm::vec4 T_u(1.0f, u, u*u, u*u*u);
  glm::vec4 T_v(1.0f, v, v*v, v*v*v);
  glm::vec4 d_u(0.0f, 1.0f, 2*u, 3*u*u);
  glm::vec4 d_v(0.0f, 1.0f, 2*v, 3*v*v);

  glm::vec3 point(
    glm::dot(T_u, glm::transpose(basis_M) * Gs_[0] * basis_M * T_v),
    glm::dot(T_u, glm::transpose(basis_M) * Gs_[1] * basis_M * T_v),
    glm::dot(T_u, glm::transpose(basis_M) * Gs_[2] * basis_M * T_v)
  );

  glm::vec3 partial_u(
    glm::dot(d_u, glm::transpose(basis_M) * Gs_[0] * basis_M * T_v),
    glm::dot(d_u, glm::transpose(basis_M) * Gs_[1] * basis_M * T_v),
    glm::dot(d_u, glm::transpose(basis_M) * Gs_[2] * basis_M * T_v)
  );

  glm::vec3 partial_v(
    glm::dot(T_u, glm::transpose(basis_M) * Gs_[0] * basis_M * d_v),
    glm::dot(T_u, glm::transpose(basis_M) * Gs_[1] * basis_M * d_v),
    glm::dot(T_u, glm::transpose(basis_M) * Gs_[2] * basis_M * d_v)
  );

  glm::vec3 normal = glm::cross(partial_u, partial_v);
  normal = -glm::normalize(normal);

  return PatchPoint{point, normal};
}

void PatchNode::PlotPatch() {
  auto positions = make_unique<PositionArray>();
  auto normals = make_unique<NormalArray>();
  auto indices = make_unique<IndexArray>();
  
  for (int i=0; i<=N_SUBDIV_; ++i) {
    for (int j=0; j<=N_SUBDIV_; ++j) {
      float u = static_cast<float>(i) / N_SUBDIV_;
      float v = static_cast<float>(j) / N_SUBDIV_;

      auto point = EvalCurve(u, v);
      positions->push_back(point.P);
      normals->push_back(point.N);

    }
  }

  for (int i = 0; i < N_SUBDIV_; ++i) {
    for (int j = 0; j<N_SUBDIV_; ++j) {
      float ind0 = i*(N_SUBDIV_+1) + j;
      float ind1 = ind0 + 1;
      float ind2 = (i+1)*(N_SUBDIV_+1)+j;
      float ind3 = ind2+1;
      indices->push_back(ind0);
      indices->push_back(ind1);
      indices->push_back(ind3);
      indices->push_back(ind3);
      indices->push_back(ind2);
      indices->push_back(ind0);
    }
  }

  patch_mesh_->UpdatePositions(std::move(positions));
  patch_mesh_->UpdateNormals(std::move(normals));
  patch_mesh_->UpdateIndices(std::move(indices));
}
}  // namespace GLOO
