#include "CurveNode.hpp"

#include "gloo/debug/PrimitiveFactory.hpp"
#include "gloo/components/RenderingComponent.hpp"
#include "gloo/components/ShadingComponent.hpp"
#include "gloo/components/MaterialComponent.hpp"
#include "gloo/shaders/PhongShader.hpp"
#include "gloo/shaders/SimpleShader.hpp"
#include "gloo/InputManager.hpp"

namespace GLOO {
CurveNode::CurveNode(SplineBasis inputbasis, glm::mat4x3 inputcontrol) {
  sphere_mesh_ = PrimitiveFactory::CreateSphere(0.015f, 25, 25);
  curve_polyline_ = std::make_shared<VertexObject>();
  tangent_line_ = std::make_shared<VertexObject>();
  shader_ = std::make_shared<PhongShader>();
  polyline_shader_ = std::make_shared<SimpleShader>();

  basis = inputbasis;
  control = inputcontrol;

  B = glm::mat4(
        1.0f, 0.0f, 0.0f, 0.0f,
        -3.0f, 3.0f, 0.0f, 0.0f, 
        3.0f, -6.0f, 3.0f, 0.0f,
        -1.0f, 3.0f, -3.0f, 1.0f);
  dB = glm::mat4(
      -3.0f, 3.0f, 0.0f, 0.0f,
      6.0f, -12.0f, 6.0f, 0.0f,
      -3.0f, 9.0f, -9.0f, 3.0f,
      0.0f, 0.0f, 0.0f, 0.0f);
  BS = (1.0f / 6.0f) * glm::mat4(
        1.0f, 4.0f, 1.0f, 0.0f,
        -3.0f, 0.0f, 3.0f, 0.0f,
        3.0f, -6.0f, 3.0f, 0.0f,
        -1.0f, 3.0f, -3.0f, 1.0f);
  dBS = (1.0f / 6.0f) * glm::mat4(
      -3.0f, 0.0f, 3.0f, 0.0f,
      6.0f, -12.0f, 6.0f, 0.0f,
      -3.0f, 9.0f, -9.0f, 3.0f, 
      0.0f, 0.0f, 0.0f, 0.0f);

  InitCurve();
  PlotCurve();
}

void CurveNode::Update(double delta_time) {

  // Prevent multiple toggle.
  static bool prev_released = true;

  if (InputManager::GetInstance().IsKeyPressed('T')) {
    if (prev_released) {
      // TODO: implement toggling spline bases.
      ToggleSplineBasis();
      PlotControlPoints();
      PlotCurve();
      PlotTangentLine();
    }
    prev_released = false;
  } else if (InputManager::GetInstance().IsKeyPressed('B')) {
    if (prev_released) {
      // TODO: implement converting conrol point geometry from Bezier to
      // B-Spline basis.
      ConvertGeometry();
      PlotControlPoints();
      PlotCurve();
      PlotTangentLine();
    }
    prev_released = false;
  } else if (InputManager::GetInstance().IsKeyPressed('Z')) {
    if (prev_released) {
      // TODO: implement converting conrol point geometry from B-Spline to
      // Bezier basis.
      ConvertGeometry();
      PlotControlPoints();
      PlotCurve();
      PlotTangentLine();
    }
    prev_released = false;
  } else {
    prev_released = true;
  }
}

void CurveNode::ToggleSplineBasis() {
  if (basis == SplineBasis::Bezier) {
    basis = SplineBasis::BSpline;
  } else {
    basis = SplineBasis::Bezier;
  }
}

void CurveNode::ConvertGeometry() {
  glm::mat4 M1;
  glm::mat4 invM2;

  if (basis == SplineBasis::Bezier) {
    M1 = B;
    invM2 = glm::inverse(BS);
  } else {
    M1 = BS;
    invM2 = glm::inverse(B);
  }
  control = control * M1 * invM2;
}

CurvePoint CurveNode::EvalCurve(float t) {
  // TODO: implement evaluating the spline curve at parameter value t.
  glm::vec4 T(1.0f, t, t*t, t*t*t);
  glm::vec3 point;
  glm::vec3 tangent;

  if (basis == SplineBasis::Bezier) {
    point = control*B*T;
    tangent = control*dB*T;
  } else {
    point = control*BS*T;
    tangent = control*dBS*T;
  }

  return CurvePoint{point, tangent};
}

void CurveNode::InitCurve() {
  auto shader = std::make_shared<SimpleShader>();

  // curve line
  PlotCurve();
  auto curve_node = make_unique<SceneNode>();
  curve_node->CreateComponent<ShadingComponent>(polyline_shader_);

  auto& rc = curve_node->CreateComponent<RenderingComponent>(curve_polyline_);
  rc.SetDrawMode(DrawMode::Lines);

  glm::vec3 color(1.f, 0.f, 1.f);
  auto material = std::make_shared<Material>(color, color, color, 0);
  curve_node->CreateComponent<MaterialComponent>(material);

  AddChild(std::move(curve_node));

  // tangent line
  PlotTangentLine();
  auto line_node = make_unique<SceneNode>();
  line_node->CreateComponent<ShadingComponent>(polyline_shader_);

  auto& rc_t = line_node->CreateComponent<RenderingComponent>(tangent_line_);
  rc_t.SetDrawMode(DrawMode::Lines);

  glm::vec3 color_t(1.f, 1.f, 1.f);
  auto material_t = std::make_shared<Material>(color_t, color_t, color_t, 0);
  line_node->CreateComponent<MaterialComponent>(material_t);

  AddChild(std::move(line_node));

  // control points
  
  for (int i=0; i<=3; ++i) {
    auto point_node = make_unique<SceneNode>();
    point_nodes.push_back(point_node.get());
    point_node->CreateComponent<ShadingComponent>(shader_);
    auto& rc = point_node->CreateComponent<RenderingComponent>(sphere_mesh_);

    if(basis == SplineBasis::Bezier) {
      glm::vec3 red_color(1.0f,0.0f,0.0f);
      auto red_material = std::make_shared<Material>(red_color, red_color, red_color, 20.0f);
      point_node->CreateComponent<MaterialComponent>(red_material);
    } else {
      glm::vec3 green_color(0.0f,1.0f,0.0f);
      auto green_material = std::make_shared<Material>(green_color, green_color, green_color, 20.0f);
      point_node->CreateComponent<MaterialComponent>(green_material);
    }

    rc.SetDrawMode(DrawMode::Triangles);
    point_node->GetTransform().SetPosition(control[i]);
    AddChild(std::move(point_node));
  }

}

void CurveNode::PlotCurve() {
  // TODO: plot the curve by updating the positions of its VertexObject.

  auto positions = make_unique<PositionArray>();
  auto indices = make_unique<IndexArray>();

  for (int i=0; i<=N_SUBDIV_; ++i) {
    float t = static_cast<float>(i) / N_SUBDIV_;

    auto point = EvalCurve(t);
    
    positions->push_back(point.P);
    indices->push_back(i);
    indices->push_back(i+1);
  }
  indices->pop_back();
  indices->pop_back();
  
  curve_polyline_->UpdatePositions(std::move(positions));
  curve_polyline_->UpdateIndices(std::move(indices));

}

void CurveNode::PlotControlPoints() {
  // TODO: plot the curve control points.
  for (int i=0; i<=3; ++i) {
    auto point = point_nodes[i];
    if(basis == SplineBasis::Bezier) {
      glm::vec3 red_color(1.0f,0.0f,0.0f);
      auto red_material = std::make_shared<Material>(red_color, red_color, red_color, 20.0f);
      point->CreateComponent<MaterialComponent>(red_material);
    } else {
      glm::vec3 green_color(0.0f,1.0f,0.0f);
      auto green_material = std::make_shared<Material>(green_color, green_color, green_color, 20.0f);
      point->CreateComponent<MaterialComponent>(green_material);
    }
    point->GetTransform().SetPosition(control[i]);
  }
}

void CurveNode::PlotTangentLine() {
  // TODO: implement plotting a line tangent to the curve.
  auto positions = make_unique<PositionArray>();
  auto indices = make_unique<IndexArray>();

  float t = 0.5;

  auto point = EvalCurve(t);
  glm::vec3 tangent = glm::normalize(point.T);
  tangent = tangent/8.0f;
    
  positions->push_back(point.P-tangent);
  positions->push_back(point.P+tangent);
  indices->push_back(0);
  indices->push_back(1);

  tangent_line_->UpdatePositions(std::move(positions));
  tangent_line_->UpdateIndices(std::move(indices));
  
}
}  
