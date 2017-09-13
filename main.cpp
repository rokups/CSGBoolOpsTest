//
// Copyright (c) 2017 Rokas Kupstys
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
// THE SOFTWARE.
//
#include <functional>

#include <Atomic/Engine/Application.h>
#include <Atomic/Core/CoreEvents.h>
#include <Atomic/Scene/Scene.h>
#include <Atomic/Graphics/Graphics.h>
#include <Atomic/Graphics/Renderer.h>
#include <Atomic/Graphics/Octree.h>
#include <Atomic/Graphics/Camera.h>
#include <Atomic/Graphics/Model.h>
#include <Atomic/Graphics/StaticModel.h>
#include <Atomic/Graphics/DebugRenderer.h>
#include <Atomic/Graphics/Technique.h>
#include <Atomic/Graphics/Texture2D.h>
#include <Atomic/IO/FileSystem.h>
#include <Atomic/IO/FileWatcher.h>
#include <Atomic/Resource/ResourceMapRouter.h>
#include <Atomic/Input/Input.h>
#include <Atomic/Graphics/CustomGeometry.h>
#include <Atomic/Graphics/Zone.h>
#include <Atomic/UI/SystemUI/SystemUI.h>
#include <Atomic/Core/StringUtils.h>
#include <Atomic/Engine/EngineDefs.h>
#include <Atomic/Graphics/Geometry.h>
#include <Atomic/IO/Log.h>

#include "CSGJS.h"
#include "ImGuizmo.h"

using namespace Atomic;
using namespace std::placeholders;
namespace ui = ImGui;

class AtomicAsLibraryExample : public Application
{
ATOMIC_OBJECT(AtomicAsLibraryExample, Application)
public:
    AtomicAsLibraryExample(Context* context) : Application(context) { }
    SharedPtr<Scene> _scene;
    WeakPtr<Node> _camera;
    WeakPtr<Node> _a;
    WeakPtr<Node> _b;
    WeakPtr<Node> _c;
    float pitch_;
    float yaw_;


    virtual void Setup()
    {
        // Modify engine startup parameters
        engineParameters_[EP_WINDOW_TITLE]   = GetTypeName();
        engineParameters_[EP_WINDOW_WIDTH]   = 1027;
        engineParameters_[EP_WINDOW_HEIGHT]  = 768;
        engineParameters_[EP_FULL_SCREEN]    = false;
        engineParameters_[EP_RESOURCE_PATHS] = "CoreData;Data";
        engineParameters_[EP_LOG_LEVEL]      = LOG_DEBUG;

        // This is a dirty hack to make example run out of the box. You likely want to fix this.
        engineParameters_[EP_RESOURCE_PREFIX_PATHS] = CMAKE_SOURCE_DIR "/AtomicGameEngine/Resources;" CMAKE_SOURCE_DIR;
    }

    virtual void Start()
    {
        context_->GetInput()->SetMouseVisible(true);
        SubscribeToEvent(E_SYSTEMUIFRAME, std::bind(&AtomicAsLibraryExample::OnSystemUI, this, _2));
        SubscribeToEvent(E_UPDATE, std::bind(&AtomicAsLibraryExample::OnUpdate, this, _2));

        _scene = new Scene(context_);
        _scene->CreateComponent<Octree>();
        auto zone = _scene->CreateComponent<Zone>();
        zone->SetBoundingBox(BoundingBox(-1000.0f, 1000.0f));
        zone->SetAmbientColor(Color(0.15f, 0.15f, 0.15f));
        zone->SetFogColor(Color(0.5f, 0.5f, 0.7f));
        zone->SetFogStart(100.0f);
        zone->SetFogEnd(300.0f);

        _scene->CreateComponent<Octree>();
        _camera = _scene->CreateChild("Camera");
        _camera->CreateComponent<Camera>();
        context_->GetRenderer()->SetViewport(0, new Viewport(context_, _scene, _camera->GetComponent<Camera>()));

        auto light = _camera->CreateComponent<Light>();
        light->SetColor(Color::WHITE);
        light->SetLightType(LIGHT_POINT);

        _a = _scene->CreateChild();
        _a->CreateComponent<StaticModel>()->SetModel(context_->GetResourceCache()->GetResource<Model>("Models/Box.mdl"));

        _b = _scene->CreateChild();
        _b->CreateComponent<StaticModel>()->SetModel(context_->GetResourceCache()->GetResource<Model>("Models/Cone.mdl"));

        _c = _scene->CreateChild();
        _c->CreateComponent<StaticModel>();
        _c->SetPosition({2, 0, 0});

        _camera->SetPosition({0, 0, -5});
        _camera->LookAt(_a->GetPosition());

        ImGuizmo::Enable(true);
    }

    virtual void Stop()
    {
    }

    void OnUpdate(VariantMap& args)
    {
        auto delta = args[Update::P_TIMESTEP].GetFloat();
        MoveCamera(delta);
    }
    void OnSystemUI(VariantMap& args)
    {
        ui::SetNextWindowSize({300, 250}, ImGuiSetCond_Once);
        if (ui::Begin("Example"))
        {
            {
                auto pos = _a->GetPosition();
                if (ui::DragFloat3("A Pos", const_cast<float*>(pos.Data()), 0.05f))
                    _a->SetPosition(pos);

                auto rot = _a->GetRotation().EulerAngles();
                if (ui::DragFloat3("A Rot", const_cast<float*>(rot.Data()), 0.05f))
                    _a->SetRotation(Quaternion(rot.x_, rot.y_, rot.z_));

                auto scale = _a->GetScale();
                if (ui::DragFloat3("A Scl", const_cast<float*>(scale.Data()), 0.05f))
                    _a->SetScale(scale);
            }

            {
                auto pos = _b->GetPosition();
                if (ui::DragFloat3("B Pos", const_cast<float*>(pos.Data()), 0.05f))
                    _b->SetPosition(pos);

                auto rot = _b->GetRotation().EulerAngles();
                if (ui::DragFloat3("B Rot", const_cast<float*>(rot.Data()), 0.05f))
                    _b->SetRotation(Quaternion(rot.x_, rot.y_, rot.z_));

                auto scale = _b->GetScale();
                if (ui::DragFloat3("B Scl", const_cast<float*>(scale.Data()), 0.05f))
                    _b->SetScale(scale);
            }

            PODVector<Geometry*> geoms;
            if (ui::Button("Union"))
                geoms.Push(csgjs_union(_a, _b));

            if (ui::Button("Intersection"))
                geoms.Push(csgjs_intersection(_a, _b));

            if (ui::Button("Difference"))
                geoms.Push(csgjs_difference(_a, _b));

            if (ui::Button("Difference Disjoint"))
                csgjs_difference(_a, _b, geoms);

            if (!geoms.Empty())
            {
                auto new_model = new Model(context_);
                new_model->SetNumGeometries(geoms.Size());
                auto index = 0;
                for (auto new_geom : geoms)
                    new_model->SetGeometry(index++, 0, new_geom);
                _c->GetComponent<StaticModel>()->SetModel(new_model);
            }

        }
        ui::End();

        ImGuizmo::BeginFrame();

        auto m = _a->GetTransform().ToMatrix4();

//        auto view = _camera->GetComponent<Camera>()->GetView().ToMatrix4();
//        auto projection = _camera->GetComponent<Camera>()->GetProjection();
//        ImGuizmo::DrawCube(view.Data(), projection.Data(), const_cast<float*>(m.Data()));

        EditTransform(_camera->GetComponent<Camera>(), m);
        if (ImGuizmo::IsUsing())
            _a->SetTransform(m);
    }

    void MoveCamera(float timeStep)
    {
        Input* input = GetSubsystem<Input>();
        input->SetMouseVisible(!input->GetMouseButtonDown(MOUSEB_RIGHT));

        // Movement speed as world units per second
        const float MOVE_SPEED = 5.0f;
        // Mouse sensitivity as degrees per pixel
        const float MOUSE_SENSITIVITY = 0.1f;

        if (!input->IsMouseVisible())
        {
            // Use this frame's mouse motion to adjust camera node yaw and pitch. Clamp the pitch between -90 and 90 degrees
            IntVector2 mouseMove = input->GetMouseMove();
            yaw_ += MOUSE_SENSITIVITY * mouseMove.x_;
            pitch_ += MOUSE_SENSITIVITY * mouseMove.y_;
            pitch_ = Clamp(pitch_, -90.0f, 90.0f);

            // Construct new orientation for the camera scene node from yaw and pitch. Roll is fixed to zero
            _camera->SetRotation(Quaternion(pitch_, yaw_, 0.0f));
        }

        // Read WASD keys and move the camera scene node to the corresponding direction if they are pressed
        if (input->GetKeyDown(KEY_W))
            _camera->Translate(Vector3::FORWARD * MOVE_SPEED * timeStep);
        if (input->GetKeyDown(KEY_S))
            _camera->Translate(Vector3::BACK * MOVE_SPEED * timeStep);
        if (input->GetKeyDown(KEY_A))
            _camera->Translate(Vector3::LEFT * MOVE_SPEED * timeStep);
        if (input->GetKeyDown(KEY_D))
            _camera->Translate(Vector3::RIGHT * MOVE_SPEED * timeStep);
    }

    void EditTransform(const Camera* camera, Matrix4& matrix)
    {
        static ImGuizmo::OPERATION mCurrentGizmoOperation(ImGuizmo::ROTATE);
        static ImGuizmo::MODE mCurrentGizmoMode(ImGuizmo::WORLD);
        if (ImGui::IsKeyPressed(90))
            mCurrentGizmoOperation = ImGuizmo::TRANSLATE;
        if (ImGui::IsKeyPressed(69))
            mCurrentGizmoOperation = ImGuizmo::ROTATE;
        if (ImGui::IsKeyPressed(82)) // r Key
            mCurrentGizmoOperation = ImGuizmo::SCALE;
        if (ImGui::RadioButton("Translate", mCurrentGizmoOperation == ImGuizmo::TRANSLATE))
            mCurrentGizmoOperation = ImGuizmo::TRANSLATE;
        ImGui::SameLine();
        if (ImGui::RadioButton("Rotate", mCurrentGizmoOperation == ImGuizmo::ROTATE))
            mCurrentGizmoOperation = ImGuizmo::ROTATE;
        ImGui::SameLine();
        if (ImGui::RadioButton("Scale", mCurrentGizmoOperation == ImGuizmo::SCALE))
            mCurrentGizmoOperation = ImGuizmo::SCALE;
        float matrixTranslation[3], matrixRotation[3], matrixScale[3];
        ImGuizmo::DecomposeMatrixToComponents(matrix.Data(), matrixTranslation, matrixRotation, matrixScale);
        ImGui::DragFloat3("Tr", matrixTranslation, 3);
        ImGui::DragFloat3("Rt", matrixRotation, 3);
        ImGui::DragFloat3("Sc", matrixScale, 3);
        ImGuizmo::RecomposeMatrixFromComponents(matrixTranslation, matrixRotation, matrixScale,
                                                const_cast<float*>(matrix.Data()));

        if (mCurrentGizmoOperation != ImGuizmo::SCALE)
        {
            if (ImGui::RadioButton("Local", mCurrentGizmoMode == ImGuizmo::LOCAL))
                mCurrentGizmoMode = ImGuizmo::LOCAL;
            ImGui::SameLine();
            if (ImGui::RadioButton("World", mCurrentGizmoMode == ImGuizmo::WORLD))
                mCurrentGizmoMode = ImGuizmo::WORLD;
        }
        static bool useSnap(false);
        if (ImGui::IsKeyPressed(83))
            useSnap = !useSnap;
        ImGui::Checkbox("", &useSnap);
        ImGui::SameLine();
        Vector3 snap;
        switch (mCurrentGizmoOperation)
        {
        case ImGuizmo::TRANSLATE:
            snap = config.mSnapTranslation;
            ImGui::InputFloat3("Snap", &snap.x_);
            break;
        case ImGuizmo::ROTATE:
            snap = config.mSnapRotation;
            ImGui::InputFloat("Angle Snap", &snap.x_);
            break;
        case ImGuizmo::SCALE:
            snap = config.mSnapScale;
            ImGui::InputFloat("Scale Snap", &snap.x_);
            break;
        }
        ImGuiIO& io = ImGui::GetIO();
        ImGuizmo::SetRect(0, 0, io.DisplaySize.x, io.DisplaySize.y);

        auto view = camera->GetView().ToMatrix4();
        auto projection = camera->GetProjection();

        ImGuizmo::Manipulate(view.Data(), projection.Data(), mCurrentGizmoOperation, mCurrentGizmoMode,
                             const_cast<float*>(matrix.Data()), NULL, useSnap ? &snap.x_ : NULL);
    }

    struct
    {
        Vector3 mSnapTranslation;
        Vector3 mSnapRotation;
        Vector3 mSnapScale;
    } config;
};

ATOMIC_DEFINE_APPLICATION_MAIN(AtomicAsLibraryExample);
