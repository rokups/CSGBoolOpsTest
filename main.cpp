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

#include "CSGJS.h"


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
        light->SetLightType(LIGHT_DIRECTIONAL);

        _a = _scene->CreateChild();
        _a->CreateComponent<StaticModel>()->SetModel(context_->GetResourceCache()->GetResource<Model>("Models/Box.mdl"));

        _b = _scene->CreateChild();
        _b->CreateComponent<StaticModel>()->SetModel(context_->GetResourceCache()->GetResource<Model>("Models/Cone.mdl"));

        _c = _scene->CreateChild();
        _c->CreateComponent<StaticModel>();
        _c->SetPosition({2, 0, 0});

        _camera->SetPosition({0, 0, -5});
        _camera->LookAt(_a->GetPosition());
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

            Geometry* new_geom = nullptr;
            if (ui::Button("Union"))
                new_geom = csgjs_union(_a, _b);

            if (ui::Button("Intersection"))
                new_geom = csgjs_intersection(_a, _b);

            if (ui::Button("Difference"))
                new_geom = csgjs_difference(_a, _b);

            if (new_geom)
            {
                auto new_model = new Model(context_);
                new_model->SetNumGeometries(1);
                new_model->SetGeometry(0, 0, new_geom);
                _c->GetComponent<StaticModel>()->SetModel(new_model);
            }
        }
        ui::End();
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
};

ATOMIC_DEFINE_APPLICATION_MAIN(AtomicAsLibraryExample);
