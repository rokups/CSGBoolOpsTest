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

class AtomicAsLibraryExample : public Application
{
ATOMIC_OBJECT(AtomicAsLibraryExample, Application)
public:
    AtomicAsLibraryExample(Context* context) : Application(context) { }
    SharedPtr<Scene> _scene;
    WeakPtr<Node> _camera;
    WeakPtr<Node> _a;
    WeakPtr<Node> _b;
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

//        _b = _scene->CreateChild();
//        _b->CreateComponent<StaticModel>()->SetModel(context_->GetResourceCache()->GetResource<Model>("Models/Cone.mdl"));
//        _b->SetScale(2);


        _camera->SetPosition({0, 0, -5});
        _camera->LookAt(_a->GetPosition());

        auto model_a = ToCSGJS(_a);
//        auto model_b = ToCSGJS(_b);
//        auto result = csgjs_difference(model_a, model_b);
//
//        _b->Remove();
        auto new_geom = FromCSGJS(model_a);
        auto new_model = new Model(context_);
        new_model->SetNumGeometries(1);
        new_model->SetGeometry(0, 0, new_geom);

        _a->GetComponent<StaticModel>()->SetModel(new_model);
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
//        if (ImGui::Begin("Example"))
//        {
//            ImGui::Text("This example application is using AtomicGameEngine as CMake dependency consumed through add_subdirectory().");
//        }
//        ImGui::End();
    }

    Geometry* FromCSGJS(csgjs_model& model)
    {
        SharedPtr<VertexBuffer> vb(new VertexBuffer(context_));
        SharedPtr<IndexBuffer> ib(new IndexBuffer(context_));
        Geometry* geom(new Geometry(context_));

        ib->SetShadowed(true);
        ib->SetSize(model.indices.size(), true);
        ib->SetData(&model.indices.front());

        PODVector<VertexElement> elements;
        elements.Push(VertexElement(TYPE_VECTOR3, SEM_POSITION, 0, false));
        elements.Push(VertexElement(TYPE_VECTOR3, SEM_NORMAL, 0, false));
        elements.Push(VertexElement(TYPE_VECTOR2, SEM_TEXCOORD, 0, false));

        const auto vds = (3 + 3 + 2);
        float* vertexData = new float[vds * model.vertices.size()];
        for (auto i = 0; i < model.vertices.size(); i++)
        {
            vertexData[vds * i + 0] = model.vertices[i].pos.x;
            vertexData[vds * i + 1] = model.vertices[i].pos.y;
            vertexData[vds * i + 2] = model.vertices[i].pos.z;

            vertexData[vds * i + 3] = model.vertices[i].normal.x;
            vertexData[vds * i + 4] = model.vertices[i].normal.y;
            vertexData[vds * i + 5] = model.vertices[i].normal.z;

            vertexData[vds * i + 6] = model.vertices[i].uv.x;
            vertexData[vds * i + 7] = model.vertices[i].uv.y;
        }
        vb->SetShadowed(true);
        vb->SetSize(model.vertices.size(), elements);
        vb->SetData(vertexData);

        geom->SetVertexBuffer(0, vb);
        geom->SetIndexBuffer(ib);
        geom->SetDrawRange(TRIANGLE_LIST, 0, model.vertices.size());

        return geom;
    }

    csgjs_model ToCSGJS(Node* node)
    {
        csgjs_model mdl{};
        auto static_model = node->GetComponent<StaticModel>();
        auto geom = static_model->GetLodGeometry(0, 0);

        mdl.vertices.reserve(geom->GetVertexCount());
        mdl.indices.reserve(geom->GetIndexCount());

        auto vb = geom->GetVertexBuffer(0);
        auto vertexData = vb->GetShadowData();
        auto elements = vb->GetElements();
        auto vertexSize = vb->GetVertexSize();

        for (unsigned i = 0; i < vb->GetVertexCount(); i++)
        {
            csgjs_vertex vt{};

            for (unsigned j = 0; j < elements.Size(); j++)
            {
                const auto& el = elements.At(j);
                switch (el.semantic_)
                {
                case SEM_POSITION:
                {
                    auto pos = reinterpret_cast<const Vector3*>(vertexData + el.offset_ + vertexSize * i);
                    vt.pos.x = pos->x_;
                    vt.pos.y = pos->y_;
                    vt.pos.z = pos->z_;
                    break;
                }
                case SEM_NORMAL:
                {
                    auto normal = reinterpret_cast<const Vector3*>(vertexData + el.offset_ + vertexSize * i);
                    vt.normal.x = normal->x_;
                    vt.normal.y = normal->y_;
                    vt.normal.z = normal->z_;
                    break;
                }
                case SEM_TEXCOORD:
                {
                    auto uv = reinterpret_cast<const Vector2*>(vertexData + el.offset_ + vertexSize * i);
                    vt.uv.x = uv->x_;
                    vt.uv.y = uv->y_;
                    vt.uv.z = 0;
                    break;
                }
                default:
                    break;
                }
            }
            mdl.vertices.push_back(vt);
        }

        auto ib = geom->GetIndexBuffer();
        auto indexes = (uint16_t*)ib->GetShadowData();
        for (unsigned i = 0; i < ib->GetIndexCount(); i++)
            mdl.indices.push_back((int)indexes[i]);

        return mdl;
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
