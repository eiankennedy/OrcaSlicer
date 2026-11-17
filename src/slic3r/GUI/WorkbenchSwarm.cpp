#include "WorkbenchSwarm.hpp"

#include "3DScene.hpp"
#include "Camera.hpp"
#include "GLCanvas3D.hpp"
#include "GUI_App.hpp"
#include "Plater.hpp"
#include "libslic3r/ClipperUtils.hpp"
#include "libslic3r/ExPolygon.hpp"
#include "libslic3r/Model.hpp"
#include "libslic3r/Tesselate.hpp"
#include "libslic3r/TriangleMesh.hpp"
#include "libslic3r/TriangleMeshSlicer.hpp"
#include "WorkbenchDock.hpp"

#include <imgui/imgui.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <map>
#include <memory>
#include <set>
#include <vector>

// The swarm works like the browser swarm that rebuilds a web page: each change is a plan of small tasks with their
// order (a layer's outline before its infill, a layer before the one above it), a pool of robots takes the nearest
// work that's ready, more join while there's more work than hands, and each piece appears as its robot finishes it.
// A new object is printed: layer by layer from the bed, each layer's outline traced by one robot and filled by
// another, the real object shown clipped at the height built so far. An arranged object is dragged along its path,
// a setting is typed into its card, slicing sweeps a plane up through each object. The geometry (slices, infill,
// caps) is worked out once per change; a frame only projects the points it draws.

namespace Slic3r {
namespace GUI {
namespace WorkbenchSwarm {

namespace {

constexpr size_t BASE = 4, MAX_BOTS = 14;
const ImU32 COLORS[4] = {IM_COL32(0xD9, 0x77, 0x57, 255), IM_COL32(0xE8, 0xA5, 0x5A, 255), IM_COL32(0xC9, 0x64, 0x42, 255),
                         IM_COL32(0xF0, 0xB0, 0x7A, 255)};

// a path in the scene: points, whether the pen is down along each segment, and the length so far at each point
struct Path
{
    std::vector<Vec3f> pts;
    std::vector<char>  pen;   // per segment
    std::vector<float> cum;
    float              len() const { return cum.empty() ? 0.f : cum.back(); }
    void               add(const Vec3f& p, bool down)
    {
        if (!pts.empty()) {
            pen.push_back(down ? 1 : 0);
            cum.push_back(cum.back() + (p - pts.back()).norm());
        } else
            cum.push_back(0.f);
        pts.push_back(p);
    }
    Vec3f at(float s) const
    {
        if (pts.empty()) return Vec3f::Zero();
        if (s <= 0.f) return pts.front();
        auto it = std::lower_bound(cum.begin(), cum.end(), s);
        if (it == cum.end()) return pts.back();
        const size_t i = size_t(it - cum.begin());
        if (i == 0) return pts.front();
        const float d = cum[i] - cum[i - 1];
        const float k = d > 1e-6f ? (s - cum[i - 1]) / d : 1.f;
        return pts[i - 1] + (pts[i] - pts[i - 1]) * k;
    }
};

struct Layer
{
    float              z0 = 0, z1 = 0;
    Path               outline;   // contours, traced
    Path               infill;    // hatch, zig-zag
    std::vector<Vec3f> cap;       // triangles over the layer's area
    float              traced = 0.f, filled = 0.f;   // how much of each is drawn (length)
    bool               done = false;
};

enum class PlanKind { Build, Move, Card, Sweep };

struct Plan
{
    int                id = 0;
    PlanKind           kind = PlanKind::Build;
    size_t             obj = 0;   // the object's ObjectID
    std::vector<Layer> layers;
    BoundingBoxf3      box, from;   // where it is (and was, for a move)
    float              move_k = 1.f;
    int                solid = -1;   // layers [0, solid] done: the real object shows up to there
    bool               finished = false;
    double             fade_at = 0.0;
    ImU32              fill = IM_COL32(0xE8, 0xA5, 0x5A, 255);
    std::string        text;          // a card's text
    float              typed = 0.f;   // a card's characters typed
    float              sweep = -1.f;  // slicing: the plane's height (0..1), or none
    int                seq = 0;
};

enum class TaskKind { Outline, Infill, Move, Type, Sweep, Reveal };

struct Task
{
    int              id = 0;
    int              plan = 0;
    TaskKind         kind = TaskKind::Outline;
    int              layer = 0;
    std::vector<int> deps;
    int              state = 0;   // 0 waiting, 1 being worked, 2 done
    std::string      tag;
};

struct Bot
{
    ImVec2      pos, from;
    ImU32       color;
    float       press = 0.f;
    int         task = -1;
    int         phase = 0;   // 0 flying to the work, 1 working, 2 a moment after
    float       t = 0.f, dur = 0.f, bend = 0.f;
    bool        leaving = false;
    float       idle = 0.f;
    std::string tag;
    double      tag_at = 0.0;
    bool        off = false;
    int         tool = 0;   // 0 none, 1 pen, 2 fill, 3 type
};

struct State
{
    bool                         on = false;
    ImU32                        accent = IM_COL32(0xF5, 0xB3, 0x35, 255);
    std::string                  label;
    std::vector<Bot>             bots;
    std::map<int, Plan>          plans;
    std::map<int, Task>          tasks;
    int                          next_id = 1;
    int                          seq = 0;
    bool                         seeded = false;
    std::map<size_t, BoundingBoxf3> seen;   // each object's box, last frame (by ObjectID)
    std::set<const GLVolume*>    off;       // volumes hidden while their object is being built or moved
    std::vector<std::string>     acts;      // settings and profiles to type in
    bool                         working = false;
    bool                         loose = false;
    double                       last_frame = 0.0, last_work = 0.0, cool = 0.0;
    unsigned                     seed = 1;
    std::map<size_t, std::pair<BoundingBoxf3, std::vector<Layer>>> slices;   // objects' layers, for slicing sweeps
    Eigen::Matrix4d              pv = Eigen::Matrix4d::Identity();
    // the view gliding to new work (and the view as Workbench left it: once the user moves it, it stays theirs)
    bool                         gliding = false;
    double                       glide_t0 = 0.0;
    Vec3d                        glide_from = Vec3d::Zero(), glide_to = Vec3d::Zero();
    double                       zoom_from = 1.0, zoom_to = 1.0;
    bool                         cam_noted = false, user_view = false;
    Vec3d                        cam_target = Vec3d::Zero(), cam_dir = Vec3d::Zero();
    double                       cam_zoom = 1.0;
    std::array<int, 4>           vp{0, 0, 0, 0};
    ImVec2                       disp{0, 0};
} S;

double now()
{
    using namespace std::chrono;
    return duration<double>(steady_clock::now().time_since_epoch()).count();
}

float rnd()
{
    S.seed = S.seed * 1664525u + 1013904223u;
    return float((S.seed >> 8) & 0xFFFF) / 65535.f;
}

float clampf(float v, float a, float b) { return std::max(a, std::min(b, v)); }
float ease_io(float t) { t = clampf(t, 0.f, 1.f); return t < 0.5f ? 4.f * t * t * t : 1.f - std::pow(-2.f * t + 2.f, 3.f) / 2.f; }
float ease_out(float t) { t = clampf(t, 0.f, 1.f); return 1.f - std::pow(1.f - t, 3.f); }
ImVec2 lerp(ImVec2 a, ImVec2 b, float k) { return ImVec2(a.x + (b.x - a.x) * k, a.y + (b.y - a.y) * k); }
ImU32 with_alpha(ImU32 c, float a) { return (c & 0x00FFFFFFu) | (ImU32(clampf(a, 0.f, 1.f) * 255.f) << 24); }

ImVec2 corner(size_t i)
{
    const ImVec2 d = ImGui::GetIO().DisplaySize;
    return ImVec2(d.x - 40.f - float(i % 2) * 18.f, d.y - 30.f - float(i / 2) * 16.f);
}

// the camera the 3D view draws with (the plater's; a canvas's own camera member stays unset)
const Camera& view_camera() { return wxGetApp().plater()->get_camera(); }

void see()
{
    const Camera& cam = view_camera();
    S.vp = cam.get_viewport();
    // as 4x4 matrices: the product of two affine transforms drops the projection's perspective row
    S.pv = cam.get_projection_matrix().matrix() * cam.get_view_matrix().matrix();
    S.disp = ImGui::GetIO().DisplaySize;
}

// a point in the scene on screen (ImGui coordinates); behind the camera: far off, in the way of it
ImVec2 xy(const Vec3d& p)
{
    if (S.vp[2] <= 0 || S.vp[3] <= 0) return ImVec2(-1e4f, -1e4f);
    const Eigen::Vector4d c = S.pv * Eigen::Vector4d(p.x(), p.y(), p.z(), 1.0);
    const float sx = S.disp.x / float(S.vp[2]), sy = S.disp.y / float(S.vp[3]);
    if (c.w() <= 1e-6) {
        const double n = std::hypot(c.x(), c.y());
        return n > 0 ? ImVec2(S.disp.x * 0.5f + float(c.x() / n) * S.disp.x * 3.f, S.disp.y * 0.5f - float(c.y() / n) * S.disp.y * 3.f)
                     : ImVec2(-1e4f, -1e4f);
    }
    return ImVec2(float((c.x() / c.w() * 0.5 + 0.5) * S.vp[2]) * sx, float((1.0 - (c.y() / c.w() * 0.5 + 0.5)) * S.vp[3]) * sy);
}
ImVec2 xy(const Vec3f& p) { return xy(Vec3d(p.cast<double>())); }

bool in_view(ImVec2 p, float pad = 0.f) { return p.x >= pad && p.y >= pad && p.x <= S.disp.x - pad && p.y <= S.disp.y - pad; }

// the user has moved the view themselves (told by the canvas's mouse handling; OrcaSlicer's own camera moves don't count)
bool user_moved() { return S.user_view; }

void note_view()
{
    Camera& cam = wxGetApp().plater()->get_camera();
    S.cam_noted = true;
    S.cam_target = cam.get_target();
    S.cam_zoom = cam.get_zoom();
    S.cam_dir = cam.get_dir_forward();
}

// new work out of view or small in it: the view glides to take in the plate's objects (unless the user moved it)
void frame(const BoundingBoxf3& all)
{
    if (!all.defined || S.gliding || user_moved()) return;
    ImVec2 a(1e9f, 1e9f), b(-1e9f, -1e9f);
    for (int i = 0; i < 8; ++i) {
        const ImVec2 p = xy(Vec3d((i & 1) ? all.max.x() : all.min.x(), (i & 2) ? all.max.y() : all.min.y(), (i & 4) ? all.max.z() : all.min.z()));
        a = ImVec2(std::min(a.x, p.x), std::min(a.y, p.y));
        b = ImVec2(std::max(b.x, p.x), std::max(b.y, p.y));
    }
    const bool inside = in_view(a) && in_view(b);
    const float span = std::max(b.x - a.x, b.y - a.y), room = std::min(S.disp.x, S.disp.y);
    if (inside && span >= 0.45f * room) { note_view(); return; }
    Camera& cam = wxGetApp().plater()->get_camera();
    if (cam.get_dir_forward().z() < -0.97) cam.select_view("iso");   // straight down, a part can't be seen rising
    Camera to = cam;
    to.zoom_to_box(all, 2.1);
    S.gliding = true;
    S.glide_t0 = now();
    S.glide_from = cam.get_target();
    S.zoom_from = cam.get_zoom();
    S.glide_to = to.get_target();
    S.zoom_to = to.get_zoom();
}

bool glide()
{
    if (!S.gliding) return false;
    Camera& cam = wxGetApp().plater()->get_camera();
    const float k = ease_io(float((now() - S.glide_t0) / 0.55));
    cam.set_target(S.glide_from + (S.glide_to - S.glide_from) * k);
    cam.set_zoom(S.zoom_from + (S.zoom_to - S.zoom_from) * k);
    if (k >= 1.f) {
        S.gliding = false;
        note_view();
    }
    return true;
}

void redraw()
{
    if (Plater* p = wxGetApp().plater())
        for (GLCanvas3D* c : {p->get_view3D_canvas3D(), p->get_preview_canvas3D()})
            if (c != nullptr) { c->set_as_dirty(); c->request_extra_frame(); }
}

const ModelObject* model_object(size_t id)
{
    for (const ModelObject* o : wxGetApp().model().objects)
        if (o->id().id == id) return o;
    return nullptr;
}

size_t object_id_of(const GLVolume* v)
{
    const ModelObjectPtrs& objs = wxGetApp().model().objects;
    const int i = v->object_idx();
    return i >= 0 && size_t(i) < objs.size() ? objs[i]->id().id : size_t(-1);
}

// ---------------------------------------------------------------- an object's layers

// the object (every instance) cut into layers from the bed up: each layer's outline, its infill (45 degrees, the next
// layer the other way, back and forth) and the triangles over its area
std::vector<Layer> layers_of(const ModelObject& o, BoundingBoxf3& box)
{
    std::vector<Layer> out;
    indexed_triangle_set all;
    for (const ModelInstance* inst : o.instances) {
        TriangleMesh m(o.raw_indexed_triangle_set());
        m.transform(inst->get_matrix(), true);
        its_merge(all, m.its);
    }
    if (all.indices.empty()) return out;
    box = BoundingBoxf3();
    for (const Vec3f& v : all.vertices) box.merge(v.cast<double>());
    const double h = box.max.z() - box.min.z();
    if (h <= 1e-3) return out;
    const int n = int(clampf(float(std::round(h / 2.5)), 5.f, 12.f));
    std::vector<float> zs;
    for (int i = 0; i < n; ++i) zs.push_back(float(box.min.z() + (i + 0.5) * h / n));
    std::vector<ExPolygons> cuts;
    try {
        cuts = slice_mesh_ex(all, zs);
    } catch (...) {
        return out;
    }
    const double diag = std::hypot(box.size().x(), box.size().y());
    const double spacing = std::max(diag / 16.0, 0.8);
    for (int i = 0; i < n; ++i) {
        Layer L;
        L.z0 = float(box.min.z() + i * h / n);
        L.z1 = float(box.min.z() + (i + 1) * h / n);
        ExPolygons ex = i < int(cuts.size()) ? expolygons_simplify(cuts[i], scale_(0.15)) : ExPolygons();
        const float z = L.z1;
        for (const ExPolygon& e : ex) {
            std::vector<const Polygon*> loops{&e.contour};
            for (const Polygon& hole : e.holes) loops.push_back(&hole);
            for (const Polygon* poly : loops) {
                if (poly->points.size() < 2) continue;
                bool first = true;
                for (const Point& p : poly->points) {
                    L.outline.add(Vec3f(float(unscale<double>(p.x())), float(unscale<double>(p.y())), z), !first);
                    first = false;
                }
                const Point& p0 = poly->points.front();
                L.outline.add(Vec3f(float(unscale<double>(p0.x())), float(unscale<double>(p0.y())), z), true);
            }
        }
        // infill: parallel lines across the layer, clipped to it, worked back and forth
        const double ang = (i % 2 == 0 ? 1.0 : -1.0) * M_PI / 4.0;
        const Vec2d dir(std::cos(ang), std::sin(ang)), nrm(-dir.y(), dir.x());
        const Vec2d c(box.center().x(), box.center().y());
        const double R = diag * 0.6 + 1.0;
        Polylines hatch;
        for (double off = -R; off <= R; off += spacing) {
            const Vec2d a = c + nrm * off - dir * R, b = c + nrm * off + dir * R;
            hatch.emplace_back(Polyline(Point(scale_(a.x()), scale_(a.y())), Point(scale_(b.x()), scale_(b.y()))));
        }
        Polylines in = ex.empty() ? Polylines() : intersection_pl(hatch, ex);
        std::vector<std::pair<double, Polyline>> rows;
        for (Polyline& pl : in) {
            if (pl.points.size() < 2) continue;
            const Vec2d m = (unscale(pl.points.front()) + unscale(pl.points.back())) * 0.5;
            rows.emplace_back((m - c).dot(nrm) * 1000.0 + (m - c).dot(dir), std::move(pl));
        }
        std::sort(rows.begin(), rows.end(), [](const auto& a, const auto& b) { return a.first < b.first; });
        bool fwd = true;
        for (auto& [key, pl] : rows) {
            Vec2d a = unscale(pl.points.front()), b = unscale(pl.points.back());
            if (((b - a).dot(dir) > 0) != fwd) std::swap(a, b);
            L.infill.add(Vec3f(float(a.x()), float(a.y()), z), false);
            L.infill.add(Vec3f(float(b.x()), float(b.y()), z), true);
            fwd = !fwd;
        }
        for (const Vec2f& p : triangulate_expolygons_2f(ex)) L.cap.emplace_back(p.x(), p.y(), z);
        out.push_back(std::move(L));
    }
    return out;
}

// ---------------------------------------------------------------- plans and tasks

int add_task(Plan& p, TaskKind kind, int layer, std::vector<int> deps, const std::string& tag = std::string())
{
    Task t;
    t.id = S.next_id++;
    t.plan = p.id;
    t.kind = kind;
    t.layer = layer;
    t.deps = std::move(deps);
    t.tag = tag;
    S.tasks[t.id] = t;
    return t.id;
}

Plan& add_plan(PlanKind kind)
{
    Plan p;
    p.id = S.next_id++;
    p.kind = kind;
    p.seq = ++S.seq;
    return S.plans[p.id] = std::move(p);
}

// a new (or reshaped) object, printed layer by layer
void plan_build(size_t obj, const BoundingBoxf3& box, const std::string& tag, ImU32 fill)
{
    const ModelObject* o = model_object(obj);
    if (o == nullptr) return;
    Plan& p = add_plan(PlanKind::Build);
    p.obj = obj;
    p.box = box;
    p.fill = fill;
    BoundingBoxf3 b;
    p.layers = layers_of(*o, b);
    std::vector<int> outlines, fills;
    for (int i = 0; i < int(p.layers.size()); ++i) {
        // two layers in hand at once: a layer's outline after the one two below it
        std::vector<int> d;
        if (i >= 2) d.push_back(outlines[i - 2]);
        // a layer's outline after the one below it; its infill once it's outlined (several layers fill at once)
        const int out = add_task(p, TaskKind::Outline, i, d, i == 0 ? tag : std::string());
        fills.push_back(add_task(p, TaskKind::Infill, i, {out}));
        outlines.push_back(out);
    }
    add_task(p, TaskKind::Reveal, 0, fills, p.layers.empty() ? tag : std::string());
}

// an object moved (arranged): a robot drags it from where it was to where it goes
void plan_move(size_t obj, const BoundingBoxf3& from, const BoundingBoxf3& to, const std::string& tag)
{
    Plan& p = add_plan(PlanKind::Move);
    p.obj = obj;
    p.from = from;
    p.box = to;
    p.move_k = 0.f;
    add_task(p, TaskKind::Move, 0, {}, tag.empty() ? std::string("arrange") : tag);
}

void plan_card(const std::string& text)
{
    Plan& p = add_plan(PlanKind::Card);
    p.text = text;
    add_task(p, TaskKind::Type, 0, {});
}

std::vector<int> ready()
{
    std::vector<int> out;
    for (auto& [id, t] : S.tasks) {
        if (t.state != 0) continue;
        bool ok = true;
        for (int d : t.deps) {
            auto it = S.tasks.find(d);
            if (it != S.tasks.end() && it->second.state != 2) { ok = false; break; }
        }
        if (ok) out.push_back(id);
    }
    return out;
}

// a card's place on screen: settings stack down the view's top-left
ImVec2 card_at(const Plan& p)
{
    int row = 0;
    for (const auto& [id, q] : S.plans)
        if (q.kind == PlanKind::Card && q.id < p.id && !q.finished) ++row;
    return ImVec2(24.f, 56.f + float(row) * 34.f);
}

ImVec2 box_center(const BoundingBoxf3& b) { return xy(b.center()); }

// where a task's work starts on screen
ImVec2 anchor(const Task& t)
{
    const Plan& p = S.plans[t.plan];
    switch (t.kind) {
    case TaskKind::Outline: return t.layer < int(p.layers.size()) && !p.layers[t.layer].outline.pts.empty() ? xy(p.layers[t.layer].outline.pts.front()) : box_center(p.box);
    case TaskKind::Infill: return t.layer < int(p.layers.size()) && !p.layers[t.layer].infill.pts.empty() ? xy(p.layers[t.layer].infill.pts.front()) : box_center(p.box);
    case TaskKind::Move: return box_center(p.from);
    case TaskKind::Type: { const ImVec2 c = card_at(p); return ImVec2(c.x + 8.f, c.y + 12.f); }
    case TaskKind::Sweep: return xy(Vec3d(p.box.max.x(), p.box.center().y(), p.box.min.z()));
    case TaskKind::Reveal: return box_center(p.box);
    }
    return box_center(p.box);
}

// how long a task's work takes (browser seconds, before the tempo), and its point at progress k
float work_time(const Task& t)
{
    const Plan& p = S.plans[t.plan];
    switch (t.kind) {
    case TaskKind::Outline: {
        if (t.layer >= int(p.layers.size())) return 0.2f;
        const Path& o = p.layers[t.layer].outline;
        float px = 0.f;
        for (size_t i = 1; i < o.pts.size(); ++i) { const ImVec2 a = xy(o.pts[i - 1]), b = xy(o.pts[i]); px += std::hypot(b.x - a.x, b.y - a.y); }
        return clampf(0.15f + std::sqrt(px) * 0.012f, 0.2f, 0.55f);
    }
    case TaskKind::Infill: return t.layer < int(p.layers.size()) ? clampf(0.12f + float(p.layers[t.layer].infill.pts.size()) * 0.012f, 0.2f, 0.6f) : 0.2f;
    case TaskKind::Move: return 1.0f;
    case TaskKind::Type: return clampf(0.2f + float(p.text.size()) * 0.035f, 0.4f, 1.4f);
    case TaskKind::Sweep: return 1.8f;
    case TaskKind::Reveal: return 0.12f;
    }
    return 0.3f;
}

ImVec2 work_point(Task& t, float k)
{
    Plan& p = S.plans[t.plan];
    switch (t.kind) {
    case TaskKind::Outline: {
        Layer& L = p.layers[t.layer];
        L.traced = L.outline.len() * k;
        return xy(L.outline.at(L.traced));
    }
    case TaskKind::Infill: {
        Layer& L = p.layers[t.layer];
        L.filled = L.infill.len() * k;
        return xy(L.infill.at(L.filled));
    }
    case TaskKind::Move: {
        p.move_k = ease_io(k);
        BoundingBoxf3 b(p.from.min + (p.box.min - p.from.min) * p.move_k, p.from.max + (p.box.max - p.from.max) * p.move_k);
        return box_center(b);
    }
    case TaskKind::Type: {
        p.typed = float(p.text.size()) * k;
        const ImVec2 c = card_at(p);
        const ImVec2 ts = ImGui::CalcTextSize(p.text.substr(0, size_t(p.typed)).c_str());
        return ImVec2(c.x + 10.f + ts.x, c.y + 14.f);
    }
    case TaskKind::Sweep: {
        p.sweep = ease_io(k);
        const double z = p.box.min.z() + (p.box.max.z() - p.box.min.z()) * p.sweep;
        return xy(Vec3d(p.box.max.x(), p.box.center().y(), z));
    }
    case TaskKind::Reveal: return box_center(p.box);
    }
    return box_center(p.box);
}

void finish_task(Task& t)
{
    t.state = 2;
    Plan& p = S.plans[t.plan];
    if (t.kind == TaskKind::Infill && t.layer < int(p.layers.size())) {
        Layer& L = p.layers[t.layer];
        L.done = true;
        L.filled = L.infill.len();
        while (p.solid + 1 < int(p.layers.size()) && p.layers[p.solid + 1].done) ++p.solid;
    }
    if (t.kind == TaskKind::Outline && t.layer < int(p.layers.size())) p.layers[t.layer].traced = p.layers[t.layer].outline.len();
    if (t.kind == TaskKind::Move) p.move_k = 1.f;
    if (t.kind == TaskKind::Type) p.typed = float(p.text.size());
    if (t.kind == TaskKind::Sweep) p.sweep = -1.f;
    bool all = true;
    for (const auto& [id, x] : S.tasks)
        if (x.plan == p.id && x.state != 2) { all = false; break; }
    if (all && !p.finished) {
        p.finished = true;
        p.fade_at = now();
    }
}

// the volumes a plan keeps out of sight shown again (a plan done, or the swarm off)
void show_object(GLCanvas3D& canvas, size_t obj)
{
    for (GLVolume* v : canvas.get_volumes().volumes)
        if (v != nullptr && S.off.count(v) && object_id_of(v) == obj) {
            v->is_active = true;
            S.off.erase(v);
        }
    canvas.set_as_dirty();
}

bool building(size_t obj)
{
    for (const auto& [id, p] : S.plans)
        if ((p.kind == PlanKind::Build || p.kind == PlanKind::Move) && p.obj == obj && !p.finished) return true;
    return false;
}

// ---------------------------------------------------------------- drawing

void robot(ImDrawList* dl, ImVec2 tip, ImU32 col, float press, int tool)
{
    const float s = 1.f - press * 0.18f;
    const ImVec2 a[7] = {{0, 0}, {0, 17}, {4.5f, 12.5f}, {8, 19}, {10.5f, 17.8f}, {7.2f, 11.4f}, {13, 11.4f}};
    ImVec2 p[7];
    for (int i = 0; i < 7; ++i) p[i] = ImVec2(tip.x + a[i].x * s, tip.y + a[i].y * s);
    const int tris[5][3] = {{0, 1, 2}, {0, 2, 6}, {2, 3, 4}, {2, 4, 5}, {2, 5, 6}};
    for (auto& t : tris) dl->AddTriangleFilled(p[t[0]], p[t[1]], p[t[2]], col);
    ImVec2 outline[8] = {p[0], p[1], p[2], p[3], p[4], p[5], p[6], p[0]};
    dl->AddPolyline(outline, 8, IM_COL32(255, 255, 255, 230), 0, 1.3f);
    const ImVec2 h(tip.x + 12.f * s, tip.y + 14.f * s);
    dl->AddRectFilled(ImVec2(h.x - 1, h.y - 1), ImVec2(h.x + 14, h.y + 11), IM_COL32(255, 255, 255, 230), 3.f);
    dl->AddRectFilled(h, ImVec2(h.x + 13, h.y + 10), col, 3.f);
    dl->AddRectFilled(ImVec2(h.x + 3, h.y + 3), ImVec2(h.x + 5, h.y + 6), IM_COL32(255, 255, 255, 255));
    dl->AddRectFilled(ImVec2(h.x + 8, h.y + 3), ImVec2(h.x + 10, h.y + 6), IM_COL32(255, 255, 255, 255));
    if (tool == 1) // a nozzle at the tip
        dl->AddTriangleFilled(ImVec2(tip.x - 1, tip.y - 1), ImVec2(tip.x - 9, tip.y - 6), ImVec2(tip.x - 6, tip.y - 9), IM_COL32(255, 255, 255, 240));
    else if (tool == 2) {
        dl->AddRectFilled(ImVec2(tip.x - 9, tip.y - 9), ImVec2(tip.x - 3, tip.y - 3), IM_COL32(255, 255, 255, 240));
        dl->AddRectFilled(ImVec2(tip.x - 8, tip.y - 8), ImVec2(tip.x - 4, tip.y - 4), col);
    }
}

void path_draw(ImDrawList* dl, const Path& path, float upto, ImU32 col, float w)
{
    for (size_t i = 1; i < path.pts.size(); ++i) {
        if (path.cum[i - 1] >= upto) break;
        if (!path.pen[i - 1]) continue;
        const ImVec2 a = xy(path.pts[i - 1]);
        const ImVec2 b = path.cum[i] <= upto ? xy(path.pts[i]) : xy(path.at(upto));
        dl->AddLine(a, b, col, w);
    }
}

void cap_draw(ImDrawList* dl, const Layer& L, ImU32 col)
{
    for (size_t i = 0; i + 2 < L.cap.size(); i += 3)
        dl->AddTriangleFilled(xy(L.cap[i]), xy(L.cap[i + 1]), xy(L.cap[i + 2]), col);
}

void box_draw(ImDrawList* dl, const BoundingBoxf3& b, ImU32 col)
{
    ImVec2 c[8];
    for (int i = 0; i < 8; ++i) c[i] = xy(Vec3d((i & 1) ? b.max.x() : b.min.x(), (i & 2) ? b.max.y() : b.min.y(), (i & 4) ? b.max.z() : b.min.z()));
    const int e[12][2] = {{0, 1}, {2, 3}, {4, 5}, {6, 7}, {0, 2}, {1, 3}, {4, 6}, {5, 7}, {0, 4}, {1, 5}, {2, 6}, {3, 7}};
    for (auto& x : e) dl->AddLine(c[x[0]], c[x[1]], col, 1.4f);
}

ImU32 shade(ImU32 c, float k)
{
    const float r = float(c & 255) * k, g = float((c >> 8) & 255) * k, b = float((c >> 16) & 255) * k;
    return IM_COL32(int(clampf(r, 0, 255)), int(clampf(g, 0, 255)), int(clampf(b, 0, 255)), int((c >> 24) & 255));
}

// ---------------------------------------------------------------- the robots at work

void step(GLCanvas3D& canvas, float dt)
{
    std::vector<int> avail = S.gliding ? std::vector<int>() : ready();
    size_t workers = 0, busy = 0;
    for (const Bot& b : S.bots) { if (!b.leaving) ++workers; if (b.task >= 0) ++busy; }
    const float tempo = clampf(2.1f + float(avail.size()) / float(std::max<size_t>(1, workers)) * 0.5f, 2.1f, 4.5f) * (S.on ? 1.f : 1.6f);
    // more hands while there's more work than robots
    S.cool -= dt;
    const size_t want = size_t(clampf(float(BASE + avail.size() / 2), float(BASE), float(MAX_BOTS)));
    if (workers < want && avail.size() > workers - busy && S.cool <= 0) {
        Bot b;
        const bool left = rnd() < 0.5f;
        b.pos = b.from = ImVec2(left ? -30.f : S.disp.x + 30.f, S.disp.y * (0.2f + 0.6f * rnd()));
        b.color = COLORS[(S.next_id++) % 4];
        S.bots.push_back(b);
        S.cool = 0.06;
    }
    // a free robot takes the nearest work (an earlier change's first)
    for (Bot& b : S.bots) {
        if (b.task >= 0 || b.leaving || avail.empty()) continue;
        size_t best = 0;
        float  best_s = 1e30f;
        for (size_t i = 0; i < avail.size(); ++i) {
            const Task& t = S.tasks[avail[i]];
            const ImVec2 a = anchor(t);
            const float s = float(S.plans[t.plan].seq) * 1500.f + std::hypot(a.x - b.pos.x, a.y - b.pos.y);
            if (s < best_s) { best_s = s; best = i; }
        }
        Task& t = S.tasks[avail[best]];
        avail.erase(avail.begin() + best);
        t.state = 1;
        b.task = t.id;
        b.phase = 0;
        b.t = 0.f;
        b.from = b.pos;
        const ImVec2 a = anchor(t);
        const float dist = std::hypot(a.x - b.pos.x, a.y - b.pos.y);
        b.dur = clampf(0.12f + std::sqrt(dist) * 0.021f, 0.14f, 0.9f);
        b.bend = (rnd() < 0.5f ? -1.f : 1.f) * (0.04f + 0.1f * rnd()) * std::min(dist, 600.f);
        b.off = !in_view(a);
        b.idle = 0.f;
        b.tool = (t.kind == TaskKind::Outline) ? 1 : (t.kind == TaskKind::Infill) ? 2 : (t.kind == TaskKind::Type) ? 3 : 0;
        b.tag = t.tag;   // a task with no tag clears the last
        b.tag_at = now();
    }
    // work
    const bool idle_all = busy == 0 && avail.empty();
    if (!idle_all) S.last_work = now();
    const bool rest = idle_all && now() - S.last_work > 1.6;
    size_t keep = 0;
    for (Bot& b : S.bots) {
        b.press = std::max(0.f, b.press - dt * 6.f);
        if (b.task >= 0) {
            auto it = S.tasks.find(b.task);
            if (it == S.tasks.end()) { b.task = -1; b.tool = 0; continue; }
            Task& t = it->second;
            const float d = dt * tempo * (b.off ? 3.f : 1.f);
            b.t += d;
            if (b.phase == 0) {   // flying to the work, along a curve, eased
                const ImVec2 p = anchor(t);
                const float k = std::min(1.f, b.t / b.dur);
                const float e = k < 0.15f ? ease_io(k) : ease_out(k);
                const float dx = p.x - b.from.x, dy = p.y - b.from.y, L = std::max(1.f, std::hypot(dx, dy));
                const float off = std::sin(float(M_PI) * e) * b.bend;
                b.pos = ImVec2(b.from.x + dx * e - dy / L * off, b.from.y + dy * e + dx / L * off);
                if (k >= 1.f) { b.phase = 1; b.t = 0.f; b.dur = work_time(t); b.press = 1.f; }
            } else if (b.phase == 1) {   // working: the piece appears behind it
                const float k = std::min(1.f, b.t / b.dur);
                b.pos = work_point(t, k);
                b.press = 1.f;
                if (k >= 1.f) {
                    finish_task(t);
                    b.phase = 2;
                    b.t = 0.f;
                    b.dur = 0.08f + 0.06f * rnd();
                }
            } else if (b.t >= b.dur) {
                b.task = -1;
                b.tool = 0;
                b.tag_at = now();
            }
            continue;
        }
        b.idle += dt;
        if (!b.leaving && ((!S.on && S.plans.empty()) || (b.idle > 2.5f && keep >= BASE))) b.leaving = true;
        if (!b.leaving) ++keep;
        ImVec2 to;
        if (b.leaving) to = ImVec2(S.disp.x + 80.f, b.pos.y + 40.f);
        else if (rest) to = corner(keep - 1);
        else continue;
        b.pos = lerp(b.pos, to, std::min(1.f, dt * 7.f));
    }
    S.bots.erase(std::remove_if(S.bots.begin(), S.bots.end(), [](const Bot& b) { return b.leaving && b.pos.x > S.disp.x + 40.f; }), S.bots.end());
    // plans done: their objects shown whole, their drawings fade
    const double t = now();
    for (auto it = S.plans.begin(); it != S.plans.end();) {
        Plan& p = it->second;
        if (p.finished && (p.kind == PlanKind::Build || p.kind == PlanKind::Move)) show_object(canvas, p.obj);
        const double linger = p.kind == PlanKind::Card ? 1.6 : 0.5;
        if (p.finished && t - p.fade_at > linger) {
            for (auto ti = S.tasks.begin(); ti != S.tasks.end();)
                ti = ti->second.plan == p.id ? S.tasks.erase(ti) : std::next(ti);
            it = S.plans.erase(it);
        } else
            ++it;
    }
}

} // namespace

void baseline(const Model& model)
{
    if (!workbench_docked()) return;
    std::map<size_t, BoundingBoxf3> boxes;
    for (const ModelObject* o : model.objects)
        for (size_t k = 0; k < o->instances.size(); ++k) boxes[o->id().id].merge(o->instance_bounding_box(k));
    S.seen = std::move(boxes);
    S.seeded = true;
    S.loose = true;
}

void user_camera()
{
    if (S.gliding) return;   // the glide's own frames
    S.user_view = true;
}

void opened(const Model& model)
{
    baseline(model);
    for (auto& [id, p] : S.plans)
        if ((p.kind == PlanKind::Build || p.kind == PlanKind::Move) && !p.finished) {
            for (auto& [tid, t] : S.tasks)
                if (t.plan == id) t.state = 2;
            p.finished = true;
            p.fade_at = now() - 10.0;
        }
    S.user_view = false;
    S.cam_noted = false;
    redraw();
}

void act(const std::string& text)
{
    if (!S.on) return;
    S.acts.push_back(text);
    redraw();
}

void working(bool on)
{
    S.working = on && S.on;
    redraw();
}

void set_on(bool on, const std::string& accent_hex)
{
    if (accent_hex.size() == 7 && accent_hex[0] == '#') {
        const unsigned v = std::stoul(accent_hex.substr(1), nullptr, 16);
        S.accent = IM_COL32((v >> 16) & 255, (v >> 8) & 255, v & 255, 255);
    }
    S.on = on;
    if (on) {
        size_t n = 0;
        for (const Bot& b : S.bots) if (!b.leaving) ++n;
        for (; n < BASE; ++n) {
            Bot b;
            b.pos = b.from = corner(S.bots.size());
            b.color = COLORS[S.bots.size() % 4];
            S.bots.push_back(b);
        }
    } else {
        S.acts.clear();
        S.working = false;   // what's being built finishes (faster), then the robots leave
    }
    redraw();
}

bool is_on() { return S.on; }

namespace { bool g_hands_off = false; }
void set_hands_off(bool off) { g_hands_off = off; }
bool hands_off() { return g_hands_off; }

void label(const std::string& text) { S.label = text; }

void before_render(GLCanvas3D& canvas, GLVolumeCollection& volumes)
{
    if (!workbench_docked()) return;   // the baseline is kept whenever docked: a change right after "swarm on" counts
    Plater* plater = wxGetApp().plater();
    const bool editor = plater != nullptr && &canvas == plater->get_view3D_canvas3D();   // the preview's volumes are toolpaths
    if (!editor) return;
    std::map<size_t, BoundingBoxf3> boxes;
    std::map<size_t, ImU32>         colors;
    for (const GLVolume* v : volumes.volumes) {
        if (v == nullptr || v->is_wipe_tower || v->is_modifier || v->object_idx() < 0) continue;
        const size_t id = object_id_of(v);
        if (id == size_t(-1)) continue;
        boxes[id].merge(v->transformed_bounding_box());
        if (!colors.count(id)) colors[id] = IM_COL32(int(v->color.r() * 255), int(v->color.g() * 255), int(v->color.b() * 255), 255);
    }
    see();
    if (S.on && S.seeded) {
        bool first = true;
        for (const auto& [obj, box] : boxes) {
            auto it = S.seen.find(obj);
            const bool fresh = it == S.seen.end();
            const double tol = S.loose ? 0.5 : 0.01;
            const bool changed = !fresh && ((it->second.min - box.min).norm() > tol || (it->second.max - box.max).norm() > tol);
            if (!fresh && !changed) continue;
            if (building(obj)) continue;   // still being drawn: it shows as it is now once its robots are done
            const std::string tag = first ? (S.label.empty() ? std::string("add") : S.label) : std::string();
            const bool same_size = !fresh && (it->second.size() - box.size()).norm() < std::max(0.05, tol);
            if (same_size) plan_move(obj, it->second, box, tag);   // arranged: dragged where it goes
            else plan_build(obj, box, tag, colors.count(obj) ? colors[obj] : S.accent);
            first = false;
        }
        if (!first) {
            S.label.clear();
            BoundingBoxf3 all;
            for (const auto& [obj, box] : boxes) all.merge(box);
            frame(all);
        }
    }
    S.seen = std::move(boxes);
    S.seeded = true;
    S.loose = false;
    if (S.on) {
        for (const std::string& text : S.acts) plan_card(text);
        S.acts.clear();
        // slicing: a plane sweeps up through each object, again and again until it's done
        if (S.working) {
            for (const auto& [obj, box] : S.seen) {
                bool sweeping = false;
                for (const auto& [id, p] : S.plans) if (p.kind == PlanKind::Sweep && p.obj == obj && !p.finished) sweeping = true;
                if (sweeping) continue;
                auto& cached = S.slices[obj];
                if (cached.second.empty() || (cached.first.min - box.min).norm() > 0.01 || (cached.first.max - box.max).norm() > 0.01) {
                    const ModelObject* o = model_object(obj);
                    BoundingBoxf3 b;
                    cached.second = o ? layers_of(*o, b) : std::vector<Layer>();
                    cached.first = box;
                }
                Plan& p = add_plan(PlanKind::Sweep);
                p.obj = obj;
                p.box = box;
                add_task(p, TaskKind::Sweep, 0, {}, S.label.empty() ? std::string("slice") : S.label);
                S.label.clear();
            }
        }
    }
    // what's being built or moved stays out of sight (the part built so far is drawn clipped, render_rising)
    {
        const std::set<const GLVolume*> now_there(volumes.volumes.begin(), volumes.volumes.end());
        for (auto it = S.off.begin(); it != S.off.end();) it = now_there.count(*it) ? std::next(it) : S.off.erase(it);
    }
    for (GLVolume* v : volumes.volumes) {
        if (v == nullptr || v->object_idx() < 0) continue;
        if (building(object_id_of(v))) {
            if (v->is_active) { v->is_active = false; S.off.insert(v); }
        }
    }
    if (!S.plans.empty() || !S.bots.empty()) canvas.request_extra_frame();
}

void render_rising(GLCanvas3D& canvas, GLVolumeCollection& volumes, const Transform3d& view, const Transform3d& projection)
{
    (void) canvas;
    if (S.plans.empty() || S.off.empty()) return;
    const std::array<float, 2> keep = volumes.get_z_range();
    for (const auto& [id, p] : S.plans) {
        if (p.kind != PlanKind::Build || p.finished || p.solid < 0 || p.solid >= int(p.layers.size())) continue;
        std::vector<GLVolume*> mine;
        for (GLVolume* v : volumes.volumes)
            if (v != nullptr && S.off.count(v) && !v->is_modifier && object_id_of(v) == p.obj) { v->is_active = true; mine.push_back(v); }
        if (mine.empty()) continue;
        volumes.set_z_range(-1e6f, p.layers[p.solid].z1);
        const size_t obj = p.obj;
        volumes.render(GLVolumeCollection::ERenderType::Opaque, false, view, projection,
                       [obj](const GLVolume& v) { return !v.is_modifier && object_id_of(&v) == obj; });
        for (GLVolume* v : mine) v->is_active = false;
    }
    volumes.set_z_range(keep[0], keep[1]);
}

void render(GLCanvas3D& canvas)
{
    if (!S.on && S.plans.empty() && S.bots.empty()) return;
    const double t = now();
    const float dt = S.last_frame > 0 ? clampf(float(t - S.last_frame), 0.f, 0.05f) : 0.016f;
    S.last_frame = t;
    if (glide()) { canvas.set_as_dirty(); see(); }
    else see();
    step(canvas, dt);
    ImDrawList* dl = ImGui::GetForegroundDrawList();
    const ImU32 acc = S.accent;
    for (auto& [id, p] : S.plans) {
        const float fade = p.finished ? 1.f - clampf(float((t - p.fade_at) / (p.kind == PlanKind::Card ? 1.6 : 0.5)), 0.f, 1.f) : 1.f;
        switch (p.kind) {
        case PlanKind::Build: {
            if (p.finished) break;
            // the top of what's built: its layer's area, with its infill; above it, layers traced and being filled
            if (p.solid >= 0) {
                const Layer& L = p.layers[p.solid];
                cap_draw(dl, L, with_alpha(shade(p.fill, 0.92f), 0.96f));
                path_draw(dl, L.infill, L.infill.len(), with_alpha(shade(p.fill, 0.7f), 0.9f), 1.2f);
                path_draw(dl, L.outline, L.outline.len(), with_alpha(shade(p.fill, 0.6f), 0.95f), 1.6f);
            }
            for (int i = p.solid + 1; i < int(p.layers.size()); ++i) {
                const Layer& L = p.layers[i];
                if (L.traced <= 0.f && L.filled <= 0.f) break;
                path_draw(dl, L.infill, L.filled, with_alpha(p.fill, 0.95f), 1.3f);
                path_draw(dl, L.outline, L.traced, with_alpha(acc, 0.95f), 1.8f);
            }
            break;
        }
        case PlanKind::Move: {
            if (p.finished && fade <= 0.f) break;
            BoundingBoxf3 b(p.from.min + (p.box.min - p.from.min) * p.move_k, p.from.max + (p.box.max - p.from.max) * p.move_k);
            if (p.move_k < 1.f) {   // where it was, and the way it goes
                box_draw(dl, p.from, with_alpha(acc, 0.25f));
                dl->AddLine(box_center(p.from), box_center(b), with_alpha(acc, 0.6f), 1.4f);
            }
            box_draw(dl, b, with_alpha(acc, 0.9f * fade));
            break;
        }
        case PlanKind::Card: {
            const ImVec2 c = card_at(p);
            const std::string shown = p.text.substr(0, size_t(std::min<float>(p.typed, float(p.text.size()))));
            const ImVec2 full = ImGui::CalcTextSize(p.text.c_str());
            dl->AddRectFilled(c, ImVec2(c.x + full.x + 22.f, c.y + 28.f), with_alpha(IM_COL32(24, 25, 28, 255), 0.92f * fade), 4.f);
            dl->AddRect(c, ImVec2(c.x + full.x + 22.f, c.y + 28.f), with_alpha(acc, 0.9f * fade), 4.f, 0, 1.2f);
            dl->AddText(ImVec2(c.x + 10.f, c.y + 6.f), with_alpha(IM_COL32(242, 241, 238, 255), fade), shown.c_str());
            if (!p.finished && std::fmod(t, 0.8) < 0.5) {   // the caret
                const ImVec2 ts = ImGui::CalcTextSize(shown.c_str());
                dl->AddLine(ImVec2(c.x + 11.f + ts.x, c.y + 6.f), ImVec2(c.x + 11.f + ts.x, c.y + 22.f), with_alpha(acc, fade), 1.5f);
            }
            break;
        }
        case PlanKind::Sweep: {
            if (p.sweep < 0.f) break;
            const auto& cached = S.slices[p.obj].second;
            const double z = p.box.min.z() + (p.box.max.z() - p.box.min.z()) * p.sweep;
            for (const Layer& L : cached)
                if (z >= L.z0 && z <= L.z1) {
                    path_draw(dl, L.outline, L.outline.len(), with_alpha(acc, 0.95f), 2.f);
                    cap_draw(dl, L, with_alpha(acc, 0.18f));
                    break;
                }
            break;
        }
        }
    }
    // robots, with what they hold and their task's tag; work out of view: at the view's edge, an arrow toward it
    const float pad = 30.f;
    for (Bot& b : S.bots) {
        ImVec2 p = b.pos;
        if (b.task >= 0 && !in_view(p, pad)) {
            float dx = p.x - S.disp.x * 0.5f, dy = p.y - S.disp.y * 0.5f;
            const float n = std::max(1e-3f, std::hypot(dx, dy));
            dx /= n; dy /= n;
            const float k = std::min(dx != 0.f ? (S.disp.x * 0.5f - pad) / std::abs(dx) : 1e9f, dy != 0.f ? (S.disp.y * 0.5f - pad) / std::abs(dy) : 1e9f);
            p = ImVec2(S.disp.x * 0.5f + dx * k, S.disp.y * 0.5f + dy * k);
            const ImVec2 c(p.x + dx * 6.f, p.y + dy * 6.f), sd(-dy * 8.f, dx * 8.f);
            dl->AddTriangleFilled(ImVec2(c.x + dx * 14.f, c.y + dy * 14.f), ImVec2(c.x + sd.x, c.y + sd.y), ImVec2(c.x - sd.x, c.y - sd.y), with_alpha(acc, 0.95f));
        }
        robot(dl, p, b.color, b.press, b.tool);
        if (!b.tag.empty()) {
            const float fade = b.task >= 0 ? 1.f : 1.f - float((t - b.tag_at) / 0.9);
            if (fade <= 0.f) { b.tag.clear(); continue; }
            const ImVec2 ts = ImGui::CalcTextSize(b.tag.c_str());
            const ImVec2 q(clampf(p.x + 28.f, 0.f, std::max(0.f, S.disp.x - ts.x - 10.f)), clampf(p.y + 18.f, 0.f, std::max(0.f, S.disp.y - ts.y - 6.f)));
            dl->AddRectFilled(q, ImVec2(q.x + ts.x + 10.f, q.y + ts.y + 6.f), with_alpha(acc, 0.95f * fade));
            dl->AddText(ImVec2(q.x + 5.f, q.y + 3.f), with_alpha(IM_COL32(255, 255, 255, 255), fade), b.tag.c_str());
        }
    }
    if (!S.plans.empty() || !S.bots.empty() || S.working) canvas.request_extra_frame();
}

} // namespace WorkbenchSwarm
} // namespace GUI
} // namespace Slic3r
