#include "WorkbenchSwarm.hpp"

#include "3DScene.hpp"
#include "Camera.hpp"
#include "GLCanvas3D.hpp"
#include "WorkbenchDock.hpp"

#include <imgui/imgui.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <map>
#include <set>
#include <vector>

namespace Slic3r {
namespace GUI {
namespace WorkbenchSwarm {

namespace {

constexpr float FLY = 0.45f, DRAG = 0.55f, AFTER = 0.9f;
constexpr size_t BASE = 4;
const ImU32 COLORS[4] = {IM_COL32(0xD9, 0x77, 0x57, 255), IM_COL32(0xE8, 0xA5, 0x5A, 255), IM_COL32(0xC9, 0x64, 0x42, 255),
                         IM_COL32(0xF0, 0xB0, 0x7A, 255)};

struct Bot
{
    ImVec2 pos;
    ImVec2 from;
    double free_at = 0.0;
    bool   leaving = false;
    float  press   = 0.f;
    int    gesture = -1; // the gesture it is flying to / dragging, or -1
    ImU32  color;
};

struct Gesture
{
    int         id;
    size_t      bot;
    ImVec4      rect; // x0, y0, x1, y1 (ImGui coords)
    double      start;
    std::string text;
    int         object = -1; // a new object, hidden until drawn
    bool        click  = false;
};

struct State
{
    bool                        on = false;
    ImU32                       accent = IM_COL32(0xF5, 0xB3, 0x35, 255);
    std::string                 label;
    std::vector<Bot>            bots;
    std::vector<Gesture>        gestures;
    int                         next_id  = 0;
    double                      last_hit = 0.0;
    bool                        seeded   = false;
    std::map<int, BoundingBoxf3> seen;   // each object's box, last frame
    std::set<int>               hidden;  // new objects, waiting for their robot
} S;

double now()
{
    using namespace std::chrono;
    return duration<double>(steady_clock::now().time_since_epoch()).count();
}

ImVec2 corner(size_t i)
{
    const ImVec2 d = ImGui::GetIO().DisplaySize;
    return ImVec2(d.x - 40.f - float(i % 2) * 18.f, d.y - 30.f - float(i / 2) * 16.f);
}

float ease(float t)
{
    t = std::clamp(t, 0.f, 1.f);
    return t * t * (3.f - 2.f * t);
}

ImVec2 lerp(ImVec2 a, ImVec2 b, float k) { return ImVec2(a.x + (b.x - a.x) * k, a.y + (b.y - a.y) * k); }

// an object's box on screen, in ImGui coordinates (or nothing when it is off screen or behind the camera)
bool screen_rect(const Camera& cam, const BoundingBoxf3& box, ImVec4& out)
{
    const std::array<int, 4>& vp = cam.get_viewport();
    if (vp[2] <= 0 || vp[3] <= 0) return false;
    const Eigen::Matrix4d pv = (cam.get_projection_matrix() * cam.get_view_matrix()).matrix();
    const ImVec2 disp = ImGui::GetIO().DisplaySize;
    const float sx = disp.x / float(vp[2]), sy = disp.y / float(vp[3]);
    float x0 = 1e9f, y0 = 1e9f, x1 = -1e9f, y1 = -1e9f;
    int in_front = 0;
    for (int i = 0; i < 8; ++i) {
        const Vec3d p((i & 1) ? box.max.x() : box.min.x(), (i & 2) ? box.max.y() : box.min.y(), (i & 4) ? box.max.z() : box.min.z());
        const Eigen::Vector4d c = pv * Eigen::Vector4d(p.x(), p.y(), p.z(), 1.0);
        if (c.w() <= 1e-6) continue;
        ++in_front;
        const double nx = c.x() / c.w(), ny = c.y() / c.w();
        const float px = float((nx * 0.5 + 0.5) * vp[2]) * sx;
        const float py = float((1.0 - (ny * 0.5 + 0.5)) * vp[3]) * sy;
        x0 = std::min(x0, px); y0 = std::min(y0, py); x1 = std::max(x1, px); y1 = std::max(y1, py);
    }
    if (in_front == 0) return false;
    x0 = std::max(0.f, x0); y0 = std::max(0.f, y0);
    x1 = std::min(disp.x, x1); y1 = std::min(disp.y, y1);
    if (x1 - x0 < 3.f || y1 - y0 < 3.f) return false;
    out = ImVec4(x0, y0, x1, y1);
    return true;
}

void target(const ImVec4& r, int object, const std::string& text)
{
    const double t = now();
    const ImVec2 c((r.x + r.z) * 0.5f, (r.y + r.w) * 0.5f);
    size_t best = SIZE_MAX;
    float  dist = 1e9f;
    for (size_t i = 0; i < S.bots.size(); ++i) {
        const Bot& b = S.bots[i];
        if (b.leaving || b.free_at > t) continue;
        const float d = std::hypot(b.pos.x - c.x, b.pos.y - c.y);
        if (d < dist) { dist = d; best = i; }
    }
    if (best == SIZE_MAX) { // every robot busy: another joins
        Bot b;
        b.pos = b.from = corner(S.bots.size());
        b.color = COLORS[S.bots.size() % 4];
        S.bots.push_back(b);
        best = S.bots.size() - 1;
    }
    Bot& bot = S.bots[best];
    const double start = std::max(t, bot.free_at);
    bot.free_at = start + FLY + DRAG + 0.05;
    bot.leaving = false;
    Gesture g;
    g.id = S.next_id++;
    g.bot = best;
    g.rect = r;
    g.start = start;
    g.text = text;
    g.object = object;
    g.click = (r.z - r.x) < 40.f && (r.w - r.y) < 30.f;
    S.gestures.push_back(g);
    S.last_hit = std::max(S.last_hit, start + FLY + DRAG);
}

void robot(ImDrawList* dl, ImVec2 tip, ImU32 col, float press)
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
}

ImU32 with_alpha(ImU32 c, float a)
{
    return (c & 0x00FFFFFFu) | (ImU32(std::clamp(a, 0.f, 1.f) * 255.f) << 24);
}

} // namespace

void set_on(bool on, const std::string& accent_hex)
{
    if (accent_hex.size() == 7 && accent_hex[0] == '#') {
        const unsigned v = std::stoul(accent_hex.substr(1), nullptr, 16);
        S.accent = IM_COL32((v >> 16) & 255, (v >> 8) & 255, v & 255, 255);
    }
    S.on = on;
    if (on) {
        while (S.bots.size() < BASE) {
            Bot b;
            b.pos = b.from = corner(S.bots.size());
            b.color = COLORS[S.bots.size() % 4];
            S.bots.push_back(b);
        }
    } else {
        S.gestures.clear();
        S.hidden.clear();
    }
}

bool is_on() { return S.on; }

namespace { bool g_hands_off = false; }
void set_hands_off(bool off) { g_hands_off = off; }
bool hands_off() { return g_hands_off; }

void label(const std::string& text) { S.label = text; }

void before_render(GLCanvas3D& canvas, GLVolumeCollection& volumes)
{
    if (!workbench_docked()) return;   // the baseline is kept whenever docked: a change right after "swarm on" counts
    std::map<int, BoundingBoxf3> boxes;
    for (const GLVolume* v : volumes.volumes) {
        if (v == nullptr || v->is_wipe_tower || v->is_modifier || v->object_idx() < 0 || v->object_idx() >= 1000) continue;
        boxes[v->object_idx()].merge(v->transformed_bounding_box());
    }
    if (S.on && S.seeded) {
        bool first = true;
        for (const auto& [obj, box] : boxes) {
            auto it = S.seen.find(obj);
            const bool fresh = it == S.seen.end();
            const bool moved = !fresh && ((it->second.min - box.min).norm() > 0.01 || (it->second.max - box.max).norm() > 0.01);
            if (!fresh && !moved) continue;
            ImVec4 r;
            if (!screen_rect(canvas.get_camera(), box, r)) continue;
            if (fresh) S.hidden.insert(obj);
            target(r, fresh ? obj : -1, first ? S.label : std::string());
            first = false;
        }
    }
    S.seen = std::move(boxes);
    S.seeded = true;
    // a new object waits for its robot (shown as soon as the robot has drawn it out)
    for (GLVolume* v : volumes.volumes)
        if (v != nullptr && S.hidden.count(v->object_idx())) v->visible = false;
}

void render(GLCanvas3D& canvas)
{
    if (!S.on && S.gestures.empty() && S.bots.empty()) return;
    const double t = now();
    ImDrawList*  dl = ImGui::GetForegroundDrawList();
    bool         busy = false;
    // gestures: the marquee drawn out from corner to corner, the tag with the command
    for (auto it = S.gestures.begin(); it != S.gestures.end();) {
        const Gesture& g = *it;
        const float    k = ease(float((t - g.start - FLY) / DRAG));
        const float    fade = 1.f - std::clamp(float((t - g.start - FLY - DRAG) / AFTER), 0.f, 1.f);
        if (t >= g.start + FLY + DRAG && g.object >= 0 && S.hidden.erase(g.object)) {
            for (GLVolume* v : canvas.get_volumes().volumes)
                if (v != nullptr && v->object_idx() == g.object) const_cast<GLVolume*>(v)->visible = true;
            canvas.set_as_dirty();
        }
        if (t > g.start + FLY + DRAG + AFTER) { it = S.gestures.erase(it); continue; }
        busy = true;
        if (t >= g.start + FLY) {
            if (g.click) {
                const ImVec2 c((g.rect.x + g.rect.z) * 0.5f, (g.rect.y + g.rect.w) * 0.5f);
                dl->AddCircle(c, 6.f + 16.f * k, with_alpha(S.accent, 0.8f * (1.f - k)), 24, 1.5f);
            } else {
                const ImVec2 a(g.rect.x, g.rect.y), e(g.rect.x + (g.rect.z - g.rect.x) * k, g.rect.y + (g.rect.w - g.rect.y) * k);
                dl->AddRectFilled(a, e, with_alpha(S.accent, 0.10f * fade));
                dl->AddRect(a, e, with_alpha(S.accent, 0.9f * fade), 0.f, 0, 1.2f);
            }
            if (!g.text.empty()) {
                const ImVec2 ts = ImGui::CalcTextSize(g.text.c_str());
                const ImVec2 p(g.rect.x, g.rect.y - ts.y - 8.f);
                dl->AddRectFilled(p, ImVec2(p.x + ts.x + 10.f, p.y + ts.y + 6.f), with_alpha(S.accent, 0.95f * fade));
                dl->AddText(ImVec2(p.x + 5.f, p.y + 3.f), with_alpha(IM_COL32(255, 255, 255, 255), fade), g.text.c_str());
            }
        }
        ++it;
    }
    // robots: fly to their gesture, drag across it; idle ones huddle; off, they leave
    const bool idle = t - S.last_hit > 1.6;
    bool       moving = false;
    for (size_t i = 0; i < S.bots.size(); ++i) {
        Bot&           b = S.bots[i];
        const Gesture* g = nullptr;
        for (const Gesture& x : S.gestures)
            if (x.bot == i && t < x.start + FLY + DRAG + 0.05 && t >= x.start) { g = &x; break; }
        if (g != nullptr) {
            const ImVec2 start = g->click ? ImVec2((g->rect.x + g->rect.z) * 0.5f, (g->rect.y + g->rect.w) * 0.5f) : ImVec2(g->rect.x, g->rect.y);
            if (b.gesture != g->id) { b.gesture = g->id; b.from = b.pos; }
            if (t < g->start + FLY) {
                b.pos = lerp(b.from, start, ease(float((t - g->start) / FLY)));
                b.press = 0.f;
            } else {
                const ImVec2 end = g->click ? start : ImVec2(g->rect.z, g->rect.w);
                b.pos = lerp(start, end, ease(float((t - g->start - FLY) / DRAG)));
                b.press = 1.f;
            }
            moving = true;
        } else {
            b.gesture = -1;
            b.press = std::max(0.f, b.press - 0.1f);
            ImVec2 to;
            if (!S.on || (idle && i >= BASE)) { b.leaving = true; to = ImVec2(ImGui::GetIO().DisplaySize.x + 60.f, ImGui::GetIO().DisplaySize.y + 60.f); }
            else if (idle) to = corner(i);
            else to = b.pos;
            const ImVec2 n = lerp(b.pos, to, 0.12f);
            if (std::hypot(n.x - b.pos.x, n.y - b.pos.y) > 0.3f) moving = true;
            b.pos = n;
        }
        robot(dl, b.pos, b.color, b.press);
    }
    const ImVec2 d = ImGui::GetIO().DisplaySize;
    S.bots.erase(std::remove_if(S.bots.begin(), S.bots.end(), [&](const Bot& b) { return b.leaving && (b.pos.x > d.x + 40.f || b.pos.y > d.y + 40.f); }),
                 S.bots.end());
    if (busy || moving || !S.hidden.empty()) canvas.request_extra_frame();
}

} // namespace WorkbenchSwarm
} // namespace GUI
} // namespace Slic3r
