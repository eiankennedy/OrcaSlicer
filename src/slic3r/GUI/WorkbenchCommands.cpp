// Workbench's commands to a docked OrcaSlicer (WorkbenchDock.hpp): what Claude, through Workbench, can do in the
// running app, so the user watches it happen. One JSON object in, the result as JSON text out; errors throw.
//   state                         the plate: objects (name, index, box), the presets, the tab, hands off, swarm
//   new                           a new project (no questions asked)
//   load {paths: [...]}           models (STL, 3MF, STEP, OBJ...) onto the plate
//   arrange                       arrange the plate
//   slice                         slice the plate (shows the preview)
//   view {name: "3d"|"preview"}   the 3D editor or the sliced preview
//   preset {type: "printer"|"filament"|"print", name}
//   save {path}                   the project as a .3mf
//   hands {off}                   only the 3D view (no sidebar, no tab bar, no toolbars) / everything back
//   swarm {on, accent?}           Workbench's build swarm in the 3D view
//   label {text}                  the command the next robot's tag shows

#include "WorkbenchCommands.hpp"

#include "GLCanvas3D.hpp"
#include "GUI_App.hpp"
#include "MainFrame.hpp"
#include "Plater.hpp"
#include "Tab.hpp"
#include "Notebook.hpp"
#include "NotificationManager.hpp"
#include "WorkbenchSwarm.hpp"
#include "libslic3r/Model.hpp"
#include "libslic3r/PresetBundle.hpp"

#include <boost/filesystem.hpp>
#include <nlohmann/json.hpp>

#include <stdexcept>

namespace Slic3r {
namespace GUI {

using json = nlohmann::json;

namespace {

bool hands_off = false;

json state()
{
    Plater* p = wxGetApp().plater();
    json objs = json::array();
    const Model& model = p->model();
    for (size_t i = 0; i < model.objects.size(); ++i) {
        const ModelObject* o = model.objects[i];
        if (o->instances.empty()) continue;
        const BoundingBoxf3 b = o->instance_bounding_box(size_t(0));
        objs.push_back({{"index", i}, {"name", o->name}, {"instances", o->instances.size()},
                        {"size", {b.size().x(), b.size().y(), b.size().z()}}});
    }
    PresetBundle* pb = wxGetApp().preset_bundle;
    json filaments = json::array();
    for (const std::string& f : pb->filament_presets) filaments.push_back(f);
    return {{"objects", objs},
            {"printer", pb->printers.get_selected_preset_name()},
            {"print", pb->prints.get_selected_preset_name()},
            {"filaments", filaments},
            {"hands_off", hands_off},
            {"swarm", WorkbenchSwarm::is_on()},
            {"project", std::string(p->get_project_filename(".3mf").ToUTF8().data())}};
}

void set_hands(bool off)
{
    hands_off = off;
    WorkbenchSwarm::set_hands_off(off);
    Plater* p = wxGetApp().plater();
    if (off) p->get_notification_manager()->bbl_close_objectsinfo_notification();
    p->collapse_sidebar(off);
    if (MainFrame* mf = wxGetApp().mainframe; mf != nullptr && mf->m_tabpanel != nullptr) {
        if (wxWindow* bar = mf->m_tabpanel->GetBtnsListCtrl()) {
            bar->Show(!off);
            mf->m_tabpanel->Layout();
        }
    }
    for (GLCanvas3D* c : {p->get_view3D_canvas3D(), p->canvas3D()}) {
        if (c == nullptr) continue;
        c->enable_main_toolbar(!off);
        c->enable_assemble_view_toolbar(!off);
        c->enable_separator_toolbar(!off);
        c->enable_collapse_toolbar(!off);
        c->enable_return_toolbar(!off);
        c->set_as_dirty();
        c->request_extra_frame();
    }
    if (MainFrame* mf = wxGetApp().mainframe) mf->Layout();
}

} // namespace

std::string workbench_command(const std::string& request)
{
    const json req = json::parse(request);
    const std::string cmd = req.value("cmd", "");
    Plater* p = wxGetApp().plater();
    if (p == nullptr) throw std::runtime_error("OrcaSlicer isn't ready yet");
    auto labelled = [&](const char* text) { WorkbenchSwarm::label(text); };
    json out;
    if (cmd == "state") {
        out = state();
    } else if (cmd == "new") {
        labelled("new");
        p->new_project(true, true);
        out = state();
    } else if (cmd == "load") {
        std::vector<boost::filesystem::path> paths;
        for (const auto& s : req.at("paths")) paths.emplace_back(s.get<std::string>());
        if (paths.empty()) throw std::runtime_error("load: no paths");
        labelled("load");
        const std::vector<size_t> added = p->load_files(paths, LoadStrategy::LoadModel);
        out = {{"added", added}, {"state", state()}};
    } else if (cmd == "arrange") {
        labelled("arrange");
        p->arrange();
        out = state();
    } else if (cmd == "slice") {
        labelled("slice");
        p->select_view_3D("Preview", false);
        p->reslice();
        out = {{"slicing", true}};
    } else if (cmd == "view") {
        const std::string name = req.value("name", "3d");
        p->select_view_3D(name == "preview" ? "Preview" : "3D");
        out = {{"view", name}};
    } else if (cmd == "preset") {
        const std::string type = req.value("type", "");
        const Preset::Type t = type == "printer" ? Preset::TYPE_PRINTER : type == "filament" ? Preset::TYPE_FILAMENT : type == "print" ? Preset::TYPE_PRINT : Preset::TYPE_INVALID;
        if (t == Preset::TYPE_INVALID) throw std::runtime_error("preset: type is printer, filament or print");
        Tab* tab = wxGetApp().get_tab(t);
        if (tab == nullptr || !tab->select_preset(req.value("name", ""))) throw std::runtime_error("preset: no preset by that name");
        out = state();
    } else if (cmd == "save") {
        const std::string path = req.value("path", "");
        if (path.empty()) throw std::runtime_error("save: path needed");
        if (p->export_3mf(boost::filesystem::path(path)) < 0) throw std::runtime_error("save: couldn't write the project");
        out = {{"saved", path}};
    } else if (cmd == "hands") {
        set_hands(req.value("off", true));
        out = {{"hands_off", hands_off}};
    } else if (cmd == "swarm") {
        WorkbenchSwarm::set_on(req.value("on", true), req.value("accent", std::string()));
        for (GLCanvas3D* c : {p->get_view3D_canvas3D(), p->canvas3D()})
            if (c != nullptr) { c->set_as_dirty(); c->request_extra_frame(); }
        out = {{"swarm", WorkbenchSwarm::is_on()}};
    } else if (cmd == "label") {
        WorkbenchSwarm::label(req.value("text", std::string()));
        out = true;
    } else {
        throw std::runtime_error("unknown command: " + cmd);
    }
    return out.dump();
}

void workbench_set_hands(bool off) { set_hands(off); }

} // namespace GUI
} // namespace Slic3r
