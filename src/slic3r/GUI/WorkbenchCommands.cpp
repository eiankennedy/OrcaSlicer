// Workbench's commands to a docked OrcaSlicer (WorkbenchDock.hpp): what Claude, through Workbench, can do in the
// running app, so the user watches it happen. One JSON object in, the result as JSON text out; errors throw.
//   state                         the plate: objects (name, index, box), the presets, the tab, hands off, swarm
//   new                           a new project (no questions asked)
//   load {paths: [...]}           models (STL, 3MF, STEP, OBJ...) onto the plate
//   arrange                       arrange the plate
//   slice                         slice the plate (shows the preview); answers once sliced: print time, filament
//                                 length and weight, warnings
//   view {name: "3d"|"preview"}   the 3D editor or the sliced preview
//   preset {type: "printer"|"filament"|"print", name}   a whole profile (unsaved tweaks to the old one dropped)
//   set {settings: {key: value, ...}}   single settings of the print / filament / printer profile in use
//                                 (e.g. sparse_infill_density: "15%", layer_height: 0.2, wall_loops: 3,
//                                 enable_support: true); answers with each value as applied
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

#include <boost/algorithm/string/predicate.hpp>
#include "WorkbenchDock.hpp"
#include "PartPlate.hpp"
#include "libslic3r/GCode/GCodeProcessor.hpp"
#include "libslic3r/Print.hpp"
#include "libslic3r/Model.hpp"
#include "libslic3r/PresetBundle.hpp"

#include <boost/filesystem.hpp>
#include <nlohmann/json.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <map>
#include <stdexcept>

namespace Slic3r {
namespace GUI {

using json = nlohmann::json;

namespace {

bool hands_off = false;

// a setting's preset type (print, filament, printer), or TYPE_INVALID
Preset::Type setting_type(const std::string& key)
{
    for (Preset::Type t : {Preset::TYPE_PRINT, Preset::TYPE_FILAMENT, Preset::TYPE_PRINTER}) {
        const std::vector<std::string>& keys = t == Preset::TYPE_PRINT ? Preset::print_options()
                                             : t == Preset::TYPE_FILAMENT ? Preset::filament_options() : Preset::printer_options();
        if (std::find(keys.begin(), keys.end(), key) != keys.end()) return t;
    }
    return Preset::TYPE_INVALID;
}

PresetCollection& presets_of(Preset::Type t)
{
    PresetBundle* pb = wxGetApp().preset_bundle;
    return t == Preset::TYPE_PRINT ? pb->prints : t == Preset::TYPE_FILAMENT ? pb->filaments : pb->printers;
}

// a few everyday names for OrcaSlicer's keys
std::string setting_key(const std::string& name)
{
    static const std::map<std::string, std::string> alias = {
        {"supports", "enable_support"}, {"support", "enable_support"}, {"infill", "sparse_infill_density"},
        {"infill_density", "sparse_infill_density"}, {"infill_pattern", "sparse_infill_pattern"}, {"walls", "wall_loops"},
        {"perimeters", "wall_loops"}, {"brim", "brim_type"}};
    auto it = alias.find(name);
    return it == alias.end() ? name : it->second;
}

std::string setting_value(const json& v)
{
    if (v.is_string()) return v.get<std::string>();
    if (v.is_boolean()) return v.get<bool>() ? "1" : "0";
    return v.dump();
}

// the slicing result of the plate shown, once slicing has finished
bool slice_done(std::string& result, std::string& error, double started)
{
    Plater* p = wxGetApp().plater();
    if (p == nullptr) { error = "OrcaSlicer closed"; return true; }
    using clock = std::chrono::steady_clock;
    const double t = std::chrono::duration<double>(clock::now().time_since_epoch()).count();
    if (p->is_background_process_slicing()) {
        if (t - started > 1800) { error = "slicing took more than 30 minutes"; WorkbenchSwarm::working(false); return true; }
        return false;
    }
    WorkbenchSwarm::working(false);
    PartPlate* plate = p->get_partplate_list().get_curr_plate();
    const std::string failed = workbench_slice_error();
    if (!failed.empty()) { error = "slicing failed: " + failed; return true; }
    if (plate == nullptr || !plate->is_slice_result_valid()) { error = "the plate didn't slice (is anything on it?)"; return true; }
    json out = {{"sliced", true}, {"plate", p->get_partplate_list().get_curr_plate_index() + 1}};
    json warnings = json::array();
    if (const GCodeProcessorResult* r = plate->get_slice_result()) {
        const double secs = r->print_statistics.modes[0].time;
        out["print_time_s"] = std::round(secs);
        out["print_time"] = get_time_dhms(float(secs));
        // filament from the volume extruded (model, supports, wipe tower), as OrcaSlicer's own legend counts it
        double mm = 0, g = 0;
        for (const auto& [extruder, volume] : r->print_statistics.total_volumes_per_extruder) {
            const double d = extruder < r->filament_diameters.size() ? r->filament_diameters[extruder] : 1.75;
            const double rho = extruder < r->filament_densities.size() ? r->filament_densities[extruder] : 1.24;
            mm += volume / (M_PI * (0.5 * d) * (0.5 * d));
            g += volume * rho * 0.001;
        }
        out["filament_m"] = std::round(mm / 10) / 100;
        out["filament_g"] = std::round(g * 100) / 100;
        for (const auto& w : r->warnings) {
            std::string m = w.msg;
            for (const std::string& prm : w.params) m += " " + prm;
            warnings.push_back(m);
        }
    }
    out["warnings"] = warnings;
    result = out.dump();
    return true;
}

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
    if (cmd == "load" || cmd == "arrange" || cmd == "new")
        WorkbenchSwarm::baseline(p->model());   // what changes from here gets a robot
    json out;
    if (cmd == "state") {
        out = state();
    } else if (cmd == "new") {
        labelled("new");
        p->new_project(true, true);
        WorkbenchSwarm::opened(p->model());
        out = state();
    } else if (cmd == "load") {
        std::vector<boost::filesystem::path> paths;
        for (const auto& s : req.at("paths")) paths.emplace_back(s.get<std::string>());
        if (paths.empty()) throw std::runtime_error("load: no paths");
        labelled("load");
        const std::vector<size_t> added = p->load_files(paths, LoadStrategy::LoadModel);
        // a plate opened (a project file) is just there; only what's added to it after is built
        bool plate = false;
        for (const auto& path : paths) plate = plate || boost::iequals(path.extension().string(), ".3mf");
        if (plate) WorkbenchSwarm::opened(p->model());
        out = {{"added", added}, {"state", state()}};
    } else if (cmd == "arrange") {
        labelled("arrange");
        p->arrange();
        out = state();
    } else if (cmd == "slice") {
        labelled("slice");
        workbench_slice_error_clear();
        p->select_view_3D("Preview", false);
        WorkbenchSwarm::working(true);   // robots over the plate until it's sliced
        p->reslice();
        const double started = std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
        workbench_reply_later([started](std::string& result, std::string& error) { return slice_done(result, error, started); });
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
        if (tab == nullptr) throw std::runtime_error("preset: OrcaSlicer isn't ready yet");
        // a whole profile: unsaved tweaks to the one in use go (OrcaSlicer would ask, with no one to answer)
        bool discarded = false;
        for (Preset::Type d : {t, Preset::TYPE_PRINT, Preset::TYPE_FILAMENT}) {
            if (d != t && t != Preset::TYPE_PRINTER) continue;
            PresetCollection& c = presets_of(d);
            if (c.current_is_dirty()) { c.discard_current_changes(); discarded = true; }
        }
        WorkbenchSwarm::act(std::string(type) + ": " + req.value("name", ""));
        if (!tab->select_preset(req.value("name", ""), false, "", false, true)) throw std::runtime_error("preset: no preset by that name");
        out = state();
        out["discarded_changes"] = discarded;
    } else if (cmd == "set" || cmd == "setting" || cmd == "settings") {
        json settings = req.contains("settings") ? req["settings"] : json::object();
        if (req.contains("key")) settings[req.value("key", "")] = req.contains("value") ? req["value"] : json("");
        if (!settings.is_object() || settings.empty()) throw std::runtime_error("set: settings {key: value, ...} needed");
        std::map<Preset::Type, std::vector<std::pair<std::string, std::string>>> by_type;
        std::vector<std::pair<std::string, Preset::Type>> keys;
        for (auto it = settings.begin(); it != settings.end(); ++it) {
            const std::string key = setting_key(it.key());
            const Preset::Type t = setting_type(key);
            if (t == Preset::TYPE_INVALID) throw std::runtime_error("set: no setting called " + it.key());
            by_type[t].emplace_back(key, setting_value(it.value()));
            keys.emplace_back(key, t);
        }
        for (auto& [t, values] : by_type) {
            Tab* tab = wxGetApp().get_tab(t);
            if (tab == nullptr) throw std::runtime_error("set: OrcaSlicer isn't ready yet");
            try { tab->apply_preset_config_values(values); }
            catch (const std::exception& e) { throw std::runtime_error(std::string("set: ") + e.what()); }
        }
        json applied = json::object();
        for (const auto& [key, t] : keys) {
            const std::string v = presets_of(t).get_edited_preset().config.opt_serialize(key);
            applied[key] = v;
            WorkbenchSwarm::act(key + " = " + v);   // a robot sets it, over the plate
        }
        out = {{"applied", applied}, {"print", wxGetApp().preset_bundle->prints.get_edited_preset().name},
               {"modified", wxGetApp().preset_bundle->prints.current_is_dirty() || wxGetApp().preset_bundle->filaments.current_is_dirty()
                                || wxGetApp().preset_bundle->printers.current_is_dirty()}};
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

namespace { std::string g_slice_error; }
void workbench_slice_failed(const std::string& message) { if (workbench_docked()) g_slice_error = message; }
std::string workbench_slice_error() { return g_slice_error; }
void workbench_slice_error_clear() { g_slice_error.clear(); }

} // namespace GUI
} // namespace Slic3r
