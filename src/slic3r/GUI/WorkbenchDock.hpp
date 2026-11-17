#ifndef slic3r_GUI_WorkbenchDock_hpp_
#define slic3r_GUI_WorkbenchDock_hpp_

// Docking into Workbench (a Mac app that runs OrcaSlicer in its Print panel). When WORKBENCH_DOCK names a Unix
// socket (the Mac app's), the main window has no frame of its own and Workbench places it over its panel;
// WORKBENCH_CONTROL names a second socket (Workbench's server's) for its commands (load, arrange, slice, hands off,
// swarm...). One JSON object per line each way:
//   placement: {"rect": [x, y, w, h] (screen points, bottom-left origin), "above": <window number>, "visible": bool}
//   command:   {"id": n, "cmd": "...", ...}  ->  {"id": n, "ok": true, "result": ...} | {"id": n, "ok": false, "error": "..."}

#include <functional>
#include <string>

namespace Slic3r { namespace GUI {

// True when OrcaSlicer runs docked in Workbench (WORKBENCH_DOCK is set).
bool workbench_docked();
// The session's accent from Workbench (WORKBENCH_ACCENT, "#rrggbb"), or empty.
std::string workbench_accent();
// Hand the main window (an NSView* of it: wxWindow::GetHandle()) to Workbench and start listening. The handler runs
// on the main thread with one command (a JSON object, as text) and returns the reply's result as JSON text, or throws
// std::exception for an error.
void workbench_dock_attach(void* ns_view, std::function<std::string(const std::string&)> handler);

}} // namespace Slic3r::GUI

#endif
