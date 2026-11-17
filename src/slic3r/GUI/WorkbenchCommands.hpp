#ifndef slic3r_GUI_WorkbenchCommands_hpp_
#define slic3r_GUI_WorkbenchCommands_hpp_

#include <string>

namespace Slic3r {
namespace GUI {

// One command from Workbench (JSON text) to its result (JSON text); throws std::exception on error. See the .cpp.
std::string workbench_command(const std::string& request);
// Hands off (only the 3D view) or on.
void workbench_set_hands(bool off);

} // namespace GUI
} // namespace Slic3r

#endif
