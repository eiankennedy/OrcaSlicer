#ifndef slic3r_GUI_WorkbenchAccent_hpp_
#define slic3r_GUI_WorkbenchAccent_hpp_

// OrcaSlicer's accent (its green, #009688), or Workbench's session colour when docked in Workbench (WorkbenchDock.hpp).

#include <wx/colour.h>

#include <cstdlib>
#include <cstring>
#include <string>

namespace Slic3r { namespace GUI {

inline std::string orca_accent_hex()
{
    const char* dock = std::getenv("WORKBENCH_DOCK");
    const char* a    = std::getenv("WORKBENCH_ACCENT");
    return (dock && dock[0] && a && a[0] == '#' && std::strlen(a) == 7) ? std::string(a) : std::string("#009688");
}

inline wxColour orca_accent() { return wxColour(orca_accent_hex()); }

}} // namespace Slic3r::GUI

#endif
