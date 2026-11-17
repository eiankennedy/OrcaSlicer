#ifndef slic3r_GUI_WorkbenchSwarm_hpp_
#define slic3r_GUI_WorkbenchSwarm_hpp_

// Workbench's build swarm, drawn in the 3D view while Claude works (docked in Workbench, WorkbenchDock.hpp): robot
// cursors act out each real change on the plate, piece by piece. A new (or reshaped) object is printed layer by layer
// from the bed, each layer's outline traced and then filled by several robots, the real object shown up to the height
// built; an arranged one is dragged along its path; a setting is typed into a card; slicing sweeps a plane up through
// each object. Each is tagged with the command that changed it. More robots join while there's more work than hands;
// idle ones huddle in a corner; off, they finish what they started and leave.

#include <string>

#include "libslic3r/Point.hpp"

namespace Slic3r {
class GLVolumeCollection;
class Model;

namespace GUI {

class GLCanvas3D;

namespace WorkbenchSwarm {

void set_on(bool on, const std::string& accent_hex = std::string());
bool is_on();
// Hands off (docked in Workbench): the 3D view draws no toolbars, gizmo bar or navigator, only the plate.
void set_hands_off(bool off);
bool hands_off();
// The command Claude is running, shown on the next robot's tag.
void label(const std::string& text);
// The plate as it is before a command changes it (the canvas may not have drawn it since the last change): what
// differs from it on the next frame gets a robot.
void baseline(const Model& model);
// The user moved the view themselves (a drag or the wheel in the 3D view): Workbench leaves it alone from here.
void user_camera();
// A plate opened or started: what's on it is just there (nothing built, no robot), and its view is its own.
void opened(const Model& model);
// A change with no object of its own (a setting, a profile): a robot sets it over the plate, tagged with the text.
void act(const std::string& text);
// Slicing: robots keep working over the plate until it's done.
void working(bool on);
// Before the scene is drawn: find what changed since the last frame, keep new objects hidden until drawn.
void before_render(GLCanvas3D& canvas, GLVolumeCollection& volumes);
// In the opaque pass (its shader in use): objects being printed, drawn up to the height built so far.
void render_rising(GLCanvas3D& canvas, GLVolumeCollection& volumes, const Transform3d& view, const Transform3d& projection);
// In the overlay pass (inside the ImGui frame): the robots and their work.
void render(GLCanvas3D& canvas);

} // namespace WorkbenchSwarm
} // namespace GUI
} // namespace Slic3r

#endif
