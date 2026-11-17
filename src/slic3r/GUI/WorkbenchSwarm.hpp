#ifndef slic3r_GUI_WorkbenchSwarm_hpp_
#define slic3r_GUI_WorkbenchSwarm_hpp_

// Workbench's build swarm, drawn in the 3D view while Claude works (docked in Workbench, WorkbenchDock.hpp): robot
// cursors act out each real change on the plate. A new object stays hidden until a robot has drawn it out from corner
// to corner; a moved or reshaped one gets a robot working over it; each is labelled with the command that changed it.
// When every robot is busy another joins, as many as the work needs; idle ones huddle in a corner; off, they leave.

#include <string>

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
// A change with no object of its own (a setting, a profile): a robot sets it over the plate, tagged with the text.
void act(const std::string& text);
// Slicing: robots keep working over the plate until it's done.
void working(bool on);
// Before the scene is drawn: find what changed since the last frame, keep new objects hidden until drawn.
void before_render(GLCanvas3D& canvas, GLVolumeCollection& volumes);
// In the overlay pass (inside the ImGui frame): the robots and their gestures.
void render(GLCanvas3D& canvas);

} // namespace WorkbenchSwarm
} // namespace GUI
} // namespace Slic3r

#endif
