#include "WorkbenchDock.hpp"

#include <cstdlib>
#include <cstring>
#include <exception>
#include <string>
#include <thread>

#include <sys/socket.h>
#include <sys/un.h>
#include <unistd.h>

#import <Cocoa/Cocoa.h>

#include <vector>

#include <wx/dialog.h>
#include <wx/progdlg.h>
#include <wx/toplevel.h>

namespace Slic3r { namespace GUI {

static NSWindow* dock_window = nil;
static std::function<std::string(const std::string&)> dock_handler;
static int dock_busy = 0;                       // commands running (or waiting to finish: slicing)
static std::vector<std::string> dock_asked;     // what dialogs would have asked meanwhile
static NSTimer* dock_watch = nil;
static std::function<bool(std::string&, std::string&)> dock_later;

bool workbench_busy() { return dock_busy > 0; }
void workbench_asked(const std::string& message) { if (dock_busy > 0) dock_asked.push_back(message); }
void workbench_reply_later(std::function<bool(std::string&, std::string&)> poll) { dock_later = std::move(poll); }

// The last resort while a command runs: a modal dialog is cancelled at once (nobody could answer it, hidden or
// driven by Claude) and the reply names it.
static void dock_dismiss_modals()
{
    if (dock_busy == 0) return;
    for (wxWindow* w : wxTopLevelWindows) {
        wxDialog* dlg = dynamic_cast<wxDialog*>(w);
        // a progress dialog asks nothing: it goes away by itself
        if (dlg && dlg->IsModal() && !dynamic_cast<wxGenericProgressDialog*>(dlg)) {
            std::string what = dlg->GetTitle().ToStdString(wxConvUTF8);
            dock_asked.push_back(what.empty() ? std::string("a dialog") : what);
            dlg->EndModal(wxID_CANCEL);
            return;
        }
    }
    // a native alert; not a modal session of another kind (a progress dialog's)
    NSWindow* modal = NSApp.modalWindow;
    if (modal && [NSStringFromClass([modal class]) containsString:@"Alert"]) {
        dock_asked.push_back(modal.title.length ? std::string(modal.title.UTF8String) : std::string("a dialog"));
        [NSApp abortModal];
    }
}

static void dock_busy_begin()
{
    if (dock_busy++ > 0) return;
    dock_asked.clear();
    // in the common modes, so it fires inside a modal dialog's run loop too
    dock_watch = [NSTimer timerWithTimeInterval:0.25 repeats:YES block:^(NSTimer*) { dock_dismiss_modals(); }];
    [[NSRunLoop mainRunLoop] addTimer:dock_watch forMode:NSRunLoopCommonModes];
}

static void dock_busy_end()
{
    if (--dock_busy > 0) return;
    [dock_watch invalidate];
    dock_watch = nil;
}

static std::string dock_said()
{
    std::string s;
    for (size_t i = 0; i < dock_asked.size(); ++i) s += (i ? "; " : "") + dock_asked[i];
    return s;
}

bool workbench_docked()
{
    const char* p = getenv("WORKBENCH_DOCK");
    return p && p[0];
}

std::string workbench_accent()
{
    const char* a = getenv("WORKBENCH_ACCENT");
    return a ? std::string(a) : std::string();
}

static void dock_write(int fd, NSDictionary* msg)
{
    if (fd < 0) return;
    NSData* data = [NSJSONSerialization dataWithJSONObject:msg options:0 error:nil];
    if (!data) return;
    NSMutableData* line = [data mutableCopy];
    [line appendBytes:"\n" length:1];
    (void)write(fd, line.bytes, line.length);
}

static void dock_place(NSDictionary* msg)
{
    NSWindow* window = dock_window;
    if (!window) return;
    NSNumber* visible = msg[@"visible"];
    if (visible && !visible.boolValue) {
        [window orderOut:nil];
        return;
    }
    NSArray* rect = msg[@"rect"];
    if ([rect isKindOfClass:[NSArray class]] && rect.count == 4) {
        NSRect frame = NSMakeRect([rect[0] doubleValue], [rect[1] doubleValue], [rect[2] doubleValue], [rect[3] doubleValue]);
        if (frame.size.width >= 1 && frame.size.height >= 1 && !NSEqualRects(frame, window.frame))
            [window setFrame:frame display:YES];
    }
    NSNumber* above = msg[@"above"];
    if (above) [window orderWindow:NSWindowAbove relativeTo:above.integerValue];
    if ([msg[@"focus"] boolValue]) {
        [NSApp activateIgnoringOtherApps:YES];
        [window makeKeyWindow];
    }
}

static NSString* dock_ns(const std::string& s) { return [NSString stringWithUTF8String:s.c_str()] ?: @""; }

static void dock_reply(int fd, id cid, const std::string& result, const std::string& error)
{
    if (!error.empty()) {
        const std::string said = dock_said();
        dock_write(fd, @{@"id": cid, @"ok": @NO, @"error": dock_ns(said.empty() ? error : error + " (OrcaSlicer: " + said + ")")});
        return;
    }
    NSData* rd = [NSData dataWithBytes:result.data() length:result.size()];
    id r = [NSJSONSerialization JSONObjectWithData:rd options:NSJSONReadingFragmentsAllowed error:nil];
    if (!dock_asked.empty() && [r isKindOfClass:[NSDictionary class]]) {
        NSMutableArray* said = [NSMutableArray array];
        for (const std::string& a : dock_asked) [said addObject:dock_ns(a)];
        NSMutableDictionary* m = [r mutableCopy];
        m[@"orca_said"] = said;
        r = m;
    }
    dock_write(fd, @{@"id": cid, @"ok": @YES, @"result": r ?: [NSNull null]});
}

static void dock_command(int fd, NSDictionary* msg)
{
    id cid = msg[@"id"] ?: [NSNull null];
    NSData* data = [NSJSONSerialization dataWithJSONObject:msg options:0 error:nil];
    std::string request(static_cast<const char*>(data.bytes), data.length);
    dock_busy_begin();
    dock_later = nullptr;
    std::string result, error;
    try {
        result = dock_handler ? dock_handler(request) : std::string("null");
    } catch (const std::exception& e) {
        error = e.what();
        if (error.empty()) error = "failed";
    } catch (...) {
        error = "failed";
    }
    if (error.empty() && dock_later) {
        // it finishes later: still busy (no dialogs) until it has
        auto poll = std::move(dock_later);
        dock_later = nullptr;
        __block NSTimer* timer = nil;
        timer = [NSTimer timerWithTimeInterval:0.2 repeats:YES block:^(NSTimer*) {
            std::string res, err;
            bool done = false;
            try { done = poll(res, err); } catch (const std::exception& e) { done = true; err = e.what(); }
            if (!done) return;
            [timer invalidate];
            timer = nil;
            dock_reply(fd, cid, res, err);
            dock_busy_end();
        }];
        [[NSRunLoop mainRunLoop] addTimer:timer forMode:NSRunLoopCommonModes];
        return;
    }
    dock_reply(fd, cid, result, error);
    dock_busy_end();
}

static void dock_dispatch(int fd, NSDictionary* msg)
{
    if (msg[@"cmd"]) {
        // a C++ exception can't cross into Objective-C's @catch: turn it into a reply here
        try {
            dock_command(fd, msg);
        } catch (const std::exception& e) {
            dock_write(fd, @{@"id": msg[@"id"] ?: [NSNull null], @"ok": @NO, @"error": [NSString stringWithUTF8String:e.what()] ?: @"failed"});
        }
    } else {
        dock_place(msg);
        // Workbench places it as it comes forward, before macOS has finished bringing its window up: over it again
        // once that's done (the latest placement)
        static NSDictionary* latest = nil;
        latest = msg;
        for (double delay : {0.15, 0.5, 1.0})
            dispatch_after(dispatch_time(DISPATCH_TIME_NOW, int64_t(delay * NSEC_PER_SEC)), dispatch_get_main_queue(), ^{
                if (latest == msg && [msg[@"visible"] boolValue] && ![msg[@"focus"] boolValue]) dock_place(msg);
            });
    }
}

// One connection to Workbench: the Mac app's (placement; its end ends OrcaSlicer) or the server's (commands).
static void dock_listen(std::string path, bool owner)
{
    const int fd = socket(AF_UNIX, SOCK_STREAM, 0);
    if (fd < 0) return;
    sockaddr_un addr = {};
    addr.sun_family = AF_UNIX;
    strncpy(addr.sun_path, path.c_str(), sizeof(addr.sun_path) - 1);
    bool connected = false;
    for (int i = 0; i < 100 && !connected; i++) {
        connected = connect(fd, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) == 0;
        if (!connected) usleep(50000);
    }
    if (!connected) { close(fd); return; }
    int on = 1;
    setsockopt(fd, SOL_SOCKET, SO_NOSIGPIPE, &on, sizeof(on));
    const std::string hello = "{\"ready\":true,\"pid\":" + std::to_string(getpid()) + "}\n";
    (void)write(fd, hello.data(), hello.size());

    std::string buf;
    char chunk[8192];
    for (;;) {
        const ssize_t n = read(fd, chunk, sizeof(chunk));
        if (n <= 0) break;
        buf.append(chunk, size_t(n));
        size_t nl;
        while ((nl = buf.find('\n')) != std::string::npos) {
            NSData* line = [NSData dataWithBytes:buf.data() length:nl];
            buf.erase(0, nl + 1);
            id msg = [NSJSONSerialization JSONObjectWithData:line options:0 error:nil];
            if ([msg isKindOfClass:[NSDictionary class]]) {
                dispatch_async(dispatch_get_main_queue(), ^{ dock_dispatch(fd, msg); });
            }
        }
    }
    close(fd);
    // Workbench owns OrcaSlicer's lifetime while docked
    if (owner) dispatch_async(dispatch_get_main_queue(), ^{ [NSApp terminate:nil]; });
}

static bool dock_shown = false;

// the window seen at last: once a frame of the 3D view is on screen (one more turn of the run loop after it's
// presented), or after a while whatever happens
static void dock_show()
{
    if (dock_shown || !dock_window) return;
    dock_shown = true;
    dock_window.alphaValue = 1.0;
}

void workbench_dock_painted()
{
    if (dock_shown || !dock_window) return;
    dispatch_after(dispatch_time(DISPATCH_TIME_NOW, int64_t(0.05 * NSEC_PER_SEC)), dispatch_get_main_queue(), ^{ dock_show(); });
}

void workbench_dock_attach(void* ns_view, std::function<std::string(const std::string&)> handler)
{
    if (dock_window || !workbench_docked()) return;
    NSView* view = (__bridge NSView*) ns_view;
    NSWindow* window = view.window;
    if (!window) return;
    dock_window = window;
    dock_handler = std::move(handler);
    // WB_DOCK_SKIP (debugging): style, policy, origin, appearance
    const char* skip = getenv("WB_DOCK_SKIP");
    auto skipped = [&](const char* what) { return skip && strstr(skip, what); };
    if (!skipped("style")) {
        // frameless to the eye, but still a titled window: wxWidgets closes the frame if its window turns borderless
        window.styleMask |= NSWindowStyleMaskFullSizeContentView;
        window.titlebarAppearsTransparent = YES;
        window.titleVisibility = NSWindowTitleHidden;
        for (NSWindowButton b : {NSWindowCloseButton, NSWindowMiniaturizeButton, NSWindowZoomButton})
            [window standardWindowButton:b].hidden = YES;
        window.hasShadow = NO;
        window.movable = NO;
    }
    if (!skipped("appearance")) NSApp.appearance = [NSAppearance appearanceNamed:NSAppearanceNameDarkAqua];
    if (!skipped("policy")) [NSApp setActivationPolicy:NSApplicationActivationPolicyAccessory];
    if (!skipped("origin")) [window setFrameOrigin:NSMakePoint(-30000, -30000)];
    if (!skipped("reveal")) {
        // until the 3D view's first frame the window is see-through (Workbench's panel shows), and anything of it not
        // painted yet is the plate's grey rather than white
        window.alphaValue = 0.0;
        window.backgroundColor = [NSColor colorWithSRGBRed:0.21 green:0.21 blue:0.23 alpha:1.0];
        dispatch_after(dispatch_time(DISPATCH_TIME_NOW, int64_t(45 * NSEC_PER_SEC)), dispatch_get_main_queue(), ^{ dock_show(); });
    } else
        dock_shown = true;
    std::thread(dock_listen, std::string(getenv("WORKBENCH_DOCK")), true).detach();
    if (const char* control = getenv("WORKBENCH_CONTROL"); control && control[0])
        std::thread(dock_listen, std::string(control), false).detach();
}

}} // namespace Slic3r::GUI
