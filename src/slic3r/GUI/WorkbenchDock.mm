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

namespace Slic3r { namespace GUI {

static NSWindow* dock_window = nil;
static std::function<std::string(const std::string&)> dock_handler;

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

static void dock_command(int fd, NSDictionary* msg)
{
    id cid = msg[@"id"] ?: [NSNull null];
    NSData* data = [NSJSONSerialization dataWithJSONObject:msg options:0 error:nil];
    std::string request(static_cast<const char*>(data.bytes), data.length);
    @try {
        std::string result = dock_handler ? dock_handler(request) : std::string("null");
        NSData* rd = [NSData dataWithBytes:result.data() length:result.size()];
        id r = [NSJSONSerialization JSONObjectWithData:rd options:NSJSONReadingFragmentsAllowed error:nil];
        dock_write(fd, @{@"id": cid, @"ok": @YES, @"result": r ?: [NSNull null]});
    } @catch (NSException* e) {
        dock_write(fd, @{@"id": cid, @"ok": @NO, @"error": e.reason ?: @"failed"});
    }
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
    std::thread(dock_listen, std::string(getenv("WORKBENCH_DOCK")), true).detach();
    if (const char* control = getenv("WORKBENCH_CONTROL"); control && control[0])
        std::thread(dock_listen, std::string(control), false).detach();
}

}} // namespace Slic3r::GUI
