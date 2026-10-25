#!/usr/bin/env python3
"""List top-level X11 windows (id, geometry, title) without x11-utils.

Used by scripts/record_screen.sh to find the Gazebo and RViz windows on WSLg/X11.

    python3 scripts/x11_windows.py            # all named windows
    python3 scripts/x11_windows.py RViz       # ids of windows whose title contains "RViz"
"""

import ctypes
import ctypes.util
import sys

xlib = ctypes.cdll.LoadLibrary(ctypes.util.find_library('X11'))
xlib.XOpenDisplay.restype = ctypes.c_void_p
xlib.XDefaultRootWindow.argtypes = [ctypes.c_void_p]
xlib.XDefaultRootWindow.restype = ctypes.c_ulong
xlib.XQueryTree.argtypes = [ctypes.c_void_p, ctypes.c_ulong, ctypes.POINTER(ctypes.c_ulong),
                            ctypes.POINTER(ctypes.c_ulong), ctypes.POINTER(ctypes.POINTER(ctypes.c_ulong)),
                            ctypes.POINTER(ctypes.c_uint)]
xlib.XFetchName.argtypes = [ctypes.c_void_p, ctypes.c_ulong, ctypes.POINTER(ctypes.c_char_p)]


class XWindowAttributes(ctypes.Structure):
    _fields_ = [('x', ctypes.c_int), ('y', ctypes.c_int), ('width', ctypes.c_int),
                ('height', ctypes.c_int), ('border_width', ctypes.c_int), ('depth', ctypes.c_int),
                ('visual', ctypes.c_void_p), ('root', ctypes.c_ulong), ('c_class', ctypes.c_int),
                ('bit_gravity', ctypes.c_int), ('win_gravity', ctypes.c_int),
                ('backing_store', ctypes.c_int), ('backing_planes', ctypes.c_ulong),
                ('backing_pixel', ctypes.c_ulong), ('save_under', ctypes.c_int),
                ('colormap', ctypes.c_ulong), ('map_installed', ctypes.c_int),
                ('map_state', ctypes.c_int), ('all_event_masks', ctypes.c_long),
                ('your_event_mask', ctypes.c_long), ('do_not_propagate_mask', ctypes.c_long),
                ('override_redirect', ctypes.c_int), ('screen', ctypes.c_void_p)]


xlib.XGetWindowAttributes.argtypes = [ctypes.c_void_p, ctypes.c_ulong, ctypes.POINTER(XWindowAttributes)]


def windows(display):
    root = xlib.XDefaultRootWindow(display)
    out = []

    def walk(win, depth):
        name = ctypes.c_char_p()
        if xlib.XFetchName(display, win, ctypes.byref(name)) and name.value:
            attrs = XWindowAttributes()
            xlib.XGetWindowAttributes(display, win, ctypes.byref(attrs))
            out.append((win, attrs.width, attrs.height, attrs.map_state,
                        name.value.decode('utf-8', 'replace')))
        if depth > 3:
            return
        r, p = ctypes.c_ulong(), ctypes.c_ulong()
        children = ctypes.POINTER(ctypes.c_ulong)()
        n = ctypes.c_uint()
        if xlib.XQueryTree(display, win, ctypes.byref(r), ctypes.byref(p), ctypes.byref(children),
                           ctypes.byref(n)):
            for i in range(n.value):
                walk(children[i], depth + 1)

    walk(root, 0)
    return out


def main() -> int:
    display = xlib.XOpenDisplay(None)
    if not display:
        print('cannot open X display', file=sys.stderr)
        return 1
    found = windows(display)
    if len(sys.argv) > 1:
        # largest viewable window whose title contains the pattern
        matches = [w for w in found if sys.argv[1].lower() in w[4].lower() and w[3] == 2]
        matches.sort(key=lambda w: w[1] * w[2], reverse=True)
        if not matches:
            return 1
        print(hex(matches[0][0]))
        return 0
    for win, w, h, state, name in found:
        print(f'{hex(win)}\t{w}x{h}\tmapped={state == 2}\t{name}')
    return 0


if __name__ == '__main__':
    sys.exit(main())
