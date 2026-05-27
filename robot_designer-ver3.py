#!/usr/bin/env python3
"""
robot_designer.py  —  Robot Designer GUI  (v3: wheels tab + drag-and-drop placement)
- Left panel: canvas preview with full drag-and-drop for sensors and wheels
- Right panel: Notebook with "Wheels" and "Sensors" tabs for list management + properties
- Sensors and Wheels can be dragged directly on the canvas to reposition
- Right-click a sensor/wheel on canvas to select and delete
- Shape, Motion, Save/Load JSON all preserved
"""

import tkinter as tk
from tkinter import ttk, messagebox, filedialog, colorchooser
import json
import math
import copy

# ─── Defaults ────────────────────────────────────────────────────────────────
DEFAULT_ROBOT = {
    "name": "SluggersBot",
    "shape": {
        "type": "rect",
        "rectw": 11.0, "recth": 11.0,
        "circler": 5.5,
        "hexbasew": 11.0, "hexmidw": 16.6, "hextopw": 8.0, "hexmidh": 0.1, "hexhalfh": 5.5,
        "pentbasew": 11.0, "pentshoulderw": 17.80, "pentshoulderh": 2.89, "penthalfh": 9.36,
        "rotation_deg": 0.0,
        # Matches sluggers_sim-ver4.py drawing convention: draw_angle = angle + forward_deg
        "forward_deg": 90.0,
    },
    # Units: inches/sec and rad/sec (matches sluggers_sim-ver4.py)
    "motion": {"speed": 90.0, "turn_speed": 2.5, "omni": True},
    "wheels": [
        # Default 4-wheel omni/mecanum-ish layout: (x forward, y left)
        {"name": "FL", "pos_x":  4.0, "pos_y":  4.0, "width_in": 0.75, "radius_in": 1.5, "height_in": 1.5, "angle_deg": 0.0, "color": "#89b4fa"},
        {"name": "FR", "pos_x":  4.0, "pos_y": -4.0, "width_in": 0.75, "radius_in": 1.5, "height_in": 1.5, "angle_deg": 0.0, "color": "#89b4fa"},
        {"name": "RL", "pos_x": -4.0, "pos_y":  4.0, "width_in": 0.75, "radius_in": 1.5, "height_in": 1.5, "angle_deg": 0.0, "color": "#89b4fa"},
        {"name": "RR", "pos_x": -4.0, "pos_y": -4.0, "width_in": 0.75, "radius_in": 1.5, "height_in": 1.5, "angle_deg": 0.0, "color": "#89b4fa"},
    ],
    "sensors": []
}

SENSOR_TYPES = ["tape", "bump", "ping", "lidar", "trackwire", "ir", "imu"]
SENSOR_DEFAULTS = {
    "tape":      {"pos_x": 5.5, "pos_y": 0.0, "angle_deg": 0,   "color": "#FF6400", "pin": "", "auto_pin": True},
    "bump":      {"pos_x": 5.0, "pos_y": 2.5, "angle_deg": 0,   "color": "#C800C8", "pin": "", "auto_pin": True,
                  "shape": "rect", "rect_w_in": 2.0, "rect_h_in": 0.5,
                  "radius_in": 0.6, "arc_radius_in": 2.0, "arc_start_deg": -35, "arc_end_deg": 35, "arc_thickness_in": 0.35},
    "ping":      {"pos_x": 5.5, "pos_y": 0.0, "angle_deg": 0,   "color": "#00C8C8", "pin": "", "auto_pin": True,
                  "fov_deg": 20, "max_range_in": 72, "nrays": 7},
    "lidar":     {"pos_x": 5.5, "pos_y": 0.0, "angle_deg": 0,   "color": "#7AE3FF", "pin": "", "auto_pin": True,
                  "max_range_in": 240, "nrays": 1},
    "trackwire": {"pos_x": 0.0, "pos_y": 3.5, "angle_deg": 90,  "color": "#00DC50", "pin": "", "auto_pin": True,
                  "max_range_in": 6, "gain": 1.0},
    "ir":        {"pos_x": 5.5, "pos_y": 0.0, "angle_deg": 0,   "color": "#FFDC00", "pin": "", "auto_pin": True,
                  "fov_deg": 40, "detect_freqs": [2000], "mode": "analog", "threshold": 0.15, "rangein": 16.0},
    "imu":       {"pos_x": 0.0, "pos_y": 0.0, "angle_deg": 0,   "color": "#6464FF", "pin": "", "auto_pin": True,
                  "gyro_noise": 0.002, "accel_noise": 0.5, "gyro_bias": 0.001},
}

def _get_valid_pins(stype, mode=None):
    analog = [f"A{i}" for i in range(12)]
    # Digital pins 0,1 are TX/RX. 14-25 are A0-A11.
    digital_only = [f"PIN_{i}" for i in range(2, 44) if i not in range(14, 26)]
    any_pin = analog + digital_only
    
    if stype == "tape":
        return ["Unassigned"] + any_pin
    elif stype == "trackwire":
        return ["Unassigned"] + analog
    elif stype == "ir":
        if mode == "digital":
            return ["Unassigned"] + any_pin
        else:
            return ["Unassigned"] + analog
    elif stype in ("bump", "ping"):
        return ["Unassigned"] + any_pin
    elif stype == "imu":
        return ["Unassigned", "I2C_BUS", "SPI_BUS"]
    elif stype == "lidar":
        return ["Unassigned", "UART_1", "UART_2", "I2C_BUS"]
    else:
        return ["Unassigned"] + any_pin

CANVAS_W, CANVAS_H = 480, 480
SCALE = 18   # pixels per inch


def robot_corners(cfg):
    s = cfg.get("shape", {})
    t = s.get("type", "rect")

    if t == "rect":
        hw = float(s.get("rectw", 11.0)) / 2 * SCALE
        hh = float(s.get("recth", 11.0)) / 2 * SCALE
        return [(-hw, -hh), (hw, -hh), (hw, hh), (-hw, hh)]

    elif t == "circle":
        r = float(s.get("circler", 5.5)) * SCALE
        n = 36
        return [(r * math.cos(2 * math.pi * i / n), r * math.sin(2 * math.pi * i / n)) for i in range(n)]

    elif t == "hexagon":
        bw = float(s.get("hexbasew", 11.0)) * SCALE / 2.0
        mw = float(s.get("hexmidw", 8.0)) * SCALE / 2.0
        tw = float(s.get("hextopw", 5.0)) * SCALE / 2.0
        mh = float(s.get("hexmidh", 3.5)) * SCALE
        th = float(s.get("hexhalfh", 10.25)) * SCALE
        mh = max(0.0, min(th - 1.0, mh))
        bottom_y = 0.0
        mid_y = mh
        top_y = th
        y_center = (bottom_y + top_y) / 2.0
        return [
            (-bw, bottom_y - y_center),
            ( bw, bottom_y - y_center),
            ( mw, mid_y - y_center),
            ( tw, top_y - y_center),
            (-tw, top_y - y_center),
            (-mw, mid_y - y_center),
        ]

    elif t == "pentagon":
        bw = float(s.get("pentbasew", 11.0)) * SCALE / 2.0
        sw = float(s.get("pentshoulderw", 7.5)) * SCALE / 2.0
        sh = float(s.get("pentshoulderh", 2.89)) * SCALE
        th = float(s.get("penthalfh", 9.36)) * SCALE
        sh = max(0.0, min(th - 1.0, sh))
        bottom_y = 0.0
        shoulder_y = sh
        top_y = th
        y_center = (bottom_y + top_y) / 2.0
        return [
            (-bw, bottom_y - y_center),
            ( bw, bottom_y - y_center),
            ( sw, shoulder_y - y_center),
            ( 0.0, top_y - y_center),
            (-sw, shoulder_y - y_center),
        ]

    else:
        return [(-60, -60), (60, -60), (60, 60), (-60, 60)]

def canvas_to_robot(cx, cy, origin_x, origin_y):
    """Convert canvas pixel (cx,cy) to robot-body inches (x forward, y left)."""
    # Canvas: +x right, +y down
    # Robot:  +x forward, +y left
    rx = (origin_y - cy) / SCALE
    ry = (origin_x - cx) / SCALE
    return rx, ry


def robot_to_canvas(rx, ry, origin_x, origin_y):
    """Convert robot-body inches to canvas pixels."""
    cx = origin_x - ry * SCALE
    cy = origin_y - rx * SCALE
    return cx, cy


def _normalize_wheel(w):
    """Keep legacy height_in mirrored to radius_in for JSON compatibility."""
    if "radius_in" in w:
        w["height_in"] = w["radius_in"]
    elif "height_in" in w:
        w["radius_in"] = w["height_in"]


# ─── Main Application ─────────────────────────────────────────────────────────
class RobotDesigner(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("Robot Designer — Sluggers v3")
        self.resizable(True, True)
        self.configure(bg="#1e1e2e")

        self.robot = copy.deepcopy(DEFAULT_ROBOT)
        self.sensor_sel = None   # int index or None
        self.wheel_sel  = None   # int index or None
        self.showitemnamesvar = tk.BooleanVar(value=True)
        self.showshapelabelsvar = tk.BooleanVar(value=True)
        self.showforwardarrowvar = tk.BooleanVar(value=True)

        # Drag state
        self._drag_kind  = None   # "sensor" or "wheel"
        self._drag_idx   = None
        self._drag_off_x = 0.0
        self._drag_off_y = 0.0

        self._build_styles()
        self._build_ui()
        self._refresh_preview()
        self._refresh_sensor_list()
        self._refresh_wheel_list()

    # ══════════════════════════════════════════════════════════════════════════
    # STYLES
    # ══════════════════════════════════════════════════════════════════════════
    def _build_styles(self):
        style = ttk.Style(self)
        style.theme_use("clam")
        bg, fg, field, border, sel = "#1e1e2e", "#cdd6f4", "#313244", "#45475a", "#89b4fa"
        style.configure(".", background=bg, foreground=fg, fieldbackground=field,
                         bordercolor=border, troughcolor=field,
                         selectbackground=sel, selectforeground="#1e1e2e")
        for cls in ("TLabel","TFrame","TCheckbutton"):
            style.configure(cls, background=bg, foreground=fg)
        style.configure("TLabelframe",       background=bg, foreground=sel)
        style.configure("TLabelframe.Label", background=bg, foreground=sel, font=("Helvetica",10,"bold"))
        style.configure("TButton", background=field, foreground=fg, bordercolor=border, padding=4)
        style.map("TButton", background=[("active","#45475a")])
        style.configure("TEntry",    fieldbackground=field, foreground=fg, insertcolor=fg, bordercolor=border)
        style.configure("TCombobox", fieldbackground=field, foreground=fg, selectbackground=sel)
        style.map("TCombobox", fieldbackground=[("readonly", field)], foreground=[("readonly", fg)], selectforeground=[("readonly", fg)])
        style.configure("Accent.TButton", background=sel, foreground="#1e1e2e",
                         font=("Helvetica",10,"bold"), padding=6)
        style.map("Accent.TButton", background=[("active","#74c7ec")])
        style.configure("Treeview", background=field, foreground=fg,
                         fieldbackground=field, rowheight=24)
        style.configure("Treeview.Heading", background=border, foreground=sel,
                         font=("Helvetica",9,"bold"))
        style.map("Treeview", background=[("selected","#585b70")])
        style.configure("TNotebook",     background="#1e1e2e", bordercolor="#45475a")
        style.configure("TNotebook.Tab", background="#313244", foreground=fg,
                         padding=[10, 4], bordercolor="#45475a")
        style.map("TNotebook.Tab",
                  background=[("selected","#89b4fa"), ("active","#45475a")],
                  foreground=[("selected","#1e1e2e")])

    # ══════════════════════════════════════════════════════════════════════════
    # UI LAYOUT
    # ══════════════════════════════════════════════════════════════════════════
    def _build_ui(self):
        # Top bar
        top = ttk.Frame(self); top.pack(fill="x", padx=10, pady=(10, 0))
        ttk.Label(top, text="🤖  Robot Designer", font=("Helvetica", 16, "bold"),
                  foreground="#89b4fa").pack(side="left")
        ttk.Button(top, text="💾  Save JSON", style="Accent.TButton",
                   command=self._save_json).pack(side="right", padx=4)
        ttk.Button(top, text="📂  Load JSON", command=self._load_json).pack(side="right", padx=4)

        # Two-column main area
        main = ttk.Frame(self); main.pack(fill="both", expand=True, padx=10, pady=10)
        left  = ttk.Frame(main); left.grid(row=0, column=0, sticky="nsew", padx=(0, 10))
        right = ttk.Frame(main); right.grid(row=0, column=1, sticky="nsew")
        main.columnconfigure(0, weight=0)
        main.columnconfigure(1, weight=1)
        main.rowconfigure(0, weight=1)

        # ── LEFT: canvas + shape + motion ─────────────────────────────────
        canvas_lf = ttk.LabelFrame(left, text="Preview  (drag sensors & wheels to reposition)")
        canvas_lf.pack(fill="x", expand=False)

        self.canvas = tk.Canvas(canvas_lf, width=CANVAS_W, height=CANVAS_H,
                                bg="#11111b", highlightthickness=0, cursor="crosshair")
        self.canvas.pack(padx=6, pady=6)

        # Canvas mouse bindings
        self.canvas.bind("<ButtonPress-1>",   self._canvas_press)
        self.canvas.bind("<B1-Motion>",       self._canvas_drag)
        self.canvas.bind("<ButtonRelease-1>", self._canvas_release)
        self.canvas.bind("<ButtonPress-3>",   self._canvas_right_click)

        # Hint label
        hint = ttk.Label(left, text="Left-drag: move  |  Right-click: select/delete",
                         foreground="#6c7086", font=("Helvetica", 8))
        hint.pack(anchor="w", padx=8)

        # Preview options
        previewopts = ttk.Frame(left)
        previewopts.pack(fill="x", padx=8, pady=(2, 4))

        ttk.Checkbutton(
            previewopts,
            text="Show wheel/sensor names",
            variable=self.showitemnamesvar,
            command=self._refresh_preview
        ).pack(side="left")

        ttk.Checkbutton(
            previewopts,
            text="Show shape labels",
            variable=self.showshapelabelsvar,
            command=self._refresh_preview
        ).pack(side="left", padx=(10, 0))

        ttk.Checkbutton(
            previewopts,
            text="Show forward arrow",
            variable=self.showforwardarrowvar,
            command=self._refresh_preview
        ).pack(side="left", padx=(10, 0))

        # Robot name row
        name_f = ttk.Frame(left); name_f.pack(fill="x", pady=(6, 0))
        ttk.Label(name_f, text="Robot name:").pack(side="left")
        self.name_var = tk.StringVar(value=self.robot["name"])
        self.name_var.trace_add("write", lambda *_: self._on_name())
        ttk.Entry(name_f, textvariable=self.name_var, width=22).pack(side="left", padx=6)

        shape_lf  = ttk.LabelFrame(left, text="Shape");  shape_lf.pack(fill="x", pady=6)
        self._build_shape_panel(shape_lf)
        motion_lf = ttk.LabelFrame(left, text="Motion"); motion_lf.pack(fill="x")
        self._build_motion_panel(motion_lf)

        # ── RIGHT: notebook ────────────────────────────────────────────────
        nb = ttk.Notebook(right); nb.pack(fill="both", expand=True)
        wheel_tab  = ttk.Frame(nb, style="TFrame")
        sensor_tab = ttk.Frame(nb, style="TFrame")
        nb.add(wheel_tab,  text="  ⚙ Wheels  ")
        nb.add(sensor_tab, text="  📡 Sensors  ")
        self._build_wheel_panel(wheel_tab)
        self._build_sensor_panel(sensor_tab)

    # ══════════════════════════════════════════════════════════════════════════
    # CANVAS DRAG-AND-DROP
    # ══════════════════════════════════════════════════════════════════════════
    def _canvas_origin(self):
        return CANVAS_W // 2, CANVAS_H // 2

    def _hit_test(self, cx, cy):
        """Return ('sensor'|'wheel', idx) for the item nearest click, or (None,None)."""
        ox, oy = self._canvas_origin()
        best_d, best_kind, best_idx = 12, None, None
        for i, s in enumerate(self.robot["sensors"]):
            sx, sy = robot_to_canvas(s.get("pos_x", 0), s.get("pos_y", 0), ox, oy)
            d = math.hypot(cx - sx, cy - sy)
            if d < best_d:
                best_d, best_kind, best_idx = d, "sensor", i
        for i, w in enumerate(self.robot["wheels"]):
            wx, wy = robot_to_canvas(w.get("pos_x", 0), w.get("pos_y", 0), ox, oy)
            ww = max(6.0, float(w.get("width_in", 0.75) * SCALE))
            rr = max(4.0, float(w.get("radius_in", w.get("height_in", 1.5)) * SCALE))
            hit_r = max(8.0, 0.5 * math.hypot(ww, rr))
            d = math.hypot(cx - wx, cy - wy)
            if d < hit_r:
                if d < best_d:
                    best_d, best_kind, best_idx = d, "wheel", i
        return best_kind, best_idx

    def _canvas_press(self, event):
        kind, idx = self._hit_test(event.x, event.y)
        if kind is None:
            self._drag_kind = None
            return
        ox, oy = self._canvas_origin()
        self._drag_kind = kind
        self._drag_idx  = idx
        if kind == "sensor":
            item = self.robot["sensors"][idx]
            self.sensor_sel = idx
            self.wheel_sel  = None
        else:
            item = self.robot["wheels"][idx]
            self.wheel_sel  = idx
            self.sensor_sel = None
        ix, iy = robot_to_canvas(item.get("pos_x", 0), item.get("pos_y", 0), ox, oy)
        self._drag_off_x = event.x - ix
        self._drag_off_y = event.y - iy
        self._sync_selection_to_lists()
        self._refresh_preview()

    def _canvas_drag(self, event):
        if self._drag_kind is None: return
        ox, oy = self._canvas_origin()
        cx = event.x - self._drag_off_x
        cy = event.y - self._drag_off_y
        rx, ry = canvas_to_robot(cx, cy, ox, oy)
        if self._drag_kind == "sensor":
            s = self.robot["sensors"][self._drag_idx]
            s["pos_x"] = round(rx, 2)
            s["pos_y"] = round(ry, 2)
            self._refresh_sensor_list()
            if self.sensor_sel == self._drag_idx:
                self._update_sensor_detail_pos(self._drag_idx)
        else:
            w = self.robot["wheels"][self._drag_idx]
            w["pos_x"] = round(rx, 2)
            w["pos_y"] = round(ry, 2)
            self._refresh_wheel_list()
            if self.wheel_sel == self._drag_idx:
                self._update_wheel_detail_pos(self._drag_idx)
        self._refresh_preview()

    def _canvas_release(self, event):
        self._drag_kind = None

    def _canvas_right_click(self, event):
        kind, idx = self._hit_test(event.x, event.y)
        if kind is None: return
        if kind == "sensor":
            name = self.robot["sensors"][idx]["name"]
            menu = tk.Menu(self, tearoff=0, bg="#313244", fg="#cdd6f4",
                           activebackground="#45475a", activeforeground="#cdd6f4")
            menu.add_command(label=f'Select "{name}"',
                             command=lambda: self._select_sensor(idx))
            menu.add_separator()
            menu.add_command(label=f'🗑 Delete "{name}"',
                             command=lambda: self._delete_sensor(idx))
        else:
            name = self.robot["wheels"][idx]["name"]
            menu = tk.Menu(self, tearoff=0, bg="#313244", fg="#cdd6f4",
                           activebackground="#45475a", activeforeground="#cdd6f4")
            menu.add_command(label=f'Select "{name}"',
                             command=lambda: self._select_wheel(idx))
            menu.add_separator()
            menu.add_command(label=f'🗑 Delete "{name}"',
                             command=lambda: self._delete_wheel(idx))
        menu.tk_popup(event.x_root, event.y_root)

    def _sync_selection_to_lists(self):
        """Push selection state into the treeviews."""
        if self.sensor_sel is not None:
            kids = self.sensor_tree.get_children()
            if self.sensor_sel < len(kids):
                self.sensor_tree.selection_set(kids[self.sensor_sel])
                self._edit_sensor(self.sensor_sel)
        if self.wheel_sel is not None:
            kids = self.wheel_tree.get_children()
            if self.wheel_sel < len(kids):
                self.wheel_tree.selection_set(kids[self.wheel_sel])
                self._edit_wheel(self.wheel_sel)

    def _update_sensor_detail_pos(self, idx):
        s = self.robot["sensors"][idx]
        for key in ("pos_x", "pos_y"):
            var = getattr(self, "_sensor_edit_vars", {}).get(key)
            if var:
                try: var.set(str(round(s[key], 2)))
                except: pass

    def _update_wheel_detail_pos(self, idx):
        w = self.robot["wheels"][idx]
        for key in ("pos_x", "pos_y"):
            var = getattr(self, "_wheel_edit_vars", {}).get(key)
            if var:
                try: var.set(str(round(w[key], 2)))
                except: pass

    # ══════════════════════════════════════════════════════════════════════════
    # SHAPE PANEL
    # ══════════════════════════════════════════════════════════════════════════
    def _build_shape_panel(self, parent):
        row = ttk.Frame(parent); row.pack(fill="x", padx=6, pady=4)
        ttk.Label(row, text="Type:").pack(side="left")
        self.shape_type_var = tk.StringVar(value=self.robot["shape"].get("type", "rect"))
        cb = ttk.Combobox(row, textvariable=self.shape_type_var,
                          values=["rect", "circle", "hexagon", "pentagon"], state="readonly", width=10)
        cb.pack(side="left", padx=6)
        cb.set(self.shape_type_var.get() or "rect")
        cb.bind("<<ComboboxSelected>>", lambda _: self._on_shape_type())
        rot_row = ttk.Frame(parent); rot_row.pack(fill="x", padx=6, pady=(0, 2))
        ttk.Label(rot_row, text="Shape Rot (deg)", width=14).pack(side="left")
        self.shape_rot_var = tk.DoubleVar(value=self.robot["shape"].get("rotation_deg", 0.0))
        self.shape_rot_var.trace_add("write", lambda *_: self._on_shape_rotation())
        ttk.Entry(rot_row, textvariable=self.shape_rot_var, width=8).pack(side="left", padx=4)
        fwd_row = ttk.Frame(parent); fwd_row.pack(fill="x", padx=6, pady=(0, 4))
        ttk.Label(fwd_row, text="Forward (deg)", width=14).pack(side="left")
        self.shape_fwd_var = tk.DoubleVar(value=self.robot["shape"].get("forward_deg", 0.0))
        self.shape_fwd_var.trace_add("write", lambda *_: self._on_shape_forward())
        ttk.Entry(fwd_row, textvariable=self.shape_fwd_var, width=8).pack(side="left", padx=4)
        self.shape_frame = ttk.Frame(parent); self.shape_frame.pack(fill="x", padx=6, pady=(0, 4))
        self._rebuild_shape_fields()

    def _rebuild_shape_fields(self):
        for w in self.shape_frame.winfo_children(): w.destroy()
        t = self.shape_type_var.get(); s = self.robot["shape"]
        if t == "rect":
            fields = [("Width (rectw) in", "rectw"), ("Height (recth) in", "recth")]
        elif t == "circle":
            fields = [("Radius (circler) in", "circler")]
        elif t == "hexagon":
            fields = [("Base-W (hexbasew) in", "hexbasew"), ("Mid-W (hexmidw) in", "hexmidw"), ("Top-W (hextopw) in", "hextopw"), ("Mid-H (hexmidh) in", "hexmidh"), ("Half-H (hexhalfh) in", "hexhalfh")]
        elif t == "pentagon":
            fields = [("Base-W (pentbasew) in", "pentbasew"), ("Shoulder-W (pentshoulderw) in", "pentshoulderw"), ("Shoulder-H (pentshoulderh) in", "pentshoulderh"), ("Half-H (penthalfh) in", "penthalfh")]
        else:
            fields = [("Width (rectw) in", "rectw"), ("Height (recth) in", "recth")]
        self._shape_vars = {}
        for label, key in fields:
            f = ttk.Frame(self.shape_frame); f.pack(fill="x", pady=2)
            ttk.Label(f, text=label, width=14).pack(side="left")
            var = tk.DoubleVar(value=s.get(key, 5.0)); self._shape_vars[key] = var
            var.trace_add("write", lambda *_, k=key, v=var: self._on_shape_field(k, v))
            ttk.Entry(f, textvariable=var, width=8).pack(side="left", padx=4)

    def _on_shape_type(self):
        self.robot["shape"]["type"] = self.shape_type_var.get()
        self._rebuild_shape_fields(); self._refresh_preview()

    def _on_shape_field(self, key, var):
        try: self.robot["shape"][key] = float(var.get()); self._refresh_preview()
        except tk.TclError: pass

    def _on_shape_rotation(self):
        try: self.robot["shape"]["rotation_deg"] = float(self.shape_rot_var.get()); self._refresh_preview()
        except tk.TclError: pass

    def _on_shape_forward(self):
        try: self.robot["shape"]["forward_deg"] = float(self.shape_fwd_var.get()); self._refresh_preview()
        except tk.TclError: pass

    # ══════════════════════════════════════════════════════════════════════════
    # MOTION PANEL
    # ══════════════════════════════════════════════════════════════════════════
    def _build_motion_panel(self, parent):
        m = self.robot["motion"]; self._motion_vars = {}
        for label, key in [("Speed (in/s)", "speed"), ("Turn speed (rad/s)", "turn_speed")]:
            f = ttk.Frame(parent); f.pack(fill="x", padx=6, pady=2)
            ttk.Label(f, text=label, width=20).pack(side="left")
            var = tk.DoubleVar(value=m[key]); self._motion_vars[key] = var
            var.trace_add("write", lambda *_, k=key, v=var: self._on_motion(k, v))
            ttk.Entry(f, textvariable=var, width=8).pack(side="left", padx=4)
        f2 = ttk.Frame(parent); f2.pack(fill="x", padx=6, pady=2)
        self.omni_var = tk.BooleanVar(value=m["omni"])
        ttk.Checkbutton(f2, text="Omni-wheel drive",
                        variable=self.omni_var, command=self._on_omni).pack(side="left")

    def _on_motion(self, key, var):
        try: self.robot["motion"][key] = float(var.get())
        except tk.TclError: pass

    def _on_omni(self): self.robot["motion"]["omni"] = self.omni_var.get()
    def _on_name(self): self.robot["name"] = self.name_var.get()

    # ══════════════════════════════════════════════════════════════════════════
    # WHEEL PANEL
    # ══════════════════════════════════════════════════════════════════════════
    def _build_wheel_panel(self, parent):
        # List
        list_f = ttk.Frame(parent); list_f.pack(fill="both", expand=True, padx=6, pady=(8, 0))
        self.wheel_tree = ttk.Treeview(list_f, columns=("name", "pos", "r"),
                                       show="headings", selectmode="browse", height=6)
        self.wheel_tree.heading("name", text="Name")
        self.wheel_tree.heading("pos",  text="Pos (x, y) in")
        self.wheel_tree.heading("r",    text="Radius (in)")
        self.wheel_tree.column("name", width=70)
        self.wheel_tree.column("pos",  width=130)
        self.wheel_tree.column("r",    width=80)
        sb = ttk.Scrollbar(list_f, orient="vertical", command=self.wheel_tree.yview)
        self.wheel_tree.configure(yscrollcommand=sb.set)
        self.wheel_tree.pack(side="left", fill="both", expand=True)
        sb.pack(side="right", fill="y")
        self.wheel_tree.bind("<<TreeviewSelect>>", self._on_wheel_select)

        # Buttons
        btn_f = ttk.Frame(parent); btn_f.pack(fill="x", padx=6, pady=4)
        ttk.Button(btn_f, text="＋ Add",      command=self._add_wheel).pack(side="left")
        ttk.Button(btn_f, text="✕ Remove",    command=self._remove_wheel).pack(side="left", padx=4)
        ttk.Button(btn_f, text="↔ Mirror",    command=self._mirror_wheel).pack(side="left")
        ttk.Button(btn_f, text="⬆", command=lambda: self._move_wheel(-1)).pack(side="left", padx=(8,2))
        ttk.Button(btn_f, text="⬇", command=lambda: self._move_wheel(1)).pack(side="left")

        ttk.Label(parent, text="Tip: drag wheels directly on the canvas",
                  foreground="#6c7086", font=("Helvetica", 8)).pack(anchor="w", padx=8)

        # Detail editor
        self.wheel_detail_lf = ttk.LabelFrame(parent, text="Wheel Properties")
        self.wheel_detail_lf.pack(fill="x", padx=6, pady=(4, 8))
        self.wheel_detail_frame = ttk.Frame(self.wheel_detail_lf)
        self.wheel_detail_frame.pack(fill="x", padx=4, pady=4)
        self._wheel_edit_vars = {}

    def _add_wheel(self):
        existing = [w["name"] for w in self.robot["wheels"]]
        idx = len(self.robot["wheels"]) + 1
        name = f"W{idx}"
        while name in existing: idx += 1; name = f"W{idx}"
        self.robot["wheels"].append({"name": name, "pos_x": 0.0, "pos_y": 0.0,
                                     "radius_in": 1.5, "color": "#89b4fa"})
        self._refresh_wheel_list()
        self._select_wheel(len(self.robot["wheels"]) - 1)

    def _remove_wheel(self):
        idx = self._get_wheel_sel()
        if idx is None: return
        self._delete_wheel(idx)

    def _delete_wheel(self, idx):
        self.robot["wheels"].pop(idx)
        self.wheel_sel = None
        self._refresh_wheel_list(); self._clear_wheel_detail(); self._refresh_preview()

    def _mirror_wheel(self):
        idx = self._get_wheel_sel()
        if idx is None: return
        orig = copy.deepcopy(self.robot["wheels"][idx])
        orig["pos_y"] = -orig["pos_y"]
        n = orig["name"]
        orig["name"] = n[:-1] + "R" if n.endswith("L") else n[:-1] + "L" if n.endswith("R") else n + "_m"
        self.robot["wheels"].append(orig)
        self._refresh_wheel_list(); self._refresh_preview()

    def _move_wheel(self, d):
        idx = self._get_wheel_sel()
        if idx is None: return
        new = idx + d
        if not (0 <= new < len(self.robot["wheels"])): return
        w = self.robot["wheels"]; w[idx], w[new] = w[new], w[idx]
        self.wheel_sel = new
        self._refresh_wheel_list()
        kids = self.wheel_tree.get_children()
        if new < len(kids): self.wheel_tree.selection_set(kids[new])
        self._refresh_preview()

    def _select_wheel(self, idx):
        self.wheel_sel = idx
        kids = self.wheel_tree.get_children()
        if idx < len(kids): self.wheel_tree.selection_set(kids[idx])
        self._edit_wheel(idx); self._refresh_preview()

    def _get_tree_sel(self, tree):
        sel = tree.selection()
        if not sel: return None
        return tree.get_children().index(sel[0])

    def _get_wheel_sel(self):
        return self._get_tree_sel(self.wheel_tree)

    def _on_wheel_select(self, _e):
        idx = self._get_wheel_sel()
        if idx is not None: self.wheel_sel = idx; self._edit_wheel(idx); self._refresh_preview()

    def _refresh_wheel_list(self):
        self.wheel_tree.delete(*self.wheel_tree.get_children())
        for w in self.robot["wheels"]:
            self.wheel_tree.insert("", "end", values=(
                w["name"],
                f"({w.get('pos_x',0):.2f}, {w.get('pos_y',0):.2f})",
                f"{w.get('width_in',0.75):.2f}×{w.get('radius_in', w.get('height_in',1.5)):.2f}"))

    def _clear_wheel_detail(self):
        for ww in self.wheel_detail_frame.winfo_children(): ww.destroy()
        self._wheel_edit_vars = {}

    def _edit_wheel(self, idx):
        self._clear_wheel_detail()
        wheel = self.robot["wheels"][idx]
        self._wheel_edit_vars = {}
        for row, (label, key, ftype) in enumerate([
            ("Name",        "name",      "str"),
            ("Pos X (in)",  "pos_x",     "float"),
            ("Pos Y (in)",  "pos_y",     "float"),
            ("Width (in)",  "width_in",  "float"),
            ("Radius (in)", "radius_in", "float"),
            ("Angle (deg)", "angle_deg", "float"),
        ]):
            ttk.Label(self.wheel_detail_frame, text=label, width=16).grid(row=row, column=0, sticky="w", pady=2)
            var = tk.StringVar(value=str(wheel.get(key, "")))
            ttk.Entry(self.wheel_detail_frame, textvariable=var, width=14).grid(row=row, column=1, sticky="ew", padx=4)
            var.trace_add("write", lambda *_, k=key, v=var, t=ftype, i=idx: self._on_wheel_field(i, k, v, t))
            self._wheel_edit_vars[key] = var
        row = 6
        ttk.Label(self.wheel_detail_frame, text="Color", width=16).grid(row=row, column=0, sticky="w", pady=2)
        chex = wheel.get("color", "#89b4fa")
        self._wheel_color_btn = tk.Button(self.wheel_detail_frame, bg=chex, width=4, relief="flat",
                                          command=lambda i=idx: self._pick_wheel_color(i))
        self._wheel_color_btn.grid(row=row, column=1, sticky="w", padx=4)
        self.wheel_detail_frame.columnconfigure(1, weight=1)

    def _on_wheel_field(self, idx, key, var, ftype):
        try:
            raw = var.get()
            self.robot["wheels"][idx][key] = float(raw) if ftype == "float" else raw
            _normalize_wheel(self.robot["wheels"][idx])
            self._refresh_wheel_list(); self._refresh_preview()
        except (ValueError, tk.TclError):
            pass

    def _pick_wheel_color(self, idx):
        cur = self.robot["wheels"][idx].get("color", "#89b4fa")
        res = colorchooser.askcolor(color=cur, title="Pick wheel color", parent=self)
        if res and res[1]:
            self.robot["wheels"][idx]["color"] = res[1]
            if hasattr(self, "_wheel_color_btn"): self._wheel_color_btn.configure(bg=res[1])
            self._refresh_preview()

    # ══════════════════════════════════════════════════════════════════════════
    # SENSOR PANEL
    # ══════════════════════════════════════════════════════════════════════════
    def _build_sensor_panel(self, parent):
        list_f = ttk.Frame(parent); list_f.pack(fill="both", expand=True, padx=6, pady=(8, 0))
        self.sensor_tree = ttk.Treeview(list_f, columns=("name", "type", "pos"),
                                        show="headings", selectmode="browse", height=8)
        self.sensor_tree.heading("name", text="Name")
        self.sensor_tree.heading("type", text="Type")
        self.sensor_tree.heading("pos",  text="Pos (x,y) in")
        self.sensor_tree.column("name", width=100)
        self.sensor_tree.column("type", width=80)
        self.sensor_tree.column("pos",  width=120)
        sb = ttk.Scrollbar(list_f, orient="vertical", command=self.sensor_tree.yview)
        self.sensor_tree.configure(yscrollcommand=sb.set)
        self.sensor_tree.pack(side="left", fill="both", expand=True)
        sb.pack(side="right", fill="y")
        self.sensor_tree.bind("<<TreeviewSelect>>", self._on_sensor_select)

        btn_f = ttk.Frame(parent); btn_f.pack(fill="x", padx=6, pady=4)
        self.add_type_var = tk.StringVar(value="tape")
        add_cb = ttk.Combobox(btn_f, textvariable=self.add_type_var, values=SENSOR_TYPES,
                     state="readonly", width=10)
        add_cb.pack(side="left")
        add_cb.set("tape")
        ttk.Button(btn_f, text="＋ Add",   command=self._add_sensor).pack(side="left", padx=4)
        ttk.Button(btn_f, text="✕ Remove", command=self._remove_sensor).pack(side="left")
        ttk.Button(btn_f, text="⬆", command=lambda: self._move_sensor(-1)).pack(side="left", padx=(8, 2))
        ttk.Button(btn_f, text="⬇", command=lambda: self._move_sensor(1)).pack(side="left")

        ttk.Label(parent, text="Tip: drag sensors directly on the canvas",
                  foreground="#6c7086", font=("Helvetica", 8)).pack(anchor="w", padx=8)

        self.sensor_detail_lf = ttk.LabelFrame(parent, text="Sensor Properties")
        self.sensor_detail_lf.pack(fill="both", expand=True, padx=6, pady=(4, 8))
        self.sensor_detail_frame = ttk.Frame(self.sensor_detail_lf)
        self.sensor_detail_frame.pack(fill="both", expand=True, padx=4, pady=4)
        self._sensor_edit_vars = {}

    def _add_sensor(self):
        stype = self.add_type_var.get()
        defaults = copy.deepcopy(SENSOR_DEFAULTS[stype])
        existing = [s["name"] for s in self.robot["sensors"]]
        base = stype; i = 1
        while f"{base}{i}" in existing: i += 1
        self.robot["sensors"].append({"name": f"{base}{i}", "type": stype, **defaults})
        self._refresh_sensor_list()
        self._select_sensor(len(self.robot["sensors"]) - 1)

    def _remove_sensor(self):
        idx = self._get_sensor_sel()
        if idx is not None: self._delete_sensor(idx)

    def _delete_sensor(self, idx):
        self.robot["sensors"].pop(idx)
        self.sensor_sel = None
        self._refresh_sensor_list(); self._clear_sensor_detail(); self._refresh_preview()

    def _move_sensor(self, d):
        idx = self._get_sensor_sel()
        if idx is None: return
        new = idx + d
        if not (0 <= new < len(self.robot["sensors"])): return
        s = self.robot["sensors"]; s[idx], s[new] = s[new], s[idx]
        self.sensor_sel = new
        self._refresh_sensor_list()
        kids = self.sensor_tree.get_children()
        if new < len(kids): self.sensor_tree.selection_set(kids[new])
        self._refresh_preview()

    def _select_sensor(self, idx):
        self.sensor_sel = idx
        kids = self.sensor_tree.get_children()
        if idx < len(kids): self.sensor_tree.selection_set(kids[idx])
        self._edit_sensor(idx); self._refresh_preview()

    def _get_sensor_sel(self):
        return self._get_tree_sel(self.sensor_tree)

    def _on_sensor_select(self, _e):
        idx = self._get_sensor_sel()
        if idx is not None:
            self.sensor_sel = idx; self._edit_sensor(idx); self._refresh_preview()

    def _refresh_sensor_list(self):
        self.sensor_tree.delete(*self.sensor_tree.get_children())
        for s in self.robot["sensors"]:
            self.sensor_tree.insert("", "end", values=(
                s["name"], s["type"],
                f"({s.get('pos_x',0):.2f}, {s.get('pos_y',0):.2f})"))

    def _clear_sensor_detail(self):
        for w in self.sensor_detail_frame.winfo_children(): w.destroy()
        self._sensor_edit_vars = {}

    def _edit_sensor(self, idx):
        self._clear_sensor_detail()
        sensor = self.robot["sensors"][idx]
        stype = sensor["type"]
        self._sensor_edit_vars = {}

        def add_row(row, label, key, ftype, value):
            ttk.Label(self.sensor_detail_frame, text=label, width=16).grid(row=row, column=0, sticky="w", pady=1)
            if ftype.startswith("combo:"):
                choices = ftype.split(":")[1].split(",")
                if value in (None, ""):
                    value = choices[0]
                    sensor[key] = value
                
                is_pin_auto = (key == "pin" and sensor.get("auto_pin", True))
                display_val = "Auto" if is_pin_auto else str(value)
                
                var = tk.StringVar(value=display_val)
                cb = ttk.Combobox(self.sensor_detail_frame, textvariable=var, values=choices if not is_pin_auto else ["Auto"],
                                  state="disabled" if is_pin_auto else "readonly", width=13)
                cb.grid(row=row, column=1, sticky="ew", padx=4)
                
                if not is_pin_auto:
                    var.trace_add("write", lambda *_, k=key, v=var, t="str": self._on_sensor_field_refresh(idx, k, v, t))
            elif ftype == "list_int":
                lst = value if isinstance(value, list) else ([value] if value not in (None, "") else [])
                var = tk.StringVar(value=",".join(str(x) for x in lst))
                ttk.Entry(self.sensor_detail_frame, textvariable=var, width=14).grid(row=row, column=1, sticky="ew", padx=4)
                var.trace_add("write", lambda *_, k=key, v=var, t="list_int": self._on_sensor_field(idx, k, v, t))
            else:
                var = tk.StringVar(value=str(value))
                ttk.Entry(self.sensor_detail_frame, textvariable=var, width=10).grid(row=row, column=1, sticky="ew", padx=4)
                var.trace_add("write", lambda *_, k=key, v=var, t=ftype: self._on_sensor_field(idx, k, v, t))
            self._sensor_edit_vars[key] = var
            return row + 1

        row = 0
        valid_pins = _get_valid_pins(stype, sensor.get("mode"))
        common = [
            ("Name", "name", "str"),
            ("Pos X (in)", "pos_x", "float"),
            ("Pos Y (in)", "pos_y", "float"),
            ("Angle (deg)", "angle_deg", "float"),
            ("Auto-Pin", "auto_pin", "bool"),
            ("Pin Binding", "pin", "combo:" + ",".join(valid_pins)),
        ]
        for label, key, ftype in common:
            if ftype == "bool":
                ttk.Label(self.sensor_detail_frame, text=label, width=16).grid(row=row, column=0, sticky="w", pady=1)
                var = tk.BooleanVar(value=sensor.get(key, True))
                cb = ttk.Checkbutton(self.sensor_detail_frame, variable=var)
                cb.grid(row=row, column=1, sticky="w", padx=4)
                var.trace_add("write", lambda *_, k=key, v=var: self._on_sensor_bool(idx, k, v))
                self._sensor_edit_vars[key] = var
                row += 1
            else:
                row = add_row(row, label, key, ftype, sensor.get(key, ""))

        if stype == "bump":
            if sensor.get("shape") not in ("rect", "circle", "arc"):
                sensor["shape"] = "rect"
            row = add_row(row, "Shape", "shape", "combo:rect,circle,arc", sensor.get("shape", "rect"))
            shape = sensor.get("shape", "rect")
            if shape == "rect":
                row = add_row(row, "Rect W (in)", "rect_w_in", "float", sensor.get("rect_w_in", 2.0))
                row = add_row(row, "Rect H (in)", "rect_h_in", "float", sensor.get("rect_h_in", 0.5))
            elif shape == "circle":
                row = add_row(row, "Circle R (in)", "radius_in", "float", sensor.get("radius_in", 0.6))
            elif shape == "arc":
                row = add_row(row, "Radius (in)", "arc_radius_in", "float", sensor.get("arc_radius_in", 2.0))
                row = add_row(row, "Thickness (in)", "arc_thickness_in", "float", sensor.get("arc_thickness_in", 0.35))
                row = add_row(row, "Start Angle (deg)", "arc_start_deg", "float", sensor.get("arc_start_deg", -35))
                row = add_row(row, "End Angle (deg)", "arc_end_deg", "float", sensor.get("arc_end_deg", 35))
        elif stype == "ping":
            row = add_row(row, "FOV (deg)", "fov_deg", "float", sensor.get("fov_deg", 20))
            row = add_row(row, "Max range (in)", "max_range_in", "float", sensor.get("max_range_in", 72))
            row = add_row(row, "Num rays", "nrays", "int", sensor.get("nrays", 7))
        elif stype == "trackwire":
            row = add_row(row, "Max range (in)", "max_range_in", "float", sensor.get("max_range_in", 6))
            row = add_row(row, "Gain", "gain", "float", sensor.get("gain", 1.0))
        elif stype == "ir":
            row = add_row(row, "FOV (deg)", "fov_deg", "float", sensor.get("fov_deg", 40))
            row = add_row(row, "Range (in)", "rangein", "float", sensor.get("rangein", 16.0))
            row = add_row(row, "Detect freqs", "detect_freqs", "list_int", sensor.get("detect_freqs", [2000]))
            row = add_row(row, "Mode", "mode", "combo:analog,digital", sensor.get("mode", "analog"))
            row = add_row(row, "Threshold", "threshold", "float", sensor.get("threshold", 0.15))
        elif stype == "imu":
            row = add_row(row, "Gyro noise", "gyro_noise", "float", sensor.get("gyro_noise", 0.002))
            row = add_row(row, "Accel noise", "accel_noise", "float", sensor.get("accel_noise", 0.5))
            row = add_row(row, "Gyro bias", "gyro_bias", "float", sensor.get("gyro_bias", 0.001))

        ttk.Label(self.sensor_detail_frame, text="Color", width=16).grid(row=row, column=0, sticky="w", pady=1)
        chex = sensor.get("color", "#ffffff")
        self._sensor_color_btn = tk.Button(self.sensor_detail_frame, bg=chex, width=4, relief="flat",
                                           command=lambda i=idx: self._pick_sensor_color(i))
        self._sensor_color_btn.grid(row=row, column=1, sticky="w", padx=4)
        self.sensor_detail_frame.columnconfigure(1, weight=1)
        
        conflicts = self._get_pin_conflicts()
        if conflicts:
            ttk.Label(self.sensor_detail_frame, text="⚠ Pin Conflicts:\n" + "\n".join(conflicts), foreground="#ff5555").grid(row=row+1, column=0, columnspan=2, sticky="w", pady=6)

    def _get_pin_conflicts(self):
        used = {}
        conflicts = []
        for s in self.robot.get("sensors", []):
            if not s.get("auto_pin", True):
                pin = s.get("pin", "")
                if pin and pin != "Unassigned":
                    used.setdefault(pin, []).append(s["name"])
        for pin, names in used.items():
            if len(names) > 1:
                conflicts.append(f"{pin}: {', '.join(names)}")
        return conflicts

    def _on_sensor_field(self, idx, key, var, ftype):
        try:
            raw = var.get()
            if ftype == "float":       self.robot["sensors"][idx][key] = float(raw)
            elif ftype == "int":       self.robot["sensors"][idx][key] = int(raw)
            elif ftype == "list_int":  self.robot["sensors"][idx][key] = [int(x.strip()) for x in raw.split(",") if x.strip()]
            else:                      self.robot["sensors"][idx][key] = raw
            self._refresh_sensor_list(); self._refresh_preview()
        except (ValueError, tk.TclError): pass

    def _on_sensor_bool(self, idx, key, var):
        self.robot["sensors"][idx][key] = var.get()
        self._refresh_preview()

    def _on_sensor_field_refresh(self, idx, key, var, ftype):
        self._on_sensor_field(idx, key, var, ftype)
        if self.sensor_sel == idx:
            self.after(1, lambda: self._edit_sensor(idx))

    def _pick_sensor_color(self, idx):
        cur = self.robot["sensors"][idx].get("color", "#ffffff")
        res = colorchooser.askcolor(color=cur, title="Pick sensor color", parent=self)
        if res and res[1]:
            self.robot["sensors"][idx]["color"] = res[1]
            if hasattr(self, "_sensor_color_btn"): self._sensor_color_btn.configure(bg=res[1])
            self._refresh_preview()

    # ══════════════════════════════════════════════════════════════════════════
    # CANVAS PREVIEW
    # ══════════════════════════════════════════════════════════════════════════
    def _refresh_preview(self):
        c = self.canvas; c.delete("all")
        ox, oy = CANVAS_W // 2, CANVAS_H // 2
        
        show_item_names = self.showitemnamesvar.get()
        show_shape_labels = self.showshapelabelsvar.get()
        show_forward_arrow = self.showforwardarrowvar.get()
        
        def rotate_xy(x, y, deg):
            a = math.radians(deg)
            ca, sa = math.cos(a), math.sin(a)
            return x * ca - y * sa, x * sa + y * ca
        
        def draw_dim(x1, y1, x2, y2, label, color="#a6adc8", offset=12, text_dx=0, text_dy=0):
            if not show_shape_labels:
                return
            dx = x2 - x1
            dy = y2 - y1
            length = math.hypot(dx, dy)
            if length < 1:
                return
            nx = -dy / length
            ny = dx / length
            ox_off = nx * offset
            oy_off = ny * offset
            ax1, ay1 = x1 + ox_off, y1 + oy_off
            ax2, ay2 = x2 + ox_off, y2 + oy_off
            c.create_line(ax1, ay1, ax2, ay2, fill=color, dash=(3, 2))
            c.create_line(x1, y1, ax1, ay1, fill=color)
            c.create_line(x2, y2, ax2, ay2, fill=color)
            mx = (ax1 + ax2) / 2
            my = (ay1 + ay2) / 2
            c.create_text(mx + text_dx, my - 8 + text_dy, text=label, fill=color, font=("Helvetica", 7, "bold"))

        def local_px_to_canvas(px, py):
            """Robot-local pixels (x forward, y left) -> canvas pixels."""
            return ox - py, oy - px

        # Grid
        for i in range(0, CANVAS_W, 40):
            c.create_line(i, 0, i, CANVAS_H, fill="#1a1a2e")
        for i in range(0, CANVAS_H, 40):
            c.create_line(0, i, CANVAS_W, i, fill="#1a1a2e")
        c.create_line(0, oy, CANVAS_W, oy, fill="#2a2a3e")
        c.create_line(ox, 0, ox, CANVAS_H, fill="#2a2a3e")
        # Axes (robot frame)
        c.create_line(ox, oy, ox, oy - 60, fill="#6c7086", arrow=tk.LAST)
        c.create_line(ox, oy, ox - 60, oy, fill="#6c7086", arrow=tk.LAST)
        c.create_text(ox + 8, 10, text="X+ forward", fill="#6c7086", anchor="nw", font=("Helvetica", 8))
        c.create_text(10, oy - 8, text="Y+ left", fill="#6c7086", anchor="w", font=("Helvetica", 8))

        # Robot body
        pts = robot_corners(self.robot)
        shape_rot = float(self.robot["shape"].get("rotation_deg", 0.0))
        csr, ssr = math.cos(math.radians(shape_rot)), math.sin(math.radians(shape_rot))
        rpts = [(px * csr - py * ssr, px * ssr + py * csr) for px, py in pts]
        screen_pts = [local_px_to_canvas(px, py) for px, py in rpts]
        flat = [c2 for pt in screen_pts for c2 in pt]
        if len(flat) >= 4:
            c.create_polygon(flat, fill="#1e1e3e", outline="#f38ba8", width=2)
            # Draw body dimensions for rect
            t = self.robot["shape"].get("type", "rect")
            if t == "rect" and len(screen_pts) >= 4:
                p0, p1, p2, p3 = screen_pts[:4]
                rectw = float(self.robot["shape"].get("rectw", 11.0))
                recth = float(self.robot["shape"].get("recth", 11.0))
                draw_dim(p0[0], p0[1], p1[0], p1[1], f"rectw={rectw:g} in", offset=16)
                draw_dim(p1[0], p1[1], p2[0], p2[1], f"recth={recth:g} in", offset=16)

        # Forward arrow (matches ver4 drawing: +forward_deg offset)
        if show_forward_arrow:
            fwd_deg = float(self.robot.get("shape", {}).get("forward_deg", 0.0))
            a = math.radians(fwd_deg + shape_rot)
            # arrow in robot-local pixel coordinates
            arrow_len_px = max(40.0, 0.55 * max(math.hypot(px, py) for px, py in rpts)) if rpts else 60.0
            ax = arrow_len_px * math.cos(a)
            ay = arrow_len_px * math.sin(a)
            x1, y1 = local_px_to_canvas(0.0, 0.0)
            x2, y2 = local_px_to_canvas(ax, ay)
            c.create_line(x1, y1, x2, y2, fill="#cdd6f4", width=3, arrow=tk.LAST)

        # Wheels
        for i, w in enumerate(self.robot.get("wheels", [])):
            try:
                wx, wy = robot_to_canvas(float(w.get("pos_x", 0.0)), float(w.get("pos_y", 0.0)), ox, oy)
                width_in = float(w.get("width_in", 0.75))
                rad_in = float(w.get("radius_in", w.get("height_in", 1.5)))
                wheel_len = max(8.0, (2.0 * rad_in) * SCALE)
                wheel_wid = max(5.0, width_in * SCALE)
                raw = str(w.get("color", "#89b4fa")).lstrip("#")
                col = "#" + raw if len(raw) == 6 else "#89b4fa"
                sel = (self.wheel_sel == i)
                ang = math.radians(float(w.get("angle_deg", 0.0)) + 90.0)
                ca, sa = math.cos(ang), math.sin(ang)
                # rectangle corners in robot-local *inches*
                hl_in = (wheel_len / SCALE) / 2.0
                hw_in = (wheel_wid / SCALE) / 2.0
                # forward unit (x,y) and left unit in robot frame
                fux, fuy = ca, sa
                lux, luy = -sa, ca
                corners_local_in = [
                    (-hl_in, -hw_in),
                    ( hl_in, -hw_in),
                    ( hl_in,  hw_in),
                    (-hl_in,  hw_in),
                ]
                pts2 = []
                for dx, dy in corners_local_in:
                    lx = float(w.get("pos_x", 0.0)) + dx*fux + dy*lux
                    ly = float(w.get("pos_y", 0.0)) + dx*fuy + dy*luy
                    pts2.append(robot_to_canvas(lx, ly, ox, oy))
                flat2 = [q for pt in pts2 for q in pt]
                c.create_polygon(flat2, fill="#1a1a2e", outline=("#f38ba8" if sel else col), width=(3 if sel else 2))
                c.create_oval(wx-2, wy-2, wx+2, wy+2, fill=col, outline="")
                if show_item_names:
                    c.create_text(wx + 8, wy - 8, text=str(w.get("name", "W")), fill="#cdd6f4", anchor="nw", font=("Helvetica", 8, "bold"))
            except Exception:
                continue

        # Sensors
        for i, s in enumerate(self.robot.get("sensors", [])):
            try:
                sx, sy = robot_to_canvas(float(s.get("pos_x", 0.0)), float(s.get("pos_y", 0.0)), ox, oy)
                col = s.get("color", "#ffffff")
                sel = (self.sensor_sel == i)
                r = 6
                c.create_oval(sx-r, sy-r, sx+r, sy+r, fill=col, outline=("#f38ba8" if sel else "#000000"), width=(3 if sel else 1))
                # Direction line (angle_deg is in robot frame; does NOT include forward_deg, matching ver4)
                ang = math.radians(float(s.get("angle_deg", 0.0)))
                lx = float(s.get("pos_x", 0.0)) + 1.2 * math.cos(ang)
                ly = float(s.get("pos_y", 0.0)) + 1.2 * math.sin(ang)
                dx, dy = robot_to_canvas(lx, ly, ox, oy)
                c.create_line(sx, sy, dx, dy, fill="#cdd6f4", width=2)
                if show_item_names:
                    c.create_text(sx + 8, sy + 8, text=str(s.get("name", "S")), fill="#cdd6f4", anchor="nw", font=("Helvetica", 8, "bold"))
            except Exception:
                continue

    # ══════════════════════════════════════════════════════════════════════════
    # SAVE / LOAD
    # ══════════════════════════════════════════════════════════════════════════
    def _save_json(self):
        path = filedialog.asksaveasfilename(
            title="Save Robot JSON",
            defaultextension=".json",
            filetypes=[("Robot JSON", "*.json"), ("All files", "*.*")],
        )
        if not path:
            return
        # Normalize wheels for schema compatibility.
        for w in self.robot.get("wheels", []):
            _normalize_wheel(w)
        try:
            with open(path, "w", encoding="utf-8") as f:
                json.dump(self.robot, f, indent=2)
        except Exception as e:
            messagebox.showerror("Save failed", str(e), parent=self)

    def _load_json(self):
        path = filedialog.askopenfilename(
            title="Load Robot JSON",
            filetypes=[("Robot JSON", "*.json"), ("All files", "*.*")],
        )
        if not path:
            return
        try:
            with open(path, "r", encoding="utf-8") as f:
                data = json.load(f)
        except Exception as e:
            messagebox.showerror("Load failed", str(e), parent=self)
            return

        # Merge with defaults so older JSONs still open.
        base = copy.deepcopy(DEFAULT_ROBOT)
        if isinstance(data, dict):
            base.update({k: v for k, v in data.items() if k in ("name", "shape", "motion", "wheels", "sensors")})
            base["shape"] = {**DEFAULT_ROBOT["shape"], **(data.get("shape", {}) if isinstance(data.get("shape"), dict) else {})}
            base["motion"] = {**DEFAULT_ROBOT["motion"], **(data.get("motion", {}) if isinstance(data.get("motion"), dict) else {})}
            base["wheels"] = data.get("wheels", DEFAULT_ROBOT["wheels"])
            base["sensors"] = data.get("sensors", DEFAULT_ROBOT["sensors"])

        self.robot = base
        for w in self.robot.get("wheels", []):
            _normalize_wheel(w)

        # Sync UI state.
        self.name_var.set(self.robot.get("name", "Robot"))
        self.shape_type_var.set(self.robot.get("shape", {}).get("type", "rect"))
        self.shape_rot_var.set(float(self.robot.get("shape", {}).get("rotation_deg", 0.0)))
        self.shape_fwd_var.set(float(self.robot.get("shape", {}).get("forward_deg", 0.0)))
        try:
            self._rebuild_shape_fields()
        except Exception:
            pass
        for k, v in self._motion_vars.items():
            try:
                v.set(float(self.robot.get("motion", {}).get(k, DEFAULT_ROBOT["motion"][k])))
            except Exception:
                continue
        self.omni_var.set(bool(self.robot.get("motion", {}).get("omni", False)))

        self.sensor_sel = None
        self.wheel_sel = None
        self._refresh_wheel_list()
        self._refresh_sensor_list()
        self._clear_wheel_detail()
        self._clear_sensor_detail()
        self._refresh_preview()


if __name__ == "__main__":
    RobotDesigner().mainloop()
