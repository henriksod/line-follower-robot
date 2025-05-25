#!/usr/bin/env python3

import tkinter as tk
from tkinter import ttk, filedialog
import json
import math


class LineEditor:
    def __init__(self, root):
        self.root = root
        self.root.title("Scenario Editor")

        self.lines = []
        self.redo_stack = []
        self.preview_line = None
        self.current_line_start = None
        self.snap_to_grid = tk.BooleanVar(value=True)
        self.snap_to_point = tk.BooleanVar(value=False)
        self.pan_mode = tk.BooleanVar(value=False)
        self.curve_mode = tk.BooleanVar(value=False)
        self.grid_resolution_var = tk.DoubleVar(value=10.0)  # cm
        self.zoom_var = tk.DoubleVar(value=100.0)  # percent
        self.zoom = 1.0
        self.offset_x = 0
        self.offset_y = 0
        self.panning = False
        self.last_pan = (0, 0)

        self.curve_points = []
        self.robot_mode = False
        self.robot_pose = None
        self.robot_preview_pos = None

        self.setup_ui()
        self.bind_events()
        self.update_zoom()
        self.draw()

    def setup_ui(self):
        self.canvas = tk.Canvas(self.root, width=800, height=600, bg="white")
        self.canvas.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)

        control_frame = tk.Frame(self.root)
        control_frame.pack(side=tk.RIGHT, fill=tk.Y)

        ttk.Checkbutton(control_frame, text="Snap to Grid", variable=self.snap_to_grid).pack(
            anchor="w"
        )
        ttk.Checkbutton(control_frame, text="Snap to Endpoints", variable=self.snap_to_point).pack(
            anchor="w"
        )
        ttk.Checkbutton(control_frame, text="Pan Mode", variable=self.pan_mode).pack(anchor="w")
        ttk.Checkbutton(control_frame, text="Draw Curves", variable=self.curve_mode).pack(
            anchor="w"
        )

        ttk.Label(control_frame, text="Grid Resolution (cm):").pack(anchor="w")
        ttk.Spinbox(control_frame, from_=1, to=100, textvariable=self.grid_resolution_var).pack(
            anchor="w"
        )

        ttk.Label(control_frame, text="Zoom (%):").pack(anchor="w")
        ttk.Spinbox(
            control_frame, from_=10, to=500, textvariable=self.zoom_var, command=self.update_zoom
        ).pack(anchor="w")

        ttk.Button(control_frame, text="Place Robot in World", command=self.enter_robot_mode).pack(
            fill="x"
        )
        ttk.Button(control_frame, text="Save Scenario", command=self.save_scenario).pack(fill="x")
        ttk.Button(control_frame, text="Load Scenario", command=self.load_scenario).pack(fill="x")
        ttk.Button(control_frame, text="Center View", command=self.center_view).pack(fill="x")
        ttk.Button(control_frame, text="Redo (Ctrl+Y)", command=self.redo).pack(fill="x")

    def bind_events(self):
        self.canvas.bind("<Button-1>", self.on_click)
        self.canvas.bind("<B1-Motion>", self.on_drag)
        self.canvas.bind("<ButtonRelease-1>", self.on_release)
        self.canvas.bind("<Motion>", self.on_mouse_move)
        self.root.bind("<Escape>", self.cancel_drawing)
        self.root.bind("<Control-z>", self.undo)
        self.root.bind("<Control-y>", self.redo)
        self.canvas.bind("<ButtonPress-2>", self.start_pan)
        self.canvas.bind("<B2-Motion>", self.do_pan)
        self.canvas.bind("<ButtonRelease-2>", self.stop_pan)

    def update_zoom(self):
        self.zoom = self.zoom_var.get() / 100.0
        self.draw()

    def zoomed(self, value_cm):
        return value_cm * self.zoom

    def to_screen(self, x_cm, y_cm):
        return self.zoomed(x_cm) + self.offset_x, self.zoomed(y_cm) + self.offset_y

    def to_world(self, x_px, y_px):
        return (x_px - self.offset_x) / self.zoom, (y_px - self.offset_y) / self.zoom

    def draw(self):
        self.canvas.delete("all")
        self.draw_grid()

        for line in self.lines:
            if line.get("type") == "curve":
                self.draw_bezier(line["start"], line["control"], line["end"])
            else:
                sx, sy = self.to_screen(*line["start"])
                ex, ey = self.to_screen(*line["end"])
                self.canvas.create_line(sx, sy, ex, ey, fill="black", width=2)

        if self.preview_line:
            if self.preview_line.get("type") == "curve":
                self.draw_bezier(
                    self.preview_line["start"],
                    self.preview_line["control"],
                    self.preview_line["end"],
                    preview=True,
                )
            else:
                sx, sy = self.to_screen(*self.preview_line["start"])
                ex, ey = self.to_screen(*self.preview_line["end"])
                self.canvas.create_line(sx, sy, ex, ey, fill="blue", dash=(4, 2), width=2)

        if self.robot_pose:
            self.draw_robot(*self.robot_pose)

        if self.robot_mode and self.robot_preview_pos:
            x, y = self.to_screen(*self.robot_preview_pos)
            self.canvas.create_oval(x - 5, y - 5, x + 5, y + 5, outline="red", dash=(2, 2))

    def draw_grid(self):
        spacing_cm = self.grid_resolution_var.get()
        width = int(self.canvas["width"])
        height = int(self.canvas["height"])
        top_left_world = self.to_world(0, 0)
        bottom_right_world = self.to_world(width, height)

        start_x = math.floor(top_left_world[0] / spacing_cm) * spacing_cm
        end_x = math.ceil(bottom_right_world[0] / spacing_cm) * spacing_cm
        start_y = math.floor(top_left_world[1] / spacing_cm) * spacing_cm
        end_y = math.ceil(bottom_right_world[1] / spacing_cm) * spacing_cm

        x = start_x
        while x <= end_x:
            sx, _ = self.to_screen(x, 0)
            self.canvas.create_line(sx, 0, sx, height, fill="lightgray", tags="grid")
            x += spacing_cm

        y = start_y
        while y <= end_y:
            _, sy = self.to_screen(0, y)
            self.canvas.create_line(0, sy, width, sy, fill="lightgray", tags="grid")
            y += spacing_cm

    def snap(self, x, y):
        if self.snap_to_grid.get():
            spacing = self.grid_resolution_var.get()
            x = round(x / spacing) * spacing
            y = round(y / spacing) * spacing

        if self.snap_to_point.get():
            for line in self.lines:
                for pt in [line.get("start"), line.get("end")]:
                    if pt and abs(pt[0] - x) < 0.2 and abs(pt[1] - y) < 0.2:
                        return pt
        return (x, y)

    def on_click(self, event):
        if self.robot_mode:
            self.robot_pose = [(self.to_world(event.x, event.y)), (0.0, 0.0)]
            self.canvas.bind("<Motion>", self.robot_rotate_preview)
            self.canvas.bind("<Button-1>", self.confirm_robot_orientation)
            return

        if self.pan_mode.get():
            self.panning = True
            self.last_pan = (event.x, event.y)
            return

        point = self.snap(*self.to_world(event.x, event.y))
        if self.curve_mode.get():
            self.curve_points.append(point)
            if len(self.curve_points) == 3:
                self.lines.append(
                    {
                        "type": "curve",
                        "start": self.curve_points[0],
                        "control": self.curve_points[1],
                        "end": self.curve_points[2],
                    }
                )
                self.curve_points = []
                self.preview_line = None
                self.redo_stack.clear()
                self.draw()
            return

        if self.current_line_start is None:
            self.current_line_start = point
        else:
            self.lines.append({"type": "line", "start": self.current_line_start, "end": point})
            self.current_line_start = point
            self.preview_line = None
            self.redo_stack.clear()
            self.draw()

    def on_mouse_move(self, event):
        if self.curve_mode.get() and self.curve_points:
            point = self.snap(*self.to_world(event.x, event.y))
            pts = self.curve_points + [point]
            if len(pts) == 2:
                self.preview_line = {"type": "line", "start": pts[0], "end": pts[1]}
            elif len(pts) == 3:
                self.preview_line = {
                    "type": "curve",
                    "start": pts[0],
                    "control": pts[1],
                    "end": pts[2],
                }
            self.draw()
            return

        if self.current_line_start:
            end = self.snap(*self.to_world(event.x, event.y))
            self.preview_line = {"type": "line", "start": self.current_line_start, "end": end}
            self.draw()

    def on_drag(self, event):
        if self.pan_mode.get() and self.panning:
            dx = event.x - self.last_pan[0]
            dy = event.y - self.last_pan[1]
            self.offset_x += dx
            self.offset_y += dy
            self.last_pan = (event.x, event.y)
            self.draw()

    def on_release(self, event):
        self.panning = False

    def start_pan(self, event):
        self.panning = True
        self.last_pan = (event.x, event.y)

    def do_pan(self, event):
        if self.panning:
            dx = event.x - self.last_pan[0]
            dy = event.y - self.last_pan[1]
            self.offset_x += dx
            self.offset_y += dy
            self.last_pan = (event.x, event.y)
            self.draw()

    def stop_pan(self, event):
        self.panning = False

    def cancel_drawing(self, event=None):
        self.current_line_start = None
        self.preview_line = None
        self.curve_points = []
        self.draw()

    def undo(self, event=None):
        if self.lines:
            self.redo_stack.append(self.lines.pop())
            self.draw()

    def redo(self, event=None):
        if self.redo_stack:
            self.lines.append(self.redo_stack.pop())
            self.draw()

    def draw_bezier(self, start, control, end, preview=False):
        points = []
        for t in [i / 20 for i in range(21)]:
            x = (1 - t) ** 2 * start[0] + 2 * (1 - t) * t * control[0] + t**2 * end[0]
            y = (1 - t) ** 2 * start[1] + 2 * (1 - t) * t * control[1] + t**2 * end[1]
            points.append(self.to_screen(x, y))
        color = "blue" if preview else "black"
        for i in range(len(points) - 1):
            self.canvas.create_line(
                *points[i], *points[i + 1], fill=color, width=2, dash=(4, 2) if preview else None
            )

    def enter_robot_mode(self):
        self.robot_mode = True
        self.robot_pose = None
        self.robot_preview_pos = None
        self.canvas.bind("<Motion>", self.robot_place_preview)
        self.canvas.bind("<Button-1>", self.on_click)
        self.draw()

    def robot_place_preview(self, event):
        self.robot_preview_pos = self.to_world(event.x, event.y)
        self.draw()

    def robot_rotate_preview(self, event):
        if not self.robot_pose:
            return
        pos = self.robot_pose[0]
        mouse_pos = self.to_world(event.x, event.y)
        dx = mouse_pos[0] - pos[0]
        dy = mouse_pos[1] - pos[1]
        angle = math.atan2(dy, dx)
        self.robot_pose = (pos, (math.cos(angle), math.sin(angle)))
        self.draw()

    def confirm_robot_orientation(self, event):
        self.canvas.unbind("<Motion>")
        self.canvas.unbind("<Button-1>")
        self.canvas.bind("<Button-1>", self.on_click)
        self.canvas.bind("<Motion>", self.on_mouse_move)
        self.robot_mode = False
        self.robot_preview_pos = None
        self.draw()

    def draw_robot(self, pos, dir_vec):
        x, y = self.to_screen(*pos)
        dx, dy = dir_vec
        length = self.zoomed(10)
        ex = x + dx * length
        ey = y + dy * length
        self.canvas.create_line(x, y, ex, ey, fill="red", width=3, arrow=tk.LAST)

    def save_scenario(self):
        path = filedialog.asksaveasfilename(defaultextension=".json")
        if not path:
            return
        robot_pose = (
            {
                "position": {
                    "x": self.robot_pose[0][0] / 100,
                    "y": self.robot_pose[0][1] / 100,
                    "z": 0,
                },
                "rotation": {"w": 1.0, "x": 0.0, "y": 0.0, "z": 0.0},
            }
            if self.robot_pose
            else None
        )

        track_lines = []
        for line in self.lines:
            if line["type"] == "line":
                track_lines.append(
                    {
                        "start": {"x": line["start"][0] / 100, "y": line["start"][1] / 100, "z": 0},
                        "end": {"x": line["end"][0] / 100, "y": line["end"][1] / 100, "z": 0},
                        "width": 10.0,
                        "whiteness": 0.0,
                        "visible": True,
                    }
                )
            elif line["type"] == "curve":
                for t in [i / 20 for i in range(20)]:
                    x1 = (
                        (1 - t) ** 2 * line["start"][0]
                        + 2 * (1 - t) * t * line["control"][0]
                        + t**2 * line["end"][0]
                    )
                    y1 = (
                        (1 - t) ** 2 * line["start"][1]
                        + 2 * (1 - t) * t * line["control"][1]
                        + t**2 * line["end"][1]
                    )
                    t2 = t + 0.05
                    x2 = (
                        (1 - t2) ** 2 * line["start"][0]
                        + 2 * (1 - t2) * t2 * line["control"][0]
                        + t2**2 * line["end"][0]
                    )
                    y2 = (
                        (1 - t2) ** 2 * line["start"][1]
                        + 2 * (1 - t2) * t2 * line["control"][1]
                        + t2**2 * line["end"][1]
                    )
                    track_lines.append(
                        {
                            "start": {"x": x1 / 100, "y": y1 / 100, "z": 0},
                            "end": {"x": x2 / 100, "y": y2 / 100, "z": 0},
                            "width": 10.0,
                            "whiteness": 0.0,
                            "visible": True,
                        }
                    )

        scenario = {
            "robot_initial_pose": robot_pose,
            "track_segments": [
                {
                    "pose": {
                        "position": {"x": 0, "y": 0, "z": 0},
                        "rotation": {"w": 1.0, "x": 0, "y": 0, "z": 0},
                    },
                    "track_lines": track_lines,
                }
            ],
        }

        with open(path, "w") as f:
            json.dump(scenario, f, indent=4)

    def load_scenario(self):
        path = filedialog.askopenfilename(filetypes=[("JSON Files", "*.json")])
        if not path:
            return
        with open(path) as f:
            data = json.load(f)

        self.lines.clear()
        self.curve_points.clear()
        self.current_line_start = None
        self.preview_line = None
        self.redo_stack.clear()

        robot = data.get("robot_initial_pose")
        if robot:
            pos = robot["position"]
            self.robot_pose = ((pos["x"] * 100, pos["y"] * 100), (1.0, 0.0))
        else:
            self.robot_pose = None

        for seg in data["track_segments"]:
            for line in seg["track_lines"]:
                start = line["start"]
                end = line["end"]
                self.lines.append(
                    {
                        "type": "line",
                        "start": (start["x"] * 100, start["y"] * 100),
                        "end": (end["x"] * 100, end["y"] * 100),
                    }
                )

        self.draw()

    def center_view(self):
        if not self.lines and not self.robot_pose:
            return

        xs, ys = [], []
        for line in self.lines:
            for pt in ["start", "end"] if line["type"] == "line" else ["start", "control", "end"]:
                xs.append(line[pt][0])
                ys.append(line[pt][1])

        if self.robot_pose:
            xs.append(self.robot_pose[0][0])
            ys.append(self.robot_pose[0][1])

        if not xs or not ys:
            return

        center_x = (min(xs) + max(xs)) / 2
        center_y = (min(ys) + max(ys)) / 2
        w, h = int(self.canvas.winfo_width()), int(self.canvas.winfo_height())

        self.offset_x = w / 2 - self.zoomed(center_x)
        self.offset_y = h / 2 - self.zoomed(center_y)
        self.draw()


if __name__ == "__main__":
    root = tk.Tk()
    app = LineEditor(root)
    root.mainloop()
