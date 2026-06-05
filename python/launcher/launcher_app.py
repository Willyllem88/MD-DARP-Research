"""
launcher_app.py -- Main DARPLauncherApp: integrates generator, solver, and viewer
into a single tabbed workflow.
"""

import os
import sys
import glob
import datetime
import subprocess
import tkinter as tk
from tkinter import ttk, filedialog, messagebox
from typing import Optional

import config
import runner
from widgets import (
    apply_dark_theme,
    LabeledEntry, CheckRow, SectionHeader, LogPane,
    DARK_BG, PANEL_BG, CARD_BG, ACCENT, ACCENT2,
    WARN, DANGER, TEXT, MUTED, BORDER,
    FONT_UI, FONT_LABEL, FONT_TITLE, FONT_HEADER, FONT_MONO,
)


class DARPLauncherApp:
    """
    Three-tab application:
      - Generate  -- launch the interactive instance generator
      - Solve     -- configure and run the C++ solver
      - View      -- open any result JSON in the route visualiser
    """

    def __init__(self, root: tk.Tk) -> None:
        self.root = root
        self.root.title("MD-DARP Launcher")
        self.root.geometry("1000x720")
        self.root.resizable(True, True)
        self.root.configure(bg=DARK_BG)

        apply_dark_theme(root)
        os.makedirs(config.TMP_DIR, exist_ok=True)

        # Shared state
        self._current_instance: Optional[str] = None   # path to selected .json instance
        self._current_result:   Optional[str] = None   # path to last solver output
        self._solver_proc: Optional[subprocess.Popen] = None

        self._build_header()
        self._build_notebook()
        self._build_status_bar()

    # ── Header ─────────────────────────────────────────────────────────────────

    def _build_header(self) -> None:
        hdr = tk.Frame(self.root, bg=PANEL_BG, pady=10)
        hdr.pack(fill=tk.X)

        tk.Label(
            hdr, text="MD-DARP LAUNCHER",
            font=("Consolas", 15, "bold"),
            bg=PANEL_BG, fg=ACCENT,
        ).pack(side=tk.LEFT, padx=18)

        tk.Label(
            hdr, text="Multi-Depot Dial-A-Ride · Integrated Workflow",
            font=FONT_LABEL, bg=PANEL_BG, fg=MUTED,
        ).pack(side=tk.LEFT, padx=4)

        # Quick-path readout
        self._hdr_instance_var = tk.StringVar(value="No instance selected")
        tk.Label(
            hdr, textvariable=self._hdr_instance_var,
            font=FONT_LABEL, bg=PANEL_BG, fg=ACCENT2,
            anchor="e",
        ).pack(side=tk.RIGHT, padx=18)

    # ── Notebook / Tabs ────────────────────────────────────────────────────────

    def _build_notebook(self) -> None:
        nb = ttk.Notebook(self.root)
        nb.pack(fill=tk.BOTH, expand=True, padx=10, pady=(6, 0))

        self._tab_gen   = ttk.Frame(nb, style="TFrame")
        self._tab_solve = ttk.Frame(nb, style="TFrame")
        self._tab_view  = ttk.Frame(nb, style="TFrame")

        nb.add(self._tab_gen,   text="  (1) Generate  ")
        nb.add(self._tab_solve, text="  (2) Solve  ")
        nb.add(self._tab_view,  text="  (3) View  ")

        self._build_gen_tab()
        self._build_solve_tab()
        self._build_view_tab()

        self._notebook = nb

    # ── Status bar ─────────────────────────────────────────────────────────────

    def _build_status_bar(self) -> None:
        bar = tk.Frame(self.root, bg=PANEL_BG, height=24)
        bar.pack(fill=tk.X, side=tk.BOTTOM)
        self._status_var = tk.StringVar(value="Ready.")
        tk.Label(
            bar, textvariable=self._status_var,
            bg=PANEL_BG, fg=MUTED, font=FONT_LABEL, anchor="w",
        ).pack(side=tk.LEFT, padx=10)

    def _set_status(self, msg: str) -> None:
        self._status_var.set(msg)

    # ══════════════════════════════════════════════════════════════════════════
    # TAB 1 -- GENERATE
    # ══════════════════════════════════════════════════════════════════════════

    def _build_gen_tab(self) -> None:
        p = self._tab_gen
        pad = {"padx": 16, "pady": 6}

        # ── Description card ──────────────────────────────────────────────────
        desc = tk.Frame(p, bg=CARD_BG, padx=14, pady=10)
        desc.pack(fill=tk.X, **pad)
        tk.Label(desc, text="Instance Generator",
                 font=FONT_HEADER, bg=CARD_BG, fg=ACCENT).pack(anchor="w")
        tk.Label(
            desc,
            text=(
                "Opens the interactive map-based generator.\n\n"
                "The produced instance JSON is saved to the tmp/ folder and automatically selected as the active instance, to \n"
                "preoceed with solving ( (2) Solve tab).\n\n"
                "Alternatively, you can load any existing (.json) instance from disk and skip the generation step."

            ),
            font=FONT_LABEL, bg=CARD_BG, fg=MUTED, justify="left",
        ).pack(anchor="w", pady=(4, 0))

        # ── Output name ───────────────────────────────────────────────────────
        row = tk.Frame(p, bg=CARD_BG, padx=14, pady=10)
        row.pack(fill=tk.X, **pad)
        tk.Label(row, text="Output filename:", font=FONT_UI,
                 bg=CARD_BG, fg=TEXT, width=18, anchor="w").pack(side=tk.LEFT)
        self._gen_filename = tk.StringVar(value=self._default_instance_name())
        tk.Entry(row, textvariable=self._gen_filename, width=32,
                 bg=CARD_BG, fg=TEXT, insertbackground=TEXT,
                 relief="flat", highlightthickness=1,
                 highlightbackground=BORDER, highlightcolor=ACCENT).pack(side=tk.LEFT, padx=(6, 4))
        tk.Label(row, text=".json", font=FONT_UI, bg=CARD_BG, fg=MUTED).pack(side=tk.LEFT)
        tk.Button(
            row, text="⟳ Refresh",
            command=lambda: self._gen_filename.set(self._default_instance_name()),
            bg=PANEL_BG, fg=MUTED, relief="flat", font=FONT_LABEL, cursor="hand2",
        ).pack(side=tk.LEFT, padx=(10, 0))

        # ── Launch button ─────────────────────────────────────────────────────
        ttk.Button(p, text="Launch Generator →", style="Accent.TButton",
                   command=self._launch_generator).pack(anchor="w", **pad)

        ttk.Separator(p, orient="horizontal").pack(fill=tk.X, padx=16, pady=8)

        # ── Select existing instance ──────────────────────────────────────────
        tk.Label(p, text="You can also load an already generated instance:",
                 font=FONT_LABEL, bg=DARK_BG, fg=MUTED).pack(anchor="w", padx=16)

        sel_row = tk.Frame(p, bg=DARK_BG)
        sel_row.pack(fill=tk.X, padx=16, pady=6)

        self._gen_sel_var = tk.StringVar(value="(none)")
        tk.Label(sel_row, textvariable=self._gen_sel_var,
                 font=FONT_LABEL, bg=DARK_BG, fg=ACCENT2, anchor="w",
                 width=55).pack(side=tk.LEFT)

        ttk.Button(sel_row, text="Browse…", style="Neutral.TButton",
                   command=self._browse_instance).pack(side=tk.LEFT, padx=(8, 0))

        # ── tmp file list ─────────────────────────────────────────────────────
        ttk.Separator(p, orient="horizontal").pack(fill=tk.X, padx=16, pady=8)
        tk.Label(p, text="Recent instances in tmp/",
                 font=FONT_LABEL, bg=DARK_BG, fg=MUTED).pack(anchor="w", padx=16)

        list_frame = tk.Frame(p, bg=CARD_BG, padx=6, pady=6)
        list_frame.pack(fill=tk.BOTH, expand=True, padx=16, pady=(4, 12))

        sb = tk.Scrollbar(list_frame, bg=PANEL_BG, troughcolor=DARK_BG)
        sb.pack(side=tk.RIGHT, fill=tk.Y)

        self._gen_listbox = tk.Listbox(
            list_frame,
            bg=CARD_BG, fg=TEXT, selectbackground=ACCENT,
            selectforeground=DARK_BG, font=FONT_MONO,
            relief="flat", borderwidth=0,
            yscrollcommand=sb.set,
        )
        self._gen_listbox.pack(fill=tk.BOTH, expand=True)
        sb.config(command=self._gen_listbox.yview)
        self._gen_listbox.bind("<Double-Button-1>", self._on_listbox_select)

        ttk.Button(p, text="↻  Refresh list", style="Neutral.TButton",
                   command=self._refresh_tmp_list).pack(anchor="w", padx=16, pady=(0, 10))

        self._refresh_tmp_list()

    def _default_instance_name(self) -> str:
        return "instance_" + datetime.datetime.now().strftime("%Y%m%d_%H%M%S")

    def _launch_generator(self) -> None:
        filename = self._gen_filename.get().strip()
        if not filename:
            messagebox.showerror("Error", "Please provide a filename.")
            return
        out_path = os.path.join(config.TMP_DIR, filename + ".json")
        self._set_status(f"Generator launched → output: {out_path}")

        def on_exit(rc: int) -> None:
            self.root.after(0, self._on_generator_exit, rc, out_path)

        runner.run_generator(config.GENERATOR_MAIN, out_path, on_exit=on_exit)

    def _on_generator_exit(self, rc: int, out_path: str) -> None:
        self._refresh_tmp_list()
        if rc == 0 and os.path.isfile(out_path):
            self._set_instance(out_path)
            self._set_status(f"Instance generated: {os.path.basename(out_path)}")
        else:
            self._set_status("Generator exited (no file saved).")

    def _browse_instance(self) -> None:
        path = filedialog.askopenfilename(
            title="Select instance JSON",
            filetypes=[("JSON files", "*.json"), ("All files", "*.*")],
            initialdir=config.TMP_DIR,
        )
        if path:
            self._set_instance(path)

    def _on_listbox_select(self, _event=None) -> None:
        sel = self._gen_listbox.curselection()
        if not sel:
            return
        name = self._gen_listbox.get(sel[0])
        path = os.path.join(config.TMP_DIR, name)
        self._set_instance(path)

    def _set_instance(self, path: str) -> None:
        self._current_instance = path
        short = os.path.basename(path)
        self._gen_sel_var.set(short)
        self._hdr_instance_var.set(f"Instance: {short}")
        # Pre-fill solver instance field
        self._solve_instance_var.set(path)
        self._set_status(f"Active instance: {short}")

    def _refresh_tmp_list(self) -> None:
        self._gen_listbox.delete(0, tk.END)
        files = sorted(
            glob.glob(os.path.join(config.TMP_DIR, "*.json")),
            key=os.path.getmtime, reverse=True,
        )
        for f in files:
            self._gen_listbox.insert(tk.END, os.path.basename(f))

    # ══════════════════════════════════════════════════════════════════════════
    # TAB 2 -- SOLVE
    # ══════════════════════════════════════════════════════════════════════════

    def _build_solve_tab(self) -> None:
        p = self._tab_solve
        outer_pad = {"padx": 16, "pady": 4}

        # Split into left-config and right-log
        panes = tk.PanedWindow(p, orient=tk.HORIZONTAL,
                               bg=DARK_BG, sashwidth=4, sashrelief="flat")
        panes.pack(fill=tk.BOTH, expand=True, padx=8, pady=8)

        left  = tk.Frame(panes, bg=DARK_BG)
        right = tk.Frame(panes, bg=DARK_BG)
        panes.add(left,  minsize=340, width=420)
        panes.add(right, minsize=280)

        # ── LEFT -- Configuration ───────────────────────────────────────────────

        # Instance selector
        inst_card = tk.Frame(left, bg=CARD_BG, padx=12, pady=8)
        inst_card.pack(fill=tk.X, pady=(0, 6))
        tk.Label(inst_card, text="Instance file", font=FONT_HEADER,
                 bg=CARD_BG, fg=ACCENT).pack(anchor="w")

        irow = tk.Frame(inst_card, bg=CARD_BG)
        irow.pack(fill=tk.X, pady=(4, 0))
        self._solve_instance_var = tk.StringVar(
            value=self._current_instance or "(none -- select in (1) Generate)"
        )
        tk.Entry(irow, textvariable=self._solve_instance_var,
                 bg=CARD_BG, fg=TEXT, insertbackground=TEXT,
                 relief="flat", highlightthickness=1,
                 highlightbackground=BORDER, highlightcolor=ACCENT,
                 font=FONT_LABEL, width=40).pack(side=tk.LEFT, expand=True, fill=tk.X)
        ttk.Button(irow, text="Browse…", style="Neutral.TButton",
                   command=self._browse_solve_instance).pack(side=tk.LEFT, padx=(6, 0))

        # Preset picker
        preset_card = tk.Frame(left, bg=CARD_BG, padx=12, pady=8)
        preset_card.pack(fill=tk.X, pady=(0, 6))
        tk.Label(preset_card, text="Preset", font=FONT_HEADER,
                 bg=CARD_BG, fg=ACCENT).pack(anchor="w")

        self._preset_var = tk.StringVar(value=config.DEFAULT_PRESET)
        cb = ttk.Combobox(
            preset_card, textvariable=self._preset_var,
            values=list(config.PRESETS.keys()), state="readonly", width=40,
        )
        cb.pack(fill=tk.X, pady=(4, 0))
        cb.bind("<<ComboboxSelected>>", self._on_preset_selected)

        self._preset_desc_var = tk.StringVar()
        tk.Label(preset_card, textvariable=self._preset_desc_var,
                 font=FONT_LABEL, bg=CARD_BG, fg=MUTED, wraplength=360,
                 justify="left", anchor="w").pack(fill=tk.X, pady=(4, 0))

        # Param card
        param_card = tk.Frame(left, bg=CARD_BG, padx=12, pady=8)
        param_card.pack(fill=tk.X, pady=(0, 6))
        tk.Label(param_card, text="Parameters", font=FONT_HEADER,
                 bg=CARD_BG, fg=ACCENT).pack(anchor="w", pady=(0, 6))

        self._method_var = tk.StringVar()
        self._time_var   = tk.StringVar()
        self._seed_var   = tk.StringVar()
        self._verbose_var = tk.BooleanVar()
        self._gice_var   = tk.BooleanVar()
        self._nr_var     = tk.BooleanVar()

        def _row(lbl, widget_fn):
            r = tk.Frame(param_card, bg=CARD_BG)
            r.pack(fill=tk.X, pady=2)
            tk.Label(r, text=lbl, font=FONT_LABEL, bg=CARD_BG, fg=MUTED,
                     width=22, anchor="w").pack(side=tk.LEFT)
            widget_fn(r)

        def _entry(parent, var, width=14):
            tk.Entry(parent, textvariable=var, width=width,
                     bg=PANEL_BG, fg=TEXT, insertbackground=TEXT,
                     relief="flat", highlightthickness=1,
                     highlightbackground=BORDER, highlightcolor=ACCENT,
                     font=FONT_UI).pack(side=tk.LEFT)

        def _method_cb(parent):
            ttk.Combobox(parent, textvariable=self._method_var,
                         values=config.METHODS, state="readonly", width=14).pack(side=tk.LEFT)

        _row("Method:", _method_cb)
        _row("Time limit (s):", lambda p: _entry(p, self._time_var))
        _row("Seed:", lambda p: _entry(p, self._seed_var, width=8))

        flag_row = tk.Frame(param_card, bg=CARD_BG)
        flag_row.pack(fill=tk.X, pady=(6, 2))
        for var, lbl in [
            (self._verbose_var, "Verbose"),
            (self._gice_var,    "GICE (for ALNS)"),
            (self._nr_var,      "NR (for ALNS)"),
        ]:
            tk.Checkbutton(
                flag_row, text=lbl, variable=var,
                bg=CARD_BG, fg=TEXT, selectcolor=CARD_BG,
                activebackground=CARD_BG, font=FONT_LABEL,
            ).pack(side=tk.LEFT, padx=(0, 12))

        # Output file
        out_card = tk.Frame(left, bg=CARD_BG, padx=12, pady=8)
        out_card.pack(fill=tk.X, pady=(4, 6))
        tk.Label(out_card, text="Result output", font=FONT_HEADER,
                 bg=CARD_BG, fg=ACCENT).pack(anchor="w")
        orow = tk.Frame(out_card, bg=CARD_BG)
        orow.pack(fill=tk.X, pady=(4, 0))
        self._solve_output_var = tk.StringVar(
            value=os.path.join(config.TMP_DIR, "result_latest.json")
        )
        tk.Entry(orow, textvariable=self._solve_output_var,
                 bg=CARD_BG, fg=TEXT, insertbackground=TEXT,
                 relief="flat", highlightthickness=1,
                 highlightbackground=BORDER, highlightcolor=ACCENT,
                 font=FONT_LABEL, width=40).pack(side=tk.LEFT, expand=True, fill=tk.X)
        ttk.Button(orow, text="Browse…", style="Neutral.TButton",
                   command=self._browse_solve_output).pack(side=tk.LEFT, padx=(6, 0))

        # Action buttons
        btn_row = tk.Frame(left, bg=DARK_BG)
        btn_row.pack(fill=tk.X, pady=8)
        self._solve_btn = ttk.Button(
            btn_row, text="▶  Run Solver", style="Accent.TButton",
            command=self._run_solver,
        )
        self._solve_btn.pack(side=tk.LEFT)
        self._stop_btn = ttk.Button(
            btn_row, text="■  Stop", style="Danger.TButton",
            command=self._stop_solver, state="disabled",
        )
        self._stop_btn.pack(side=tk.LEFT, padx=(10, 0))
        ttk.Button(
            btn_row, text="View Result →", style="Blue.TButton",
            command=self._solve_view_result,
        ).pack(side=tk.LEFT, padx=(10, 0))

        # ── RIGHT -- Log ────────────────────────────────────────────────────────
        tk.Label(right, text="Solver Output", font=FONT_HEADER,
                 bg=DARK_BG, fg=ACCENT).pack(anchor="w", pady=(0, 4))

        log_frame = tk.Frame(right, bg="#0f1117")
        log_frame.pack(fill=tk.BOTH, expand=True)

        self._solve_log = LogPane(log_frame, height=20)
        self._solve_log.setup_tags()
        self._solve_log.pack_with_scrollbar(fill=tk.BOTH, expand=True)

        ttk.Button(right, text="Clear log", style="Neutral.TButton",
                   command=self._solve_log.clear).pack(anchor="e", pady=(4, 0))

        # Apply default preset
        self._on_preset_selected()

    def _toggle_alns_panel(self) -> None:
        if self._alns_expanded:
            self._alns_panel.pack_forget()
            self._alns_toggle_btn.config(text="▶  ALNS Advanced Parameters")
        else:
            self._alns_panel.pack(fill=tk.X, pady=(0, 6))
            self._alns_toggle_btn.config(text="▼  ALNS Advanced Parameters")
        self._alns_expanded = not self._alns_expanded

    def _on_preset_selected(self, _event=None) -> None:
        name = self._preset_var.get()
        preset = config.PRESETS.get(name, config.PRESETS[config.DEFAULT_PRESET])
        self._preset_desc_var.set(preset["description"])
        self._method_var.set(preset["method"])
        tl = preset["time_limit"]
        self._time_var.set("" if tl is None else str(tl))
        self._seed_var.set(str(preset["seed"]))
        self._verbose_var.set(preset["verbose"])
        self._gice_var.set(preset["enable_gice"])
        self._nr_var.set(preset["enable_nr"])

    def _browse_solve_instance(self) -> None:
        path = filedialog.askopenfilename(
            title="Select instance JSON",
            filetypes=[("JSON files", "*.json"), ("All files", "*.*")],
            initialdir=config.TMP_DIR,
        )
        if path:
            self._solve_instance_var.set(path)
            self._set_instance(path)

    def _browse_solve_output(self) -> None:
        path = filedialog.asksaveasfilename(
            title="Save result JSON",
            defaultextension=".json",
            filetypes=[("JSON files", "*.json")],
            initialdir=config.TMP_DIR,
        )
        if path:
            self._solve_output_var.set(path)

    def _run_solver(self) -> None:
        instance = self._solve_instance_var.get().strip()
        if not instance or not os.path.isfile(instance):
            messagebox.showerror("Missing instance",
                                 "Please select a valid instance JSON file.")
            return

        method  = self._method_var.get()
        tl_str  = self._time_var.get().strip()
        tl      = float(tl_str) if tl_str else None
        seed    = int(self._seed_var.get()) if self._seed_var.get().strip() else 42
        verbose = self._verbose_var.get()
        gice    = self._gice_var.get()
        nr      = self._nr_var.get()
        output  = self._solve_output_var.get().strip()

        self._solve_log.clear()
        self._solve_log.append(f"[{_now()}] Starting solver…", "info")
        self._solve_log.append(f"  Instance : {instance}", "muted")
        self._solve_log.append(f"  Method   : {method}", "muted")
        self._solve_log.append(f"  Output   : {output}", "muted")

        self._solve_btn.config(state="disabled")
        self._stop_btn.config(state="normal")
        self._set_status("Solver running…")

        def on_line(line: str) -> None:
            tag = "ok" if "best" in line.lower() or "optimal" in line.lower() else None
            self.root.after(0, self._solve_log.append, line, tag)

        def on_finish(rc: int, out: str) -> None:
            self.root.after(0, self._on_solver_finish, rc, out)

        self._solver_proc = runner.run_solver(
            config.SOLVER_BIN, instance, output,
            method, tl, seed, verbose, gice, nr,
            on_stdout_line=on_line, on_finish=on_finish,
        )

    def _stop_solver(self) -> None:
        if self._solver_proc and self._solver_proc.poll() is None:
            self._solver_proc.terminate()
            self._solve_log.append(f"[{_now()}] Solver terminated by user.", "warn")

    def _on_solver_finish(self, rc: int, output: str) -> None:
        self._solve_btn.config(state="normal")
        self._stop_btn.config(state="disabled")
        if rc == 0:
            self._current_result = output
            self._solve_log.append(f"[{_now()}] Solver finished successfully.", "ok")
            self._set_status(f"Done -- result: {os.path.basename(output)}")
        else:
            tag = "error" if rc != 0 else "warn"
            self._solve_log.append(f"[{_now()}] Solver exited with code {rc}.", tag)
            self._set_status(f"Solver exited (code {rc}).")

    def _solve_view_result(self) -> None:
        result = self._current_result or self._solve_output_var.get().strip()
        if not result or not os.path.isfile(result):
            messagebox.showinfo("No result",
                                "No result file found. Run the solver first, "
                                "or go to the (3) View tab.")
            return
        self._open_viewer(result)

    # ══════════════════════════════════════════════════════════════════════════
    # TAB 3 -- VIEW
    # ══════════════════════════════════════════════════════════════════════════

    def _build_view_tab(self) -> None:
        p = self._tab_view
        pad = {"padx": 16, "pady": 6}

        # Description
        desc = tk.Frame(p, bg=CARD_BG, padx=14, pady=10)
        desc.pack(fill=tk.X, **pad)
        tk.Label(desc, text="Route Visualiser",
                 font=FONT_HEADER, bg=CARD_BG, fg=ACCENT).pack(anchor="w")
        tk.Label(
            desc,
            text=(
                "Open any solver result JSON to visualise routes on the real road network.\n"
                "You can load a result independently -- no need to run the solver first."
            ),
            font=FONT_LABEL, bg=CARD_BG, fg=MUTED, justify="left",
        ).pack(anchor="w", pady=(4, 0))

        # File selector
        file_card = tk.Frame(p, bg=CARD_BG, padx=14, pady=10)
        file_card.pack(fill=tk.X, **pad)
        tk.Label(file_card, text="Result JSON file",
                 font=FONT_HEADER, bg=CARD_BG, fg=ACCENT).pack(anchor="w")

        frow = tk.Frame(file_card, bg=CARD_BG)
        frow.pack(fill=tk.X, pady=(6, 0))

        self._view_file_var = tk.StringVar(value="(none selected)")
        tk.Entry(frow, textvariable=self._view_file_var,
                 bg=CARD_BG, fg=TEXT, insertbackground=TEXT,
                 relief="flat", highlightthickness=1,
                 highlightbackground=BORDER, highlightcolor=ACCENT,
                 font=FONT_LABEL, width=50).pack(side=tk.LEFT, expand=True, fill=tk.X)
        ttk.Button(frow, text="Browse…", style="Neutral.TButton",
                   command=self._browse_view_file).pack(side=tk.LEFT, padx=(8, 0))

        # Auto-fill from solver result
        ttk.Button(
            file_card, text="← Use last solver result",
            style="Neutral.TButton",
            command=self._view_use_last_result,
        ).pack(anchor="w", pady=(6, 0))

        # Mode selector
        mode_card = tk.Frame(p, bg=CARD_BG, padx=14, pady=10)
        mode_card.pack(fill=tk.X, **pad)
        tk.Label(mode_card, text="Drawing mode",
                 font=FONT_HEADER, bg=CARD_BG, fg=ACCENT).pack(anchor="w")

        self._view_mode_var = tk.StringVar(value="streets")
        modes = [
            ("streets", "Streets  -- trace shortest path on real road network (slower)"),
            ("direct",  "Direct   -- straight arcs between stops (fast)"),
        ]
        for val, lbl in modes:
            tk.Radiobutton(
                mode_card, text=lbl, variable=self._view_mode_var, value=val,
                bg=CARD_BG, fg=TEXT, selectcolor=CARD_BG,
                activebackground=CARD_BG, font=FONT_LABEL,
            ).pack(anchor="w", pady=1)

        ttk.Separator(p, orient="horizontal").pack(fill=tk.X, padx=16, pady=10)

        # Recent results list
        tk.Label(p, text="Recent results in tmp/",
                 font=FONT_LABEL, bg=DARK_BG, fg=MUTED).pack(anchor="w", padx=16)

        rlist_frame = tk.Frame(p, bg=CARD_BG, padx=6, pady=6)
        rlist_frame.pack(fill=tk.BOTH, expand=True, padx=16, pady=(4, 6))

        rsb = tk.Scrollbar(rlist_frame, bg=PANEL_BG, troughcolor=DARK_BG)
        rsb.pack(side=tk.RIGHT, fill=tk.Y)
        self._view_listbox = tk.Listbox(
            rlist_frame,
            bg=CARD_BG, fg=TEXT, selectbackground=ACCENT,
            selectforeground=DARK_BG, font=FONT_MONO,
            relief="flat", borderwidth=0,
            yscrollcommand=rsb.set,
        )
        self._view_listbox.pack(fill=tk.BOTH, expand=True)
        rsb.config(command=self._view_listbox.yview)
        self._view_listbox.bind("<Double-Button-1>", self._on_view_listbox_select)

        btn_row = tk.Frame(p, bg=DARK_BG)
        btn_row.pack(fill=tk.X, padx=16, pady=(0, 10))

        ttk.Button(btn_row, text="↻  Refresh list", style="Neutral.TButton",
                   command=self._refresh_view_list).pack(side=tk.LEFT)
        ttk.Button(btn_row, text="Open Visualiser →", style="Blue.TButton",
                   command=self._open_selected_result).pack(side=tk.LEFT, padx=(12, 0))

        self._refresh_view_list()

    def _browse_view_file(self) -> None:
        path = filedialog.askopenfilename(
            title="Select result JSON",
            filetypes=[("JSON files", "*.json"), ("All files", "*.*")],
            initialdir=config.TMP_DIR,
        )
        if path:
            self._view_file_var.set(path)

    def _view_use_last_result(self) -> None:
        if self._current_result and os.path.isfile(self._current_result):
            self._view_file_var.set(self._current_result)
        else:
            out = self._solve_output_var.get().strip()
            if out and os.path.isfile(out):
                self._view_file_var.set(out)
            else:
                messagebox.showinfo("No result", "No solver result available yet.")

    def _on_view_listbox_select(self, _event=None) -> None:
        sel = self._view_listbox.curselection()
        if not sel:
            return
        name = self._view_listbox.get(sel[0])
        path = os.path.join(config.TMP_DIR, name)
        self._view_file_var.set(path)

    def _open_selected_result(self) -> None:
        path = self._view_file_var.get().strip()
        if not path or not os.path.isfile(path):
            messagebox.showerror("File not found",
                                 "Please select a valid result JSON file.")
            return
        self._open_viewer(path)

    def _refresh_view_list(self) -> None:
        self._view_listbox.delete(0, tk.END)
        files = sorted(
            glob.glob(os.path.join(config.TMP_DIR, "*.json")),
            key=os.path.getmtime, reverse=True,
        )
        for f in files:
            self._view_listbox.insert(tk.END, os.path.basename(f))

    def _open_viewer(self, path: str) -> None:
        mode = self._view_mode_var.get() if hasattr(self, "_view_mode_var") else "streets"
        self._set_status(f"Viewer launched for {os.path.basename(path)}")

        def on_exit(rc: int) -> None:
            self.root.after(0, self._set_status, "Viewer closed.")

        runner.run_viewer(config.VIEWER_MAIN, path, mode=mode, on_exit=on_exit)


# ── Helpers ────────────────────────────────────────────────────────────────────

def _now() -> str:
    return datetime.datetime.now().strftime("%H:%M:%S")
