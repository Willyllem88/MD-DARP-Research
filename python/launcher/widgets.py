"""
widgets.py -- Reusable UI components for the DARP Launcher.
"""

import tkinter as tk
from tkinter import ttk
from typing import Optional


# ── Colour palette ─────────────────────────────────────────────────────────────
MAIN_BG       = "#f5f7fa"   # fondo principal
PANEL_BG      = "#ffffff"   # paneles
CARD_BG       = "#eef2f7"   # tarjetas
ACCENT        = "#00b894"   # teal-green
ACCENT2       = "#3b82f6"   # blue
WARN          = "#f59e0b"   # amber
DANGER        = "#dc2626"   # red
TEXT          = "#1f2937"   # texto principal
MUTED         = "#6b7280"   # texto secundario
BORDER        = "#d1d5db"   # bordes
LOG_BG        = "#ffffff"
LOG_TEXT      = "#111827"
FONT_MONO     = ("Consolas", 9)
FONT_UI       = ("Segoe UI", 10)
FONT_LABEL    = ("Segoe UI", 9)
FONT_TITLE    = ("Segoe UI", 13, "bold")
FONT_HEADER   = ("Segoe UI", 11, "bold")


def apply_theme(root: tk.Tk) -> None:
    """Configure ttk styles for theme."""
    style = ttk.Style(root)
    style.theme_use("clam")

    style.configure(".",
        background=MAIN_BG, foreground=TEXT,
        fieldbackground=CARD_BG, troughcolor=PANEL_BG,
        selectbackground=ACCENT, selectforeground=MAIN_BG,
        font=FONT_UI,
    )
    style.configure("TFrame", background=MAIN_BG)
    style.configure("Card.TFrame", background=CARD_BG)
    style.configure("Panel.TFrame", background=PANEL_BG)

    style.configure("TLabel", background=MAIN_BG, foreground=TEXT, font=FONT_UI)
    style.configure("Muted.TLabel", background=MAIN_BG, foreground=MUTED, font=FONT_LABEL)
    style.configure("Card.TLabel", background=CARD_BG, foreground=TEXT, font=FONT_UI)
    style.configure("Header.TLabel", background=MAIN_BG, foreground=TEXT, font=FONT_HEADER)
    style.configure("Title.TLabel", background=MAIN_BG, foreground=ACCENT, font=FONT_TITLE)
    style.configure("Accent.TLabel", background=CARD_BG, foreground=ACCENT, font=FONT_UI)
    style.configure("Warn.TLabel", background=CARD_BG, foreground=WARN, font=FONT_LABEL)

    style.configure("TEntry",
        fieldbackground=CARD_BG, foreground=TEXT,
        insertcolor=TEXT, bordercolor=BORDER, relief="flat",
    )
    style.map("TEntry", bordercolor=[("focus", ACCENT)])

    style.configure("TCombobox",
        fieldbackground=CARD_BG, foreground=TEXT,
        selectbackground=CARD_BG, selectforeground=TEXT,
        arrowcolor=MUTED,
    )
    style.map("TCombobox",
        fieldbackground=[("readonly", CARD_BG)],
        foreground=[("readonly", TEXT)],
    )

    style.configure("TCheckbutton",
        background=CARD_BG, foreground=TEXT,
        indicatorcolor=CARD_BG,
    )
    style.map("TCheckbutton",
        indicatorcolor=[("selected", ACCENT), ("!selected", BORDER)],
    )

    style.configure("TSpinbox",
        fieldbackground=CARD_BG, foreground=TEXT,
        insertcolor=TEXT, arrowcolor=MUTED,
    )

    style.configure("TNotebook", background=MAIN_BG, borderwidth=0)
    style.configure("TNotebook.Tab",
        background=PANEL_BG, foreground=MUTED,
        padding=(14, 6), font=FONT_UI,
    )
    style.map("TNotebook.Tab",
        background=[("selected", CARD_BG)],
        foreground=[("selected", ACCENT)],
    )

    style.configure("TSeparator", background=BORDER)
    style.configure("TScrollbar",
        background=PANEL_BG, troughcolor=MAIN_BG,
        arrowcolor=MUTED, bordercolor=MAIN_BG,
    )

    # Custom button styles
    for name, bg, fg, abg in [
        ("Accent.TButton",  ACCENT,   MAIN_BG, "#00b899"),
        ("Blue.TButton",    ACCENT2,  MAIN_BG, "#3a7aed"),
        ("Danger.TButton",  DANGER,   "#fff",  "#c94444"),
        ("Neutral.TButton", BORDER,   TEXT,    "#454d5e"),
    ]:
        style.configure(name, background=bg, foreground=fg,
                        font=("Segoe UI", 10, "bold"), relief="flat",
                        padding=(10, 6), borderwidth=0)
        style.map(name,
            background=[("active", abg), ("disabled", PANEL_BG)],
            foreground=[("disabled", MUTED)],
        )


# ── Composite widgets ──────────────────────────────────────────────────────────

class LabeledEntry(ttk.Frame):
    """A label + entry pair with optional units suffix."""

    def __init__(self, parent, label: str, default: str = "",
                 unit: str = "", width: int = 12, **kwargs):
        super().__init__(parent, style="Card.TFrame", **kwargs)
        ttk.Label(self, text=label, style="Card.TLabel", width=26, anchor="w").pack(side=tk.LEFT)
        self.var = tk.StringVar(value=default)
        ttk.Entry(self, textvariable=self.var, width=width).pack(side=tk.LEFT, padx=(4, 0))
        if unit:
            ttk.Label(self, text=unit, style="Muted.TLabel").pack(side=tk.LEFT, padx=(4, 0))

    @property
    def value(self) -> str:
        return self.var.get().strip()

    @value.setter
    def value(self, v: str) -> None:
        self.var.set(v)


class CheckRow(ttk.Frame):
    """A checkbutton with an inline description."""

    def __init__(self, parent, label: str, tooltip: str = "", default: bool = False, **kwargs):
        super().__init__(parent, style="Card.TFrame", **kwargs)
        self.var = tk.BooleanVar(value=default)
        cb = ttk.Checkbutton(self, variable=self.var, style="TCheckbutton", text=label)
        cb.pack(side=tk.LEFT)
        if tooltip:
            ttk.Label(self, text=tooltip, style="Warn.TLabel").pack(side=tk.LEFT, padx=(8, 0))

    @property
    def value(self) -> bool:
        return self.var.get()

    @value.setter
    def value(self, v: bool) -> None:
        self.var.set(v)


class SectionHeader(ttk.Frame):
    """Styled section separator with title."""

    def __init__(self, parent, title: str, **kwargs):
        bg = kwargs.pop("bg", CARD_BG)
        super().__init__(parent, style="Card.TFrame", **kwargs)
        ttk.Label(self, text=title, style="Accent.TLabel").pack(side=tk.LEFT)
        ttk.Separator(self, orient="horizontal").pack(side=tk.LEFT, fill=tk.X, expand=True, padx=(8, 0))


class LogPane(tk.Text):
    """Scrollable, read-only log widget."""

    def __init__(self, parent, **kwargs):
        kwargs.setdefault("bg", LOG_BG)
        kwargs.setdefault("fg", LOG_TEXT)
        kwargs.setdefault("font", FONT_MONO)
        kwargs.setdefault("relief", "flat")
        kwargs.setdefault("borderwidth", 0)
        kwargs.setdefault("wrap", "word")
        super().__init__(parent, state="disabled", **kwargs)
        self._sb = ttk.Scrollbar(parent, orient="vertical", command=self.yview)
        self.configure(yscrollcommand=self._sb.set)

    def pack_with_scrollbar(self, **kwargs):
        self._sb.pack(side=tk.RIGHT, fill=tk.Y)
        self.pack(**kwargs)

    def append(self, text: str, tag: Optional[str] = None) -> None:
        self.configure(state="normal")
        if tag:
            self.insert(tk.END, text + "\n", tag)
        else:
            self.insert(tk.END, text + "\n")
        self.see(tk.END)
        self.configure(state="disabled")

    def clear(self) -> None:
        self.configure(state="normal")
        self.delete("1.0", tk.END)
        self.configure(state="disabled")

    def setup_tags(self) -> None:
        self.tag_configure("ok",    foreground=ACCENT)
        self.tag_configure("warn",  foreground=WARN)
        self.tag_configure("error", foreground=DANGER)
        self.tag_configure("info",  foreground=ACCENT2)
        self.tag_configure("muted", foreground=MUTED)
