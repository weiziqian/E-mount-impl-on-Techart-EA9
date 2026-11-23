"""Community EA9 Firmware Updater for macOS."""

from concurrent.futures import ThreadPoolExecutor
from queue import SimpleQueue
import tkinter as tk
from tkinter import messagebox, ttk

import gen_optical

from firmware import APERTURES, DEFAULT_NAME, FOCAL_PRESETS, Firmware, normalize_slots, resource_dir
from ea9_usb import Connection

TITLE = "Community EA9 Firmware Updater"


class LensRow:
    def __init__(self, parent, row, label, values):
        focal, name, pupil = values
        self.focal = tk.StringVar(value=str(focal) if focal else "")
        self.name = tk.StringVar(value=name)
        self.pupil = tk.StringVar(value=str(pupil) if pupil else "")
        ttk.Label(parent, text=label).grid(row=row, column=0, sticky="w", pady=3)
        ttk.Combobox(parent, textvariable=self.focal, values=FOCAL_PRESETS, width=8).grid(
            row=row, column=1, sticky="ew", padx=6)
        ttk.Entry(parent, textvariable=self.name, width=30).grid(
            row=row, column=2, sticky="ew", padx=6)
        ttk.Combobox(parent, textvariable=self.pupil, values=gen_optical.XPD_CHOICES,
                     state="readonly", width=10).grid(row=row, column=3, sticky="ew", padx=6)
        self.focal.trace_add("write", self.suggest_pupil)

    def suggest_pupil(self, *_):
        digits = self.focal.get().strip().lstrip("0")
        if 0 < len(digits) <= 3 and digits.isascii() and digits.isdecimal():
            self.pupil.set(str(gen_optical.auto_xpd(int(digits))))
        else:
            self.pupil.set("")

    def values(self):
        return self.focal.get(), self.name.get(), self.pupil.get()


class Updater:
    def __init__(self, root, firmware, changelog):
        self.root = root
        self.firmware = firmware
        self.changelog = changelog
        self.connection = Connection()
        self.executor = ThreadPoolExecutor(max_workers=1)
        self.progress = SimpleQueue()
        self.probe = None
        self.update = None
        self.confirming = False
        self.path = None
        self.next_poll = root.after(0, self.poll)

        root.title(TITLE)
        root.protocol("WM_DELETE_WINDOW", self.close)
        if root.tk.call("tk", "windowingsystem") == "aqua":
            root.createcommand("tk::mac::Quit", self.close)
        root.columnconfigure(0, weight=1)
        panel = ttk.Frame(root, padding=18)
        panel.grid(sticky="nsew")
        panel.columnconfigure(0, weight=1)

        top = ttk.Frame(panel)
        top.grid(sticky="ew", pady=(0, 12))
        ttk.Label(top, text=f"Firmware {firmware.version_text}").pack(side="left")
        ttk.Button(top, text="Release Notes", command=self.show_notes).pack(side="right")

        hardware = ttk.LabelFrame(panel, text="Adapter", padding=10)
        hardware.grid(sticky="ew", pady=(0, 12))
        self.hardware = tk.StringVar(value=str(firmware.hardware))
        ttk.Label(hardware, text="Hardware version").pack(side="left", padx=(0, 8))
        ttk.Combobox(hardware, textvariable=self.hardware, values=(2, 1),
                     state="readonly", width=4).pack(side="left")
        ttk.Button(hardware, text="Which version?", command=self.show_hardware_help).pack(side="right")

        settings = ttk.LabelFrame(panel, text="Lenses", padding=10)
        settings.grid(sticky="ew", pady=(0, 12))
        settings.columnconfigure(0, weight=1)
        modes = ttk.Frame(settings)
        modes.grid(row=0, column=0, sticky="ew", pady=(0, 8))
        self.mode = tk.StringVar(value="single")
        for value, label in (("single", "Single lens"), ("multiple", "Multiple lenses")):
            ttk.Radiobutton(modes, text=label, value=value, variable=self.mode,
                            command=self.show_mode).pack(side="left", padx=(0, 12))
        ttk.Button(modes, text="Lens setup help", command=self.show_lens_help).pack(side="right")

        self.single_panel = ttk.Frame(settings)
        self.multi_panel = ttk.Frame(settings)
        for group in (self.single_panel, self.multi_panel):
            group.columnconfigure(2, weight=1)
            for col, label in enumerate(("", "Focal length (mm)", "Lens name", "Exit pupil (mm)")):
                ttk.Label(group, text=label).grid(row=0, column=col, padx=(0 if col == 0 else 6, 6),
                                                   sticky="w")

        self.single = LensRow(self.single_panel, 1, "Lens", (50, DEFAULT_NAME, gen_optical.auto_xpd(50)))
        self.multiple = [
            LensRow(self.multi_panel, i + 1, aperture, slot)
            for i, (aperture, slot) in enumerate(zip(APERTURES, firmware.slots))
        ]
        self.show_mode()

        self.status = tk.StringVar(value="Looking for the adapter…")
        ttk.Label(panel, textvariable=self.status, wraplength=610).grid(sticky="w", pady=(0, 6))
        self.bar = ttk.Progressbar(panel)
        self.bar.grid(sticky="ew", pady=(0, 10))
        self.write_button = ttk.Button(panel, text="Write Firmware", command=self.confirm,
                                       state="disabled")
        self.write_button.grid(sticky="e")

    def show_mode(self):
        self.single_panel.grid_remove()
        self.multi_panel.grid_remove()
        panel = self.multi_panel if self.mode.get() == "multiple" else self.single_panel
        panel.grid(row=1, column=0, sticky="ew")

    def show_notes(self):
        dialog = tk.Toplevel(self.root)
        dialog.title("Release Notes")
        dialog.transient(self.root)
        dialog.columnconfigure(0, weight=1)
        dialog.rowconfigure(0, weight=1)
        notes = tk.Text(dialog, wrap="word", width=64, height=22, padx=12, pady=12)
        notes.insert("1.0", self.changelog)
        notes.configure(state="disabled")
        scrollbar = ttk.Scrollbar(dialog, command=notes.yview)
        notes.configure(yscrollcommand=scrollbar.set)
        notes.grid(row=0, column=0, sticky="nsew")
        scrollbar.grid(row=0, column=1, sticky="ns")
        ttk.Button(dialog, text="Close", command=dialog.destroy).grid(row=1, column=0, pady=8)

    def show_hardware_help(self):
        messagebox.showinfo(
            "Adapter Hardware",
            "Version 1 has a continuous metal spring strip. Version 2 has a separate "
            "triangular block. Start with version 2. Select version 1 if you have "
            "version 1 hardware and it cannot focus at infinity.",
            parent=self.root,
        )

    def show_lens_help(self):
        messagebox.showinfo(
            "Lens Setup",
            "Enter each lens's focal length and a name for the camera to show. "
            "Keep the suggested exit pupil unless you know the value for your lens. "
            "You can try nearby values if focus is consistently off toward the edges.\n\n"
            "Use a single lens to keep the camera at a fixed aperture. To switch between "
            "configured lenses, set the camera to the aperture shown next to the lens "
            "and take a picture. Then return the camera to its widest aperture. "
            "Clear the focal length of any slot you want to leave empty.",
            parent=self.root,
        )

    def poll(self):
        if self.update is not None or self.confirming:
            self.next_poll = self.root.after(100, self.poll)
        elif self.probe is None:
            self.probe = self.executor.submit(self.connection.poll)
            self.next_poll = self.root.after(100, self.poll)
        elif self.probe.done():
            try:
                self.path, detail = self.probe.result()
            except OSError as error:
                self.path, detail = None, f"USB detection failed. {error}"
            self.probe = None
            self.status.set(detail)
            self.write_button.configure(state="normal" if self.path else "disabled")
            self.next_poll = self.root.after(1500, self.poll)
        else:
            self.next_poll = self.root.after(100, self.poll)

    def confirm(self):
        if self.path is None or self.update is not None or self.confirming:
            return
        multiple = self.mode.get() == "multiple"
        fields = self.multiple if multiple else [self.single]
        hardware = int(self.hardware.get())
        try:
            slots = normalize_slots([field.values() for field in fields], multiple)
            image = self.firmware.patch(hardware, slots)
        except ValueError as error:
            messagebox.showerror("Check Lens Settings", str(error), parent=self.root)
            return
        count = sum(bool(mm) for mm, _, _ in slots)
        summary = []
        for aperture, (mm, name, pupil) in zip(APERTURES, slots):
            if mm:
                label = aperture if count > 1 else "Single lens"
                summary.append(f"{label} · {mm} mm · {name} · exit pupil {pupil} mm")
        self.confirming = True
        try:
            confirmed = messagebox.askokcancel(
                "Write Firmware",
                f"Install firmware {self.firmware.version_text} on hardware version {hardware}?\n\n"
                + "\n".join(summary)
                + "\n\nKeep the USB cable connected until the transfer completes.",
                parent=self.root,
            )
        finally:
            self.confirming = False
        if not confirmed:
            return
        # If a probe is still running, the write is queued behind it and the
        # worker rechecks the selected connection before sending START.
        self.probe = None
        self.write_button.configure(state="disabled")
        self.status.set("Preparing the adapter…")
        self.bar.configure(value=0)
        self.update = self.executor.submit(
            self.connection.flash, self.path, image, self.firmware.version,
            lambda sent, total: self.progress.put((sent, total)),
        )
        self.root.after(50, self.show_progress)

    def show_progress(self):
        while not self.progress.empty():
            sent, total = self.progress.get_nowait()
            self.bar.configure(maximum=total, value=sent)
            self.status.set(f"Writing firmware · {sent * 100 // total}%")
        if not self.update.done():
            self.root.after(50, self.show_progress)
            return
        try:
            self.update.result()
        except (OSError, ValueError) as error:
            self.status.set("Transfer failed. Reconnect the adapter before retrying.")
            messagebox.showerror("Transfer Failed", f"{error}\n\nReconnect the adapter before retrying.",
                                 parent=self.root)
        else:
            self.bar.configure(value=self.bar.cget("maximum"))
            self.status.set("Transfer complete. Unplug the adapter.")
            messagebox.showinfo("Transfer Complete", "Unplug the adapter from USB.",
                                parent=self.root)
        self.update = None
        self.path = None

    def close(self):
        if self.update is not None or self.confirming:
            return
        self.root.after_cancel(self.next_poll)
        self.executor.shutdown(wait=False, cancel_futures=True)
        self.root.destroy()


def main():
    root = tk.Tk()
    resources = resource_dir()
    try:
        firmware = Firmware(resources)
        notes = (resources / "changelog.txt").read_text(encoding="utf-8")
    except (OSError, ValueError, KeyError) as error:
        root.withdraw()
        messagebox.showerror(TITLE, f"Could not open the bundled firmware.\n\n{error}", parent=root)
        root.destroy()
        return
    Updater(root, firmware, notes)
    root.mainloop()


if __name__ == "__main__":
    main()
