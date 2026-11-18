// MainForm.cs -- the one window: release notes, hardware version, the lens
// configuration (one focal length, or up to eight selected by aperture), and
// a button that writes the firmware.  Laid out in code (no designer), so it
// builds on Linux.  spec.md section 4.

using System;
using System.Collections.Generic;
using System.Drawing;
using System.Linq;
using System.Threading.Tasks;
using System.Windows.Forms;

namespace EA9Updater
{
    sealed class MainForm : Form
    {
        readonly Firmware fw;
        readonly TextBox notes;
        readonly ComboBox hwVersion;
        readonly RadioButton multiRadio;
        readonly RadioButton singleRadio;
        readonly Label singleFocalLabel;
        readonly TextBox singleFocal;
        readonly Label singleNameLabel;
        readonly TextBox singleName;
        readonly TableLayoutPanel slotGrid;
        readonly TextBox[] slotFocal = new TextBox[LensConfig.SlotCount];
        readonly TextBox[] slotName = new TextBox[LensConfig.SlotCount];
        readonly Button writeButton;
        readonly ProgressBar bar;
        readonly Label status;
        readonly Label device;
        readonly Timer poll = new Timer { Interval = 2000 };
        bool busy;
        bool polling;
        bool deviceOk;      // the adapter on USB answered CHECK
        // The adapter's device path at the last poll, null if absent, or
        // NotPolled to force the next poll to act even if nothing changed.
        const string NotPolled = "?";
        string seenPath = NotPolled;

        public MainForm()
        {
            Text = string.Format("Community EA9 Firmware Updater: v{0}.{1}.{2}",
                                 Firmware.DeclaredVersion[0], Firmware.DeclaredVersion[1],
                                 Firmware.DeclaredVersion[2]);
            Font = SystemFonts.MessageBoxFont;
            AutoScaleMode = AutoScaleMode.Font;
            StartPosition = FormStartPosition.CenterScreen;
            // 840 x 880, but never taller or wider than the screen's work
            // area (a 1366 x 768 laptop has about 730 px of it); the release
            // notes take what height is left.
            var work = Screen.PrimaryScreen.WorkingArea.Size;
            ClientSize = new Size(Math.Min(840, work.Width - 40), Math.Min(880, work.Height - 60));
            MinimumSize = new Size(680, 640);

            var t = new TableLayoutPanel {
                Dock = DockStyle.Fill,
                ColumnCount = 2,
                RowCount = 10,
                Padding = new Padding(12),
            };
            t.ColumnStyles.Add(new ColumnStyle(SizeType.AutoSize));
            t.ColumnStyles.Add(new ColumnStyle(SizeType.Percent, 100));
            t.RowStyles.Add(new RowStyle(SizeType.AutoSize));         // 0 notes label
            t.RowStyles.Add(new RowStyle(SizeType.Percent, 100));     // 1 notes
            t.RowStyles.Add(new RowStyle(SizeType.AutoSize));         // 2 hardware version
            t.RowStyles.Add(new RowStyle(SizeType.AutoSize));         // 3 single / multiple
            t.RowStyles.Add(new RowStyle(SizeType.AutoSize));         // 4 single: focal
            t.RowStyles.Add(new RowStyle(SizeType.AutoSize));         // 5 single: name
            t.RowStyles.Add(new RowStyle(SizeType.AutoSize));         // 6 multiple: slot table
            t.RowStyles.Add(new RowStyle(SizeType.AutoSize));         // 7 adapter
            t.RowStyles.Add(new RowStyle(SizeType.AutoSize));         // 8 button + bar
            t.RowStyles.Add(new RowStyle(SizeType.AutoSize));         // 9 status

            var notesLabel = new Label { Text = "Release notes", AutoSize = true,
                                         Margin = new Padding(3, 0, 3, 3) };
            t.Controls.Add(notesLabel, 0, 0);
            t.SetColumnSpan(notesLabel, 2);

            notes = new TextBox {
                Multiline = true,
                ReadOnly = true,
                ScrollBars = ScrollBars.Vertical,
                WordWrap = true,
                Dock = DockStyle.Fill,
                BackColor = SystemColors.Window,
                TabStop = false,
            };
            t.Controls.Add(notes, 0, 1);
            t.SetColumnSpan(notes, 2);

            hwVersion = new ComboBox { DropDownStyle = ComboBoxStyle.DropDownList, Width = 90,
                                       Anchor = AnchorStyles.Left };
            foreach (int v in LensConfig.HwVersions) {
                hwVersion.Items.Add(v.ToString());
            }
            hwVersion.SelectedIndex = 0;
            var hwHelp = HelpLink(HelpPage.AdapterVersion);
            var hwRow = new FlowLayoutPanel { AutoSize = true, WrapContents = false,
                                              Anchor = AnchorStyles.Left, Margin = new Padding(0) };
            hwRow.Controls.Add(hwVersion);
            hwRow.Controls.Add(hwHelp);
            t.Controls.Add(FieldLabel("Adapter version"), 0, 2);
            t.Controls.Add(hwRow, 1, 2);

            multiRadio = new RadioButton { Text = "Multiple Lenses", AutoSize = true, Checked = true };
            singleRadio = new RadioButton { Text = "Single Lens", AutoSize = true,
                                            Margin = new Padding(16, 3, 3, 3) };
            multiRadio.CheckedChanged += (s, e) => ShowLensMode();
            var modes = new FlowLayoutPanel { AutoSize = true, WrapContents = false,
                                              Anchor = AnchorStyles.Left, Margin = new Padding(0) };
            modes.Controls.Add(multiRadio);
            modes.Controls.Add(singleRadio);
            modes.Controls.Add(HelpLink(HelpPage.FocalLength));
            t.Controls.Add(FieldLabel("Focal length"), 0, 3);
            t.Controls.Add(modes, 1, 3);

            singleFocal = FocalBox();
            singleFocal.Text = LensConfig.DefaultSingleFocalMm.ToString();
            singleFocalLabel = FieldLabel("Focal length (mm)");
            t.Controls.Add(singleFocalLabel, 0, 4);
            t.Controls.Add(singleFocal, 1, 4);

            singleName = new TextBox { Anchor = AnchorStyles.Left | AnchorStyles.Right,
                                       MaxLength = LensConfig.NameMax,
                                       Text = LensConfig.DefaultName };
            singleNameLabel = FieldLabel("Lens name");
            t.Controls.Add(singleNameLabel, 0, 5);
            t.Controls.Add(singleName, 1, 5);

            slotGrid = SlotGrid();
            t.Controls.Add(slotGrid, 0, 6);
            t.SetColumnSpan(slotGrid, 2);

            device = new Label { AutoSize = true, Anchor = AnchorStyles.Left,
                                 Margin = new Padding(3, 8, 3, 8) };
            t.Controls.Add(FieldLabel("Adapter"), 0, 7);
            t.Controls.Add(device, 1, 7);

            writeButton = new Button { Text = "Write firmware", AutoSize = true,
                                       Padding = new Padding(8, 2, 8, 2) };
            writeButton.Click += OnWrite;
            bar = new ProgressBar { Dock = DockStyle.Fill, Minimum = 0, Maximum = 100 };
            t.Controls.Add(writeButton, 0, 8);
            t.Controls.Add(bar, 1, 8);

            status = new Label { AutoSize = true, Margin = new Padding(3, 6, 3, 0) };
            t.Controls.Add(status, 0, 9);
            t.SetColumnSpan(status, 2);

            Controls.Add(t);
            AcceptButton = writeButton;
            ShowLensMode();

            try {
                notes.Text = Firmware.LoadChangelog().Replace("\r\n", "\n").Replace("\n", "\r\n");
            } catch (Exception ex) {
                notes.Text = "(release notes unavailable: " + ex.Message + ")";
            }
            try {
                fw = Firmware.LoadEmbedded();
                // Writing stays off until a poll has found the adapter.
                writeButton.Enabled = false;
                device.Text = "Checking...";
                Say("Looking for the adapter...");
            } catch (Exception ex) {
                writeButton.Enabled = false;
                SetInputsEnabled(false);
                Say("This updater is damaged and cannot write: " + ex.Message, true);
            }
        }

        // The multiple-focal-length table: slot, the aperture that selects
        // it, focal length, name.  A name is editable only once its slot has
        // a focal length above 0.
        TableLayoutPanel SlotGrid()
        {
            var g = new TableLayoutPanel {
                ColumnCount = 4,
                RowCount = LensConfig.SlotCount + 1,
                AutoSize = true,
                AutoSizeMode = AutoSizeMode.GrowAndShrink,
                Dock = DockStyle.Top,
                CellBorderStyle = TableLayoutPanelCellBorderStyle.Single,
                Margin = new Padding(3, 6, 3, 6),
            };
            g.ColumnStyles.Add(new ColumnStyle(SizeType.AutoSize));
            g.ColumnStyles.Add(new ColumnStyle(SizeType.AutoSize));
            g.ColumnStyles.Add(new ColumnStyle(SizeType.AutoSize));
            g.ColumnStyles.Add(new ColumnStyle(SizeType.Percent, 100));
            for (int r = 0; r <= LensConfig.SlotCount; r++) {
                g.RowStyles.Add(new RowStyle(SizeType.AutoSize));
            }
            string[] heads = { "Slot", "Aperture", "Focal Length (mm)", "Lens Name" };
            for (int c = 0; c < heads.Length; c++) {
                g.Controls.Add(new Label { Text = heads[c], AutoSize = true,
                                           Font = new Font(Font, FontStyle.Bold),
                                           Anchor = AnchorStyles.Left,
                                           Margin = new Padding(6, 4, 6, 4) }, c, 0);
            }
            for (int i = 0; i < LensConfig.SlotCount; i++) {
                g.Controls.Add(CellLabel((i + 1).ToString()), 0, i + 1);
                g.Controls.Add(CellLabel(LensConfig.SlotAperture[i]), 1, i + 1);

                var f = FocalBox();
                f.Margin = new Padding(6, 3, 6, 3);
                var n = new TextBox { Anchor = AnchorStyles.Left | AnchorStyles.Right,
                                      MaxLength = LensConfig.NameMax, Enabled = false,
                                      Margin = new Padding(6, 3, 6, 3) };
                f.TextChanged += (s, e) => UpdateSlotNames();
                slotFocal[i] = f;
                slotName[i] = n;
                g.Controls.Add(f, 2, i + 1);
                g.Controls.Add(n, 3, i + 1);
            }
            return g;
        }

        // A "How to choose?" link that opens the page `page` makes.
        LinkLabel HelpLink(Func<HelpPage> page)
        {
            var l = new LinkLabel { Text = "How to choose?", AutoSize = true,
                                    Anchor = AnchorStyles.Left, Margin = new Padding(12, 3, 3, 3) };
            l.LinkClicked += (s, e) => {
                using (var d = page()) {
                    d.ShowDialog(this);
                }
            };
            return l;
        }

        static Label CellLabel(string text)
        {
            return new Label { Text = text, AutoSize = true, Anchor = AnchorStyles.Left,
                               Margin = new Padding(6, 6, 6, 6) };
        }

        // A text box that takes digits only (typed or pasted).
        static TextBox FocalBox()
        {
            var b = new TextBox { Width = 90, MaxLength = 4, Anchor = AnchorStyles.Left };
            b.KeyPress += (s, e) => {
                if (!char.IsDigit(e.KeyChar) && !char.IsControl(e.KeyChar)) {
                    e.Handled = true;
                }
            };
            b.TextChanged += (s, e) => {
                string d = new string(b.Text.Where(c => c >= '0' && c <= '9').ToArray());
                if (d != b.Text) {
                    int at = Math.Max(0, b.SelectionStart - (b.Text.Length - d.Length));
                    b.Text = d;
                    b.SelectionStart = Math.Min(at, d.Length);
                }
            };
            return b;
        }

        static int FocalValue(TextBox b)
        {
            int v;
            return int.TryParse(b.Text, out v) ? v : 0;
        }

        void ShowLensMode()
        {
            bool single = singleRadio.Checked;
            SuspendLayout();
            singleFocalLabel.Visible = singleFocal.Visible = single;
            singleNameLabel.Visible = singleName.Visible = single;
            slotGrid.Visible = !single;
            ResumeLayout(true);
        }

        void UpdateSlotNames()
        {
            bool on = !busy && fw != null;
            for (int i = 0; i < LensConfig.SlotCount; i++) {
                slotName[i].Enabled = on && FocalValue(slotFocal[i]) > 0;
            }
        }

        void SetInputsEnabled(bool on)
        {
            hwVersion.Enabled = multiRadio.Enabled = singleRadio.Enabled = on;
            singleFocal.Enabled = singleName.Enabled = on;
            foreach (var f in slotFocal) {
                f.Enabled = on;
            }
            UpdateSlotNames();
        }

        // The configuration as entered, checked and normalised; null (after
        // telling the user why) if it cannot be written.
        LensConfig ReadConfig()
        {
            bool single = singleRadio.Checked;
            var entries = new List<LensConfig.Entry>();
            if (single) {
                entries.Add(new LensConfig.Entry(singleFocal.Text, singleName.Text));
            } else {
                for (int i = 0; i < LensConfig.SlotCount; i++) {
                    entries.Add(new LensConfig.Entry(slotFocal[i].Text, slotName[i].Text));
                }
            }
            string error;
            var cfg = LensConfig.FromForm(int.Parse((string)hwVersion.SelectedItem), single,
                                          entries, out error);
            if (cfg == null) {
                MessageBox.Show(this, error, Text, MessageBoxButtons.OK, MessageBoxIcon.Warning);
            }
            return cfg;
        }

        // The status line; problems the user must act on are shown in red.
        void Say(string text, bool problem = false)
        {
            status.Text = text;
            status.ForeColor = problem ? Color.FromArgb(0xC0, 0x00, 0x00) : SystemColors.ControlText;
        }

        static Label FieldLabel(string text)
        {
            return new Label { Text = text, AutoSize = true, Anchor = AnchorStyles.Left,
                               Margin = new Padding(3, 8, 12, 8) };
        }

        async void OnWrite(object sender, EventArgs e)
        {
            LensConfig cfg = ReadConfig();
            if (cfg == null) {
                return;
            }
            byte[] image = fw.Patch(cfg);

            if (MessageBox.Show(this,
                    "The firmware will now be written to the adapter with this configuration:\n\n" +
                    cfg.Describe() + "\n" +
                    "Do not unplug the USB cable or close this window until it finishes.",
                    Text, MessageBoxButtons.OKCancel, MessageBoxIcon.Information) != DialogResult.OK) {
                return;
            }

            SetBusy(true);
            bar.Value = 0;
            var progress = new Progress<int>(p => bar.Value = p);
            var say = new Progress<string>(s => Say(s));
            try {
                await Task.Run(() => Run(image, progress, say));
                bar.Value = 100;
                // No second write until the adapter is unplugged and comes
                // back: the poll sees the same path and does not re-enable.
                deviceOk = false;
                device.Text = "Firmware written";
                Say("Done. Unplug the USB cable.");
                MessageBox.Show(this,
                    "The firmware was written successfully.\n\nUnplug the USB cable now.",
                    Text, MessageBoxButtons.OK, MessageBoxIcon.Information);
            } catch (Exception ex) {
                // Make the next poll check the adapter afresh.
                seenPath = NotPolled;
                deviceOk = false;
                Say("Failed: " + ex.Message, true);
                MessageBox.Show(this, "Writing the firmware failed.\n\n" + ex.Message,
                                Text, MessageBoxButtons.OK, MessageBoxIcon.Error);
            } finally {
                SetBusy(false);
            }
        }

        // Every 2 s, look for the adapter.  Polling only enumerates USB; the
        // adapter itself is spoken to (CHECK, read-only) once, when it
        // appears, so the bootloader is not sent a command every tick.
        // Writing is enabled only while an adapter that answered is present.
        protected override void OnShown(EventArgs e)
        {
            base.OnShown(e);
            if (fw == null) {
                return;
            }
            poll.Tick += OnPoll;
            poll.Start();
            OnPoll(null, EventArgs.Empty);
        }

        protected override void OnFormClosed(FormClosedEventArgs e)
        {
            poll.Dispose();
            base.OnFormClosed(e);
        }

        async void OnPoll(object sender, EventArgs e)
        {
            if (polling || busy) {
                return;
            }
            polling = true;
            try {
                var paths = await Task.Run(() => HidDevice.Find(Flasher.Vid, Flasher.Pid));
                string path = paths.Count > 0 ? paths[0] : null;
                if (path == seenPath || busy) {
                    return;                      // nothing changed
                }
                seenPath = path;
                deviceOk = false;
                writeButton.Enabled = false;
                if (path == null) {
                    device.Text = "Not connected";
                    Say("Please connect the adapter to this computer by USB.", true);
                    return;
                }

                device.Text = "Checking...";
                Say("Adapter found, reading its version...");
                byte[] v;
                try {
                    v = await Task.Run(() => ReadBootloaderVersion(path));
                } catch (Exception ex) {
                    device.Text = "Not responding";
                    Say("The adapter did not answer (" + ex.Message + "). " +
                        "Unplug it and plug it in again.", true);
                    return;
                }
                deviceOk = true;
                device.Text = string.Format("Connected, bootloader version {0}.{1}.{2}", v[0], v[1], v[2]);
                Say(string.Format("Ready to write firmware {0}.{1}.{2}.",
                                  Firmware.DeclaredVersion[0], Firmware.DeclaredVersion[1],
                                  Firmware.DeclaredVersion[2]));
                writeButton.Enabled = true;
            } catch (Exception ex) {
                Say("Looking for the adapter failed: " + ex.Message, true);
            } finally {
                polling = false;
            }
        }

        // CHECK: the version the bootloader recorded at the last START.
        // Read-only; the device is closed again.
        static byte[] ReadBootloaderVersion(string path)
        {
            using (var dev = HidDevice.Open(path)) {
                return new Flasher(dev).Check();
            }
        }

        // Runs on a worker thread; reports through the two Progress objects.
        static void Run(byte[] image, IProgress<int> progress, IProgress<string> say)
        {
            say.Report("Looking for the adapter...");
            var paths = HidDevice.Find(Flasher.Vid, Flasher.Pid);
            if (paths.Count == 0) {
                throw new InvalidOperationException(
                    "The adapter was not found.\n\n" +
                    "Connect it to this computer's USB port.");
            }

            using (var dev = HidDevice.Open(paths[0])) {
                var flasher = new Flasher(dev);
                byte[] v = flasher.Check();
                say.Report(string.Format("Found the adapter (bootloader record {0}.{1}.{2}). Preparing...",
                                         v[0], v[1], v[2]));
                flasher.Start(Firmware.DeclaredVersion);
                say.Report("Writing firmware...");
                int last = -1;
                flasher.Write(image, (sent, total) => {
                    int pct = Math.Min(100, sent * 100 / total);
                    if (pct != last) {
                        last = pct;
                        progress.Report(pct);
                    }
                });
            }
        }

        void SetBusy(bool on)
        {
            busy = on;
            SetInputsEnabled(!on);
            writeButton.Enabled = !on && deviceOk;
            UseWaitCursor = on;
        }

        protected override void OnFormClosing(FormClosingEventArgs e)
        {
            if (busy && e.CloseReason == CloseReason.UserClosing) {
                MessageBox.Show(this, "Please wait until the firmware has been written.",
                                Text, MessageBoxButtons.OK, MessageBoxIcon.Warning);
                e.Cancel = true;
            }
            base.OnFormClosing(e);
        }
    }
}
