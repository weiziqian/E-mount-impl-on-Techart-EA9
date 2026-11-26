// MainForm.cs -- the one window: release notes, hardware version, the lens
// configuration (up to eight focal lengths, selected by aperture), and
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
        readonly ComboBox language;
        readonly Label notesLabel;
        readonly TextBox notes;
        readonly Label hwLabel;
        readonly LinkLabel hwHelp;
        readonly LinkLabel focalHelp;
        readonly Label[] slotHeads = new Label[4];
        readonly Label deviceLabel;
        readonly ComboBox hwVersion;
        readonly LinkLabel slotXpdHelp;
        readonly Label homeResetText;
        readonly TableLayoutPanel slotGrid;
        readonly TextBox[] slotFocal = new TextBox[LensConfig.SlotCount];
        readonly TextBox[] slotName = new TextBox[LensConfig.SlotCount];
        readonly ComboBox[] slotXpd = new ComboBox[LensConfig.SlotCount];
        readonly Button writeButton;
        readonly ProgressBar bar;
        readonly Label status;
        readonly Label device;
        readonly Timer poll = new Timer { Interval = 2000 };
        bool busy;
        bool polling;
        bool deviceOk;      // the adapter on USB answered CHECK
        // The adapter and status lines, kept as functions so a language
        // switch can say the same thing again in the other language.
        Func<string> deviceText = () => "";
        Func<string> statusText = () => "";
        // The adapter's device path at the last poll, null if absent, or
        // NotPolled to force the next poll to act even if nothing changed.
        const string NotPolled = "?";
        string seenPath = NotPolled;

        public MainForm()
        {
            Font = SystemFonts.MessageBoxFont;
            AutoScaleMode = AutoScaleMode.Font;
            StartPosition = FormStartPosition.CenterScreen;
            // 1008 x 880, but never taller or wider than the screen's work
            // area (a 1366 x 768 laptop has about 730 px of it); the release
            // notes take what height is left.
            var work = Screen.PrimaryScreen.WorkingArea.Size;
            ClientSize = new Size(Math.Min(1008, work.Width - 40), Math.Min(880, work.Height - 60));
            MinimumSize = new Size(680, 640);

            var t = new TableLayoutPanel {
                Dock = DockStyle.Fill,
                ColumnCount = 2,
                RowCount = 8,
                Padding = new Padding(12),
            };
            t.ColumnStyles.Add(new ColumnStyle(SizeType.AutoSize));
            t.ColumnStyles.Add(new ColumnStyle(SizeType.Percent, 100));
            t.RowStyles.Add(new RowStyle(SizeType.AutoSize));         // 0 notes label
            t.RowStyles.Add(new RowStyle(SizeType.Percent, 100));     // 1 notes
            t.RowStyles.Add(new RowStyle(SizeType.AutoSize));         // 2 hardware version
            t.RowStyles.Add(new RowStyle(SizeType.AutoSize));         // 3 slot table
            t.RowStyles.Add(new RowStyle(SizeType.AutoSize));         // 4 help links
            t.RowStyles.Add(new RowStyle(SizeType.AutoSize));         // 5 adapter
            t.RowStyles.Add(new RowStyle(SizeType.AutoSize));         // 6 button + bar
            t.RowStyles.Add(new RowStyle(SizeType.AutoSize));         // 7 status

            notesLabel = new Label { AutoSize = true, Anchor = AnchorStyles.Left | AnchorStyles.Bottom,
                                     Margin = new Padding(3, 0, 3, 3) };
            t.Controls.Add(notesLabel, 0, 0);
            // The language box: not translated, each name in its own language.
            language = new ComboBox { DropDownStyle = ComboBoxStyle.DropDownList,
                                      Anchor = AnchorStyles.Right, Margin = new Padding(3, 0, 3, 3) };
            language.Items.Add("English");
            language.Items.Add("中文");
            language.SelectedIndex = S.Zh ? 1 : 0;
            // Wide enough for the longer name plus the drop-down button.
            language.Width = new[] { "English", "中文" }
                .Max(n => TextRenderer.MeasureText(n, Font).Width)
                + SystemInformation.VerticalScrollBarWidth + 24;
            language.SelectedIndexChanged += (s, e) => {
                S.Zh = language.SelectedIndex == 1;
                ApplyText();
            };
            t.Controls.Add(language, 1, 0);

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
            hwHelp = HelpLink(HelpPage.AdapterVersion);
            var hwRow = new FlowLayoutPanel { AutoSize = true, WrapContents = false,
                                              Anchor = AnchorStyles.Left, Margin = new Padding(0) };
            hwRow.Controls.Add(hwVersion);
            hwRow.Controls.Add(hwHelp);
            hwLabel = FieldLabel();
            t.Controls.Add(hwLabel, 0, 2);
            t.Controls.Add(hwRow, 1, 2);

            homeResetText = new Label { AutoSize = true, Anchor = AnchorStyles.Left,
                                        Margin = new Padding(6, 6, 6, 6) };
            slotGrid = SlotGrid();
            t.Controls.Add(slotGrid, 0, 3);
            t.SetColumnSpan(slotGrid, 2);

            // Under the table, right-aligned: how to switch lenses, and how
            // to choose the exit pupil.
            focalHelp = HelpLink(HelpPage.FocalLength);
            slotXpdHelp = HelpLink(HelpPage.ExitPupil);
            var links = new FlowLayoutPanel { AutoSize = true, WrapContents = false,
                                              Anchor = AnchorStyles.Right,
                                              Margin = new Padding(3, 0, 3, 6) };
            links.Controls.Add(focalHelp);
            links.Controls.Add(slotXpdHelp);
            t.Controls.Add(links, 0, 4);
            t.SetColumnSpan(links, 2);

            device = new Label { AutoSize = true, Anchor = AnchorStyles.Left,
                                 Margin = new Padding(3, 8, 3, 8) };
            deviceLabel = FieldLabel();
            t.Controls.Add(deviceLabel, 0, 5);
            t.Controls.Add(device, 1, 5);

            writeButton = new Button { AutoSize = true,
                                       Padding = new Padding(8, 2, 8, 2) };
            writeButton.Click += OnWrite;
            bar = new ProgressBar { Dock = DockStyle.Fill, Minimum = 0, Maximum = 100 };
            t.Controls.Add(writeButton, 0, 6);
            t.Controls.Add(bar, 1, 6);

            status = new Label { AutoSize = true, Margin = new Padding(3, 6, 3, 0) };
            t.Controls.Add(status, 0, 7);
            t.SetColumnSpan(status, 2);

            Controls.Add(t);
            AcceptButton = writeButton;

            try {
                fw = Firmware.LoadEmbedded();
                // Writing stays off until a poll has found the adapter.
                writeButton.Enabled = false;
                SetDevice(() => S.Checking);
                Say(() => S.LookingForAdapter);
            } catch (Exception ex) {
                writeButton.Enabled = false;
                SetInputsEnabled(false);
                Say(() => string.Format(S.Damaged, ex.Message), true);
            }
            ApplyText();
        }

        // Every text in the window, in the current language (S.Zh).  Run at
        // start and on a language switch; what the user typed is kept.
        void ApplyText()
        {
            SuspendLayout();
            Text = string.Format(S.Title, Firmware.DeclaredVersion[0], Firmware.DeclaredVersion[1],
                                 Firmware.DeclaredVersion[2]);
            notesLabel.Text = S.ReleaseNotes;
            try {
                notes.Text = Firmware.LoadChangelog().Replace("\r\n", "\n").Replace("\n", "\r\n");
            } catch (Exception ex) {
                notes.Text = string.Format(S.NotesUnavailable, ex.Message);
            }
            hwLabel.Text = S.AdapterVersion;
            hwHelp.Text = S.HowToChoose;
            focalHelp.Text = S.HowToSwitch;
            slotXpdHelp.Text = S.HowToChooseXpd;
            homeResetText.Text = S.HomeReset;
            slotHeads[0].Text = S.ColAperture;
            slotHeads[1].Text = S.ColFocal;
            slotHeads[2].Text = S.ColName;
            slotHeads[3].Text = S.ColXpd;
            deviceLabel.Text = S.Adapter;
            writeButton.Text = S.WriteFirmware;
            device.Text = deviceText();
            status.Text = statusText();
            ResumeLayout(true);
        }

        // The lens table: the aperture that selects the slot, focal length,
        // name, exit pupil.  The name and the exit pupil are shown only once
        // the slot has a focal length above 0.  A last row, text only, says
        // what f6.3 does (LensConfig.HomeResetAperture).
        TableLayoutPanel SlotGrid()
        {
            var g = new TableLayoutPanel {
                ColumnCount = 4,
                RowCount = LensConfig.SlotCount + 2,
                AutoSize = true,
                AutoSizeMode = AutoSizeMode.GrowAndShrink,
                Dock = DockStyle.Top,
                CellBorderStyle = TableLayoutPanelCellBorderStyle.Single,
                Margin = new Padding(3, 6, 3, 6),
            };
            g.ColumnStyles.Add(new ColumnStyle(SizeType.AutoSize));
            g.ColumnStyles.Add(new ColumnStyle(SizeType.AutoSize));
            g.ColumnStyles.Add(new ColumnStyle(SizeType.Percent, 100));
            g.ColumnStyles.Add(new ColumnStyle(SizeType.AutoSize));
            for (int r = 0; r <= LensConfig.SlotCount + 1; r++) {
                g.RowStyles.Add(new RowStyle(SizeType.AutoSize));
            }
            for (int c = 0; c < slotHeads.Length; c++) {
                slotHeads[c] = new Label { AutoSize = true,
                                           Font = new Font(Font, FontStyle.Bold),
                                           Anchor = AnchorStyles.Left,
                                           Margin = new Padding(6, 4, 6, 4) };
                g.Controls.Add(slotHeads[c], c, 0);
            }
            for (int i = 0; i < LensConfig.SlotCount; i++) {
                g.Controls.Add(CellLabel(LensConfig.SlotAperture[i]), 0, i + 1);

                var f = FocalBox();
                f.Margin = new Padding(6, 3, 6, 3);
                var n = new TextBox { Anchor = AnchorStyles.Left | AnchorStyles.Right,
                                      MaxLength = LensConfig.NameMax, Visible = false,
                                      Margin = new Padding(6, 3, 6, 3) };
                var x = XpdBox();
                x.Margin = new Padding(6, 3, 6, 3);
                x.Visible = false;
                f.TextChanged += (s, e) => {
                    AutoXpd(f, x);
                    UpdateSlotNames();
                };
                slotFocal[i] = f;
                slotName[i] = n;
                slotXpd[i] = x;
                g.Controls.Add(f, 1, i + 1);
                g.Controls.Add(n, 2, i + 1);
                g.Controls.Add(x, 3, i + 1);
            }
            int last = LensConfig.SlotCount + 1;
            g.Controls.Add(CellLabel(LensConfig.HomeResetAperture), 0, last);
            g.Controls.Add(homeResetText, 1, last);
            g.SetColumnSpan(homeResetText, 3);
            return g;
        }

        // A "How to choose?" link that opens the page `page` makes.
        LinkLabel HelpLink(Func<HelpPage> page)
        {
            var l = new LinkLabel { AutoSize = true,
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

        // The exit pupil drop-down: OpticalFile.XpdChoices, in mm.
        ComboBox XpdBox()
        {
            var b = new ComboBox { DropDownStyle = ComboBoxStyle.DropDownList,
                                   Anchor = AnchorStyles.Left };
            foreach (int v in OpticalFile.XpdChoices) {
                b.Items.Add(v.ToString());
            }
            b.Width = TextRenderer.MeasureText("000", Font).Width
                      + SystemInformation.VerticalScrollBarWidth + 24;
            return b;
        }

        // Selects the exit pupil OpticalFile.AutoXpd gives for `focalMm`.
        static void SelectXpd(ComboBox b, int focalMm)
        {
            b.SelectedItem = OpticalFile.AutoXpd(focalMm).ToString();
        }

        // On every change of a focal length, preselect its exit pupil; the
        // user may then pick another, until the focal length changes again.
        static void AutoXpd(TextBox focal, ComboBox xpd)
        {
            int mm = FocalValue(focal);
            if (mm > 0) {
                SelectXpd(xpd, mm);
            }
        }

        static int XpdValue(ComboBox b)
        {
            int v;
            return b.SelectedItem != null && int.TryParse((string)b.SelectedItem, out v) ? v : 0;
        }

        static int FocalValue(TextBox b)
        {
            int v;
            return int.TryParse(b.Text, out v) ? v : 0;
        }

        void UpdateSlotNames()
        {
            bool on = !busy && fw != null;
            for (int i = 0; i < LensConfig.SlotCount; i++) {
                // Hidden, not just greyed, while the slot has no focal length.
                bool set = FocalValue(slotFocal[i]) > 0;
                slotName[i].Visible = slotXpd[i].Visible = set;
                slotName[i].Enabled = slotXpd[i].Enabled = on;
            }
        }

        void SetInputsEnabled(bool on)
        {
            hwVersion.Enabled = on;
            foreach (var f in slotFocal) {
                f.Enabled = on;
            }
            UpdateSlotNames();
        }

        // The configuration as entered, checked and normalised; null (after
        // telling the user why) if it cannot be written.
        LensConfig ReadConfig()
        {
            var entries = new List<LensConfig.Entry>();
            for (int i = 0; i < LensConfig.SlotCount; i++) {
                entries.Add(new LensConfig.Entry(slotFocal[i].Text, slotName[i].Text,
                                                 XpdValue(slotXpd[i])));
            }
            string error;
            var cfg = LensConfig.FromForm(int.Parse((string)hwVersion.SelectedItem),
                                          entries, out error);
            if (cfg == null) {
                MessageBox.Show(this, error, Text, MessageBoxButtons.OK, MessageBoxIcon.Warning);
            }
            return cfg;
        }

        // The status line; problems the user must act on are shown in red.
        void Say(Func<string> text, bool problem = false)
        {
            statusText = text;
            status.Text = text();
            status.ForeColor = problem ? Color.FromArgb(0xC0, 0x00, 0x00) : SystemColors.ControlText;
        }

        void SetDevice(Func<string> text)
        {
            deviceText = text;
            device.Text = text();
        }

        static Label FieldLabel()
        {
            return new Label { AutoSize = true, Anchor = AnchorStyles.Left,
                               Margin = new Padding(3, 8, 12, 8) };
        }

        async void OnWrite(object sender, EventArgs e)
        {
            LensConfig cfg = ReadConfig();
            if (cfg == null) {
                return;
            }
            byte[] image = fw.Patch(cfg);

            if (MessageBox.Show(this, string.Format(S.Confirm, cfg.Describe()), Text, MessageBoxButtons.OKCancel, MessageBoxIcon.Information) != DialogResult.OK) {
                return;
            }

            SetBusy(true);
            bar.Value = 0;
            var progress = new Progress<int>(p => bar.Value = p);
            var say = new Progress<Func<string>>(s => Say(s));
            try {
                await Task.Run(() => Run(image, progress, say));
                bar.Value = 100;
                // No second write until the adapter is unplugged and comes
                // back: the poll sees the same path and does not re-enable.
                deviceOk = false;
                SetDevice(() => S.FirmwareWritten);
                Say(() => S.Done);
                MessageBox.Show(this, S.Success, Text, MessageBoxButtons.OK, MessageBoxIcon.Information);
            } catch (Exception ex) {
                // Make the next poll check the adapter afresh.
                seenPath = NotPolled;
                deviceOk = false;
                Say(() => string.Format(S.Failed, ex.Message), true);
                MessageBox.Show(this, string.Format(S.WriteFailed, ex.Message), Text, MessageBoxButtons.OK, MessageBoxIcon.Error);
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
                    SetDevice(() => S.NotConnected);
                    Say(() => S.PleaseConnect, true);
                    return;
                }

                SetDevice(() => S.Checking);
                Say(() => S.AdapterFound);
                byte[] v;
                try {
                    v = await Task.Run(() => ReadBootloaderVersion(path));
                } catch (Exception ex) {
                    SetDevice(() => S.NotResponding);
                    Say(() => string.Format(S.NoAnswer, ex.Message), true);
                    return;
                }
                deviceOk = true;
                SetDevice(() => string.Format(S.Connected, v[0], v[1], v[2]));
                Say(() => string.Format(S.ReadyToWrite, Firmware.DeclaredVersion[0],
                                        Firmware.DeclaredVersion[1], Firmware.DeclaredVersion[2]));
                writeButton.Enabled = true;
            } catch (Exception ex) {
                Say(() => string.Format(S.LookingFailed, ex.Message), true);
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
        static void Run(byte[] image, IProgress<int> progress, IProgress<Func<string>> say)
        {
            say.Report(() => S.LookingForAdapter);
            var paths = HidDevice.Find(Flasher.Vid, Flasher.Pid);
            if (paths.Count == 0) {
                throw new InvalidOperationException(S.NotFound);
            }

            using (var dev = HidDevice.Open(paths[0])) {
                var flasher = new Flasher(dev);
                byte[] v = flasher.Check();
                say.Report(() => string.Format(S.Preparing, v[0], v[1], v[2]));
                flasher.Start(Firmware.DeclaredVersion);
                say.Report(() => S.Writing);
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
                MessageBox.Show(this, S.PleaseWait, Text, MessageBoxButtons.OK, MessageBoxIcon.Warning);
                e.Cancel = true;
            }
            base.OnFormClosing(e);
        }
    }
}
