// MainForm.cs -- the one window: release notes, focal length, lens name,
// and a button that writes the firmware.  Laid out in code (no designer), so
// it builds on Linux.  spec.md section 4.

using System;
using System.Drawing;
using System.Threading.Tasks;
using System.Windows.Forms;

namespace EA9Updater
{
    sealed class MainForm : Form
    {
        readonly Firmware fw;
        readonly TextBox notes;
        readonly NumericUpDown focal;
        readonly TextBox lensName;
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
            // 840 x 780, but never taller or wider than the screen's work
            // area (a 1366 x 768 laptop has about 730 px of it).
            var work = Screen.PrimaryScreen.WorkingArea.Size;
            ClientSize = new Size(Math.Min(840, work.Width - 40), Math.Min(780, work.Height - 60));
            MinimumSize = new Size(630, 600);

            var t = new TableLayoutPanel {
                Dock = DockStyle.Fill,
                ColumnCount = 2,
                RowCount = 7,
                Padding = new Padding(12),
            };
            t.ColumnStyles.Add(new ColumnStyle(SizeType.AutoSize));
            t.ColumnStyles.Add(new ColumnStyle(SizeType.Percent, 100));
            t.RowStyles.Add(new RowStyle(SizeType.AutoSize));         // notes label
            t.RowStyles.Add(new RowStyle(SizeType.Percent, 100));     // notes
            t.RowStyles.Add(new RowStyle(SizeType.AutoSize));         // focal
            t.RowStyles.Add(new RowStyle(SizeType.AutoSize));         // name
            t.RowStyles.Add(new RowStyle(SizeType.AutoSize));         // adapter
            t.RowStyles.Add(new RowStyle(SizeType.AutoSize));         // button + bar
            t.RowStyles.Add(new RowStyle(SizeType.AutoSize));         // status

            t.Controls.Add(new Label { Text = "Release notes", AutoSize = true,
                                       Margin = new Padding(3, 0, 3, 3) }, 0, 0);
            t.SetColumnSpan(t.GetControlFromPosition(0, 0), 2);

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

            focal = new NumericUpDown {
                Minimum = Firmware.FocalMinMm,
                Maximum = Firmware.FocalMaxMm,
                DecimalPlaces = 0,
                Width = 90,
                Anchor = AnchorStyles.Left,
            };
            t.Controls.Add(FieldLabel("Focal length (mm)"), 0, 2);
            t.Controls.Add(focal, 1, 2);

            lensName = new TextBox { Anchor = AnchorStyles.Left | AnchorStyles.Right };
            t.Controls.Add(FieldLabel("Lens name"), 0, 3);
            t.Controls.Add(lensName, 1, 3);

            device = new Label { AutoSize = true, Anchor = AnchorStyles.Left,
                                 Margin = new Padding(3, 8, 3, 8) };
            t.Controls.Add(FieldLabel("Adapter"), 0, 4);
            t.Controls.Add(device, 1, 4);

            writeButton = new Button { Text = "Write firmware", AutoSize = true,
                                       Padding = new Padding(8, 2, 8, 2) };
            writeButton.Click += OnWrite;
            bar = new ProgressBar { Dock = DockStyle.Fill, Minimum = 0, Maximum = 100 };
            t.Controls.Add(writeButton, 0, 5);
            t.Controls.Add(bar, 1, 5);

            status = new Label { AutoSize = true, Margin = new Padding(3, 6, 3, 0) };
            t.Controls.Add(status, 0, 6);
            t.SetColumnSpan(status, 2);

            Controls.Add(t);
            AcceptButton = writeButton;

            try {
                notes.Text = Firmware.LoadChangelog().Replace("\r\n", "\n").Replace("\n", "\r\n");
            } catch (Exception ex) {
                notes.Text = "(release notes unavailable: " + ex.Message + ")";
            }
            try {
                fw = Firmware.LoadEmbedded();
                focal.Value = fw.DefaultFocalMm;
                lensName.Text = fw.DefaultName;
                lensName.MaxLength = fw.NameMax;
                // Writing stays off until a poll has found the adapter.
                writeButton.Enabled = false;
                device.Text = "Checking...";
                Say("Looking for the adapter...");
            } catch (Exception ex) {
                writeButton.Enabled = false;
                focal.Enabled = lensName.Enabled = false;
                Say("This updater is damaged and cannot write: " + ex.Message, true);
            }
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
            string why = fw.CheckName(lensName.Text);
            if (why != null) {
                MessageBox.Show(this, why, Text, MessageBoxButtons.OK, MessageBoxIcon.Warning);
                lensName.Focus();
                return;
            }
            byte[] image = fw.Patch((int)focal.Value, lensName.Text);

            if (MessageBox.Show(this,
                    "The firmware will now be written to the adapter.\n\n" +
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
            focal.Enabled = lensName.Enabled = !on;
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
