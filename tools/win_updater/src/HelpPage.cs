// HelpPage.cs -- the pages the "How to choose?" links open: one next to the
// adapter version, one next to the focal length.  spec.md section 4.

using System;
using System.Drawing;
using System.IO;
using System.Windows.Forms;

namespace EA9Updater
{
    sealed class HelpPage : Form
    {
        const int ContentWidth = 600;

        // How to tell the two adapter versions apart, and which to choose.
        public static HelpPage AdapterVersion()
        {
            return new HelpPage("Adapter Version", "hw_version.jpg",
                "What is the difference",
                "Version 1 has less room past infinity than version 2. To focus at infinity, " +
                "version 1 has to retract all the way to its end stop. On version 2 the firmware " +
                "does not need to, which is easier on the mechanism and gives it a longer life.",
                "How to choose",
                "Only if your adapter is version 1 hardware (see the picture) " +
                "and it cannot focus at infinity, try version 1.");
        }

        // Single Lens or Multiple Lenses.
        public static HelpPage FocalLength()
        {
            return new HelpPage("Focal Length", null,
                "Single Lens",
                "Choose this if you only use one lens. The aperture is then fixed while you " +
                "shoot, so it cannot be changed by mistake.",
                "Multiple Lenses",
                "Enter each lens's details in the row of the aperture you want to use for it. " +
                "To change lenses, set the camera to that aperture and press the shutter: the " +
                "adapter will restart and switch to that lens's details. Remember to set the " +
                "aperture back to its widest value before you shoot again.");
        }

        // image: an embedded resource shown first, or null.  sections:
        // heading, text, heading, text, ...
        HelpPage(string title, string image, params string[] sections)
        {
            Text = title;
            Font = SystemFonts.MessageBoxFont;
            AutoScaleMode = AutoScaleMode.Font;
            FormBorderStyle = FormBorderStyle.FixedDialog;
            MaximizeBox = MinimizeBox = false;
            ShowInTaskbar = false;
            StartPosition = FormStartPosition.CenterParent;
            AutoSize = true;
            AutoSizeMode = AutoSizeMode.GrowAndShrink;

            var flow = new FlowLayoutPanel {
                FlowDirection = FlowDirection.TopDown,
                WrapContents = false,
                AutoSize = true,
                Padding = new Padding(12),
            };

            if (image != null) {
                try {
                    var img = Image.FromStream(new MemoryStream(Firmware.Resource(image)));
                    FormClosed += (s, e) => img.Dispose();
                    // Full width, but no taller than fits a 768 px screen
                    // together with the text; Zoom keeps the aspect ratio.
                    int h = Math.Min(img.Height * ContentWidth / img.Width,
                                     Math.Max(200, Screen.FromControl(this).WorkingArea.Height - 330));
                    flow.Controls.Add(new PictureBox {
                        Image = img,
                        SizeMode = PictureBoxSizeMode.Zoom,
                        Size = new Size(ContentWidth, h),
                        Margin = new Padding(0, 0, 0, 8),
                    });
                } catch (Exception ex) {
                    flow.Controls.Add(Para("(picture unavailable: " + ex.Message + ")"));
                }
            }

            for (int i = 0; i + 1 < sections.Length; i += 2) {
                flow.Controls.Add(Heading(sections[i]));
                flow.Controls.Add(Para(sections[i + 1]));
            }

            var ok = new Button { Text = "OK", DialogResult = DialogResult.OK, AutoSize = true,
                                  Padding = new Padding(12, 2, 12, 2),
                                  Anchor = AnchorStyles.Right, Margin = new Padding(0, 12, 0, 0) };
            var okRow = new FlowLayoutPanel { FlowDirection = FlowDirection.RightToLeft,
                                              MinimumSize = new Size(ContentWidth, 0), AutoSize = true,
                                              Margin = new Padding(0) };
            okRow.Controls.Add(ok);
            flow.Controls.Add(okRow);

            Controls.Add(flow);
            AcceptButton = CancelButton = ok;
        }

        Label Heading(string text)
        {
            return new Label { Text = text, AutoSize = true, Font = new Font(Font.FontFamily, Font.Size + 1, FontStyle.Bold),
                               Margin = new Padding(0, 8, 0, 4) };
        }

        static Label Para(string text)
        {
            return new Label { Text = text, AutoSize = true,
                               MaximumSize = new Size(ContentWidth, 0),
                               Margin = new Padding(0, 0, 0, 4) };
        }
    }
}
