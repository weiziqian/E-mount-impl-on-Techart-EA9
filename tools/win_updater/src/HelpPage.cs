// HelpPage.cs -- the pages the "How to choose?" links open: one next to the
// adapter version, one next to the focal length, and one for the exit pupil.
// spec.md section 4.

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
            return new HelpPage(S.HelpAdapterTitle, "hw_version.jpg",
                S.HelpDifference, S.HelpDifferenceText,
                S.HelpHowToChoose, S.HelpAdapterChoose);
        }

        // Single Lens or Multiple Lenses.
        public static HelpPage FocalLength()
        {
            return new HelpPage(S.HelpFocalTitle, null,
                S.SingleLens, S.HelpSingleText,
                S.MultipleLenses, S.HelpMultipleText);
        }

        // What the exit pupil is for, and which value to pick.
        public static HelpPage ExitPupil()
        {
            return new HelpPage(S.HelpXpdTitle, null, null, S.HelpXpdText);
        }

        // image: an embedded resource shown first, or null.  sections:
        // heading, text, heading, text, ...; a null heading is left out.
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
                    flow.Controls.Add(Para(string.Format(S.PictureUnavailable, ex.Message)));
                }
            }

            for (int i = 0; i + 1 < sections.Length; i += 2) {
                if (sections[i] != null) {
                    flow.Controls.Add(Heading(sections[i]));
                }
                flow.Controls.Add(Para(sections[i + 1]));
            }

            var ok = new Button { Text = S.Ok, DialogResult = DialogResult.OK, AutoSize = true,
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
