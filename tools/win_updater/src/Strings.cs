// Strings.cs -- every text the user sees, in English and Chinese.  Saved as
// UTF-8 with a BOM; build.sh refuses to build if it is not valid UTF-8.
//
// Zh picks the language: Program.Main sets it from the Windows display
// language, and the language box in the window changes it.  Errors from the
// USB and firmware layers (Hid.cs, Flasher.cs, Firmware.cs) are diagnostics
// and stay in English.

namespace EA9Updater
{
    static class S
    {
        public static bool Zh;

        static string T(string en, string zh) { return Zh ? zh : en; }

        // ---- main window
        public static string Title { get { return T("Community EA9 Firmware Updater: v{0}.{1}.{2}",
                                                    "Community EA9 固件更新工具：v{0}.{1}.{2}"); } }
        public static string ReleaseNotes { get { return T("Release notes", "更新说明"); } }
        public static string NotesUnavailable { get { return T("(release notes unavailable: {0})",
                                                               "（无法读取更新说明：{0}）"); } }
        public static string AdapterVersion { get { return T("Adapter version", "转接环版本"); } }
        public static string HowToChoose { get { return T("How to choose?", "如何选择？"); } }
        public static string HowToChooseXpd { get { return T("How to choose the exit pupil?",
                                                             "如何选择出瞳距离？"); } }
        public static string HowToSwitch { get { return T("How to switch lenses?", "如何切换镜头？"); } }
        public static string ColAperture { get { return T("Aperture", "光圈"); } }
        public static string ColFocal { get { return T("Focal Length (mm)", "焦距 (mm)"); } }
        public static string ColName { get { return T("Lens Name", "镜头名称"); } }
        public static string ColXpd { get { return T("Exit Pupil (mm)", "出瞳距离 (mm)"); } }
        public static string HomeReset { get { return T("Reset motor homing", "重置马达归位"); } }
        public static string Adapter { get { return T("Adapter", "转接环"); } }
        public static string WriteFirmware { get { return T("Write firmware", "写入固件"); } }

        // ---- adapter and status line
        public static string Checking { get { return T("Checking...", "正在检查…"); } }
        public static string NotConnected { get { return T("Not connected", "未连接"); } }
        public static string NotResponding { get { return T("Not responding", "无响应"); } }
        public static string Connected { get { return T("Connected, bootloader version {0}.{1}.{2}",
                                                        "已连接，引导程序版本 {0}.{1}.{2}"); } }
        public static string FirmwareWritten { get { return T("Firmware written", "固件已写入"); } }
        public static string LookingForAdapter { get { return T("Looking for the adapter...",
                                                                "正在查找转接环…"); } }
        public static string Damaged { get { return T("This updater is damaged and cannot write: {0}",
                                                      "此更新工具已损坏，无法写入：{0}"); } }
        public static string PleaseConnect { get { return T("Please connect the adapter to this computer by USB.",
                                                            "请用 USB 线将转接环连接到这台电脑。"); } }
        public static string AdapterFound { get { return T("Adapter found, reading its version...",
                                                           "已找到转接环，正在读取版本…"); } }
        public static string NoAnswer { get { return T("The adapter did not answer ({0}). Unplug it and plug it in again.",
                                                       "转接环没有响应（{0}）。请拔下后重新插上。"); } }
        public static string ReadyToWrite { get { return T("Ready to write firmware {0}.{1}.{2}.",
                                                           "可以写入固件 {0}.{1}.{2}。"); } }
        public static string LookingFailed { get { return T("Looking for the adapter failed: {0}",
                                                            "查找转接环失败：{0}"); } }
        public static string Preparing { get { return T("Found the adapter (bootloader record {0}.{1}.{2}). Preparing...",
                                                        "已找到转接环（引导程序记录 {0}.{1}.{2}），正在准备…"); } }
        public static string Writing { get { return T("Writing firmware...", "正在写入固件…"); } }
        public static string Done { get { return T("Done. Unplug the USB cable.", "完成。请拔下 USB 线。"); } }
        public static string Failed { get { return T("Failed: {0}", "失败：{0}"); } }

        // ---- dialogs
        public static string Confirm { get { return T(
            "The firmware will now be written to the adapter with this configuration:\n\n{0}\n" +
            "Do not unplug the USB cable or close this window until it finishes.",
            "即将按以下配置把固件写入转接环：\n\n{0}\n" +
            "写入完成前，请勿拔下 USB 线，也不要关闭本窗口。"); } }
        public static string Success { get { return T(
            "The firmware was written successfully.\n\nUnplug the USB cable now.",
            "固件写入成功。\n\n现在请拔下 USB 线。"); } }
        public static string WriteFailed { get { return T("Writing the firmware failed.\n\n{0}",
                                                          "固件写入失败。\n\n{0}"); } }
        public static string NotFound { get { return T(
            "The adapter was not found.\n\nConnect it to this computer's USB port.",
            "未找到转接环。\n\n请将它连接到这台电脑的 USB 接口。"); } }
        public static string PleaseWait { get { return T("Please wait until the firmware has been written.",
                                                         "请等待固件写入完成。"); } }

        // ---- help pages
        public static string Ok { get { return T("OK", "确定"); } }
        public static string PictureUnavailable { get { return T("(picture unavailable: {0})",
                                                                 "（无法显示图片：{0}）"); } }
        public static string HelpAdapterTitle { get { return T("Adapter Version", "转接环版本"); } }
        public static string HelpDifference { get { return T("What is the difference", "有什么区别"); } }
        public static string HelpDifferenceText { get { return T(
            "Version 1 has less room past infinity than version 2. To focus at infinity, " +
            "version 1 has to retract all the way to its end stop. On version 2 the firmware " +
            "does not need to, which is easier on the mechanism and gives it a longer life.",
            "版本 1 在无穷远之外的余量比版本 2 小。为了对焦到无穷远，版本 1 必须一直退到机械限位；" +
            "版本 2 的固件则不需要这样做，对机构的磨损更小，寿命也更长。"); } }
        public static string HelpHowToChoose { get { return T("How to choose", "如何选择"); } }
        public static string HelpAdapterChoose { get { return T(
            "Only if your adapter is version 1 hardware (see the picture) " +
            "and it cannot focus at infinity, try version 1.",
            "只有当你的转接环是版本 1 的硬件（见上图），并且无法对焦到无穷远时，才尝试选择版本 1。"); } }
        public static string HelpFocalTitle { get { return T("Switching Lenses", "切换镜头"); } }
        public static string HelpMultipleText { get { return T(
            "Enter each lens's details in the row of the aperture you want to use for it. " +
            "To change lenses, set the camera to that aperture and press the shutter: the " +
            "adapter will restart and switch to that lens's details. Remember to set the " +
            "aperture back to its widest value before you shoot again.",
            "把每支镜头的信息填在你想用来选择它的那个光圈所在的行。换镜头时，把相机设为对应的光圈" +
            "并按下快门：转接环会重启，并切换到这支镜头的信息。再次拍摄前，记得把光圈调回最大。"); } }
        public static string HelpHomeResetText { get { return T(
            "f6.3 resets the motor homing: set the camera to f6.3 and press the shutter. The " +
            "adapter restarts and finds its infinity end stop again. Do this if focus at " +
            "infinity is off.",
            "f6.3 用来重置马达归位：把相机设为 f6.3 并按下快门，转接环会重启，并重新寻找无穷远限位。" +
            "如果无穷远对焦不准，可以这样做一次。"); } }

        public static string HelpXpdTitle { get { return T("Exit Pupil", "出瞳距离"); } }
        public static string HelpXpdText { get { return T(
            "The exit pupil distance affects how accurately phase-detect autofocus works. " +
            "Usually the exact optical data of a lens is not known; then follow these rules:\n\n" +
            "1. Start with a value close to the focal length (one is selected for you when you " +
            "enter the focal length).\n\n" +
            "2. If you are not happy with how the camera focuses toward the edges of the frame, " +
            "try a smaller or a larger value.\n\n" +
            "3. Rangefinder lenses usually have a shorter exit pupil distance than SLR lenses, and " +
            "lenses with better image quality usually have a longer one.",
            "出瞳距离会影响相位对焦的准确性。多数情况下你无法得知镜头精确的光学参数，" +
            "可以按以下原则选择：\n\n" +
            "1. 先选一个与焦距相近的数值（输入焦距后会自动选好）。\n\n" +
            "2. 如果对相机在画面边缘的对焦效果不满意，尝试减小或增大这个数值。\n\n" +
            "3. 通常旁轴镜头的出瞳距离比单反镜头短；画质更好的镜头，出瞳距离通常更长。"); } }

        // ---- lens configuration
        public static string ChooseHwVersion { get { return T("Choose a hardware version.", "请选择硬件版本。"); } }
        // Where an error is, put in front of the messages below: the slot.
        public static string InSlot { get { return T(" in the {0} row", "{0} 那一行的"); } }
        public static string FocalWhole { get { return T("The focal length{0} must be a whole number of millimetres.",
                                                         "{0}焦距必须是整数（单位 mm）。"); } }
        public static string FocalRange { get { return T("The focal length{0} must be from {1} to {2} mm.",
                                                         "{0}焦距必须在 {1} 到 {2} mm 之间。"); } }
        public static string NameTooLong { get { return T("The lens name{0} can be at most {1} characters.",
                                                          "{0}镜头名称最多 {1} 个字符。"); } }
        public static string NameAscii { get { return T(
            "The lens name{0} can only use English letters, digits, spaces and common symbols.",
            "{0}镜头名称只允许输入英文字母、数字和符号。"); } }
        public static string NoLens { get { return T(
            "No lens is configured: enter the focal length of at least one slot.",
            "尚未配置镜头：请至少在一行中输入焦距。"); } }
        public static string DescAdapter { get { return T("Adapter version: {0}", "转接环版本：{0}"); } }
        public static string ChooseXpd { get { return T("Choose an exit pupil{0}.", "请选择{0}出瞳距离。"); } }
        // {0}, the slot number, is not shown: the window names rows by aperture.
        public static string DescSlot { get { return T("{1}: {2} mm, exit pupil {3} mm, \"{4}\"",
                                                       "{1}：{2} mm，出瞳 {3} mm，“{4}”"); } }
    }
}
