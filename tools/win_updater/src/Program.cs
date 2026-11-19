using System;
using System.Globalization;
using System.Windows.Forms;

namespace EA9Updater
{
    static class Program
    {
        [STAThread]
        static void Main()
        {
            Application.EnableVisualStyles();
            Application.SetCompatibleTextRenderingDefault(false);
            // Chinese on a Chinese Windows (simplified or traditional),
            // English everywhere else.
            S.Zh = CultureInfo.CurrentUICulture.TwoLetterISOLanguageName == "zh";
            Application.Run(new MainForm());
        }
    }
}
