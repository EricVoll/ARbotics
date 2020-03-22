using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace Assets.RosSharp.Scripts
{
    public static class Logger
    {
        private static Action<string> LogDelegate { get; set; } = DefaultOutput;
        private static Action<string> ErrorDelegate { get; set; } = DefaultOutput;

        public static void setLogDelegate(Action<string> logDelegate)
        {
            LogDelegate = logDelegate;
        }
        public static void setErrorDelegate(Action<string> errorDelegate)
        {
            ErrorDelegate = errorDelegate;
        }

        public static void Log(string text)
        {
            LogDelegate?.Invoke(text);
        }
        public static void Error(string text)
        {
            ErrorDelegate?.Invoke(text);
        }

        public static void DefaultOutput(string text)
        {
            Console.WriteLine(text);
        }
    }
}
