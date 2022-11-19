using OVRSharp;
using System;
using System.Diagnostics;
using System.Threading;
using uOSC;
using Emgu.CV; // the mom
using Emgu.CV.Aruco; // the hero
using Emgu.CV.CvEnum; // the book
using Emgu.CV.Structure; // the storage
using Emgu.CV.Util; // the side kick
using System.Collections.Generic;
using System.Numerics;
using System.IO;

namespace TrackingSmoothing {
    class Program {
        public static bool sendRaw = false;
        public static bool legsFolded = false;

        public static bool poseAdjust = true;
        public static string poseAdjustWaist = "waist";
        public static float poseAdjustLegDist = 0.5f;

        public static float[] hmdPos = new float[3];

        public static Matrix4x4 offsetMat = Matrix4x4.Identity;
        public static Vector3 offset {
            get {
                return offsetMat.Translation;
            }
        }
        public static float rotationY = 0;
        public static List<Vector4> hmdList = new List<Vector4>();
        public static float trackerDelay = 300f;
        public static bool wantToShowFrame = false;
        public static bool wantToCloseWindows = false;
        public static int updateFPS = 80;

        public static bool ovrNotFound = false;

        public static uOscClient oscClient;
        static string oscAddress = "127.0.0.1";
        static int oscPortOut = 39570;//39570;

        public static Stopwatch timer = new Stopwatch();
        public static Valve.VR.TrackedDevicePose_t[] devPos = new Valve.VR.TrackedDevicePose_t[4];
        public static int frameCount = 0;
        static void Main(string[] args) {
            Console.WriteLine("Starting...");
            timer.Start();

            //initialize ovr application
            Application app = null;
            try {
                app = new Application(Application.ApplicationType.Background);
            } catch (Exception e) {
                Console.WriteLine($"Could not initialize ovr\nerror:{e}");
                System.Threading.Thread.Sleep(1000);
                ovrNotFound = true;
            }

            ReadConfig();

            //initialize OSC Sender
            oscClient = new uOscClient();
            oscClient.port = oscPortOut;
            oscClient.address = oscAddress;
            oscClient.StartClient();

            Aruco.Init();
            Console.Clear();
            LoadOffsets();
            Tag.ReadTrackers();
            Console.WriteLine("Started");

            //Main loop
            ShowHint();
            double previousTime = 0;

            //I fucking hate async programing, the only thing it does is crash
            System.Threading.ThreadStart updateVideoFunc = new ThreadStart(Aruco.Update);
            System.Threading.Thread videoCapture = new System.Threading.Thread(updateVideoFunc);
            videoCapture.Start();
            while (true) {
                double delta = timer.Elapsed.TotalMilliseconds - previousTime;
                previousTime = timer.Elapsed.TotalMilliseconds;
                //Console.WriteLine("time: " + delta);
                //if (frameCount % 10 == 0) {
                //    (int x, int y) = Console.GetCursorPosition();
                //    Console.SetCursorPosition(0, 0);
                //    for (int i = 0; i < Tag.cameraTPS.Length; i++) {
                //        if (i != 0) Console.Write(" / ");
                //        Console.Write($"Cam {i} TPS: {Tag.cameraTPS[i].ToString("0.00")}");
                //    }
                //    Console.Write($"\tApp TPS: {delta}");
                //    for (int i = Console.CursorLeft; i < Console.WindowWidth; i++) {
                //        Console.Write(" ");
                //    }
                //    Console.SetCursorPosition(x, y);
                //}
                //Check for keys
                if (Console.KeyAvailable) {
                    ConsoleKey key = Console.ReadKey(true).Key;
                    KeyPressed(key);
                }
                //Update OSC recieved messages
                //Get OVR Device Positions
                if (!ovrNotFound) {
                    app.OVRSystem.GetDeviceToAbsoluteTrackingPose(Valve.VR.ETrackingUniverseOrigin.TrackingUniverseRawAndUncalibrated, 0, devPos);
                }
                var m = devPos[0].mDeviceToAbsoluteTracking;
                var m2 = devPos[1].mDeviceToAbsoluteTracking;
                //Save only the HMD
                hmdPos[0] = m.m3;
                hmdPos[1] = m.m7;
                hmdPos[2] = m.m11;
                float time = timer.ElapsedMilliseconds / 1000000f;
                trackerDelay = 300f;
                hmdList.Add(new Vector4(m.m3, m.m7, m.m11, time));
                while (hmdList.Count > 1 && hmdList[0].W < time - trackerDelay / 1000000f)
                    hmdList.RemoveAt(0);

                Tag.Update();
                Tag.GetTrackers();
                if (poseAdjust)
                    Tag.AdjustPose();
                Tag.SendTrackers();
                //A mimir, wait for next frame (80fps)
                System.Threading.Thread.Sleep(1000 / updateFPS);

                var mm = devPos[3].mDeviceToAbsoluteTracking;
                frameCount++;
            }
        }
        static void ShowHint() {
            Console.WriteLine($"\nReset Trackers: O - Show Hints: Space\n" +
                $"X: 5/6 {offset.X} - Y: T/Y {offset.Y} - Z: G/H {offset.Z} - Yaw: B/N {rotationY} - Clear Console: 0 - Show Windows: 9");
        }
        static void KeyPressed(ConsoleKey key) {
            Console.WriteLine($"Pressed {key}");
            if (key == ConsoleKey.O) {
                oscClient.Send("VMT/Reset");
                Console.WriteLine($"Sent Reset to VMT");
            } else if (key == ConsoleKey.Spacebar) {
                ShowHint();
            } else if (key == ConsoleKey.D1) {
                //ArUco.cameras[0].saveMat = false;
                //ArUco.cameras[1].saveMat = false;
                offsetMat.M41 = 0f;
                offsetMat.M43 = 0f;
                offsetMat.M42 = 0f;
                rotationY = 0;
                ApplyOffset();
                Tag.cameras[0].minScore = float.MaxValue;
                Tag.cameras[1].minScore = float.MaxValue;
                Tag.endedSearching = false;
                if (timer.ElapsedMilliseconds - Tag.saveMatTime < 20000) {
                    Tag.saveMatTime = -20000;
                    Console.WriteLine("Stopped...");
                } else {
                    Tag.saveMatTime = timer.ElapsedMilliseconds;
                    Console.WriteLine("Averaging...");
                }
            } else if (key == ConsoleKey.D5) {
                offsetMat.M41 -= 0.01f;
                Console.WriteLine($"Decreased X offset {offsetMat.M41}");
            } else if (key == ConsoleKey.D6) {
                offsetMat.M41 += 0.01f;
                Console.WriteLine($"Increased X offset {offsetMat.M41}");
            } else if (key == ConsoleKey.T) {
                offsetMat.M43 -= 0.01f;
                Console.WriteLine($"Decreased Y offset {offsetMat.M43}");
            } else if (key == ConsoleKey.Y) {
                offsetMat.M43 += 0.01f;
                Console.WriteLine($"Increased Y offset {offsetMat.M43}");
            } else if (key == ConsoleKey.G) {
                offsetMat.M42 -= 0.01f;
                Console.WriteLine($"Decreased Z offset {offsetMat.M42}");
            } else if (key == ConsoleKey.H) {
                offsetMat.M42 += 0.01f;
                Console.WriteLine($"Increased Z offset {offsetMat.M42}");
            } else if (key == ConsoleKey.B) {
                rotationY -= 0.01f;
                Console.WriteLine($"Decreased Yaw offset {rotationY}");
                ApplyOffset();
            } else if (key == ConsoleKey.N) {
                rotationY += 0.01f;
                Console.WriteLine($"Increased Yaw offset {rotationY}");
                ApplyOffset();
            } else if (key == ConsoleKey.D9) {
                wantToShowFrame = !wantToShowFrame;
                if (!wantToShowFrame) wantToCloseWindows = true;
            } else if (key == ConsoleKey.D0) {
                Console.Clear();
                ShowHint();
            }
            using (StreamWriter sw = new StreamWriter("offsets")) {
                sw.WriteLine(rotationY);
                sw.WriteLine(offsetMat.M41);
                sw.WriteLine(offsetMat.M42);
                sw.WriteLine(offsetMat.M43);
            }
        }

        private static void ApplyOffset() {
            Matrix4x4 newMat = Matrix4x4.CreateFromYawPitchRoll(0, 0, rotationY);
            newMat.M41 = offsetMat.M41;
            newMat.M42 = offsetMat.M42;
            newMat.M43 = offsetMat.M43;
            offsetMat = newMat;
        }

        static void LoadOffsets() {
            Tag.ReadMatrix();
            if (File.Exists("offsets")) {
                string[] lines = File.ReadAllLines("offsets");
                int l = 0;
                rotationY = float.Parse(lines[l++]);
                Matrix4x4 newMat = Matrix4x4.CreateFromYawPitchRoll(0, 0, rotationY);
                newMat.M41 = float.Parse(lines[l++]);
                newMat.M42 = float.Parse(lines[l++]);
                newMat.M43 = float.Parse(lines[l++]);
                offsetMat = newMat;
            }
        }
        static void ReadConfig() {
            if (!File.Exists("config.txt")) return;
            string[] lines = File.ReadAllLines("config.txt");
            for (int i = 0; i < lines.Length; i++) {
                string[] split = lines[i].Split("=");
                System.Globalization.NumberStyles any = System.Globalization.NumberStyles.Any;
                System.Globalization.CultureInfo invariantCulture = System.Globalization.CultureInfo.InvariantCulture;
                if (split[0].Equals("trackerDelay")) trackerDelay = float.Parse(split[1], any, invariantCulture);
                else if (split[0].Equals("poseAdjust")) poseAdjust = split[1].Equals("true");
                else if (split[0].Equals("poseAdjustLegDistance")) poseAdjustLegDist = float.Parse(split[1], any, invariantCulture);
                else if (split[0].Equals("poseAdjustWaist")) poseAdjustWaist = split[1];
                else if (split[0].Equals("oscAddress")) oscAddress = split[1];
                else if (split[0].Equals("oscPort")) oscPortOut = int.Parse(split[1]);
                else if (split[0].Equals("camera0Quality")) { if (Tag.cameras.Length > 0) { Tag.cameras[0].quality = float.Parse(split[1], any, invariantCulture); } } //aahahahahhahahahahah
                else if (split[0].Equals("camera1Quality")) { if (Tag.cameras.Length > 1) { Tag.cameras[1].quality = float.Parse(split[1], any, invariantCulture); } }
                else if (split[0].Equals("camera0File")) { if (Tag.cameras.Length > 0) { Tag.cameras[0].file = split[1]; } }
                else if (split[0].Equals("camera1File")) { if (Tag.cameras.Length > 1) { Tag.cameras[1].file = split[1]; } }
                else if (split[0].Equals("camera0Width")) { if (Tag.cameras.Length > 0) { Tag.cameras[0].width = int.Parse(split[1]); } }
                else if (split[0].Equals("camera1Width")) { if (Tag.cameras.Length > 1) { Tag.cameras[1].width = int.Parse(split[1]); } }
                else if (split[0].Equals("camera0Height")) { if (Tag.cameras.Length > 0) { Tag.cameras[0].height = int.Parse(split[1]); } }
                else if (split[0].Equals("camera1Height")) { if (Tag.cameras.Length > 1) { Tag.cameras[1].height = int.Parse(split[1]); } }
                else if (split[0].Equals("trackerSize")) { Aruco.markersLength = int.Parse(split[1]); }
                else if (split[0].Equals("updatesPerSecond")) { updateFPS = int.Parse(split[1]); }
            }
        }
    }
}
