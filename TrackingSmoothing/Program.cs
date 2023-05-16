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

        public static bool preNoise = true;
        public static int postNoise = 1;
        public static bool ignoreNoisyRotation = true;

        public static float[] hmdPos = new float[3];
        public static float[] hmdRot = new float[3];

        public static Matrix4x4 offsetMat = Matrix4x4.Identity;
        public static Vector3 offset {
            get {
                return offsetMat.Translation;
            }
        }
        public static float rotationX = 0;
        public static float rotationY = 0;
        public static float rotationZ = 0;
        public static List<Vector4> hmdList = new List<Vector4>();
        public static float trackerDelay = 300f;
        public static bool wantToShowFrame = false;
        public static bool adjustOffset = false;
        public static bool cannotGetControllers = false;
        public static bool isMoveKeyPressed = false;
        public static bool isRotateKeyPressed = false;
        public static bool wantToCloseWindows = false;
        public static int updateFPS = 80;

        public static bool ovrNotFound = false;

        public static uOscClient oscClient;
        public static uOscClient oscClientDebug;
        static string oscAddress = "127.0.0.1";
        static int oscPortOut = 39570;//39570;
        static int oscPortOutDebug = 15460;//15460
        public static bool useVRChatOSCTrackers = false;
        public static bool sendHeadTracker = true;
        public static bool debugSendTrackerOSC = false;

        public static Stopwatch timer = new Stopwatch();
        public static Valve.VR.TrackedDevicePose_t[] devPos = new Valve.VR.TrackedDevicePose_t[4];
        public static Valve.VR.HmdMatrix34_t prevCont = new();
        public static int frameCount = 0;
        static void Main(string[] args) {
            /////////////////////////////////////////////////
            ///TODO:    Fix all this crap of laggy code
            ///         Add auto offset adjust
            /////////////////////////////////////////////////


            Console.WriteLine("Starting...");
            timer.Start();

            //initialize ovr application
            Application app = null;
            try {
                app = new Application(Application.ApplicationType.Background);
            } catch (Exception e) {
                Console.ForegroundColor = ConsoleColor.Red;
                Console.WriteLine($"Could not initialize ovr\nerror:{e}");
                System.Threading.Thread.Sleep(1000);
                ovrNotFound = true;
                Console.ResetColor();
            }

            ReadConfig();

            //initialize OSC Sender
            oscClient = new uOscClient();
            oscClient.port = oscPortOut;
            oscClient.address = oscAddress;
            oscClient.StartClient();
            oscClientDebug = new uOscClient();
            oscClientDebug.port = oscPortOutDebug;
            oscClientDebug.address = oscAddress;

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
                if (frameCount % 10 == 0) {
                    Console.ForegroundColor = ConsoleColor.Black;
                    Console.BackgroundColor = ConsoleColor.Gray;
                    (int x, int y) = Console.GetCursorPosition();
                    int ch = Console.WindowHeight;
                    int ypos = (y + 1) - ch;
                    if (ypos < 0) ypos = 0;
                    Console.SetCursorPosition(0, ypos);
                    for (int i = 0; i < Tag.cameraTPS.Length; i++) {
                        if (i != 0) Console.Write(" / ");
                        Console.Write($"Cam {i} TPS: {Tag.cameraTPS[i]:0.00}");
                    }
                    int initX = Console.CursorLeft;
                    int endX = (initX / 8 + 1) * 8;
                    for (int i = initX; i < endX; i++) {
                        Console.Write(" ");
                    }
                    Console.Write($"App TPS: {delta:0.00}");
                    for (int i = Console.CursorLeft; i < Console.WindowWidth; i++) {
                        Console.Write(" ");
                    }
                    Console.SetCursorPosition(x, y);
                    Console.ResetColor();
                }
                //Check for keys
                if (Console.KeyAvailable) {
                    ConsoleKey key = Console.ReadKey(true).Key;
                    KeyPressed(key);
                }
                //Update OSC recieved messages
                //Get OVR Device Positions
                if (!ovrNotFound) {
                    app.OVRSystem.GetDeviceToAbsoluteTrackingPose(Valve.VR.ETrackingUniverseOrigin.TrackingUniverseRawAndUncalibrated, 0, devPos);

                    if (adjustOffset) {
                        var controller = app.OVRSystem.GetTrackedDeviceIndexForControllerRole(Valve.VR.ETrackedControllerRole.RightHand);
                        Valve.VR.VRControllerState_t state = new();
                        bool lol = app.OVRSystem.GetControllerState(controller, ref state, (uint)System.Runtime.InteropServices.Marshal.SizeOf(state));
                        bool moveTrigger = state.rAxis2.x > 0.5f;
                        bool rotateTrigger = state.rAxis1.x > 0.5f;

                        //lazy fix
                        if (!lol) {
                            if (!cannotGetControllers) {
                                cannotGetControllers = true;
                                Console.ForegroundColor = ConsoleColor.Red;
                                Console.WriteLine("Cannot get controller input, use [P] to move and [O] to rotate instead");
                                Console.ResetColor();
                            }
                        }
                        if (isMoveKeyPressed) moveTrigger = true;
                        if (isRotateKeyPressed) rotateTrigger = true;

                        //i separate translate offset and rotate offset bc my brain just thinking about matrices
                        var m2 = devPos[2].mDeviceToAbsoluteTracking;
                        //Console.WriteLine($"{m2.m0:0.00},{m2.m1:0.00},{m2.m2:0.00},{m2.m3:0.00},{m2.m4:0.00},{m2.m5:0.00},{m2.m6:0.00},{m2.m7:0.00},{m2.m8:0.00},{m2.m9:0.00},{m2.m10:0.00},{m2.m11:0.00},");
                        Matrix4x4 d = new();
                        d.M41 = prevCont.m3 - m2.m3;
                        d.M42 = prevCont.m7 - m2.m7;
                        d.M43 = prevCont.m11 - m2.m11;
                        d = Matrix4x4.Multiply(Matrix4x4.CreateFromYawPitchRoll(rotationZ, rotationX, rotationY), d);
                        if (moveTrigger) {
                            offsetMat.M41 += -d.M41;
                            offsetMat.M43 += -d.M42;
                            offsetMat.M42 += d.M43;
                        } else if (rotateTrigger) {
                            rotationY += d.M41;
                            ApplyOffset();
                        }
                        prevCont = m2;
                    }
                }
                var m = devPos[0].mDeviceToAbsoluteTracking;
                //Save only the HMD
                Matrix4x4 hmdRotMat = new Matrix4x4(m.m0, m.m4, m.m8, 0, m.m1, m.m5, m.m9, 0, m.m2, m.m6, m.m10, 0, m.m3, m.m7, m.m11, 1);
                if (useVRChatOSCTrackers || true) {
                    Vector3 headInOffset = new Vector3(0, 0, 0.1f);
                    hmdRotMat = Matrix4x4.Multiply(hmdRotMat, Matrix4x4.CreateTranslation(headInOffset));
                }
                Vector3 hmdPosV3 = hmdRotMat.Translation;
                hmdPos[0] = hmdPosV3.X;
                hmdPos[1] = hmdPosV3.Y;
                hmdPos[2] = hmdPosV3.Z;
                Vector3 hmdRotEuler = Tag.ToEulerAngles(Quaternion.CreateFromRotationMatrix(hmdRotMat));
                hmdRot[0] = hmdRotEuler.X;
                hmdRot[1] = hmdRotEuler.Y;
                hmdRot[2] = hmdRotEuler.Z;

                float time = timer.ElapsedMilliseconds / 1000000f;
                trackerDelay = 300f;
                hmdList.Add(new Vector4(m.m3, m.m7, m.m11, time));
                while (hmdList.Count > 1 && hmdList[0].W < time - trackerDelay / 1000000f)
                    hmdList.RemoveAt(0);

                Tag.Update();
                Tag.GetTrackers();
                Tag.SendTrackers();
                if (debugSendTrackerOSC) {
                    oscClientDebug.Send("/debug/final/position", 10,
                                               hmdPosV3.X, hmdPosV3.Y, -hmdPosV3.Z, //1f, 1.7f, 1f
                                               0, 0, 0, 1);
                    Matrix4x4 mat = hmdRotMat;
                    mat = Matrix4x4.Multiply(Matrix4x4.CreateTranslation(new Vector3(0f, 0f, 0.09f)), mat);
                    Vector3 matT = mat.Translation;
                    oscClientDebug.Send("/debug/final/position", 9,
                                               matT.X, matT.Y, -matT.Z, //1f, 1.7f, 1f
                                               0, 0, 0, 1);
                }
                //A mimir, wait for next frame (80fps)
                System.Threading.Thread.Sleep(1000 / updateFPS);

                var mm = devPos[3].mDeviceToAbsoluteTracking;
                frameCount++;
            }
        }
        static void ShowHint() {
            Console.WriteLine($"\n[D8] Reset Trackers (VMT)\n[Space] Show Hints\n[D1] Calibrate Cameras\n[D2] Manual Offset Adjust: {Show(adjustOffset)}" +
                $"\n[D3] Reload Offsets\n[D4] Reloaded Config/Trackers\n[D7] Send Debug Trackers: {Show(debugSendTrackerOSC)}\n[D9] Show Camera Windows\n[D0] Clear Console" +
                $"\n[M] Pre-Pose Noise Reduction: {Show(preNoise)}\n[N] Post-Pose Noise Reduction: {(postNoise == 0 ? "Disabled" : postNoise == 1 ? "Enabled" : "Partial")}" +
                $"\n[Q]-[W] X: {offset.X}\n[A]-[S] Y: {offset.Y}\n[Z]-[X] Z: {offset.Z}\n[E]-[R] Yaw: {rotationY}\n[D]-[F] xRot: {rotationX}\n[C]-[V] zRot: {rotationZ}");
            if (ovrNotFound) {
                Console.ForegroundColor = ConsoleColor.Red;
                Console.WriteLine($"OVR not initialized, Restart app");
                Console.ResetColor();
            }
        }
        static void KeyPressed(ConsoleKey key) {
            Console.Write($"Pressed {key}: ");
            if (key == ConsoleKey.D8) {
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
            } else if (key == ConsoleKey.Q) {
                offsetMat.M41 -= 0.01f;
                Console.WriteLine($"Decreased X offset {offsetMat.M41}");
            } else if (key == ConsoleKey.W) {
                offsetMat.M41 += 0.01f;
                Console.WriteLine($"Increased X offset {offsetMat.M41}");
            } else if (key == ConsoleKey.A) {
                offsetMat.M43 -= 0.01f;
                Console.WriteLine($"Decreased Y offset {offsetMat.M43}");
            } else if (key == ConsoleKey.S) {
                offsetMat.M43 += 0.01f;
                Console.WriteLine($"Increased Y offset {offsetMat.M43}");
            } else if (key == ConsoleKey.Z) {
                offsetMat.M42 -= 0.01f;
                Console.WriteLine($"Decreased Z offset {offsetMat.M42}");
            } else if (key == ConsoleKey.X) {
                offsetMat.M42 += 0.01f;
                Console.WriteLine($"Increased Z offset {offsetMat.M42}");
            } else if (key == ConsoleKey.E) {
                rotationY -= 0.01f;
                Console.WriteLine($"Decreased Yaw offset {rotationY}");
                ApplyOffset();
            } else if (key == ConsoleKey.R) {
                rotationY += 0.01f;
                Console.WriteLine($"Increased Yaw offset {rotationY}");
                ApplyOffset();
            } else if (key == ConsoleKey.D) {
                rotationX -= 0.01f;
                Console.WriteLine($"Decreased xRot offset {rotationX}");
                ApplyOffset();
            } else if (key == ConsoleKey.F) {
                rotationX += 0.01f;
                Console.WriteLine($"Increased xRot offset {rotationX}");
                ApplyOffset();
            } else if (key == ConsoleKey.C) {
                rotationZ -= 0.01f;
                Console.WriteLine($"Decreased zRot offset {rotationZ}");
                ApplyOffset();
            } else if (key == ConsoleKey.V) {
                rotationZ += 0.01f;
                Console.WriteLine($"Increased zRot offset {rotationZ}");
                ApplyOffset();
            } else if (key == ConsoleKey.D9) {
                wantToShowFrame = !wantToShowFrame;
                if (!wantToShowFrame) wantToCloseWindows = true;
            } else if (key == ConsoleKey.D0) {
                Console.Clear();
                Console.WriteLine();
                ShowHint();
            } else if (key == ConsoleKey.D2) {
                cannotGetControllers = false;
                adjustOffset = !adjustOffset;
                isMoveKeyPressed = false;
                isRotateKeyPressed = false;
                Console.WriteLine($"Adjust offset by hand: " + Show(adjustOffset));
                if (adjustOffset)
                    Console.WriteLine("Move offset with Grip, rotate with Trigger. Or press [P] to move and [O] to rotate instead");
            } else if (key == ConsoleKey.D7) {
                debugSendTrackerOSC = !debugSendTrackerOSC;
                Console.WriteLine($"Send Debug Trackers: " + Show(debugSendTrackerOSC));
                if (debugSendTrackerOSC) {
                    if (!oscClientDebug.isRunning) {
                        oscClientDebug.StartClient();
                    }
                    //Process.Start(@"viewer\tagTrackingViewer.exe");
                    //ProcessStartInfo processInfo = new ProcessStartInfo();
                    //processInfo.FileName = @"viewer\tagTrackingViewer.exe";
                    //processInfo.ErrorDialog = true;
                    //processInfo.UseShellExecute = false;
                    //processInfo.RedirectStandardOutput = true;
                    //processInfo.RedirectStandardError = true;
                    //processInfo.WorkingDirectory = Path.GetDirectoryName(@"viewer\tagTrackingViewer.exe");
                    //Process.Start(processInfo);
                } else {
                    if (oscClientDebug.isRunning) {
                        oscClientDebug.StopClient();
                    }
                }
            } else if (key == ConsoleKey.D3) {
                LoadOffsets();
                Console.WriteLine($"Reloaded Offsets");
            } else if (key == ConsoleKey.D4) {
                ReadConfig();
                Tag.ReadTrackers();
                Console.WriteLine($"Reloaded Config / Trackers");
            } else if (key == ConsoleKey.P) {
                if (adjustOffset) {
                    isMoveKeyPressed = !isMoveKeyPressed;
                    Console.WriteLine($"Move offset manually {Show(isMoveKeyPressed)}");
                }
            } else if (key == ConsoleKey.O) {
                if (adjustOffset) {
                    isRotateKeyPressed = !isRotateKeyPressed;
                    Console.WriteLine($"Rotate offset manually {Show(isRotateKeyPressed)}");
                }
            } else if (key == ConsoleKey.M) {
                preNoise = !preNoise;
                Console.WriteLine($"Toggle pre-pose noise reduction {Show(preNoise)}");
            } else if (key == ConsoleKey.N) {
                postNoise++;
                if (postNoise > 2) postNoise = 0;
                Tag.SetFinalTrackers(postNoise == 2);
                Console.WriteLine($"Toggle post-pose noise reduction {(postNoise == 0 ? "Disabled" : postNoise == 1 ? "Enabled" : "Partial")}");
            }
            //if (debugSendTrackerOSC) {
            //    if (key == ConsoleKey.Z) {
            //        Matrix4x4 newRot = Matrix4x4.CreateFromYawPitchRoll(0, 0, 0.01f);
            //        Tag.cameras[0].matrix = Matrix4x4.Multiply(Tag.cameras[0].matrix, newRot);
            //    } else if (key == ConsoleKey.X) {
            //        Matrix4x4 newRot = Matrix4x4.CreateFromYawPitchRoll(0, 0, -0.01f);
            //        Tag.cameras[0].matrix = Matrix4x4.Multiply(Tag.cameras[0].matrix, newRot);
            //    } else if (key == ConsoleKey.A) {
            //        Matrix4x4 newRot = Matrix4x4.CreateFromYawPitchRoll(0, 0.01f, 0);
            //        Tag.cameras[0].matrix = Matrix4x4.Multiply(Tag.cameras[0].matrix, newRot);
            //    } else if (key == ConsoleKey.S) {
            //        Matrix4x4 newRot = Matrix4x4.CreateFromYawPitchRoll(0, -0.01f, 0);
            //        Tag.cameras[0].matrix = Matrix4x4.Multiply(Tag.cameras[0].matrix, newRot);
            //    } else if (key == ConsoleKey.Q) {
            //        Matrix4x4 newRot = Matrix4x4.CreateFromYawPitchRoll(0.01f, 0, 0);
            //        Tag.cameras[0].matrix = Matrix4x4.Multiply(Tag.cameras[0].matrix, newRot);
            //    } else if (key == ConsoleKey.W) {
            //        Matrix4x4 newRot = Matrix4x4.CreateFromYawPitchRoll(-0.01f, 0, 0);
            //        Tag.cameras[0].matrix = Matrix4x4.Multiply(Tag.cameras[0].matrix, newRot);
            //    }
            //}
            Console.WriteLine();
            using (StreamWriter sw = new StreamWriter("offsets")) {
                sw.WriteLine(rotationX);
                sw.WriteLine(rotationY);
                sw.WriteLine(rotationZ);
                sw.WriteLine(offsetMat.M41);
                sw.WriteLine(offsetMat.M42);
                sw.WriteLine(offsetMat.M43);
            }
        }

        private static string Show(bool b) {
            return b ? "Enabled" : "Disabled";
        }

        private static void ApplyOffset() {
            Matrix4x4 newMat = Matrix4x4.CreateFromYawPitchRoll(rotationZ, rotationX, rotationY);
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
                rotationX = float.Parse(lines[l++]);
                rotationY = float.Parse(lines[l++]);
                rotationZ = float.Parse(lines[l++]);
                Matrix4x4 newMat = Matrix4x4.CreateFromYawPitchRoll(rotationZ, rotationX, rotationY);
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
                else if (split[0].Equals("poseAdjustBackwardsAngle")) Tag.backwardsAngle = float.Parse(split[1], any, invariantCulture);
                else if (split[0].Equals("poseAdjustPanAngleRight")) Tag.panAngleR = float.Parse(split[1], any, invariantCulture);
                else if (split[0].Equals("poseAdjustPanAngleLeft")) Tag.panAngleL = float.Parse(split[1], any, invariantCulture);
                else if (split[0].Equals("poseAdjustWaist")) poseAdjustWaist = split[1];
                else if (split[0].Equals("preNoiseReduction")) preNoise = split[1].Equals("true");
                else if (split[0].Equals("postNoiseReduction")) postNoise = int.Parse(split[1]);
                else if (split[0].Equals("enableIgnoreNoisyRotation")) ignoreNoisyRotation = split[1].Equals("true");
                else if (split[0].Equals("oscAddress")) oscAddress = split[1];
                else if (split[0].Equals("oscPort")) oscPortOut = int.Parse(split[1]);
                else if (split[0].Equals("useVrchatOscTrackers")) useVRChatOSCTrackers = split[1].Equals("true");
                else if (split[0].Equals("sendHeadTracker")) sendHeadTracker = split[1].Equals("true");
                else if (split[0].Equals("trackerSize")) { Aruco.markersLength = int.Parse(split[1]); } else if (split[0].Equals("updatesPerSecond")) { updateFPS = int.Parse(split[1]); } else if (split[0].Equals("useSmoothCorners")) { Aruco.useSmoothCorners = split[1].Equals("true"); } else if (split[0].Equals("cornersMaxDistance")) { Aruco.cornersMaxDistance = int.Parse(split[1]); } else if (split[0].Equals("cornersSmoothFactor")) { Aruco.cornersSmoothFactor = float.Parse(split[1], any, invariantCulture); } else if (split[0].Equals("refineSearch")) { Tag.refineSearch = split[1].Equals("true"); } else if (split[0].Equals("refineIterations")) { Tag.refineIterations = int.Parse(split[1]); } else if (split[0].Equals("tagsToCalibrate")) {
                    string[] tags = split[1].Split(',');
                    Tag.tagToCalibrate = new int[tags.Length];
                    for (int j = 0; j < tags.Length; j++) {
                        Tag.tagToCalibrate[j] = int.Parse(tags[j]);
                    }
                } else if (split[0].Equals("tagsToCalibrateWeight")) {
                    string[] tags = split[1].Split(',');
                    Tag.tagToCalibrateWeight = new float[tags.Length];
                    for (int j = 0; j < tags.Length; j++) {
                        Tag.tagToCalibrateWeight[j] = float.Parse(tags[j], any, invariantCulture); ;
                    }
                } else if (split[0].Equals("tagsOnFloor")) {
                    string[] tags = split[1].Split(',');
                    Tag.tagsOnFloor = new int[tags.Length];
                    for (int j = 0; j < tags.Length; j++) {
                        Tag.tagsOnFloor[j] = int.Parse(tags[j]);
                    }
                } else if (split[0].Contains("camera")) {
                    for (int j = 0; j < 2; j++) {
                        if (split[0].Equals($"camera{j}Quality")) { if (Tag.cameras.Length > j) { Tag.cameras[j].quality = float.Parse(split[1], any, invariantCulture); } } else if (split[0].Equals($"camera{j}File")) { if (Tag.cameras.Length > j) { Tag.cameras[j].file = split[1]; } } else if (split[0].Equals($"camera{j}Width")) { if (Tag.cameras.Length > j) { Tag.cameras[j].width = int.Parse(split[1]); } } else if (split[0].Equals($"camera{j}Height")) { if (Tag.cameras.Length > j) { Tag.cameras[j].height = int.Parse(split[1]); } } else if (split[0].Equals($"camera{j}Index")) { if (Tag.cameras.Length > j) { Tag.cameras[j].index = int.Parse(split[1]); } } else if (split[0].Equals($"camera{j}WorldResize")) { if (Tag.cameras.Length > j) { Tag.cameras[j].depthMult = float.Parse(split[1], any, invariantCulture); } }
                    }
                }

            }
        }
    }
}
