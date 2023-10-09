﻿using OVRSharp;
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

        public static int preNoise = 1;
        public static int postNoise = 1;
        public static int clusterRotationGuess = 1;
        public static bool ignoreNoisyRotation = true;

        public static float[] hmdPos = new float[3];
        public static float[] hmdRot = new float[3];

        public static Matrix4x4 offsetMat = Matrix4x4.Identity;
        public static Vector3 offset {
            get {
                return offsetMat.Translation;
            }
        }
        public static Vector3 roomOffset = Vector3.Zero;
        public static float rotationX = 0;
        public static float rotationY = 0;
        public static float rotationZ = 0;
        public static List<Vector4> hmdList = new List<Vector4>();
        public static float trackerDelay = 300f;
        public static bool wantToShowFrame = false;
        public static bool adjustOffset = false;
        public static bool autoOffset = false;
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
        public static int nextSave = 10;
        public static Valve.VR.TrackedDevicePose_t[] devPos = new Valve.VR.TrackedDevicePose_t[4];
        public static Valve.VR.HmdMatrix34_t prevCont = new();
        public static int frameCount = 0;
        public static string infoBarWarning = "";
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
#if RELEASE
                System.Threading.Thread.Sleep(1000);
#endif
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
            Tag.lastFrameTime = new double[Tag.cameras.Length];
            System.Threading.Thread[] videoCapture = new System.Threading.Thread[Tag.cameras.Length];
            for (int i = 0; i < Tag.cameras.Length; i++) {
                int cam = i;
                videoCapture[i] = new System.Threading.Thread(() => Aruco.Update(cam));
                //Console.WriteLine($"start {i}");
                videoCapture[i].Start();
            }
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
                    Console.Write($"App TPS: {(1000.0 / delta):0.00}");
                    initX = Console.CursorLeft;
                    endX = (initX / 8 + 2) * 8;
                    for (int i = initX; i < endX; i++) {
                        Console.Write(" ");
                    }
                    Console.Write(infoBarWarning);
                    infoBarWarning = "";
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
                Matrix4x4 hmdCentered = hmdRotMat;
                hmdCentered = Matrix4x4.Multiply(Matrix4x4.CreateTranslation(new Vector3(0f, 0f, 0.09f)), hmdCentered);
                Vector3 hmdPosV3 = hmdCentered.Translation;
                hmdPos[0] = hmdPosV3.X;
                hmdPos[1] = hmdPosV3.Y;
                hmdPos[2] = hmdPosV3.Z;
                Vector3 hmdRotEuler = Tag.ToEulerAngles(Quaternion.CreateFromRotationMatrix(hmdCentered));
                hmdRot[0] = hmdRotEuler.X;
                hmdRot[1] = hmdRotEuler.Y;
                hmdRot[2] = hmdRotEuler.Z;

                float time = timer.ElapsedMilliseconds / 1000000f;
                hmdList.Add(new Vector4(hmdPosV3.X, hmdPosV3.Y, hmdPosV3.Z, time));
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
                    //mat.M42 = hmdRotMat.M42;
                    Vector3 matT = mat.Translation;
                    oscClientDebug.Send("/debug/final/position", 9,
                                               matT.X, matT.Y, -matT.Z, //1f, 1.7f, 1f
                                               0, 0, 0, 1);

                    int waist = -1;
                    int leftfoot = -1;
                    int rightfoot = -1;
                    for (int i = 0; i < Tag.finals.Length; i++) {
                        if (Tag.finals[i].name.Equals("leftfoot"))
                            leftfoot = i;
                        if (Tag.finals[i].name.Equals("rightfoot"))
                            rightfoot = i;
                        if (Tag.finals[i].name.Equals(poseAdjustWaist))
                            waist = i;
                    }
                    Vector3 pos = Tag.finals[waist].fpos;
                    Quaternion q = Tag.finals[waist].frot;
                    Matrix4x4 matw = Matrix4x4.Multiply(Matrix4x4.CreateFromQuaternion(q), Matrix4x4.CreateTranslation(pos));
                    matw = Matrix4x4.Multiply(Matrix4x4.CreateTranslation(new Vector3(-0.15f, 0f, 0f)), matw);
                    //matw = Matrix4x4.Multiply(mat, Program.offsetMat);
                    //matw.M41 -= (Program.hmdList[0].X - Program.hmdPos[0]) * Tag.trackers[waist].trackerFollowWeight;
                    //matw.M43 -= (Program.hmdList[0].Y - Program.hmdPos[1]) * Tag.trackers[waist].trackerFollowWeight;
                    //matw.M42 += (Program.hmdList[0].Z - Program.hmdPos[2]) * Tag.trackers[waist].trackerFollowWeight;
                    oscClientDebug.Send("/debug/final/position", 6,
                                               matw.Translation.X, matw.Translation.Z, matw.Translation.Y, //1f, 1.7f, 1f
                                               -q.X, -q.Z, -q.Y, q.W);
                }
                if (autoOffset) {
                    AdjustOffset(hmdRotMat);
                    autoOffset = false;
                    Console.WriteLine("Adjusted offsets");
                }
                //A mimir, wait for next frame (80fps)
                System.Threading.Thread.Sleep(1000 / updateFPS);

                var mm = devPos[3].mDeviceToAbsoluteTracking;
                frameCount++;
            }
        }

        private static void AdjustOffset(Matrix4x4 hmdRotMat) {
            offsetMat = Matrix4x4.Identity;
            Matrix4x4 mat = hmdRotMat;
            mat = Matrix4x4.Multiply(Matrix4x4.CreateTranslation(new Vector3(0f, 0f, 0.18f)), mat); //0.09 = center of head, 0.2 = center of body
            mat.M42 = hmdRotMat.M42;
            //hmdRotMat = Matrix4x4.Multiply(hmdRotMat, Matrix4x4.CreateRotationY((float)Math.PI));
            float asd = mat.M42;
            mat.M42 = mat.M43;
            mat.M43 = -asd;
            mat.M41 = -mat.M41;



            asd = hmdRotMat.M42;
            hmdRotMat.M42 = hmdRotMat.M43;
            hmdRotMat.M43 = -asd;
            hmdRotMat.M41 = -hmdRotMat.M41;
            int waist = -1;
            int leftfoot = -1;
            int rightfoot = -1;
            for (int i = 0; i < Tag.finals.Length; i++) {
                if (Tag.finals[i].name.Equals("leftfoot"))
                    leftfoot = i;
                if (Tag.finals[i].name.Equals("rightfoot"))
                    rightfoot = i;
                if (Tag.finals[i].name.Equals(poseAdjustWaist))
                    waist = i;
            }
            Vector3 mid = (Tag.finals[leftfoot].preMat.Translation + Tag.finals[rightfoot].preMat.Translation) / 2f;
            Matrix4x4 dir = Matrix4x4.CreateLookAt(mid, Tag.finals[rightfoot].preMat.Translation, new Vector3(0, 1f, 0));
            Matrix4x4 newWaist = Matrix4x4.Multiply(Tag.finals[waist].preMat, dir);
            newWaist = Matrix4x4.Multiply(Matrix4x4.CreateTranslation(new Vector3(-0.15f, 0f, 0f)), newWaist);
            dir = Matrix4x4.Multiply(dir, Matrix4x4.CreateLookAt(Vector3.Zero, newWaist.Translation, new Vector3(0, 1f, 0)));
            dir = Matrix4x4.Multiply(dir, Matrix4x4.CreateLookAt(Vector3.Zero, hmdRotMat.Translation - mat.Translation, new Vector3(0, 1f, 0)));
            dir = Matrix4x4.Multiply(dir, Matrix4x4.CreateFromYawPitchRoll(-(float)Math.PI / 2f, (float)Math.PI * 1.5f, 0));
            newWaist = Matrix4x4.Multiply(Tag.finals[waist].preMat, dir);
            //asd = newWaist.M42;
            //newWaist.M42 = newWaist.M43;
            //newWaist.M43 = -asd;
            newWaist.M43 = -newWaist.M43;
            newWaist = Matrix4x4.Multiply(Matrix4x4.CreateTranslation(new Vector3(-0.15f, 0f, 0f)), newWaist);
            //oscClientDebug.Send("/debug/final/position", 10,
            //                           mat.Translation.X, mat.Translation.Y, -mat.Translation.Z, //1f, 1.7f, 1f
            //                           0, 0, 0, 1);
            //oscClientDebug.Send("/debug/final/position", 9,
            //                           newWaist.Translation.X, newWaist.Translation.Y, -newWaist.Translation.Z, //1f, 1.7f, 1f
            //                           0, 0, 0, 1);
            Matrix4x4 diff;
            Matrix4x4.Invert(Matrix4x4.CreateTranslation(mat.Translation - newWaist.Translation), out diff);
            dir = Matrix4x4.Multiply(dir, diff);
            dir.M43 -= poseAdjustLegDist * 1f;
            //Matrix4x4.Invert(dir, out offsetMat);
            offsetMat = dir;
            offsetMat.M41 = offsetMat.M41 + roomOffset.X;
            offsetMat.M42 = offsetMat.M42 + roomOffset.Y;
            offsetMat.M43 = offsetMat.M43 + roomOffset.Z;

            Quaternion q = Quaternion.CreateFromRotationMatrix(offsetMat);
            var yaw = Math.Atan2(2.0 * (q.Y * q.Z + q.W * q.X), q.W * q.W - q.X * q.X - q.Y * q.Y + q.Z * q.Z);
            var pitch = Math.Asin(-2.0 * (q.X * q.Z - q.W * q.Y));
            var roll = Math.Atan2(2.0 * (q.X * q.Y + q.W * q.Z), q.W * q.W + q.X * q.X - q.Y * q.Y - q.Z * q.Z);
            rotationZ = (float)yaw;
            rotationX = (float)pitch;
            rotationY = (float)roll;
            //Matrix4x4.Invert(diff, out offsetMat);
        }

        static void ShowHint() {
            Console.WriteLine($"\n[D8] Reset Trackers (VMT)\n[Space] Show Hints\n[D1] Calibrate Cameras\n[D2] Manual Offset Adjust: {Show(adjustOffset)}" +
                $"\n[D3] Reload Offsets\n[D4] Reloaded Config/Trackers\n[D5] Auto Adjust Offsets\n[D7] Send Debug Trackers: {Show(debugSendTrackerOSC)}\n[D9] Show Camera Windows\n[D0] Clear Console" +
                $"\n[M] Pre-Pose Noise Reduction: {(preNoise == 0 ? "Disabled" : preNoise == 1 ? "Enabled" : "Smooth rects off")}\n[N] Post-Pose Noise Reduction: {(postNoise == 0 ? "Disabled" : postNoise == 1 ? "Enabled" : "Partial")}" +
                $"\n[Q]-[W] X: {offset.X}\n[A]-[S] Y: {offset.Y}\n[Z]-[X] Z: {offset.Z}\n[E]-[R] Yaw: {rotationY}\n[D]-[F] xRot: {rotationX}\n[C]-[V] zRot: {rotationZ}");
            if (ovrNotFound) {
                Console.ForegroundColor = ConsoleColor.Red;
                Console.WriteLine($"OVR not initialized, Restart app");
                Console.ResetColor();
            }
        }
        static void KeyPressed(ConsoleKey key) {
            Console.Write($"Pressed {key}: ");
            offsetMat.M41 = offsetMat.M41 - roomOffset.X;
            offsetMat.M42 = offsetMat.M42 - roomOffset.Y;
            offsetMat.M43 = offsetMat.M43 - roomOffset.Z;
            if (key == ConsoleKey.D8) {
                oscClient.Send("VMT/Reset");
                Console.WriteLine($"Sent Reset to VMT");
            } else if (key == ConsoleKey.Spacebar) {
                ShowHint();
            } else if (key == ConsoleKey.D1) {
                if (Tag.getRawTrackersStep > -1) {
                    if (Tag.getRawTrackersStep != -2) {
                        Tag.cameras[0].minScore = float.MaxValue;
                        Tag.cameras[1].minScore = float.MaxValue;
                        Tag.endedSearching = false;
                        Tag.saveMatTime = timer.ElapsedMilliseconds;
                        Tag.getRawTrackersStep = -2;
                        Console.WriteLine("Averaging...");
                    }
                } else if (Tag.getRawTrackersStep == -2) {
                    Tag.saveMatTime = -20000;
                    Console.WriteLine("Stopped...");
                } else {
                    Tag.addNewRaw = false;
                    Tag.newTrackersReady = false;
                    Tag.combinedTrackers = new();
                    offsetMat.M41 = 0f;
                    offsetMat.M43 = 0f;
                    offsetMat.M42 = 0f;
                    rotationY = 0;
                    rotationZ = 0;
                    rotationX = 0;
                    ApplyOffset();
                    Tag.getRawTrackersStep = 0;
                    Tag.getSnapshot = false;
                    Tag.timedSnapshot = false;
                    Console.WriteLine("Press [1] to start averaging, press [2] to add available trackers, press [3] to add automatically");
                }
            } else if (key == ConsoleKey.D2) {
                if (Tag.getRawTrackersStep > -1) {
                    Tag.addNewRaw = true;
                } else {
                    cannotGetControllers = false;
                    adjustOffset = !adjustOffset;
                    isMoveKeyPressed = false;
                    isRotateKeyPressed = false;
                    Console.WriteLine($"Adjust offset by hand: " + Show(adjustOffset));
                    if (adjustOffset)
                        Console.WriteLine("Move offset with Grip, rotate with Trigger. Or press [P] to move and [O] to rotate instead");
                }
            } else if (key == ConsoleKey.D3) {
                if (Tag.getRawTrackersStep > -1) {
                    Tag.timedSnapshot = !Tag.timedSnapshot;
                    Console.WriteLine($"Timed snap {(Tag.timedSnapshot ? "activated" : "deactivated")}");
                } else {
                    LoadOffsets();
                    Console.WriteLine($"Reloaded Offsets");
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
                rotationY -= 0.03f;
                Console.WriteLine($"Decreased Yaw offset {rotationY}");
                ApplyOffset();
            } else if (key == ConsoleKey.R) {
                rotationY += 0.03f;
                Console.WriteLine($"Increased Yaw offset {rotationY}");
                ApplyOffset();
            } else if (key == ConsoleKey.D) {
                rotationX -= 0.03f;
                Console.WriteLine($"Decreased xRot offset {rotationX}");
                ApplyOffset();
            } else if (key == ConsoleKey.F) {
                rotationX += 0.03f;
                Console.WriteLine($"Increased xRot offset {rotationX}");
                ApplyOffset();
            } else if (key == ConsoleKey.C) {
                rotationZ -= 0.03f;
                Console.WriteLine($"Decreased zRot offset {rotationZ}");
                ApplyOffset();
            } else if (key == ConsoleKey.V) {
                rotationZ += 0.03f;
                Console.WriteLine($"Increased zRot offset {rotationZ}");
                ApplyOffset();
            } else if (key == ConsoleKey.D9) {
                wantToShowFrame = !wantToShowFrame;
                if (!wantToShowFrame) wantToCloseWindows = true;
            } else if (key == ConsoleKey.D0) {
                Console.Clear();
                Console.WriteLine();
                ShowHint();
            } else if (key == ConsoleKey.D7) {
                debugSendTrackerOSC = !debugSendTrackerOSC;
                Console.WriteLine($"Send Debug Trackers: " + Show(debugSendTrackerOSC));
                if (debugSendTrackerOSC) {
                    if (!oscClientDebug.isRunning) {
                        oscClientDebug.StartClient();
                    }
                    Process.Start(@"viewer\tagTrackingViewer.exe");
                    ProcessStartInfo processInfo = new ProcessStartInfo();
                    processInfo.FileName = @"viewer\tagTrackingViewer.exe";
                    processInfo.ErrorDialog = true;
                    processInfo.UseShellExecute = false;
                    processInfo.RedirectStandardOutput = true;
                    processInfo.RedirectStandardError = true;
                    processInfo.WorkingDirectory = Path.GetDirectoryName(@"viewer\tagTrackingViewer.exe");
                    Process.Start(processInfo);
                } else {
                    if (oscClientDebug.isRunning) {
                        oscClientDebug.StopClient();
                    }
                }
            } else if (key == ConsoleKey.D4) {
                ReadConfig();
                Tag.ReadTrackers();
                Console.WriteLine($"Reloaded Config / Trackers");
                ApplyOffset();
                LoadOffsets();
            } else if (key == ConsoleKey.D5) {
                autoOffset = !autoOffset;
                Console.WriteLine($"Auto Adjust Offset");
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
                preNoise++;
                if (preNoise > 2) preNoise = 0;
                Console.WriteLine($"Toggle pre-pose noise reduction {(preNoise == 0 ? "Disabled" : preNoise == 1 ? "Enabled" : "Smooth rects off")}");
            } else if (key == ConsoleKey.N) {
                postNoise++;
                if (postNoise > 2) postNoise = 0;
                Tag.SetFinalTrackers(postNoise == 2 ? 0.5f : 1f);
                Console.WriteLine($"Toggle post-pose noise reduction {(postNoise == 0 ? "Disabled" : postNoise == 1 ? "Enabled" : "Partial")}");
            } else if (key == ConsoleKey.B) {
                clusterRotationGuess++;
                if (clusterRotationGuess > 2) clusterRotationGuess = 0;
                Console.WriteLine($"Guess cluster rotation: {(clusterRotationGuess == 0 ? "Disabled" : clusterRotationGuess == 1 ? "Enabled" : "CPU Heavy")}");
            }
            offsetMat.M41 = offsetMat.M41 + roomOffset.X;
            offsetMat.M42 = offsetMat.M42 + roomOffset.Y;
            offsetMat.M43 = offsetMat.M43 + roomOffset.Z;
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
                if (lines[i][0] == ';') continue;
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
                else if (split[0].Equals("preNoiseReduction")) preNoise = int.Parse(split[1]);
                else if (split[0].Equals("postNoiseReduction")) postNoise = int.Parse(split[1]);
                else if (split[0].Equals("enableIgnoreNoisyRotation")) ignoreNoisyRotation = split[1].Equals("true");
                else if (split[0].Equals("oscAddress")) oscAddress = split[1];
                else if (split[0].Equals("oscPort")) oscPortOut = int.Parse(split[1]);
                else if (split[0].Equals("useVrchatOscTrackers")) useVRChatOSCTrackers = split[1].Equals("true");
                else if (split[0].Equals("sendHeadTracker")) sendHeadTracker = split[1].Equals("true");
                else if (split[0].Equals("roomOffsetX")) roomOffset.X = float.Parse(split[1], any, invariantCulture);
                else if (split[0].Equals("roomOffsetY")) roomOffset.Y = float.Parse(split[1], any, invariantCulture);
                else if (split[0].Equals("roomOffsetZ")) roomOffset.Z = float.Parse(split[1], any, invariantCulture);
                else if (split[0].Equals("trackerSize")) Aruco.markersLength = int.Parse(split[1]);
                else if (split[0].Equals("updatesPerSecond")) updateFPS = int.Parse(split[1]);
                else if (split[0].Equals("useSmoothCorners")) Aruco.useSmoothCorners = split[1].Equals("true");
                else if (split[0].Equals("cornersMaxDistance")) Aruco.cornersMaxDistance = int.Parse(split[1]);
                else if (split[0].Equals("cornersSmoothFactor")) Aruco.cornersSmoothFactor = float.Parse(split[1], any, invariantCulture);
                else if (split[0].Equals("refineSearch")) Tag.refineSearch = split[1].Equals("true");
                else if (split[0].Equals("refineIterations")) Tag.refineIterations = int.Parse(split[1]);
                else if (split[0].Equals("dynamicFiltering")) Tag.dynamicFiltering = split[1].Equals("true");
                else if (split[0].Equals("tagsToCalibrate")) {
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
                } else if (split[0].Equals("totalCameras")) {
                    Tag.cameras = new Tag.Camera[int.Parse(split[1])];
                    for (int j = 0; j < Tag.cameras.Length; j++) {
                        Tag.cameras[j] = new Tag.Camera(new Matrix4x4(
                            -0.611f, 0.489f, -0.623f, -0.369f,
                            0.790f, 0.324f, -0.520f, 0.026f,
                            -0.053f, -0.81f, -0.584f, 2.268f,
                            0.0f, 0.0f, 0.0f, 1.0f), 1f, 1f);
                    }
                } else if (split[0].Contains("camera")) {
                    for (int j = 0; j < Tag.cameras.Length; j++) {
                        if (split[0].Equals($"camera{j}Quality")) {
                            Tag.cameras[j].quality = float.Parse(split[1], any, invariantCulture);
                        } else if (split[0].Equals($"camera{j}File")) {
                            Tag.cameras[j].file = split[1];
                        } else if (split[0].Equals($"camera{j}Width")) {
                            Tag.cameras[j].width = int.Parse(split[1]);
                        } else if (split[0].Equals($"camera{j}Height")) {
                            Tag.cameras[j].height = int.Parse(split[1]);
                        } else if (split[0].Equals($"camera{j}ResizeWidth")) {
                            Tag.cameras[j].rsWidth = int.Parse(split[1]);
                        } else if (split[0].Equals($"camera{j}ResizeHeight")) {
                            Tag.cameras[j].rsHeight = int.Parse(split[1]);
                        } else if (split[0].Equals($"camera{j}Index")) {
                            Tag.cameras[j].index = int.Parse(split[1]);
                        } else if (split[0].Equals($"camera{j}WorldResize")) {
                            Tag.cameras[j].depthMult = float.Parse(split[1], any, invariantCulture);
                        } else if (split[0].Equals($"camera{j}Brightness")) {
                            Tag.cameras[j].brightness = float.Parse(split[1], any, invariantCulture);
                        } else if (split[0].Equals($"camera{j}FrameSkip")) {
                            Tag.cameras[j].skipFrames = int.Parse(split[1]);
                        }
                    }
                }
            }
            Tag.cameraTPS = new double[Tag.cameras.Length];
        }
    }
}
