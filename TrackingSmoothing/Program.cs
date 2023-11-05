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

namespace TagTracking {
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
        public static Vector3 shoulderCenterPos = new Vector3();
        public static Vector3 rightHandPos = new Vector3();
        public static Vector3 leftHandPos = new Vector3();
        public static float[] hmdRot = new float[3];
        static int rightHandID = 2;
        static int leftHandID = 1;
        static bool needsToSearchHands = false;

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
        public static List<Vector4> leftList = new List<Vector4>();
        public static List<Vector4> rightList = new List<Vector4>();
        public static List<Vector4> shoulderList = new List<Vector4>();
        public static float trackerDelay = 300f;
        public static bool wantToShowFrame = false;
        public static bool adjustOffset = false;
        public static bool autoOffset = false;
        public static bool cannotGetControllers = false;
        public static bool isMoveKeyPressed = false;
        public static bool isRotateKeyPressed = false;
        public static bool wantToCloseWindows = false;
        public static int updateFPS = 80;
        public static int interpolationTPS = 80;
        public static bool useInterpolation = true;
        public static bool performanceMode = false;
        public static float performanceUnderSample = 1.2f;
        private static bool autoActivatePerformanceMode = false;
        public static System.Threading.Thread interpolationThread;

        public static bool ovrNotFound = false;

        public static uOscClient oscClient;
        public static uOscClient oscClientDebug;
        static string oscAddress = "127.0.0.1";
        static int oscPortOut = 39570;//39570;
        static int oscPortOutDebug = 15460;//15460
        public static bool useVRChatOSCTrackers = false;
        public static bool sendHeadTracker = true;
        public static bool debugSendTrackerOSC = false;
        public static bool debugShowCamerasPosition = false;
        public static int debugTrackerToBorrow = 0;

        public static Stopwatch timer = new Stopwatch();
        public static int nextSave = 10;
        public static Valve.VR.TrackedDevicePose_t[] devPos = new Valve.VR.TrackedDevicePose_t[10];
        public static Valve.VR.HmdMatrix34_t prevCont = new();
        public static int frameCount = 0;
        public static string infoBarWarning = "";

        public static bool showThreadsMS = false;
        public static double[] threadsWorkTime = new double[4];
        public static double[] threadsIdleTime = new double[4];

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
            threadsWorkTime = new double[2 + Tag.cameras.Length]; //1. main, 2. extrapolate 3+. cameras
            threadsIdleTime = new double[2 + Tag.cameras.Length];

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
            interpolationThread = new System.Threading.Thread(() => Extrapolate.UpdateLoop());
            interpolationThread.Start();
#if DEBUG
            debugSendTrackerOSC = true;
            if (debugSendTrackerOSC) {
                if (!oscClientDebug.isRunning) {
                    oscClientDebug.StartClient();
                }
            } else {
                if (oscClientDebug.isRunning) {
                    oscClientDebug.StopClient();
                }
            }
#endif
            while (true) previousTime = UpdateLoop(app, previousTime);
        }

        private static double UpdateLoop(Application app, double previousTime) {
            Stopwatch mainThreadBenchmark = new Stopwatch();
            mainThreadBenchmark.Start();
            double delta = timer.Elapsed.TotalMilliseconds - previousTime;
            previousTime = timer.Elapsed.TotalMilliseconds;
            if (frameCount % 10 == 0) {
                SlowTickLoop(delta);
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
                    bool lol = app.OVRSystem.GetControllerState(controller, ref state, (uint)System.Runtime.InteropServices.Marshal.SizeOf(typeof(Valve.VR.VRControllerState_t)));
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
                    //getting controller state stopped working long ago
                    if (isMoveKeyPressed) moveTrigger = true;
                    if (isRotateKeyPressed) rotateTrigger = true;

                    //i separate translate offset and rotate offset bc my brain hurts just thinking about matrices
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
            hmdList.Add(new Vector4(hmdPosV3, time));
            float timeDiff = time - trackerDelay / 1000000f;
            while (hmdList.Count > 1 && hmdList[0].W < timeDiff)
                hmdList.RemoveAt(0);

            m = devPos[leftHandID].mDeviceToAbsoluteTracking;
            Matrix4x4 leftHandRotMat = new Matrix4x4(m.m0, m.m4, m.m8, 0, m.m1, m.m5, m.m9, 0, m.m2, m.m6, m.m10, 0, m.m3, m.m7, m.m11, 1);
            leftHandPos = leftHandRotMat.Translation;
            leftList.Add(new Vector4(leftHandRotMat.Translation, time));
            while (leftList.Count > 1 && leftList[0].W < timeDiff)
                leftList.RemoveAt(0);
            m = devPos[rightHandID].mDeviceToAbsoluteTracking;
            Matrix4x4 rightHandRotMat = new Matrix4x4(m.m0, m.m4, m.m8, 0, m.m1, m.m5, m.m9, 0, m.m2, m.m6, m.m10, 0, m.m3, m.m7, m.m11, 1);
            rightHandPos = rightHandRotMat.Translation;
            rightList.Add(new Vector4(rightHandRotMat.Translation, time));
            while (rightList.Count > 1 && rightList[0].W < timeDiff)
                rightList.RemoveAt(0);
            Matrix4x4 shoulderMat = Matrix4x4.Multiply(Matrix4x4.CreateTranslation(new Vector3(0f, -0.09f, 0f)), hmdCentered);
            shoulderCenterPos = shoulderMat.Translation;
            shoulderList.Add(new Vector4(shoulderMat.Translation, time));
            while (shoulderList.Count > 1 && shoulderList[0].W < timeDiff)
                shoulderList.RemoveAt(0);

            Tag.Update();
            Tag.GetTrackers();
            Tag.SendTrackers();
            //Send OVR Headset, Controller and its offsets
            if (debugSendTrackerOSC) {
                //lefthand
                var mlh = devPos[leftHandID].mDeviceToAbsoluteTracking;
                Matrix4x4 hmdRotMatLH = new Matrix4x4(mlh.m0, mlh.m4, mlh.m8, 0, mlh.m1, mlh.m5, mlh.m9, 0, mlh.m2, mlh.m6, mlh.m10, 0, mlh.m3, mlh.m7, mlh.m11, 1);
                Vector3 hmdPosV3LH = hmdRotMatLH.Translation;
                oscClientDebug.Send("/debug/final/position", 8,
                                   hmdPosV3LH.X, hmdPosV3LH.Y, -hmdPosV3LH.Z, //1f, 1.7f, 1f
                                   0, 0, 0, 1);
                //righthand
                var mrh = devPos[rightHandID].mDeviceToAbsoluteTracking;
                Matrix4x4 hmdRotMatRH = new Matrix4x4(mrh.m0, mrh.m4, mrh.m8, 0, mrh.m1, mrh.m5, mrh.m9, 0, mrh.m2, mrh.m6, mrh.m10, 0, mrh.m3, mrh.m7, mrh.m11, 1);
                Vector3 hmdPosV3RH = hmdRotMatRH.Translation;
                oscClientDebug.Send("/debug/final/position", 7,
                                   hmdPosV3RH.X, hmdPosV3RH.Y, -hmdPosV3RH.Z, //1f, 1.7f, 1f
                                   0, 0, 0, 1);

                oscClientDebug.Send("/debug/final/position", 10,
                                   hmdPosV3.X, hmdPosV3.Y, -hmdPosV3.Z, //1f, 1.7f, 1f
                                   0, 0, 0, 1);

                Matrix4x4 mat = hmdRotMat;
                //mat = Matrix4x4.Multiply(Matrix4x4.CreateTranslation(new Vector3(0f, 0f, 0.09f)), mat);
                ////mat.M42 = hmdRotMat.M42;
                Vector3 matT = mat.Translation;
                oscClientDebug.Send("/debug/final/position", 9,
                                           matT.X, matT.Y, -matT.Z, //1f, 1.7f, 1f
                                           0, 0, 0, 1);
                int waist = -1;
                for (int i = 0; i < Tag.finals.Length; i++) {
                    if (Tag.finals[i].name.Equals(poseAdjustWaist))
                        waist = i;
                }
                Vector3 pos = Tag.finals[waist].fpos;

                mat = Matrix4x4.Multiply(Matrix4x4.CreateTranslation(new Vector3(0f, -0.09f, 0f)), hmdCentered);
                oscClientDebug.Send("/debug/final/position", 6,
                                           mat.Translation.X, mat.Translation.Y, -mat.Translation.Z, //1f, 1.7f, 1f
                                           0, 0, 0, 1);

                //Quaternion q = Tag.finals[waist].frot;
                //Matrix4x4 matw = Matrix4x4.Multiply(Matrix4x4.CreateFromQuaternion(q), Matrix4x4.CreateTranslation(pos));
                //matw = Matrix4x4.Multiply(Matrix4x4.CreateTranslation(new Vector3(-0.15f, 0f, 0f)), matw);
                //oscClientDebug.Send("/debug/final/position", 6,
                //                           matw.Translation.X, matw.Translation.Z, matw.Translation.Y, //1f, 1.7f, 1f
                //                           -q.X, -q.Z, -q.Y, q.W);
            }
            if (autoOffset) {
                AdjustOffset(hmdRotMat);
                autoOffset = false;
                Console.WriteLine("Adjusted offsets");
            }
            if (needsToSearchHands) {
                SearchHands(hmdCentered);
            }
            mainThreadBenchmark.Stop();
            threadsWorkTime[0] = mainThreadBenchmark.Elapsed.TotalMilliseconds;
            mainThreadBenchmark.Restart();
            //A mimir, wait for next frame (80fps)
            System.Threading.Thread.Sleep(1000 / updateFPS);
            mainThreadBenchmark.Stop();
            threadsIdleTime[0] = mainThreadBenchmark.Elapsed.TotalMilliseconds;

            var mm = devPos[3].mDeviceToAbsoluteTracking;
            frameCount++;

            return previousTime;
        }
        static void SearchHands(Matrix4x4 hmdCentered) {
            needsToSearchHands = false;
            Matrix4x4 matLeft = Matrix4x4.Multiply(Matrix4x4.CreateTranslation(new Vector3(0.5f, 0f, 0f)), hmdCentered);
            Matrix4x4 matRight = Matrix4x4.Multiply(Matrix4x4.CreateTranslation(new Vector3(-0.5f, 0f, 0f)), hmdCentered);
            float minLeftDist = 99999999999;
            int minLeftID = 1;
            float minRightDist = 99999999999;
            int minRightID = 2;
            for (int i = 1; i < devPos.Length; i++) {
                //vmt doesnt send velocities, so filter those out
                if (devPos[i].vVelocity.v0 == 0 || devPos[i].vVelocity.v1 == 0 || devPos[i].vVelocity.v2 == 0) continue;
                if (devPos[i].vAngularVelocity.v0 == 0 || devPos[i].vAngularVelocity.v1 == 0 || devPos[i].vAngularVelocity.v2 == 0) continue;
                var m = devPos[i].mDeviceToAbsoluteTracking;
                Matrix4x4 rotMat = new Matrix4x4(m.m0, m.m4, m.m8, 0, m.m1, m.m5, m.m9, 0, m.m2, m.m6, m.m10, 0, m.m3, m.m7, m.m11, 1);
                Vector3 dPos = rotMat.Translation;
                float distL = Utils.GetDistance(dPos.X, dPos.Y, dPos.Z, matLeft.Translation.X, matLeft.Translation.Y, matLeft.Translation.Z);
                float distR = Utils.GetDistance(dPos.X, dPos.Y, dPos.Z, matRight.Translation.X, matRight.Translation.Y, matRight.Translation.Z);
                if (distL < minLeftDist) {
                    minLeftDist = distL;
                    minLeftID = i;
                }
                if (distR < minRightDist) {
                    minRightDist = distR;
                    minRightID = i;
                }
            }
            if (minLeftID == minRightID) {
                Console.WriteLine("Somthing went wrong getting hands...");
                return;
            }
            leftHandID = minLeftID;
            rightHandID = minRightID;
        }

        private static void SlowTickLoop(double delta) {
            DrawTopBar(delta);
            //double cpuUsage2 = await GetCpuUsageForProcess();
            //Console.WriteLine(">" + cpuUsage2);
        }
        private static async System.Threading.Tasks.Task<double> GetCpuUsageForProcess() {
            var startTime = DateTime.UtcNow;
            var startCpuUsage = Process.GetCurrentProcess().TotalProcessorTime;
            await System.Threading.Tasks.Task.Delay(500);

            var endTime = DateTime.UtcNow;
            var endCpuUsage = Process.GetCurrentProcess().TotalProcessorTime;
            var cpuUsedMs = (endCpuUsage - startCpuUsage).TotalMilliseconds;
            var totalMsPassed = (endTime - startTime).TotalMilliseconds;
            var cpuUsageTotal = cpuUsedMs / (Environment.ProcessorCount * totalMsPassed);
            return cpuUsageTotal * 100;
        }

        private static void DrawTopBar(double delta) {
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
            if (showThreadsMS) {
                for (int i = 0; i < threadsWorkTime.Length; i++) {
                    Console.SetCursorPosition(0, ypos + 1 + i);
                    if (i == 0) Console.Write("main");
                    else if (i == 1) Console.Write("interpolate");
                    else Console.Write($"camera {i - 2}");
                    Console.Write($": work {threadsWorkTime[i]:0.0000}ms / idle {threadsIdleTime[i]:0.0000}ms");
                    for (int j = Console.CursorLeft; j < Console.WindowWidth; j++) {
                        Console.Write(" ");
                    }
                }
            }
            Console.SetCursorPosition(x, y);
            Console.ResetColor();
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

            Quaternion asdd = Quaternion.CreateFromRotationMatrix(offsetMat);
            Utils.ToYawPitchRoll(asdd, out float newY, out float newX, out float newZ);
            rotationY = newY;
            rotationX = newX;
            rotationZ = newZ;
            //Matrix4x4.Invert(diff, out offsetMat);
        }

        static void ShowHint() {
            if (showThreadsMS)
                for (int i = 0; i < threadsWorkTime.Length; i++) {
                    Console.WriteLine();
                }
            Console.WriteLine($"\n[D8] Reset Trackers (VMT)\n[Space] Show Hints\n[D1] Calibrate Camera Positions\n[D2] Manual Offset Adjust: {Show(adjustOffset)}\n[D4] Calibrate Cameras (Distortion): {Show(CameraCalibrate.onCalibration)}" +
                $"\n[J] Reload Offsets\n[L] Reloaded Config/Trackers\n[D5] Auto Adjust Offsets\n[D7] Send Debug Trackers: {Show(debugSendTrackerOSC)}\n[D9] Show Camera Windows\n[D0] Clear Console" +
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
            } else if (key == ConsoleKey.D4) {
                if (CameraCalibrate.onCalibration) {
                    if (CameraCalibrate.startCalibrating) {
                        if (CameraCalibrate.framesSaved == 0)
                            CameraCalibrate.Cancel();
                        else
                            CameraCalibrate.StartCalibration();
                    }
                } else {
                    Console.WriteLine($"Calibrate Camera");
                    CameraCalibrate.Init();
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
                //if (CameraCalibrate.startCalibrating) {
                //    CameraCalibrate.removeBadFrames = !CameraCalibrate.removeBadFrames;
                //    Console.WriteLine($"Auto-remove frames: " + Show(CameraCalibrate.removeBadFrames));
                //} else 
                if (Tag.getRawTrackersStep > -1) {
                    Tag.timedSnapshot = !Tag.timedSnapshot;
                    Console.WriteLine($"Timed snap {(Tag.timedSnapshot ? "activated" : "deactivated")}");
                } else {
                    if (TrackerCalibrate.startCalibrating) {
                        TrackerCalibrate.End();
                    } else {
                        Console.WriteLine($"Calibrate Tracker");
                        TrackerCalibrate.Init();
                    }
                }
            } else if (key == ConsoleKey.J) {
                LoadOffsets();
                Console.WriteLine($"Reloaded Offsets");
            } else if (key == ConsoleKey.H) {
                performanceMode = !performanceMode;
                Console.WriteLine($"Performance Mode: " + Show(performanceMode));
            } else if (key == ConsoleKey.G) {
                needsToSearchHands = true;
                Console.WriteLine($"Searching hands");
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
                rotationY -= 0.02f;
                Console.WriteLine($"Decreased Yaw offset {rotationY}");
                ApplyOffset();
            } else if (key == ConsoleKey.R) {
                rotationY += 0.02f;
                Console.WriteLine($"Increased Yaw offset {rotationY}");
                ApplyOffset();
            } else if (key == ConsoleKey.D) {
                rotationX -= 0.02f;
                Console.WriteLine($"Decreased xRot offset {rotationX}");
                ApplyOffset();
            } else if (key == ConsoleKey.F) {
                rotationX += 0.02f;
                Console.WriteLine($"Increased xRot offset {rotationX}");
                ApplyOffset();
            } else if (key == ConsoleKey.C) {
                rotationZ -= 0.02f;
                Console.WriteLine($"Decreased zRot offset {rotationZ}");
                ApplyOffset();
            } else if (key == ConsoleKey.V) {
                rotationZ += 0.02f;
                Console.WriteLine($"Increased zRot offset {rotationZ}");
                ApplyOffset();
            } else if (key == ConsoleKey.D9) {
                wantToShowFrame = !wantToShowFrame;
                if (!wantToShowFrame) wantToCloseWindows = true;
            } else if (key == ConsoleKey.D0) {
                Console.Clear();
                Console.WriteLine();
                ShowHint();
            } else if (key == ConsoleKey.D6) {
                debugShowCamerasPosition = !debugShowCamerasPosition;
                Console.WriteLine($"Show Cameras as tracker: " + Show(debugShowCamerasPosition));
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
            } else if (key == ConsoleKey.L) {
                ReadConfig();
                Tag.ReadTrackers();
                Console.WriteLine($"Reloaded Config / Trackers");
                ApplyOffset();
                LoadOffsets();
            } else if (key == ConsoleKey.K) {
                var info = new System.Diagnostics.ProcessStartInfo(Environment.GetCommandLineArgs()[0]);
                System.Diagnostics.Process.Start("TagTracking.exe");
                //Environment.Exit(0);
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
                else if (split[0].Equals("interpolationTPS")) interpolationTPS = int.Parse(split[1]);
                else if (split[0].Equals("useInterpolation")) useInterpolation = split[1].Equals("true");
                else if (split[0].Equals("seekAhead")) Extrapolate.extrapolationRatio = float.Parse(split[1], any, invariantCulture);
                else if (split[0].Equals("useSmoothCorners")) Aruco.useSmoothCorners = split[1].Equals("true");
                else if (split[0].Equals("cornersMaxDistance")) Aruco.cornersMaxDistance = int.Parse(split[1]);
                else if (split[0].Equals("cornersSmoothFactor")) Aruco.cornersSmoothFactor = float.Parse(split[1], any, invariantCulture);
                else if (split[0].Equals("refineSearch")) Tag.refineSearch = split[1].Equals("true");
                else if (split[0].Equals("showThreadsTime")) showThreadsMS = split[1].Equals("true");
                else if (split[0].Equals("refineIterations")) Tag.refineIterations = int.Parse(split[1]);
                else if (split[0].Equals("dynamicFiltering")) Tag.dynamicFiltering = split[1].Equals("true");
                else if (split[0].Equals("performanceUnderSample")) performanceUnderSample = float.Parse(split[1], any, invariantCulture);
                else if (split[0].Equals("autoActivatePerformanceMode")) Program.autoActivatePerformanceMode = split[1].Equals("true");
                else if (split[0].Equals("perTrackerSize")) {
                    Aruco.perMarkerLength.Clear();
                    string[] split2 = split[1].Split(",");
                    for (int j = 0; j < split2.Length; j++) {
                        string[] split3 = split2[j].Split(" ");
                        if (split3.Length != 2) continue;
                        int.TryParse(split3[0], out int id);
                        int.TryParse(split3[1], out int length);
                        if (length == 0) continue;
                        Aruco.perMarkerLength.Add((id, length));
                    }
                } else if (split[0].Equals("tagsToCalibrate")) {
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
                        } else if (split[0].Equals($"camera{j}AdjustCurrentDistortion")) {
                            Tag.cameras[j].adjustCurrentDistortion = split[1].Equals("true");
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
