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
        static int coreCount = 1;

        public static bool poseAdjust = true;
        public static string poseAdjustWaist = "waist";
        public static string poseAdjustChest = "chest";
        public static string poseAdjustLeftFoot = "leftfoot";
        public static string poseAdjustRightFoot = "rightfoot";
        public static string poseAdjustLeftElbow = "leftarm";
        public static string poseAdjustRightElbow = "rightarm";
        public static string poseAdjustLeftKnee = "leftknee";
        public static string poseAdjustRightKnee = "rightknee";
        public static float poseAdjustLegDist = 0.5f;

        public static Tag.PreNoise preNoise = Tag.PreNoise.DisabledSmooth;
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
        public static List<Vector4> hmdList = new List<Vector4>();
        public static List<Vector4> leftList = new List<Vector4>();
        public static List<Vector4> rightList = new List<Vector4>();
        public static List<Vector4> shoulderList = new List<Vector4>();
        public static float trackerDelay = 300f;
        public static bool wantToShowFrame = false;
        public static bool adjustOffset = false;
        public static bool useTag0AsRotationPoint = false;
        public static long adjustOffsetTime = 0;
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
        public static bool sendHeadPositionVRC = true;
        public static bool sendHeadRotationVRC = false;
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
            coreCount = Environment.ProcessorCount / 2; //hehe

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
            StartOscClient();
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

        private static void StartOscClient() {
            if (oscClient != null) {
                if (oscClient.isRunning)
                    oscClient.StopClient();
            } else {
                oscClient = new uOscClient();
            }
            oscClient.port = oscPortOut;
            oscClient.address = oscAddress;
            oscClient.StartClient();
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
                    var m2 = devPos[rightHandID].mDeviceToAbsoluteTracking;
                    Matrix4x4 conMat = new Matrix4x4(m2.m0, m2.m4, m2.m8, 0, m2.m1, m2.m5, m2.m9, 0, m2.m2, m2.m6, m2.m10, 0, m2.m3, m2.m7, m2.m11, 1);
                    Matrix4x4 prevMat = new Matrix4x4(prevCont.m0, prevCont.m4, prevCont.m8, 0, prevCont.m1, prevCont.m5, prevCont.m9, 0, prevCont.m2, prevCont.m6, prevCont.m10, 0, prevCont.m3, prevCont.m7, prevCont.m11, 1);
                    //Console.WriteLine($"{m2.m0:0.00},{m2.m1:0.00},{m2.m2:0.00},{m2.m3:0.00},{m2.m4:0.00},{m2.m5:0.00},{m2.m6:0.00},{m2.m7:0.00},{m2.m8:0.00},{m2.m9:0.00},{m2.m10:0.00},{m2.m11:0.00},");
                    //d = Matrix4x4.Multiply(Matrix4x4.CreateFromYawPitchRoll(rotationZ, rotationX, rotationY), d);
                    if (moveTrigger) {
                        ApplyOffset(Matrix4x4.CreateTranslation(prevMat.Translation), Matrix4x4.CreateTranslation(conMat.Translation));
                    }
                    if (rotateTrigger) {
                        if (useTag0AsRotationPoint) {
                            int nearId = 0;
                            bool foundNearMat = false;
                            for (int i = 0; i < Tag.trackers.Length; i++) {
                                if (Tag.trackers[i].trackerNotSeen) continue;
                                foundNearMat = true;
                                nearId = i;
                                break;
                            }
                            if (foundNearMat) {
                                conMat.M41 = Tag.finals[nearId].pos.X;
                                conMat.M42 = Tag.finals[nearId].pos.Y;
                                conMat.M43 = Tag.finals[nearId].pos.Z;
                            }
                        }
                        prevMat.M41 = conMat.M41;
                        prevMat.M42 = conMat.M42;
                        prevMat.M43 = conMat.M43;
                        ApplyOffset(conMat, prevMat, !useTag0AsRotationPoint);
                    }
                    prevCont = m2;
                }
            }
            Matrix4x4 hmdRotMat, hmdCentered;
            Vector3 hmdPosV3;
            GetDevices(out hmdRotMat, out hmdCentered, out hmdPosV3);

            float time = timer.ElapsedMilliseconds / 1000000f;
            float timeDiff = time - trackerDelay / 1000000f;
            hmdList.Add(new Vector4(hmdPosV3, time));
            while (hmdList.Count > 1 && hmdList[0].W < timeDiff)
                hmdList.RemoveAt(0);
            leftList.Add(new Vector4(leftHandPos, time));
            while (leftList.Count > 1 && leftList[0].W < timeDiff)
                leftList.RemoveAt(0);
            rightList.Add(new Vector4(rightHandPos, time));
            while (rightList.Count > 1 && rightList[0].W < timeDiff)
                rightList.RemoveAt(0);
            shoulderList.Add(new Vector4(shoulderCenterPos, time));
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
                oscClientDebug.Send("/debug/final/position", 18,
                                   hmdPosV3LH.X, hmdPosV3LH.Y, -hmdPosV3LH.Z, //1f, 1.7f, 1f
                                   0f, 1f, 1f, 0f, "L_Hand");
                //righthand
                var mrh = devPos[rightHandID].mDeviceToAbsoluteTracking;
                Matrix4x4 hmdRotMatRH = new Matrix4x4(mrh.m0, mrh.m4, mrh.m8, 0, mrh.m1, mrh.m5, mrh.m9, 0, mrh.m2, mrh.m6, mrh.m10, 0, mrh.m3, mrh.m7, mrh.m11, 1);
                Vector3 hmdPosV3RH = hmdRotMatRH.Translation;
                oscClientDebug.Send("/debug/final/position", 17,
                                   hmdPosV3RH.X, hmdPosV3RH.Y, -hmdPosV3RH.Z, //1f, 1.7f, 1f
                                   0f, 1f, 1f, 0f, "R_Hand");

                oscClientDebug.Send("/debug/final/position", 16,
                                   hmdPosV3.X, hmdPosV3.Y, -hmdPosV3.Z, //1f, 1.7f, 1f
                                   0f, 1f, 1f, 0f, "Head");

                Matrix4x4 mat = hmdRotMat;
                //mat = Matrix4x4.Multiply(Matrix4x4.CreateTranslation(new Vector3(0f, 0f, 0.09f)), mat);
                ////mat.M42 = hmdRotMat.M42;
                Vector3 matT = mat.Translation;
                oscClientDebug.Send("/debug/final/position", 15,
                                           matT.X, matT.Y, -matT.Z, //1f, 1.7f, 1f
                                           0f, 1f, 1f, 0f, "HMD");
                int waist = -1;
                for (int i = 0; i < Tag.finals.Length; i++) {
                    if (Tag.finals[i].name.Equals(poseAdjustWaist))
                        waist = i;
                }
                Vector3 pos = Tag.finals[waist].fpos;

                mat = Matrix4x4.Multiply(Matrix4x4.CreateTranslation(new Vector3(0f, -0.09f, 0f)), hmdCentered);
                oscClientDebug.Send("/debug/final/position", 14,
                                           mat.Translation.X, mat.Translation.Y, -mat.Translation.Z, //1f, 1.7f, 1f
                                           0f, 1f, 1f, 0f, "Neck");

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
            bool ActiveCams = false;
            for (int i = 0; i < Tag.cameras.Length; i++) {
                if (!Tag.cameras[i].inWaitMode) {
                    ActiveCams = true;
                    break;
                }
            }
            int sleepTime = 1000 / updateFPS;
            if (!ActiveCams) sleepTime *= 4;
            System.Threading.Thread.Sleep(sleepTime);
            mainThreadBenchmark.Stop();
            threadsIdleTime[0] = mainThreadBenchmark.Elapsed.TotalMilliseconds;

            var mm = devPos[3].mDeviceToAbsoluteTracking;
            frameCount++;

            return previousTime;
        }
        public static void GetDevices() {
            GetDevices(out Matrix4x4 a, out Matrix4x4 b, out Vector3 c);
        }
        public static void GetDevices(out Matrix4x4 hmdRotMat, out Matrix4x4 hmdCentered, out Vector3 hmdPosV3) {
            var m = devPos[0].mDeviceToAbsoluteTracking;
            //Save only the HMD
            hmdRotMat = new Matrix4x4(m.m0, m.m4, m.m8, 0, m.m1, m.m5, m.m9, 0, m.m2, m.m6, m.m10, 0, m.m3, m.m7, m.m11, 1);
            if (useVRChatOSCTrackers || true) {
                Vector3 headInOffset = new Vector3(0, 0, 0.1f);
                hmdRotMat = Matrix4x4.Multiply(hmdRotMat, Matrix4x4.CreateTranslation(headInOffset));
            }
            hmdCentered = hmdRotMat;
            hmdCentered = Matrix4x4.Multiply(Matrix4x4.CreateTranslation(new Vector3(0f, 0f, 0.09f)), hmdCentered);
            hmdPosV3 = hmdCentered.Translation;
            hmdPos[0] = hmdPosV3.X;
            hmdPos[1] = hmdPosV3.Y;
            hmdPos[2] = hmdPosV3.Z;
            Vector3 hmdRotEuler = Utils.ToEulerAngles(Quaternion.CreateFromRotationMatrix(hmdCentered));
            hmdRot[0] = hmdRotEuler.X;
            hmdRot[1] = hmdRotEuler.Y;
            hmdRot[2] = hmdRotEuler.Z;

            m = devPos[leftHandID].mDeviceToAbsoluteTracking;
            Matrix4x4 leftHandRotMat = new Matrix4x4(m.m0, m.m4, m.m8, 0, m.m1, m.m5, m.m9, 0, m.m2, m.m6, m.m10, 0, m.m3, m.m7, m.m11, 1);
            leftHandPos = leftHandRotMat.Translation;
            m = devPos[rightHandID].mDeviceToAbsoluteTracking;
            Matrix4x4 rightHandRotMat = new Matrix4x4(m.m0, m.m4, m.m8, 0, m.m1, m.m5, m.m9, 0, m.m2, m.m6, m.m10, 0, m.m3, m.m7, m.m11, 1);
            rightHandPos = rightHandRotMat.Translation;
            Matrix4x4 shoulderMat = Matrix4x4.Multiply(Matrix4x4.CreateTranslation(new Vector3(0f, -0.09f, 0f)), hmdCentered);
            shoulderCenterPos = shoulderMat.Translation;
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
                if (Tag.cameras[i].inWaitMode)
                    Console.Write($"Cam {i} TPS: Idle ");
                else
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
                double overallUsage = 0;
                for (int i = 0; i < threadsWorkTime.Length; i++) {
                    Console.SetCursorPosition(0, ypos + 1 + i);
                    if (i == 0) Console.Write("main");
                    else if (i == 1) Console.Write("interpolate");
                    else Console.Write($"camera {i - 2}");
                    double percent = ((threadsWorkTime[i] / threadsIdleTime[i]) * 100);
                    overallUsage += percent / coreCount;
                    Console.Write($": work {threadsWorkTime[i]:0.0000}ms / idle {threadsIdleTime[i]:0.0000}ms / {percent:0.0}% core usage / {(percent / coreCount):0.0}% CPU usage");
                    for (int j = Console.CursorLeft; j < Console.WindowWidth; j++) {
                        Console.Write(" ");
                    }
                }
                Console.SetCursorPosition(0, ypos + 1 + threadsWorkTime.Length);
                Console.Write($"Total CPU Usage: {overallUsage:0.00}%");
                for (int j = Console.CursorLeft; j < Console.WindowWidth; j++) {
                    Console.Write(" ");
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
            //rotationY = newY;
            //rotationX = newX;
            //rotationZ = newZ;
            //Matrix4x4.Invert(diff, out offsetMat);
        }

        static void ShowHint() {
            if (showThreadsMS)
                for (int i = 0; i < threadsWorkTime.Length; i++) {
                    Console.WriteLine();
                }
            Console.WriteLine($"\n[Space] Show Hints\n[D1] Calibrate Camera Room Positions\n[D2] Manual Offset Adjust: {Show(adjustOffset)}\n[D3] Calibrate Trackers: {Show(TrackerCalibrate.onCalibration)}\n[D4] Calibrate Cameras (Distortion): {Show(CameraCalibrate.onCalibration)}\n[D5] Auto Adjust Offsets" +
                $"\n[J] Reload Offsets\n[L] Reloaded Config/Trackers\n[D6] Show Cameras as Tracker 0: {Show(debugShowCamerasPosition)}\n[D7] Send Debug Trackers: {Show(debugSendTrackerOSC)}\n[D8] Reset Trackers (VMT)\n[D9] Show Camera Windows\n[D0] Clear Console" +
                $"\n[M] Pre-Pose Noise Reduction: {(preNoise == Tag.PreNoise.Disabled ? "Disabled" : preNoise == Tag.PreNoise.DisabledSmooth ? "Disabled + Smooth rects" : preNoise == Tag.PreNoise.EnabledSmooth ? "Enabled + Smooth rects" : "Enabled")}\n[N] Post-Pose Noise Reduction: {(postNoise == 0 ? "Disabled" : postNoise == 1 ? "Enabled" : "Partial")}\n[B] Tracker Center Guessing: {(clusterRotationGuess == 0 ? "Disabled" : clusterRotationGuess == 1 ? "Enabled" : "CPU Heavy")}" +
                $"\n[H] Enable Performance Mode: {Show(performanceMode)}\n[G] Use Dynamic Framing: {Show(Aruco.useDynamicFraming)}\n[U] Search for Hands" + 
                $"\n[Q]-[W] X: {offset.X}\n[A]-[S] Y: {offset.Y}\n[Z]-[X] Z: {offset.Z}\n[E]-[R] Yaw\n[D]-[F] Pitch\n[C]-[V] Roll");
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
                if (RoomCalibrate.getRawTrackersStep > -1) {
                    RoomCalibrate.Calibrate();
                } else if (RoomCalibrate.getRawTrackersStep == -2) {
                    RoomCalibrate.Stop();
                } else {
                    RoomCalibrate.Init();
                }
            } else if (key == ConsoleKey.D4) {
                if (RoomCalibrate.getRawTrackersStep > -1 && false) {
                    RoomCalibrate.AnchorDialog();
                } else {
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
                }
            } else if (key == ConsoleKey.D2) {
                if (RoomCalibrate.getRawTrackersStep > -1) {
                    RoomCalibrate.addNewRaw = true;
                } else {
                    cannotGetControllers = false;
                    adjustOffset = !adjustOffset;
                    isMoveKeyPressed = false;
                    isRotateKeyPressed = false;
                    Console.WriteLine($"Adjust offset by hand: " + Show(adjustOffset));
                    if (adjustOffset) {
                        adjustOffsetTime = timer.ElapsedMilliseconds;
                        useTag0AsRotationPoint = false;
                        Console.WriteLine("Move offset with Grip, rotate with Trigger, Press [I] to use Tracker 0 as rotation point. Or press [P] to move and [O] to rotate instead");
                    }
                }
            } else if (key == ConsoleKey.D3) {
                //if (CameraCalibrate.startCalibrating) {
                //    CameraCalibrate.removeBadFrames = !CameraCalibrate.removeBadFrames;
                //    Console.WriteLine($"Auto-remove frames: " + Show(CameraCalibrate.removeBadFrames));
                //} else 
                if (RoomCalibrate.getRawTrackersStep > -1) {
                    RoomCalibrate.timedSnapshot = !RoomCalibrate.timedSnapshot;
                    Console.WriteLine($"Timed snap {(RoomCalibrate.timedSnapshot ? "activated" : "deactivated")}");
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
                Aruco.useDynamicFraming = !Aruco.useDynamicFraming;
                Console.WriteLine($"Dynamic Framing: " + Show(Aruco.useDynamicFraming));
            } else if (key == ConsoleKey.U) {
                needsToSearchHands = true;
                Console.WriteLine($"Searching hands");
            } else if (key == ConsoleKey.Q) {
                ApplyOffset(moveX: -0.01f);
                Console.WriteLine($"Decreased X offset {offsetMat.M41}");
            } else if (key == ConsoleKey.W) {
                ApplyOffset(moveX: 0.01f);
                Console.WriteLine($"Increased X offset {offsetMat.M41}");
            } else if (key == ConsoleKey.A) {
                ApplyOffset(moveY: -0.01f);
                Console.WriteLine($"Decreased Y offset {offsetMat.M43}");
            } else if (key == ConsoleKey.S) {
                ApplyOffset(moveY: 0.01f);
                Console.WriteLine($"Increased Y offset {offsetMat.M43}");
            } else if (key == ConsoleKey.Z) {
                ApplyOffset(moveZ: -0.01f);
                Console.WriteLine($"Decreased Z offset {offsetMat.M42}");
            } else if (key == ConsoleKey.X) {
                ApplyOffset(moveZ: 0.01f);
                Console.WriteLine($"Increased Z offset {offsetMat.M42}");
            } else if (key == ConsoleKey.E) {
                Console.WriteLine($"Decreased Yaw offset");
                ApplyOffset(moveYaw: -0.02f);
            } else if (key == ConsoleKey.R) {
                Console.WriteLine($"Increased Yaw offset");
                ApplyOffset(moveYaw: 0.02f);
            } else if (key == ConsoleKey.D) {
                Console.WriteLine($"Decreased Pitch offset");
                ApplyOffset(movePitch: -0.02f);
            } else if (key == ConsoleKey.F) {
                Console.WriteLine($"Increased Pitch offset");
                ApplyOffset(movePitch: 0.02f);
            } else if (key == ConsoleKey.C) {
                Console.WriteLine($"Decreased Roll offset");
                ApplyOffset(moveRoll: -0.02f);
            } else if (key == ConsoleKey.V) {
                Console.WriteLine($"Increased Roll offset");
                ApplyOffset(moveRoll: 0.02f);
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
            } else if (key == ConsoleKey.I) {
                if (adjustOffset) {
                    useTag0AsRotationPoint = !useTag0AsRotationPoint;
                    Console.WriteLine($"Use Tag 0 as rotation point {Show(useTag0AsRotationPoint)}");
                }
            } else if (key == ConsoleKey.M) {
                preNoise++;
                if ((int)preNoise > 3) preNoise = 0;
                Console.WriteLine($"Toggle pre-pose noise reduction {(preNoise == Tag.PreNoise.Disabled ? "Disabled" : preNoise == Tag.PreNoise.DisabledSmooth ? "Disabled + Smooth rects" : preNoise == Tag.PreNoise.EnabledSmooth ? "Enabled + Smooth rects" : "Enabled")}");
                if (preNoise == Tag.PreNoise.Enabled || preNoise == Tag.PreNoise.EnabledSmooth)
                    Console.WriteLine("\tHaving it enabled can be buggy and cause flickering.\n\tIt is recommended to use 'Disabled + Smooth rects'");
            } else if (key == ConsoleKey.N) {
                postNoise++;
                if (postNoise > 2) postNoise = 0;
                Tag.SetFinalTrackers(postNoise == 2 ? 0.5f : 1f);
                Console.WriteLine($"Toggle post-pose noise reduction {(postNoise == 0 ? "Disabled" : postNoise == 1 ? "Enabled" : "Partial")}");
            } else if (key == ConsoleKey.B) {
                clusterRotationGuess++;
                if (clusterRotationGuess > 2) clusterRotationGuess = 0;
                Console.WriteLine($"Guess tracker rotation: {(clusterRotationGuess == 0 ? "Disabled" : clusterRotationGuess == 1 ? "Enabled" : "CPU Heavy")}");
            }
            offsetMat.M41 = offsetMat.M41 + roomOffset.X;
            offsetMat.M42 = offsetMat.M42 + roomOffset.Y;
            offsetMat.M43 = offsetMat.M43 + roomOffset.Z;
            Console.WriteLine();
            using (StreamWriter sw = new StreamWriter("offsets")) {
                sw.WriteLine(offsetMat.M11);
                sw.WriteLine(offsetMat.M12);
                sw.WriteLine(offsetMat.M13);
                sw.WriteLine(offsetMat.M14);
                sw.WriteLine(offsetMat.M21);
                sw.WriteLine(offsetMat.M22);
                sw.WriteLine(offsetMat.M23);
                sw.WriteLine(offsetMat.M24);
                sw.WriteLine(offsetMat.M31);
                sw.WriteLine(offsetMat.M32);
                sw.WriteLine(offsetMat.M33);
                sw.WriteLine(offsetMat.M34);
                sw.WriteLine(offsetMat.M41);
                sw.WriteLine(offsetMat.M42);
                sw.WriteLine(offsetMat.M43);
                sw.WriteLine(offsetMat.M44);
            }
        }

        private static string Show(bool b) {
            return b ? "Enabled" : "Disabled";
        }
        public static void ApplyOffset(Matrix4x4 moveOrigin, Matrix4x4 movetarget, bool adjustSteamVRMatrix = true) {
            if (adjustSteamVRMatrix) {
                float tmp = moveOrigin.M42;
                moveOrigin.M42 = -moveOrigin.M43;
                moveOrigin.M43 = tmp;
                tmp = movetarget.M42;
                movetarget.M42 = -movetarget.M43;
                movetarget.M43 = tmp;
            }

            Quaternion q1 = Quaternion.CreateFromRotationMatrix(moveOrigin);
            Quaternion q2 = Quaternion.CreateFromRotationMatrix(movetarget);
            q1 = new Quaternion(-q1.X, q1.Z, -q1.Y, -q1.W);
            q2 = new Quaternion(-q2.X, q2.Z, -q2.Y, -q2.W);
            Matrix4x4 rot1 = Matrix4x4.CreateFromQuaternion(q1);
            Matrix4x4 rot2 = Matrix4x4.CreateFromQuaternion(q2);

            Matrix4x4 matReference;
            if (adjustSteamVRMatrix) {
                Matrix4x4 newOffsetMat = offsetMat;
                Matrix4x4 invOffset = Utils.MatrixInvert(newOffsetMat);
                matReference = Matrix4x4.Multiply(movetarget, invOffset);
            } else {
                matReference = movetarget;
            }

            Matrix4x4 newMat = offsetMat;
            Matrix4x4 nearMat = Matrix4x4.Multiply(matReference, newMat);
            Matrix4x4.Invert(rot2, out Matrix4x4 invRot);
            Matrix4x4 rotMat = Matrix4x4.Multiply(rot1, invRot);
            newMat = Matrix4x4.Multiply(newMat, invRot);
            newMat = Matrix4x4.Multiply(newMat, rotMat);
            newMat = Matrix4x4.Multiply(newMat, rot2);
            Matrix4x4 nearMatNew = Matrix4x4.Multiply(matReference, newMat);

            Vector3 offPos = nearMatNew.Translation - nearMat.Translation;
            newMat.M41 -= offPos.X;
            newMat.M42 -= offPos.Y;
            newMat.M43 -= offPos.Z;

            Vector3 movePos = moveOrigin.Translation - movetarget.Translation;
            newMat.M41 -= movePos.X;
            newMat.M42 -= movePos.Y;
            newMat.M43 -= movePos.Z;

            offsetMat = newMat;
        }
        public static void ApplyOffset(float moveX = 0f, float moveY = 0f, float moveZ = 0f, float moveYaw = 0f, float movePitch = 0f, float moveRoll = 0f) {
            Matrix4x4 newMat = offsetMat;
            Matrix4x4 matReference = Matrix4x4.Identity;
            int nearId = 0;
            bool foundNearMat = false;
            for (int i = 0; i < Tag.trackers.Length; i++) {
                if (Tag.trackers[i].trackerNotSeen) continue;
                foundNearMat = true;
                nearId = i;
                break;
            }
            if (foundNearMat)
                matReference = Matrix4x4.Multiply(Matrix4x4.CreateFromQuaternion(Tag.finals[nearId].frot), Matrix4x4.CreateTranslation(Tag.finals[nearId].fpos));

            Matrix4x4 nearMat = Matrix4x4.Multiply(matReference, newMat);
            if (movePitch != 0f) newMat = Matrix4x4.Multiply(newMat, Matrix4x4.CreateFromAxisAngle(new Vector3(1, 0, 0), movePitch));
            if (moveRoll != 0f) newMat = Matrix4x4.Multiply(newMat, Matrix4x4.CreateFromAxisAngle(new Vector3(0, 1, 0), moveRoll));
            if (moveYaw != 0f) newMat = Matrix4x4.Multiply(newMat, Matrix4x4.CreateFromAxisAngle(new Vector3(0, 0, 1), moveYaw));

            Matrix4x4 nearMatNew = Matrix4x4.Multiply(matReference, newMat);
            Vector3 offPos = nearMatNew.Translation - nearMat.Translation;
            newMat.M41 -= offPos.X;
            newMat.M42 -= offPos.Y;
            newMat.M43 -= offPos.Z;

            if (moveX != 0f) newMat.M41 += moveX;
            if (moveY != 0f) newMat.M42 += moveY;
            if (moveZ != 0f) newMat.M43 += moveZ;
            offsetMat = newMat;
        }

        static void LoadOffsets() {
            Tag.ReadMatrix();
            offsetMat = Matrix4x4.Identity;
            if (File.Exists("offsets")) {
                string[] lines = File.ReadAllLines("offsets");
                int l = 0;
                offsetMat.M11 = float.Parse(lines[l++]);
                offsetMat.M12 = float.Parse(lines[l++]);
                offsetMat.M13 = float.Parse(lines[l++]);
                offsetMat.M14 = float.Parse(lines[l++]);
                offsetMat.M21 = float.Parse(lines[l++]);
                offsetMat.M22 = float.Parse(lines[l++]);
                offsetMat.M23 = float.Parse(lines[l++]);
                offsetMat.M24 = float.Parse(lines[l++]);
                offsetMat.M31 = float.Parse(lines[l++]);
                offsetMat.M32 = float.Parse(lines[l++]);
                offsetMat.M33 = float.Parse(lines[l++]);
                offsetMat.M34 = float.Parse(lines[l++]);
                offsetMat.M41 = float.Parse(lines[l++]);
                offsetMat.M42 = float.Parse(lines[l++]);
                offsetMat.M43 = float.Parse(lines[l++]);
                offsetMat.M44 = float.Parse(lines[l++]);
            }
        }
        static void ReadConfig() {
            if (!File.Exists("config.txt")) return;
            string[] lines = File.ReadAllLines("config.txt");
            bool restartOSC = false;
            bool reloadCameraParameters = false;
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
                else if (split[0].Equals("preNoiseReduction")) preNoise = (Tag.PreNoise)int.Parse(split[1]);
                else if (split[0].Equals("postNoiseReduction")) postNoise = int.Parse(split[1]);
                else if (split[0].Equals("enableIgnoreNoisyRotation")) ignoreNoisyRotation = split[1].Equals("true");
                else if (split[0].Equals("oscAddress")) oscAddress = split[1];
                else if (split[0].Equals("oscPort")) {
                    oscPortOut = int.Parse(split[1]);
                    restartOSC = true;
                } else if (split[0].Equals("useVrchatOscTrackers")) useVRChatOSCTrackers = split[1].Equals("true");
                else if (split[0].Equals("sendHeadPositionVRC")) sendHeadPositionVRC = split[1].Equals("true");
                else if (split[0].Equals("sendHeadRotationVRC")) sendHeadRotationVRC = split[1].Equals("true");
                else if (split[0].Equals("roomOffsetX")) roomOffset.X = float.Parse(split[1], any, invariantCulture);
                else if (split[0].Equals("roomOffsetY")) roomOffset.Y = float.Parse(split[1], any, invariantCulture);
                else if (split[0].Equals("roomOffsetZ")) roomOffset.Z = float.Parse(split[1], any, invariantCulture);
                else if (split[0].Equals("trackerSize")) Aruco.markersLength = int.Parse(split[1]);
                else if (split[0].Equals("updatesPerSecond")) updateFPS = int.Parse(split[1]);
                else if (split[0].Equals("interpolationTPS")) interpolationTPS = int.Parse(split[1]);
                else if (split[0].Equals("useInterpolation")) useInterpolation = split[1].Equals("true");
                else if (split[0].Equals("seekAhead")) Extrapolate.extrapolationRatio = float.Parse(split[1], any, invariantCulture);
                else if (split[0].Equals("useSmoothCorners")) Aruco.useSmoothCorners = split[1].Equals("true");
                else if (split[0].Equals("cornersMaxDistance")) Aruco.cornersMaxDistance = float.Parse(split[1], any, invariantCulture);
                else if (split[0].Equals("cornersSmoothFactor")) Aruco.cornersSmoothFactor = float.Parse(split[1], any, invariantCulture);
                else if (split[0].Equals("refineSearch")) RoomCalibrate.refineSearch = split[1].Equals("true");
                else if (split[0].Equals("showThreadsTime")) showThreadsMS = split[1].Equals("true");
                else if (split[0].Equals("refineIterations")) RoomCalibrate.refineIterations = int.Parse(split[1]);
                else if (split[0].Equals("dynamicFiltering")) Tag.dynamicFiltering = split[1].Equals("true");
                else if (split[0].Equals("dynamicFraming")) Aruco.useDynamicFraming = split[1].Equals("true");
                else if (split[0].Equals("markerDictionary")) Aruco.markerDictionary = split[1].ToLower();
                else if (split[0].Equals("performanceUnderSample")) performanceUnderSample = float.Parse(split[1], any, invariantCulture);
                else if (split[0].Equals("autoActivatePerformanceMode")) autoActivatePerformanceMode = split[1].Equals("true");
                else if (split[0].Equals("showRejected")) Aruco.showRejected = split[1].Equals("true");
                else if (split[0].Equals("recoverRejected")) Aruco.recoverRejecteds = split[1].Equals("true");
                else if (split[0].Equals("rejectedDistanceTolerance")) Aruco.rejectedDistanceTolerance = float.Parse(split[1], any, invariantCulture);
                else if (split[0].Equals("refinementMethod")) {
                    Aruco.refinementMethod = int.Parse(split[1]);
                    Aruco.ApplyParams();
                } else if (split[0].Equals("refinementMinAccuracy")) {
                    Aruco.refinementMinAcc = float.Parse(split[1], any, invariantCulture);
                    Aruco.ApplyParams();
                } else if (split[0].Equals("refinementWindowSize")) {
                    Aruco.refinementWinSize = int.Parse(split[1]);
                    Aruco.ApplyParams();
                } else if (split[0].Equals("perTrackerSize")) {
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
                            if (!Tag.cameras[j].file.Equals(split[1]))
                                reloadCameraParameters = true;
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
                            Tag.cameras[j].url = "";
                            Tag.cameras[j].index = 0;
                            bool ok = int.TryParse(split[1], out int number);
                            if (!ok)
                                Tag.cameras[j].url = split[1];
                            else
                                Tag.cameras[j].index = number;
                        } else if (split[0].Equals($"camera{j}WorldResize")) {
                            Tag.cameras[j].depthMult = float.Parse(split[1], any, invariantCulture);
                        } else if (split[0].Equals($"camera{j}Brightness")) {
                            Tag.cameras[j].brightness = float.Parse(split[1], any, invariantCulture);
                        } else if (split[0].Equals($"camera{j}AdjustCurrentDistortion")) {
                            Tag.cameras[j].adjustCurrentDistortion = split[1].Equals("true");
                        } else if (split[0].Equals($"camera{j}FrameSkip")) {
                            Tag.cameras[j].skipFrames = int.Parse(split[1]);
                        } else if (split[0].Equals($"camera{j}IdleWaitTime")) {
                            Tag.cameras[j].timeBeforeWaiting = int.Parse(split[1]);
                        } else if (split[0].Equals($"camera{j}IdleUpdateTime")) {
                            Tag.cameras[j].waitTimeUpdate = int.Parse(split[1]);
                        } else if (split[0].Equals($"camera{j}CanEnterIdleMode")) {
                            Tag.cameras[j].canEnterInWaitMode = split[1].Equals("true");
                        }
                    }
                }
            }
            Tag.cameraTPS = new double[Tag.cameras.Length];
            if (restartOSC)
                StartOscClient();
            if (reloadCameraParameters)
                Aruco.GetCameraParameters();
        }
    }
}
