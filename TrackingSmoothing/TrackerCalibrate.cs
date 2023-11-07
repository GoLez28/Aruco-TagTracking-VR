using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;
using System.Numerics;
using System.Text;
using System.Threading;
using System.IO;
using System.Threading.Tasks;

namespace TagTracking {
    class TrackerCalibrate {
        public static bool onCalibration;
        public static bool startCalibrating = false;
        public static int cameraToUse = 0;
        static string name = "name";
        static int[] ids = new int[4];
        static int lead = 0;
        static int leadIndex = 0;
        static List<Matrix4x4>[] trackerList = new List<Matrix4x4>[4];
        static Matrix4x4[] trackerLastSaved = new Matrix4x4[4];
        static Matrix4x4[] trackerAvg = new Matrix4x4[4];
        static Vector3[] trackerPos = new Vector3[4];
        static Vector3[] trackerRot = new Vector3[4];
        static bool[] trackerSeen = new bool[4];
        static bool[] trackerOk = new bool[4];
        static int frameCount = 0;
        public static void Init() {
            onCalibration = true;
            startCalibrating = false;
            cameraToUse = 0;
            name = "name";
            ids = new int[4];
            lead = 0;
            trackerList = new List<Matrix4x4>[4];
            Program.oscClientDebug.Send("/debug/calibrate/clear", 0);
            Dialog();
        }
        static void Dialog() {
            if (Tag.cameras.Length > 1) {
                Console.WriteLine($"Choose a camera [0 to {Tag.cameras.Length - 1}] (ID is indicated in 'config.txt')");
                try {
                    cameraToUse = int.Parse(Console.ReadLine());
                } catch { cameraToUse = -1; }
                if (cameraToUse < 0 || cameraToUse >= Tag.cameras.Length) {
                    Console.WriteLine("Camera ID not available\nCancelling...");
                    onCalibration = false;
                    return;
                }
            }
            Console.WriteLine("Write tracker name:");
            name = Console.ReadLine();
            Console.WriteLine("Write tracker ids to use (separated by spaces):");
            string idsStr = Console.ReadLine();
            string[] split = idsStr.Split(" ");
            if (split.Length == 0 || split[0] == "") {
                Console.WriteLine("Ids provided not correct, you try something like:\n0 1 2 3\nCancelling");
                onCalibration = false;
                return;
            }
            ids = new int[split.Length];
            for (int i = 0; i < split.Length; i++) {
                ids[i] = int.Parse(split[i]);
            }
            trackerList = new List<Matrix4x4>[ids.Length];
            trackerLastSaved = new Matrix4x4[ids.Length];
            trackerPos = new Vector3[ids.Length];
            trackerRot = new Vector3[ids.Length];
            trackerSeen = new bool[ids.Length];
            trackerAvg = new Matrix4x4[ids.Length];
            trackerOk = new bool[ids.Length];
            for (int i = 0; i < trackerList.Length; i++) {
                trackerList[i] = new List<Matrix4x4>();
            }
            Console.WriteLine("Write tracker lead id to use:");
            string leadStr = Console.ReadLine();
            if (leadStr == "") {
                Console.WriteLine("Lead id provided not correct, you try something like:\nCancelling");
                onCalibration = false;
                return;
            }
            lead = int.Parse(leadStr);
            bool badId = true;
            for (int i = 0; i < ids.Length; i++) {
                if (ids[i] == lead) {
                    badId = false;
                    leadIndex = i;
                    break;
                }
            }
            if (badId) {
                Console.WriteLine("Lead id provided not correct, you try something like:\nCancelling");
                onCalibration = false;
                return;
            }
            Console.WriteLine("Starting Calibration... (press [3] to end calibration)");
            startCalibrating = true;
        }
        public static void GetTracker(int id, Vector3 pos, Matrix4x4 rot) {
            bool badId = true;
            int index = 0;
            for (int i = 0; i < ids.Length; i++) {
                if (ids[i] == id) {
                    badId = false;
                    index = i;
                    break;
                }
            }
            if (badId) return;

            Matrix4x4 mat = Matrix4x4.Multiply(rot, Matrix4x4.CreateTranslation(pos));
            trackerLastSaved[index] = mat;
            trackerSeen[index] = true;
        }
        internal static void GetTrackerEnd() {
            bool leadPresent = trackerSeen[leadIndex];
            for (int index = 0; index < ids.Length; index++) {
                if (!trackerSeen[index]) continue;
                if (trackerOk[index]) continue;
                if (index == leadIndex) {
                    trackerAvg[index] = Matrix4x4.Identity;
                    SendQuadsDebug(index);
                    trackerOk[index] = true;
                    continue;
                }
                int id = ids[index];
                Matrix4x4 mat = trackerLastSaved[index];

                int refIndex = leadIndex;
                if (!trackerSeen[leadIndex]) {
                    Random rnd = new Random();
                    int startAt = rnd.Next(0, ids.Length);
                    int step = startAt;
                    bool found = false;
                    do {
                        if (step != index)
                            if (trackerSeen[step] && trackerOk[step]) {
                                refIndex = step;
                                found = true;
                                break;
                            }
                        step++;
                        if (step == ids.Length) step = 0;
                    } while (step != startAt);
                    if (!found) continue;
                }
                Matrix4x4 refMat = trackerLastSaved[refIndex];
                if (refMat.Translation.X == 0) return;
                Vector3 refPos = refMat.Translation;
                Quaternion refRot = Quaternion.CreateFromRotationMatrix(refMat);
                Quaternion invRot = Quaternion.Inverse(refRot);
                Vector3 posSub = mat.Translation - refPos;
                Matrix4x4 sub = mat;
                sub.M41 = posSub.X;
                sub.M42 = posSub.Y;
                sub.M43 = posSub.Z;
                sub = Matrix4x4.Multiply(sub, Matrix4x4.CreateFromQuaternion(invRot));

                if (refIndex != leadIndex) {
                    sub = Matrix4x4.Multiply(sub, trackerAvg[refIndex]);
                }

                trackerList[index].Add(sub);
                while (trackerList[index].Count > 100) {
                    trackerOk[index] = true;
                    trackerList[index].RemoveAt(0);
                    Console.WriteLine($"tag {ids[index]} OK!");
                }
                int count = trackerList[index].Count;
                Quaternion rotSum = new Quaternion(0, 0, 0, 0);
                Vector3 posSum = new Vector3(0, 0, 0);
                for (int i = 0; i < trackerList[index].Count; i++) {
                    float add = 1f / count;
                    Quaternion q = Quaternion.CreateFromRotationMatrix(trackerList[index][i]);
                    Vector3 p = trackerList[index][i].Translation;

                    rotSum.X += add * q.X;
                    rotSum.Y += add * q.Y;
                    rotSum.Z += add * q.Z;
                    rotSum.W += add * q.W;

                    posSum.X += add * p.X;
                    posSum.Y += add * p.Y;
                    posSum.Z += add * p.Z;
                }
                rotSum = Quaternion.Normalize(rotSum);
                trackerAvg[index] = Matrix4x4.Multiply(Matrix4x4.CreateFromQuaternion(rotSum), Matrix4x4.CreateTranslation(posSum));
                SendQuadsDebug(index);
            }
            //Debug
            ShowFinalPointsDebug();
            for (int index = 0; index < ids.Length; index++) {
                trackerSeen[index] = false;
            }
        }
        static void SendQuadsDebug(int index) {
            if (trackerOk[index]) return;
            if (trackerList[index].Count % 10 != 0) return;
            if (!Program.debugSendTrackerOSC) {
                return;
            }
            Matrix4x4 point1 = Matrix4x4.Multiply(Matrix4x4.CreateTranslation(new Vector3(Aruco.markersLength / 2000f, Aruco.markersLength / 2000f, 0)), trackerAvg[index]);
            Matrix4x4 point2 = Matrix4x4.Multiply(Matrix4x4.CreateTranslation(new Vector3(-Aruco.markersLength / 2000f, Aruco.markersLength / 2000f, 0)), trackerAvg[index]);
            Matrix4x4 point3 = Matrix4x4.Multiply(Matrix4x4.CreateTranslation(new Vector3(Aruco.markersLength / 2000f, -Aruco.markersLength / 2000f, 0)), trackerAvg[index]);
            Matrix4x4 point4 = Matrix4x4.Multiply(Matrix4x4.CreateTranslation(new Vector3(-Aruco.markersLength / 2000f, -Aruco.markersLength / 2000f, 0)), trackerAvg[index]);
            Program.oscClientDebug.Send("/debug/calibrate/position", 0, 0, point1.Translation.X * 5, point1.Translation.Z * 5, point1.Translation.Y * 5);
            Program.oscClientDebug.Send("/debug/calibrate/position", 0, 0, point2.Translation.X * 5, point2.Translation.Z * 5, point2.Translation.Y * 5);
            Program.oscClientDebug.Send("/debug/calibrate/position", 0, 0, point3.Translation.X * 5, point3.Translation.Z * 5, point3.Translation.Y * 5);
            Program.oscClientDebug.Send("/debug/calibrate/position", 0, 0, point4.Translation.X * 5, point4.Translation.Z * 5, point4.Translation.Y * 5);
        }

        private static void ShowFinalPointsDebug() {
            frameCount++;
            if (!Program.debugSendTrackerOSC) {
                return;
            }
            for (int index = 0; index < ids.Length; index++) {
                if (index > 10) break;
                //Quaternion debugRot = Quaternion.CreateFromRotationMatrix(sub);
                Matrix4x4 final = trackerAvg[index];
                final.M41 *= 5;
                final.M42 *= 5;
                final.M43 *= 5;
                Program.oscClientDebug.Send("/debug/calibrate/position", index, 1,
                                           final.Translation.X, final.Translation.Z, final.Translation.Y);
                Matrix4x4 point = Matrix4x4.Multiply(Matrix4x4.CreateTranslation(new Vector3(0, 0, 0.25f)), final);
                Vector3 pointPos = point.Translation;
                Program.oscClientDebug.Send("/debug/calibrate/position", index + 10, 2,
                                           pointPos.X, pointPos.Z, pointPos.Y);
            }
        }

        public static void End() {
            onCalibration = false;
            startCalibrating = false;
            Console.WriteLine("Ended Reading, calibrate and save it? (Y), or do it again with same tags? (N)");
            string ans = Console.ReadLine();
            if (ans.Equals("") || ans.ToLower().Equals("y") || ans.ToLower().Equals("yes") || ans.ToLower().Equals("yeah") || ans.ToLower().Equals("si") || ans.ToLower().Equals("s")) { //lol
                Thread calibThread = new Thread(() => NewMethod());
                calibThread.Start();
            } else {
                try {
                    bool doItAll = false;
                    int[] segId = new int[ids.Length];
                    if (ans.ToLower().Equals("no") || ans.ToLower().Equals("n") || ans.ToLower().Equals("nah"))
                        doItAll = true;
                    if (!doItAll) {
                        string[] split = ans.Split(" ");
                        if (split.Length == 0 || split[0] == "") {
                            doItAll = true;
                        }
                        segId = new int[split.Length];
                        for (int i = 0; i < split.Length; i++) {
                            segId[i] = int.Parse(split[i]);
                        }
                    }
                    Program.oscClientDebug.Send("/debug/calibrate/clear", 0);
                    onCalibration = true;
                    startCalibrating = true;
                    if (doItAll) ids.CopyTo(segId, 0);
                    for (int i = 0; i < segId.Length; i++) {
                        for (int j = 0; j < ids.Length; j++) {
                            if (segId[i] == ids[j]) { segId[i] = j; break; }
                        }
                    }
                    for (int i = 0; i < segId.Length; i++) {
                        trackerList[segId[i]] = new List<Matrix4x4>();
                        trackerLastSaved[segId[i]] = new();
                        trackerPos[segId[i]] = new();
                        trackerRot[segId[i]] = new();
                        trackerSeen[segId[i]] = false;
                        trackerAvg[segId[i]] = new();
                        trackerOk[segId[i]] = false;
                    }
                } catch {
                    Console.WriteLine("Bad answer, too bad!");
                    Thread calibThread = new Thread(() => NewMethod());
                    calibThread.Start();
                }
            }
        }

        private static void NewMethod() {
            Console.WriteLine("Calibrating...");

            //timer to preview in debug view
            Stopwatch previewTimer = new Stopwatch();
            previewTimer.Start();

            //variables to save later
            float[] rotX = new float[trackerAvg.Length];
            float[] rotY = new float[trackerAvg.Length];
            float[] rotZ = new float[trackerAvg.Length];
            Vector3[] pos = new Vector3[trackerAvg.Length];

            //in debug mode this wil be run for 1 seconds, in normal mode it will break at the end instantly
            while (true) {
                //for debug purpose
                ShowFinalPointsDebug();
                Vector3 centerPos = new();
                for (int i = 0; i < trackerAvg.Length; i++) {
                    centerPos += trackerAvg[i].Translation;
                }
                centerPos /= trackerAvg.Length;
                for (int index = 0; index < ids.Length; index++) {
                    Matrix4x4 osCenter = Matrix4x4.CreateTranslation(centerPos - trackerAvg[index].Translation);
                    Matrix4x4 matRot = trackerAvg[index];
                    matRot.M41 = 0;
                    matRot.M42 = 0;
                    matRot.M43 = 0;
                    Matrix4x4 matRotInv;
                    Matrix4x4.Invert(matRot, out matRotInv);
                    Matrix4x4 resCenter = Matrix4x4.Multiply(osCenter, matRotInv);
                    pos[index] = resCenter.Translation;
                    Matrix4x4 finalCenter = Matrix4x4.Multiply(Matrix4x4.CreateTranslation(resCenter.Translation), trackerAvg[index]);
                    Vector3 finalCenterPos = finalCenter.Translation;

                    //Fucking thanks chatgpt, rotation are hard as fuck
                    Quaternion orgQuat = Quaternion.CreateFromRotationMatrix(trackerAvg[index]);
                    Matrix4x4 orgRot = Matrix4x4.CreateFromQuaternion(orgQuat);

                    Vector3 eulerAngles = Utils.ToEulerAngles(Quaternion.Inverse(orgQuat));

                    float pitchRad = eulerAngles.Y;   // Ángulo de cabeceo (X)
                    float yawRad = eulerAngles.Z;     // Ángulo de guiñada (Y)
                    float rollRad = eulerAngles.X;    // Ángulo de alabeo (Z)

                    rotX[index] = pitchRad;
                    rotY[index] = rollRad;
                    rotZ[index] = yawRad;

                    if (!Program.debugSendTrackerOSC) continue;

                    float cy = MathF.Cos(yawRad * 0.5f);
                    float sy = MathF.Sin(yawRad * 0.5f);
                    float cr = MathF.Cos(rollRad * 0.5f);
                    float sr = MathF.Sin(rollRad * 0.5f);
                    float cp = MathF.Cos(pitchRad * 0.5f);
                    float sp = MathF.Sin(pitchRad * 0.5f);
                    // Calcular los componentes del quaternion
                    float w = cy * cr * cp + sy * sr * sp;
                    float x = cy * sr * cp - sy * cr * sp;
                    float y = cy * cr * sp + sy * sr * cp;
                    float z = sy * cr * cp - cy * sr * sp;
                    // Crear el quaternion
                    Quaternion quaternion = new Quaternion(x, y, z, w);
                    Matrix4x4 testRot = Matrix4x4.CreateFromQuaternion(quaternion);
                    testRot = Matrix4x4.Multiply(testRot, orgRot);
                    Matrix4x4 testPoint = Matrix4x4.Multiply(Matrix4x4.CreateTranslation(new Vector3(frameCount % 4 == 4 ? 0.01f : 0, frameCount % 4 == 4 ? 0.01f : 0, 0.05f)), testRot);
                    testPoint.M41 += finalCenterPos.X;
                    testPoint.M42 += finalCenterPos.Y;
                    testPoint.M43 += finalCenterPos.Z;

                    if (index != 0)
                        Program.oscClientDebug.Send("/debug/calibrate/position", index + 20, 3,
                                                   testPoint.Translation.X * 5, testPoint.Translation.Z * 5, testPoint.Translation.Y * 5);
                    //Matrix4x4.Multiply(trackerOffsetsMat[i], mat);
                    if (index == 0)
                        Program.oscClientDebug.Send("/debug/calibrate/position", index + 20, 3,
                                                       finalCenterPos.X * 5, finalCenterPos.Z * 5, finalCenterPos.Y * 5);
                }
                if (!Program.debugSendTrackerOSC) break;
                if (previewTimer.ElapsedMilliseconds > 1000) break;
                System.Threading.Thread.Sleep(33);
            }
            previewTimer.Stop();
            Console.WriteLine("Ended tracker calibration");

            string fileName = $"{name}.txt";
            StreamWriter streamWriter = new StreamWriter(fileName);
            streamWriter.WriteLine(name);
            for (int i = 0; i < trackerAvg.Length; i++) {
                string output = $"{ids[i]} {rotX[i]} {rotY[i]} {rotZ[i]} {pos[i].X} {pos[i].Y} {pos[i].Z}";
                output = output.Replace(',', '.');
                streamWriter.WriteLine(output);
            }
            streamWriter.WriteLine("generatedByCalibration=true");
            streamWriter.Flush();
            streamWriter.Close();
            Console.WriteLine($"File saved as '{fileName}', contents need to paste it in 'trackers.txt'");

            Program.oscClientDebug.Send("/debug/calibrate/clear", 0);
        }
    }
}
