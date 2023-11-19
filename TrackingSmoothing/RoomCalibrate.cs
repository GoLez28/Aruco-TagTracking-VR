using System;
using System.Collections.Generic;
using System.Linq;
using System.Numerics;
using System.Text;
using System.Threading.Tasks;
using static TagTracking.Tag;

namespace TagTracking {
    class RoomCalibrate {
        public static bool endedSearching = true;
        public static bool refineSearch = true;
        public static int refineIterations = 7;
        public static double saveMatTime = -40000;

        public static bool getSnapshot = false;
        public static int getRawTrackersStep = -1;
        public static bool newTrackersReady = false;
        public static bool addNewRaw = false;
        public static long lastTimeGetRaw = 0;
        public static bool timedSnapshot = false;
        public static bool useAnchor = false;
        public static int cameraAnchor = 0;
        public static Matrix4x4 cameraAnchorMat = Matrix4x4.Identity;

        public static void Init() {
            addNewRaw = false;
            newTrackersReady = false;
            combinedTrackers = new();
            getRawTrackersStep = 0;
            getSnapshot = false;
            timedSnapshot = false;
            Console.WriteLine("Press [1] to start averaging, press [2] to add available trackers, press [3] to add automatically");
        }
        public static void Stop(bool wasCancel = true) {
            saveMatTime = -20000;
            if (wasCancel)
                Console.WriteLine("Stopped...");
            else {
                if (!useAnchor)
                    return;
                //fuck this
                Matrix4x4 resMat = cameras[cameraAnchor].matrix;
                //Matrix4x4.Invert(Matrix4x4.Subtract(resMat, cameraAnchorMat), out Matrix4x4 sub);
                Matrix4x4 sourceMatrix = resMat;
                Matrix4x4 targetMatrix = cameraAnchorMat;
                Vector3 sourceTranslation = sourceMatrix.Translation;
                Vector3 targetTranslation = targetMatrix.Translation;
                Matrix4x4 sourceRotation = Matrix4x4.CreateFromQuaternion(Quaternion.CreateFromRotationMatrix(sourceMatrix));

                Matrix4x4 targetRotation = Matrix4x4.CreateFromQuaternion(Quaternion.CreateFromRotationMatrix(targetMatrix));
                Vector3 translationDiff = targetTranslation - sourceTranslation;
                Matrix4x4.Invert(sourceRotation, out Matrix4x4 srcInv);
                Matrix4x4 rotationDiff = Matrix4x4.Multiply(targetRotation, srcInv);
                Matrix4x4 alignedMatrix = Matrix4x4.Multiply(rotationDiff, Matrix4x4.CreateTranslation(translationDiff));

                for (int i = 0; i < cameras.Length; i++) {
                    Vector3 tr = Vector3.Transform(cameras[i].matrix.Translation, alignedMatrix);
                    Matrix4x4 ro = Matrix4x4.Multiply(Matrix4x4.CreateFromQuaternion(Quaternion.CreateFromRotationMatrix(cameras[i].matrix)), rotationDiff);
                    Matrix4x4 newm = Matrix4x4.Multiply(ro, Matrix4x4.CreateTranslation(tr));
                    cameras[i].matrix = newm;
                    Console.WriteLine("lol");
                }
            }
        }
        public static void Calibrate() {
            if (getRawTrackersStep == -2) {
                return;
            }
            if (useAnchor) {
                cameraAnchorMat = cameras[cameraAnchor].matrix;
            } else {
                Program.offsetMat.M41 = 0f;
                Program.offsetMat.M43 = 0f;
                Program.offsetMat.M42 = 0f;
                Program.rotationY = 0;
                Program.rotationZ = 0;
                Program.rotationX = 0;
                Program.ApplyOffset();
            }
            cameras[0].minScore = float.MaxValue;
            cameras[1].minScore = float.MaxValue;
            endedSearching = false;
            saveMatTime = Program.timer.ElapsedMilliseconds;
            getRawTrackersStep = -2;
            Console.WriteLine("Averaging...");
        }
        public static void AnchorDialog() {
            Console.WriteLine("Which camera do you want to anchor");
            string read = Console.ReadLine();
            if (read.Equals("")) {
                Console.WriteLine("Input not valid");
                return;
            }
            int number;
            bool ok = int.TryParse(read, out number);
            if (!ok) {
                Console.WriteLine("Input not valid");
                return;
            }
            useAnchor = true;
            cameraAnchor = number;
            Console.WriteLine($"Camera {cameraAnchor} will be used as anchor");
        }
        public static void GetCalibrationTags() {
            if (getRawTrackersStep <= -1) {
                return;
            }
            if (timedSnapshot) {
                if (Program.timer.ElapsedMilliseconds - lastTimeGetRaw > 3000) {
                    addNewRaw = true;
                    lastTimeGetRaw = Program.timer.ElapsedMilliseconds;
                }
            }
            bool ready = false;
            int addCount = 0;
            List<string> trackersName = new();
            List<int> trackerId = new();
            for (int i = 0; i < trackers.Length; i++) {
                for (int j = 0; j < trackers[i].trackers.Length; j++) {
                    CombinedTracker cbt = trackers[i].trackers[j];
                    bool lessThan = true;
                    for (int k = 0; k < cbt.updateCount.Length; k++) {
                        if (cbt.updateCount[k] > 2) {
                            lessThan = false;
                            break;
                        }
                    }
                    if (lessThan) {
                        ready = true;
                    }
                    if (lessThan && addNewRaw) {
                        addCount++;
                        CombinedTracker cbt2 = new CombinedTracker(cbt.index);
                        //search for name and id
                        for (int k = 0; k < trackers.Length; k++) {
                            for (int l = 0; l < trackers[k].trackerIndex.Length; l++) {
                                if (trackers[k].trackerIndex[l] == cbt2.index) {
                                    trackersName.Add(trackers[k].trackerName);
                                    trackerId.Add(cbt2.index);
                                    break;
                                }
                            }
                        }
                        for (int k = 0; k < cbt.singles.Length; k++) {
                            cbt2.Recieve(k, cbt.singles[k].pos, cbt.singles[k].rot, -1);
                        }
                        combinedTrackers.Add(cbt2);
                    }
                }
            }
            if (addNewRaw && addCount > 0) {
                Console.WriteLine($"Added {addCount}, total: {combinedTrackers.Count}");
                for (int i = 0; i < trackersName.Count; i++) {
                    Console.WriteLine($"\t{trackersName[i]}: {trackerId[i]}");
                }
            }
            addNewRaw = false;
            if (ready && !newTrackersReady) {
                newTrackersReady = true;
                Console.WriteLine("Trackers OK");
            } else if (!ready && newTrackersReady) {
                newTrackersReady = false;
                Console.WriteLine("Trackers Missing");
            }
            newTrackersReady = ready;
        }
        public static void RevieveTrackers(int index, int camera, Matrix4x4 rot, Vector3 pos, int altRot) {
            if (index != 0)
                return;
            if (Program.timer.ElapsedMilliseconds - saveMatTime < 15000) {
                Vector3 vec = pos * cameras[camera].depthMult;
                Matrix4x4 vecMat = Matrix4x4.CreateTranslation(vec);
                Matrix4x4 newMat = Matrix4x4.Multiply(rot, vecMat);
                Matrix4x4 invMat;
                Matrix4x4.Invert(newMat, out invMat);
                string message = $"                              ";
                ApplyNewMatrix(camera, invMat, message);
                Random rnd = new Random();
                //System.Diagnostics.Stopwatch sw = new System.Diagnostics.Stopwatch();
                //sw.Start();
                for (int i = 0; i < 10; i++) {
                    //cameras[camera].matrix.CopyTo(invMat);
                    Matrix4x4 newRot = Matrix4x4.CreateFromYawPitchRoll(
                        (float)(rnd.NextDouble() - 0.5) / (10000f / cameras[camera].minScore),
                        (float)(rnd.NextDouble() - 0.5) / (10000f / cameras[camera].minScore),
                        (float)(rnd.NextDouble() - 0.5) / (10000f / cameras[camera].minScore));
                    Vector3 newTran = new Vector3(
                        (float)(rnd.NextDouble() - 0.5) / (100f / cameras[camera].minScore),
                        (float)(rnd.NextDouble() - 0.5) / (100f / cameras[camera].minScore),
                        (float)(rnd.NextDouble() - 0.5) / (100f / cameras[camera].minScore));
                    Matrix4x4 newRotd = Matrix4x4.Multiply(newRot, Matrix4x4.CreateTranslation(newTran));

                    Matrix4x4 invMat2;
                    int ite = 0;
                    do {
                        rot = Matrix4x4.Multiply(rot, newRot);
                        Matrix4x4 newMat2 = Matrix4x4.Multiply(rot, vecMat);
                        //Vector3 possss = invMat.Translation;
                        Matrix4x4.Invert(newMat2, out invMat2);
                        ite++;
                        message = $"{ite}, by exsisting rnd             ";
                    } while (ApplyNewMatrix(camera, invMat2, message) && ite < 10);

                    //get from current matrix
                    message = $"by new rnd             ";
                    Matrix4x4 rndMat = Matrix4x4.Multiply(invMat, newRotd);
                    while (ApplyNewMatrix(camera, rndMat, message)) {
                        message = $"by repeating new rnd             ";
                        rndMat = Matrix4x4.Multiply(invMat, newRotd);
                    }
                }
                //sw.Stop();
                //Console.WriteLine(sw.Elapsed.TotalMilliseconds);
                //}
            } else {
                if (!endedSearching) {
                    endedSearching = true;
                    Console.WriteLine("Ended Searching for matrices");
                    if (refineSearch && combinedTrackers.Count > 4) {
                        RefineSearch();
                    }
                    Stop(false);
                    SaveMatrix();
                    getRawTrackersStep = -1;
                }
            }
        }
        private static void RefineSearch() {
            if (cameras.Length < 2) {
                Console.WriteLine("Not sufficient cameras to refine matrix");
                return;
            }
            for (int c = 0; c < cameras.Length; c++) {
                if (c == 1) continue;
                Console.WriteLine($"Refining camera:{c} Matrix with {refineIterations} iterations");
                Matrix4x4 m = cameras[c].matrix;
                float pdist = 0;
                for (int k = 0; k < refineIterations; k++) {
                    float[] vals = new float[] {m.M11, m.M12, m.M13, m.M14, m.M21, m.M22, m.M23, m.M24,
                                m.M31, m.M32, m.M33, m.M34, m.M41, m.M42, m.M43, m.M44};
                    float mult = 1 - ((float)k / refineIterations);
                    float span = 0.010f * mult;
                    float step = 0.0005f * mult;
                    float mini = 0;
                    float score = 1000f;
                    Console.Write($"{k}, ({span:0.000}/{step:0.0000}) s: ");
                    for (int j = 0; j < vals.Length; j++) {
                        Console.Write(j + ", ");
                        if (j == 3 || j == 7 || j >= 11) continue; //those dont move //3, 7, 11, 12, 13, 14, 15
                        vals = new float[] {m.M11, m.M12, m.M13, m.M14, m.M21, m.M22, m.M23, m.M24,
                                m.M31, m.M32, m.M33, m.M34, m.M41, m.M42, m.M43, m.M44};
                        float save = vals[j];
                        for (float i = -span; i <= span; i += step) {
                            vals[j] = save + i;
                            ApplyRefinedMatrix(vals, c);
                            float scr = GetDistanceFromEachTracker(c, cameras[c].matrix, 1);
                            if (scr < score) {
                                score = scr;
                                mini = i;
                            }
                        }
                        vals[j] = save + mini;
                        ApplyRefinedMatrix(vals, c);
                        m = cameras[c].matrix;
                        if (Program.debugSendTrackerOSC) {
                            SendSingleTrackersOSC();
                            System.Threading.Thread.Sleep(50);
                        }
                    }
                    Console.WriteLine();
                    float dist = GetDistanceFromEachTracker(c, cameras[c].matrix, 1);
                    if (pdist == dist) {
                        Console.WriteLine("Pretty close already");
                        break;
                    }
                    pdist = dist;
                    if (Program.debugSendTrackerOSC) {
                        SendSingleTrackersOSC();
                        System.Threading.Thread.Sleep(50);
                    }
                }
            }
        }

        static void ApplyRefinedMatrix(float[] vals, int c) {
            cameras[c].matrix = new Matrix4x4(vals[0], vals[1], vals[2], vals[3], vals[4], vals[5], vals[6], vals[7],
                                            vals[8], vals[9], vals[10], vals[11], vals[12], vals[13], vals[14], vals[15]);
            Matrix4x4[] cbt = combinedTrackers[0].Obtain();
            Vector3 pos1 = cbt[c].Translation;
            Vector3 pos2 = cbt[1].Translation;
            Vector3 diff = pos2 - pos1;
            cameras[c].matrix = new Matrix4x4(vals[0], vals[1], vals[2], vals[3], vals[4], vals[5], vals[6], vals[7],
                vals[8], vals[9], vals[10], vals[11], vals[12] + diff.X, vals[13] + diff.Y, vals[14] + diff.Z, vals[15]);
        }

        private static bool ApplyNewMatrix(int camera, Matrix4x4 rndMat, string message, bool raw = false) {
            float score = GetDistanceFromEachTracker(camera, rndMat);
            float dist = score;
            score += GetDistanceFromZero(camera, rndMat) * 0.25f;
            if ((score < cameras[camera].minScore || raw) && !float.IsNaN(score)) {
                cameras[camera].minScore = score;
                cameras[camera].matrix = rndMat;
                //Console.CursorTop--;
                Console.WriteLine($"Updated matrix for cam {camera}: {dist} / {score} " + message);
                return true;
            }
            return false;
        }

        private static float GetDistanceFromEachTracker(int camera, Matrix4x4 mat1, int against = -1) {
            float distSum = 0;
            float[] depthMul = new float[cameras.Length];
            for (int i = 0; i < cameras.Length; i++)
                depthMul[i] = 1f;
            for (int i = 0; i < cameras.Length; i++)
                depthMul[i] = cameras[i].depthMult;
            for (int i = 0; i < combinedTrackers.Count; i++) {
                CombinedTracker tracker = combinedTrackers[i];
                float weight = 1f;
                //for (int j = 0; j < tagToCalibrate.Length; j++) {
                //    if (id == tagToCalibrate[j]) {
                //        found = true;
                //        weight = tagToCalibrateWeight[j];
                //        break;
                //    }
                //}
                //if (!found) continue;
                if (tracker.singles.Length < 2) continue;
                Vector3 pos1 = tracker.singles[camera].pos * depthMul[camera];
                pos1 = Vector3.Transform(pos1, mat1);
                for (int j = 0; j < cameras.Length; j++) {
                    //TODO: this may be broken
                    if (j == camera) continue;
                    if (against != -1) {
                        if (against != j) continue;
                    }
                    Vector3 pos2 = tracker.singles[j].pos * depthMul[j];
                    pos2 = Vector3.Transform(pos2, cameras[j].matrix);
                    float dist = Utils.GetDistance(pos1.X, pos1.Y, pos1.Z, pos2.X, pos2.Y, pos2.Z);
                    dist *= weight;
                    distSum += dist;
                }
            }
            return distSum;
        }
        private static float GetDistanceFromZero(int camera, Matrix4x4 mat, bool modDepth = true) {
            float distSum = 0;
            float depthMul = 1f;
            if (modDepth) {
                depthMul = cameras[camera].depthMult;
            }
            Vector3 sum = new Vector3();
            int count = 0;
            for (int i = 0; i < combinedTrackers.Count; i++) {
                CombinedTracker tracker = combinedTrackers[i];
                //if (!(tracker.index == 0 || tracker.index == 2 || tracker.index == 4 || tracker.index == 6)) continue; //to not get garbage
                bool found = false;
                int id = tracker.index;
                for (int j = 0; j < tagsOnFloor.Length; j++) {
                    if (id == tagsOnFloor[j]) {
                        found = true;
                        break;
                    }
                }
                if (!found) continue;
                Vector3 pos1 = tracker.singles[camera].pos * depthMul;
                pos1 = Vector3.Transform(pos1, mat);
                sum += pos1;
                count++;
            }
            float dist = Utils.GetDistance(sum.X, sum.Y, sum.Z, 0f, 0f, 0f);
            distSum += dist;
            //emphazises in floor level
            dist = Math.Abs(sum.Z);
            distSum += dist;
            return distSum;
        }
    }
}
