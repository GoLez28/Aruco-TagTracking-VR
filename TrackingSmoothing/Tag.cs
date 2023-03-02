using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Numerics;
using System.IO;

namespace TrackingSmoothing {
    static partial class Tag {
        public class FinalTracker {
            public Vector3 pos;
            public Quaternion rot;
            public Quaternion prot;
            public string name;

            public Vector3 fpos;
            public Quaternion frot;

            public OneEuroFilter<Vector3> smoothedPos = new OneEuroFilter<Vector3>(2); //w 25
            public OneEuroFilter<Quaternion> smoothedRot = new OneEuroFilter<Quaternion>(5); //w 150
            public Vector3 alwaysSmoothed = new Vector3();
            public Vector3 smoothPrevPos = new Vector3();
            public Quaternion smoothPrevRot = new Quaternion();
            public float avgSmoothDistTrigger = 0.025f; //w 0.05
            public float avgSmoothVal = 0.2f; //w 0.04
            public float avgSmoothRecoverVal = 0.9f;
            public float avgSmoothAlwaysVal = 0.08f;

            public FinalTracker(Vector3 pos, Quaternion rot, Quaternion prot, string name) {
                this.pos = pos;
                this.rot = rot;
                this.prot = prot;
                this.name = name;
            }

            public void Update() {
                if (!Program.postNoise) {
                    fpos = pos;
                    frot = rot;
                    return;
                }

                if (!float.IsNaN(rot.X))
                    frot = smoothedRot.Filter(rot);
                if (!float.IsNaN(pos.X))
                    fpos = smoothedPos.Filter(pos);
                //fpos = pos;

                float distTh = avgSmoothDistTrigger; //0.0004f
                float smoothiness = avgSmoothVal;
                alwaysSmoothed += (fpos - alwaysSmoothed) * avgSmoothAlwaysVal;
                if (Utils.GetDistance(fpos.X, fpos.Y, fpos.Z, alwaysSmoothed.X, alwaysSmoothed.Y, alwaysSmoothed.Z) > distTh) {
                    smoothiness = avgSmoothRecoverVal;
                }
                smoothPrevPos += (fpos - smoothPrevPos) * smoothiness;
                fpos = smoothPrevPos;

                float smoothinessRot = smoothiness + (1 - smoothiness) / 2; //to be less smoothed
                smoothPrevRot.X += (frot.X - smoothPrevRot.X) * smoothinessRot;
                smoothPrevRot.Y += (frot.Y - smoothPrevRot.Y) * smoothinessRot;
                smoothPrevRot.Z += (frot.Z - smoothPrevRot.Z) * smoothinessRot;
                smoothPrevRot.W += (frot.W - smoothPrevRot.W) * smoothinessRot;
                smoothPrevRot = Quaternion.Normalize(smoothPrevRot);
                frot = smoothPrevRot;
            }
        }
        public class RecieveTag {
            public Matrix4x4 rot;
            public Vector3 pos;
            public int index;
            public int camera;
            public RecieveTag(Matrix4x4 rot, Vector3 pos, int index, int camera) {
                this.rot = rot;
                this.pos = pos;
                this.index = index;
                this.camera = camera;
            }
        }
        public class Camera {
            public Matrix4x4 matrix = new Matrix4x4();
            public float minScore = Single.MaxValue;
            public float depthMult = 1f;
            public float[,] smoothening = new float[7, 3];
            public float quality = 1f;
            public string file = "";
            public int width = 640;
            public int height = 480;
            public int index = 0;
            public Camera(Matrix4x4 m, float q, float d) {
                matrix = m;
                quality = q;
                depthMult = d;
            }
        }
        static Quaternion Rotation(this Matrix4x4 mat) {
            return Quaternion.CreateFromRotationMatrix(mat);
        }
        public static Camera[] cameras = new Camera[] {
            //matrixes will be updated after
            new Camera(new Matrix4x4(
                -0.611f, 0.489f, -0.623f, -0.369f,
                0.790f, 0.324f, -0.520f, 0.026f,
                -0.053f, -0.81f, -0.584f, 2.268f,
                0.0f, 0.0f, 0.0f, 1.0f ) , 1f, 0.965f),
            new Camera(new Matrix4x4(
                -0.611f, 0.489f, -0.623f, -0.369f,
                0.790f, 0.324f, -0.520f, 0.026f,
                -0.053f, -0.81f, -0.584f, 2.268f,
                0.0f, 0.0f, 0.0f, 1.0f ) , 1.1f, 1f)
        };
        public static ClusterTracker[] trackers = new ClusterTracker[] {
            new ClusterTracker("rightfoot",
                new int[] { 0, 1, 2, 3 },
                new Vector3 [] {
                    new Vector3 (0f, 0f, -0.055f),
                    new Vector3 (0f, 0f, -0.055f),
                    new Vector3 (0f, 0f, -0.055f),
                    new Vector3 (0f, 0f, -0.055f)
                },
                new float[] {0f, (float)Math.PI / 2, (float)Math.PI, (float)Math.PI * 1.5f}
                ),
            new ClusterTracker("leftfoot",
                new int[] { 4, 5, 6, 7 },
                new Vector3 [] {
                    new Vector3 (0f, 0f, -0.055f),
                    new Vector3 (0f, 0f, -0.055f),
                    new Vector3 (0f, 0f, -0.055f),
                    new Vector3 (0f, 0f, -0.055f)
                },
                new float[] {0f, (float)Math.PI / 2, (float)Math.PI, (float)Math.PI * 1.5f}
                ),
            new ClusterTracker("waist",
                new int[] { 8, 9, 10, 11 },
                new Vector3 [] {
                    new Vector3 (0.0f, 0.00f, -0.17f),
                    new Vector3 (0.0f, 0.00f, -0.12f),
                    new Vector3 (-0.0f, -0.00f, -0.12f),
                    new Vector3 (-0.0f, -0.00f, -0.17f)
                },
                new float[] {0f, (float)Math.PI * 0.3f, (float)Math.PI * 0.7f, (float)Math.PI * 1f}
                )
        };
        public static CombinedTracker[] combinedTrackers = new CombinedTracker[] {
            new CombinedTracker(0), new CombinedTracker(1), new CombinedTracker(2), new CombinedTracker(3), new CombinedTracker(4), new CombinedTracker(5), new CombinedTracker(6),
            new CombinedTracker(7), new CombinedTracker(8), new CombinedTracker(9), new CombinedTracker(10), new CombinedTracker(11), new CombinedTracker(12), new CombinedTracker(13),
            new CombinedTracker(14), new CombinedTracker(15)
        };
        public static int[] tagToCalibrate = new int[] { 0, 2, 4, 6 };
        public static int[] tagsOnFloor = new int[] { 0, 2, 4, 6 };
        public static float[] tagToCalibrateWeight = new float[] { 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1 };
        public static FinalTracker[] finals;
        public static bool newInfoReady = false;
        public static bool newInfo = false;

        public static bool refineSearch = true;
        public static int refineIterations = 7;

        public static bool endedSearching = true;
        public static double saveMatTime = -40000;

        public static double lastFrameTime = 0;
        public static int lastCamera = 0;
        public static int lastIndex = 0;
        public static double[] cameraTPS = new double[2];
        public static void SaveMatrix() {
            Console.WriteLine("Saving...");
            for (int i = 0; i < 2; i++) {
                using (StreamWriter sw = new StreamWriter("camMat" + i)) {
                    sw.WriteLine(cameras[i].matrix.M11);
                    sw.WriteLine(cameras[i].matrix.M12);
                    sw.WriteLine(cameras[i].matrix.M13);
                    sw.WriteLine(cameras[i].matrix.M14);
                    sw.WriteLine(cameras[i].matrix.M21);
                    sw.WriteLine(cameras[i].matrix.M22);
                    sw.WriteLine(cameras[i].matrix.M23);
                    sw.WriteLine(cameras[i].matrix.M24);
                    sw.WriteLine(cameras[i].matrix.M31);
                    sw.WriteLine(cameras[i].matrix.M32);
                    sw.WriteLine(cameras[i].matrix.M33);
                    sw.WriteLine(cameras[i].matrix.M34);
                    sw.WriteLine(cameras[i].matrix.M41);
                    sw.WriteLine(cameras[i].matrix.M42);
                    sw.WriteLine(cameras[i].matrix.M43);
                    sw.WriteLine(cameras[i].matrix.M44);
                }
            }
        }
        public static void ReadMatrix() {
            for (int i = 0; i < 2; i++) {
                if (!File.Exists("camMat" + i)) continue;
                string[] lines = File.ReadAllLines("camMat" + i);
                int l = 0;
                cameras[i].matrix.M11 = float.Parse(lines[l++]);
                cameras[i].matrix.M12 = float.Parse(lines[l++]);
                cameras[i].matrix.M13 = float.Parse(lines[l++]);
                cameras[i].matrix.M14 = float.Parse(lines[l++]);
                cameras[i].matrix.M21 = float.Parse(lines[l++]);
                cameras[i].matrix.M22 = float.Parse(lines[l++]);
                cameras[i].matrix.M23 = float.Parse(lines[l++]);
                cameras[i].matrix.M24 = float.Parse(lines[l++]);
                cameras[i].matrix.M31 = float.Parse(lines[l++]);
                cameras[i].matrix.M32 = float.Parse(lines[l++]);
                cameras[i].matrix.M33 = float.Parse(lines[l++]);
                cameras[i].matrix.M34 = float.Parse(lines[l++]);
                cameras[i].matrix.M41 = float.Parse(lines[l++]);
                cameras[i].matrix.M42 = float.Parse(lines[l++]);
                cameras[i].matrix.M43 = float.Parse(lines[l++]);
                cameras[i].matrix.M44 = float.Parse(lines[l++]);
            }
        }
        public static void ReadTrackers() {
            if (!File.Exists("trackers.txt")) {
                Console.WriteLine("No Trackers Availables, check file \"trackers.txt\" to add trackers");
                return;
            }
            string[] lines = File.ReadAllLines("trackers.txt");
            List<ClusterTracker> trackerss = new List<ClusterTracker>();
            ClusterTracker currentTracker = null;
            List<int> indexes = new List<int>();
            List<float> rots = new List<float>();
            List<Vector3> offsets = new List<Vector3>();
            string name = "";
            int i = 0;
            while (true) {
                if (i >= lines.Length) break;
                string[] split = lines[i++].Split();
                if (split.Length == 1) {
                    if (split[0].Equals("")) continue;
                    if (indexes.Count > 0 || currentTracker != null) {
                        if (currentTracker == null)
                            currentTracker = new ClusterTracker(name, indexes.ToArray(), offsets.ToArray(), rots.ToArray());

                        if (split[0].Contains("=")) {
                            string[] split2 = split[0].Split("=");
                            if (split2.Length != 2) continue;

                            float val = float.Parse(split2[1], System.Globalization.NumberStyles.Any, System.Globalization.CultureInfo.InvariantCulture);
                            if (split2[0].Equals("avgSmoothDistTrigger")) currentTracker.avgSmoothDistTrigger = val;
                            else if (split2[0].Equals("avgSmoothVal")) currentTracker.avgSmoothVal = val;
                            else if (split2[0].Equals("avgSmoothRecoverVal")) currentTracker.avgSmoothRecoverVal = val;
                            else if (split2[0].Equals("avgSmoothAlwaysVal")) currentTracker.avgSmoothAlwaysVal = val;
                            else if (split2[0].Equals("rotationComparison")) currentTracker.rotationComparison = val;
                            else if (split2[0].Equals("straightTrackerWeight")) currentTracker.straightTrackerWeight = val;
                            else if (split2[0].Equals("filterSmoothRot")) currentTracker.smoothedRot = val;
                            else if (split2[0].Equals("filterSmoothPos")) currentTracker.smoothedPos = val;
                            else if (split2[0].Equals("filterSmoothPerTrackerRot")) currentTracker.UpdatePerTrackerFilterRot(val);
                            else if (split2[0].Equals("filterSmoothPerTrackerDepth")) currentTracker.UpdatePerTrackerFilterDepth(val);
                            else if (split2[0].Equals("trackerFollowWeight")) currentTracker.trackerFollowWeight = val;

                        } else {
                            trackerss.Add(currentTracker);
                            currentTracker = null;
                        }
                    }
                    name = split[0];
                    indexes.Clear();
                    rots.Clear();
                    offsets.Clear();
                    continue;
                }
                if (split.Length != 5) {
                    Console.WriteLine($"Incorrect tracker: line {(i - 1)}");
                }
                indexes.Add(int.Parse(split[0]));
                rots.Add(float.Parse(split[1], System.Globalization.NumberStyles.Any, System.Globalization.CultureInfo.InvariantCulture));
                Vector3 offset = new Vector3();
                offset.X = float.Parse(split[2], System.Globalization.NumberStyles.Any, System.Globalization.CultureInfo.InvariantCulture);
                offset.Y = float.Parse(split[3], System.Globalization.NumberStyles.Any, System.Globalization.CultureInfo.InvariantCulture);
                offset.Z = float.Parse(split[4], System.Globalization.NumberStyles.Any, System.Globalization.CultureInfo.InvariantCulture);
                offsets.Add(offset);
            }
            if (currentTracker != null) {
                trackerss.Add(currentTracker);
            }
            trackers = trackerss.ToArray();
            finals = new FinalTracker[trackers.Length];
            for (int j = 0; j < trackers.Length; j++) {
                finals[j] = new(new(), new(), new(), trackers[j].trackerName);
                finals[j].avgSmoothAlwaysVal = trackers[j].avgSmoothAlwaysVal;
                finals[j].avgSmoothVal = trackers[j].avgSmoothVal;
                finals[j].avgSmoothDistTrigger = trackers[j].avgSmoothDistTrigger;
                finals[j].avgSmoothRecoverVal = trackers[j].avgSmoothRecoverVal;
                finals[j].smoothedRot.UpdateParams(trackers[j].smoothedRot);
                finals[j].smoothedPos.UpdateParams(trackers[j].smoothedPos);
            }
        }
        public static List<RecieveTag> tagsList = new List<RecieveTag>();
        public static void RecieveTrackerAsync(int index, int camera, Matrix4x4 rot, Vector3 pos) {
            tagsList.Add(new RecieveTag(rot, pos, index, camera));
        }
        public static void Update() {
            if (!newInfoReady) return;
            List<RecieveTag> tagsListCopy = tagsList;
            tagsList = new List<RecieveTag>();
            newInfoReady = false;
            if (Program.debugSendTrackerOSC)
                Program.oscClient.Send($"/debug/tick", 0);
            for (int i = 0; i < tagsListCopy.Count; i++) {
                RecieveTag tag = tagsListCopy[i];
                RecieveTracker(tag.index, tag.camera, tag.rot, tag.pos);
            }
            SendSingleTrackersOSC();
            newInfo = true;
        }
        private static void SendSingleTrackersOSC() {
            if (!Program.debugSendTrackerOSC) {
                return;
            }
            for (int i = 0; i < combinedTrackers.Length; i++) {
                CombinedTracker tracker = combinedTrackers[i];
                int id = tracker.index;
                Matrix4x4[] poss = tracker.Obtain();
                Program.oscClient.Send($"/debug/trackers/position", id, 0, poss[0].Translation.X, poss[0].Translation.Z, poss[0].Translation.Y);
                Program.oscClient.Send($"/debug/trackers/position", id, 1, poss[1].Translation.X, poss[1].Translation.Z, poss[1].Translation.Y);
            }
        }

        public static void RecieveTracker(int index, int camera, Matrix4x4 rot, Vector3 pos) {
            //if (!positionObtained) GetCameraPosition();

            if (camera != lastCamera || (camera == lastCamera && lastIndex >= index)) {
                double tps = 1000.0 / (Program.timer.Elapsed.TotalMilliseconds - lastFrameTime);
                if (tps < 100.0)
                    cameraTPS[camera] += (tps - cameraTPS[camera]) * 0.5f;
                lastFrameTime = Program.timer.Elapsed.TotalMilliseconds;
            }
            lastCamera = camera;
            lastIndex = index;
            for (int k = 0; k < combinedTrackers.Length; k++) {
                if (combinedTrackers[k].index == index) {
                    combinedTrackers[k].Recieve(camera, pos, rot);
                    break;
                }
            }
            if (index == 0)
                if (Program.timer.ElapsedMilliseconds - saveMatTime < 20000) {
                    Vector3 vec = new Vector3();
                    //int count = 0;
                    //for (int i = 0; i < combinedTrackers.Length; i++) {
                    //    if (!(index == 0 || index == 2 || index == 4 || index == 6)) continue; //to not get garbage
                    //    vec.X += cameras[camera].smoothening[index, 0];
                    //    vec.Y += cameras[camera].smoothening[index, 1];
                    //    vec.Z += cameras[camera].smoothening[index, 2];
                    //    count++;
                    //}
                    //vec /= count;
                    vec = pos * cameras[camera].depthMult;
                    //Quaternion quat = new Quaternion();
                    //for (int i = 0; i < combinedTrackers.Length; i++) {
                    //    if (!(index == 0 || index == 2 || index == 4 || index == 6)) continue; //to not get garbage
                    //    float mult = (1f / count) / cameras.Length;
                    //    Matrix4x4 results = combinedTrackers[i].singles[camera].rot;
                    //    quat += results.Rotation() * mult;
                    //}
                    //quat = Quaternion.Normalize(quat);
                    //Vector3 vec = new Vector3(
                    //    cameras[camera].smoothening[index, 0],
                    //    cameras[camera].smoothening[index, 1],
                    //    cameras[camera].smoothening[index, 2]);
                    //rot = Matrix4x4.Negate(rot);
                    Matrix4x4 vecMat = Matrix4x4.CreateTranslation(vec);
                    Matrix4x4 newMat = Matrix4x4.Multiply(rot, vecMat);
                    Matrix4x4 invMat;
                    //Vector3 possss = invMat.Translation;
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
                        //get from exixsting matrix

                        message = $"0, by exsisting rnd             ";
                        Matrix4x4 rndMat = Matrix4x4.Multiply(cameras[camera].matrix, newRotd);
                        //while (ApplyNewMatrix(camera, depthMult, rndMat, message)) {
                        //    message = $"by repeating exsisting rnd {depthMult}             ";
                        //    rndMat = Matrix4x4.Multiply(cameras[camera].matrix, newRotd);
                        //}

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
                        rndMat = Matrix4x4.Multiply(invMat, newRotd);
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
                        if (refineSearch) {
                            RefineSearch();
                        }
                        SaveMatrix();
                    }
                }
            for (int i = 0; i < trackers.Length; i++) {
                for (int j = 0; j < trackers[i].trackerIndex.Length; j++) {
                    if (trackers[i].trackerIndex[j] == index) {
                        trackers[i].trackers[j].Recieve(camera, pos, rot);
                        //trackers[i].updateCount[j] = 0;
                        break;
                    }
                }
            }
        }

        private static void RefineSearch() {
            Console.WriteLine($"Refining Matrix with {refineIterations} iterations");
            Matrix4x4 trueMat = cameras[0].matrix;
            float pdist = 0;
            for (int k = 0; k < refineIterations; k++) {
                float[] vals = new float[] {trueMat.M11, trueMat.M12, trueMat.M13, trueMat.M14, trueMat.M21, trueMat.M22, trueMat.M23, trueMat.M24,
                                trueMat.M31, trueMat.M32, trueMat.M33, trueMat.M34, trueMat.M41, trueMat.M42, trueMat.M43, trueMat.M44};
                float mult = (float)((((refineIterations - 1) * 2) - k) / (float)((refineIterations - 1) * 2));
                float span = 0.25f * mult;
                float step = 0.01f * mult;
                float mini = 0;
                float score = 1000f;
                Console.Write($"{k}, ({span:0.00}/{step:0.00}) s: ");
                for (int j = 0; j < vals.Length; j++) {
                    Console.Write(j + ", ");
                    if (j == 3 || j == 7 || j >= 11) continue; //those dont move //3, 7, 11, 12, 13, 14, 15
                    vals = new float[] {trueMat.M11, trueMat.M12, trueMat.M13, trueMat.M14, trueMat.M21, trueMat.M22, trueMat.M23, trueMat.M24,
                                trueMat.M31, trueMat.M32, trueMat.M33, trueMat.M34, trueMat.M41, trueMat.M42, trueMat.M43, trueMat.M44};
                    float save = vals[j];
                    for (float i = -0.25f; i <= 0.25f; i += 0.01f) {
                        vals[j] = save + i;
                        ApplyRefinedMatrix(vals);
                        //SendTrackerOSC();
                        //System.Threading.Thread.Sleep(100);
                        float scr = GetDistanceFromEachTracker(0, cameras[0].matrix, cameras[1].matrix);
                        if (scr < score) {
                            score = scr;
                            mini = i;
                        }
                    }
                    vals[j] = save + mini;
                    ApplyRefinedMatrix(vals);
                    trueMat = cameras[0].matrix;
                    if (Program.debugSendTrackerOSC) {
                        SendSingleTrackersOSC();
                        System.Threading.Thread.Sleep(50);
                    }
                }
                Console.WriteLine();
                float dist = GetDistanceFromEachTracker(0, cameras[0].matrix, cameras[1].matrix);
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

        static void ApplyRefinedMatrix(float[] vals) {
            cameras[0].matrix = new Matrix4x4(vals[0], vals[1], vals[2], vals[3], vals[4], vals[5], vals[6], vals[7],
                                            vals[8], vals[9], vals[10], vals[11], vals[12], vals[13], vals[14], vals[15]);
            Matrix4x4[] cbt = combinedTrackers[0].Obtain();
            Vector3 pos1 = cbt[0].Translation;
            Vector3 pos2 = cbt[1].Translation;
            Vector3 diff = pos2 - pos1;
            cameras[0].matrix = new Matrix4x4(vals[0], vals[1], vals[2], vals[3], vals[4], vals[5], vals[6], vals[7],
                vals[8], vals[9], vals[10], vals[11], vals[12] + diff.X, vals[13] + diff.Y, vals[14] + diff.Z, vals[15]);
        }

        private static bool ApplyNewMatrix(int camera, Matrix4x4 rndMat, string message, bool raw = false) {
            float score = 0;
            if (camera == 0)
                score = GetDistanceFromEachTracker(camera, rndMat, cameras[1].matrix);
            else if (camera == 1)
                score = GetDistanceFromEachTracker(camera, cameras[0].matrix, rndMat);
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

        private static float GetDistanceFromEachTracker(int camera, Matrix4x4 mat1, Matrix4x4 mat2, bool modDepth = true) {
            float distSum = 0;
            float depthMul1 = 1f;
            float depthMul2 = 1f;
            if (modDepth) {
                depthMul1 = cameras[0].depthMult;
                depthMul2 = cameras[1].depthMult;
            }
            for (int i = 0; i < combinedTrackers.Length; i++) {
                CombinedTracker tracker = combinedTrackers[i];
                int id = tracker.index;
                bool found = false;
                float weight = 1f;
                for (int j = 0; j < tagToCalibrate.Length; j++) {
                    if (id == tagToCalibrate[j]) {
                        found = true;
                        weight = tagToCalibrateWeight[j];
                        break;
                    }
                }
                if (!found) continue;
                Vector3 pos1 = tracker.singles[0].pos * depthMul1;
                pos1 = Vector3.Transform(pos1, mat1);
                Vector3 pos2 = tracker.singles[1].pos * depthMul2;
                pos2 = Vector3.Transform(pos2, mat2);
                float dist = Utils.GetDistance(pos1.X, pos1.Y, pos1.Z, pos2.X, pos2.Y, pos2.Z);
                dist *= weight;
                distSum += dist;
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
            for (int i = 0; i < combinedTrackers.Length; i++) {
                CombinedTracker tracker = combinedTrackers[i];
                if (!(tracker.index == 0 || tracker.index == 2 || tracker.index == 4 || tracker.index == 6)) continue; //to not get garbage
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

        public static void GetTrackers() {
            if (finals == null) return;
            for (int i = 0; i < trackers.Length; i++) {
                Matrix4x4 mat = trackers[i].Obtain();
                Vector4 headPos = Program.hmdList[0];
                mat = Matrix4x4.Multiply(mat, Program.offsetMat);
                mat.M41 -= (headPos.X - Program.hmdPos[0]) * trackers[i].trackerFollowWeight;
                mat.M43 -= (headPos.Y - Program.hmdPos[1]) * trackers[i].trackerFollowWeight;
                mat.M42 += (headPos.Z - Program.hmdPos[2]) * trackers[i].trackerFollowWeight;
                Vector3 pos = mat.Translation;
                Quaternion q = mat.Rotation();
                Quaternion pq = trackers[i].prevRotFinal;
                string name = trackers[i].trackerName;
                finals[i].pos = pos;
                finals[i].prot = finals[i].rot;
                finals[i].rot = q;
                finals[i].Update();
                Matrix4x4 previewMat = Matrix4x4.Multiply(Matrix4x4.CreateFromQuaternion(finals[i].frot), Matrix4x4.CreateTranslation(finals[i].fpos));
                Matrix4x4 inv;
                Matrix4x4.Invert(Program.offsetMat, out inv);
                mat = Matrix4x4.Multiply(previewMat, inv);
                mat.M41 += (headPos.X - Program.hmdPos[0]) * trackers[i].trackerFollowWeight;
                mat.M43 += (headPos.Y - Program.hmdPos[1]) * trackers[i].trackerFollowWeight;
                mat.M42 -= (headPos.Z - Program.hmdPos[2]) * trackers[i].trackerFollowWeight;
                Aruco.DrawAxis(mat);
                //Program.oscClient.Send("/VMT/Room/Unity", i + 1, 1, 0f,
                //                            pos.X, pos.Z, pos.Y, //1f, 1.7f, 1f
                //                            -q.X, -q.Z, -q.Y, q.W); //idk, this works lol //XZYW 2.24
            }
            newInfo = false;
        }

        public static void SendTrackers() {
            //for (int i = 0; i < trackers.Length; i++) {
            //    Matrix4x4 mat = trackers[i].Obtain();
            //    Vector4 headPos = Program.hmdList[0];
            //    mat = Matrix4x4.Multiply(mat, Program.offsetMat);
            //    mat.M41 -= (headPos.X - Program.hmdPos[0]) * trackers[i].trackerFollowWeight;
            //    mat.M43 -= (headPos.Y - Program.hmdPos[1]) * trackers[i].trackerFollowWeight;
            //    mat.M42 += (headPos.Z - Program.hmdPos[2]) * trackers[i].trackerFollowWeight;
            //    Vector3 pos = mat.Translation;
            //    Quaternion q = Quaternion.CreateFromRotationMatrix(mat);
            //    Program.oscClient.Send("/VMT/Room/Unity", i + 1, 1, 0f,
            //                                pos.X, pos.Z, pos.Y, //1f, 1.7f, 1f
            //                                -q.X, -q.Z, -q.Y, q.W); //idk, this works lol //XZYW 2.24
            //}
            //newInfo = false;
            for (int i = 0; i < finals.Length; i++) {
                Vector3 pos = finals[i].fpos;
                Quaternion q = finals[i].frot;
                if (Program.useVRChatOSCTrackers) {
                    Program.oscClient.Send($"/tracking/trackers/{i + 1}/position", pos.X, pos.Z, pos.Y);
                    Vector3 e = ToEulerAngles(q);
                    Program.oscClient.Send($"/tracking/trackers/{i + 1}/rotation", e.X, e.Z, e.Y);
                } else {
                    Program.oscClient.Send("/VMT/Room/Unity", i + 1, 1, 0f,
                                                pos.X, pos.Z, pos.Y, //1f, 1.7f, 1f
                                                -q.X, -q.Z, -q.Y, q.W); //idk, this works lol //XZYW 2.24
                }
            }
            //i didnt implement vrcosc correctly bc it doesnt use the rotation and idk why
            if (Program.useVRChatOSCTrackers && Program.sendHeadTracker) {
                float[] headpos = Program.hmdPos;
                float[] headrot = Program.hmdRot;
                Program.oscClient.Send($"/tracking/trackers/head/position", headpos[0], headpos[1], headpos[2]);
                Program.oscClient.Send($"/tracking/trackers/head/rotation", headrot[0], headrot[1], headrot[2]);
            }
        }

        public static float backwardsAngle = 0.6f;
        public static float panAngleR = -0.5f;
        public static float panAngleL = 0.5f;
        public static void AdjustPose() {
            int waist = -1;
            for (int i = 0; i < finals.Length; i++) {
                if (finals[i].name.Equals(Program.poseAdjustWaist)) {
                    waist = i;
                    break;
                }
            }
            List<int> adjustables = new();
            for (int i = 0; i < finals.Length; i++) {
                if (!finals[i].name.Equals(Program.poseAdjustWaist)) {
                    adjustables.Add(i);
                }
            }

            Matrix4x4 waistMat = Matrix4x4.Multiply(Matrix4x4.CreateFromQuaternion(finals[waist].rot), Matrix4x4.CreateTranslation(finals[waist].pos));
            Matrix4x4 waistInv;
            Matrix4x4.Invert(waistMat, out waistInv);


            //how tf do i use quaternions
            waistMat = Matrix4x4.Multiply(waistMat, waistInv);
            int sendCount = 0;
            if (Program.debugSendTrackerOSC) {
                Program.oscClient.Send($"/debug/adjust/position", sendCount, 2, waistMat.Translation.X, waistMat.Translation.Z, waistMat.Translation.Y);
                sendCount++;
            }
            //finals[waist].pos = waistMat.Translation;
            //finals[waist].rot = waistMat.Rotation();
            float legDist = Program.poseAdjustLegDist;
            for (int i = 0; i < adjustables.Count; i++) {
                bool isIncorrect = false;
                float lol = Program.timer.ElapsedMilliseconds / 1000f;
                Matrix4x4 adjmp, adjmr;

                //get pos
                adjmp = Matrix4x4.Multiply(Matrix4x4.CreateTranslation(finals[adjustables[i]].pos), waistInv);


                //Console.WriteLine(adjmp.Translation.Y - waistMat.Translation.Y);
                float yDist = adjmp.Translation.Y - waistMat.Translation.Y;
                //finals[adjustables[i]].pos = adjmp.Translation;
                if (yDist > -legDist * 0.5f) {
                    if (Program.debugSendTrackerOSC) {
                        sendCount += 2;
                        Program.oscClient.Send($"/debug/adjust/position", sendCount, 0, adjmp.Translation.X, adjmp.Translation.Z, adjmp.Translation.Y);
                        sendCount++;
                    }
                    continue; //dont adjust
                }

                float pi = (float)Math.PI;
                //get rot
                adjmr = Matrix4x4.Multiply(Matrix4x4.CreateFromQuaternion(finals[adjustables[i]].rot), waistInv);
                //finals[adjustables[i]].rot = adjmr.Rotation();

                //backwards rotation fix
                Matrix4x4 mat = Matrix4x4.Multiply(Matrix4x4.CreateFromQuaternion(adjmr.Rotation()), Matrix4x4.CreateTranslation(adjmp.Translation));
                float brfX = (float)(legDist * Math.Sin(backwardsAngle));
                float brfY = (float)(legDist * Math.Cos(backwardsAngle));
                mat = Matrix4x4.Multiply(Matrix4x4.CreateTranslation(new Vector3(brfX, brfY, 0)), mat);

                if (Program.debugSendTrackerOSC) {
                    Program.oscClient.Send($"/debug/adjust/position", sendCount, 1, mat.Translation.X, mat.Translation.Z, mat.Translation.Y);
                    sendCount++;
                }

                //finals[1].pos = mat.Translation;
                //finals[1].pos.X += (float)Math.Sin(lol) / 5f;

                if (mat.Translation.X < adjmp.Translation.X * 0.4f)
                    isIncorrect = true;

                //vertical rotation fix
                mat = Matrix4x4.Multiply(Matrix4x4.CreateFromQuaternion(adjmr.Rotation()), Matrix4x4.CreateTranslation(adjmp.Translation));
                float angleDir = 0;
                if (finals[adjustables[i]].name.Contains("right")) {
                    angleDir = panAngleR;
                } else if (finals[adjustables[i]].name.Contains("left")) {
                    angleDir = panAngleL;
                }
                float vrfX = (float)(legDist * Math.Sin(angleDir));
                float vrfY = (float)(legDist * Math.Cos(angleDir));
                mat = Matrix4x4.Multiply(Matrix4x4.CreateTranslation(new Vector3(vrfY, 0, vrfX)), mat);

                if (Program.debugSendTrackerOSC) {
                    Program.oscClient.Send($"/debug/adjust/position", sendCount, 1, mat.Translation.X, mat.Translation.Z, mat.Translation.Y);
                    sendCount++;
                }

                //finals[1].pos = mat.Translation;

                if (mat.Translation.X < (adjmp.Translation.X - legDist / 1.75f))
                    isIncorrect = true;

                //Console.WriteLine(isIncorrect);
                if (isIncorrect) {
                    finals[adjustables[i]].rot = finals[adjustables[i]].prot;
                    if (Program.debugSendTrackerOSC) {
                        Program.oscClient.Send($"/debug/adjust/position", sendCount, 3, adjmp.Translation.X, adjmp.Translation.Z, adjmp.Translation.Y);
                        sendCount++;
                    }
                } else {
                    if (Program.debugSendTrackerOSC) {
                        Program.oscClient.Send($"/debug/adjust/position", sendCount, 0, adjmp.Translation.X, adjmp.Translation.Z, adjmp.Translation.Y);
                        sendCount++;
                    }
                }
            }

            //for (int i = 0; i < finals.Length; i++) {
            //    Matrix4x4 mat = Matrix4x4.Multiply(Matrix4x4.CreateFromQuaternion(finals[i].rot), Matrix4x4.CreateTranslation(finals[i].pos));
            //    Aruco.DrawAxis(mat);
            //}
            //System.Threading.Thread.Sleep(50);

            static void GetYawPitchRoll(Quaternion q, out double yaw, out double pitch, out double roll) {
                yaw = Math.Atan2(2.0 * (q.Y * q.Z + q.W * q.X), q.W * q.W - q.X * q.X - q.Y * q.Y + q.Z * q.Z);
                pitch = Math.Asin(-2.0 * (q.X * q.Z - q.W * q.Y));
                roll = Math.Atan2(2.0 * (q.X * q.Y + q.W * q.Z), q.W * q.W + q.X * q.X - q.Y * q.Y - q.Z * q.Z);
            }
        }
        public static Vector3 ToEulerAngles(Quaternion q) {
            Vector3 angles = new();

            // roll / x
            double sinr_cosp = 2 * (q.W * q.X + q.Y * q.Z);
            double cosr_cosp = 1 - 2 * (q.X * q.X + q.Y * q.Y);
            angles.X = (float)Math.Atan2(sinr_cosp, cosr_cosp);

            // pitch / y
            double sinp = 2 * (q.W * q.Y - q.Z * q.X);
            if (Math.Abs(sinp) >= 1) {
                angles.Y = (float)Math.CopySign(Math.PI / 2, sinp);
            } else {
                angles.Y = (float)Math.Asin(sinp);
            }

            // yaw / z
            double siny_cosp = 2 * (q.W * q.Z + q.X * q.Y);
            double cosy_cosp = 1 - 2 * (q.Y * q.Y + q.Z * q.Z);
            angles.Z = (float)Math.Atan2(siny_cosp, cosy_cosp);

            return angles;
        }
    }
}
