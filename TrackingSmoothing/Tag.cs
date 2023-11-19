using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Numerics;
using System.IO;

namespace TagTracking {
    static partial class Tag {
        public class RecieveTag {
            public Matrix4x4 rot;
            public Vector3 pos;
            public int index;
            public int camera;
            public int altRot;
            public RecieveTag(Matrix4x4 rot, Vector3 pos, int index, int camera, int altRot) {
                this.rot = rot;
                this.pos = pos;
                this.index = index;
                this.camera = camera;
                this.altRot = altRot;
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
            public int rsWidth = 0;
            public int rsHeight = 0;
            public int index = 0;
            public string url = "";
            public bool useCustomDistortion = false;
            public bool newData = false;
            public float brightness = 1f;
            public int skipFrames = 0;
            public int skipFrameCount = 0;
            public float xRatio = 1f;
            public bool canEnterInWaitMode = true;
            public bool inWaitMode = false;
            public long lastSeenMarkerTime = 0;
            public int timeBeforeWaiting = 20000;
            public int waitTimeUpdate = 500;
            public long lastWaitCheck = 0;
            public float yRatio = 1f;
            public float[] customDist = new float[] {
                    1.075f, 1f, 1.075f,
                    1.025f, 0.975f, 1.025f,
                    1.075f, 1f, 1.075f
                };
            internal bool adjustCurrentDistortion = true;

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
                new Vector3[] {
                    new(0f,                     0f, 0f),
                    new((float)Math.PI / 2,     0f, 0f),
                    new((float)Math.PI,         0f, 0f),
                    new((float)Math.PI * 1.5f,  0f, 0f)
                }
                ),
            new ClusterTracker("leftfoot",
                new int[] { 4, 5, 6, 7 },
                new Vector3 [] {
                    new Vector3 (0f, 0f, -0.055f),
                    new Vector3 (0f, 0f, -0.055f),
                    new Vector3 (0f, 0f, -0.055f),
                    new Vector3 (0f, 0f, -0.055f)
                },
                new Vector3[] {
                    new(0f,                     0f, 0f),
                    new((float)Math.PI / 2,     0f, 0f),
                    new((float)Math.PI,         0f, 0f),
                    new((float)Math.PI * 1.5f,  0f, 0f)
                }
                ),
            new ClusterTracker("waist",
                new int[] { 8, 9, 10, 11 },
                new Vector3 [] {
                    new Vector3 (0.0f, 0.00f, -0.17f),
                    new Vector3 (0.0f, 0.00f, -0.12f),
                    new Vector3 (-0.0f, -0.00f, -0.12f),
                    new Vector3 (-0.0f, -0.00f, -0.17f)
                },
                new Vector3[] {
                    new(0f,                     0f, 0f),
                    new((float)Math.PI / 2,     0f, 0f),
                    new((float)Math.PI,         0f, 0f),
                    new((float)Math.PI * 1.5f,  0f, 0f)
                }
                )
        };
        public static CombinedTracker[] rawTrackers = new CombinedTracker[] {
            new(0), new(1), new(2), new(3), new(4), new(5), new(6), new(7), new(8), new(9), new(10), new(11), new(12), new(13), new(14), new(15)
        };
        public static List<CombinedTracker> combinedTrackers = new();
        public static int[] tagToCalibrate = new int[] { 0, 2, 4, 6 };
        public static int[] tagsOnFloor = new int[] { 0, 2, 4, 6 };
        public static float[] tagToCalibrateWeight = new float[] { 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1 };
        public static FinalTracker[] finals;
        public static bool newInfoReady = false;
        public static bool newInfo = false;

        public static double[] lastFrameTime;
        public static int lastCamera = 0;
        public static int lastIndex = 0;
        public static double[] cameraTPS = new double[2];


        public static bool dynamicFiltering = true;
        public static void SaveMatrix() {
            Console.WriteLine("Saving...");
            for (int i = 0; i < cameras.Length; i++) {
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
            for (int i = 0; i < cameras.Length; i++) {
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
            List<Vector3> rots = new List<Vector3>();
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

                            float val;
                            float.TryParse(split2[1], System.Globalization.NumberStyles.Any, System.Globalization.CultureInfo.InvariantCulture, out val);
                            if (split2[0].Equals("avgSmoothDistTrigger")) currentTracker.avgSmoothDistTrigger = val;
                            else if (split2[0].Equals("avgSmoothVal")) currentTracker.avgSmoothVal = val;
                            else if (split2[0].Equals("avgSmoothRecoverVal")) currentTracker.avgSmoothRecoverVal = val;
                            else if (split2[0].Equals("avgSmoothAlwaysVal")) currentTracker.avgSmoothAlwaysVal = val;
                            else if (split2[0].Equals("maxSpikePosDist")) currentTracker.maxSpikePosDist = val;
                            else if (split2[0].Equals("maxSpikeRotDiff")) currentTracker.maxSpikeRotDiff = val;
                            else if (split2[0].Equals("rotationComparison")) currentTracker.rotationComparison = val;
                            else if (split2[0].Equals("straightTrackerWeight")) currentTracker.straightTrackerWeight = val;
                            else if (split2[0].Equals("filterSmoothRot")) currentTracker.smoothedRot = val;
                            else if (split2[0].Equals("filterSmoothPos")) currentTracker.smoothedPos = val;
                            else if (split2[0].Equals("filterSmoothPerTrackerRot")) currentTracker.UpdatePerTrackerFilterRot(val);
                            else if (split2[0].Equals("filterSmoothPerTrackerDepth")) currentTracker.UpdatePerTrackerFilterDepth(val);
                            else if (split2[0].Equals("trackerFollowWeight")) currentTracker.trackerFollowWeight = val;
                            else if (split2[0].Equals("trackerIdleTime")) currentTracker.trackerDisableMax = (long)val;
                            else if (split2[0].Equals("leftElbowtrackerFollowWeight"))
                                currentTracker.leftElbowtrackerFollowWeight = val;
                            else if (split2[0].Equals("rightElbowtrackerFollowWeight"))
                                currentTracker.rightElbowtrackerFollowWeight = val;
                            else if (split2[0].Equals("reduceOffset"))
                                currentTracker.reduceOffset = val;
                            else if (split2[0].Equals("generatedByCalibration")) currentTracker.generatedByCalibration = split2[1].Equals("true");

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
                if (split.Length != 5 && split.Length != 7) {
                    Console.WriteLine($"Incorrect tracker: line {(i - 1)}");
                    continue;
                }
                if (split.Length == 5) {
                    Console.WriteLine($"Please use 7 parameters for tracker: line {(i - 1)}");
                }
                indexes.Add(int.Parse(split[0]));
                Vector3 rot = new Vector3();
                rot.X = float.Parse(split[1], System.Globalization.NumberStyles.Any, System.Globalization.CultureInfo.InvariantCulture);
                if (split.Length == 7) {
                    rot.Y = float.Parse(split[2], System.Globalization.NumberStyles.Any, System.Globalization.CultureInfo.InvariantCulture);
                    rot.Z = float.Parse(split[3], System.Globalization.NumberStyles.Any, System.Globalization.CultureInfo.InvariantCulture);
                }
                int newI = split.Length == 7 ? 2 : 0;
                rots.Add(rot);
                Vector3 offset = new Vector3();
                offset.X = float.Parse(split[2 + newI], System.Globalization.NumberStyles.Any, System.Globalization.CultureInfo.InvariantCulture);
                offset.Y = float.Parse(split[3 + newI], System.Globalization.NumberStyles.Any, System.Globalization.CultureInfo.InvariantCulture);
                offset.Z = float.Parse(split[4 + newI], System.Globalization.NumberStyles.Any, System.Globalization.CultureInfo.InvariantCulture);
                offsets.Add(offset);
            }
            if (currentTracker != null) {
                trackerss.Add(currentTracker);
            }
            trackers = trackerss.ToArray();
            //Extrapolation
            if (Program.useInterpolation) {
                Extrapolate.interruptFlag = true;
                System.Threading.Thread.Sleep(100);
                Extrapolate.trackers = new Extrapolate.Tracker[trackers.Length];
                for (int j = 0; j < trackers.Length; j++) {
                    Extrapolate.trackers[j] = new Extrapolate.Tracker(new Vector3(), Quaternion.Identity, j);
                }
            }
            SetFinalTrackers(Program.postNoise == 2 ? 0.5f : 1f);

            combinedTrackers = new();
            rawTrackers = new CombinedTracker[36];
            for (int j = 0; j < rawTrackers.Length; j++) {
                rawTrackers[j] = new(j);
            }
        }
        public static void SetFinalTrackers(float mult = 1f) {
            finals = new FinalTracker[trackers.Length];
            for (int j = 0; j < trackers.Length; j++) {
                finals[j] = new(new(), Quaternion.Identity, Quaternion.Identity, trackers[j].trackerName);
                UpdateFinalTrackerParams(j, mult);
            }
        }
        public static void UpdateFinalTrackerParams(int id, float mult = 1f) {
            finals[id].avgSmoothAlwaysVal = (float)Math.Pow(trackers[id].avgSmoothAlwaysVal, mult);
            finals[id].avgSmoothVal = (float)Math.Pow(trackers[id].avgSmoothVal, mult);
            finals[id].avgSmoothRecoverVal = (float)Math.Pow(trackers[id].avgSmoothRecoverVal, mult);
            finals[id].avgSmoothDistTrigger = trackers[id].avgSmoothDistTrigger * mult;
            finals[id].smoothedRot.UpdateParams(trackers[id].smoothedRot * mult);
            finals[id].smoothedPos.UpdateParams(trackers[id].smoothedPos * mult);
            finals[id].maxSpikePosDist = trackers[id].maxSpikePosDist;
            finals[id].maxSpikeRotDiff = trackers[id].maxSpikeRotDiff;
        }
        public static List<RecieveTag> tagsList = new List<RecieveTag>();
        public static void RecieveTrackerAsync(int index, int camera, Matrix4x4 rot, Vector3 pos, int altRot) {
            tagsList.Add(new RecieveTag(rot, pos, index, camera, altRot));
        }
        public static void Update() {
            if (!newInfoReady) return;
            List<RecieveTag> tagsListCopy = tagsList;
            tagsList = new List<RecieveTag>();
            newInfoReady = false;
            if (Program.debugSendTrackerOSC)
                Program.oscClientDebug.Send($"/debug/tick", 0);
            for (int i = 0; i < tagsListCopy.Count; i++) {
                RecieveTag tag = tagsListCopy[i];
                if (tag != null) {
                    RecieveTracker(tag.index, tag.camera, tag.rot, tag.pos, tag.altRot);
                }
            }
            if (TrackerCalibrate.startCalibrating) {
                TrackerCalibrate.GetTrackerEnd();
            }
            if (Program.debugSendTrackerOSC) {
                if (Program.timer.ElapsedMilliseconds - RoomCalibrate.saveMatTime < 15000) {
                    SendSingleTrackersOSC();
                } else {
                    for (int i = 0; i < rawTrackers.Length; i++) {
                        CombinedTracker tracker = rawTrackers[i];
                        Matrix4x4[] poss = tracker.Obtain();
                        for (int j = 0; j < poss.Length; j++) {
                            if (cameras[j].newData)
                                rawTrackers[i].updateCount[j]++;
                            if (rawTrackers[i].updateCount[j] > 2) continue;
                            Program.oscClientDebug.Send($"/debug/trackers/position", i, j, poss[j].Translation.X, poss[j].Translation.Z, poss[j].Translation.Y);
                        }
                    }
                }
            }
            newInfo = true;

            RoomCalibrate.GetCalibrationTags();
        }
        public static void SendSingleTrackersOSC() {
            if (!Program.debugSendTrackerOSC) {
                return;
            }
            for (int i = 0; i < combinedTrackers.Count; i++) {
                CombinedTracker tracker = combinedTrackers[i];
                Matrix4x4[] poss = tracker.Obtain();
                for (int j = 0; j < poss.Length; j++) {
                    Program.oscClientDebug.Send($"/debug/trackers/position", i, j, poss[j].Translation.X, poss[j].Translation.Z, poss[j].Translation.Y);
                }
            }
        }

        public static void RecieveTracker(int index, int camera, Matrix4x4 rot, Vector3 pos, int altRot) {
            //if (!positionObtained) GetCameraPosition();

            //if (cameras[camera].useCustomDistortion || true) {
            //    Vector3 normalizedPos = pos / pos.Z;
            //    normalizedPos.Y = -normalizedPos.Y;
            //    //Console.WriteLine($"{normalizedPos.X}\t{normalizedPos.Y}\t{normalizedPos.Z}");
            //    float[] distortion;
            //    if (camera == 0) {
            //        distortion = new float[] {
            //            0.9990f, 0.9700f, 1.0250f,
            //            1.0000f, 0.9200f, 1.0030f,
            //            1.0070f, 0.9500f, 1.0300f
            //        };
            //    } else {
            //        distortion = new float[] {
            //            1.0600f, 0.9700f, 1.0470f,
            //            1.0300f, 0.9300f, 1.0150f,
            //            1.0800f, 1.0010f, 1.0600f
            //        };
            //    }
            //    bool bot = normalizedPos.Y < 0;
            //    bool lft = normalizedPos.X < 0;
            //    int yT = bot ? 3 : 0;
            //    int yB = bot ? 6 : 3;
            //    float y = bot ? normalizedPos.Y + 0.3f : normalizedPos.Y;
            //    float y1 = Utils.GetMap(y, 0f, 0.3f, distortion[lft ? yB : yB + 1], distortion[lft ? yT : yT + 1]);
            //    float y2 = Utils.GetMap(y, 0f, 0.3f, distortion[lft ? yB + 1 : yB + 2], distortion[lft ? yT + 1 : yT + 2]);
            //    float x = lft ? normalizedPos.X + 0.4f : normalizedPos.X;
            //    float final = Utils.GetMap(x, 0f, 0.4f, y1, y2);
            //    //Console.WriteLine(final);
            //    //Console.WriteLine($"pre\t{pos.X}\t{pos.Y}\t{pos.Z}");
            //    pos *= (final-1) * -0.5f + 1;
            //    //Console.WriteLine($"post\t{pos.X}\t{pos.Y}\t{pos.Z}");
            //    //Console.WriteLine();
            //}
            if (TrackerCalibrate.startCalibrating) {
                if (altRot == -1 && camera == TrackerCalibrate.cameraToUse) {
                    TrackerCalibrate.GetTracker(index, pos, rot);
                }
                return;
            }

            if (camera != lastCamera || (camera == lastCamera && lastIndex >= index)) {
                double tps = 1000.0 / (Program.timer.Elapsed.TotalMilliseconds - lastFrameTime[camera]);
                if (tps < 100.0)
                    cameraTPS[camera] += (tps - cameraTPS[camera]) * 0.5f;
                lastFrameTime[camera] = Program.timer.Elapsed.TotalMilliseconds;
            }
            lastCamera = camera;
            lastIndex = index;
            if (altRot == -1) {
                for (int k = 0; k < rawTrackers.Length; k++) {
                    if (rawTrackers[k].index == index) {
                        rawTrackers[k].Recieve(camera, pos, rot, -1);
                        break;
                    }
                }
            }
            RoomCalibrate.RevieveTrackers(index, camera, rot, pos, altRot);
            for (int i = 0; i < trackers.Length; i++) {
                for (int j = 0; j < trackers[i].trackerIndex.Length; j++) {
                    if (trackers[i].trackerIndex[j] == index) {
                        trackers[i].trackers[j].Recieve(camera, pos, rot, altRot);
                        //trackers[i].updateCount[j] = 0;
                        break;
                    }
                }
            }
        }



        public static void GetTrackers() {
            if (finals == null) return;
            Vector4 headPos = Program.hmdList[0];
            for (int i = 0; i < trackers.Length; i++) {
                Matrix4x4 mat = trackers[i].Obtain();
                if (!trackers[i].trackerNotSeen) {
                    finals[i].lastTimeSeen = Program.timer.ElapsedMilliseconds;
                }
                finals[i].preMat = mat;

                Vector3 pos = mat.Translation;
                Quaternion q = mat.Rotation();
                finals[i].pos = pos;
                finals[i].prot = finals[i].rot;
                finals[i].rot = q;

                //if (i == 0) {
                //    finals[i].pos = new Vector3(-0.06240213f, -0.11492123f, 0.09790712f);
                //    finals[i].rot = new Quaternion(0.53902847f, 0.43966487f, 0.5416175f, 0.47201034f);
                //} else if (i == 1) {
                //    finals[i].pos = new Vector3(-0.3197458f, -0.09293179f, 0.06688197f);
                //    finals[i].rot = new Quaternion(0.45106563f, 0.4960238f, 0.6241316f, 0.4011982f);
                //} else if (i == 2) {
                //    Random rnd = new Random();
                //    finals[i].pos = new Vector3(-0.27016774f, -0.05897135f, 0.9725861f);
                //    finals[i].rot = new Quaternion(0.46770912f, 0.441457f, 0.526209f, 0.55629855f);
                //}
                ////finals[i].pos.Y += (float)Math.Sin(Program.timer.ElapsedMilliseconds / 1000f) / 5f;
                //mat = Matrix4x4.Multiply(Matrix4x4.CreateFromQuaternion(finals[i].rot), Matrix4x4.CreateTranslation(finals[i].pos));
                //finals[i].preMat = mat;
                //mat = Matrix4x4.Multiply(mat, Program.offsetMat);
                //pos = mat.Translation;
                //q = mat.Rotation();
                //finals[i].pos = pos;
                //finals[i].prot = finals[i].rot;
                //finals[i].rot = q;
            }
            for (int i = 0; i < cameras.Length; i++) {
                cameras[i].newData = false;
            }
            if (Program.timer.Elapsed.TotalSeconds > Program.nextSave) {
                //Console.WriteLine(">> new tick: " + Program.nextSave);
                //for (int i = 0; i < trackers.Length; i++) {
                //    finals[i].Update();
                //    Console.WriteLine("t: " + i);
                //    Console.WriteLine(finals[i].fpos);
                //    Console.WriteLine(finals[i].frot);
                //}
                //Console.WriteLine();
                Program.nextSave += 5;
            }
            if (Program.poseAdjust && Program.postNoise != 0)
                AdjustPose();
            for (int i = 0; i < trackers.Length; i++) {
                finals[i].Update();

                Matrix4x4 mat = Matrix4x4.Multiply(Matrix4x4.CreateFromQuaternion(finals[i].frot), Matrix4x4.CreateTranslation(finals[i].fpos));
                Matrix4x4 preSmooth = mat;
                if (!Program.useInterpolation)
                    preSmooth = GetOffsetTracker(preSmooth, trackers[i].trackerFollowWeight, trackers[i].leftElbowtrackerFollowWeight, trackers[i].rightElbowtrackerFollowWeight);
                Vector3 pos = preSmooth.Translation;
                Quaternion q = preSmooth.Rotation();
                finals[i].fpos = pos;
                finals[i].frot = q;
                //mat.M41 += (headPos.X - Program.hmdPos[0]) * trackers[i].trackerFollowWeight;
                //mat.M43 += (headPos.Y - Program.hmdPos[1]) * trackers[i].trackerFollowWeight;
                //mat.M42 -= (headPos.Z - Program.hmdPos[2]) * trackers[i].trackerFollowWeight;
                Aruco.DrawCube(mat);
                //Program.oscClient.Send("/VMT/Room/Unity", i + 1, 1, 0f,
                //                            pos.X, pos.Z, pos.Y, //1f, 1.7f, 1f
                //                            -q.X, -q.Z, -q.Y, q.W); //idk, this works lol //XZYW 2.24
            }
            newInfo = false;
        }

        public static Matrix4x4 GetOffsetTracker(Matrix4x4 mat, float headWeight, float leftHandWeight = 1, float rightHandWeight = 1) {
            Vector4 headPos = Program.hmdList[0];
            mat = Matrix4x4.Multiply(mat, Program.offsetMat);
            mat.M41 -= (headPos.X - Program.hmdPos[0]) * headWeight;
            mat.M43 -= (headPos.Y - Program.hmdPos[1]) * headWeight;
            mat.M42 += (headPos.Z - Program.hmdPos[2]) * headWeight;

            if (leftHandWeight != 0f) {
                Vector4 delayElbow = (Program.rightList[0] + Program.shoulderList[0]) / 2f;
                Vector3 currElbow = (Program.rightHandPos + Program.shoulderCenterPos) / 2f;
                mat.M41 -= (delayElbow.X - currElbow.X) * leftHandWeight;
                mat.M43 -= (delayElbow.Y - currElbow.Y) * leftHandWeight;
                mat.M42 += (delayElbow.Z - currElbow.Z) * leftHandWeight;
            }
            if (rightHandWeight != 0f) {
                Vector4 delayElbow = (Program.leftList[0] + Program.shoulderList[0]) / 2f;
                Vector3 currElbow = (Program.leftHandPos + Program.shoulderCenterPos) / 2f;
                mat.M41 -= (delayElbow.X - currElbow.X) * rightHandWeight;
                mat.M43 -= (delayElbow.Y - currElbow.Y) * rightHandWeight;
                mat.M42 += (delayElbow.Z - currElbow.Z) * rightHandWeight;
            }

            return mat;
        }

        public static void SendTrackers() {
            float noiseRed = Program.postNoise == 2 ? 0.5f : 1f;
            for (int i = 0; i < finals.Length; i++) {
                float score = 1f - finals[i].velScore;
                score = (score * 0.7f) + 0.3f;
                score *= noiseRed;
                if (dynamicFiltering)
                    UpdateFinalTrackerParams(i, score);
                if (Program.debugShowCamerasPosition && Program.debugTrackerToBorrow == i) continue;
                Vector3 pos = finals[i].fpos;
                Quaternion q = finals[i].frot;
                if (Program.useInterpolation) {
                    Extrapolate.trackers[i].UpdatePos(pos, q, i);
                } else {
                    SendTracker(i, pos, q);
                }
            }
            if (!Program.useVRChatOSCTrackers) {
                if (Program.debugShowCamerasPosition) {
                    int t = Program.debugTrackerToBorrow;
                    int c = (int)(Program.timer.ElapsedMilliseconds / 125) % cameras.Length;
                    Matrix4x4 nullPos = cameras[c].matrix;
                    nullPos = Matrix4x4.Multiply(nullPos, Program.offsetMat);
                    Vector3 pos = nullPos.Translation;
                    Quaternion q = Quaternion.Identity;
                    Program.oscClient.Send("/VMT/Room/Unity", t + 1, 1, 0f,
                                                    pos.X, pos.Z, pos.Y, //1f, 1.7f, 1f
                                                    -q.X, -q.Z, -q.Y, q.W); //idk, this works lol //XZYW 2.24
                    if (Program.debugSendTrackerOSC) {
                        Program.oscClientDebug.Send("/debug/final/position", t + 1,
                                               pos.X, pos.Z, pos.Y, //1f, 1.7f, 1f
                                               -q.X, -q.Z, -q.Y, q.W);
                    }
                }
            }
            //i didnt implement vrcosc correctly bc it doesnt use the rotation and idk why
            if (Program.useVRChatOSCTrackers && !Program.useInterpolation) {
                float[] headpos = Program.hmdPos;
                float[] headrot = Program.hmdRot;
                float m = 114.591559f / 2f;
                if (Program.sendHeadPositionVRC)
                    Program.oscClient.Send($"/tracking/trackers/head/position", headpos[0], headpos[1], -headpos[2]);
                if (Program.sendHeadRotationVRC)
                    Program.oscClient.Send($"/tracking/trackers/head/rotation", -headrot[0] * m, -headrot[2] * m, headrot[1] * m);
            }
        }

        public static void SendTracker(int i, Vector3 pos, Quaternion q) {
            bool disable = Program.timer.ElapsedMilliseconds - finals[i].lastTimeSeen > trackers[i].trackerDisableMax;
            if (Program.useVRChatOSCTrackers) {
                Program.oscClient.Send($"/tracking/trackers/{i + 1}/position", pos.X, pos.Z, pos.Y);
                Vector3 e = Utils.ToEulerAngles(q);
                Program.oscClient.Send($"/tracking/trackers/{i + 1}/rotation", e.X, e.Z, e.Y);
            } else {
                Program.oscClient.Send("/VMT/Room/Unity", i + 1, disable ? 0 : 1, 0f,
                                            pos.X, pos.Z, pos.Y, //1f, 1.7f, 1f
                                            -q.X, -q.Z, -q.Y, q.W); //idk, this works lol //XZYW 2.24
                if (Program.debugSendTrackerOSC) {
                    Program.oscClientDebug.Send("/debug/final/position", i + 1,
                                           pos.X, pos.Z, pos.Y, //1f, 1.7f, 1f
                                           -q.X, -q.Z, -q.Y, q.W);
                }
            }
        }

        public static float backwardsAngle = 0.6f;
        public static float panAngleR = 0.5f;
        public static float panAngleL = -0.5f;
        public static void AdjustPose() {
            int waist = -1;
            for (int i = 0; i < finals.Length; i++) {
                if (finals[i].name.Equals(Program.poseAdjustWaist)) {
                    waist = i;
                    break;
                }
            }
            List<int> adjustablesFoot = new();
            for (int i = 0; i < finals.Length; i++) {
                if (finals[i].name.Contains("foot")) {
                    adjustablesFoot.Add(i);
                }
            }
            List<int> adjustablesElbow = new();
            for (int i = 0; i < finals.Length; i++) {
                if (finals[i].name.Contains("arm")) {
                    adjustablesElbow.Add(i);
                }
            }

            Matrix4x4 waistMat = Matrix4x4.Multiply(Matrix4x4.CreateFromQuaternion(finals[waist].rot), Matrix4x4.CreateTranslation(finals[waist].pos));
            Matrix4x4 waistInv;
            Matrix4x4.Invert(waistMat, out waistInv);


            //how tf do i use quaternions
            waistMat = Matrix4x4.Multiply(waistMat, waistInv);
            int sendCount = 0;
            if (Program.debugSendTrackerOSC) {
                Program.oscClientDebug.Send($"/debug/adjust/position", sendCount, 2, waistMat.Translation.X, waistMat.Translation.Z, waistMat.Translation.Y);
                sendCount++;
            }
            float legDist = Program.poseAdjustLegDist;
            //adjust feet
            for (int i = 0; i < adjustablesFoot.Count; i++) {
                bool isIncorrect = false;
                float lol = Program.timer.ElapsedMilliseconds / 1000f;
                Matrix4x4 adjmp, adjmr;

                //get pos
                adjmp = Matrix4x4.Multiply(Matrix4x4.CreateTranslation(finals[adjustablesFoot[i]].pos), waistInv);


                //Console.WriteLine(adjmp.Translation.Y - waistMat.Translation.Y);
                float yDist = adjmp.Translation.Y - waistMat.Translation.Y;
                //finals[adjustables[i]].pos = adjmp.Translation;
                if (yDist > -legDist * 0.5f) {
                    if (Program.debugSendTrackerOSC) {
                        sendCount += 2;
                        Program.oscClientDebug.Send($"/debug/adjust/position", sendCount, 0, adjmp.Translation.X, adjmp.Translation.Z, adjmp.Translation.Y);
                        sendCount++;
                    }
                    continue; //dont adjust
                }

                //get rot
                adjmr = Matrix4x4.Multiply(Matrix4x4.CreateFromQuaternion(finals[adjustablesFoot[i]].rot), waistInv);
                //finals[adjustables[i]].rot = adjmr.Rotation();

                //backwards rotation fix
                Matrix4x4 mat = Matrix4x4.Multiply(Matrix4x4.CreateFromQuaternion(adjmr.Rotation()), Matrix4x4.CreateTranslation(adjmp.Translation));
                float brfX = (float)(legDist * Math.Sin(backwardsAngle));
                float brfY = (float)(legDist * Math.Cos(backwardsAngle));
                mat = Matrix4x4.Multiply(Matrix4x4.CreateTranslation(new Vector3(brfX, brfY, 0)), mat);

                if (Program.debugSendTrackerOSC) {
                    Program.oscClientDebug.Send($"/debug/adjust/position", sendCount, 1, mat.Translation.X, mat.Translation.Z, mat.Translation.Y);
                    sendCount++;
                }

                //finals[1].pos = mat.Translation;
                //finals[1].pos.X += (float)Math.Sin(lol) / 5f;

                if (mat.Translation.X < adjmp.Translation.X * 0.4f)
                    isIncorrect = true;

                //vertical rotation fix
                mat = Matrix4x4.Multiply(Matrix4x4.CreateFromQuaternion(adjmr.Rotation()), Matrix4x4.CreateTranslation(adjmp.Translation));
                float angleDir = 0;
                if (finals[adjustablesFoot[i]].name.Contains("right")) {
                    angleDir = panAngleR;
                } else if (finals[adjustablesFoot[i]].name.Contains("left")) {
                    angleDir = panAngleL;
                }
                float vrfX = (float)(legDist * Math.Sin(angleDir));
                float vrfY = (float)(legDist * Math.Cos(angleDir));
                mat = Matrix4x4.Multiply(Matrix4x4.CreateTranslation(new Vector3(vrfY, 0, vrfX)), mat);

                if (Program.debugSendTrackerOSC) {
                    Program.oscClientDebug.Send($"/debug/adjust/position", sendCount, 1, mat.Translation.X, mat.Translation.Z, mat.Translation.Y);
                    sendCount++;
                }

                //finals[1].pos = mat.Translation;

                if (mat.Translation.X < (adjmp.Translation.X - legDist / 1.75f))
                    isIncorrect = true;

                //Console.WriteLine(isIncorrect);
                if (isIncorrect) {
                    finals[adjustablesFoot[i]].rot = finals[adjustablesFoot[i]].prot;
                    if (Program.debugSendTrackerOSC) {
                        Program.oscClientDebug.Send($"/debug/adjust/position", sendCount, 3, adjmp.Translation.X, adjmp.Translation.Z, adjmp.Translation.Y);
                        sendCount++;
                    }
                } else {
                    if (Program.debugSendTrackerOSC) {
                        Program.oscClientDebug.Send($"/debug/adjust/position", sendCount, 0, adjmp.Translation.X, adjmp.Translation.Z, adjmp.Translation.Y);
                        sendCount++;
                    }
                }
            }
            waistMat = Matrix4x4.Multiply(Matrix4x4.CreateFromQuaternion(finals[waist].rot), Matrix4x4.CreateTranslation(finals[waist].pos));

            //this should adjust the rotation of the shoulders, which is this experimental, because shoulder will lose tracking a lot
            //so implementing somthing like skeleton with ik, should do the trick at predicting where it should be when its lost
            ////but thats a pain in the ass
            for (int i = 0; i < adjustablesElbow.Count; i++) {
                //Vector3 shoulderCenterPos = new Vector3(finals[adjustablesElbow[i]].pos.X, finals[adjustablesElbow[i]].pos.Z, -finals[adjustablesElbow[i]].pos.Y);
                //Vector3 shoulderCenterPos = new Vector3(finals[adjustablesElbow[i]].pos.X, finals[adjustablesElbow[i]].pos.Z, -finals[adjustablesElbow[i]].pos.Y);
                //Matrix4x4 look = Matrix4x4.CreateLookAt(Program.shoulderCenterPos, shoulderCenterPos, new Vector3(0, 0, 1));
                //finals[adjustablesElbow[i]].rot = Quaternion.CreateFromRotationMatrix(look);
                Matrix4x4 shoulderCenterMatRot = Matrix4x4.CreateFromQuaternion(Quaternion.CreateFromRotationMatrix(waistMat));
                Matrix4x4.Invert(shoulderCenterMatRot, out Matrix4x4 shoulderCenterInv);
                Matrix4x4 shoulderCenterMatPos = Matrix4x4.Identity;
                shoulderCenterMatPos.M41 = Program.shoulderCenterPos.X;
                shoulderCenterMatPos.M42 = -Program.shoulderCenterPos.Z;
                shoulderCenterMatPos.M43 = Program.shoulderCenterPos.Y;
                Matrix4x4 centeredElbow = Matrix4x4.CreateTranslation(finals[adjustablesElbow[i]].pos - shoulderCenterMatPos.Translation);
                Matrix4x4 normalizedElbow = Matrix4x4.Multiply(centeredElbow, shoulderCenterInv);
                normalizedElbow.M43 -= normalizedElbow.M43 > 0 ? 0.12f : -0.12f;
                Matrix4x4 os = normalizedElbow;
                float angVer = os.Translation.Y;
                float angHor = (float)Math.Atan2(os.Translation.X, os.Translation.Z);
                //-0.35     0
                finals[adjustablesElbow[i]].rot = Quaternion.Lerp(finals[adjustablesElbow[i]].rot, Quaternion.Multiply(
                    Quaternion.CreateFromAxisAngle(new Vector3(0, 0, 1), angHor),
                    Quaternion.CreateFromAxisAngle(new Vector3(0, 1, 0), angVer * -4)),
                    0.5f);
                //finals[adjustablesElbow[i]].pos = os.Translation;
            }

            //for (int i = 0; i < finals.Length; i++) {
            //    Matrix4x4 mat = Matrix4x4.Multiply(Matrix4x4.CreateFromQuaternion(finals[i].rot), Matrix4x4.CreateTranslation(finals[i].pos));
            //    Aruco.DrawAxis(mat);
            //}
            //System.Threading.Thread.Sleep(50);
        }

    }
}
