using System;
using System.Collections.Generic;
using System.Linq;
using System.Numerics;
using System.Text;
using System.Threading.Tasks;

namespace TagTracking {
    static class Extrapolate {
        public class Tracker {
            List<Vector3> posList = new();
            List<Quaternion> rotList = new();
            double lastTimeNewPos = 0;
            public float timeOffset = 0;
            public int enableTracker = 1;
            public int index = -1;
            public Quaternion rotation = Quaternion.Identity;
            public string trackbase = "";
            public int inactiveSince = 0;
            Vector3 lastPosGiven = new();
            Quaternion lastRotGiven = Quaternion.Identity;
            public Tracker(Vector3 pos, Quaternion rot, int i) {
                UpdatePos(pos, rot, i);
            }

            public void UpdatePos(Vector3 pos, Quaternion rot, int i) {
                index = i;
                enableTracker = 1;
                timeOffset = 0;
                rotation = rot;
                posList.Add(pos);
                rotList.Add(rot);
                lastTimeNewPos = Program.timer.Elapsed.TotalMilliseconds;
                while (posList.Count() > 5) {
                    posList.RemoveAt(0);
                }
                while (rotList.Count() > 5) {
                    rotList.RemoveAt(0);
                }
            }
            public Vector3 GetLastPosition() {
                if (posList.Count() == 0)
                    return new Vector3();
                Vector3 last;
                try {
                    last = posList[posList.Count() - 1];
                } catch (Exception e) {
                    Console.WriteLine("Something went wrong in getLastPosition(): " + e);
                    return lastPosGiven;
                }
                return last;
            }
            public (Vector3, Quaternion) GetEstimatedPosition() {
                if (posList.Count() < 3) {
                    if (posList.Count() == 0)
                        return (new Vector3(), Quaternion.Identity);
                    Vector3 lastP = posList[posList.Count - 1];
                    Quaternion lastR = rotList[rotList.Count - 1];
                    return (lastP, lastR);
                }
                int size = posList.Count();
                Vector3 pm3, pm2, pm1;
                try {
                    pm3 = posList[size - 3];
                    pm2 = posList[size - 2];
                    pm1 = posList[size - 1];
                } catch (Exception e) {
                    Console.WriteLine("Something went wrong getting posList in getEstimatedPosition(): " + e);
                    return (lastPosGiven, lastRotGiven);
                }
                Quaternion qm3, qm2, qm1;
                try {
                    qm3 = rotList[size - 3];
                    qm2 = rotList[size - 2];
                    qm1 = rotList[size - 1];
                } catch (Exception e) {
                    Console.WriteLine("Something went wrong getting rotList in getEstimatedPosition(): " + e);
                    return (lastPosGiven, lastRotGiven);
                }

                float seekAhead = extrapolationRatio - 1f;
                double timeSinceNewPos = Program.timer.Elapsed.TotalMilliseconds - lastTimeNewPos;
                float seekPos = (float)(timeSinceNewPos / (1000f / Program.updateFPS) + seekAhead);
                seekPos = Math.Min(seekPos, 1.5f);
                Vector3 finalPred = CurveExtra(new Vector3[] { pm3, pm2, pm1 }, seekPos);
                lastPosGiven.X = finalPred.X;
                lastPosGiven.Y = finalPred.Y;
                lastPosGiven.Z = finalPred.Z;

                Quaternion interpQ = lastRotGiven;
                if (seekPos <= 1) {
                    interpQ = Quaternion.Slerp(qm2, qm1, seekPos + 1);
                } else if (seekPos > 1) {
                    Quaternion predict = Quaternion.Add(qm1, Quaternion.Subtract(qm2, qm1));
                    interpQ = Quaternion.Slerp(predict, qm1, seekPos);
                }
                lastRotGiven = interpQ;
                //}
                return (lastPosGiven, interpQ);
            }
        }

        static public Tracker[] trackers = new Tracker[4];
        static public float extrapolationRatio = 0.75f;
        static public bool interruptFlag = true;
        static public void UpdateLoop() {
            while (true) {
                System.Diagnostics.Stopwatch extrapolateIdleLoopBenchmark = new System.Diagnostics.Stopwatch();
                extrapolateIdleLoopBenchmark.Start();
                if (interruptFlag) {
                    System.Threading.Thread.Sleep(1000);
                    interruptFlag = false;
                }
                if (!Program.useInterpolation) {
                    System.Threading.Thread.Sleep(2000);
                    continue;
                }
                extrapolateIdleLoopBenchmark.Stop();
                System.Diagnostics.Stopwatch extrapolateIdleWorkBenchmark = new System.Diagnostics.Stopwatch();
                extrapolateIdleWorkBenchmark.Start();
                Program.GetDevices();
                for (int i = 0; i < trackers.Length; i++) {
                    if (Program.debugShowCamerasPosition && Program.debugTrackerToBorrow == i) continue;
                    (Vector3 pos, Quaternion q) = trackers[i].GetEstimatedPosition();
                    Matrix4x4 preSmooth = Matrix4x4.Multiply(Matrix4x4.CreateFromQuaternion(q), Matrix4x4.CreateTranslation(pos));
                    preSmooth = Tag.GetOffsetTracker(preSmooth, Tag.trackers[i].trackerFollowWeight, Tag.trackers[i].leftElbowtrackerFollowWeight, Tag.trackers[i].rightElbowtrackerFollowWeight);
                    pos = preSmooth.Translation;
                    q = Quaternion.CreateFromRotationMatrix(preSmooth);
                    //Quaternion q = trackers[i].rotation;
                    Tag.SendTracker(i, pos, q);
                }
                if (Program.useVRChatOSCTrackers) {
                    float[] headpos = Program.hmdPos;
                    float[] headrot = Program.hmdRot;
                    float m = 114.591559f / 2f;
                    if (Program.sendHeadPositionVRC)
                        Program.oscClient.Send($"/tracking/trackers/head/position", headpos[0], headpos[1], -headpos[2]);
                    if (Program.sendHeadRotationVRC)
                        Program.oscClient.Send($"/tracking/trackers/head/rotation", -headrot[0] * m, -headrot[2] * m, headrot[1] * m);
                }

                extrapolateIdleWorkBenchmark.Stop();
                Program.threadsWorkTime[1] = extrapolateIdleWorkBenchmark.Elapsed.TotalMilliseconds;
                extrapolateIdleLoopBenchmark.Start();


                bool ActiveCams = false;
                for (int i = 0; i < Tag.cameras.Length; i++) {
                    if (!Tag.cameras[i].inWaitMode) {
                        ActiveCams = true;
                        break;
                    }
                }
                int sleepTime = 1000 / Program.interpolationTPS;
                if (!ActiveCams) sleepTime *= 4;
                System.Threading.Thread.Sleep(sleepTime);

                extrapolateIdleLoopBenchmark.Stop();
                Program.threadsIdleTime[1] = extrapolateIdleLoopBenchmark.Elapsed.TotalMilliseconds;
            }
        }
        public static Vector3 ToEulerAngles(Quaternion q) {
            // Normalizar el quaternion
            q = Quaternion.Normalize(q);

            // Extraer los componentes del quaternion
            float w = q.W;
            float x = q.X;
            float y = q.Y;
            float z = q.Z;

            // Calcular los ángulos de Euler
            float pitch = MathF.Atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y));
            float yaw = MathF.Asin(2 * (w * y - z * x));
            float roll = MathF.Atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z));

            // Crear un vector de ángulos de Euler
            Vector3 eulerAngles = new Vector3(pitch, yaw, roll);

            return eulerAngles;
            //Vector3 angles = new();

            //// roll / x
            //double sinr_cosp = 2 * (q.W * q.X + q.Y * q.Z);
            //double cosr_cosp = 1 - 2 * (q.X * q.X + q.Y * q.Y);
            //angles.X = (float)Math.Atan2(sinr_cosp, cosr_cosp);

            //// pitch / y
            //double sinp = 2 * (q.W * q.Y - q.Z * q.X);
            //if (Math.Abs(sinp) >= 1) {
            //    angles.Y = (float)Math.CopySign(Math.PI / 2, sinp);
            //} else {
            //    angles.Y = (float)Math.Asin(sinp);
            //}

            //// yaw / z
            //double siny_cosp = 2 * (q.W * q.Z + q.X * q.Y);
            //double cosy_cosp = 1 - 2 * (q.Y * q.Y + q.Z * q.Z);
            //angles.Z = (float)Math.Atan2(siny_cosp, cosy_cosp);

            //return angles;
        }
        //THIS IS A MADE UP EXTRAPOLATION METHOD, DO NOT COPY, IT IS BAD LOL
        public static Vector3 CurveExtra(Vector3[] points, float dist) {
            float curviness = 0.7f;

            if (dist < 0) {
                float x = points[2].X - points[1].X;
                float y = points[2].Y - points[1].Y;
                float z = points[2].Z - points[1].Z;
                float revD = dist + 1;
                return new Vector3(
                  points[1].X + x * revD,
                  points[1].Y + y * revD,
                  points[1].Z + z * revD
                );
            }

            float[][] dX = new float[][] { new float[] { 1, points[1].X }, new float[] { 2, points[2].X } };
            float pX = LinearExtrapolate(dX, 3.0f);
            float[][] dY = new float[][] { new float[] { 1, points[1].Y }, new float[] { 2, points[2].Y } };
            float pY = LinearExtrapolate(dY, 3.0f);
            float[][] dZ = new float[][] { new float[] { 1, points[1].Z }, new float[] { 2, points[2].Z } };
            float pZ = LinearExtrapolate(dZ, 3.0f);
            Vector3 pr = new(pX, pY, pZ);

            float pM = Utils.GetDistance(pr, points[2]);
            float[] mid = new float[] {
                points[0].X * 0.9f + points[2].X * 0.1f,
                points[0].Y * 0.9f + points[2].Y * 0.1f,
                points[0].Z * 0.9f + points[2].Z * 0.1f
            };
            float curve = (1 + dist * curviness);
            mid[0] -= (mid[0] - points[1].X) * curve;
            mid[1] -= (mid[1] - points[1].Y) * curve;
            mid[2] -= (mid[2] - points[1].Z) * curve;
            float[] newPred = new float[] {
                points[2].X-mid[0],
                points[2].Y-mid[1],
                points[2].Z-mid[2]
            };
            float mag = GetMag(newPred[0], newPred[1], newPred[2]);
            float d = (pM * dist);
            if (newPred[0] == 0) newPred[0] = points[2].X;
            else newPred[0] = points[2].X + ((newPred[0] / mag) * d);
            if (newPred[1] == 0) newPred[1] = points[2].Y;
            else newPred[1] = points[2].Y + ((newPred[1] / mag) * d);
            if (newPred[2] == 0) newPred[2] = points[2].Z;
            else newPred[2] = points[2].Z + ((newPred[2] / mag) * d);
            return new Vector3(newPred[0], newPred[1], newPred[2]);
        }
        static float GetMag(float x, float y, float z) {
            //From Processing
            return (float)Math.Sqrt(x * x + y * y + z * z);
        }
        static float LinearExtrapolate(float[][] d, float x) {
            float y = d[0][1] + (x - 1) / (2 - 1) * (d[1][1] - d[0][1]);
            return y;
        }
    }
}
