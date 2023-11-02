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
                    System.Threading.Thread.Sleep(1000);
                    continue;
                }
                extrapolateIdleLoopBenchmark.Stop();
                System.Diagnostics.Stopwatch extrapolateIdleWorkBenchmark = new System.Diagnostics.Stopwatch();
                extrapolateIdleWorkBenchmark.Start();
                for (int i = 0; i < trackers.Length; i++) {
                    if (Program.debugShowCamerasPosition && Program.debugTrackerToBorrow == i) continue;
                    (Vector3 pos, Quaternion q) = trackers[i].GetEstimatedPosition();
                    //Quaternion q = trackers[i].rotation;
                    Program.oscClient.Send("/VMT/Room/Unity", i + 1, 1, 0f,
                                                        pos.X, pos.Z, pos.Y, //1f, 1.7f, 1f
                                                        -q.X, -q.Z, -q.Y, q.W); //idk, this works lol //XZYW 2.24
                    if (Program.debugSendTrackerOSC) {
                        Program.oscClientDebug.Send("/debug/final/position", i + 1,
                                               pos.X, pos.Z, pos.Y, //1f, 1.7f, 1f
                                               -q.X, -q.Z, -q.Y, q.W);
                    }
                }

                extrapolateIdleWorkBenchmark.Stop();
                Program.threadsWorkTime[1] = extrapolateIdleWorkBenchmark.Elapsed.TotalMilliseconds;
                extrapolateIdleLoopBenchmark.Start();

                System.Threading.Thread.Sleep(1000 / Program.interpolationTPS);

                extrapolateIdleLoopBenchmark.Stop();
                Program.threadsIdleTime[1] = extrapolateIdleLoopBenchmark.Elapsed.TotalMilliseconds;
            }
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
            newPred[0] = points[2].X + ((newPred[0] / mag) * d);
            newPred[1] = points[2].Y + ((newPred[1] / mag) * d);
            newPred[2] = points[2].Z + ((newPred[2] / mag) * d);
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
