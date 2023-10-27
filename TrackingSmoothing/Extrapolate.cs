using System;
using System.Collections.Generic;
using System.Linq;
using System.Numerics;
using System.Text;
using System.Threading.Tasks;

namespace TrackingSmoothing {
    static class Extrapolate {
        public class Tracker {
            static float velRangeStart = 0.01f;
            static float velRangeEnd = 0.1f;
            static float velMapStart = -1f;
            static float velMapEnd = -0.25f;

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
            float speedometer = -1;
            public Tracker(Vector3 pos, Quaternion rot, int i) {
                UpdatePos(pos, rot, i);
            }

            public void UpdatePos(Vector3 pos, Quaternion rot, int i) {
                index = i;
                enableTracker = 1;
                timeOffset = 0;
                //if (legsFolded) {
                //    for (int i = 0; i < 4; i++) {
                //        rotation[i] += (newRot[i] - rotation[i]) * 0.1f;
                //    }
                //} else {
                //    rotation = newRot;
                //}
                rotation = rot;
                //if (trackingBase.equals("/VMT/Follow/Unity"))
                //    trackbase = (String)info[10];
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
                    Vector3 last = posList[posList.Count() - 1];
                    Quaternion lastR = rotList[rotList.Count() - 1];
                    return (last, lastR);
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
                //if (index == 0)
                //  println(pm1.y);
                if (index == 0) {
                    if (pm1.Y > -0.65) {
                        //println("SEATEDaaaaaaaaaaaaaaaaaa");
                        //waistIsBelowThreshold = true;
                    } else {
                        //waistIsBelowThreshold = false;
                    }
                }
                float velocity1 = Utils.GetDistance(pm1.X, pm1.Y, pm1.Z, pm2.X, pm2.Y, pm2.Z);
                float velocity2 = Utils.GetDistance(pm2.X, pm2.Y, pm2.Z, pm3.X, pm3.Y, pm3.Z);
                //float velocity3 = dist(pm1.X, pm1.Y, pm1.Z, pm3.X, pm3.Y, pm3.Z);
                //if (legsFolded) {
                //    pm1.X = (pm1.X + pm2.X) * 0.5f;
                //    pm1.Y = (pm1.Y + pm2.Y) * 0.5f;
                //    pm1.Z = (pm1.Z + pm2.Z) * 0.5f;

                //    pm2.X = (pm3.X + pm2.X) * 0.5f;
                //    pm2.Y = (pm3.Y + pm2.Y) * 0.5f;
                //    pm2.Z = (pm3.Z + pm2.Z) * 0.5f;
                //}
                //float acc = -1;
                //if (velocity1 - velocity2 > velRangeStart && velocity2 > velRangeStart) {
                //    //println(velocity1 - velocity2, velocity1, velocity2);
                //    acc = Utils.GetMap(velocity1 - velocity2, velRangeStart, velRangeEnd, velMapStart, velMapEnd);
                //    if (acc > speedometer)
                //        speedometer = acc;
                //    //triggeredVel = true;          THIS NEEDS TO BE REVIEWED -----------------------------------------------------------
                //}
                ////println(speedometer);
                //speedometer -= 0.005f;
                //speedometer = Math.Min(Math.Max(speedometer, velMapStart), velMapEnd);
                //if (legsFolded) {
                //    speedometer = velMapStart;
                //}
                float seekAhead = extrapolationRatio - 1f;
                double timeSinceNewPos = Program.timer.Elapsed.TotalMilliseconds - lastTimeNewPos;
                float seekPos = (float)(timeSinceNewPos / (1000f / Program.updateFPS) + seekAhead);
                seekPos = Math.Min(seekPos, 1.5f);
                Vector3 finalPred = CurveExtra(new float[][] { new float[] { pm3.X, pm3.Y, pm3.Z }, new float[] { pm2.X, pm2.Y, pm2.Z }, new float[] { pm1.X, pm1.Y, pm1.Z } }, seekPos);
                //if (legsFolded) {
                //    lastPosGiven.X += (finalPred.X - lastPosGiven.X) * 0.3;
                //    lastPosGiven.Y += (finalPred.Y - lastPosGiven.Y) * 0.3;
                //    lastPosGiven.Z += (finalPred.Z - lastPosGiven.Z) * 0.3;
                //} else {
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
        static float getAngle(float[] a, float[] b) {
            float angL = GetAngleBetween(new float[] { 1, 0, 0 }, new float[] { a[0] - b[0], a[1] - b[1], a[2] - b[0] });
            //if (b[1] > a[1]) angL = -angL;
            return angL;
        }
        //THIS IS A MADE UP EXTRAPOLATION METHOD, DO NOT COPY, IT IS BAD LOL
        public static Vector3 CurveExtra(float[][] points, float dist) {

            float curviness = 0.7f;

            if (dist < 0) {
                float x = points[2][0] - points[1][0];
                float y = points[2][1] - points[1][1];
                float z = points[2][2] - points[1][2];
                float revD = dist + 1;
                return new Vector3(
                  points[1][0] + x * revD,
                  points[1][1] + y * revD,
                  points[1][2] + z * revD
                );
            }

            float[][] dX = new float[][] { new float[] { 1, points[1][0] }, new float[] { 2, points[2][0] } };
            float pX = LinearExtrapolate(dX, 3.0f);
            float[][] dY = new float[][] { new float[] { 1, points[1][1] }, new float[] { 2, points[2][1] } };
            float pY = LinearExtrapolate(dY, 3.0f);
            float[][] dZ = new float[][] { new float[] { 1, points[1][2] }, new float[] { 2, points[2][2] } };
            float pZ = LinearExtrapolate(dZ, 3.0f);
            float[] pr = new float[] { pX, pY, pZ };
            //fill(0, 255, 0);
            //ellipse(predicted.x, predicted.y, 10, 10);

            //float pM = dist(points[2][0], points[2][1], points[2][2], predicted.x, predicted.y, predicted.z);
            //float pM = (float)Math.Sqrt(Math.Pow(points[2][0] - pr[0], 2) + Math.Pow(points[2][1] - pr[1], 2) + Math.Pow(points[2][2] - pr[2], 2));
            float pM = Utils.GetDistance(pr, points[2]);
            //float[] mid = new float[] {(points[0][0] + points[2][0]) * 0.5f, (points[0][1] + points[2][1]) * 0.5f, (points[0][2] + points[2][2]) * 0.5f};
            float[] mid = new float[] {
                points[0][0] * 0.9f + points[2][0] * 0.1f,
                points[0][1] * 0.9f + points[2][1] * 0.1f,
                points[0][2] * 0.9f + points[2][2] * 0.1f
            };
            float curve = (1 + dist * curviness);
            mid[0] -= (mid[0] - points[1][0]) * curve;
            mid[1] -= (mid[1] - points[1][1]) * curve;
            mid[2] -= (mid[2] - points[1][2]) * curve;
            //fill(25, 25, 25);
            //ellipse(mid[0], mid[1], 3, 3);
            float[] newPred = new float[] {
                points[2][0]-mid[0],
                points[2][1]-mid[1],
                points[2][2]-mid[2]
            };
            //fill(0, 0, 127);
            //ellipse(newPred[0], newPred[1], 3, 3);
            float mag = GetMag(newPred[0], newPred[1], newPred[2]);
            //float mag2 = dist(0, 0, newPred[0], newPred[1]);
            float d = (pM * dist);
            newPred[0] = points[2][0] + ((newPred[0] / mag) * d);
            newPred[1] = points[2][1] + ((newPred[1] / mag) * d);
            newPred[2] = points[2][2] + ((newPred[2] / mag) * d);
            //newPred[0] *= pM * dist;
            //newPred[1] *= pM * dist;
            return new Vector3(newPred[0], newPred[1], newPred[2]);
        }
        static float GetMag(float x, float y, float z) {
            //From Processing
            return (float)Math.Sqrt(x * x + y * y + z * z);
        }
        static float LinearExtrapolate(float[][] d, float x) {
            float y = d[0][1] + (x - d[0][0]) / (d[1][0] - d[0][0]) * (d[1][1] - d[0][1]);
            return y;
        }
        static float GetAngleBetween(float[] v1, float[] v2) {
            //From Processing
            if (v1[0] == 0 && v1[1] == 0 && v1[2] == 0) return 0.0f;
            if (v2[0] == 0 && v2[1] == 0 && v2[2] == 0) return 0.0f;

            double dot = v1[0] * v2[0] + v1[1] * v2[1] + v1[2] * v2[2];
            double v1mag = Math.Sqrt(v1[0] * v1[0] + v1[1] * v1[1] + v1[2] * v1[2]);
            double v2mag = Math.Sqrt(v2[0] * v2[0] + v2[1] * v2[1] + v2[2] * v2[2]);
            double amt = dot / (v1mag * v2mag);
            if (amt <= -1) {
                return 3.14159265f;
            } else if (amt >= 1) {
                return 0;
            }
            return (float)Math.Acos(amt);
        }
        static float[] FromAngle(float angle) {
            //From Processing
            float[] target = new float[] { (float)Math.Cos(angle), (float)Math.Sin(angle) };
            return target;
        }
    }
}
