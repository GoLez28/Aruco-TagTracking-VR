using Emgu.CV;
using System;
using System.Collections.Generic;
using System.ComponentModel.DataAnnotations;
using System.Numerics;

namespace TagTracking {
    static partial class Tag {
        public class ClusterTracker {
            public CombinedTracker[] trackers;
            public OneEuroFilter<Quaternion>[] trackersRotationsFilter; //w 25
            public int[] trackerIndex = new int[] { 0, 1, 2, 3 };
            public Vector3[] trackerOffsets = new Vector3[] {
                new Vector3 (0f, 0f, -0.05f),
                new Vector3 (0f, 0f, -0.05f),
                new Vector3 (0f, 0f, -0.05f),
                new Vector3 (0f, 0f, -0.05f)
            };
            public Vector3[] trackerRotations = new Vector3[] {
                new(0f,                     0f, 0f),
                new((float)Math.PI / 2,     0f, 0f),
                new((float)Math.PI,         0f, 0f),
                new((float)Math.PI * 1.5f,  0f, 0f) };
            public float reduceOffset = 0f;
            public float[] trackerPresence = new float[4];
            public int[] updateCount = new int[4];
            public int[] updateCountp = new int[4];
            public Matrix4x4[] prevMat;

            public Quaternion prevRot = new Quaternion();
            public Quaternion prevx2Rot = new Quaternion();
            public Quaternion prevRotFinal = new();
            public Quaternion currentRot = new();
            public Quaternion prevRotRaw = new Quaternion();
            public int lastRotCount = 0;
            public Vector3 prevPos = new Vector3();
            public Vector3 prevx2Pos = new Vector3();
            public string trackerName = "unknown";

            public float avgSmoothDistTrigger = 0.01f; //w 0.05
            public float avgSmoothVal = 0.1f; //w 0.04
            public float avgSmoothRecoverVal = 0.95f;
            public float avgSmoothAlwaysVal = 0.08f;
            public float maxSpikePosDist = 0.5f;
            public float maxSpikeRotDiff = 0.9f;
            public float smoothedRot = 5;
            public float smoothedPos = 2;
            public bool generatedByCalibration = false;
            public bool trackerNotSeen = false;
            public long trackerDisableMax = 30000; //30 seconds
            int ghost = 0; //cheap way to avoid all uppdateCount at 0 and crash

            public float rotationComparison = 0.95f;
            public float straightTrackerWeight = 1.5f;

            public float trackerFollowWeight = 0.0f;
            public float rightElbowtrackerFollowWeight = 0.0f;
            public float leftElbowtrackerFollowWeight = 0.0f;

            public bool isHead = false;
            public bool isLeftHand = false;
            public bool isRightHand = false;
            public Vector3 positionOffset = new();

            public string anchorTracker = "";
            public Vector3? defaultPose = null;//if null, try to follow last position

            public ClusterTracker(string name, int[] ids, Vector3[] ofss, Vector3[] rots) {
                trackerName = name;
                trackerIndex = ids;
                trackerOffsets = ofss;
                trackerRotations = rots;
                updateCount = new int[ids.Length * cameras.Length];
                updateCountp = new int[ids.Length * cameras.Length];
                trackers = new CombinedTracker[ids.Length];
                prevMat = new Matrix4x4[ids.Length * cameras.Length];
                trackersRotationsFilter = new OneEuroFilter<Quaternion>[ids.Length * cameras.Length];
                trackerPresence = new float[ids.Length * cameras.Length];
                for (int i = 0; i < ids.Length; i++) {
                    trackers[i] = new CombinedTracker(ids[i]);
                }
                for (int i = 0; i < ids.Length * cameras.Length; i++) {
                    prevMat[i] = new();
                    trackersRotationsFilter[i] = new OneEuroFilter<Quaternion>(1);
                }
                //newInfo = true;
            }
            public void UpdatePerTrackerFilterRot(float frec) {
                for (int i = 0; i < trackersRotationsFilter.Length; i++) {
                    trackersRotationsFilter[i].UpdateParams(frec);
                }
            }
            public void UpdatePerTrackerFilterDepth(float frec) {
                for (int i = 0; i < trackers.Length; i++) {
                    for (int j = 0; j < trackers[i].singles.Length; j++) {
                        trackers[i].singles[j].filter_pos.UpdateParams(frec);
                    }
                }
            }
            public Matrix4x4 Obtain() {
                //THIS SHIT NEEDS TO IMPLEMENT MORE THAN 2 CAMERAS, OR JUST ONE

                //if (!trackerName.Equals("rightfoot")) return new();
                //if (trackerName.Equals("waist")) {
                //    trackerRotations = new float[] { 0f, (float)Math.PI * 0.35f, (float)Math.PI * 0.65f, (float)Math.PI * 1f };
                //    trackerOffsets = new Vector3[] {
                //        new Vector3 (0.05f, 0.055f, -0.23f),
                //        new Vector3 (0.04f, 0.0f, -0.12f),
                //        new Vector3 (-0.04f, -0.0f, -0.12f),
                //        new Vector3 (-0.05f, -0.055f, -0.23f)
                //    };
                //}

                //INCREMENT COUNT SINCE
                if (newInfo) {
                    for (int k = 0; k < updateCount.Length; k++) {
                        updateCount[k]++;
                    }
                }

                //PRELOAD VARIABLES
                for (int i = 0; i < trackers.Length * cameras.Length; i += cameras.Length) {
                    int ix2 = i / cameras.Length;
                    for (int j = 0; j < cameras.Length; j++) {
                        if (trackers[ix2].updateCount[j] == 0)
                            updateCount[i + j] = 0;
                        if (cameras[j].newData)
                            trackers[ix2].updateCount[j]++;
                    }
                }
                int cams = cameras.Length;
                Matrix4x4[] trackersMat, trackerRotationsMat, trackerOffsetsMat;
                float[] trackerStraightness;

                //INITIALIZE VARIABLES
                Vector3[] estimatedPos, poss;
                Vector3[] estimatedPos0;
                List<List<Quaternion>> posibleRotsCount;
                Quaternion[] estimatedRot, rots;
                List<List<int>> posibleRotsCountId;
                int actualAvailableTrackers = 0, maxId;
                for (int i = 0; i < updateCount.Length; i++) {
                    if (updateCount[i] <= 1) {
                        actualAvailableTrackers++;
                    }
                }
                trackerNotSeen = false;
                if (actualAvailableTrackers == 0) {
                    Matrix4x4 resultZero = Matrix4x4.Multiply(Matrix4x4.CreateFromQuaternion(prevRot), Matrix4x4.CreateTranslation(prevPos));
                    trackerNotSeen = true;
                    return resultZero;
                }

                //GET CENTERED
                //filter repeated rotations
                for (int i = 0; i < trackers.Length; i++) {
                    for (int j = 0; j < trackers[i].singles.Length; j++) {
                        if (trackers[i].updateCount[j] > 3) continue;

                        Matrix4x4 aRot = trackers[i].singles[j].rot;
                        Quaternion qRot = aRot.Rotation();
                        for (int k = 0; k < trackers[i].singles[j].altRots.Length; k++) {
                            Matrix4x4 aRota = trackers[i].singles[j].altRots[k];
                            Quaternion qRota = aRota.Rotation();
                            float dot = Quaternion.Dot(qRot, qRota);
                            trackers[i].singles[j].repeatedRots[k] = dot > 0.99f;
                        }

                        //for (int k = 0; k < trackers[i].singles[j].altRots.Length; k++) {
                        //    Matrix4x4 aRot;
                        //    //if (k == -1) aRot = trackers[i].singles[j].rot;
                        //    aRot = trackers[i].singles[j].altRots[k];
                        //    Quaternion qRot = aRot.Rotation();
                        //    bool reptd = false;
                        //    for (int l = k - 1; l >= -1; l--) {
                        //        Matrix4x4 aRot2;
                        //        if (l == -1) aRot2 = trackers[i].singles[j].rot;
                        //        else aRot2 = trackers[i].singles[j].altRots[l];
                        //        Quaternion qRot2 = aRot2.Rotation();
                        //        float dot = Quaternion.Dot(qRot, qRot2);
                        //        //Console.WriteLine($"({k}, {l}): {dot}");
                        //        if (dot > 0.99f) {
                        //            reptd = true;
                        //            break;
                        //        }
                        //    }
                        //    trackers[i].singles[j].repeatedRots[k] = reptd;
                        //}
                        //Console.WriteLine($"{trackers[i].singles[j].repeatedRots[0]}, {trackers[i].singles[j].repeatedRots[1]}, {trackers[i].singles[j].repeatedRots[2]}, {trackers[i].singles[j].repeatedRots[3]}");
                        //Console.WriteLine();
                    }
                }

                //start first -1 outside of loop bc of fkn declarations
                //Console.WriteLine(trackerName);
                int[] bestCorner = new int[trackers.Length * cameras.Length];// { -1, -1, -1, -1, -1, -1, -1, -1 };
                float bestDistance = float.MaxValue;
                int[] getCorner = new int[trackers.Length * cameras.Length];
                for (int i = 0; i < bestCorner.Length; i++) {
                    bestCorner[i] = -1;
                    getCorner[i] = -1;
                }
                int[] getCornerEx = getCorner;// new int[] { getCorner[0], getCorner[0], getCorner[1], getCorner[1], getCorner[2], getCorner[2], getCorner[3], getCorner[3] };
                //set first guess
                InitializeValues(cams, getCornerEx, out trackersMat, out trackerRotationsMat, out trackerOffsetsMat, out trackerStraightness);
                GetFinalCenteredPositions(true, cams, trackersMat, trackerRotationsMat, trackerOffsetsMat, trackerStraightness, out estimatedPos0, out estimatedRot, out posibleRotsCount, out posibleRotsCountId, out maxId, out poss, out rots);
                float centerDistance = GetAveragePositionDifference(poss, estimatedPos0, updateCount);
                if (centerDistance < bestDistance) {
                    bestDistance = centerDistance;
                    getCornerEx.CopyTo(bestCorner, 0);
                }

                //search for best guess
                int maxVal = 3;
                int arrEnd = getCorner.Length - 1;
                System.Diagnostics.Stopwatch sw = new();
                sw.Start();
                bool first = true;
                int calcCount = 0;
                int cInc = 1;
                bool warn = false;
                while (ghost > 4 && Program.clusterRotationGuess != 0) {
                    if (sw.ElapsedMilliseconds > 20) {
                        warn = true;
                        break;
                    }
                    if (getCorner[0] > maxVal) break;
                    int pointer = 0;
                    bool endLoop = false;
                    if (calcCount > 64) cInc = 2;
                    int pointArr = arrEnd - pointer;
                    while (getCorner[pointArr] > maxVal) {
                        if (updateCount[pointArr] < 3)
                            getCorner[pointArr] = -1;
                        if (pointer < arrEnd) {
                            pointer++;
                            pointArr = arrEnd - pointer;
                            if (updateCount[pointArr] < 3) {
                                bool next = false;
                                do {
                                    getCorner[pointArr] += cInc;
                                    next = Program.clusterRotationGuess == 1 && getCorner[pointArr] <= 3 && trackers[pointArr / cameras.Length].singles[pointArr % cameras.Length].repeatedRots[getCorner[pointArr]];
                                }
                                while (next);
                            } else
                                getCorner[pointArr] = 4;
                        } else {
                            endLoop = true;
                            break;
                        }
                    }
                    if (endLoop) break;
                    getCornerEx = getCorner; //new int[] { getCorner[0], getCorner[0], getCorner[1], getCorner[1], getCorner[2], getCorner[2], getCorner[3], getCorner[3] };
                    bool hasRepeated = false;
                    if (updateCount[arrEnd - pointer] < 2 || first) {
                        if (!hasRepeated) {
                            System.Diagnostics.Stopwatch persw = new();
                            persw.Start();
                            calcCount++;
                            InitializeValues(cams, getCornerEx, out trackersMat, out trackerRotationsMat, out trackerOffsetsMat, out trackerStraightness);
                            GetFinalCenteredPositions(false, cams, trackersMat, trackerRotationsMat, trackerOffsetsMat, trackerStraightness, out estimatedPos, out estimatedRot, out posibleRotsCount, out posibleRotsCountId, out maxId, out poss, out rots);
                            centerDistance = GetAveragePositionDifference(poss, estimatedPos0, updateCount);
                            if (centerDistance < bestDistance && !first) {
                                bestDistance = centerDistance;
                                getCornerEx.CopyTo(bestCorner, 0);
                            }
                            persw.Stop();
                        }
                        //Console.WriteLine($"{getCorner[0]}, {getCorner[1]}, {getCorner[2]}, {getCorner[3]}, {getCorner[4]}, {getCorner[5]}, {getCorner[6]}, {getCorner[7]} - {persw.ElapsedTicks}t - {persw.ElapsedMilliseconds}ms");
                        //getCorner[arrEnd] += cInc;
                        do {
                            getCorner[arrEnd] += cInc;
                        }
                        while (Program.clusterRotationGuess == 1 && getCorner[arrEnd] <= 3 && trackers[arrEnd / cameras.Length].singles[arrEnd % cameras.Length].repeatedRots[getCorner[arrEnd]]);
                    } else {
                        getCorner[arrEnd] = 4;
                    }
                    first = false;
                }
                ghost++;
                sw.Stop();
                if (sw.Elapsed.TotalMilliseconds > 5) {
                    //Console.WriteLine($"Tracker ({trackerName}) took too long: {sw.ElapsedMilliseconds}ms / {calcCount} times");
                    Program.infoBarWarning = $"{trackerName} {sw.Elapsed.TotalMilliseconds.ToString("0.00")}ms {(warn ? "skip" : "")}";
                }
                //bestCorner = new int[] { 3, 2, -1, -1 };
                InitializeValues(cams, bestCorner, out trackersMat, out trackerRotationsMat, out trackerOffsetsMat, out trackerStraightness);
                GetFinalCenteredPositions(true, cams, trackersMat, trackerRotationsMat, trackerOffsetsMat, trackerStraightness, out estimatedPos, out estimatedRot, out posibleRotsCount, out posibleRotsCountId, out maxId, out poss, out rots);
                //if (trackerName.Equals("leftfoot")) Console.WriteLine($"Best:\t{bestCorner[0]}\t/\t{bestCorner[1]}\t/\t{bestCorner[2]}\t/\t{bestCorner[3]}\t - {centerDistance}");
                //if (trackerName.Equals("leftfoot"))
                //System.Threading.Thread.Sleep(100);
                //Console.WriteLine();

                for (int i = 0; i < updateCount.Length; i++) {
                    if (updateCount[i] <= 2) {
                        bool found = false;
                        for (int j = 0; j < posibleRotsCount[maxId].Count; j++) {
                            if (i == posibleRotsCountId[maxId][j]) {
                                found = true;
                                break;
                            }
                        }
                        if (!found) {
                            updateCount[i]++;
                        }
                    }
                }

                for (int i = 0; i < estimatedPos0.Length; i++) {
                    //if (i == 7)
                    //    Console.WriteLine(estimatedRot[i]);
                    ////prevMat[i] = Matrix4x4.Multiply(Matrix4x4.CreateFromQuaternion(estimatedRot[i]), Matrix4x4.CreateTranslation(estimatedPos0[i]));
                    //if (i == 7)
                    //    Console.WriteLine(prevMat[i].Rotation());
                    //draw current trackers
                    if (!float.IsNaN(estimatedRot[i].X) && !float.IsNaN(estimatedPos0[i].X) && (Program.frameCount / 8) % 1 == 0) {
                        if (updateCount[i] > 3) {
                            if (updateCount[i] < 50)
                                Aruco.DrawAxisGray(prevMat[i], Utils.GetMap(trackerPresence[i], 0, ticksToFadeTag, 1f, 0.2f));
                        } else
                            Aruco.DrawAxis(prevMat[i], Utils.GetMap(trackerPresence[i], 0, ticksToFadeTag, 1f, 0.2f));
                    }

                }
                for (int i = 0; i < poss.Length; i++) {
                    if (updateCount[i] > 3) continue;
                    //this show the separate position before sum and final , should be a point
                    Aruco.DrawDot(Matrix4x4.CreateTranslation(poss[i]), 0.1f);
                }

                if (Program.debugSendTrackerOSC) {
                    for (int i = 0; i < trackerIndex.Length; i++) {
                        int id = trackerIndex[i];
                        for (int j = 0; j < cameras.Length; j++) {
                            int v = i * cameras.Length + j;
                            if (updateCount[v] > 200) continue;
                            Program.oscClientDebug.Send($"/debug/predicted/position", id, 0,
                                poss[v].X, poss[v].Z, poss[v].Y,
                                -rots[v].X, -rots[v].Z, -rots[v].Y, rots[v].W);
                            Program.oscClientDebug.Send($"/debug/predicted/position", id, 1,
                                estimatedPos[v].X, estimatedPos[v].Z, estimatedPos[v].Y,
                                -estimatedRot[v].X, -estimatedRot[v].Z, -estimatedRot[v].Y, estimatedRot[v].W);
                        }
                    }
                }

                //GET WHEN WAS LAST TIME WEIGHTS
                float minPosPresence = 0;
                float maxPosPresence = 0;
                Vector3 avgPosPresence = new();
                for (int i = 0; i < estimatedPos0.Length; i++) {
                    //if (trackerPresence[i] < minPosPresence) minPosPresence = trackerPresence[i];
                    if (trackerPresence[i] > maxPosPresence) maxPosPresence = trackerPresence[i];
                }
                if (maxPosPresence == minPosPresence)
                    maxPosPresence++;
                float sumPosPresence = 0;
                for (int i = 0; i < estimatedPos0.Length; i++) {
                    float m = trackerStraightness[i] > 0.25f ? (Utils.GetMap(trackerStraightness[i], 0.25f, 0.5f, 1f, 0f)) : 1f;
                    m = (float)Math.Pow(m, 2);
                    sumPosPresence += Utils.GetMap(trackerPresence[i], minPosPresence, maxPosPresence, 1f, 0f) * m;
                }
                //SET WEIGHTED CENTER FROM TRACKERS
                if (actualAvailableTrackers == 0 || sumPosPresence == 0) {
                    avgPosPresence = prevPos;
                } else {
                    //avgPosPresence += prevPos * (1f / sumPosPresence);
                    for (int i = 0; i < estimatedPos0.Length; i++) {
                        float m = trackerStraightness[i] > 0.25f ? (Utils.GetMap(trackerStraightness[i], 0.25f, 0.5f, 1f, 0f)) : 1f;
                        m = (float)Math.Pow(m, 2);
                        float add = Utils.GetMap(trackerPresence[i], minPosPresence, maxPosPresence, 1f / sumPosPresence, 0f) * m;
                        if (!float.IsNaN(add))
                            avgPosPresence += poss[i] * add;
                    }
                }
                //AVERAGE VALIDATED ROTATIONS
                Quaternion prevRotPresence = new Quaternion(0, 0, 0, 0); //DONT CHANGE, LEAVE ALL AT ZERO
                //if (trackerName.Equals("waist")) {
                //    for (int i = 0; i < trackerPresence.Length; i++) {
                //        Console.SetCursorPosition(0, i + 200);
                //        Console.WriteLine(i + ", " + trackerPresence[i]);
                //    }
                //}
                //Console.WriteLine();
                if (maxId == -1 || actualAvailableTrackers == 0) {
                    prevRotPresence = prevRot; //do this in case nothing is recognized
                    lastRotCount++;
                } else {
                    if (posibleRotsCountId[maxId].Count == 1 && posibleRotsCountId[maxId][0] == -1) { //if the only rot was the previous
                        lastRotCount++;
                    } else {
                        lastRotCount = 0; //set 0 if sure
                    }
                    float[] trackerPresenceRots = new float[posibleRotsCount[maxId].Count];
                    for (int i = 0; i < trackerPresenceRots.Length; i++) {
                        int index = posibleRotsCountId[maxId][i];
                        if (index == -1) {
                            trackerPresenceRots[i] = 75;
                        } else
                            trackerPresenceRots[i] = trackerPresence[posibleRotsCountId[maxId][i]];
                    }

                    //GET WHEN WAS LAST TIME WEIGHTS, FOR ROTATIONS
                    minPosPresence = int.MaxValue;
                    maxPosPresence = 0;
                    for (int i = 0; i < trackerPresenceRots.Length; i++) {
                        if (trackerPresenceRots[i] < minPosPresence) minPosPresence = trackerPresenceRots[i];
                        if (trackerPresenceRots[i] > maxPosPresence) maxPosPresence = trackerPresenceRots[i];
                    }
                    if (maxPosPresence == minPosPresence)
                        maxPosPresence++;
                    sumPosPresence = 0;
                    for (int i = 0; i < trackerPresenceRots.Length; i++) {
                        sumPosPresence += Utils.GetMap(trackerPresenceRots[i], minPosPresence, maxPosPresence, 1f, 0f);
                    }
                    for (int i = 0; i < posibleRotsCount[maxId].Count; i++) { //basically sum(val[i] * 1/length)
                        float add = Utils.GetMap(trackerPresenceRots[i], minPosPresence, maxPosPresence, 1f / sumPosPresence, 0f);
                        //Quaternion qm = Utils.QLerp(new Quaternion(0, 0, 0, 0), posibleRotsCount[maxId][i], add);
                        Quaternion q1 = posibleRotsCount[maxId][i];
                        prevRotPresence.X += add * q1.X;
                        prevRotPresence.Y += add * q1.Y;
                        prevRotPresence.Z += add * q1.Z;
                        prevRotPresence.W += add * q1.W;
                        //Quaternion qm = Utils.QLerp(Quaternion.Identity, posibleRotsCount[maxId][i], 1f / posibleRotsCount[maxId].Count);
                    }
                    prevRotPresence = Quaternion.Normalize(prevRotPresence);
                }

                //SET PREVIOUS POS AND ROT
                prevRotPresence = Quaternion.Normalize(prevRotPresence);
                prevx2Pos = prevPos;
                prevPos = avgPosPresence;
                prevx2Rot = prevRot;
                prevRot = prevRotPresence;
                //UPDATE PREVIOUS COUNT
                if (newInfo) {
                    for (int k = 0; k < updateCount.Length; k++) {
                        updateCountp[k] = updateCount[k];
                    }
                }

                //GET MATRIX FROM VECTOR AND QUATERNION
                Matrix4x4 result2 = Matrix4x4.Multiply(Matrix4x4.CreateFromQuaternion(prevRotPresence), Matrix4x4.CreateTranslation(avgPosPresence));
                //if ((Program.frameCount / 2) % 1 == 0 && !float.IsNaN(avgPosPresence.X))
                //    Aruco.DrawAxis(result2);
                prevRotFinal = currentRot;
                currentRot = prevRotPresence;
                return result2;

            }

            private void InitializeValues(int cams, int[] altCorners, out Matrix4x4[] trackersMat, out Matrix4x4[] trackerRotationsMat, out Matrix4x4[] trackerOffsetsMat, out float[] trackerStraightness) {
                trackersMat = new Matrix4x4[trackers.Length * cams];
                trackerRotationsMat = new Matrix4x4[trackers.Length * cams];
                trackerOffsetsMat = new Matrix4x4[trackers.Length * cams];
                trackerStraightness = new float[trackers.Length * cams];
                for (int i = 0; i < trackers.Length * cams; i += cams) {
                    int ix2 = i / cams;
                    int[] getCorners = new int[cams];
                    for (int j = 0; j < cams; j++) {
                        getCorners[j] = altCorners[i + j];
                    }
                    Matrix4x4[] get = trackers[ix2].Obtain(getCorners);
                    for (int j = 0; j < cams; j++) {
                        trackersMat[i + j] = get[j];
                        trackerStraightness[i + j] = trackers[ix2].trackerStraightness[j];
                        if (generatedByCalibration) {
                            float cy = MathF.Cos(trackerRotations[ix2].Z * 0.5f);
                            float sy = MathF.Sin(trackerRotations[ix2].Z * 0.5f);
                            float cr = MathF.Cos(trackerRotations[ix2].Y * 0.5f);
                            float sr = MathF.Sin(trackerRotations[ix2].Y * 0.5f);
                            float cp = MathF.Cos(trackerRotations[ix2].X * 0.5f);
                            float sp = MathF.Sin(trackerRotations[ix2].X * 0.5f);
                            float w = cy * cr * cp + sy * sr * sp;
                            float x = cy * sr * cp - sy * cr * sp;
                            float y = cy * cr * sp + sy * sr * cp;
                            float z = sy * cr * cp - cy * sr * sp;
                            Quaternion quaternion = new Quaternion(x, y, z, w);
                            Matrix4x4 rotMat = Matrix4x4.CreateFromQuaternion(quaternion);
                            trackerRotationsMat[i + j] = rotMat;
                        } else {
                            trackerRotationsMat[i + j] = Matrix4x4.CreateFromAxisAngle(new Vector3(0, -1, 0), trackerRotations[ix2].X);
                            trackerRotationsMat[i + j] = Matrix4x4.Multiply(trackerRotationsMat[i + j], Matrix4x4.CreateFromAxisAngle(new Vector3(1, 0, 0), trackerRotations[ix2].Y));
                            trackerRotationsMat[i + j] = Matrix4x4.Multiply(trackerRotationsMat[i + j], Matrix4x4.CreateFromAxisAngle(new Vector3(0, 0, 1), trackerRotations[ix2].Z));
                        }


                        trackerOffsetsMat[i + j] = Matrix4x4.CreateTranslation(trackerOffsets[ix2] * (1f - reduceOffset));
                    }
                }
            }

            private static float GetAveragePositionDifference(Vector3[] poss, Vector3[] toCenter, int[] updateCount) {
                float centerDistance = 0;
                //Vector3 centerPosition = new();
                //int count = 0;
                //for (int i = 0; i < toCenter.Length; i++) {
                //    if (updateCount[i] > 3)
                //        continue;
                //    centerPosition += toCenter[i];
                //    count++;
                //}
                //centerPosition /= count;
                //for (int i = 0; i < poss.Length; i++) {
                //    if (updateCount[i] > 3)
                //        continue;
                //    centerDistance += Math.Abs(centerPosition.X - poss[i].X) + Math.Abs(centerPosition.Y - poss[i].Y) + Math.Abs(centerPosition.Z - poss[i].Z);
                //}
                for (int i = 0; i < poss.Length; i++) {
                    if (updateCount[i] > 4)
                        continue;
                    for (int j = i + 1; j < poss.Length; j++) {
                        if (updateCount[j] > 4)
                            continue;
                        centerDistance += Math.Abs(poss[j].X - poss[i].X) + Math.Abs(poss[j].Y - poss[i].Y) + Math.Abs(poss[j].Z - poss[i].Z);
                    }
                }
                return centerDistance;
            }

            private void GetFinalCenteredPositions(bool final, int cams, Matrix4x4[] trackersMat, Matrix4x4[] trackerRotationsMat, Matrix4x4[] trackerOffsetsMat, float[] trackerStraightness, out Vector3[] estimatedPos, out Quaternion[] estimatedRot, out List<List<Quaternion>> posibleRotsCount, out List<List<int>> posibleRotsCountId, out int maxId, out Vector3[] poss, out Quaternion[] rots) {
                estimatedPos = new Vector3[trackers.Length * cams];
                estimatedRot = new Quaternion[trackers.Length * cams];
                Matrix4x4[] estimatedMat = new Matrix4x4[trackers.Length * cams];
                Vector3 availableAvgPos = new();
                Vector3 availableAvgPosDiff = new();
                Quaternion availableAvgRot = Quaternion.Identity;
                Quaternion availableAvgRotDiff = Quaternion.Identity;
                int availableCount = 0;
                List<Quaternion> posibleRots = new List<Quaternion>();
                posibleRotsCount = new List<List<Quaternion>>();
                posibleRotsCountId = new List<List<int>>();
                poss = new Vector3[estimatedPos.Length];
                rots = new Quaternion[estimatedRot.Length];
                maxId = -1;
                if (lastRotCount < 2) { //add previous rotation
                    posibleRots.Add(prevRot);
                    posibleRotsCount.Add(new List<Quaternion>());
                    posibleRotsCount[0].Add(prevRot);
                    posibleRotsCountId.Add(new List<int>());
                    posibleRotsCountId[0].Add(-1);
                }

                //for (int i = 0; i < updateCount.Length; i++) {
                //    if (updateCount[i] <= 1) {
                //        Matrix4x4 newMat = trackersMat[i];

                //        estimatedPos[i] = newMat.Translation;
                //        estimatedRot[i] = newMat.Rotation();
                //    }
                //}
                //Quaternion[] rots = new Quaternion[estimatedRot.Length];
                //for (int i = 0; i < estimatedPos.Length; i++) {
                //    Matrix4x4 mat = Matrix4x4.Multiply(Matrix4x4.CreateFromQuaternion(estimatedRot[i]), Matrix4x4.CreateTranslation(estimatedPos[i]));
                //    mat = Matrix4x4.Multiply(trackerOffsetsMat[i], mat);
                //    mat = Matrix4x4.Multiply(trackerRotationsMat[i], mat);
                //    poss[i] = mat.Translation;
                //    if (float.IsNaN(poss[i].X))
                //        poss[i] = new();
                //    rots[i] = mat.Rotation();
                //    if (float.IsNaN(rots[i].X))
                //        rots[i] = Quaternion.Identity;
                //    //Aruco.DrawAxis(Matrix4x4.Multiply(Matrix4x4.CreateFromQuaternion(rots[i]), Matrix4x4.CreateTranslation(poss[i])));
                //}
                //return;

                //CHECK FOR PARS
                for (int i = 0; i < updateCount.Length; i++) {
                    if (updateCount[i] <= 2) {

                        Matrix4x4 newMat = trackersMat[i];
                        Quaternion newMatRot = newMat.Rotation();
                        Matrix4x4 result = Matrix4x4.Multiply(trackerOffsetsMat[i], newMat);
                        Matrix4x4 rotMat = trackerRotationsMat[i];
                        result = Matrix4x4.Multiply(rotMat, result);
                        bool added = false;
                        Quaternion resultRot = result.Rotation();
                        //if (resultRot.X < 0 && resultRot.Y > 0 && resultRot.Z > 0 && resultRot.W < 0)
                        //    resultRot = Quaternion.Negate(resultRot);
                        for (int j = 0; j < posibleRots.Count; j++) {
                            Quaternion qDif = Quaternion.Inverse(posibleRots[j]) * resultRot;
                            float rotDiff = Quaternion.Dot(qDif, Quaternion.Identity);
                            if (rotDiff > /*rotationComparison*/0.9f) {
                                posibleRotsCount[j].Add(resultRot);
                                posibleRotsCountId[j].Add(i);
                                posibleRots[j] = Quaternion.Lerp(posibleRots[j], resultRot, 0.25f);
                                added = true;
                                break;
                            }
                        }
                        if (!added) {
                            posibleRots.Add(resultRot);
                            List<Quaternion> newAdd = new List<Quaternion>();
                            newAdd.Add(resultRot);
                            posibleRotsCount.Add(newAdd);
                            List<int> newAddi = new List<int>();
                            newAddi.Add(i);
                            posibleRotsCountId.Add(newAddi);
                        }
                    }
                }

                //GET HIGHEST PAR COUNT
                int maxPosible = 0;
                for (int i = 0; i < posibleRots.Count; i++) {
                    if (maxPosible < posibleRotsCount[i].Count) {
                        maxId = i;
                        maxPosible = posibleRotsCount[i].Count;
                    }
                }
                //if (maxPosible == 1) {
                //    float straightness = -0.4f;
                //    for (int i = 0; i < posibleRots.Count; i++) {
                //        int id = posibleRotsCountId[i][0];
                //        if (id == -1) continue;
                //        float val = trackers[id].trackerStraightnessMax;
                //        if (val > straightness) {
                //            straightness = val;
                //            maxId = i;
                //        }
                //    }
                //}

                //INCREMENT LAST SEEN COUNT FOR TAGS THAT FAILED PAR CHECK
                int[] updateCountCpy = new int[updateCount.Length];
                updateCount.CopyTo(updateCountCpy, 0);
                for (int i = 0; i < updateCountCpy.Length; i++) {
                    if (updateCountCpy[i] <= 1) {
                        bool found = false;
                        for (int j = 0; j < posibleRotsCount[maxId].Count; j++) {
                            if (i == posibleRotsCountId[maxId][j]) {
                                found = true;
                                break;
                            }
                        }
                        if (!found) {
                            updateCountCpy[i]++;
                        }
                    }
                }

                //GET DIFFERENCE FROM LAST TIME, AND SET ABSOLUTE FOR TAG SEEN
                bool first = true;
                for (int i = 0; i < updateCountCpy.Length; i++) {
                    if (updateCountCpy[i] < 2) {
                        Matrix4x4 newMat = trackersMat[i];
                        bool allowTransform = false;
                        if (maxId != -1) {
                            for (int j = 0; j < posibleRotsCountId[maxId].Count; j++) {
                                if (posibleRotsCountId[maxId][j] == i) {
                                    allowTransform = true;
                                    break;
                                }
                            }
                        }
                        if (updateCountp[i] <= 1 && allowTransform) {
                            Vector3 newMatPos = newMat.Translation;
                            Vector3 posDiff = newMatPos - prevMat[i].Translation;
                            availableAvgPosDiff += posDiff;
                            availableAvgPos += newMatPos;
                            //Quaternion rotDiff = newMat.Rotation() * Quaternion.Inverse(prevMat[i].Rotation());
                            //availableAvgRot = rotDiff;// first ? rotDiff : Quaternion.Lerp(availableAvgRot, rotDiff, availableCount == 0 ? 1f : 0.5f);
                            availableCount++;
                            first = false;
                        } else if (final) {
                            for (int j = 0; j < 5; j++) { //rectify filtering
                                trackersRotationsFilter[i].Filter(estimatedRot[i]);
                            }
                        }
                        estimatedMat[i] = newMat;
                        estimatedPos[i] = newMat.Translation;
                        estimatedRot[i] = Quaternion.Normalize(newMat.Rotation());
                        if (final) {
                            trackerPresence[i] -= straightTrackerWeight + trackerStraightness[i];

                            int camera = i % cameras.Length;
                            float min = Utils.GetMap(cameras[camera].quality, 1f, 2f, 10f, 0f) + ((float)Math.Pow(trackerStraightness[i] * 2, 2) * 20);
                            min = (float)Math.Max(min, 0);
                            if (min >= ticksToFadeTag) min = ticksToFadeTag - 1;
                            if (trackerPresence[i] < min)
                                trackerPresence[i] = min;
                        }
                    }
                }
                if (availableCount != 0) {
                    availableAvgPos /= availableCount;
                    availableAvgPosDiff /= availableCount;
                }

                //GET DIFFERENCE FROM LAST TIME, EVEN THOUGHT IT IS FAR AWAY, AND SET ABSOLUTE 
                //if (availableCount == 0) {
                //    for (int i = 0; i < trackers.Length; i++) {
                //        if (updateCount[i] == 1) {
                //            Matrix4x4 newMat = trackersMat[i];
                //            newMat = Matrix4x4.Multiply(trackerOffsetsMat[i], newMat);
                //            newMat = Matrix4x4.Multiply(trackerRotationsMat[i], newMat);
                //            Vector3 newMatPos = newMat.Translation;
                //            Vector3 posDiff = newMatPos - prevPos;
                //            availableAvgPos += newMatPos;
                //            availableAvgPosDiff += posDiff;
                //            Quaternion rotDiff = newMat.Rotation() * Quaternion.Inverse(prevRot);
                //            availableAvgRot = Utils.QLerp(availableAvgRot, rotDiff, availableCount == 0 ? 1f : 0.5f);
                //            availableCount++;
                //            estimatedPos[i] = newMatPos;
                //            estimatedRot[i] = newMat.Rotation();
                //            trackerPresence[i] -= 10;
                //            if (trackerPresence[i] < 0)
                //                trackerPresence[i] = 0;
                //        } else if (updateCount[i] > 1) {
                //            trackerPresence[i] += 10;
                //            if (trackerPresence[i] > 50)
                //                trackerPresence[i] = 50;
                //        }
                //    }
                //}
                //if (availableCount != 0) {
                //    availableAvgPos /= availableCount;
                //    availableAvgPosDiff /= availableCount;
                //}


                availableAvgRotDiff = prevRot * Quaternion.Inverse(prevx2Rot);
                //GET PREDICTED MATRIX FOR TAGS THAT ARE NOT SEEN
                for (int i = 0; i < updateCountCpy.Length; i++) {
                    if (updateCountCpy[i] >= 2 && updateCountCpy[i] < 1000) {
                        Quaternion prevAdjust = prevMat[i].Rotation();
                        prevAdjust.X = prevAdjust.X * 1.0000002f;
                        prevAdjust.Y = prevAdjust.Y * 1.0000002f;
                        prevAdjust.Z = prevAdjust.Z * 1.0000002f;
                        prevAdjust.W = prevAdjust.W * 1.0000008f;
                        //Type of shit that i have to do when something is not perfect
                        if (availableCount == 0) {
                            estimatedPos[i] = prevMat[i].Translation;
                            //Console.WriteLine(estimatedPos[i]);
                            estimatedRot[i] = prevAdjust;
                            //estimatedRot[i] = new Quaternion(0.66621256f, 0.32460567f, 0.38732502f, 0.5484261f);
                        } else {
                            Matrix4x4 newPos = Matrix4x4.CreateTranslation(prevMat[i].Translation - availableAvgPos);
                            estimatedPos[i] = Matrix4x4.Multiply(newPos, Matrix4x4.CreateFromQuaternion(availableAvgRotDiff)).Translation + availableAvgPos + availableAvgPosDiff;
                            //Matrix4x4 newPos = Matrix4x4.CreateTranslation(prevMat[i].Translation - prevPos);
                            //estimatedPos[i] = Matrix4x4.Multiply(newPos, Matrix4x4.CreateFromQuaternion(availableAvgRotDiff)).Translation + prevPos + (prevPos - prevx2Pos);
                            estimatedRot[i] = availableAvgRotDiff * prevAdjust;
                        }
                        if (final) {
                            //if (trackerPresence[i] < ticksToFadeTag * 0.5f) trackerPresence[i] = ticksToFadeTag * 0.25f;
                            trackerPresence[i] += straightTrackerWeight * 2;
                            float max = ticksToFadeTag + ((float)Math.Pow(trackerStraightness[i] * 2, 2) * 20);
                            max = (float)Math.Max(max, 0);
                            if (trackerPresence[i] > ticksToFadeTag)
                                trackerPresence[i] = ticksToFadeTag;
                            //trackerPresence[i] = 100;

                            if (trackerPresence[i] < ticksToFadeTag) {
                                int camera = i % cameras.Length;
                                int id = trackerIndex[i / cameras.Length];
                                Program.oscClientDebug.Send($"/debug/trackers/position", id, camera, estimatedPos[i].X, estimatedPos[i].Z, estimatedPos[i].Y, -estimatedRot[i].X, -estimatedRot[i].Z, -estimatedRot[i].Y, estimatedRot[i].W, (int)1);
                            }
                        }
                    }
                }
                //Console.WriteLine(estimatedRot[7]);

                //SET PREVIOUS MATRIX AS CURENT
                if (final)
                    for (int i = 0; i < estimatedPos.Length; i++) {
                        prevMat[i] = Matrix4x4.Multiply(Matrix4x4.CreateFromQuaternion(estimatedRot[i]), Matrix4x4.CreateTranslation(estimatedPos[i]));
                        //prevMat[i] = estimatedMat[i];
                        //if (i == 7)
                        //    Console.WriteLine(prevMat[i].Translation);
                        //draw current trackers
                        //if (!float.IsNaN(estimatedRot[i].X) && !float.IsNaN(estimatedPos[i].X) && (Program.frameCount / 8) % 2 == 0)
                        //    Aruco.DrawAxis(prevMat[i], Utils.GetMap(trackerPresence[i], 0, 100, 1f, 0.2f));
                    }
                //if (final)
                //    for (int i = 0; i < estimatedRot.Length; i++) {
                //        Quaternion filteredRot = trackersRotationsFilter[i].Filter(estimatedRot[i]);
                //        if (float.IsNaN(filteredRot.X)) {
                //            trackersRotationsFilter[i] = new OneEuroFilter<Quaternion>(trackersRotationsFilter[i].freq);
                //            //Console.WriteLine("recenter for " + trackerIndex[i] + " " + Program.frameCount);
                //            continue;
                //        }
                //        estimatedRot[i] = filteredRot;
                //    }
                //GET CENTERED TRACKERS MATRIX
                for (int i = 0; i < estimatedPos.Length; i++) {
                    Matrix4x4 mat = Matrix4x4.Multiply(Matrix4x4.CreateFromQuaternion(estimatedRot[i]), Matrix4x4.CreateTranslation(estimatedPos[i]));
                    mat = Matrix4x4.Multiply(trackerOffsetsMat[i], mat);
                    mat = Matrix4x4.Multiply(trackerRotationsMat[i], mat);
                    poss[i] = mat.Translation;
                    if (float.IsNaN(poss[i].X))
                        poss[i] = new();
                    rots[i] = mat.Rotation();
                    if (float.IsNaN(rots[i].X))
                        rots[i] = Quaternion.Identity;
                    if (final) {
                        if (Program.debugSendTrackerOSC && updateCountCpy[i] < 2) {
                            Matrix4x4 dir = Matrix4x4.Multiply(Matrix4x4.CreateTranslation(new Vector3(-0.25f, 0, 0)), Matrix4x4.Multiply(Matrix4x4.CreateFromQuaternion(rots[i]), Matrix4x4.CreateTranslation(poss[i])));
                            Vector3 dirV = dir.Translation;
                            int id = trackerIndex[i / cameras.Length];
                            Program.oscClientDebug.Send($"/debug/predicted/position", id, 2, dirV.X, dirV.Z, dirV.Y, 0, 0, 0, 1);
                        }
                    }
                    //Aruco.DrawAxis(Matrix4x4.Multiply(Matrix4x4.CreateFromQuaternion(rots[i]), Matrix4x4.CreateTranslation(poss[i])));
                }
            }
        }
    }
}
