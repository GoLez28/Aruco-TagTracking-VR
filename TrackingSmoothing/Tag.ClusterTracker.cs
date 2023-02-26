using System;
using System.Collections.Generic;
using System.Numerics;

namespace TrackingSmoothing {
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
            public float[] trackerRotations = new float[] { 0f, (float)Math.PI / 2, (float)Math.PI, (float)Math.PI * 1.5f };
            public float[] trackerPresence = new float[4];
            public int[] updateCount = new int[4];
            public int[] updateCountp = new int[4];
            public Matrix4x4[] prevMat;

            public Quaternion prevRot = new Quaternion();
            public Quaternion prevRotFinal = new();
            public Quaternion currentRot = new();
            public Quaternion prevRotRaw = new Quaternion();
            public int lastRotCount = 0;
            public Vector3 prevPos = new Vector3();
            public string trackerName = "unknown";

            public float avgSmoothDistTrigger = 0.025f; //w 0.05
            public float avgSmoothVal = 0.1f; //w 0.04
            public float avgSmoothRecoverVal = 0.9f;
            public float avgSmoothAlwaysVal = 0.08f;
            public float smoothedRot = 5;
            public float smoothedPos = 2;

            public float rotationComparison = 0.95f;
            public float straightTrackerWeight = 0.8f;

            public float trackerFollowWeight = 0.0f;

            public ClusterTracker(string name, int[] ids, Vector3[] ofss, float[] rots) {
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
                int cams = cameras.Length;
                Matrix4x4[] trackersMat = new Matrix4x4[trackers.Length * cams];
                Matrix4x4[] trackerRotationsMat = new Matrix4x4[trackers.Length * cams];
                Matrix4x4[] trackerOffsetsMat = new Matrix4x4[trackers.Length * cams];
                float[] trackerStraightness = new float[trackers.Length * cams];
                for (int i = 0; i < trackers.Length * 2; i += 2) {
                    int ix2 = i / 2;
                    Matrix4x4[] get = trackers[ix2].Obtain();
                    trackersMat[i] = get[0];
                    trackersMat[i + 1] = get[1];
                    if (trackers[ix2].updateCount[0] == 0) {
                        updateCount[i] = 0;
                    }
                    trackers[ix2].updateCount[0]++;
                    if (trackers[ix2].updateCount[1] == 0) {
                        updateCount[i + 1] = 0;
                    }
                    trackers[ix2].updateCount[1]++;
                    trackerStraightness[i] = trackers[ix2].trackerStraightness[0];
                    trackerStraightness[i + 1] = trackers[ix2].trackerStraightness[1];
                    trackerRotationsMat[i] = Matrix4x4.CreateFromAxisAngle(new Vector3(0, -1, 0), trackerRotations[ix2]);
                    trackerRotationsMat[i + 1] = trackerRotationsMat[i];
                    trackerOffsetsMat[i] = Matrix4x4.CreateTranslation(trackerOffsets[ix2]);
                    trackerOffsetsMat[i + 1] = trackerOffsetsMat[i];
                }

                //INITIALIZE VARIABLES
                Vector3[] estimatedPos = new Vector3[trackers.Length * cams];
                Quaternion[] estimatedRot = new Quaternion[trackers.Length * cams];
                Vector3 availableAvgPos = new();
                Vector3 availableAvgPosDiff = new();
                Quaternion availableAvgRot = Quaternion.Identity;
                int availableCount = 0;
                List<Quaternion> posibleRots = new List<Quaternion>();
                List<List<Quaternion>> posibleRotsCount = new List<List<Quaternion>>();
                List<List<int>> posibleRotsCountId = new List<List<int>>();
                int actualAvailableTrackers = 0;
                for (int i = 0; i < updateCount.Length; i++) {
                    if (updateCount[i] <= 1) {
                        actualAvailableTrackers++;
                    }
                }
                if (lastRotCount < 2) { //add previous rotation
                    posibleRots.Add(prevRot);
                    posibleRotsCount.Add(new List<Quaternion>());
                    posibleRotsCount[0].Add(prevRot);
                    posibleRotsCountId.Add(new List<int>());
                    posibleRotsCountId[0].Add(-1);
                }

                //CHECK FOR PARS
                for (int i = 0; i < updateCount.Length; i++) {
                    if (updateCount[i] <= 1) {

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
                int maxId = -1;
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
                for (int i = 0; i < updateCount.Length; i++) {
                    if (updateCount[i] <= 1) {
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

                //GET DIFFERENCE FROM LAST TIME, AND SET ABSOLUTE FOR TAG SEEN
                for (int i = 0; i < updateCount.Length; i++) {
                    if (updateCount[i] <= 1) {
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
                            Quaternion rotDiff = newMat.Rotation() * Quaternion.Inverse(prevMat[i].Rotation());
                            availableAvgRot = Quaternion.Lerp(availableAvgRot, rotDiff, availableCount == 0 ? 1f : 0.5f);
                            availableCount++;
                        } else {
                            for (int j = 0; j < 5; j++) { //rectify filtering
                                trackersRotationsFilter[i].Filter(estimatedRot[i]);
                            }
                        }
                        estimatedPos[i] = newMat.Translation;
                        estimatedRot[i] = newMat.Rotation();
                        trackerPresence[i] -= 1 - (1 - (trackerStraightness[i] + straightTrackerWeight)) * 2;

                        int camera = i % cameras.Length;
                        float min = Utils.GetMap(cameras[camera].quality, 1f, 2f, 50f, 0f);
                        min = (float)Math.Max(min, 0);
                        if (trackerPresence[i] < min)
                            trackerPresence[i] = min;
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

                //GET PREDICTED MATRIX FOR TAGS THAT ARE NOT SEEN
                for (int i = 0; i < updateCount.Length; i++) {
                    if (updateCount[i] > 1) {
                        if (availableCount == 0) {
                            estimatedPos[i] = prevMat[i].Translation;
                            estimatedRot[i] = prevMat[i].Rotation();
                        } else {
                            Matrix4x4 newPos = Matrix4x4.CreateTranslation(prevMat[i].Translation - availableAvgPos);
                            estimatedPos[i] = Matrix4x4.Multiply(newPos, Matrix4x4.CreateFromQuaternion(availableAvgRot)).Translation + availableAvgPos + availableAvgPosDiff;
                            estimatedRot[i] = availableAvgRot * prevMat[i].Rotation();
                        }
                        trackerPresence[i] += 1 + (1 - (trackerStraightness[i] + straightTrackerWeight)) * 2;
                        if (trackerPresence[i] > 100)
                            trackerPresence[i] = 100;
                    }
                }

                //SET PREVIOUS MATRIX AS CURENT
                for (int i = 0; i < estimatedPos.Length; i++) {
                    prevMat[i] = Matrix4x4.Multiply(Matrix4x4.CreateFromQuaternion(estimatedRot[i]), Matrix4x4.CreateTranslation(estimatedPos[i]));
                    //draw current trackers
                    if (!float.IsNaN(estimatedRot[i].X) && !float.IsNaN(estimatedPos[i].X) && (Program.frameCount / 16) % 2 == 0)
                        Aruco.DrawAxis(prevMat[i], Utils.GetMap(trackerPresence[i], 0, 100, 1f, 0.2f));

                }

                for (int i = 0; i < estimatedRot.Length; i++) {
                    Quaternion filteredRot = trackersRotationsFilter[i].Filter(estimatedRot[i]);
                    if (float.IsNaN(filteredRot.X)) {
                        if (trackerName.Equals("waist")) {
                            trackersRotationsFilter[i] = new OneEuroFilter<Quaternion>(25);
                        } else {
                            trackersRotationsFilter[i] = new OneEuroFilter<Quaternion>(2);
                        }
                        Console.WriteLine("recenter for " + trackerIndex[i] + " " + Program.frameCount);
                        continue;
                    }
                    estimatedRot[i] = filteredRot;
                }

                Vector3[] poss = new Vector3[estimatedPos.Length];
                Quaternion[] rots = new Quaternion[estimatedRot.Length];

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
                    //Aruco.DrawAxis(Matrix4x4.Multiply(Matrix4x4.CreateFromQuaternion(rots[i]), Matrix4x4.CreateTranslation(poss[i])));
                }
                if (Program.debugSendTrackerOSC) {
                    for (int i = 0; i < trackerIndex.Length; i++) {
                        if (updateCount[i * 2] > 3) continue;
                        int id = trackerIndex[i];
                        Program.oscClient.Send($"/debug/predicted/position", id, 0, poss[i * 2].X, poss[i * 2].Z, poss[i * 2].Y);
                        Program.oscClient.Send($"/debug/predicted/position", id, 1, estimatedPos[i * 2].X, estimatedPos[i * 2].Z, estimatedPos[i * 2].Y);
                    }
                }
                //GET WHEN WAS LAST TIME WEIGHTS
                float minPosPresence = 0;
                float maxPosPresence = 0;
                Vector3 avgPosPresence = new();
                for (int i = 0; i < estimatedPos.Length; i++) {
                    //if (trackerPresence[i] < minPosPresence) minPosPresence = trackerPresence[i];
                    if (trackerPresence[i] > maxPosPresence) maxPosPresence = trackerPresence[i];
                }
                if (maxPosPresence == minPosPresence)
                    maxPosPresence++;
                float sumPosPresence = 0;
                for (int i = 0; i < estimatedPos.Length; i++) {
                    sumPosPresence += Utils.GetMap(trackerPresence[i], minPosPresence, maxPosPresence, 1f, 0f);
                }
                //SET WEIGHTED CENTER FROM TRACKERS
                if (actualAvailableTrackers == 0) {
                    avgPosPresence = prevPos;
                } else {
                    //avgPosPresence += prevPos * (1f / sumPosPresence);
                    for (int i = 0; i < estimatedPos.Length; i++) {
                        float add = Utils.GetMap(trackerPresence[i], minPosPresence, maxPosPresence, 1f / sumPosPresence, 0f);
                        avgPosPresence += poss[i] * add;
                    }
                }
                //AVERAGE VALIDATED ROTATIONS
                Quaternion prevRotPresence = new Quaternion(0, 0, 0, 0);
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
                            trackerPresenceRots[i] = 50;
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
                prevPos = avgPosPresence;
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
        }
    }
}
