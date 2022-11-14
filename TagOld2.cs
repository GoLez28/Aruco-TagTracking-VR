using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Numerics;
using System.IO;

namespace TrackingSmoothing {
    static class Tag {
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
        static Quaternion Rotation(this Matrix4x4 mat) {
            return Quaternion.CreateFromRotationMatrix(mat);
        }
        // i like everything public
        public class SingleTracker {
            public Vector3 p_pos = new Vector3();
            public Vector3 pos = new Vector3();
            public OneEuroFilter<Vector3> filter_pos = new OneEuroFilter<Vector3>(10);
            public Vector3 smooth_pos = new Vector3();
            public List<float> zList = new List<float>();
            public Matrix4x4 p_rot = new Matrix4x4();
            public Matrix4x4 rot = new Matrix4x4();
            public List<Quaternion> rotList = new List<Quaternion>();
            public bool consistentRot = true;
        }
        public class CombinedTracker {
            public SingleTracker[] singles = new SingleTracker[] {
                new SingleTracker(), new SingleTracker()
            };
            public int[] updateCount = new int[2];
            public bool newInfo = false;
            public int index = 0;
            public CombinedTracker(int index) {
                this.index = index;
            }
            public Quaternion lastRotAgreed = new Quaternion();
            public bool allAgreed = false;
            public Matrix4x4 lastSent = new Matrix4x4();
            public void Recieve(int camera, Vector3 pos, Matrix4x4 rot) {
                //TEST DEPTH
                //Random rnd = new Random();
                //if (rnd.NextDouble() < 0.3f)
                //    pos.Z -= 0.3f;
                //pos.Z += (float)Math.Sin(Program.timer.ElapsedMilliseconds / 500.0) * 0.3f;
                //Aruco.DrawAxis(Matrix4x4.Multiply(rot, Matrix4x4.CreateTranslation(pos)), camera);

                //FILTER DEPTH
                singles[camera].zList.Insert(0, pos.Z);
                if (singles[camera].zList.Count > 10)
                    singles[camera].zList.RemoveAt(singles[camera].zList.Count - 1);
                List<float> d1List = new List<float>();
                List<float> d2List = new List<float>();
                float lastDepth1 = singles[camera].zList[0];
                d1List.Add(lastDepth1);
                for (int i = 1; i < singles[camera].zList.Count; i++) {
                    float currentDepth = singles[camera].zList[i];
                    float diff1 = Math.Abs(lastDepth1 - currentDepth);
                    if (diff1 < 0.1f) {
                        d1List.Add(currentDepth);
                        lastDepth1 = currentDepth;
                    } else {
                        d2List.Add(currentDepth);
                    }
                }
                if (d1List.Count < d2List.Count) {
                    pos.Z = d2List[0];
                }

                //TEST ROTATION
                //Random rnd = new Random();
                //if (rnd.NextDouble() < 0.4f) {
                //    Matrix4x4 rotMat = Matrix4x4.CreateFromAxisAngle(new Vector3(0, -1, 0f), (float)Math.PI * 0.5f);
                //    rot = Matrix4x4.Multiply(rotMat, rot);
                //    rotMat = Matrix4x4.CreateFromAxisAngle(new Vector3(-0.8f, 0, 0.7f), (float)Math.PI * 0.25f);
                //    rot = Matrix4x4.Multiply(rotMat, rot);
                //    rotMat = Matrix4x4.CreateFromAxisAngle(new Vector3(0, -0.3f, 0), (float)Math.PI * 0.25f);
                //    rot = Matrix4x4.Multiply(rotMat, rot);
                //}

                //FILTER ROTATION
                if (updateCount[camera] != 0) singles[camera].rotList.Clear();
                singles[camera].rotList.Insert(0, Quaternion.CreateFromRotationMatrix(rot));
                if (singles[camera].rotList.Count > 10)
                    singles[camera].rotList.RemoveAt(singles[camera].rotList.Count - 1);
                List<Quaternion> r1List = new List<Quaternion>();
                List<Quaternion> r2List = new List<Quaternion>();
                Quaternion lastRot1 = singles[camera].rotList[0];
                r1List.Add(lastRot1);
                bool switched = false;
                for (int i = 1; i < singles[camera].rotList.Count; i++) {
                    Quaternion curRot = singles[camera].rotList[i];
                    float rotDiff = Quaternion.Dot(Quaternion.Inverse(lastRot1) * curRot, Quaternion.Identity);
                    if (rotDiff < 0.9f) {
                        switched = !switched;
                    }
                    if (switched) r2List.Add(curRot);
                    else r1List.Add(curRot);
                    lastRot1 = curRot;
                }
                singles[camera].consistentRot = false;
                if (r1List.Count < r2List.Count) {
                    rot = Matrix4x4.CreateFromQuaternion(r2List[0]);
                    if (r2List.Count > 7) singles[camera].consistentRot = true;
                } else {
                    if (r1List.Count > 7) singles[camera].consistentRot = true;
                }

                //APPLY PREVIOUS
                singles[camera].p_pos = singles[camera].pos;
                singles[camera].pos = pos;
                singles[camera].p_rot = singles[camera].rot;
                singles[camera].rot = rot;
                //APPLY ONE EURO FILTER TO Z
                singles[camera].smooth_pos = singles[camera].filter_pos.Filter(pos);
                //RESET UPDATE
                updateCount[camera] = 0;
                newInfo = true;
            }
            public Matrix4x4 Obtain() {
                //for now just the average
                //i plan to use projection if the two are available

                //USE DEPTH FROM THE FILTERED VERSION
                float depth1 = singles[0].smooth_pos.Z;
                float depth2 = singles[1].smooth_pos.Z;

                //GUESS WHICH CAM IS CLOSER BASED ON DEPTH
                int closestCam = 0;
                if ((depth2 < depth1 && updateCount[1] <= 2) || (updateCount[0] > 2 && updateCount[1] <= 2))
                    closestCam = 1;
                Matrix4x4 rot1 = singles[0].rot;
                Matrix4x4 rot2 = singles[1].rot;
                Matrix4x4[] pos = new Matrix4x4[] {
                    Matrix4x4.Multiply(rot1, Matrix4x4.CreateTranslation(singles[0].pos.X * cameras[0].depthMult, singles[0].pos.Y * cameras[0].depthMult, depth1 * cameras[0].depthMult)),
                    Matrix4x4.Multiply(rot2, Matrix4x4.CreateTranslation(singles[1].pos.X * cameras[1].depthMult, singles[1].pos.Y * cameras[1].depthMult, depth2 * cameras[1].depthMult))
                };
                pos[0] = Matrix4x4.Multiply(pos[0], cameras[0].matrix);
                pos[1] = Matrix4x4.Multiply(pos[1], cameras[1].matrix);
                Matrix4x4 result = pos[closestCam];

                Quaternion[] quat = new Quaternion[] {
                    Quaternion.CreateFromRotationMatrix(pos[0]),
                    Quaternion.CreateFromRotationMatrix(pos[1])
                };
                float rotDiff = Quaternion.Dot(Quaternion.Inverse(quat[0]) * quat[1], Quaternion.Identity);
                if (rotDiff > 0.9f || true) {
                    lastRotAgreed = quat[closestCam];
                    allAgreed = true;
                } else {
                    allAgreed = false;
                    rotDiff = Quaternion.Dot(Quaternion.Inverse(lastRotAgreed) * quat[0], Quaternion.Identity);
                    if ((rotDiff > 0.9f && updateCount[0] < 2) || (updateCount[0] < 2 && updateCount[1] >= 2)) {
                        lastRotAgreed = quat[0];
                    }
                    rotDiff = Quaternion.Dot(Quaternion.Inverse(lastRotAgreed) * quat[1], Quaternion.Identity);
                    if ((rotDiff < 0.9f && updateCount[1] < 2) || (updateCount[1] < 2 && updateCount[0] >= 2)) {
                        lastRotAgreed = quat[1];
                    }
                    if (rotDiff < 0.9f) {
                        Console.WriteLine();
                    }
                }
                Matrix4x4 newRes = Matrix4x4.CreateFromQuaternion(lastRotAgreed);
                newRes.M41 = result.M41;
                newRes.M42 = result.M42;
                newRes.M43 = result.M43;
                result = newRes;


                Vector3 newPos = new Vector3();
                int lost = 0;
                if (updateCount[0] > 2) {
                    newPos = pos[1].Translation;
                    lost++;
                }
                if (updateCount[1] > 2) {
                    newPos = pos[0].Translation;
                    lost++;
                }
                if (newInfo) {
                    updateCount[1]++;
                    updateCount[0]++;

                    newInfo = false;
                }
                if (lost == 0) {
                    float z1 = 1f / depth1;
                    float z2 = 1f / depth2;
                    float zS = z1 + z2;
                    newPos.X = (pos[0].M41 * z1 + pos[1].M41 * z2) / zS;
                    newPos.Y = (pos[0].M42 * z1 + pos[1].M42 * z2) / zS;
                    newPos.Z = (pos[0].M43 * z1 + pos[1].M43 * z2) / zS;
                } else if (lost == 2) {
                    return lastSent;
                }
                //------------------------Test--------------------------------
                //closestCam = (int)(Program.timer.ElapsedMilliseconds / 500) % 2;
                //newPos.X = pos[closestCam].M41;
                //newPos.Y = pos[closestCam].M42;
                //newPos.Z = pos[closestCam].M43;
                //------------------------------------------------------------
                if (float.IsNaN(newPos.X))
                    newPos = new Vector3();
                result.M41 = newPos.X;
                result.M42 = newPos.Y;
                result.M43 = newPos.Z;
                lastSent = result;

                //------------------------Test--------------------------------
                //Aruco.DrawAxis(result);
                //------------------------------------------------------------
                return result;
            }
        }
        public class ClusterTracker {
            public CombinedTracker[] trackers;
            public OneEuroFilter<Quaternion>[] trackersRotationsFilter;
            public int[] trackerIndex = new int[] { 0, 1, 2, 3 };
            public Vector3[] trackerOffsets = new Vector3[] {
                new Vector3 (0f, 0f, -0.05f),
                new Vector3 (0f, 0f, -0.05f),
                new Vector3 (0f, 0f, -0.05f),
                new Vector3 (0f, 0f, -0.05f)
            };
            public float[] trackerRotations = new float[] { 0f, (float)Math.PI / 2, (float)Math.PI, (float)Math.PI * 1.5f };
            public int[] trackerPresence = new int[4];
            public int[] updateCount = new int[4];
            public int[] updateCountp = new int[4];
            public bool newInfo = false;
            public Matrix4x4[] prevMat;

            public Quaternion prevRot = new Quaternion();
            public int lastRotCount = 0;
            public Vector3 prevPos = new Vector3();
            public OneEuroFilter<Vector3> smoothedPos = new OneEuroFilter<Vector3>(5);
            public OneEuroFilter<Quaternion> smoothedRot = new OneEuroFilter<Quaternion>(15);
            public string trackerName = "unknown";
            float minScore = float.MaxValue;
            public ClusterTracker(string name, int[] ids, Vector3[] ofss, float[] rots) {
                trackerName = name;
                trackerIndex = ids;
                trackerOffsets = ofss;
                trackerRotations = rots;
                updateCount = new int[ids.Length];
                trackers = new CombinedTracker[ids.Length];
                prevMat = new Matrix4x4[ids.Length];
                trackersRotationsFilter = new OneEuroFilter<Quaternion>[ids.Length];
                trackerPresence = new int[ids.Length];
                if (name.Equals("waist")) {
                    smoothedRot = new OneEuroFilter<Quaternion>(25);
                    smoothedPos = new OneEuroFilter<Vector3>(10);
                }
                for (int i = 0; i < ids.Length; i++) {
                    trackers[i] = new CombinedTracker(ids[i]);
                    prevMat[i] = new();
                    if (name.Equals("waist")) {
                        trackersRotationsFilter[i] = new OneEuroFilter<Quaternion>(100);
                    } else {
                        trackersRotationsFilter[i] = new OneEuroFilter<Quaternion>(2);
                    }
                };
            }
            public Matrix4x4 Obtain() {
                if (!trackerName.Equals("waist")) return new();

                //INCREMENT COUNT SINCE
                if (newInfo) {
                    for (int k = 0; k < trackerIndex.Length; k++) {
                        updateCount[k]++;
                    }
                }

                //INITIALIZE VARIABLES
                Vector3[] estimatedPos = new Vector3[trackers.Length];
                Quaternion[] estimatedRot = new Quaternion[trackers.Length];
                Vector3[] realPos = new Vector3[trackers.Length];
                Quaternion[] realRot = new Quaternion[trackers.Length];
                Vector3 availableAvgPos = new();
                Vector3 availableAvgPosDiff = new();
                Quaternion availableAvgRot = Quaternion.Identity;
                int availableCount = 0;
                List<Quaternion> posibleRots = new List<Quaternion>();
                List<List<Quaternion>> posibleRotsCount = new List<List<Quaternion>>();
                List<List<int>> posibleRotsCountId = new List<List<int>>();
                int actualAvailableTrackers = 0;
                if (lastRotCount < 10) { //add previous rotation
                    posibleRots.Add(prevRot);
                    posibleRotsCount.Add(new List<Quaternion>());
                    posibleRotsCount[0].Add(prevRot);
                    posibleRotsCountId.Add(new List<int>());
                    posibleRotsCountId[0].Add(-1);
                }
                //PRELOAD VARIABLES
                Matrix4x4[] trackersMat = new Matrix4x4[trackers.Length];
                Matrix4x4[] trackerRotationsMat = new Matrix4x4[trackers.Length];
                Matrix4x4[] trackerOffsetsMat = new Matrix4x4[trackers.Length];
                for (int i = 0; i < trackers.Length; i++) {
                    trackersMat[i] = trackers[i].Obtain();
                    trackerRotationsMat[i] = Matrix4x4.CreateFromAxisAngle(new Vector3(0, -1, 0), trackerRotations[i]);
                    trackerOffsetsMat[i] = Matrix4x4.CreateTranslation(trackerOffsets[i]);
                }

                //CHECK FOR PARS
                for (int i = 0; i < trackers.Length; i++) {
                    if (updateCount[i] == 1) {
                        actualAvailableTrackers++;
                        Matrix4x4 newMat = trackersMat[i];
                        Matrix4x4 result = Matrix4x4.Multiply(trackerOffsetsMat[i], newMat);
                        Matrix4x4 rotMat = trackerRotationsMat[i];
                        result = Matrix4x4.Multiply(rotMat, result);
                        bool added = false;
                        for (int j = 0; j < posibleRots.Count; j++) {
                            Quaternion qDif = Quaternion.Inverse(posibleRots[j]) * result.Rotation();
                            float rotDiff = Quaternion.Dot(qDif, Quaternion.Identity);
                            if (rotDiff > 0.95f) {
                                posibleRotsCount[j].Add(result.Rotation());
                                posibleRotsCountId[j].Add(i);
                                posibleRots[j] = Utils.QLerp(posibleRots[j], result.Rotation(), 0.25f);
                                added = true;
                                break;
                            }
                        }
                        if (!added) {
                            posibleRots.Add(result.Rotation());
                            List<Quaternion> newAdd = new List<Quaternion>();
                            newAdd.Add(result.Rotation());
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

                //INCREMENT LAST SEEN COUNT FOR TAGS THAT FAILED PAR CHECK
                for (int i = 0; i < trackers.Length; i++) {
                    if (updateCount[i] == 1) {
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
                for (int i = 0; i < trackers.Length; i++) {
                    if (updateCount[i] == 1) {
                        Matrix4x4 newMat = trackersMat[i];
                        bool allowTransform = false;
                        for (int j = 0; j < posibleRotsCountId[maxId].Count; j++) {
                            if (posibleRotsCountId[maxId][j] == i) {
                                allowTransform = true;
                                break;
                            }
                        }
                        if (updateCountp[i] == 1 && allowTransform) {
                            Vector3 newMatPos = newMat.Translation;
                            Vector3 posDiff = newMatPos - prevMat[i].Translation;
                            availableAvgPosDiff += posDiff;
                            availableAvgPos += newMatPos;
                            Quaternion rotDiff = newMat.Rotation() * Quaternion.Inverse(prevMat[i].Rotation());
                            availableAvgRot = Utils.QLerp(availableAvgRot, rotDiff, availableCount == 0 ? 1f : 0.5f);
                            availableCount++;
                        } else {
                            for (int j = 0; j < 5; j++) { //rectify filtering
                                trackersRotationsFilter[i].Filter(estimatedRot[i]);
                            }
                        }
                        estimatedPos[i] = newMat.Translation;
                        estimatedRot[i] = newMat.Rotation();
                        trackerPresence[i]--;
                        if (trackerPresence[i] < 0)
                            trackerPresence[i] = 0;
                    }
                }

                if (availableCount != 0) {
                    availableAvgPos /= availableCount;
                    availableAvgPosDiff /= availableCount;
                }

                //GET DIFFERENCE FROM LAST TIME, EVEN THOUGHT IT IS FAR AWAY, AND SET ABSOLUTE 
                if (availableCount == 0) {
                    for (int i = 0; i < trackers.Length; i++) {
                        if (updateCount[i] == 1) {
                            Matrix4x4 newMat = trackersMat[i];
                            newMat = Matrix4x4.Multiply(trackerOffsetsMat[i], newMat);
                            newMat = Matrix4x4.Multiply(trackerRotationsMat[i], newMat);
                            Vector3 newMatPos = newMat.Translation;
                            Vector3 posDiff = newMatPos - prevPos;
                            availableAvgPos += newMatPos;
                            availableAvgPosDiff += posDiff;
                            Quaternion rotDiff = newMat.Rotation() * Quaternion.Inverse(prevRot);
                            availableAvgRot = Utils.QLerp(availableAvgRot, rotDiff, availableCount == 0 ? 1f : 0.5f);
                            availableCount++;
                            estimatedPos[i] = newMatPos;
                            estimatedRot[i] = newMat.Rotation();
                            trackerPresence[i] -= 10;
                            if (trackerPresence[i] < 0)
                                trackerPresence[i] = 0;
                        } else if (updateCount[i] > 1) {
                            trackerPresence[i] += 10;
                            if (trackerPresence[i] > 50)
                                trackerPresence[i] = 50;
                        }
                    }
                }
                if (availableCount != 0) {
                    availableAvgPos /= availableCount;
                    availableAvgPosDiff /= availableCount;
                }

                //GET PREDICTED MATRIX FOR TAGS THAT ARE NOT SEEN
                for (int i = 0; i < trackers.Length; i++) {
                    if (updateCount[i] > 1) {
                        if (availableCount == 0) {
                            estimatedPos[i] = prevMat[i].Translation;
                            estimatedRot[i] = prevMat[i].Rotation();
                        } else {
                            Matrix4x4 newPos = Matrix4x4.CreateTranslation(prevMat[i].Translation - availableAvgPos);
                            estimatedPos[i] = Matrix4x4.Multiply(newPos, Matrix4x4.CreateFromQuaternion(availableAvgRot)).Translation + availableAvgPos + availableAvgPosDiff;
                            estimatedRot[i] = availableAvgRot * prevMat[i].Rotation();
                        }
                        trackerPresence[i]++;
                        if (trackerPresence[i] > 50)
                            trackerPresence[i] = 50;
                    }
                }
                for (int i = 0; i < estimatedRot.Length; i++) {
                    Quaternion filteredRot = trackersRotationsFilter[i].Filter(estimatedRot[i]);
                    if (float.IsNaN(filteredRot.X)) {
                        if (trackerName.Equals("waist")) {
                            trackersRotationsFilter[i] = new OneEuroFilter<Quaternion>(100);
                        } else {
                            trackersRotationsFilter[i] = new OneEuroFilter<Quaternion>(2);
                        }
                        Console.WriteLine("recenter for " + trackerIndex[i] + " " + Program.frameCount);
                        continue;
                    }
                    estimatedRot[i] = filteredRot;
                }

                //SET PREVIOUS MATRIX AS CURENT
                for (int i = 0; i < estimatedPos.Length; i++) {
                    prevMat[i] = Matrix4x4.Multiply(Matrix4x4.CreateFromQuaternion(estimatedRot[i]), Matrix4x4.CreateTranslation(estimatedPos[i]));
                    //draw current trackers
                    if (!float.IsNaN(estimatedRot[i].X) && !float.IsNaN(estimatedPos[i].X) && Program.frameCount % 2 == 0)
                        Aruco.DrawAxis(prevMat[i], Utils.GetMap(trackerPresence[i], 0, 50, 1f, 0.5f));

                }

                Vector3[] poss = new Vector3[estimatedPos.Length];
                Quaternion[] rots = new Quaternion[estimatedRot.Length];

                //GET CENTERED TRACKERS MATRIX
                for (int i = 0; i < estimatedPos.Length; i++) {
                    Matrix4x4 mat = prevMat[i];
                    mat = Matrix4x4.Multiply(trackerOffsetsMat[i], mat);
                    mat = Matrix4x4.Multiply(trackerRotationsMat[i], mat);
                    poss[i] = mat.Translation;
                    if (float.IsNaN(poss[i].X))
                        poss[i] = new();
                    rots[i] = mat.Rotation();
                    if (float.IsNaN(rots[i].X))
                        rots[i] = Quaternion.Identity;
                    Aruco.DrawAxis(Matrix4x4.Multiply(Matrix4x4.CreateFromQuaternion(rots[i]), Matrix4x4.CreateTranslation(poss[i])));
                }

                //GET WHEN WAS LAST TIME WEIGHTS
                int minPosPresence = int.MaxValue;
                int maxPosPresence = 0;
                Vector3 avgPosPresence = new();
                for (int i = 0; i < estimatedPos.Length; i++) {
                    if (trackerPresence[i] < minPosPresence) minPosPresence = trackerPresence[i];
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
                    for (int i = 0; i < estimatedPos.Length; i++) {
                        float add = Utils.GetMap(trackerPresence[i], minPosPresence, maxPosPresence, 1f / sumPosPresence, 0f);
                        avgPosPresence += poss[i] * add;
                    }
                }
                //AVERAGE VALIDATED ROTATIONS
                Quaternion prevRotPresence = Quaternion.Identity;
                if (maxId == -1 || actualAvailableTrackers == 0) {
                    prevRotPresence = prevRot; //do this in case nothing is recognized
                    lastRotCount++;
                } else {
                    if (posibleRotsCountId[maxId].Count == 1 && posibleRotsCountId[maxId][0] == -1) { //if the only rot was the previous
                        lastRotCount++;
                    } else {
                        lastRotCount = 0; //set 0 if sure
                    }
                    int[] trackerPresenceRots = new int[posibleRotsCount[maxId].Count];
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
                        Quaternion qm = Utils.QLerp(Quaternion.Identity, posibleRotsCount[maxId][i], add);
                        //Quaternion qm = Utils.QLerp(Quaternion.Identity, posibleRotsCount[maxId][i], 1f / posibleRotsCount[maxId].Count);
                        prevRotPresence = qm * prevRotPresence;
                    }
                }

                float smoothRot = 0.5f;
                if (trackerName.Equals("waist")) {
                    smoothRot = 0.2f;
                }
                if (!float.IsNaN(prevRotPresence.X)) {
                    //avgRot = smoothedRot.Filter(avgRot);
                    //if (float.IsNaN(prevRot.X))
                    //    prevRot = prevRotPresence;
                    //prevRotPresence = Quaternion.Lerp(prevRot, prevRotPresence, smoothRot);
                    //prevRotPresence = Quaternion.Normalize(prevRotPresence);

                    prevRotPresence = smoothedRot.Filter(prevRotPresence);
                }
                if (!float.IsNaN(avgPosPresence.X))
                    avgPosPresence = smoothedPos.Filter(avgPosPresence);

                //SET PREVIOUS POS AND ROT
                prevPos = avgPosPresence;
                prevRot = prevRotPresence;
                //GET MATRIX FROM VECTOR AND QUATERNION
                Matrix4x4 result2 = Matrix4x4.Multiply(Matrix4x4.CreateFromQuaternion(prevRotPresence), Matrix4x4.CreateTranslation(avgPosPresence));
                //if ((Program.frameCount / 2) % 1 == 0 && !float.IsNaN(avgPosPresence.X))
                //    Aruco.DrawAxis(result2);
                //UPDATE PREVIOUS COUNT
                if (newInfo) {
                    for (int k = 0; k < trackerIndex.Length; k++) {
                        updateCountp[k] = updateCount[k];
                    }
                    newInfo = false;
                }
                return result2;

            }
        }
        public class Camera {
            public Matrix4x4 matrix = new Matrix4x4();
            public float minScore = Single.MaxValue;
            public float depthMult = 1f;
            public float testDepthMult = 1f;
            public float[,] smoothening = new float[7, 3];
            public Camera(Matrix4x4 m) {
                matrix = m;
            }
        }
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
            new CombinedTracker(0), new CombinedTracker(1), new CombinedTracker(2), new CombinedTracker(3), new CombinedTracker(4), new CombinedTracker(5), new CombinedTracker(6)
        };
        public static Camera[] cameras = new Camera[] {
            //matrixes will be updated after
            new Camera(new Matrix4x4(
                -0.611f, 0.489f, -0.623f, -0.369f,
                0.790f, 0.324f, -0.520f, 0.026f,
                -0.053f, -0.81f, -0.584f, 2.268f,
                0.0f, 0.0f, 0.0f, 1.0f ) ),
            new Camera(new Matrix4x4(
                -0.611f, 0.489f, -0.623f, -0.369f,
                0.790f, 0.324f, -0.520f, 0.026f,
                -0.053f, -0.81f, -0.584f, 2.268f,
                0.0f, 0.0f, 0.0f, 1.0f ) )
        };
        public static bool endedSearching = true;
        public static double saveMatTime = -40000;

        public static double lastFrameTime = 0;
        public static int lastCamera = 0;
        public static int lastIndex = 0;
        public static double[] cameraTPS = new double[2];
        public static void SaveMatrix() {
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
        public static List<RecieveTag> tagsList = new List<RecieveTag>();
        public static void RecieveTrackerAsync(int index, int camera, Matrix4x4 rot, Vector3 pos) {
            tagsList.Add(new RecieveTag(rot, pos, index, camera));
        }
        public static void Update() {
            List<RecieveTag> tagsListCopy = tagsList;
            tagsList = new List<RecieveTag>();
            for (int i = 0; i < tagsListCopy.Count; i++) {
                RecieveTag tag = tagsListCopy[i];
                RecieveTracker(tag.index, tag.camera, tag.rot, tag.pos);
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
            //Vector3 pos = new Vector3((float)info[5] / 1000f, (float)info[9] / 1000f, (float)info[13] / 1000f);
            //Matrix4x4 rot = new Matrix4x4(
            //    (float)info[2], (float)info[3], (float)info[4], 0f,
            //    (float)info[6], (float)info[7], (float)info[8], 0f,
            //    (float)info[10], (float)info[11], (float)info[12], 0f,
            //    0f, 0f, 0f, 1f
            //);
            //Matrix4x4 rot = new Matrix4x4(
            //    (float)info[2], (float)info[6], (float)info[10], 0f,
            //    (float)info[3], (float)info[7], (float)info[11], 0f,
            //    (float)info[4], (float)info[8], (float)info[12], 0f,
            //    0f, 0f, 0f, 1f
            //);
            if (Program.timer.ElapsedMilliseconds - saveMatTime < 20000) {
                //if ((saveMat1 && camera == 0) || (saveMat2 && camera == 1)) {
                //    double[] avg = new double[] {
                //            (smoothening[camera,0,0] + smoothening[camera,2,0] + smoothening[camera,4,0] + smoothening[camera,6,0]) / 4.0,
                //            (smoothening[camera,0,1] + smoothening[camera,2,1] + smoothening[camera,4,1] + smoothening[camera,6,1]) / 4.0,
                //            (smoothening[camera,0,2] + smoothening[camera,2,2] + smoothening[camera,4,2] + smoothening[camera,6,2]) / 4.0
                //        };
                //    double[][] newMat = new double[][] {
                //            new double[]{ rot[0][0], rot[0][1], rot[0][2], avg[0] },
                //            new double[]{ rot[1][0], rot[1][1], rot[1][2], avg[1] },
                //            new double[]{ rot[2][0], rot[2][1], rot[2][2], avg[2] },
                //            new double[]{0.0, 0.0, 0.0, 1.0 }
                //        };
                //    if (camera == 0) {
                //        cameraMat1 = newMat;
                //        cameraMat1 = MatrixInverse(cameraMat1);
                //        saveMat1 = false;
                //    } else if (camera == 1) {
                //        cameraMat2 = newMat;
                //        cameraMat2 = MatrixInverse(cameraMat2);
                //        saveMat2 = false;
                //    }
                //    Console.WriteLine($"Saved! {camera}");
                //} else {
                if (index == 0 || index == 2 || index == 4 || index == 6) {
                    if (index < 7) {
                        cameras[camera].smoothening[index, 0] = cameras[camera].smoothening[index, 0] == 0 ? pos.X : (cameras[camera].smoothening[index, 0] + pos.X) / 2.0f;
                        cameras[camera].smoothening[index, 1] = cameras[camera].smoothening[index, 1] == 0 ? pos.Y : (cameras[camera].smoothening[index, 1] + pos.Y) / 2.0f;
                        cameras[camera].smoothening[index, 2] = cameras[camera].smoothening[index, 2] == 0 ? pos.Z : (cameras[camera].smoothening[index, 2] + pos.Z) / 2.0f;
                        //smoothening[camera, index, 1] = smoothening[camera, index, 1] == 0 ? pos[1] : (smoothening[camera, index, 1] + pos[1]) / 2.0;
                        //smoothening[camera, index, 2] = smoothening[camera, index, 2] == 0 ? pos[2] : (smoothening[camera, index, 2] + pos[2]) / 2.0;
                    }
                    Vector3 vec = new Vector3(
                        cameras[camera].smoothening[index, 0],
                        cameras[camera].smoothening[index, 1],
                        cameras[camera].smoothening[index, 2]);
                    Matrix4x4 vecMat = Matrix4x4.CreateTranslation(vec);
                    Matrix4x4 newMat = Matrix4x4.Multiply(rot, vecMat);
                    Matrix4x4 invMat;
                    Matrix4x4.Invert(newMat, out invMat);
                    string message = $"                              ";
                    ApplyNewMatrix(camera, 1f, invMat, message);
                    Random rnd = new Random();
                    //System.Diagnostics.Stopwatch sw = new System.Diagnostics.Stopwatch();
                    //sw.Start();
                    for (int i = 0; i < 50; i++) {
                        //cameras[camera].matrix.CopyTo(invMat);
                        Matrix4x4 newRot = Matrix4x4.CreateFromYawPitchRoll(
                            (float)(rnd.NextDouble() - 0.5) / 1000f,
                            (float)(rnd.NextDouble() - 0.5) / 1000f,
                            (float)(rnd.NextDouble() - 0.5) / 1000f);
                        Vector3 newTran = new Vector3(
                            (float)(rnd.NextDouble() - 0.5) / 100f,
                            (float)(rnd.NextDouble() - 0.5) / 100f,
                            (float)(rnd.NextDouble() - 0.5) / 100f);
                        Matrix4x4 newRotd = Matrix4x4.Multiply(newRot, Matrix4x4.CreateTranslation(newTran));
                        float depthMult = ((float)(rnd.NextDouble() - 0.5) / 1000f) + 1;
                        //depthMult = 1f;
                        cameras[camera].testDepthMult = depthMult;
                        //get from exixsting matrix
                        message = $"by exsisting rnd {depthMult}             ";
                        Matrix4x4 rndMat = Matrix4x4.Multiply(cameras[camera].matrix, newRotd);
                        while (ApplyNewMatrix(camera, depthMult, rndMat, message)) {
                            message = $"by repeating exsisting rnd {depthMult}             ";
                            rndMat = Matrix4x4.Multiply(cameras[camera].matrix, newRotd);
                        }

                        //get from current matrix
                        message = $"by new rnd {depthMult}             ";
                        rndMat = Matrix4x4.Multiply(invMat, newRotd);
                        while (ApplyNewMatrix(camera, depthMult, rndMat, message)) {
                            message = $"by repeating new rnd {depthMult}             ";
                            rndMat = Matrix4x4.Multiply(invMat, newRotd);
                        }
                    }
                    //sw.Stop();
                    //Console.WriteLine(sw.Elapsed.TotalMilliseconds);
                }
                //}
            } else {
                if (!endedSearching) {
                    endedSearching = true;
                    cameras[0].depthMult = cameras[0].testDepthMult;
                    cameras[1].depthMult = cameras[1].testDepthMult;
                    Console.WriteLine("Ended Searching for matrices");
                    SaveMatrix();
                }
            }
            for (int i = 0; i < trackers.Length; i++) {
                for (int j = 0; j < trackers[i].trackerIndex.Length; j++) {
                    if (trackers[i].trackerIndex[j] == index) {
                        trackers[i].trackers[j].Recieve(camera, pos, rot);
                        trackers[i].updateCount[j] = 0;
                        trackers[i].newInfo = true;
                        break;
                    }
                }
            }
            for (int k = 0; k < combinedTrackers.Length; k++) {
                if (combinedTrackers[k].index == index) {
                    combinedTrackers[k].Recieve(camera, pos, rot);
                    break;
                }
            }
            //finalPos[0][0] -= 2.6420467248736776;
            //finalPos[1][0] -= 6.4147866527909132;
            //finalPos[2][0] -= 5.01607364667175;
            //finalPos[0][0] *= 0.25f;
            //finalPos[1][0] *= 0.25f;
            //finalPos[2][0] *= 0.25f;
            //finalPos[0][0] = pos[0];
            //finalPos[1][0] = pos[1];
            //finalPos[2][0] = pos[2];
            //Program.oscClient.Send("/VMT/Room/Unity", index + 1 + camera * 3, 1, 0f,
            //                            (float)finalPos[0][0], (float)finalPos[2][0], (float)finalPos[1][0], //1f, 1.7f, 1f
            //                            1f, 0f, 0f, 0f);
            //Console.SetCursorPosition(0, 0);
            //Console.WriteLine($"{pos[0] / 1f}, {pos[1] / 1f}, {pos[2] / 1f}                                    ");
            //Console.WriteLine($"{finalPos[0][0] / 1f}, {finalPos[1][0] / 1f}, {finalPos[2][0] / 1f}                                    ");
            //Console.WriteLine("                                              ");
            //if (Program.timer.ElapsedMilliseconds - prevTimer > 1000) {
            //    prevTimer = Program.timer.ElapsedMilliseconds;
            //    countdown--;
            //    if (countdown < 0) {
            //        countdown = 10;
            //        Console.WriteLine($"Snapshot {snapshotCount}");
            //        Console.WriteLine($"{finalPos[0, 0] / 1f}, {finalPos[1, 0] / 1f}, {finalPos[2, 0] / 1f}");
            //        Console.WriteLine($"{Program.hmdPos[0] / 1f}, {Program.hmdPos[1] / 1f}, {Program.hmdPos[2] / 1f}");
            //        snapshotCount++;
            //    } else {
            //        Console.WriteLine(countdown);
            //    }
            //}
            //bool found = false;
            //for (int i = 0; i < combinedTrackers.Count; i++) {
            //    if (combinedTrackers[i].index == index) {
            //        combinedTrackers[i].Recieve(camera, new float[] { pos[0], pos[1], pos[2] }, rot);
            //        found = true;
            //        break;
            //    }
            //}
            //if (!found) {
            //    CombinedTracker combinedTracker = new CombinedTracker();
            //    combinedTracker.index = index;
            //    combinedTrackers.Add(combinedTracker);
            //}
        }

        private static bool ApplyNewMatrix(int camera, float depthMult, Matrix4x4 rndMat, string message) {
            float dist = 0;
            if (camera == 0)
                dist = GetDistanceFromEachTracker(camera, rndMat, cameras[1].matrix);
            else if (camera == 1)
                dist = GetDistanceFromEachTracker(camera, cameras[0].matrix, rndMat);
            dist += GetDistanceFromZero(camera, rndMat) * 0.25f;
            if (dist < cameras[camera].minScore) {
                cameras[camera].minScore = dist;
                cameras[camera].matrix = rndMat;
                cameras[camera].testDepthMult = depthMult;
                //Console.CursorTop--;
                Console.WriteLine($"Updated matrix for cam {camera}: {dist} " + message);
                return true;
            }
            return false;
        }

        private static float GetDistanceFromEachTracker(int camera, Matrix4x4 mat1, Matrix4x4 mat2, bool modDepth = true) {
            float distSum = 0;
            float depthMul1 = 1f;
            float depthMul2 = 1f;
            if (modDepth) {
                depthMul1 = cameras[0].testDepthMult;
                depthMul2 = cameras[1].testDepthMult;
            }
            for (int i = 0; i < combinedTrackers.Length; i++) {
                CombinedTracker tracker = combinedTrackers[i];
                if (!(tracker.index == 0 || tracker.index == 2 || tracker.index == 4 || tracker.index == 6)) continue; //to not get garbage
                Vector3 pos1 = tracker.singles[0].pos * depthMul1;
                pos1 = Vector3.Transform(pos1, mat1);
                //Quaternion baseq = Quaternion.CreateFromYawPitchRoll(90f, 0, 0);
                //Quaternion q1 = Quaternion.CreateFromRotationMatrix(Matrix4x4.CreateTranslation(pos1));
                //float qdist1 = Math.Abs(baseq.X - q1.X) + Math.Abs(baseq.Y - q1.Y) + Math.Abs(baseq.Z - q1.Z) + Math.Abs(baseq.W - q1.W);
                //distSum += qdist1 * 0.5f;
                Vector3 pos2 = tracker.singles[1].pos * depthMul2;
                pos2 = Vector3.Transform(pos2, mat2);
                //Quaternion q2 = Quaternion.CreateFromRotationMatrix(Matrix4x4.CreateTranslation(pos1));
                //float qdist2 = Math.Abs(baseq.X - q2.X) + Math.Abs(baseq.Y - q2.Y) + Math.Abs(baseq.Z - q2.Z) + Math.Abs(baseq.W - q2.W);
                //distSum += qdist2 * 0.5f;
                float dist = Utils.GetDistance(pos1.X, pos1.Y, pos1.Z, pos2.X, pos2.Y, pos2.Z);
                distSum += dist;
            }
            return distSum;
        }
        private static float GetDistanceFromZero(int camera, Matrix4x4 mat, bool modDepth = true) {
            float distSum = 0;
            float depthMul = 1f;
            if (modDepth) {
                depthMul = cameras[camera].testDepthMult;
            }
            for (int i = 0; i < combinedTrackers.Length; i++) {
                CombinedTracker tracker = combinedTrackers[i];
                if (!(tracker.index == 0 || tracker.index == 2 || tracker.index == 4 || tracker.index == 6)) continue; //to not get garbage
                Vector3 pos1 = tracker.singles[camera].pos * depthMul;
                pos1 = Vector3.Transform(pos1, mat);
                float dist = Utils.GetDistance(pos1.X, pos1.Y, pos1.Z, 0f, 0f, 0f);
                distSum += dist;
                //emphazises in floor level
                dist = Math.Abs(pos1.Z);
                distSum += dist;
            }
            return distSum;
        }

        public static void SendTrackers() {
            if (Program.timer.ElapsedMilliseconds - saveMatTime < 20000) {
                for (int i = 0; i < combinedTrackers.Length; i++) {
                    if (!(combinedTrackers[i].index == 0 || combinedTrackers[i].index == 4)) continue;
                    //Matrix4x4 mat = combinedTrackers[i].Obtain();
                    Vector3 pos = combinedTrackers[i].singles[0].pos * cameras[0].depthMult;
                    Quaternion q = combinedTrackers[i].singles[0].rot.Rotation();
                    Program.oscClient.Send("/VMT/Room/Unity", (i / 4) + 1, 1, 0f,
                                                pos.X + Program.offsetX, pos.Z + Program.offsetY, pos.Y + Program.offsetZ, //1f, 1.7f, 1f
                                                -q.X, -q.Z, -q.Y, q.W); //idk, this works lol //XZYW
                }
                return;
            }
            for (int i = 0; i < trackers.Length; i++) {
                Matrix4x4 mat = trackers[i].Obtain();
                Vector3 pos = mat.Translation;
                Quaternion q = Quaternion.CreateFromRotationMatrix(mat);
                Vector4 headPos = Program.hmdList[0];
                pos.X += Program.hmdOffset[0];
                pos.Y += Program.hmdOffset[1];
                pos.Z += Program.hmdOffset[2];
                pos.X -= ((headPos.X + Program.hmdOffset[0]) - Program.hmdPos[0]) * Program.trackerFollowWeight[i];
                pos.Y -= ((headPos.Y + Program.hmdOffset[1]) - Program.hmdPos[1]) * Program.trackerFollowWeight[i];
                pos.Z -= ((headPos.Z + Program.hmdOffset[2]) - Program.hmdPos[2]) * Program.trackerFollowWeight[i];
                Program.oscClient.Send("/VMT/Room/Unity", i + 1, 1, 0f,
                                            pos.X + Program.offsetX, pos.Z + Program.offsetY, pos.Y + Program.offsetZ, //1f, 1.7f, 1f
                                            -q.X, -q.Z, -q.Y, q.W); //idk, this works lol //XZYW
            }
        }
    }
}
