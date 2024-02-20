using System;
using System.Collections.Generic;
using System.Numerics;

namespace TagTracking {
    static partial class Tag {
        public class FinalTracker {
            public Vector3 pos;
            public Quaternion rot;
            public Quaternion prot;
            public Matrix4x4 preMat;
            public string name;

            public Vector3 fpos;
            public Quaternion frot;
            public Vector3 pfpos;
            public Quaternion pfrot;
            public float pVel;
            public float rVel;
            public float velScore;
            public long lastTimeSeen = 0;
            public bool notSeen = false;
            public bool firstTimeFaking = false;

            public OneEuroFilter<Vector3> smoothedPos = new OneEuroFilter<Vector3>(2); //w 25
            public OneEuroFilter<Quaternion> smoothedRot = new OneEuroFilter<Quaternion>(5); //w 150
            public List<Quaternion> rotList = new();
            public List<Vector3> posList = new();
            public Vector3 alwaysSmoothed = new Vector3();
            public Vector3 smoothPrevPos = new Vector3();
            public Quaternion smoothPrevRot = new Quaternion();
            public float avgSmoothDistTrigger = 0.025f; //w 0.05
            public float avgSmoothVal = 0.2f; //w 0.04
            public float avgSmoothRecoverVal = 0.9f;
            public float avgSmoothAlwaysVal = 0.08f;
            public float maxSpikePosDist = 0.5f;
            public float maxSpikeRotDiff = 0.9f;

            public FinalTracker(Vector3 pos, Quaternion rot, Quaternion prot, string name) {
                this.pos = pos;
                this.rot = rot;
                this.prot = prot;
                this.name = name;
            }

            public void Update() {
                pfpos = fpos;
                pfrot = frot;
                if (Program.postNoise == 0) {
                    fpos = pos;
                    frot = rot;
                    return;
                }

                //filter rotation spikes
                rotList.Insert(0, rot);
                if (rotList.Count > 10)
                    rotList.RemoveAt(rotList.Count - 1);
                List<Quaternion> r1List = new List<Quaternion>();
                List<Quaternion> r2List = new List<Quaternion>();
                Quaternion lastRot = rotList[0];
                r1List.Add(lastRot);
                bool switched = false;
                for (int i = 1; i < rotList.Count; i++) {
                    Quaternion curRot = rotList[i];
                    float rotDiff = Quaternion.Dot(Quaternion.Inverse(lastRot) * curRot, Quaternion.Identity);
                    if (rotDiff < 0.9f) {
                        switched = !switched;
                    }
                    if (switched) r2List.Add(curRot);
                    else r1List.Add(curRot);
                    lastRot = curRot;
                }
                if (r1List.Count < r2List.Count && (Program.preNoise == PreNoise.Enabled || Program.preNoise == PreNoise.EnabledSmooth)) {
                    if (r2List.Count > 7) rot = r2List[0];
                }

                //filter position spikes
                posList.Insert(0, pos);
                if (posList.Count > 10)
                    posList.RemoveAt(posList.Count - 1);
                List<Vector3> p1List = new List<Vector3>();
                List<Vector3> p2List = new List<Vector3>();
                Vector3 lastPos = posList[0];
                p1List.Add(lastPos);
                switched = false;
                for (int i = 1; i < posList.Count; i++) {
                    Vector3 curPos = posList[i];
                    float posDist = Utils.GetDistance(lastPos.X, lastPos.Y, lastPos.Z, curPos.X, curPos.Y, curPos.Z);
                    if (posDist > 0.05f) {
                        switched = !switched;
                    }
                    if (switched) p2List.Add(curPos);
                    else p1List.Add(curPos);
                    lastPos = curPos;
                }
                if (p1List.Count < p2List.Count && (Program.preNoise == PreNoise.Enabled || Program.preNoise == PreNoise.EnabledSmooth)) {
                    if (p2List.Count > 7) pos = p2List[0];
                }

                //one euro filtering
                if (!float.IsNaN(rot.X)) {
                    if (float.IsNaN(smoothedRot.currValue.X))
                        smoothedRot = new OneEuroFilter<Quaternion>(smoothedRot.freq);
                    Quaternion frotf = smoothedRot.Filter(Quaternion.Normalize(rot));

                    if (float.IsNaN(smoothPrevRot.X))
                        smoothPrevRot = rot;
                    float dot = Math.Abs(Quaternion.Dot(rot, frotf));
                    float sqrtFreq = (float)Math.Sqrt(smoothedRot.freq);
                    float end = (float)Math.Pow(dot / 1.5f, sqrtFreq);
                    frot = Quaternion.Slerp(smoothPrevRot, rot, end);
                }
                if (!float.IsNaN(pos.X))
                    fpos = smoothedPos.Filter(pos);

                //make an average filtering
                float distTh = avgSmoothDistTrigger; //0.0004f
                float smoothiness = avgSmoothVal;
                alwaysSmoothed += (fpos - alwaysSmoothed) * avgSmoothAlwaysVal;
                if (Utils.GetDistance(fpos.X, fpos.Y, fpos.Z, alwaysSmoothed.X, alwaysSmoothed.Y, alwaysSmoothed.Z) > distTh) {
                    smoothiness = avgSmoothRecoverVal;
                }
                smoothPrevPos += (fpos - smoothPrevPos) * smoothiness;
                fpos = smoothPrevPos;

                float smoothinessRot = smoothiness + (1 - smoothiness) / 2; //to be less smoothed
                smoothinessRot = smoothinessRot * 0.8f + smoothiness * 0.2f;
                smoothPrevRot = Quaternion.Lerp(smoothPrevRot, frot, smoothinessRot);
                smoothPrevRot = Quaternion.Normalize(smoothPrevRot);
                frot = smoothPrevRot;

                if (dynamicFiltering) {
                    pVel = (fpos - pfpos).Length();
                    rVel = (1f - Quaternion.Dot(frot, pfrot)) * 7f;
                    velScore += rVel;
                    velScore += pVel * 0.6f;
                    velScore -= 0.015f;
                    if (velScore < 0) velScore = 0;
                    else if (velScore > 1) velScore = 1;
                }
            }
        }
    }
}
