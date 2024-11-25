using System;
using System.Collections.Generic;
using System.Numerics;

namespace TagTracking {
    static partial class Tag {
        public enum PreNoise {
            Disabled, DisabledSmooth, Enabled, EnabledSmooth
        }
        public class CombinedTracker {
            public SingleTracker[] singles = new SingleTracker[] {
                new SingleTracker(), new SingleTracker()
            };
            public int[] updateCount = new int[2];
            public int[] trackerPresence = new int[2];
            public float[] trackerStraightness = new float[2]; //higher is better
            public float trackerStraightnessMax = 0.2f;
            public bool[] newData = new bool[2];
            public int index = 0;
            public CombinedTracker(int index) {
                this.index = index;
                trackerStraightness = new float[cameras.Length];
                trackerPresence = new int[cameras.Length];
                updateCount = new int[cameras.Length];
                newData = new bool[cameras.Length];
                singles = new SingleTracker[cameras.Length];
                for (int i = 0; i < singles.Length; i++) {
                    singles[i] = new SingleTracker();
                }
            }
            public Quaternion lastRotAgreed = new Quaternion();
            public bool allAgreed = false;
            public Matrix4x4 lastSent = new Matrix4x4();
            public void Recieve(int camera, Vector3 pos, Matrix4x4 rot, int altRot) {
                //int cameraAllowed = (int)(Program.timer.ElapsedMilliseconds / 2000f) % 3;
                //if (camera != cameraAllowed) return;
                //Console.WriteLine("recieving " + cameraAllowed);
                if (altRot != -1) {
                    //FILTER ROTATION FOR ALTERNATES
                    List<Quaternion>[] altRotList = new List<Quaternion>[singles[camera].altRotList.Length];
                    for (int i = 0; i < altRotList.Length; i++)
                        altRotList[i] = new List<Quaternion>(singles[camera].altRotList[i]);

                    if (updateCount[camera] > 1)
                        altRotList[altRot].Clear();
                    altRotList[altRot].Insert(0, Quaternion.CreateFromRotationMatrix(rot));
                    if (altRotList[altRot].Count > 10)
                        altRotList[altRot].RemoveAt(altRotList[altRot].Count - 1);

                    singles[camera].altRotList = altRotList;
                    List<Quaternion> r1ListAlt = new List<Quaternion>();
                    List<Quaternion> r2ListAlt = new List<Quaternion>();
                    Quaternion lastRot1Alt = singles[camera].altRotList[altRot][0];
                    r1ListAlt.Add(lastRot1Alt);
                    bool switchedAlt = false;
                    for (int i = 1; i < singles[camera].altRotList[altRot].Count; i++) {
                        Quaternion curRot = singles[camera].altRotList[altRot][i];
                        float rotDiff = Quaternion.Dot(Quaternion.Inverse(lastRot1Alt) * curRot, Quaternion.Identity);
                        if (rotDiff < 0.9f) {
                            switchedAlt = !switchedAlt;
                        }
                        if (switchedAlt) r2ListAlt.Add(curRot);
                        else r1ListAlt.Add(curRot);
                        lastRot1Alt = curRot;
                    }
                    if (r1ListAlt.Count < r2ListAlt.Count && (Program.preNoise == PreNoise.Enabled || Program.preNoise == PreNoise.EnabledSmooth)) {
                        rot = Matrix4x4.CreateFromQuaternion(r2ListAlt[0]);
                    }
                    singles[camera].altRots[altRot] = rot;
                    return;
                }

                //TEST DEPTH
                //Random rnd = new Random();
                //if (rnd.NextDouble() < 0.3f)
                //    pos.Z -= 0.3f;
                //pos.Z += (float)Math.Sin(Program.timer.ElapsedMilliseconds / 500.0) * 0.3f;
                //Aruco.DrawAxis(Matrix4x4.Multiply(rot, Matrix4x4.CreateTranslation(pos)), camera);

                //FILTER DEPTH
                List<float> zList = new List<float>(singles[camera].zList);

                if (updateCount[camera] > 1) {
                    zList.Clear();
                    trackerPresence[camera] = 0;
                }
                zList.Insert(0, pos.Z);
                if (zList.Count > 10)
                    zList.RemoveAt(zList.Count - 1);

                singles[camera].zList = zList;
                List<float> d1List = new List<float>();
                List<float> d2List = new List<float>();
                float lastDepth1 = singles[camera].zList[0];
                d1List.Add(lastDepth1);
                for (int i = 1; i < singles[camera].zList.Count; i++) {
                    float currentDepth = singles[camera].zList[i];
                    float diff1 = Math.Abs(lastDepth1 - currentDepth);
                    if (diff1 < 0.05f) {
                        d1List.Add(currentDepth);
                        lastDepth1 = currentDepth;
                    } else {
                        d2List.Add(currentDepth);
                    }
                }
                if (d1List.Count < d2List.Count && (Program.preNoise == PreNoise.Enabled || Program.preNoise == PreNoise.EnabledSmooth)) {
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
                List<Quaternion> rotList = new List<Quaternion>(singles[camera].rotList);

                if (updateCount[camera] > 1)
                    rotList.Clear();
                rotList.Insert(0, Quaternion.CreateFromRotationMatrix(rot));
                if (rotList.Count > 10)
                    rotList.RemoveAt(singles[camera].rotList.Count - 1);

                singles[camera].rotList = rotList;
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
                if (r1List.Count < r2List.Count && (Program.preNoise == PreNoise.Enabled || Program.preNoise == PreNoise.EnabledSmooth)) {
                    rot = Matrix4x4.CreateFromQuaternion(r2List[0]);
                    if (r2List.Count > 7) singles[camera].consistentRot = true;
                } else {
                    if (r1List.Count > 7) singles[camera].consistentRot = true;
                }
                if (!singles[camera].consistentRot && !Program.ignoreNoisyRotation) {
                    rot = singles[camera].p_rot;
                }

                //rot = Matrix4x4.CreateFromYawPitchRoll(0, (float)Math.PI, 0);

                //APPLY PREVIOUS
                singles[camera].p_pos = singles[camera].pos;
                singles[camera].pos = pos;
                singles[camera].p_rot = singles[camera].rot;
                singles[camera].rot = rot;

                //APPLY ONE EURO FILTER TO Z
                singles[camera].filter_pos.UpdateParams(trackerPresence[camera] + 0.01f);
                if (trackerPresence[camera] == 0) {
                    for (int i = 0; i < 5; i++) { //to make sure is snapped
                        singles[camera].smooth_pos = singles[camera].filter_pos.Filter(pos);
                    }
                }
                singles[camera].smooth_pos = singles[camera].filter_pos.Filter(pos);
                if (Program.preNoise == PreNoise.Disabled || Program.preNoise == PreNoise.DisabledSmooth)
                    singles[camera].smooth_pos = pos;

                trackerPresence[camera]++;
                if (trackerPresence[camera] > 5) {
                    trackerPresence[camera] = 5;
                }

                //CHECK FOR ROTATION STARIGHTNESS
                //i dont remember what is this for lol

                //Quaternion q = Quaternion.CreateFromRotationMatrix(singles[camera].rot);
                //Matrix4x4 mat = Matrix4x4.Multiply(Matrix4x4.CreateTranslation(new Vector3(0, 0, -0.1f)), Matrix4x4.CreateTranslation(pos));
                //float depth = singles[camera].pos.Z;
                //if (depth == 0) depth = 1;
                //float d = Utils.GetDistance(singles[camera].pos.X / depth, singles[camera].pos.Y / depth, 0, 0, 0, 0) * 0.12f;
                //d -= depth / cameras[camera].quality / 4f;
                //float r1 = mat.Translation.Z - pos.Z + 0.1f;
                //float r2 = Utils.GetMap(r1, 0.2f, 0, 0.2f, d);
                //trackerStraightness[camera] = r2;

                //other plans for this

                //Console.WriteLine(pos);
                //Vector3 posInv = pos;
                //posInv *= -1;
                //posInv.Y *= -1;
                //var lookatHor = Matrix4x4.CreateLookAt(posInv, new Vector3(), new Vector3(0, -1, 0));
                //lookatHor.M43 = 0;
                //lookatHor.M42 = 0;
                //lookatHor.M41 = 0;
                Matrix4x4 lookatMat = Matrix4x4.Multiply(rot, Matrix4x4.CreateTranslation(pos));
                Matrix4x4 moveMat = Matrix4x4.Multiply(Matrix4x4.CreateTranslation(new Vector3(0, 0, pos.Z)), lookatMat);
                float d = Utils.GetDistance(moveMat.M41, moveMat.M42, moveMat.M43, 0, 0, 0) / pos.Z;
                d = (float)Math.Pow(d * 0.5f, 2);
                trackerStraightness[camera] = d;
                //if (index == 0) {
                //    Console.WriteLine(d);
                //    //Aruco.DrawAxis(moveMat, camera);
                //}

                if (d > tagStraightnessThreshold) return;

                //RESET UPDATE
                updateCount[camera] = 0;
                newData[camera] = true;

                //trackerStraightness[camera] = 0;

                //if (Program.preNoise && false) {
                //    Matrix4x4 matp = Matrix4x4.Multiply(rot, Matrix4x4.CreateTranslation(pos));
                //    Matrix4x4 mat0 = Matrix4x4.CreateLookAt(Vector3.Zero, pos, new Vector3(0, 1, 0));
                //    matp = Matrix4x4.Multiply(matp, mat0);
                //    pos = matp.Translation;
                //    rot = Matrix4x4.CreateFromQuaternion(matp.Rotation());
                //    //singles[camera].pos = pos;
                //    //singles[camera].rot = rot;
                //    Quaternion dif = Quaternion.Subtract(rot.Rotation(), singles[camera].rot.Rotation());
                //    Quaternion rrr = singles[camera].rot.Rotation();
                //    if (camera == 0 && (index == 1 || index == 6))
                //        Console.WriteLine($"{index} - {rrr.X}, {rrr.Y}, {rrr.Z}, {rrr.W}");
                //    rrr.Z = -rrr.Z;
                //    rrr.W = -rrr.W;
                //    singles[camera].rot = Matrix4x4.CreateFromQuaternion(rrr);
                //}
                //Console.WriteLine($"{mat.Translation.Z.ToString("0.00")} / {pos.Z.ToString("0.00")} / {r1.ToString("0.00")} ( {r2.ToString("0.00")} ) // {d.ToString("0.00")} / {singles[i].pos.Z.ToString("0.00")}");
                //Console.WriteLine($"{pitch.ToString("0.00")} // {(singles[i].pos.Y / singles[i].pos.Z).ToString("0.00")} / {singles[i].pos.Y.ToString("0.00")} / {singles[i].pos.Z.ToString("0.00")}");
            }
            public Matrix4x4[] Obtain(int[] altRot = null) {
                if (altRot == null) {
                    altRot = new int[cameras.Length];
                    for (int i = 0; i < cameras.Length; i++) {
                        altRot[i] = -1;
                    }
                }
                //COMBINE ROT MATRIX WITH POS VECTOR
                float[] depth = new float[cameras.Length];
                Matrix4x4[] rots = new Matrix4x4[cameras.Length];
                Matrix4x4[] pos = new Matrix4x4[cameras.Length];
                for (int i = 0; i < cameras.Length; i++) {
                    depth[i] = singles[i].smooth_pos.Z;
                    rots[i] = singles[i].rot;
                    if (altRot[i] != -1 && altRot[i] < 4) {
                        rots[i] = singles[i].altRots[altRot[i]];
                    }
                    pos[i] = Matrix4x4.Multiply(rots[i], Matrix4x4.CreateTranslation(singles[i].pos.X * cameras[i].depthMult, singles[i].pos.Y * cameras[i].depthMult, depth[i] * cameras[i].depthMult));
                    pos[i] = Matrix4x4.Multiply(pos[i], cameras[i].matrix);
                }
                return pos;

                //-------------------!!!! UNUSED CODE AHEAD !!!!--------------------------------------

                ////for now just the average
                ////i plan to use projection if the two are available

                ////USE DEPTH FROM THE FILTERED VERSION
                //float depth1 = singles[0].smooth_pos.Z;
                //float depth2 = singles[1].smooth_pos.Z;

                ////GUESS WHICH CAM IS CLOSER BASED ON DEPTH
                //int closestCam = 0;
                //if ((depth2 / cameras[1].quality < depth1 / cameras[0].quality && updateCount[1] <= 2) || (updateCount[0] > 2 && updateCount[1] <= 2))
                //    closestCam = 1;

                ////COMBINE ROT MATRIX WITH POS VECTOR
                //Matrix4x4 rot1 = singles[0].rot;
                //Matrix4x4 rot2 = singles[1].rot;
                //Matrix4x4[] pos = new Matrix4x4[] {
                //    Matrix4x4.Multiply(rot1, Matrix4x4.CreateTranslation(singles[0].pos.X * cameras[0].depthMult, singles[0].pos.Y * cameras[0].depthMult, depth1 * cameras[0].depthMult)),
                //    Matrix4x4.Multiply(rot2, Matrix4x4.CreateTranslation(singles[1].pos.X * cameras[1].depthMult, singles[1].pos.Y * cameras[1].depthMult, depth2 * cameras[1].depthMult))
                //};

                ////TRANSFORM WITH CAMERA MATRIX
                //pos[0] = Matrix4x4.Multiply(pos[0], cameras[0].matrix);
                //pos[1] = Matrix4x4.Multiply(pos[1], cameras[1].matrix);
                //Matrix4x4 result = pos[closestCam];

                ////GET QUATERNIONS FROM POS MATRIX
                //Quaternion[] quat = new Quaternion[] {
                //    Quaternion.CreateFromRotationMatrix(pos[0]),
                //    Quaternion.CreateFromRotationMatrix(pos[1])
                //};

                ////CHECK IF BOTH ROTATIONS ARE IDENTICAL,
                ////IF NOT, CHECK FOR THE LESS ROTATED DISCARD ONE, IF NOT BOTH, THE PREVIOUS WILL REMAIN
                //float rotDiff = Quaternion.Dot(Quaternion.Inverse(quat[0]) * quat[1], Quaternion.Identity);
                //if (rotDiff > 0.9f) {
                //    lastRotAgreed = quat[closestCam];
                //    allAgreed = true;
                //} else {
                //    allAgreed = false;
                //    if (updateCount[0] < 2 && updateCount[1] < 2) {
                //        if (trackerStraightness[0] > trackerStraightness[1]) {
                //            lastRotAgreed = quat[0];
                //        } else {
                //            lastRotAgreed = quat[1];
                //        }
                //    } else {
                //        rotDiff = Quaternion.Dot(Quaternion.Inverse(lastRotAgreed) * quat[0], Quaternion.Identity);
                //        if ((rotDiff > 0.9f && updateCount[0] < 2) || (updateCount[0] < 2 && updateCount[1] >= 2)) {
                //            lastRotAgreed = quat[0];
                //        }
                //        rotDiff = Quaternion.Dot(Quaternion.Inverse(lastRotAgreed) * quat[1], Quaternion.Identity);
                //        if ((rotDiff < 0.9f && updateCount[1] < 2) || (updateCount[1] < 2 && updateCount[0] >= 2)) {
                //            lastRotAgreed = quat[1];
                //        }
                //    }
                //}
                //trackerStraightnessMax = Math.Max(trackerStraightness[0], trackerStraightness[1]);
                //if (updateCount[0] >= 2 || updateCount[1] >= 2) {
                //    if (updateCount[0] < 2)
                //        trackerStraightnessMax = trackerStraightness[0];
                //    if (updateCount[1] < 2)
                //        trackerStraightnessMax = trackerStraightness[1];
                //}

                ////APPLY NEW MATRIX WITH AVERAGED ROTATIONS
                //Matrix4x4 newRes = Matrix4x4.CreateFromQuaternion(lastRotAgreed);
                //newRes.M41 = result.M41;
                //newRes.M42 = result.M42;
                //newRes.M43 = result.M43;
                //result = newRes;

                ////CHECK BOTH TRACKER VISIBILTY TO AVERAGE
                //Vector3 newPos = new Vector3();
                //int lost = 0;
                //if (updateCount[0] > 2) {
                //    newPos = pos[1].Translation;
                //    lost++;
                //}
                //if (updateCount[1] > 2) {
                //    newPos = pos[0].Translation;
                //    lost++;
                //}
                //if (newInfo) {
                //    updateCount[1]++;
                //    updateCount[0]++;
                //}
                //if (lost == 0) {
                //    float z1 = 1f / (depth1 / cameras[0].quality);
                //    float z2 = 1f / (depth2 / cameras[1].quality);
                //    float zS = z1 + z2;
                //    newPos.X = (pos[0].M41 * z1 + pos[1].M41 * z2) / zS;
                //    newPos.Y = (pos[0].M42 * z1 + pos[1].M42 * z2) / zS;
                //    newPos.Z = (pos[0].M43 * z1 + pos[1].M43 * z2) / zS;
                //} else if (lost == 2) {
                //    return null;// lastSent;
                //}
                ////------------------------Test--------------------------------
                ////closestCam = (int)(Program.timer.ElapsedMilliseconds / 500) % 2;
                ////newPos.X = pos[closestCam].M41;
                ////newPos.Y = pos[closestCam].M42;
                ////newPos.Z = pos[closestCam].M43;
                ////------------------------------------------------------------
                //if (float.IsNaN(newPos.X))
                //    newPos = new Vector3();

                ////APPLY NEW POSITION
                //result.M41 = newPos.X;
                //result.M42 = newPos.Y;
                //result.M43 = newPos.Z;
                //lastSent = result;
                //return null;//result;
            }
        }
    }
}
