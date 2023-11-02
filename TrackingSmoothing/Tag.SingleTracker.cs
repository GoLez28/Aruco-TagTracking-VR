using System.Collections.Generic;
using System.Numerics;

namespace TagTracking {
    static partial class Tag {
        // i like everything public
        public class SingleTracker {
            public Vector3 p_pos = new Vector3();
            public Vector3 pos = new Vector3();
            public OneEuroFilter<Vector3> filter_pos = new OneEuroFilter<Vector3>(5);
            public Vector3 smooth_pos = new Vector3();
            public List<float> zList = new List<float>();
            public Matrix4x4 p_rot = new Matrix4x4();
            public Matrix4x4 rot = new Matrix4x4();
            public Matrix4x4[] altRots = new Matrix4x4[4];
            public bool[] repeatedRots = new bool[4];
            public List<Quaternion> rotList = new List<Quaternion>();
            public bool consistentRot = true;
            public List<Quaternion>[] altRotList = new List<Quaternion>[] {
                new List<Quaternion>(), new List<Quaternion>(), new List<Quaternion>(), new List<Quaternion>()
             };
        }
    }
}
