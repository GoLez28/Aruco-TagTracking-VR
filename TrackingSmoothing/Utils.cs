using System;
using System.Collections.Generic;
using System.Linq;
using System.Numerics;
using System.Text;
using System.Threading.Tasks;

namespace TrackingSmoothing {
    static class Utils {
        public static float GetDistance(float[] a, float[] b) {
            return GetDistance(a[0], a[1], a[2], b[0], b[1], b[2]);
        }
        public static float GetDistance(float x1, float y1, float z1, float x2, float y2, float z2) {
            return (float)Math.Sqrt(
                Math.Pow(x1 - x2, 2) +
                Math.Pow(y1 - y2, 2) +
                Math.Pow(z1 - z2, 2));
        }
        public static float GetRotationDifference(Quaternion q1, Quaternion q2) {
            Quaternion d = q1 * Quaternion.Inverse(q2);
            float ad = Math.Abs(d.X) + Math.Abs(d.Y) + Math.Abs(d.Z);
            return ad;
        }
        static public float GetMap(float value, float start1, float stop1, float start2, float stop2) {
            //From Processing
            float outgoing = start2 + (stop2 - start2) * ((value - start1) / (stop1 - start1));
            string badness = null;
            if (Single.IsNaN(outgoing)) {
                badness = "NaN (not a number)";
            } else if (outgoing == Single.NegativeInfinity ||
                       outgoing == Single.PositiveInfinity) {
                badness = "infinity";
            }
            if (badness != null) {
                string msg = $"map({value}, {start1}, {stop1}, {start2}, {stop2}) called, which returns {badness}";
            }
            return outgoing;
        }
        static public Quaternion CalcAverageQuaternion(List<Quaternion> quats) {
            if (quats.Count == 0) return new();
            if (quats.Count == 1)
                return quats[0];
            List<Quaternion> firstHalf = quats.GetRange(0, quats.Count / 2);// first half of quats list;
            List<Quaternion> secondHalf = quats.GetRange(quats.Count / 2, quats.Count - (quats.Count / 2));//second half of quats list;
            Quaternion q1 = CalcAverageQuaternion(firstHalf);
            Quaternion q2 = CalcAverageQuaternion(secondHalf);
            return Quaternion.Lerp(q1, q2, firstHalf.Count / quats.Count);
        }
        static public Quaternion QLerp (Quaternion q1, Quaternion q2, float amount) {
            float t = amount;
            float t1 = 1.0f - t;
            Quaternion r = new Quaternion();
            r.X = t1 * q1.X + t * q2.X;
            r.Y = t1 * q1.Y + t * q2.Y;
            r.Z = t1 * q1.Z + t * q2.Z;
            r.W = t1 * q1.W + t * q2.W;
            r = Quaternion.Normalize(r);
            return r;
        }
    }
}
