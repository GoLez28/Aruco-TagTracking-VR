using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace TrackingSmoothing {
    static class Extrapolate {
        static float getAngle(float[] a, float[] b) {
            float angL = GetAngleBetween(new float[] { 1, 0, 0 }, new float[] { a[0] - b[0], a[1] - b[1], a[2] - b[0] });
            //if (b[1] > a[1]) angL = -angL;
            return angL;
        }
        //THIS IS A MADE UP EXTRAPOLATION METHOD, DO NOT COPY, IT IS BAD LOL
        public static float[] CurveExtra(float[][] points, float dist) {

            float curviness = 0.7f;

            if (dist < 0) {
                float x = points[2][0] - points[1][0];
                float y = points[2][1] - points[1][1];
                float z = points[2][2] - points[1][2];
                float revD = dist + 1;
                return new float[] {
                  points[1][0] + x*revD,
                  points[1][1] + y*revD,
                  points[1][2] + z*revD
                };
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
            return newPred;
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
