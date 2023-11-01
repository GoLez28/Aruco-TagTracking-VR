using Emgu.CV;
using Emgu.CV.CvEnum;
using Emgu.CV.Structure;
using Emgu.CV.Util;
using System;
using System.Collections.Generic;
using System.Drawing;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace TrackingSmoothing {
    class Draw {
        public enum ShapeType {
            Dot, Axis, AxisGrey, Cube
        }
        public struct Line {
            public MCvScalar color;
            public int start;
            public int end;
            public Line(MCvScalar color, int start, int end) {
                this.color = color;
                this.start = start;
                this.end = end;
            }
        }
        static readonly MCvPoint3D32f[] axisVerts = new MCvPoint3D32f[] {
            new MCvPoint3D32f(0, 0, 0), // Origin
            new MCvPoint3D32f(1f, 0, 0), // X-Axis
            new MCvPoint3D32f(0, 1f, 0), // Y-Axis
            new MCvPoint3D32f(0, 0, 1f) // Z-Axis
        };
        static readonly Line[] axisLines = new Line[] {
            new Line(new Bgr(Color.Red).MCvScalar, 0, 1),
            new Line(new Bgr(Color.LimeGreen).MCvScalar, 0, 2),
            new Line(new Bgr(Color.Blue).MCvScalar, 0, 3)
        };

        public static void Axis(Mat frame, Mat cameraMatrix, Mat distortionMatrix, VectorOfDouble rvec, VectorOfDouble tvec, double axisLength) {
            // Convert rotation vector to rotation matrix
            Shape(frame, cameraMatrix, distortionMatrix, rvec, tvec, axisLength, ShapeType.Axis);
        }
        public static void Shape(Mat frame, Mat cameraMatrix, Mat distortionMatrix, VectorOfDouble rvec, VectorOfDouble tvec, double axisLength, ShapeType shape) {
            // Convert rotation vector to rotation matrix
            Matrix<double> rotationMatrix = new Matrix<double>(4, 4);
            CvInvoke.Rodrigues(rvec, rotationMatrix);

            // Define the 3D points for drawing the axis lines
            MCvPoint3D32f[] objectPoints = GetShapeVerts(shape, (float)axisLength);
            Line[] objectLines = GetShapeLines(shape);

            // Project the 3D points onto the image plane
            Mat imagePoints = new Mat();
            PointF[] points = CvInvoke.ProjectPoints(objectPoints, rotationMatrix, tvec, cameraMatrix, distortionMatrix, imagePoints);

            //adjust points to canvas
            bool adjustSize = Program.performanceMode;
            if (!adjustSize)
                for (int i = 0; i < Tag.cameras.Length; i++) {
                    if (Tag.cameras[i].adjustCurrentDistortion) {
                        adjustSize = true;
                        break;
                    }
                }
            if (adjustSize) {
                int cam = 0;
                for (int i = 0; i < Aruco.cameraMatrix.Length; i++) {
                    if (Aruco.cameraMatrix[i] == cameraMatrix) {
                        cam = i;
                        break;
                    }
                }
                for (int i = 0; i < points.Length; i++) {
                    points[i].X /= Tag.cameras[cam].xRatio;
                    points[i].Y /= Tag.cameras[cam].yRatio;
                }
            }

            // Draw the lines
            for (int i = 0; i < objectLines.Length; i++) {
                CvInvoke.Line(frame, Point.Round(points[objectLines[i].start]), Point.Round(points[objectLines[i].end]), objectLines[i].color, 2, LineType.AntiAlias);
            }
        }
        static MCvPoint3D32f[] GetShapeVerts(ShapeType shape) {
            switch (shape) {
                case ShapeType.Axis:
                    return (MCvPoint3D32f[])axisVerts.Clone();
                default:
                    return (MCvPoint3D32f[])axisVerts.Clone();
            }
        }
        static Line[] GetShapeLines(ShapeType shape) {
            switch (shape) {
                case ShapeType.Axis:
                    return axisLines;
                default:
                    return axisLines;
            }
        }
        static MCvPoint3D32f[] GetShapeVerts(ShapeType shape, float size) {
            MCvPoint3D32f[] verts = GetShapeVerts(shape);
            for (int i = 0; i < verts.Length; i++) {
                verts[i] *= size;
            }
            return verts;
        }
    }
}
