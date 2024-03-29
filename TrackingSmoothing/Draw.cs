﻿using Emgu.CV;
using Emgu.CV.CvEnum;
using Emgu.CV.Structure;
using Emgu.CV.Util;
using System;
using System.Collections.Generic;
using System.Drawing;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace TagTracking {
    class Draw {
        public enum ShapeType {
            Dot, Axis, AxisGray, Cube
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
        static readonly MCvPoint3D32f[] cubeVerts = new MCvPoint3D32f[] {
            new MCvPoint3D32f(0.3f, 0.3f, -0.3f),
            new MCvPoint3D32f(0.3f, -0.3f, -0.3f),
            new MCvPoint3D32f(-0.3f, -0.3f, -0.3f),
            new MCvPoint3D32f(-0.3f, 0.3f, -0.3f),
            new MCvPoint3D32f(0.3f, 0.3f, 0.3f),
            new MCvPoint3D32f(0.3f, -0.3f, 0.3f),
            new MCvPoint3D32f(-0.3f, -0.3f, 0.3f),
            new MCvPoint3D32f(-0.3f, 0.3f, 0.3f),
        };
        static readonly MCvPoint3D32f[] dotVerts = new MCvPoint3D32f[] {
            new MCvPoint3D32f(0, 0, 0), // Origin
        };
        static readonly Line[] dotLines = new Line[] {
            new Line(new Bgr(Color.DarkOrange).MCvScalar, 0, 0)
        };
        static readonly Line[] axisLines = new Line[] {
            new Line(new Bgr(Color.Red).MCvScalar, 0, 1),
            new Line(new Bgr(Color.LimeGreen).MCvScalar, 0, 2),
            new Line(new Bgr(Color.Blue).MCvScalar, 0, 3)
        };
        static readonly Line[] grayAxisLines = new Line[] {
            new Line(new Bgr(Color.IndianRed).MCvScalar, 0, 1),
            new Line(new Bgr(Color.ForestGreen).MCvScalar, 0, 2),
            new Line(new Bgr(Color.DarkSlateBlue).MCvScalar, 0, 3)
        };
        static readonly Line[] cubeLines = new Line[] {
            new Line(new Bgr(Color.OrangeRed).MCvScalar, 0, 1),
            new Line(new Bgr(Color.Orange).MCvScalar, 1, 2),
            new Line(new Bgr(Color.Orange).MCvScalar, 2, 3),
            new Line(new Bgr(Color.Orange).MCvScalar, 3, 0),

            new Line(new Bgr(Color.OrangeRed).MCvScalar, 4, 0),
            new Line(new Bgr(Color.Orange).MCvScalar, 5, 1),
            new Line(new Bgr(Color.Orange).MCvScalar, 6, 2),
            new Line(new Bgr(Color.Orange).MCvScalar, 7, 3),

            new Line(new Bgr(Color.OrangeRed).MCvScalar, 4, 5),
            new Line(new Bgr(Color.Orange).MCvScalar, 5, 6),
            new Line(new Bgr(Color.Orange).MCvScalar, 6, 7),
            new Line(new Bgr(Color.Orange).MCvScalar, 7, 4),
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
            if (objectLines.Length == 1 && objectLines[0].start == 0 && objectLines[0].end == 0) {
                CvInvoke.Circle(frame, Point.Round(points[0]), 1, objectLines[0].color, 2, LineType.FourConnected);
                return;
            }
            for (int i = 0; i < objectLines.Length; i++) {
                CvInvoke.Line(frame, Point.Round(points[objectLines[i].start]), Point.Round(points[objectLines[i].end]), objectLines[i].color, 2, LineType.AntiAlias);
            }
        }
        static MCvPoint3D32f[] GetShapeVerts(ShapeType shape) {
            switch (shape) {
                case ShapeType.Axis:
                    return (MCvPoint3D32f[])axisVerts.Clone();
                case ShapeType.Cube:
                    return (MCvPoint3D32f[])cubeVerts.Clone();
                case ShapeType.AxisGray:
                    return (MCvPoint3D32f[])axisVerts.Clone();
                case ShapeType.Dot:
                    return (MCvPoint3D32f[])dotVerts.Clone();
                default:
                    return (MCvPoint3D32f[])axisVerts.Clone();
            }
        }
        static Line[] GetShapeLines(ShapeType shape) {
            switch (shape) {
                case ShapeType.Axis:
                    return axisLines;
                case ShapeType.Cube:
                    return cubeLines;
                case ShapeType.AxisGray:
                    return grayAxisLines;
                case ShapeType.Dot:
                    return dotLines;
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
