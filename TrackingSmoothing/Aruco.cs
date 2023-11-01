using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Drawing;
using Emgu.CV; // the mom
using Emgu.CV.Aruco; // the hero
using Emgu.CV.CvEnum; // the book
using Emgu.CV.Structure; // the storage
using Emgu.CV.Util; // the side kick
using System.Numerics;
using System.Runtime.InteropServices;

namespace TrackingSmoothing {
    class Aruco {
        class RectEx {
            public PointF[] smoothedVals = new PointF[4];
            public PointF[] lockedVals = new PointF[4];
            public PointF[] prevVals = new PointF[4];
            public bool locked;
        }
        static void PrintArucoBoard(GridBoard ArucoBoard, int markersX = 4, int markersY = 4, int markersLength = 80, int markersSeparation = 30) {
            // Size of the border of a marker in bits
            int borderBits = 1;

            // Draw the board on a cv::Mat
            Size imageSize = new Size();
            Mat boardImage = new Mat();
            imageSize.Width = markersX * (markersLength + markersSeparation) - markersSeparation + 2 * markersSeparation;
            imageSize.Height = markersY * (markersLength + markersSeparation) - markersSeparation + 2 * markersSeparation;
            ArucoBoard.Draw(imageSize, boardImage, markersSeparation, borderBits);

            // Save the image
            boardImage.Save("arucotags.png");
        }
        static RectEx[][] betterRects = new RectEx[][] {
            new RectEx[] { new(), new(), new(), new(), new(), new(), new(), new(), new(), new(), new(), new(), new(), new(), new(), new() },
            new RectEx[] { new(), new(), new(), new(), new(), new(), new(), new(), new(), new(), new(), new(), new(), new(), new(), new() },
            new RectEx[] { new(), new(), new(), new(), new(), new(), new(), new(), new(), new(), new(), new(), new(), new(), new(), new() }
        };
        static VideoCapture[] capture;
        static Dictionary ArucoDict;
        static DetectorParameters ArucoParameters;
        public static int markersLength = 56;
        static Mat[] cameraMatrix;
        static Mat[] distortionMatrix;

        public static bool useSmoothCorners = true;
        public static int cornersMaxDistance = 1;
        public static float cornersSmoothFactor = 0.1f;
        //static Bitmap whiteLogo;
        //static Bitmap blackLogo;
        //static double[,,] whiteLogoGamma = new double[250, 100, 3];
        //static double[,,] blackLogoGamma = new double[250, 100, 3];
        //static double[] byte2floatGamma = new double[256];
        //static byte[] float2byteGamma = new byte[256];
        public static void Init() {
            queueCount = new int[Tag.cameras.Length];
            sclQueue = new float[Tag.cameras.Length][];
            posQueue = new VectorOfDouble[Tag.cameras.Length][];
            rotQueue = new VectorOfDouble[Tag.cameras.Length][];
            for (int i = 0; i < Tag.cameras.Length; i++) {
                sclQueue[i] = new float[maxQueue];
                posQueue[i] = new VectorOfDouble[maxQueue];
                rotQueue[i] = new VectorOfDouble[maxQueue];
            }
            betterRects = new RectEx[Tag.cameras.Length][];
            for (int i = 0; i < betterRects.Length; i++) {
                betterRects[i] = new RectEx[36];
                for (int j = 0; j < betterRects[i].Length; j++) {
                    betterRects[i][j] = new();
                }
            }
            capture = new VideoCapture[Tag.cameras.Length];
            //whiteLogo = new Bitmap(@"iVCam\black.jpg");
            //blackLogo = new Bitmap(@"iVCam\white.jpg");
            //for (int i = 0; i < 256; i++) {
            //    byte2floatGamma[i] = Math.Pow(i / 255.0, 2.2);
            //    float2byteGamma[(int)((1 - (1 - byte2floatGamma[i]) * (1 - byte2floatGamma[i])) * 255)] = (byte)(Math.Round(Math.Pow(byte2floatGamma[i], 1.0 / 2.2) * 255.0));
            //}
            //for (int x = 0; x < 100; x++) {
            //    for (int y = 0; y < 250; y++) {
            //        double lwR = Math.Pow(whiteLogo.GetPixel(y, x).R / 255f * 1.02f, 2.2f);
            //        double lwG = Math.Pow(whiteLogo.GetPixel(y, x).G / 255f * 1.02f, 2.2f);
            //        double lwB = Math.Pow(whiteLogo.GetPixel(y, x).B / 255f * 1.02f, 2.2f);
            //        double lbR = Math.Pow(blackLogo.GetPixel(y, x).R / 255f, 2.2f);
            //        double lbG = Math.Pow(blackLogo.GetPixel(y, x).G / 255f, 2.2f);
            //        double lbB = Math.Pow(blackLogo.GetPixel(y, x).B / 255f, 2.2f);
            //        whiteLogoGamma[y, x, 0] = lwR;
            //        whiteLogoGamma[y, x, 1] = lwG;
            //        whiteLogoGamma[y, x, 2] = lwB;
            //        blackLogoGamma[y, x, 0] = lbR;
            //        blackLogoGamma[y, x, 1] = lbG;
            //        blackLogoGamma[y, x, 2] = lbB;
            //    }
            //}
            for (int i = 0; i < Tag.cameras.Length; i++) {
                try {
                    capture[i] = new VideoCapture(Tag.cameras[i].index, VideoCapture.API.DShow);
                } catch (Exception e) {
                    Console.WriteLine($"Something went wrong opening camera {Tag.cameras[i].index}\n{e}");
                    System.Threading.Thread.Sleep(5000);
                }
                capture[i].Set(CapProp.FrameWidth, Tag.cameras[i].width);
                capture[i].Set(CapProp.FrameHeight, Tag.cameras[i].height);
            }

            int markersX = 6;
            int markersY = 6;
            int markersSeparation = 30;
            ArucoDict = new Dictionary(36, 4/*Dictionary.PredefinedDictionaryName.Dict4X4_1000*/); // bits x bits (per marker) _ number of markers in dict
            //ArucoDict = new Dictionary(false);
            GridBoard ArucoBoard = null;
            ArucoBoard = new GridBoard(markersX, markersY, markersLength, markersSeparation, ArucoDict);
            PrintArucoBoard(ArucoBoard, markersX, markersY, markersLength, markersSeparation);
            CameraCalibrate.DrawBoard();

            ArucoParameters = new DetectorParameters();
            ArucoParameters = DetectorParameters.GetDefault();
            //ArucoParameters.PolygonalApproxAccuracyRate = 0.1;

            // Calibration done with https://docs.opencv.org/3.4.3/d7/d21/tutorial_interactive_calibration.html
            cameraMatrix = new Mat[Tag.cameras.Length];
            distortionMatrix = new Mat[Tag.cameras.Length];
            for (int i = 0; i < Tag.cameras.Length; i++) {
                cameraMatrix[i] = new Mat(new Size(3, 3), DepthType.Cv32F, 1);
                distortionMatrix[i] = new Mat(1, 8, DepthType.Cv32F, 1);
            }
            for (int c = 0; c < Tag.cameras.Length; c++) {
                string cameraConfigurationFile = "cameraParameters" + (c + 1) + ".xml";
                if (!Tag.cameras[c].file.Equals(""))
                    cameraConfigurationFile = Tag.cameras[c].file;
                FileStorage fs = new FileStorage(cameraConfigurationFile, FileStorage.Mode.Read);
                if (!fs.IsOpened) {
                    Console.WriteLine("Could not open configuration file " + cameraConfigurationFile);
                    return;
                }
                fs["cameraMatrix"].ReadMat(cameraMatrix[c]);
                fs["dist_coeffs"].ReadMat(distortionMatrix[c]);
            }
        }
        static float smoothbenchmark = 0;
        public static void Update(int c) {
            int frameCount = 0;
            while (true) {
                System.Diagnostics.Stopwatch arucoThreadWorkBenchmark = new System.Diagnostics.Stopwatch();
                System.Diagnostics.Stopwatch arucoThreadIdleBenchmark = new System.Diagnostics.Stopwatch();
                arucoThreadWorkBenchmark.Start();
                frameCount++;
                if (Program.wantToCloseWindows) {
                    CvInvoke.DestroyAllWindows();
                    Program.wantToCloseWindows = false;
                }
                bool shouldShowFrame = Program.wantToShowFrame;
                //Capture a frame with webcam
                Mat frame = new Mat();
                if (c >= capture.Length) {
                    Console.WriteLine("Wait... this is illegal");
                }
                System.Diagnostics.Debug.WriteLine(c);
                arucoThreadWorkBenchmark.Stop();
                arucoThreadIdleBenchmark.Start();
                while (true) {
                    frame = capture[c].QueryFrame();
                    int skipFrames = Tag.cameras[c].skipFrames;
                    if (Program.performanceMode) {
                        if (skipFrames == 0) skipFrames = 1;
                        else if (skipFrames > 0) skipFrames *= 2;
                    }
                    if (Tag.cameras[c].skipFrameCount >= skipFrames) {
                        Tag.cameras[c].skipFrameCount = 0;
                        break;
                    }
                    Tag.cameras[c].skipFrameCount++;
                }
                arucoThreadIdleBenchmark.Stop();
                arucoThreadWorkBenchmark.Start();
                bool correctRes = Tag.cameras[c].rsHeight > 10 && Tag.cameras[c].rsWidth > 10;
                if (correctRes) {
                    int newWidth = Tag.cameras[c].rsWidth;
                    int newHeight = Tag.cameras[c].rsHeight;
                    System.Diagnostics.Stopwatch sw = new System.Diagnostics.Stopwatch();
                    bool xb = frameCount % 2 == 0;
                    bool yb = frameCount % 4 > 1;
                    sw.Start();
                    byte[] asd = frame.GetRawData();
                    float mult = (float)newHeight / (float)frame.Height;
                    byte[] dsds = new byte[newWidth * newHeight * 3];
                    GCHandle pinnedArray = GCHandle.Alloc(dsds, GCHandleType.Pinned);
                    IntPtr pointer = pinnedArray.AddrOfPinnedObject();
                    int[] ptrx1 = new int[newWidth];
                    int[] ptrx2 = new int[newWidth];
                    int nw3 = newWidth * 3;
                    int ow3 = frame.Width * 3;
                    unsafe {
                        fixed (byte* pntr = &dsds[0], pntr2 = &asd[0]) {
                            for (int x = 0; x < newWidth; x++) {
                                ptrx1[x] = x * 3;
                                int x2 = xb ? (int)(x / mult) : (int)Math.Ceiling(x / mult);
                                ptrx2[x] = x2 * 3;
                            }
                            for (int y = 0; y < newHeight; y++) {
                                int y2 = yb ? (int)(y / mult) : (int)Math.Ceiling(y / mult);
                                int ay1 = y * nw3;
                                int ay2 = y2 * ow3;
                                byte* c1 = pntr + ay1;
                                byte* c2 = pntr2 + ay2;
                                for (int x = 0; x < newWidth; x++) {
                                    int* cc1 = (int*)(c1 + ptrx1[x]);
                                    int* cc2 = (int*)(c2 + ptrx2[x]);
                                    *cc1 = *cc2;
                                }
                            }
                        }
                    }

                    frame = new Mat(newHeight, newWidth, DepthType.Cv8U, 3, pointer, 0);
                    pinnedArray.Free();
                    sw.Stop();
                    //if (c == 2) {
                    //    smoothbenchmark += (sw.ElapsedMilliseconds - smoothbenchmark) * 0.01f;
                    //    Console.WriteLine($"cam {c}: {smoothbenchmark}ms");
                    //}
                }
                bool adjustBrightness = Tag.cameras[c].brightness != 1f;
                if (adjustBrightness) {
                    System.Diagnostics.Stopwatch sw = new System.Diagnostics.Stopwatch();
                    sw.Start();
                    frame *= Tag.cameras[c].brightness;
                    sw.Stop();
                    //if (c == 2) {
                    //    smoothbenchmark += (sw.ElapsedMilliseconds - smoothbenchmark) * 0.1f;
                    //    Console.WriteLine($"cam {c}: {smoothbenchmark}ms");
                    //}
                }
                if (CameraCalibrate.onCalibration && CameraCalibrate.startCalibrating) {
                    if (c == CameraCalibrate.cameraToUse) {
                        CameraCalibrate.GetFrame(frame);
                        frame.Dispose();
                        continue;
                    }
                }
                //if (c == 0)
                //    frame = new Mat(@"C:\Users\\Videos\iVCam\.jpg");
                //if (c == 1
                //    frame = new Mat(@"C:\Users\\Videos\iVCam\.jpg");


                //frame *= 2f;
                //var asd = (Byte[,,])frame.GetData();
                //try {
                //    for (int x = 0; x < 100; x++) {
                //        for (int y = 0; y < 250; y++) {
                //            double cR = byte2floatGamma[asd[x, y, 0]];
                //            double cG = byte2floatGamma[asd[x, y, 1]];
                //            double cB = byte2floatGamma[asd[x, y, 2]];
                //            double lwR = whiteLogoGamma[y, x, 0];
                //            double lwG = whiteLogoGamma[y, x, 1];
                //            double lwB = whiteLogoGamma[y, x, 2];
                //            double lbR = blackLogoGamma[y, x, 0];
                //            double lbG = blackLogoGamma[y, x, 1];
                //            double lbB = blackLogoGamma[y, x, 2];
                //            //max 210, min 163
                //            //double lwR = Math.Pow(whiteLogo.GetPixel(y, x).R / 255f * 1.02f, 2.2f);
                //            //double lwG = Math.Pow(whiteLogo.GetPixel(y, x).G / 255f * 1.02f, 2.2f);
                //            //double lwB = Math.Pow(whiteLogo.GetPixel(y, x).B / 255f * 1.02f, 2.2f);
                //            //double lbR = Math.Pow(blackLogo.GetPixel(y, x).R / 255f, 2.2f);
                //            //double lbG = Math.Pow(blackLogo.GetPixel(y, x).G / 255f, 2.2f);
                //            //double lbB = Math.Pow(blackLogo.GetPixel(y, x).B / 255f, 2.2f);
                //            asd[x, y, 0] = (byte)(Math.Pow((cR - lwR) / lbR, 1 / 2.2f) * 255f);
                //            asd[x, y, 1] = (byte)(Math.Pow((cG - lwG) / lbG, 1 / 2.2f) * 255f);
                //            asd[x, y, 2] = (byte)(Math.Pow((cB - lwB) / lbB, 1 / 2.2f) * 255f);
                //        }
                //    }
                //} catch (Exception e) {
                //    Console.WriteLine();
                //}
                //GCHandle pinnedArray = GCHandle.Alloc(asd, GCHandleType.Pinned);
                //IntPtr pointer = pinnedArray.AddrOfPinnedObject();
                //// Do your stuff...
                //frame = new Mat(frame.Height, frame.Width, DepthType.Cv8U, 3, pointer, 0);
                //pinnedArray.Free();

                if (!frame.IsEmpty) {
                    //Detect markers on last retrieved frame
                    VectorOfInt ids = new VectorOfInt(); // name/id of the detected markers
                    VectorOfVectorOfPointF corners = new VectorOfVectorOfPointF(); // corners of the detected marker
                    VectorOfVectorOfPointF rejected = new VectorOfVectorOfPointF(); // rejected contours
                    ArucoInvoke.DetectMarkers(frame, ArucoDict, corners, ids, ArucoParameters, rejected);

                    //smooth corners
                    try {
                        if (Program.preNoise == 1) {
                            corners = SmoothCorners(c, ids, corners);
                        }
                    } catch (Exception e) {
                        Console.WriteLine("Couldnt smooth corners\n" + e);
                    }
                    // If we detected at least one marker
                    if (ids.Size > 0) {
                        //Draw detected markers
                        VectorOfVectorOfPointF skew;
                        //corners = SkewRects(c, ids, corners, 2, -1);
                        if (shouldShowFrame)
                            ArucoInvoke.DrawDetectedMarkers(frame, corners, ids, new MCvScalar(255, 0, 255));

                        //Estimate pose for each marker using camera calibration matrix and distortion coefficents
                        //get first position to not mess with the others
                        Mat rvecs = new Mat(); // rotation vector
                        Mat tvecs = new Mat(); // translation vector
                        ArucoInvoke.EstimatePoseSingleMarkers(corners, markersLength, cameraMatrix[c], distortionMatrix[c], rvecs, tvecs);
                        SendDetectedRect(c, frame, ids, corners, -1, tvecs);

                        skew = SkewRects(c, ids, corners, 0, -1);
                        SendDetectedRect(c, frame, ids, skew, 0, tvecs);
                        skew = SkewRects(c, ids, corners, 1, -1);
                        SendDetectedRect(c, frame, ids, skew, 1, tvecs);
                        skew = SkewRects(c, ids, corners, 2, -1);
                        SendDetectedRect(c, frame, ids, skew, 2, tvecs);
                        skew = SkewRects(c, ids, corners, 3, -1);
                        SendDetectedRect(c, frame, ids, skew, 3, tvecs);
                    }
                    int queueSize = queueCount[c];
                    queueCount[c] = 0;
                    for (int i = 0; i < queueSize; i++) {
                        if (posQueue[c][i] == null || rotQueue[c][i] == null) continue;
                        if (posQueue[c][i][2] == 0) continue;
                        try {
                            if (!Program.debugSendTrackerOSC && shouldShowFrame)
                                Draw.Axis(frame, cameraMatrix[c], distortionMatrix[c], rotQueue[c][i], posQueue[c][i], markersLength * 0.5f * sclQueue[c][i]);
                        } catch {
                            Console.WriteLine("lol");
                        }
                    }
                    //---------------------

                    //Display current frame plus drawings
                    if (shouldShowFrame) {
                        CvInvoke.Imshow($"Image{c}", frame);
                        CvInvoke.WaitKey(1);
                    }
                    frame.Dispose();
                }
                Tag.cameras[c].newData = true;
                Tag.newInfoReady = true;
                arucoThreadWorkBenchmark.Stop();
                Program.threadsWorkTime[c + 2] = arucoThreadWorkBenchmark.Elapsed.TotalMilliseconds;
                Program.threadsIdleTime[c + 2] = arucoThreadIdleBenchmark.Elapsed.TotalMilliseconds;
            }

            static void SendDetectedRect(int c, Mat frame, VectorOfInt ids, VectorOfVectorOfPointF corners, int altCorner, Mat tvecs0) {
                Mat rvecs = new Mat(); // rotation vector
                Mat tvecs = new Mat(); // translation vector
                ArucoInvoke.EstimatePoseSingleMarkers(corners, markersLength, cameraMatrix[c], distortionMatrix[c], rvecs, tvecs);
                tvecs = tvecs0;

                //Draw 3D orthogonal axis on markers using estimated pose
                for (int i = 0; i < ids.Size; i++) {
                    //if (ids[i] != 0 && ids[i] != 4) continue;
                    using (Mat rvecMat = rvecs.Row(i))
                    using (Mat tvecMat = tvecs.Row(i))
                    using (VectorOfDouble rvec = new VectorOfDouble())
                    using (VectorOfDouble tvec = new VectorOfDouble()) {
                        double[] values = new double[3];
                        rvecMat.CopyTo(values);
                        rvec.Push(values);
                        tvecMat.CopyTo(values);
                        tvec.Push(values);
                        Mat RotMat = GetRotationMatrixFromRotationVector(rvec);
                        double[] dRotMat = new double[3 * 3];
                        RotMat.CopyTo(dRotMat);
                        Vector3 pos = new Vector3((float)tvec[0] / 1000f, (float)tvec[1] / 1000f, (float)tvec[2] / 1000f);
                        Matrix4x4 rot = new Matrix4x4(
                            (float)dRotMat[0], (float)dRotMat[3], (float)dRotMat[6], 0f,
                            (float)dRotMat[1], (float)dRotMat[4], (float)dRotMat[7], 0f,
                            (float)dRotMat[2], (float)dRotMat[5], (float)dRotMat[8], 0f,
                            0f, 0f, 0f, 1f);
                        //if (!Tag.newInfoReady)
                        Tag.RecieveTrackerAsync(ids[i], c, rot, pos, altCorner);
                        //Console.WriteLine($"{c} - {i} = {pos.X}\t{pos.Y}\t{pos.Z}");
                        pos.Y -= 0.1f;
                        Matrix4x4 finalMat = Matrix4x4.Multiply(rot, Matrix4x4.CreateTranslation(pos));
                        if (Program.debugSendTrackerOSC && altCorner == -1 && Program.wantToShowFrame)
                            Draw.Axis(frame, cameraMatrix[c], distortionMatrix[c], rvec, tvec, markersLength);
                    }

                }
            }
        }

        private static VectorOfVectorOfPointF SmoothCorners(int c, VectorOfInt ids, VectorOfVectorOfPointF corners) {
            PointF[][] rects = new PointF[ids.Size][];
            for (int i = 0; i < ids.Size; i++) {
                rects[i] = new PointF[4];
                int id = ids[i];
                PointF[] prev = betterRects[c][id].prevVals;
                PointF[] lokd = betterRects[c][id].lockedVals;
                PointF[] curr = corners[i].ToArray();
                int wrongs = 0;
                for (int j = 0; j < 4; j++) {
                    if (Math.Abs(prev[j].X - curr[j].X) > cornersMaxDistance || Math.Abs(prev[j].Y - curr[j].Y) > cornersMaxDistance)
                        wrongs++;
                }
                if (wrongs == 0) {
                    if (betterRects[c][id].locked == false) {
                        betterRects[c][id].locked = true;
                        betterRects[c][id].lockedVals = curr;
                    }
                } else {
                    if (betterRects[c][id].locked) {
                        wrongs = 0;
                        for (int j = 0; j < 4; j++) {
                            if (Math.Abs(lokd[j].X - curr[j].X) > cornersMaxDistance || Math.Abs(lokd[j].Y - curr[j].Y) > cornersMaxDistance)
                                wrongs++;
                        }
                        if (wrongs > 1) {
                            betterRects[c][id].locked = false;
                        }
                    }
                }
                if (betterRects[c][id].locked) {
                    for (int j = 0; j < 4; j++) {
                        rects[i][j] = betterRects[c][id].smoothedVals[j];
                        rects[i][j].X += (corners[i][j].X - betterRects[c][id].smoothedVals[j].X) * cornersSmoothFactor;
                        rects[i][j].Y += (corners[i][j].Y - betterRects[c][id].smoothedVals[j].Y) * cornersSmoothFactor;
                    }
                    betterRects[c][id].smoothedVals = rects[i];
                } else {
                    betterRects[c][id].smoothedVals = curr;
                }
                rects[i] = betterRects[c][id].smoothedVals;
                betterRects[c][id].prevVals = curr;
            }
            corners = new(rects);
            return corners;
        }
        static VectorOfVectorOfPointF SkewRects(int c, VectorOfInt ids, VectorOfVectorOfPointF corners, int c1, int c2) {
            PointF[][] rects = new PointF[ids.Size][];
            for (int i = 0; i < ids.Size; i++) {
                rects[i] = new PointF[4];
                rects[i] = corners[i].ToArray();
                Vector3 center = new Vector3();
                for (int j = 0; j < 4; j++) {
                    center.X += rects[i][j].X;
                    center.Y += rects[i][j].Y;
                }
                center /= 4;
                rects[i][c1].X = rects[i][c1].X * 0.95f + center.X * 0.05f;
                rects[i][c1].Y = rects[i][c1].Y * 0.95f + center.Y * 0.05f;
                int[] inv = new int[] { 2, 3, 0, 1 };
                int ic1 = inv[c1];
                rects[i][ic1].X = rects[i][ic1].X + (rects[i][ic1].X - center.X) * 0.08f;
                rects[i][ic1].Y = rects[i][ic1].Y + (rects[i][ic1].Y - center.Y) * 0.08f;
                if (c2 == -1) continue;
                rects[i][c2].X = rects[i][c2].X * 0.95f - center.X * 0.05f;
                rects[i][c2].Y = rects[i][c2].Y * 0.95f - center.Y * 0.05f;
            }
            return new(rects);
        }

        //TODO: add multiple camera support
        static int[] queueCount = new int[3];
        static int maxQueue = 55;
        static float[][] sclQueue = new float[][] { new float[maxQueue], new float[maxQueue] };
        static VectorOfDouble[][] posQueue = new VectorOfDouble[][] { new VectorOfDouble[maxQueue], new VectorOfDouble[maxQueue] };
        static VectorOfDouble[][] rotQueue = new VectorOfDouble[][] { new VectorOfDouble[maxQueue], new VectorOfDouble[maxQueue] };
        public static void DrawAxis(Matrix4x4 rot, float scl = 1) {
            for (int i = 0; i < Tag.cameras.Length; i++) {
                Matrix4x4 camMat = Tag.cameras[i].matrix;
                DrawAxis(rot, camMat, i, scl);
            }
        }
        public static void DrawAxis(Matrix4x4 rot, Matrix4x4 camMat, int cam, float scl = 1) {
            Matrix4x4 invCam;
            Matrix4x4.Invert(camMat, out invCam);
            Matrix4x4 mat = Matrix4x4.Multiply(rot, invCam);
            DrawAxis(mat, cam, scl);
        }
        public static void DrawAxis(Matrix4x4 rot, int cam, float scl = 1) {
            if (!Program.wantToShowFrame) return;
            Vector3 pos = rot.Translation;
            double[] asdasd = new double[] {
                rot.M11,rot.M21,rot.M31,
                rot.M12,rot.M22,rot.M32,
                rot.M13,rot.M23,rot.M33
            };
            Mat asd = new Mat(new Size(3, 3), DepthType.Cv64F, 1);
            asd.SetTo(asdasd);
            Mat asdsds = new Mat(new Size(1, 3), DepthType.Cv64F, 1);
            CvInvoke.Rodrigues(asd, asdsds);
            double[] valwewewues = new double[3];
            asdsds.CopyTo(valwewewues);
            VectorOfDouble rvec2 = new VectorOfDouble(new double[] { valwewewues[0], valwewewues[1], valwewewues[2] }); //pitch roll yaw
            VectorOfDouble tvec2 = new VectorOfDouble(new double[] { pos.X * 1000f, pos.Y * 1000f, pos.Z * 1000f });
            if (queueCount[cam] == maxQueue) return;
            posQueue[cam][queueCount[cam]] = tvec2;
            rotQueue[cam][queueCount[cam]] = rvec2;
            if (scl == 0)
                scl = 0.1f;
            sclQueue[cam][queueCount[cam]] = scl * 3;
            queueCount[cam]++;
        }
        static Mat GetRotationMatrixFromRotationVector(VectorOfDouble rvec) {
            Mat rmat = new Mat();
            double[] dRotMat = new double[4 * 4];
            rmat.CopyTo(dRotMat);
            CvInvoke.Rodrigues(rvec, rmat);
            return rmat;
        }
    }
}
