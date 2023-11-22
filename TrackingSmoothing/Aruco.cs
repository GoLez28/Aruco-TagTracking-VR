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

namespace TagTracking {
    class Aruco {
        class RectEx {
            public PointF[] smoothedVals = new PointF[4];
            public PointF[] lockedVals = new PointF[4];
            public PointF[] prevVals = new PointF[4];
            public bool needUpdate = true;
            public int time = 0;
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
        public static List<(int, int)> perMarkerLength = new List<(int, int)>();
        public static Mat[] cameraMatrix;
        public static Mat[] distortionMatrix;

        public static bool useSmoothCorners = true;
        public static float cornersMaxDistance = 1;
        public static float cornersSmoothFactor = 0.1f;
        public static int refinementMethod = 1;
        public static float refinementMinAcc = -1337;
        public static int refinementWinSize = -1337;
        public static string markerDictionary = "4x4x50";
        public static int markerIdsLength = 50;
        public static bool useDynamicFraming = true;
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
            ArucoDict = new Dictionary(Dictionary.PredefinedDictionaryName.Dict4X4_50); // bits x bits (per marker) _ number of markers in dict
            GetDictionary();
            for (int i = 0; i < Tag.cameras.Length; i++) {
                sclQueue[i] = new float[maxQueue];
                posQueue[i] = new VectorOfDouble[maxQueue];
                rotQueue[i] = new VectorOfDouble[maxQueue];
            }
            betterRects = new RectEx[Tag.cameras.Length][];
            for (int i = 0; i < betterRects.Length; i++) {
                betterRects[i] = new RectEx[markerIdsLength];
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
                    if (Tag.cameras[i].url.Equals(""))
                        capture[i] = new VideoCapture(Tag.cameras[i].index, VideoCapture.API.DShow);
                    else
                        capture[i] = new VideoCapture(Tag.cameras[i].url, VideoCapture.API.Ffmpeg);
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
            //ArucoDict = new Dictionary(false);
            if (markerIdsLength == 50) {
                markersX = 10;
                markersY = 5;
            } else if (markerIdsLength == 100) {
                markersX = 10;
                markersY = 10;
            } else if (markerIdsLength == 250) {
                markersX = 10;
                markersY = 25;
            } else if (markerIdsLength == 1000) {
                markersX = 40;
                markersY = 25;
            } else if (markerIdsLength == 1024) {
                markersX = 32;
                markersY = 32;
            } else if (markerIdsLength == 36) {
                markersX = 6;
                markersY = 6;
            }
            GridBoard ArucoBoard = null;
            ArucoBoard = new GridBoard(markersX, markersY, markersLength, markersSeparation, ArucoDict);
            PrintArucoBoard(ArucoBoard, markersX, markersY, markersLength, markersSeparation);
            CameraCalibrate.DrawBoard();

            ArucoParameters = new DetectorParameters();
            ApplyParams();
            //ArucoParameters.PolygonalApproxAccuracyRate = 0.1;

            // Calibration done with https://docs.opencv.org/3.4.3/d7/d21/tutorial_interactive_calibration.html
            GetCameraParameters();
        }

        private static void GetDictionary() {
            if (markerDictionary.Equals("4x4x50"))
                ArucoDict = new Dictionary(Dictionary.PredefinedDictionaryName.Dict4X4_50);
            else if (markerDictionary.Equals("4x4x100"))
                ArucoDict = new Dictionary(Dictionary.PredefinedDictionaryName.Dict4X4_100);
            else if (markerDictionary.Equals("4x4x250"))
                ArucoDict = new Dictionary(Dictionary.PredefinedDictionaryName.Dict4X4_250);
            else if (markerDictionary.Equals("4x4x1000"))
                ArucoDict = new Dictionary(Dictionary.PredefinedDictionaryName.Dict4X4_1000);
            else if (markerDictionary.Equals("5x5x50"))
                ArucoDict = new Dictionary(Dictionary.PredefinedDictionaryName.Dict5X5_50);
            else if (markerDictionary.Equals("5x5x100"))
                ArucoDict = new Dictionary(Dictionary.PredefinedDictionaryName.Dict5X5_100);
            else if (markerDictionary.Equals("5x5x250"))
                ArucoDict = new Dictionary(Dictionary.PredefinedDictionaryName.Dict5X5_250);
            else if (markerDictionary.Equals("5x5x1000"))
                ArucoDict = new Dictionary(Dictionary.PredefinedDictionaryName.Dict5X5_1000);
            else if (markerDictionary.Equals("6x6x50"))
                ArucoDict = new Dictionary(Dictionary.PredefinedDictionaryName.Dict6X6_50);
            else if (markerDictionary.Equals("6x6x100"))
                ArucoDict = new Dictionary(Dictionary.PredefinedDictionaryName.Dict6X6_100);
            else if (markerDictionary.Equals("6x6x250"))
                ArucoDict = new Dictionary(Dictionary.PredefinedDictionaryName.Dict6X6_250);
            else if (markerDictionary.Equals("6x6x1000"))
                ArucoDict = new Dictionary(Dictionary.PredefinedDictionaryName.Dict6X6_1000);
            else if (markerDictionary.Equals("7x7x50"))
                ArucoDict = new Dictionary(Dictionary.PredefinedDictionaryName.Dict7X7_50);
            else if (markerDictionary.Equals("7x7x100"))
                ArucoDict = new Dictionary(Dictionary.PredefinedDictionaryName.Dict7X7_100);
            else if (markerDictionary.Equals("7x7x250"))
                ArucoDict = new Dictionary(Dictionary.PredefinedDictionaryName.Dict7X7_250);
            else if (markerDictionary.Equals("7x7x1000"))
                ArucoDict = new Dictionary(Dictionary.PredefinedDictionaryName.Dict7X7_1000);
            else if (markerDictionary.Equals("ori5x5x1024"))
                ArucoDict = new Dictionary(Dictionary.PredefinedDictionaryName.DictArucoOriginal);
            else if (markerDictionary.Contains("custom4x4x36")) {
                ArucoDict = new Dictionary(36, 4);
                Console.WriteLine("The custom 4x4 36 tags will be removed soon!");
            } else markerDictionary = "4x4x50";

            string[] mDictSplit = markerDictionary.Split('x');
            if (mDictSplit.Length == 3)
                int.TryParse(mDictSplit[2], out markerIdsLength);
        }

        public static void ApplyParams() {
            ArucoParameters = DetectorParameters.GetDefault();
            ArucoParameters.CornerRefinementMethod = (DetectorParameters.RefinementMethod)refinementMethod;
            ArucoParameters.CornerRefinementMaxIterations = 30;
            ArucoParameters.CornerRefinementWinSize = refinementWinSize == -1337 ? (refinementMethod == 1 ? 2 : 5) : refinementWinSize;
            ArucoParameters.CornerRefinementMinAccuracy = refinementMinAcc == -1337 ? (refinementMethod == 1 ? 50 : 0.1) : refinementMinAcc;
        }

        public static void GetCameraParameters() {
            Mat[] newCameraMatrix = new Mat[Tag.cameras.Length];
            Mat[] newDistortionMatrix = new Mat[Tag.cameras.Length];
            for (int i = 0; i < Tag.cameras.Length; i++) {
                newCameraMatrix[i] = new Mat(new Size(3, 3), DepthType.Cv32F, 1);
                newDistortionMatrix[i] = new Mat(1, 8, DepthType.Cv32F, 1);
            }
            for (int c = 0; c < Tag.cameras.Length; c++) {
                string cameraConfigurationFile = "cameraParameters" + (c + 1) + ".xml";
                if (!Tag.cameras[c].file.Equals(""))
                    cameraConfigurationFile = Tag.cameras[c].file;
                if (!System.IO.File.Exists(cameraConfigurationFile)) {
                    Console.WriteLine("Configuration file " + cameraConfigurationFile + " does not exist!");
                    return;
                }
                FileStorage fs = new FileStorage(cameraConfigurationFile, FileStorage.Mode.Read);
                if (!fs.IsOpened) {
                    Console.WriteLine("Could not open configuration file " + cameraConfigurationFile);
                    return;
                }
                fs["cameraMatrix"].ReadMat(newCameraMatrix[c]);
                fs["dist_coeffs"].ReadMat(newDistortionMatrix[c]);
            }
            cameraMatrix = newCameraMatrix;
            distortionMatrix = newDistortionMatrix;
        }

        static float smoothbenchmark = 0;
        public static (VectorOfInt, VectorOfVectorOfPointF) GetCut(Mat frame, int x, int y, int width, int height, int i, int c) {
            VectorOfInt ids = new VectorOfInt(); // name/id of the detected markers
            VectorOfVectorOfPointF corners = new VectorOfVectorOfPointF(); // corners of the detected marker
            try {
                VectorOfVectorOfPointF rejected = new VectorOfVectorOfPointF(); // rejected contours
                Mat roi = new Mat(frame, new Rectangle(x, y, width, height));
                ArucoInvoke.DetectMarkers(roi, ArucoDict, corners, ids, ArucoParameters, rejected);
                //Mat show = new Mat(roi, new Rectangle(0, 0, frame.Width, frame.Height));
                //CvInvoke.Imshow($"Cut{i}-{c}", show);
                //CvInvoke.WaitKey(1);
                roi.Dispose();
            } catch {
                Console.WriteLine();
            }
            return (ids, corners);
        }
        public static void Update(int c) {
            int frameCount = 0;
            while (true) {
                System.Diagnostics.Stopwatch arucoThreadWorkBenchmark = new System.Diagnostics.Stopwatch();
                System.Diagnostics.Stopwatch arucoThreadIdleBenchmark = new System.Diagnostics.Stopwatch();
                arucoThreadWorkBenchmark.Start();
                if (Program.wantToCloseWindows) {
                    CvInvoke.DestroyWindow($"Image{c}");
                    System.Threading.Thread.Sleep(1000);
                    Program.wantToCloseWindows = false;
                }
                bool shouldShowFrame = Program.wantToShowFrame;
                //Capture a frame with webcam
                Mat frame = new Mat();
                if (c >= capture.Length) {
                    Console.WriteLine("Wait... this is illegal");
                }
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
                        if (Tag.cameras[c].canEnterInWaitMode) {
                            bool ActiveCams = false;
                            for (int i = 0; i < Tag.cameras.Length; i++) {
                                if (!Tag.cameras[i].inWaitMode) {
                                    ActiveCams = true;
                                    break;
                                }
                            }
                            int sleepTime = Tag.cameras[c].waitTimeUpdate;
                            if (!ActiveCams) sleepTime *= 2;
                            if (Program.timer.ElapsedMilliseconds - Tag.cameras[c].lastSeenMarkerTime <= Tag.cameras[c].timeBeforeWaiting ||
                            Program.timer.ElapsedMilliseconds - Tag.cameras[c].lastWaitCheck >= sleepTime) 
                                break;
                        } else
                            break;
                    }
                    Tag.cameras[c].skipFrameCount++;
                    System.Threading.Thread.Sleep(4);
                }
                Tag.cameras[c].lastWaitCheck = Program.timer.ElapsedMilliseconds;
                frameCount++;
                arucoThreadIdleBenchmark.Stop();
                arucoThreadWorkBenchmark.Start();
                bool correctRes = Tag.cameras[c].rsHeight > 10 && Tag.cameras[c].rsWidth > 10;
                int newRsWidth = Tag.cameras[c].rsWidth;
                int newRsHeight = Tag.cameras[c].rsHeight;
                float xRatio = 1f, yRatio = 1f;
                if (correctRes) {
                    if (Tag.cameras[c].adjustCurrentDistortion) {
                        xRatio = (float)Tag.cameras[c].width / (float)Tag.cameras[c].rsWidth;
                        yRatio = (float)Tag.cameras[c].height / (float)Tag.cameras[c].rsHeight;
                    }
                }
                if (Program.performanceMode && Program.performanceUnderSample > 1) {
                    if (correctRes) {
                        newRsWidth = (int)(newRsWidth / Program.performanceUnderSample);
                        newRsHeight = (int)(newRsHeight / Program.performanceUnderSample);
                        xRatio *= Program.performanceUnderSample;
                        yRatio *= Program.performanceUnderSample;
                    } else {
                        correctRes = true;
                        newRsWidth = (int)(Tag.cameras[c].width / Program.performanceUnderSample);
                        newRsHeight = (int)(Tag.cameras[c].height / Program.performanceUnderSample);
                        xRatio *= Program.performanceUnderSample;
                        yRatio *= Program.performanceUnderSample;
                    }
                }
                Tag.cameras[c].xRatio = xRatio;
                Tag.cameras[c].yRatio = yRatio;
                if (correctRes) {
                    frame = ResizeFrame(c, frameCount, frame, newRsWidth, newRsHeight);
                }
                bool adjustBrightness = Tag.cameras[c].brightness != 1f;
                if (adjustBrightness) {
                    System.Diagnostics.Stopwatch sw = new System.Diagnostics.Stopwatch();
                    sw.Start();
                    frame *= Tag.cameras[c].brightness;
                    sw.Stop();
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
                if (frame == null)
                    Console.WriteLine($"Wrong video input!!! (cam: {c})");
                if (frame != null && !frame.IsEmpty) {
                    //Detect markers on last retrieved frame
                    VectorOfInt ids = new VectorOfInt(); // name/id of the detected markers
                    VectorOfVectorOfPointF corners = new VectorOfVectorOfPointF(); // corners of the detected marker
                    VectorOfVectorOfPointF rejected = new VectorOfVectorOfPointF(); // rejected contours

                    bool halfFrame = useDynamicFraming && frameCount % 6 != 0;
                    halfFrame &= !(TrackerCalibrate.startCalibrating || CameraCalibrate.startCalibrating || RoomCalibrate.getRawTrackersStep > -1);
                    halfFrame &= !Tag.cameras[c].inWaitMode;
                    if (!halfFrame) {
                        ArucoInvoke.DetectMarkers(frame, ArucoDict, corners, ids, ArucoParameters, rejected);
                    } else {
                        List<(int x, int y, int w, int h)> rects;
                        GetFrameRectangle(frame, c, (int)(frame.Width * xRatio), (int)(frame.Height * yRatio), out rects);
                        int numTasks = rects.Count;
                        Task<(VectorOfInt, VectorOfVectorOfPointF)>[] tasks = new Task<(VectorOfInt, VectorOfVectorOfPointF)>[numTasks];
                        if (numTasks > 0) {
                            for (int i = 0; i < numTasks; i++) {
                                int taskId = i;
                                (int x, int y, int width, int height) = rects[i];
                                if (correctRes) {
                                    x = (int)(x / xRatio);
                                    width = (int)(width / xRatio);
                                    y = (int)(y / yRatio);
                                    height = (int)(height / yRatio);
                                }
                                rects[i] = (x, y, width, height);
                                tasks[i] = Task.Run(() => GetCut(frame, x, y, width, height, i, c));
                            }
                            Task.WaitAll(tasks);
                            for (int i = 0; i < numTasks; i++) {
                                int taskId = i;
                                (int x, int y, int width, int height) = rects[i];
                                CvInvoke.Line(frame, new Point(x, y), new Point(x + width, y), new Bgr(Color.PaleTurquoise).MCvScalar, 1, LineType.AntiAlias);
                                CvInvoke.Line(frame, new Point(x + width, y), new Point(x + width, y + height), new Bgr(Color.PaleTurquoise).MCvScalar, 1, LineType.AntiAlias);
                                CvInvoke.Line(frame, new Point(x + width, y + height), new Point(x, y + height), new Bgr(Color.PaleTurquoise).MCvScalar, 1, LineType.AntiAlias);
                                CvInvoke.Line(frame, new Point(x, y + height), new Point(x, y), new Bgr(Color.PaleTurquoise).MCvScalar, 1, LineType.AntiAlias);
                            }
                            for (int i = 0; i < numTasks; i++) {
                                ids.Push(tasks[i].Result.Item1);
                                var i2 = tasks[i].Result.Item2;
                                i2 = AdjustDetectedRectsToCut(i2, rects[i].x, rects[i].y);
                                corners.Push(i2);
                            }
                        }
                    }
                    //smooth corners
                    try {
                        if (Program.preNoise == 1) {
                            corners = SmoothCorners(c, ids, corners, halfFrame);
                        }
                    } catch (Exception e) {
                        Console.WriteLine("Couldnt smooth corners\n" + e);
                    }
                    // If we detected at least one marker
                    if (ids.Size > 0) {
                        Tag.cameras[c].lastSeenMarkerTime = Program.timer.ElapsedMilliseconds;
                        if (Tag.cameras[c].inWaitMode) {
                            Tag.cameras[c].inWaitMode = false;
                            //Console.WriteLine("Active camera " + c);
                        }
                        //Draw detected markers
                        VectorOfVectorOfPointF skew;
                        //corners = SkewRects(c, ids, corners, 2, -1);
                        if (shouldShowFrame)
                            ArucoInvoke.DrawDetectedMarkers(frame, corners, ids, new MCvScalar(255, 0, 255));
                        corners = AdjustDetectedRectsToLowRes(corners, xRatio, yRatio);

                        //Estimate pose for each marker using camera calibration matrix and distortion coefficents
                        Mat rvecs = new Mat(); // rotation vector
                        Mat tvecs = new Mat(); // translation vector
                        ArucoInvoke.EstimatePoseSingleMarkers(corners, markersLength, cameraMatrix[c], distortionMatrix[c], rvecs, tvecs);
                        SendDetectedRect(c, frame, ids, corners, -1, tvecs);

                        for (int i = 0; i < 4; i++) {
                            skew = SkewRects(c, ids, corners, i, -1);
                            SendDetectedRect(c, frame, ids, skew, i, tvecs);
                        }
                    } else {
                        if (!Tag.cameras[c].inWaitMode && Program.timer.ElapsedMilliseconds - Tag.cameras[c].lastSeenMarkerTime > Tag.cameras[c].timeBeforeWaiting) {
                            Tag.cameras[c].inWaitMode = true;
                            //Console.WriteLine("Suspending camera " + c);
                        }
                    }
                    int queueSize = queueCount[c];
                    queueCount[c] = 0;
                    for (int i = 0; i < queueSize; i++) {
                        if (posQueue[c][i] == null || rotQueue[c][i] == null) continue;
                        if (posQueue[c][i][2] == 0) continue;
                        try {
                            if (!Program.debugSendTrackerOSC && shouldShowFrame) {
                                Draw.Shape(frame, cameraMatrix[c], distortionMatrix[c], rotQueue[c][i], posQueue[c][i], markersLength * 0.5f * sclQueue[c][i], typeQueue[c][i]);
                            }
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
        }

        private static Mat ResizeFrame(int c, int frameCount, Mat frame, int newRsWidth, int newRsHeight) {
            int newWidth = newRsWidth;
            int newHeight = newRsHeight;
            System.Diagnostics.Stopwatch sw = new System.Diagnostics.Stopwatch();
            sw.Start();

            //i dont know which is more cpu efficient, of course this is faster, but uses more cpu
            //Mat resizedMat = new Mat(new Size(newWidth, newHeight), DepthType.Cv8U, 3);
            //bool xb = frameCount % 2 == 0;
            //CvInvoke.Resize(frame, resizedMat, new Size(newWidth, newHeight), interpolation: xb ? Inter.Nearest : Inter.NearestExact);
            //frame = resizedMat;

            bool xb = frameCount % 2 == 0;
            bool yb = frameCount % 4 > 1;
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
                fixed (byte* pntr1 = &dsds[0], pntr2 = &asd[0]) {
                    byte* pntr_1 = pntr1;
                    byte* pntr_2 = pntr2;
                    for (int x = 0; x < newWidth; x++) {
                        ptrx1[x] = x * 3;
                        int x2 = xb ? (int)(x / mult) : (int)Math.Ceiling(x / mult);
                        ptrx2[x] = x2 * 3;
                    }
                    for (int y = 0; y < newHeight; y++) {
                        int y2 = yb ? (int)(y / mult) : (int)Math.Ceiling(y / mult);
                        int ay1 = y * nw3;
                        int ay2 = y2 * ow3;
                        byte* c1 = pntr_1 + ay1;
                        byte* c2 = pntr_2 + ay2;
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
            if (c == 0) {
                smoothbenchmark += (float)(sw.Elapsed.TotalMilliseconds - smoothbenchmark) * 0.03f;
                Console.WriteLine($"cam {c}: {smoothbenchmark}ms");
            }

            return frame;
        }

        static (int, int, int) CalculateIntersectionArea(int x1, int y1, int width1, int height1, int x2, int y2, int width2, int height2) {
            // Calcular las coordenadas y dimensiones del rectángulo de intersección
            int xIntersect = Math.Max(x1, x2);
            int yIntersect = Math.Max(y1, y2);
            int widthIntersect = Math.Max(0, Math.Min(x1 + width1, x2 + width2) - xIntersect);
            int heightIntersect = Math.Max(0, Math.Min(y1 + height1, y2 + height2) - yIntersect);

            // Calcular el área de intersección
            int intersectionArea = widthIntersect * heightIntersect;

            return (intersectionArea, widthIntersect, heightIntersect);
        }
        static VectorOfVectorOfPointF AdjustDetectedRectsToCut(VectorOfVectorOfPointF corners, float x, float y) {
            PointF[][] rects = new PointF[corners.Size][];
            for (int i = 0; i < corners.Size; i++) {
                rects[i] = new PointF[4];
                rects[i] = corners[i].ToArray();
                for (int j = 0; j < rects[i].Length; j++) {
                    rects[i][j].X += x;
                    rects[i][j].Y += y;
                }
            }
            return new(rects);
        }
        static VectorOfVectorOfPointF AdjustDetectedRectsToLowRes(VectorOfVectorOfPointF corners, float xRatio, float yRatio) {
            PointF[][] rects = new PointF[corners.Size][];
            for (int i = 0; i < corners.Size; i++) {
                rects[i] = new PointF[4];
                rects[i] = corners[i].ToArray();
                for (int j = 0; j < rects[i].Length; j++) {
                    rects[i][j].X *= xRatio;
                    rects[i][j].Y *= yRatio;
                }
            }
            return new(rects);
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

                    for (int j = 0; j < perMarkerLength.Count; j++) {
                        (int id, int length) = perMarkerLength[j];
                        if (id != ids[i]) continue;
                        float m = (float)markersLength / length;
                        pos /= m;
                    }
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

        private static void GetFrameRectangle(Mat frame, int c, int width, int height, out List<(int x, int y, int w, int h)> rects) {
            MCvPoint3D32f[] dotVerts = new MCvPoint3D32f[] {
                    new MCvPoint3D32f(0, 0, 0),
                    new MCvPoint3D32f(markersLength, markersLength, 0),
                    new MCvPoint3D32f(markersLength, -markersLength, 0),
                };
            rects = new List<(int, int, int, int)>();
            VectorOfDouble rvec = new VectorOfDouble(3);
            for (int j = 0; j < Tag.trackers.Length; j++) {
                if (Program.timer.ElapsedMilliseconds - Tag.finals[j].lastTimeSeen > 500) continue;
                float minX = 100000;
                float maxX = 0;
                float minY = 100000;
                float maxY = 0;
                for (int i = 0; i < Tag.trackers[j].trackers.Length; i++) {
                    if (Tag.trackers[j].trackers[i].updateCount[c] > 20) continue;
                    Tag.SingleTracker st = Tag.trackers[j].trackers[i].singles[c];
                    VectorOfDouble tvec = new VectorOfDouble(new double[] { st.pos.X * 1000, st.pos.Y * 1000, st.pos.Z * 1000 });
                    PointF[] points = CvInvoke.ProjectPoints(dotVerts, rvec, tvec, cameraMatrix[c], distortionMatrix[c], null);
                    float tVel = Utils.GetDistance(st.pos, st.p_pos);
                    tVel = ((tVel / markersLength) * 750) + 1;
                    float addX = Math.Max(Math.Abs(points[0].X - points[1].X), Math.Abs(points[0].X - points[2].X)) * tVel;
                    float addY = Math.Max(Math.Abs(points[0].Y - points[1].Y), Math.Abs(points[0].Y - points[2].Y)) * tVel;
                    minX = Math.Min(minX, points[0].X - addX);
                    minY = Math.Min(minY, points[0].Y - addY);
                    maxX = Math.Max(maxX, points[0].X + addX);
                    maxY = Math.Max(maxY, points[0].Y + addY);
                }
                if (minX == 100000 || minY == 100000 || maxX == 0 || maxY == 0) continue;
                minX = Math.Max(0, minX - 100);
                minY = Math.Max(0, minY - 100);
                maxX = Math.Min(width, maxX + 100) - minX;
                maxY = Math.Min(height, maxY + 100) - minY;
                rects.Add(((int)minX, (int)minY, (int)maxX, (int)maxY));
            }

            for (int i = 0; i < rects.Count - 1; i++) {
                var r1 = rects[i];
                for (int j = i + 1; j < rects.Count; j++) {
                    var r2 = rects[j];
                    //discarded cross area
                    (int crossArea, int crossWidth, int crossHeight) = CalculateIntersectionArea(r1.x, r1.y, r1.w, r1.h, r2.x, r2.y, r2.w, r2.h);
                    if (crossArea <= 0) {
                        crossWidth = 0;
                        crossHeight = 0;
                    }
                    int area1 = r1.w * r1.h;
                    int area2 = r2.w * r2.h;
                    int sumArea = area1 + area2;
                    int totalX = Math.Min(r1.x, r2.x);
                    int totalY = Math.Min(r1.y, r2.y);
                    int totalW = r1.w + r2.w - crossWidth;
                    int totalH = r1.h + r2.h - crossHeight;
                    int outsideArea = (totalW * totalH) - sumArea;
                    float ratio = (float)outsideArea / sumArea;
                    if (ratio < 0.25) {
                        rects[i] = (totalX, totalY, totalW, totalH);
                        r1 = rects[i];
                        rects.RemoveAt(j);
                        j--;
                    }

                    int x = totalX;
                    int y = totalY;
                    int w = totalW;
                    int h = totalH;
                    if (!Program.debugSendTrackerOSC) continue;
                    CvInvoke.Line(frame, new Point(x, y), new Point(x + w, y), new Bgr(Color.Orange).MCvScalar, 2, LineType.AntiAlias);
                    CvInvoke.Line(frame, new Point(x + w, y), new Point(x + w, y + h), new Bgr(Color.Orange).MCvScalar, 2, LineType.AntiAlias);
                    CvInvoke.Line(frame, new Point(x + w, y + h), new Point(x, y + h), new Bgr(Color.Orange).MCvScalar, 2, LineType.AntiAlias);
                    CvInvoke.Line(frame, new Point(x, y + h), new Point(x, y), new Bgr(Color.Orange).MCvScalar, 2, LineType.AntiAlias);
                }
            }
        }

        private static VectorOfVectorOfPointF SmoothCorners(int c, VectorOfInt ids, VectorOfVectorOfPointF corners, bool fullFrame) {
            PointF[][] rects = new PointF[ids.Size][];
            float quickCornersSmoothFactor = (float)Math.Pow(cornersSmoothFactor, 0.6f);
            for (int i = 0; i < ids.Size; i++) {
                rects[i] = new PointF[4];
                int id = ids[i];
                RectEx rectEx = betterRects[c][id];
                PointF[] prev = rectEx.prevVals;
                PointF[] lokd = rectEx.lockedVals;
                PointF[] curr = corners[i].ToArray();
                int wrongs = 0;
                for (int j = 0; j < 4; j++) {
                    if (Math.Abs(prev[j].X - curr[j].X) > cornersMaxDistance || Math.Abs(prev[j].Y - curr[j].Y) > cornersMaxDistance)
                        wrongs++;
                }
                if (wrongs == 0) {
                    if (rectEx.locked == false) {
                        rectEx.locked = true;
                        rectEx.lockedVals = curr;
                    }
                } else {
                    if (rectEx.locked) {
                        wrongs = 0;
                        for (int j = 0; j < 4; j++) {
                            if (Math.Abs(lokd[j].X - curr[j].X) > cornersMaxDistance || Math.Abs(lokd[j].Y - curr[j].Y) > cornersMaxDistance)
                                wrongs++;
                        }
                        if (wrongs > 1) {
                            rectEx.locked = false;
                        }
                    }
                }
                if (rectEx.locked) {
                    float smoothness = rectEx.time >= 30 ? cornersSmoothFactor : quickCornersSmoothFactor;
                    for (int j = 0; j < 4; j++) {
                        rects[i][j] = rectEx.smoothedVals[j];
                        rects[i][j].X += (corners[i][j].X - rectEx.smoothedVals[j].X) * smoothness;
                        rects[i][j].Y += (corners[i][j].Y - rectEx.smoothedVals[j].Y) * smoothness;
                    }
                    rectEx.smoothedVals = rects[i];
                    if (rectEx.time >= 30 && fullFrame && rectEx.needUpdate) {
                        rects[i].CopyTo(rectEx.lockedVals, 0);
                        rectEx.time = 30;
                        rectEx.needUpdate = false;
                    } else if (rectEx.time >= 60) {
                        rectEx.time = 29;
                        rectEx.needUpdate = true;
                    }
                } else {
                    rectEx.smoothedVals = curr;
                    if (wrongs > 2) rectEx.time = 0;
                }
                rects[i] = rectEx.smoothedVals;
                rectEx.prevVals = curr;
            }
            corners = new(rects);
            for (int i = 0; i < betterRects.Length; i++) {
                for (int j = 0; j < betterRects[i].Length; j++) {
                    betterRects[i][j].time++;
                }
            }
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
        static Draw.ShapeType[][] typeQueue = new Draw.ShapeType[][] { new Draw.ShapeType[maxQueue], new Draw.ShapeType[maxQueue] };
        public static void DrawDot(Matrix4x4 rot, float scl = 1) {
            DrawShape(Draw.ShapeType.Dot, rot, scl);
        }
        public static void DrawAxisGray(Matrix4x4 rot, float scl = 1) {
            DrawShape(Draw.ShapeType.AxisGray, rot, scl);
        }
        public static void DrawAxis(Matrix4x4 rot, float scl = 1) {
            DrawShape(Draw.ShapeType.Axis, rot, scl);
        }
        public static void DrawCube(Matrix4x4 rot, float scl = 1) {
            DrawShape(Draw.ShapeType.Cube, rot, scl);
        }
        static void DrawShape(Draw.ShapeType shape, Matrix4x4 rot, float scl = 1) {
            for (int i = 0; i < Tag.cameras.Length; i++) {
                Matrix4x4 camMat = Tag.cameras[i].matrix;
                DrawShape(rot, camMat, shape, i, scl);
            }
        }
        public static void DrawShape(Matrix4x4 rot, Matrix4x4 camMat, Draw.ShapeType type, int cam, float scl = 1) {
            Matrix4x4 invCam;
            Matrix4x4.Invert(camMat, out invCam);
            Matrix4x4 mat = Matrix4x4.Multiply(rot, invCam);
            DrawShape(mat, cam, type, scl);
        }
        public static void DrawShape(Matrix4x4 rot, int cam, Draw.ShapeType type, float scl = 1) {
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
            int listIndex = queueCount[cam];
            bool foundSpot = false;
            if (queueCount[cam] == maxQueue) {
                for (int i = 0; i < typeQueue[cam].Length; i++) {
                    if (typeQueue[cam][i] == Draw.ShapeType.AxisGray) {
                        foundSpot = true;
                        listIndex = i;
                        break;
                    }
                }
                if (!foundSpot)
                    return;
            }
            posQueue[cam][listIndex] = tvec2;
            rotQueue[cam][listIndex] = rvec2;
            typeQueue[cam][listIndex] = type;
            if (scl == 0)
                scl = 0.1f;
            sclQueue[cam][listIndex] = scl * 3;
            if (!foundSpot)
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
