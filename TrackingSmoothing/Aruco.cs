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
            boardImage.Save("arucoboard.png");
        }
        //78
        static VideoCapture[] capture;
        static Dictionary ArucoDict;
        static DetectorParameters ArucoParameters;
        public static int markersLength = 81;
        static Mat[] cameraMatrix;
        static Mat[] distortionMatrix;
        public static void Init() {
            capture = new VideoCapture[2];
            capture[0] = new VideoCapture(1);
            capture[1] = new VideoCapture(2);
            capture[0].Set(CapProp.FrameWidth, Tag.cameras[0].width);
            capture[0].Set(CapProp.FrameHeight, Tag.cameras[0].height);
            capture[1].Set(CapProp.FrameWidth, Tag.cameras[1].width);
            capture[1].Set(CapProp.FrameHeight, Tag.cameras[1].height);

            int markersX = 4;
            int markersY = 4;
            int markersSeparation = 30;
            ArucoDict = new Dictionary(16, 4/*Dictionary.PredefinedDictionaryName.Dict4X4_1000*/); // bits x bits (per marker) _ number of markers in dict
            //ArucoDict = new Dictionary(false);
            GridBoard ArucoBoard = null;
            ArucoBoard = new GridBoard(markersX, markersY, markersLength, markersSeparation, ArucoDict);
            PrintArucoBoard(ArucoBoard, markersX, markersY, markersLength, markersSeparation);

            ArucoParameters = new DetectorParameters();
            ArucoParameters = DetectorParameters.GetDefault();
            //ArucoParameters.PolygonalApproxAccuracyRate = 0.1;

            // Calibration done with https://docs.opencv.org/3.4.3/d7/d21/tutorial_interactive_calibration.html
            cameraMatrix = new Mat[] { new Mat(new Size(3, 3), DepthType.Cv32F, 1), new Mat(new Size(3, 3), DepthType.Cv32F, 1) };
            distortionMatrix = new Mat[] { new Mat(1, 8, DepthType.Cv32F, 1), new Mat(1, 8, DepthType.Cv32F, 1) };
            for (int c = 0; c < 2; c++) {
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
        public static void Update() {
            while (true) {
                if (Program.wantToCloseWindows) {
                    CvInvoke.DestroyAllWindows();
                    Program.wantToCloseWindows = false;
                }
                bool shouldShowFrame = Program.wantToShowFrame;
                for (int c = 0; c < 2; c++) {
                    //Capture a frame with webcam
                    Mat frame = new Mat();
                    frame = capture[c].QueryFrame();
                    //frame *= 2f;
                    //var asd = (Byte[,,])frame.GetData();
                    //try {
                    //for (int x = 0; x < frame.Height; x++) {
                    //    for (int y = 0; y < frame.Width; y++) {
                    //        asd[x, y, 0] = (Byte)Math.Min(asd[x, y, 0] * 2, 255);
                    //    }
                    //}
                    //} catch (Exception e) {
                    //    Console.WriteLine();
                    //}

                    if (!frame.IsEmpty) {
                        //Detect markers on last retrieved frame
                        VectorOfInt ids = new VectorOfInt(); // name/id of the detected markers
                        VectorOfVectorOfPointF corners = new VectorOfVectorOfPointF(); // corners of the detected marker
                        VectorOfVectorOfPointF rejected = new VectorOfVectorOfPointF(); // rejected contours
                        ArucoInvoke.DetectMarkers(frame, ArucoDict, corners, ids, ArucoParameters, rejected);

                        // If we detected at least one marker
                        if (ids.Size > 0) {
                            //Draw detected markers
                            if (shouldShowFrame)
                                ArucoInvoke.DrawDetectedMarkers(frame, corners, ids, new MCvScalar(255, 0, 255));

                            //Estimate pose for each marker using camera calibration matrix and distortion coefficents
                            Mat rvecs = new Mat(); // rotation vector
                            Mat tvecs = new Mat(); // translation vector
                            ArucoInvoke.EstimatePoseSingleMarkers(corners, markersLength, cameraMatrix[c], distortionMatrix[c], rvecs, tvecs);

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
                                    if (!Tag.newInfoReady)
                                        Tag.RecieveTrackerAsync(ids[i], c, rot, pos);
                                    pos.Y -= 0.1f;
                                    Matrix4x4 finalMat = Matrix4x4.Multiply(rot, Matrix4x4.CreateTranslation(pos));
                                    //ArucoInvoke.DrawAxis(frame, cameraMatrix[c], distortionMatrix[c], rvec, tvec, markersLength);
                                }

                            }
                        }
                        int queueSize = queueCount[c];
                        queueCount[c] = 0;
                        for (int i = 0; i < queueSize; i++) {
                            if (posQueue[c][i] == null || rotQueue[c][i] == null) continue;
                            if (posQueue[c][i][2] == 0) continue;
                            try {
                                ArucoInvoke.DrawAxis(frame, cameraMatrix[c], distortionMatrix[c], rotQueue[c][i], posQueue[c][i], markersLength * 0.5f * sclQueue[c][i]);
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
                }
                Tag.newInfoReady = true;
            }
        }
        static int[] queueCount = new int[2];
        static int maxQueue = 30;
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
            sclQueue[cam][queueCount[cam]] = scl;
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
