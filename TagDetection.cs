using System;
using System.Collections;
using System.Text.RegularExpressions;
using OpenCvSharp;
namespace csharp {
    public class TagDetection {
        public bool good; //汉明距离是否在误差范围内 Whether the hamming distance is within the margin of error.
        public long obsCode; //解析出的code Observed Code
        public long matchCode; //匹配上的code match Code
        public int id; //匹配上的id tag`id 
        public int hammingDistance; // hamming distance between Observed code and matched code
        public int rotation;
        public Point[] points;//four corner
        public Mat homography;

        public TagDetection () {
            this.obsCode = -1;
            this.id = -1;
            this.hammingDistance = -1;
            this.rotation = 1;
            this.good = false;
            this.matchCode = -1;
        }
        /// <summary>
        /// 将Point转化成Point2d
        /// Translate normal point to 2d point
        /// </summary>
        /// <returns>转化后的Point2d 2d point which has been translated
        /// </returns>
        Point2d[] _convertPoint2Point2D () {
            Point2d[] points2d = new Point2d[4];
            for (int i = 0; i < 4; i++) {
                points2d[i] = new Point2d (this.points[i].X, this.points[i].Y);
            }
            return points2d;
        }
        /// <summary>
        /// 得到旋转矩阵，需要使用opencv
        /// </summary>
        void _computeHomographyOpencv () {
            Point2d[] src = {
                new Point2d (-1, -1),
                new Point2d (1, -1),
                new Point2d (1, 1),
                new Point2d (-1, 1)
            };

            Point2d[] dst = _convertPoint2Point2D ();
            Mat retval = Cv2.FindHomography (src, dst);
            this.homography = retval;
        }
        /// <summary>
        /// 得到旋转矩阵，不使用opencv
        /// get homography but not using opencv
        /// </summary>
        void _computeHomography () { }
        public void addPoint (Point[] points) {
            this.points = points;
            _computeHomographyOpencv ();
        }
    }
}