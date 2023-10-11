using System;
using System.Collections.Generic;
using System.Numerics;
using DotRecast.Core;
using DotRecast.Recast.Geom;

namespace DotRecast.Recast.Toolset.Tools
{
    public class RcConvexVolumeTool : IRcToolable
    {
        private readonly List<Vector3> _pts;
        private readonly List<int> _hull;

        public RcConvexVolumeTool()
        {
            _pts = new List<Vector3>();
            _hull = new List<int>();
        }

        public string GetName()
        {
            return "Convex Volumes";
        }

        public List<Vector3> GetShapePoint()
        {
            return _pts;
        }

        public List<int> GetShapeHull()
        {
            return _hull;
        }

        public void ClearShape()
        {
            _pts.Clear();
            _hull.Clear();
        }

        public bool PlottingShape(Vector3 p, out List<Vector3> pts, out List<int> hull)
        {
            pts = null;
            hull = null;

            // Create
            // If clicked on that last pt, create the shape.
            if (_pts.Count > 0 && Vector3.DistanceSquared(p, _pts[^1]) < 0.2f * 0.2f)
            {
                pts = new List<Vector3>(_pts);
                hull = new List<int>(_hull);

                _pts.Clear();
                _hull.Clear();

                return true;
            }

            // Add new point
            _pts.Add(p);

            // Update hull.
            if (_pts.Count > 3)
            {
                _hull.Clear();
                _hull.AddRange(RcConvexUtils.Convexhull(_pts));
            }
            else
            {
                _hull.Clear();
            }

            return false;
        }


        public static RcConvexVolume RemoveByPos(IInputGeomProvider geom, Vector3 pos)
        {
            // Delete
            int nearestIndex = -1;
            IList<RcConvexVolume> vols = geom.ConvexVolumes();
            for (int i = 0; i < vols.Count; ++i)
            {
                if (RcAreas.PointInPoly(vols[i].verts, pos) && pos.Y >= vols[i].hmin
                                                              && pos.Y <= vols[i].hmax)
                {
                    nearestIndex = i;
                }
            }

            // If end point close enough, delete it.
            if (nearestIndex == -1)
                return default;

            var removal = geom.ConvexVolumes()[nearestIndex];
            geom.ConvexVolumes().RemoveAt(nearestIndex);
            return removal;
        }

        public static void Add(IInputGeomProvider geom, RcConvexVolume volume)
        {
            geom.AddConvexVolume(volume);
        }

        public static RcConvexVolume CreateConvexVolume(List<Vector3> pts, List<int> hull, RcAreaModification areaType, float boxDescent, float boxHeight, float polyOffset)
        {
            // 
            if (hull.Count <= 2)
            {
                return default;
            }

            // Create shape.
            float[] verts = new float[hull.Count * 3];
            for (int i = 0; i < hull.Count; ++i)
            {
                verts[i * 3] = pts[hull[i]].X;
                verts[i * 3 + 1] = pts[hull[i]].Y;
                verts[i * 3 + 2] = pts[hull[i]].Z;
            }

            float minh = float.MaxValue;
            for (int i = 0; i < hull.Count; ++i)
            {
                minh = Math.Min(minh, verts[i * 3 + 1]);
            }

            minh -= boxDescent;
            float maxh = minh + boxHeight;
            if (polyOffset > 0.01f)
            {
                float[] offset = new float[verts.Length * 2];
                int noffset = RcAreas.OffsetPoly(verts, hull.Count, polyOffset, offset, offset.Length);
                if (noffset > 0)
                {
                    verts = RcArrayUtils.CopyOf(offset, 0, noffset * 3);
                }
            }

            return new RcConvexVolume(verts, minh, maxh, areaType);
        }
    }
}