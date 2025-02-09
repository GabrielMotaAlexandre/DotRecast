using System;
using System.Numerics;
using System.Runtime.CompilerServices;
using DotRecast.Core;
using UnityEngine;

namespace DotRecast.Detour
{
    public static class DtUtils
    {
        private static readonly float EQUAL_THRESHOLD = RcMath.Sqr(1f / 16384.0f);

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static int NextPow2(int v)
        {
            v--;
            v |= v >> 1;
            v |= v >> 2;
            v |= v >> 4;
            v |= v >> 8;
            v |= v >> 16;
            return v + 1;
        }

        public static int Ilog2(int v)
        {
            int r;
            int shift;
            r = (v > 0xffff ? 1 : 0) << 4;
            v >>= r;
            shift = (v > 0xff ? 1 : 0) << 3;
            v >>= shift;
            r |= shift;
            shift = (v > 0xf ? 1 : 0) << 2;
            v >>= shift;
            r |= shift;
            shift = (v > 0x3 ? 1 : 0) << 1;
            v >>= shift;
            r |= shift;
            r |= v >> 1;
            return r;
        }

        /// Performs a 'sloppy' colocation check of the specified points.
        /// @param[in] p0 A point. [(x, y, z)]
        /// @param[in] p1 A point. [(x, y, z)]
        /// @return True if the points are considered to be at the same location.
        ///
        /// Basically, this function will return true if the specified points are
        /// close enough to eachother to be considered colocated.
        public static bool VEqual(Vector3 p0, Vector3 p1)
        {
            return VEqual(p0, p1, EQUAL_THRESHOLD);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static bool VEqual(Vector3 p0, Vector3 p1, float thresholdSqr)
        {
            float d = Vector3.DistanceSquared(p0, p1);
            return d < thresholdSqr;
        }

        /// Determines if two axis-aligned bounding boxes overlap.
        /// @param[in] amin Minimum bounds of box A. [(x, y, z)]
        /// @param[in] amax Maximum bounds of box A. [(x, y, z)]
        /// @param[in] bmin Minimum bounds of box B. [(x, y, z)]
        /// @param[in] bmax Maximum bounds of box B. [(x, y, z)]
        /// @return True if the two AABB's overlap.
        /// @see dtOverlapBounds
        public static bool OverlapQuantBounds(Vector3 amin, Vector3 amax, in Vector3Int bmin, in Vector3Int bmax)
        {
            bool overlap = ((int)amin.X & 0x7ffffffe) <= bmax.x && ((((int)amax.X) + 1) | 1) >= bmin.x
                && ((int)amin.Y & 0x7ffffffe) <= bmax.y && ((((int)amax.Y) + 1) | 1) >= bmin.y
                && ((int)amin.Z & 0x7ffffffe) <= bmax.z && ((((int)amax.Z) + 1) | 1) >= bmin.z;

            return overlap;
        }

        /// Determines if two axis-aligned bounding boxes overlap.
        /// @param[in] amin Minimum bounds of box A. [(x, y, z)]
        /// @param[in] amax Maximum bounds of box A. [(x, y, z)]
        /// @param[in] bmin Minimum bounds of box B. [(x, y, z)]
        /// @param[in] bmax Maximum bounds of box B. [(x, y, z)]
        /// @return True if the two AABB's overlap.
        /// @see dtOverlapQuantBounds
        public static bool OverlapBounds(Vector3 amin, Vector3 amax, Vector3 bmin, Vector3 bmax)
        {
            bool overlap = amin.X <= bmax.X && amax.X >= bmin.X;
            overlap &= amin.Y <= bmax.Y && amax.Y >= bmin.Y;
            overlap &= amin.Z <= bmax.Z && amax.Z >= bmin.Z;
            return overlap;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static bool OverlapRange(float amin, float amax, float bmin, float bmax, float eps)
        {
            return (amin + eps) <= bmax && (amax - eps) >= bmin;
        }

        /// @par
        ///
        /// All vertices are projected onto the xz-plane, so the y-values are ignored.
        public static bool OverlapPolyPoly2D(in Span<Vector2> polya, int npolya, in Span<Vector2> polyb, int npolyb)
        {
            const float eps = 1e-4f;
            for (int i = 0, j = npolya - 1; i < npolya; j = i++)
            {
                ref readonly var va = ref polyb[i];
                ref readonly var vj = ref polyb[j];

                Vector2 n = new(va.Y - vj.Y, -(va.X - vj.X));

                var aminmax = ProjectPoly(n, polya, npolya);
                var bminmax = ProjectPoly(n, polyb, npolyb);
                if (!OverlapRange(aminmax.X, aminmax.Y, bminmax.X, bminmax.Y, eps))
                {
                    // Found separating axis
                    return false;
                }
            }

            for (int i = 0, j = npolyb - 1; i < npolyb; j = i++)
            {
                ref readonly var va = ref polyb[j];
                ref readonly var vb = ref polyb[i];

                Vector2 n = new(vb.Y - va.Y, -(vb.X - va.X));

                var aminmax = ProjectPoly(n, polya, npolya);
                var bminmax = ProjectPoly(n, polyb, npolyb);
                if (!OverlapRange(aminmax.X, aminmax.Y, bminmax.X, bminmax.Y, eps))
                {
                    // Found separating axis
                    return false;
                }
            }

            return true;
        }


        /// @}
        /// @name Computational geometry helper functions.
        /// @{
        /// Derives the signed xz-plane area of the triangle ABC, or the
        /// relationship of line AB to point C.
        /// @param[in] a Vertex A. [(x, y, z)]
        /// @param[in] b Vertex B. [(x, y, z)]
        /// @param[in] c Vertex C. [(x, y, z)]
        /// @return The signed xz-plane area of the triangle.
        public static float TriArea2D(ReadOnlySpan<float> verts, int a, int b, int c)
        {
            float abx = verts[b] - verts[a];
            float abz = verts[b + 2] - verts[a + 2];
            float acx = verts[c] - verts[a];
            float acz = verts[c + 2] - verts[a + 2];
            return acx * abz - abx * acz;
        }

        /// @}
        /// @name Computational geometry helper functions.
        /// @{
        /// Derives the signed xz-plane area of the triangle ABC, or the
        /// relationship of line AB to point C.
        /// @param[in] a Vertex A. [(x, y, z)]
        /// @param[in] b Vertex B. [(x, y, z)]
        /// @param[in] c Vertex C. [(x, y, z)]
        /// @return The signed xz-plane area of the triangle.
        public static float TriArea2D(ReadOnlySpan<Vector3> verts, int a, int b, int c)
        {
            return TriArea2D(verts[a], verts[b], verts[c]);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static float TriArea2D(in Vector3 a, in Vector3 b, in Vector3 c)
        {
            var abx = b.X - a.X;
            var abz = b.Z - a.Z;
            var acx = c.X - a.X;
            var acz = c.Z - a.Z;
            return acx * abz - abx * acz;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static float TriArea2D(in Vector2 a, in Vector2 b, in Vector2 c)
        {
            var ab = b - a;
            var ac = c - a;
            return ac.X * ab.Y - ab.X * ac.Y;
        }

        // Returns a random point in a convex polygon.
        // Adapted from Graphics Gems article.
        public static Vector3 RandomPointInConvexPoly(ReadOnlySpan<Vector3> pts, int npts, Span<float> areas, float s, float t)
        {
            // Calc triangle araes
            float areasum = 0f;
            var pt0 = pts[0];
            var ptPrevious = pts[1];
            for (int i = 2; i < npts; i++)
            {
                var pt = pts[i];
                areas[i] = TriArea2D(pt0, ptPrevious, pt);
                areasum += MathF.Max(0.001f, areas[i]);
                ptPrevious = pt;
            }

            // Find sub triangle weighted by area.
            float thr = s * areasum;
            float acc = 0f;
            float u = 1f;
            int tri = npts - 1;
            for (int i = 2; i < npts; i++)
            {
                float dacc = areas[i];
                if (thr >= acc && thr < (acc + dacc))
                {
                    u = (thr - acc) / dacc;
                    tri = i;
                    break;
                }

                acc += dacc;
            }

            float v = MathF.Sqrt(t);

            float a = 1 - v;
            float b = (1 - u) * v;
            float c = u * v;
            int pb = tri - 1;
            int pc = tri;

            return a * pt0 + b * pts[pb] + c * pts[pc];
        }

        public static bool ClosestHeightPointTriangle(Vector3 p, Vector3 a, Vector3 b, Vector3 c, out float h)
        {
            const float EPS = 1e-6f;

            h = 0;
            Vector3 v0 = c - a;
            Vector3 v1 = b - a;
            Vector3 v2 = p - a;

            // Compute scaled barycentric coordinates
            float denom = v0.X * v1.Z - v0.Z * v1.X;
            if (Math.Abs(denom) < EPS)
            {
                return false;
            }

            float u = v1.Z * v2.X - v1.X * v2.Z;
            float v = v0.X * v2.Z - v0.Z * v2.X;

            if (denom < 0)
            {
                denom = -denom;
                u = -u;
                v = -v;
            }

            // If point lies inside the triangle, return interpolated ycoord.
            if (u >= 0f && v >= 0f && (u + v) <= denom)
            {
                h = a.Y + (v0.Y * u + v1.Y * v) / denom;
                return true;
            }

            return false;
        }

        public static Vector2 ProjectPoly(in Vector2 axis, in Span<Vector2> poly, int npoly)
        {
            float rmin = float.MaxValue;
            float rmax = float.MinValue;

            for (int i = 0; i < npoly; ++i)
            {
                float d = Vector2.Dot(axis, poly[i]);
                rmin = MathF.Min(rmin, d);
                rmax = MathF.Max(rmax, d);
            }

            return new Vector2(rmin, rmax);
        }

        /// @par
        ///
        /// All points are projected onto the xz-plane, so the y-values are ignored.
        public static bool PointInPolygon(Vector3 pt, ReadOnlySpan<Vector3> verts, int nverts)
        {
            // todo replace with vertices and triangles input parameters instead of only verts

            // TODO: Replace pnpoly with triArea2D tests?
            int i, j;
            bool c = false;
            for (i = 0, j = nverts - 1; i < nverts; j = i++)
            {
                var vi = verts[i];
                var vj = verts[j];

                if (((vi.Z > pt.Z) != (vj.Z > pt.Z)) && (pt.X < (vj.X - vi.X) * (pt.Z - vi.Z) / (vj.Z - vi.Z) + vi.X))
                {
                    c = !c;
                }
            }

            return c;
        }

        public static bool DistancePtPolyEdgesSqr(Vector3 pt, float[] verts, int nverts, float[] ed, float[] et)
        {
            // TODO: Replace pnpoly with triArea2D tests?
            int i, j;
            bool c = false;
            for (i = 0, j = nverts - 1; i < nverts; j = i++)
            {
                int vi = i * 3;
                int vj = j * 3;
                if (((verts[vi + 2] > pt.Z) != (verts[vj + 2] > pt.Z)) &&
                    (pt.X < (verts[vj] - verts[vi]) * (pt.Z - verts[vi + 2]) / (verts[vj + 2] - verts[vi + 2]) + verts[vi]))
                {
                    c = !c;
                }

                ed[j] = DistancePtSegSqr2D(pt, verts, vj, vi, out et[j]);
            }

            return c;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static float DistancePtSegSqr2D(Vector3 pt, float[] verts, int p, int q, out float t)
        {
            var vp = Vector3Extensions.Of(verts, p);
            var vq = Vector3Extensions.Of(verts, q);
            return DistancePtSegSqr2D(pt, vp, vq, out t);
        }

        public static float DistancePtSegSqr2D(Vector3 pt, Vector3 p, Vector3 q, out float t)
        {
            return DistancePtSegSqr2D(pt.AsVector2XZ(), p.AsVector2XZ(), q.AsVector2XZ(), out t);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining | MethodImplOptions.AggressiveOptimization)]
        public static float DistancePtSegSqr2D(Vector2 pt, Vector2 p, Vector2 q, out float t)
        {
            var pq = q - p;
            var dd = pt - p;
            t = Vector2.Dot(pq, dd);

            float d = pq.LengthSquared();
            if (d > 0)
            {
                t /= d;
            }

            t = Math.Clamp(t, 0, 1);

            return (t * pq - dd).LengthSquared();
        }

        public static bool IntersectSegmentPoly2D(Vector3 p0, Vector3 p1,
            Vector3[] verts, int nverts,
            out float tmin, out float tmax,
            out int segMin, out int segMax)
        {
            const float EPS = 0.000001f;

            tmin = 0;
            tmax = 1;
            segMin = -1;
            segMax = -1;

            var dir = p1 - p0;

            var p0v = p0;
            for (int i = 0, j = nverts - 1; i < nverts; j = i++)
            {
                Vector3 vpj = verts[j];
                Vector3 vpi = verts[i];
                var edge = vpi - vpj;
                var diff = p0v - vpj;
                float n = Vector3Extensions.Perp2D(edge, diff);
                float d = Vector3Extensions.Perp2D(dir, edge);
                if (Math.Abs(d) < EPS)
                {
                    // S is nearly parallel to this edge
                    if (n < 0)
                    {
                        return false;
                    }
                    else
                    {
                        continue;
                    }
                }

                float t = n / d;
                if (d < 0)
                {
                    // segment S is entering across this edge
                    if (t > tmin)
                    {
                        tmin = t;
                        segMin = j;
                        // S enters after leaving polygon
                        if (tmin > tmax)
                        {
                            return false;
                        }
                    }
                }
                else
                {
                    // segment S is leaving across this edge
                    if (t < tmax)
                    {
                        tmax = t;
                        segMax = j;
                        // S leaves before entering polygon
                        if (tmax < tmin)
                        {
                            return false;
                        }
                    }
                }
            }

            return true;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static int OppositeTile(int side)
        {
            return (side + 4) & 0x7;
        }


        public static bool IntersectSegSeg2D(Vector3 ap, Vector3 aq, Vector3 bp, Vector3 bq, out float s, out float t)
        {
            s = 0;
            t = 0;

            Vector3 u = aq - ap;
            Vector3 v = bq - bp;
            Vector3 w = ap - bp;
            float d = Vector3Extensions.PerpXZ(u, v);
            if (MathF.Abs(d) < 1e-6f)
            {
                return false;
            }

            s = Vector3Extensions.PerpXZ(v, w) / d;
            t = Vector3Extensions.PerpXZ(u, w) / d;

            return true;
        }
    }
}