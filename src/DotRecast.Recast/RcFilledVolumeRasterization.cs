/*
recast4j copyright (c) 2021 Piotr Piastucki piotr@jtilia.org
DotRecast Copyright (c) 2023 Choi Ikpil ikpil@naver.com

This software is provided 'as-is', without any express or implied
warranty.  In no event will the authors be held liable for any damages
arising from the use of this software.
Permission is granted to anyone to use this software for any purpose,
including commercial applications, and to alter it and redistribute it
freely, subject to the following restrictions:
1. The origin of this software must not be misrepresented; you must not
 claim that you wrote the original software. If you use this software
 in a product, an acknowledgment in the product documentation would be
 appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be
 misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/

using System;
using System.Buffers;
using System.Numerics;
using System.Runtime.CompilerServices;
using DotRecast.Core;
using static DotRecast.Recast.RcConstants;


namespace DotRecast.Recast
{
    public static class RcFilledVolumeRasterization
    {
        private const float EPSILON = 0.00001f;

        public static void RasterizeSphere(in RcHeightfield hf, Vector3 center, float radius, int area, int flagMergeThr)
        {
            float[] bounds =
            {
                center.X - radius, center.Y - radius, center.Z - radius, center.X + radius, center.Y + radius,
                center.Z + radius
            };
            RasterizationFilledShape(hf, bounds, area, flagMergeThr,
                rectangle => IntersectSphere(rectangle, center, radius * radius));
        }

        public static void RasterizeCapsule(in RcHeightfield hf, Vector3 start, Vector3 end, float radius, int area, int flagMergeThr)
        {
            float[] bounds =
            {
                MathF.Min(start.X, end.X) - radius, Math.Min(start.Y, end.Y) - radius,
                MathF.Min(start.Z, end.Z) - radius, Math.Max(start.X, end.X) + radius, Math.Max(start.Y, end.Y) + radius,
                MathF.Max(start.Z, end.Z) + radius
            };
            Vector3 axis = new(end.X - start.X, end.Y - start.Y, end.Z - start.Z);
            RasterizationFilledShape(hf, bounds, area, flagMergeThr,
                rectangle => IntersectCapsule(rectangle, start, end, axis, radius * radius));
        }

        public static void RasterizeCylinder(in RcHeightfield hf, Vector3 start, Vector3 end, float radius, int area, int flagMergeThr)
        {
            float[] bounds =
            {
                MathF.Min(start.X, end.X) - radius, Math.Min(start.Y, end.Y) - radius,
                MathF.Min(start.Z, end.Z) - radius, Math.Max(start.X, end.X) + radius, Math.Max(start.Y, end.Y) + radius,
                MathF.Max(start.Z, end.Z) + radius
            };
            Vector3 axis = new(end.X - start.X, end.Y - start.Y, end.Z - start.Z);
            RasterizationFilledShape(hf, bounds, area, flagMergeThr,
                rectangle => IntersectCylinder(rectangle, start, end, axis, radius * radius));
        }

        [SkipLocalsInit]
        public static void RasterizeBox(in RcHeightfield hf, Vector3 center, Vector3[] halfEdges, int area, int flagMergeThr)
        {
            Span<Vector3> normals = stackalloc Vector3[3 * 3];
            halfEdges.CopyTo(normals);
            Vector3Extensions.Normalize(ref normals[0]);
            Vector3Extensions.Normalize(ref normals[1]);
            Vector3Extensions.Normalize(ref normals[2]);

            float[] vertices = new float[8 * 3];
            Span<float> bounds = stackalloc float[]
            {
                float.PositiveInfinity,
                float.PositiveInfinity,
                float.PositiveInfinity,
                float.NegativeInfinity,
                float.NegativeInfinity,
                float.NegativeInfinity
            };
            for (int i = 0; i < 8; ++i)
            {
                float s0 = (i & 1) != 0 ? 1f : -1f;
                float s1 = (i & 2) != 0 ? 1f : -1f;
                float s2 = (i & 4) != 0 ? 1f : -1f;
                vertices[i * 3] = center.X + s0 * halfEdges[0].X + s1 * halfEdges[1].X + s2 * halfEdges[2].X;
                vertices[i * 3 + 1] = center.Y + s0 * halfEdges[0].Y + s1 * halfEdges[1].Y + s2 * halfEdges[2].Y;
                vertices[i * 3 + 2] = center.Z + s0 * halfEdges[0].Z + s1 * halfEdges[1].Z + s2 * halfEdges[2].Z;
                bounds[0] = MathF.Min(bounds[0], vertices[i * 3]);
                bounds[1] = MathF.Min(bounds[1], vertices[i * 3 + 1]);
                bounds[2] = MathF.Min(bounds[2], vertices[i * 3 + 2]);
                bounds[3] = MathF.Max(bounds[3], vertices[i * 3]);
                bounds[4] = MathF.Max(bounds[4], vertices[i * 3 + 1]);
                bounds[5] = MathF.Max(bounds[5], vertices[i * 3 + 2]);
            }

            float[][] planes = RcArrayUtils.Of<float>(6, 4);
            for (int i = 0; i < 6; i++)
            {
                float m = i < 3 ? -1 : 1;
                int vi = i < 3 ? 0 : 7;
                planes[i][0] = m * normals[i % 3].X;
                planes[i][1] = m * normals[i % 3].Y;
                planes[i][2] = m * normals[i % 3].Z;
                planes[i][3] = vertices[vi * 3] * planes[i][0] + vertices[vi * 3 + 1] * planes[i][1]
                                                               + vertices[vi * 3 + 2] * planes[i][2];
            }

            RasterizationFilledShape(in hf, bounds, area, flagMergeThr, rectangle => IntersectBox(rectangle, vertices, planes));
        }

        public static void RasterizeConvex(in RcHeightfield hf, float[] vertices, int[] triangles, int area, int flagMergeThr)
        {
            Span<float> bounds = stackalloc float[] { vertices[0], vertices[1], vertices[2], vertices[0], vertices[1], vertices[2] };

            for (int i = 0, end = vertices.Length / 3; i < end; i++)
            {
                ref var b1 = ref bounds.UnsafeAs<float, Vector3>();
                ref var b2 = ref bounds.UnsafeAs<float, Vector3>();
                ref readonly var vertice = ref vertices.UnsafeAs<float, Vector3>(i);
                b1 = Vector3.Min(b1, vertice);
                b2 = Vector3.Min(b2, vertice);
            }

            var planes = RcArrayUtils.Of<float>(triangles.Length, 4);
            var triBounds = RcArrayUtils.Of<float>(triangles.Length / 3, 4);
            for (int i = 0, j = 0; i < triangles.Length; i += 3, j++)
            {
                var verticeA = vertices.UnsafeAs<float, Vector3>(triangles[i]);
                var verticeB = vertices.UnsafeAs<float, Vector3>(triangles[i + 1]);
                var verticeC = vertices.UnsafeAs<float, Vector3>(triangles[i + 2]);

                Plane(planes, i, verticeB - verticeA, verticeC - verticeA, verticeA);
                Plane(planes, i + 1, planes[i].UnsafeAs<float, Vector3>(), verticeC - verticeB, verticeB);
                Plane(planes, i + 2, planes[i].UnsafeAs<float, Vector3>(), verticeA - verticeC, verticeC);

                var plane1 = planes[i + 1];
                var s = 1f / (Vector3.Dot(verticeA, plane1.UnsafeAs<float, Vector3>()) - plane1[3]);
                plane1.UnsafeAs<float, Vector4>() *= s;

                var plane2 = planes[i + 2];
                s = 1f / (Vector3.Dot(verticeB, plane2.UnsafeAs<float, Vector3>()) - plane2[3]);
                plane2.UnsafeAs<float, Vector4>() *= s;

                triBounds[j][0] = MathF.Min(MathF.Min(verticeA.X, verticeB.X), verticeC.X);
                triBounds[j][1] = MathF.Min(MathF.Min(verticeA.Z, verticeB.Z), verticeC.Z);
                triBounds[j][2] = MathF.Max(MathF.Max(verticeA.X, verticeB.X), verticeC.X);
                triBounds[j][3] = MathF.Max(MathF.Max(verticeA.Z, verticeB.Z), verticeC.Z);
            }

            RasterizationFilledShape(in hf, bounds, area, flagMergeThr,
                rectangle => IntersectConvex(rectangle, triangles, vertices, planes, triBounds));
        }

        private static void Plane(float[][] planes, int p, Vector3 v1, Vector3 v2, Vector3 vertices)
        {
            var plane = planes[p].UnsafeAs<float, Vector3>() = Vector3.Cross(v1, v2);
            planes[p][3] = Vector3.Dot(plane, vertices);
        }

        [SkipLocalsInit]
        private static void RasterizationFilledShape(in RcHeightfield hf, Span<float> bounds, int area, int flagMergeThr,
            Func<float[], Vector2?> intersection)
        {
            if (!OverlapBounds(hf.bmin, hf.bmax, bounds))
            {
                return;
            }

            bounds[0] = MathF.Max(bounds[0], hf.bmin.X);
            bounds[2] = MathF.Max(bounds[2], hf.bmin.Z);
            bounds[3] = MathF.Min(bounds[3], hf.bmax.X);
            bounds[5] = MathF.Min(bounds[5], hf.bmax.Z);

            if (bounds[3] <= bounds[0] || bounds[4] <= bounds[1] || bounds[5] <= bounds[2])
            {
                return;
            }

            float ics = 1f / hf.cs;
            float ich = 1f / hf.ch;
            int xMin = (int)((bounds[0] - hf.bmin.X) * ics);
            int zMin = (int)((bounds[2] - hf.bmin.Z) * ics);
            int xMax = Math.Min(hf.width - 1, (int)((bounds[3] - hf.bmin.X) * ics));
            int zMax = Math.Min(hf.height - 1, (int)((bounds[5] - hf.bmin.Z) * ics));

            var rectangle = new float[5];
            rectangle[4] = hf.bmin.Y;
            for (int x = xMin; x <= xMax; x++)
            {
                for (int z = zMin; z <= zMax; z++)
                {
                    rectangle[0] = x * hf.cs + hf.bmin.X;
                    rectangle[1] = z * hf.cs + hf.bmin.Z;
                    rectangle[2] = rectangle[0] + hf.cs;
                    rectangle[3] = rectangle[1] + hf.cs;
                    var h = intersection(rectangle);
                    if (h != null)
                    {
                        int smin = (int)MathF.Floor((h.Value.X - hf.bmin.Y) * ich);
                        int smax = (int)MathF.Ceiling((h.Value.Y - hf.bmin.Y) * ich);
                        if (smin != smax)
                        {
                            int ismin = Math.Clamp(smin, 0, SPAN_MAX_HEIGHT);
                            int ismax = Math.Clamp(smax, ismin + 1, SPAN_MAX_HEIGHT);
                            RcRasterizations.AddSpan(hf, x, z, new(ismin, ismax, area), flagMergeThr);
                        }
                    }
                }
            }
        }

        private static Vector2? IntersectSphere(float[] rectangle, Vector3 center, float radiusSqr)
        {
            float x = Math.Max(rectangle[0], Math.Min(center.X, rectangle[2]));
            float y = rectangle[4];
            float z = Math.Max(rectangle[1], Math.Min(center.Z, rectangle[3]));

            float mx = x - center.X;
            float my = y - center.Y;
            float mz = z - center.Z;

            float b = my; // Dot(m, d) d = (0, 1, 0)
            float c = LenSqr(mx, my, mz) - radiusSqr;
            if (c > 0f && b > 0f)
            {
                return null;
            }

            float discr = b * b - c;
            if (discr < 0f)
            {
                return null;
            }

            float discrSqrt = MathF.Sqrt(discr);
            float tmin = -b - discrSqrt;
            float tmax = -b + discrSqrt;

            if (tmin < 0f)
            {
                tmin = 0f;
            }

            return new Vector2(y + tmin, y + tmax);
        }

        private static Vector2 IntersectCapsule(float[] rectangle, Vector3 start, Vector3 end, Vector3 axis, float radiusSqr)
        {
            var s = MergeIntersections(IntersectSphere(rectangle, start, radiusSqr), IntersectSphere(rectangle, end, radiusSqr));
            float axisLen2dSqr = axis.X * axis.X + axis.Z * axis.Z;
            if (axisLen2dSqr > EPSILON)
            {
                s = SlabsCylinderIntersection(rectangle, start, end, axis, radiusSqr, s);
            }

            return s;
        }

        private static Vector2 IntersectCylinder(float[] rectangle, Vector3 start, Vector3 end, Vector3 axis, float radiusSqr)
        {
            var s = MergeIntersections(
                RayCylinderIntersection(new Vector3(
                    Math.Clamp(start.X, rectangle[0], rectangle[2]), rectangle[4],
                    Math.Clamp(start.Z, rectangle[1], rectangle[3])
                ), start, axis, radiusSqr),
                RayCylinderIntersection(new Vector3(
                    Math.Clamp(end.X, rectangle[0], rectangle[2]), rectangle[4],
                    Math.Clamp(end.Z, rectangle[1], rectangle[3])
                ), start, axis, radiusSqr));
            float axisLen2dSqr = axis.X * axis.X + axis.Z * axis.Z;
            if (axisLen2dSqr > EPSILON)
            {
                s = SlabsCylinderIntersection(rectangle, start, end, axis, radiusSqr, s);
            }

            if (axis.Y * axis.Y > EPSILON)
            {
                Vector3[] rectangleOnStartPlane = new Vector3[4];
                Vector3[] rectangleOnEndPlane = new Vector3[4];
                float ds = Vector3.Dot(axis, start);
                float de = Vector3.Dot(axis, end);
                for (int i = 0; i < 4; i++)
                {
                    float x = rectangle[(i + 1) & 2];
                    float z = rectangle[(i & 2) + 1];
                    Vector3 a = new(x, rectangle[4], z);
                    float dotAxisA = Vector3.Dot(axis, a);
                    float t = (ds - dotAxisA) / axis.Y;
                    rectangleOnStartPlane[i].X = x;
                    rectangleOnStartPlane[i].Y = rectangle[4] + t;
                    rectangleOnStartPlane[i].Z = z;
                    t = (de - dotAxisA) / axis.Y;
                    rectangleOnEndPlane[i].X = x;
                    rectangleOnEndPlane[i].Y = rectangle[4] + t;
                    rectangleOnEndPlane[i].Z = z;
                }

                for (int i = 0; i < 4; i++)
                {
                    s = CylinderCapIntersection(start, radiusSqr, s, i, rectangleOnStartPlane);
                    s = CylinderCapIntersection(end, radiusSqr, s, i, rectangleOnEndPlane);
                }
            }

            return s;
        }

        private static Vector2 CylinderCapIntersection(Vector3 start, float radiusSqr, Vector2 s, int i, Vector3[] rectangleOnPlane)
        {
            int j = (i + 1) % 4;
            // Ray against sphere intersection
            var m = rectangleOnPlane[i] - start;
            var d = rectangleOnPlane[j] - rectangleOnPlane[i];
            float dl = d.LengthSquared();
            float b = Vector3.Dot(m, d) / dl;
            float c = (m.LengthSquared() - radiusSqr) / dl;
            float discr = b * b - c;
            if (discr > EPSILON)
            {
                float discrSqrt = MathF.Sqrt(discr);
                float t1 = -b - discrSqrt;
                float t2 = -b + discrSqrt;
                if (t1 <= 1 && t2 >= 0)
                {
                    t1 = MathF.Max(0, t1);
                    t2 = MathF.Min(1, t2);
                    float y1 = rectangleOnPlane[i].Y + t1 * d.Y;
                    float y2 = rectangleOnPlane[i].Y + t2 * d.Y;
                    s = MergeIntersections(s, new Vector2(Math.Min(y1, y2), Math.Max(y1, y2)));
                }
            }

            return s;
        }

        private static Vector2 SlabsCylinderIntersection(float[] rectangle, Vector3 start, Vector3 end, Vector3 axis, float radiusSqr, Vector2 s)
        {
            if (MathF.Min(start.X, end.X) < rectangle[0])
            {
                s = MergeIntersections(s, XSlabCylinderIntersection(rectangle, start, axis, radiusSqr, rectangle[0]));
            }

            if (MathF.Max(start.X, end.X) > rectangle[2])
            {
                s = MergeIntersections(s, XSlabCylinderIntersection(rectangle, start, axis, radiusSqr, rectangle[2]));
            }

            if (MathF.Min(start.Z, end.Z) < rectangle[1])
            {
                s = MergeIntersections(s, ZSlabCylinderIntersection(rectangle, start, axis, radiusSqr, rectangle[1]));
            }

            if (MathF.Max(start.Z, end.Z) > rectangle[3])
            {
                s = MergeIntersections(s, ZSlabCylinderIntersection(rectangle, start, axis, radiusSqr, rectangle[3]));
            }

            return s;
        }

        private static Vector2? XSlabCylinderIntersection(float[] rectangle, Vector3 start, Vector3 axis, float radiusSqr, float x)
        {
            return RayCylinderIntersection(XSlabRayIntersection(rectangle, start, axis, x), start, axis, radiusSqr);
        }

        private static Vector3 XSlabRayIntersection(float[] rectangle, Vector3 start, Vector3 direction, float x)
        {
            // 2d intersection of plane and segment
            float t = (x - start.X) / direction.X;
            float z = Math.Clamp(start.Z + t * direction.Z, rectangle[1], rectangle[3]);
            return new Vector3(x, rectangle[4], z);
        }

        private static Vector2? ZSlabCylinderIntersection(float[] rectangle, Vector3 start, Vector3 axis, float radiusSqr, float z)
        {
            return RayCylinderIntersection(ZSlabRayIntersection(rectangle, start, axis, z), start, axis, radiusSqr);
        }

        private static Vector3 ZSlabRayIntersection(float[] rectangle, Vector3 start, Vector3 direction, float z)
        {
            // 2d intersection of plane and segment
            float t = (z - start.Z) / direction.Z;
            float x = Math.Clamp(start.X + t * direction.X, rectangle[0], rectangle[2]);
            return new Vector3(x, rectangle[4], z);
        }

        // Based on Christer Ericsons's "Real-Time Collision Detection"
        private static Vector2? RayCylinderIntersection(Vector3 point, Vector3 start, Vector3 axis, float radiusSqr)
        {
            Vector3 d = axis;
            Vector3 m = new(point.X - start.X, point.Y - start.Y, point.Z - start.Z);
            // float[] n = { 0, 1, 0 };
            float md = Vector3.Dot(m, d);
            // float nd = Dot(n, d);
            float nd = axis.Y;
            float dd = Vector3.Dot(d, d);

            // float nn = Dot(n, n);
            float nn = 1;
            // float mn = Dot(m, n);
            float mn = m.Y;
            // float a = dd * nn - nd * nd;
            float a = dd - nd * nd;
            float k = Vector3.Dot(m, m) - radiusSqr;
            float c = dd * k - md * md;
            if (Math.Abs(a) < EPSILON)
            {
                // Segment runs parallel to cylinder axis
                if (c > 0f)
                {
                    return null; // ’a’ and thus the segment lie outside cylinder
                }

                // Now known that segment intersects cylinder; figure out how it intersects
                float tt1 = -mn / nn; // Intersect segment against ’p’ endcap
                float tt2 = (nd - mn) / nn; // Intersect segment against ’q’ endcap
                return new Vector2(point.Y + MathF.Min(tt1, tt2), point.Y + MathF.Max(tt1, tt2));
            }

            float b = dd * mn - nd * md;
            float discr = b * b - a * c;
            if (discr < 0f)
            {
                return null; // No real roots; no intersection
            }

            float discSqrt = MathF.Sqrt(discr);
            float t1 = (-b - discSqrt) / a;
            float t2 = (-b + discSqrt) / a;

            if (md + t1 * nd < 0f)
            {
                // Intersection outside cylinder on ’p’ side
                t1 = -md / nd;
                if (k + t1 * (2 * mn + t1 * nn) > 0f)
                {
                    return null;
                }
            }
            else if (md + t1 * nd > dd)
            {
                // Intersection outside cylinder on ’q’ side
                t1 = (dd - md) / nd;
                if (k + dd - 2 * md + t1 * (2 * (mn - nd) + t1 * nn) > 0f)
                {
                    return null;
                }
            }

            if (md + t2 * nd < 0f)
            {
                // Intersection outside cylinder on ’p’ side
                t2 = -md / nd;
                if (k + t2 * (2 * mn + t2 * nn) > 0f)
                {
                    return null;
                }
            }
            else if (md + t2 * nd > dd)
            {
                // Intersection outside cylinder on ’q’ side
                t2 = (dd - md) / nd;
                if (k + dd - 2 * md + t2 * (2 * (mn - nd) + t2 * nn) > 0f)
                {
                    return null;
                }
            }

            return new Vector2(point.Y + MathF.Min(t1, t2), point.Y + MathF.Max(t1, t2));
        }

        private static Vector2? IntersectBox(float[] rectangle, float[] vertices, float[][] planes)
        {
            float yMin = float.PositiveInfinity;
            float yMax = float.NegativeInfinity;
            // check intersection with rays starting in box vertices first
            for (int i = 0; i < 8; i++)
            {
                int vi = i * 3;
                if (vertices[vi] >= rectangle[0] && vertices[vi] < rectangle[2] && vertices[vi + 2] >= rectangle[1]
                    && vertices[vi + 2] < rectangle[3])
                {
                    yMin = MathF.Min(yMin, vertices[vi + 1]);
                    yMax = MathF.Max(yMax, vertices[vi + 1]);
                }
            }

            // check intersection with rays starting in rectangle vertices
            var point = new Vector3(0, rectangle[1], 0);
            for (int i = 0; i < 4; i++)
            {
                point.X = ((i & 1) is 0) ? rectangle[0] : rectangle[2];
                point.Z = ((i & 2) is 0) ? rectangle[1] : rectangle[3];
                for (int j = 0; j < 6; j++)
                {
                    if (MathF.Abs(planes[j][1]) > EPSILON)
                    {
                        float dotNormalPoint = Vector3.Dot(planes[j].UnsafeAs<float, Vector3>(), point);
                        float t = (planes[j][3] - dotNormalPoint) / planes[j][1];
                        float y = point.Y + t;
                        bool valid = true;
                        for (int k = 0; k < 6; k++)
                        {
                            if (k != j)
                            {
                                if (point.X * planes[k][0] + y * planes[k][1] + point.Z * planes[k][2] > planes[k][3])
                                {
                                    valid = false;
                                    break;
                                }
                            }
                        }

                        if (valid)
                        {
                            yMin = MathF.Min(yMin, y);
                            yMax = MathF.Max(yMax, y);
                        }
                    }
                }
            }

            ReadOnlySpan<int> BOX_EDGES = stackalloc int[] { 0, 1, 0, 2, 0, 4, 1, 3, 1, 5, 2, 3, 2, 6, 3, 7, 4, 5, 4, 6, 5, 7, 6, 7 };

            // check intersection with box edges
            for (int i = 0; i < BOX_EDGES.Length; i += 2)
            {
                int vi = BOX_EDGES[i] * 3;
                int vj = BOX_EDGES[i + 1] * 3;
                float x = vertices[vi];
                float z = vertices[vi + 2];
                // edge slab intersection
                float y = vertices[vi + 1];
                float dx = vertices[vj] - x;
                float dy = vertices[vj + 1] - y;
                float dz = vertices[vj + 2] - z;
                if (MathF.Abs(dx) > EPSILON)
                {
                    if (XSlabSegmentIntersection(rectangle, x, y, z, dx, dy, dz, rectangle[0], out var iy))
                    {
                        yMin = MathF.Min(yMin, iy);
                        yMax = MathF.Max(yMax, iy);
                    }

                    if (XSlabSegmentIntersection(rectangle, x, y, z, dx, dy, dz, rectangle[2], out iy))
                    {
                        yMin = MathF.Min(yMin, iy);
                        yMax = MathF.Max(yMax, iy);
                    }
                }

                if (MathF.Abs(dz) > EPSILON)
                {
                    if (ZSlabSegmentIntersection(rectangle, x, y, z, dx, dy, dz, rectangle[1], out var iy))
                    {
                        yMin = MathF.Min(yMin, iy);
                        yMax = MathF.Max(yMax, iy);
                    }

                    if (ZSlabSegmentIntersection(rectangle, x, y, z, dx, dy, dz, rectangle[3], out iy))
                    {
                        yMin = MathF.Min(yMin, iy);
                        yMax = MathF.Max(yMax, iy);
                    }
                }
            }

            return yMin <= yMax ? new Vector2(yMin, yMax) : null;
        }

        private static Vector2? IntersectConvex(ReadOnlySpan<float> rectangle, ReadOnlySpan<int> triangles, ReadOnlySpan<float> verts, float[][] planes,
            float[][] triBounds)
        {
            float imin = float.PositiveInfinity;
            float imax = float.NegativeInfinity;
            for (int tr = 0, tri = 0; tri < triangles.Length; tr++, tri += 3)
            {
                if (triBounds[tr][0] > rectangle[2] || triBounds[tr][2] < rectangle[0] || triBounds[tr][1] > rectangle[3]
                    || triBounds[tr][3] < rectangle[1])
                {
                    continue;
                }

                if (Math.Abs(planes[tri][1]) < EPSILON)
                {
                    continue;
                }

                for (int i = 0; i < 3; i++)
                {
                    int vi = triangles[tri + i] * 3;
                    int vj = triangles[tri + (i + 1) % 3] * 3;
                    float x = verts[vi];
                    float z = verts[vi + 2];
                    // triangle vertex
                    if (x >= rectangle[0] && x <= rectangle[2] && z >= rectangle[1] && z <= rectangle[3])
                    {
                        imin = MathF.Min(imin, verts[vi + 1]);
                        imax = MathF.Max(imax, verts[vi + 1]);
                    }

                    // triangle slab intersection
                    float y = verts[vi + 1];
                    float dx = verts[vj] - x;
                    float dy = verts[vj + 1] - y;
                    float dz = verts[vj + 2] - z;
                    if (MathF.Abs(dx) > EPSILON)
                    {
                        if (XSlabSegmentIntersection(rectangle, x, y, z, dx, dy, dz, rectangle[0], out var iy))
                        {
                            imin = MathF.Min(imin, iy);
                            imax = MathF.Max(imax, iy);
                        }

                        if (XSlabSegmentIntersection(rectangle, x, y, z, dx, dy, dz, rectangle[2], out iy))
                        {
                            imin = MathF.Min(imin, iy);
                            imax = MathF.Max(imax, iy);
                        }
                    }

                    if (MathF.Abs(dz) > EPSILON)
                    {
                        if (ZSlabSegmentIntersection(rectangle, x, y, z, dx, dy, dz, rectangle[1], out var iy))
                        {
                            imin = MathF.Min(imin, iy);
                            imax = MathF.Max(imax, iy);
                        }

                        if (ZSlabSegmentIntersection(rectangle, x, y, z, dx, dy, dz, rectangle[3], out iy))
                        {
                            imin = MathF.Min(imin, iy);
                            imax = MathF.Max(imax, iy);
                        }
                    }
                }

                // rectangle vertex
                var point = new Vector3(0, rectangle[1], 0);
                for (int i = 0; i < 4; i++)
                {
                    point.X = ((i & 1) is 0) ? rectangle[0] : rectangle[2];
                    point.Z = ((i & 2) is 0) ? rectangle[1] : rectangle[3];
                    if (RayTriangleIntersection(point, tri, planes, out var y))
                    {
                        imin = MathF.Min(imin, y);
                        imax = MathF.Max(imax, y);
                    }
                }
            }

            if (imin < imax)
            {
                return new Vector2(imin, imax);
            }

            return null;
        }

        private static bool XSlabSegmentIntersection(ReadOnlySpan<float> rectangle, float x, float y, float z, float dx, float dy, float dz, float slabX, out float iy)
        {
            float x2 = x + dx;
            if ((x < slabX && x2 > slabX) || (x > slabX && x2 < slabX))
            {
                float t = (slabX - x) / dx;
                float iz = z + dz * t;
                if (iz >= rectangle[1] && iz <= rectangle[3])
                {
                    iy = y + dy * t;
                    return true;
                }
            }

            iy = 0f;
            return false;
        }

        private static bool ZSlabSegmentIntersection(ReadOnlySpan<float> rectangle, float x, float y, float z, float dx, float dy, float dz, float slabZ, out float iy)
        {
            float z2 = z + dz;
            if ((z < slabZ && z2 > slabZ) || (z > slabZ && z2 < slabZ))
            {
                float t = (slabZ - z) / dz;
                float ix = x + dx * t;
                if (ix >= rectangle[0] && ix <= rectangle[2])
                {
                    iy = y + dy * t;
                    return true;
                }
            }

            iy = 0f;
            return false;
        }

        private static bool RayTriangleIntersection(Vector3 point, int plane, float[][] planes, out float y)
        {
            y = 0;
            float t = (planes[plane][3] - Vector3.Dot(planes[plane].UnsafeAs<float, Vector3>(), point)) / planes[plane][1];
            point.Y += t;

            float u = Vector3.Dot(point, planes[plane + 1].UnsafeAs<float, Vector3>()) - planes[plane + 1][3];
            if (u < 0f || u > 1f)
            {
                return false;
            }

            float v = Vector3.Dot(point, planes[plane + 2].UnsafeAs<float, Vector3>()) - planes[plane + 2][3];
            if (v < 0f)
            {
                return false;
            }

            float w = 1f - u - v;
            if (w < 0f)
            {
                return false;
            }

            y = point.Y;
            return true;
        }

        private static Vector2 MergeIntersections(Vector2? s1, Vector2? s2)
        {
            if (s1 is null)
            {
                return s2.Value;
            }

            if (s2 is null)
            {
                return s1.Value;
            }

            return new Vector2(MathF.Min(s1.Value.X, s2.Value.X), MathF.Max(s1.Value.Y, s2.Value.Y));
        }

        private static float LenSqr(float dx, float dy, float dz)
        {
            return dx * dx + dy * dy + dz * dz;
        }

        private static bool OverlapBounds(Vector3 amin, Vector3 amax, ReadOnlySpan<float> bounds)
        {
            bool overlap = amin.X <= bounds[3] && amax.X >= bounds[0];
            overlap = amin.Y <= bounds[4] && overlap;
            overlap = amin.Z <= bounds[5] && amax.Z >= bounds[2] && overlap;
            return overlap;
        }
    }
}