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
using System.Numerics;
using System.Runtime.CompilerServices;

namespace DotRecast.Detour
{
    /**
    * Convex-convex intersection based on "Computational Geometry in C" by Joseph O'Rourke
*/
    public static class ConvexConvexIntersection
    {
        private static readonly float EPSILON = 0.0001f;

        public static ReadOnlySpan<Vector3> Intersect(ReadOnlySpan<Vector3> pVerts, ReadOnlySpan<Vector3> qVerts)
        {
            int pLength = pVerts.Length;
            int qLength = qVerts.Length;
            var inters = new Vector3[Math.Max(qLength, pLength) * 3];
            int ii = 0;
            /* Initialize variables. */

            int aa = 0;
            int ba = 0;
            int ai = 0;
            int bi = 0;

            InFlag f = InFlag.Unknown;
            bool FirstPoint = true;

            do
            {
                var a = pVerts[ai % pLength];
                var b = qVerts[bi % qLength];
                var a1 = pVerts[(ai + pLength - 1) % pLength]; // prev a
                var b1 = qVerts[(bi + qLength - 1) % qLength]; // prev b

                Vector3 aV = a - a1;
                Vector3 bV = b - b1;

                float cross = bV.X * aV.Z - aV.X * bV.Z; // TriArea2D({0, 0}, A, B);
                float aHB = DtUtils.TriArea2D(b1, b, a);
                float bHA = DtUtils.TriArea2D(a1, a, b);
                if (MathF.Abs(cross) < EPSILON)
                {
                    cross = 0f;
                }

                Vector3 iq = default;
                bool parallel = cross is 0f;
                Intersection code = parallel ? ParallelInt(a1, a, b1, b, out var ip, out iq) : SegSegInt(a1, a, b1, b, out ip);

                if (code == Intersection.Single)
                {
                    if (FirstPoint)
                    {
                        FirstPoint = false;
                        aa = ba = 0;
                    }

                    ii = AddVertex(inters, ii, ip);
                    f = InOut(f, aHB, bHA);
                }

                /*-----Advance rules-----*/

                /// Derives the dot product of two vectors on the xz-plane. (@p u . @p v)
                /// @param[in] u A vector [(x, y, z)]
                /// @param[in] v A vector [(x, y, z)]
                /// @return The dot product on the xz-plane.
                ///
                /// The vectors are projected onto the xz-plane, so the y-values are
                /// ignored.
                /* Special case: A & B overlap and oppositely oriented. */
                if (code == Intersection.Overlap && aV.X * bV.X + aV.Z * bV.Z < 0)
                {
                    ii = AddVertex(inters, ii, ip);
                    ii = AddVertex(inters, ii, iq);
                    break;
                }

                /* Special case: A & B parallel and separated. */
                if (parallel && aHB < 0f && bHA < 0f)
                {
                    return default;
                }
                /* Special case: A & B collinear. */
                else if (parallel && MathF.Abs(aHB) < EPSILON && Math.Abs(bHA) < EPSILON)
                {
                    /* Advance but do not output point. */
                    if (f == InFlag.Pin)
                    {
                        ba++;
                        bi++;
                    }
                    else
                    {
                        aa++;
                        ai++;
                    }
                }
                /* Generic cases. */
                else if (cross >= 0)
                {
                    if (bHA > 0)
                    {
                        if (f == InFlag.Pin)
                        {
                            ii = AddVertex(inters, ii, a);
                        }

                        aa++;
                        ai++;
                    }
                    else
                    {
                        if (f == InFlag.Qin)
                        {
                            ii = AddVertex(inters, ii, b);
                        }

                        ba++;
                        bi++;
                    }
                }
                else
                {
                    if (aHB > 0)
                    {
                        if (f == InFlag.Qin)
                        {
                            ii = AddVertex(inters, ii, b);
                        }

                        ba++;
                        bi++;
                    }
                    else
                    {
                        if (f == InFlag.Pin)
                        {
                            ii = AddVertex(inters, ii, a);
                        }

                        aa++;
                        ai++;
                    }
                }
                /* Quit when both adv. indices have cycled, or one has cycled twice. */
            } while ((aa < pLength || ba < qLength) && aa < 2 * pLength && ba < 2 * qLength);

            /* Deal with special cases: not implemented. */
            if (f == InFlag.Unknown)
            {
                return null;
            }

            return inters.AsSpan(0, ii);
        }

        private static int AddVertex(Span<Vector3> inters, int ii, Vector3 p)
        {
            if (ii > 0)
            {
                if (inters[ii - 1] == p)
                {
                    return ii;
                }

                if (inters[0] == p)
                {
                    return ii;
                }
            }

            inters[ii] = p;
            return ii + 1;
        }


        private static InFlag InOut(InFlag inflag, float aHB, float bHA)
        {
            if (aHB > 0)
            {
                return InFlag.Pin;
            }
            else if (bHA > 0)
            {
                return InFlag.Qin;
            }

            return inflag;
        }

        private static Intersection SegSegInt(Vector3 a, Vector3 b, Vector3 c, Vector3 d, out Vector3 p)
        {
            if (DtUtils.IntersectSegSeg2D(a, b, c, d, out var s, out var t))
            {
                if (s >= 0f && s <= 1f && t >= 0f && t <= 1f)
                {
                    p = a + (b - a) * s;
                    return Intersection.Single;
                }
            }
            p = default;
            return Intersection.None;
        }

        private static Intersection ParallelInt(Vector3 a, Vector3 b, Vector3 c, Vector3 d, out Vector3 p, out Vector3 q)
        {
            if (Between2D(a, b, c) && Between2D(a, b, d))
            {
                p = c;
                q = d;
                return Intersection.Overlap;
            }

            if (Between2D(c, d, a) && Between2D(c, d, b))
            {
                p = a;
                q = b;
                return Intersection.Overlap;
            }

            if (Between2D(a, b, c) && Between2D(c, d, b))
            {
                p = c;
                q = b;
                return Intersection.Overlap;
            }

            if (Between2D(a, b, c) && Between2D(c, d, a))
            {
                p = c;
                q = a;
                return Intersection.Overlap;
            }

            if (Between2D(a, b, d) && Between2D(c, d, b))
            {
                p = d;
                q = b;
                return Intersection.Overlap;
            }

            if (Between2D(a, b, d) && Between2D(c, d, a))
            {
                p = d;
                q = a;
                return Intersection.Overlap;
            }

            p = q = default;

            return Intersection.None;
        }

        private static bool Between2D(Vector3 a, Vector3 b, Vector3 c)
        {
            if (MathF.Abs(a.X - b.X) > MathF.Abs(a.Z - b.Z))
            {
                return ((a.X <= c.X) && (c.X <= b.X)) || ((a.X >= c.X) && (c.X >= b.X));
            }
            else
            {
                return ((a.Z <= c.Z) && (c.Z <= b.Z)) || ((a.Z >= c.Z) && (c.Z >= b.Z));
            }
        }
    }
}