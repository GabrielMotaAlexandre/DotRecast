/*
Copyright (c) 2009-2010 Mikko Mononen memon@inside.org
recast4j copyright (c) 2015-2019 Piotr Piastucki piotr@jtilia.org
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
using DotRecast.Core;
using DotRecast.Detour.Crowd.Tracking;

namespace DotRecast.Detour.Crowd
{


    public class DtObstacleAvoidanceQuery
    {
        public const int DT_MAX_PATTERN_DIVS = 32;

        /// < Max numver of adaptive divs.
        public const int DT_MAX_PATTERN_RINGS = 4;


        private DtObstacleAvoidanceParams m_params;
        private float m_invHorizTime;
        private float m_vmax;
        private float m_invVmax;

        private readonly int m_maxCircles;
        private readonly DtObstacleCircle[] m_circles;
        private int m_ncircles;

        private readonly int m_maxSegments;
        private readonly DtObstacleSegment[] m_segments;
        private int m_nsegments;

        public DtObstacleAvoidanceQuery(int maxCircles, int maxSegments)
        {
            m_maxCircles = maxCircles;
            m_ncircles = 0;
            m_circles = new DtObstacleCircle[m_maxCircles];
            for (int i = 0; i < m_maxCircles; i++)
            {
                m_circles[i] = new DtObstacleCircle();
            }

            m_maxSegments = maxSegments;
            m_nsegments = 0;
            m_segments = new DtObstacleSegment[m_maxSegments];
            for (int i = 0; i < m_maxSegments; i++)
            {
                m_segments[i] = new DtObstacleSegment();
            }
        }

        public void Reset()
        {
            m_ncircles = 0;
            m_nsegments = 0;
        }

        public void AddCircle(Vector3 pos, float rad, Vector2 vel, Vector2 dvel)
        {
            if (m_ncircles >= m_maxCircles)
                return;

            DtObstacleCircle cir = m_circles[m_ncircles++];
            cir.p = pos;
            cir.rad = rad;
            cir.vel = vel;
            cir.dvel = dvel;
        }

        public void AddSegment(Vector3 p, Vector3 q)
        {
            if (m_nsegments >= m_maxSegments)
                return;

            DtObstacleSegment seg = m_segments[m_nsegments++];
            seg.p = p;
            seg.q = q;
        }

        public int GetObstacleCircleCount()
        {
            return m_ncircles;
        }

        public DtObstacleCircle GetObstacleCircle(int i)
        {
            return m_circles[i];
        }

        public int GetObstacleSegmentCount()
        {
            return m_nsegments;
        }

        public DtObstacleSegment GetObstacleSegment(int i)
        {
            return m_segments[i];
        }

        private void Prepare(Vector3 pos, Vector2 dvel)
        {
            // Prepare obstacles
            for (int i = 0; i < m_ncircles; ++i)
            {
                DtObstacleCircle cir = m_circles[i];

                // Side
                Vector3 pa = pos;
                Vector3 pb = cir.p;

                Vector3 orig = new();
                cir.dp = Vector3.Normalize(pb - pa);

                var dv = cir.dvel - dvel;

                float a = DtUtils.TriArea2D(orig, cir.dp, dv);
                if (a < 0.01f)
                {
                    cir.np.X = -cir.dp.Z;
                    cir.np.Z = cir.dp.X;
                }
                else
                {
                    cir.np.X = cir.dp.Z;
                    cir.np.Z = -cir.dp.X;
                }
            }

            for (int i = 0; i < m_nsegments; ++i)
            {
                DtObstacleSegment seg = m_segments[i];

                // Precalc if the agent is really close to the segment.
                float r = 0.01f;

                var distSqr = DtUtils.DistancePtSegSqr2D(pos, seg.p, seg.q, out _);
                seg.touch = distSqr < RcMath.Sqr(r);
            }
        }

        private static bool SweepCircleCircle(Vector3 c0, float r0, Vector2 v, Vector3 c1, float r1, out float tmin, out float tmax)
        {
            const float EPS = 0.0001f;

            tmin = 0;
            tmax = 0;

            var s = (c1 - c0).AsVector2XZ();
            float r = r0 + r1;
            float c = s.LengthSquared() - r * r;
            float a = v.LengthSquared();
            if (a < EPS)
                return false; // not moving

            // Overlap, calc time to exit.
            float b = Vector2.Dot(v, s);
            float d = b * b - a * c;
            if (d < 0.0f)
                return false; // no intersection.

            a = 1.0f / a;
            float rd = (float)Math.Sqrt(d);

            tmin = (b - rd) * a;
            tmax = (b + rd) * a;

            return true;
        }

        private static bool IsectRaySeg(Vector3 ap, Vector3 u, Vector3 bp, Vector3 bq, ref float t)
        {
            Vector3 v = bq - bp;
            Vector3 w = ap - bp;
            float d = Vector3Extensions.Perp2D(u, v);
            if (Math.Abs(d) < 1e-6f)
                return false;

            t = Vector3Extensions.Perp2D(v, w) / d;
            if (t < 0 || t > 1)
                return false;

            float s = Vector3Extensions.Perp2D(u, w) / d;
            if (s < 0 || s > 1)
                return false;

            return true;
        }

        /**
     * Calculate the collision penalty for a given velocity vector
     *
     * @param vcand
     *            sampled velocity
     * @param dvel
     *            desired velocity
     * @param minPenalty
     *            threshold penalty for early out
     */
        private float ProcessSample(Vector2 vcand, float cs, Vector3 pos, float rad, Vector2 vel, Vector2 dvel,
            float minPenalty, DtObstacleAvoidanceDebugData debug)
        {
            // penalty for straying away from the desired and current velocities
            float vpen = m_params.weightDesVel * m_invVmax * Vector2.Distance(vcand, dvel);
            float vcpen = m_params.weightCurVel * m_invVmax * Vector2.Distance(vcand, vel);

            // find the threshold hit time to bail out based on the early out penalty
            // (see how the penalty is calculated below to understand)
            float minPen = minPenalty - vpen - vcpen;
            float tThresold = (m_params.weightToi / minPen - 0.1f) * m_params.horizTime;
            if (tThresold - m_params.horizTime > -float.MinValue)
                return minPenalty; // already too much

            // Find min time of impact and exit amongst all obstacles.
            float tmin = m_params.horizTime;
            float side = 0;
            int nside = 0;

            for (int i = 0; i < m_ncircles; ++i)
            {
                DtObstacleCircle cir = m_circles[i];

                // RVO
                var vab = vcand * 2 - vel - cir.vel;

                // Side
                side += Math.Clamp(Math.Min(Vector2.Dot(cir.dp.AsVector2XZ(), vab) * 0.5f + 0.5f, Vector2.Dot(cir.np.AsVector2XZ(), vab) * 2), 0.0f, 1.0f);
                nside++;

                if (!SweepCircleCircle(pos, rad, vab, cir.p, cir.rad, out var htmin, out var htmax))
                    continue;

                // Handle overlapping obstacles.
                if (htmin < 0.0f && htmax > 0.0f)
                {
                    // Avoid more when overlapped.
                    htmin = -htmin * 0.5f;
                }

                if (htmin >= 0.0f)
                {
                    // The closest obstacle is somewhere ahead of us, keep track of nearest obstacle.
                    if (htmin < tmin)
                    {
                        tmin = htmin;
                        if (tmin < tThresold)
                            return minPenalty;
                    }
                }
            }

            for (int i = 0; i < m_nsegments; ++i)
            {
                DtObstacleSegment seg = m_segments[i];
                float htmin = 0;

                if (seg.touch)
                {
                    // Special case when the agent is very close to the segment.
                    Vector3 sdir = seg.q - seg.p;
                    Vector2 snorm = new(-sdir.Z, sdir.X);
                    // If the velocity is pointing towards the segment, no collision.
                    if (Vector2.Dot(snorm, vcand) < 0.0f)
                        continue;
                    // Else immediate collision.
                    htmin = 0.0f;
                }
                else
                {
                    if (!IsectRaySeg(pos, vcand.AsVector3(), seg.p, seg.q, ref htmin))
                        continue;
                }

                // Avoid less when facing walls.
                htmin *= 2.0f;

                // The closest obstacle is somewhere ahead of us, keep track of nearest obstacle.
                if (htmin < tmin)
                {
                    tmin = htmin;
                    if (tmin < tThresold)
                        return minPenalty;
                }
            }

            // Normalize side bias, to prevent it dominating too much.
            if (nside != 0)
                side /= nside;

            float spen = m_params.weightSide * side;
            float tpen = m_params.weightToi * (1.0f / (0.1f + tmin * m_invHorizTime));

            float penalty = vpen + vcpen + spen + tpen;
            // Store different penalties for debug viewing
            debug?.AddSample(vcand, cs, penalty, vpen, vcpen, spen, tpen);

            return penalty;
        }

        public int SampleVelocityGrid(Vector3 pos, float rad, float vmax, Vector2 vel, Vector2 dvel, out Vector2 nvel,
            DtObstacleAvoidanceParams option, DtObstacleAvoidanceDebugData debug)
        {
            Prepare(pos, dvel);
            m_params = option;
            m_invHorizTime = 1.0f / m_params.horizTime;
            m_vmax = vmax;
            m_invVmax = vmax > 0 ? 1.0f / vmax : float.MaxValue;

            nvel = Vector2.Zero;

            debug?.Reset();

            var cv = dvel * m_params.velBias;
            float cs = vmax * 2 * (1 - m_params.velBias) / (m_params.gridSize - 1);
            float half = (m_params.gridSize - 1) * cs * 0.5f;

            float minPenalty = float.MaxValue;
            int ns = 0;

            for (int y = 0; y < m_params.gridSize; ++y)
            {
                for (int x = 0; x < m_params.gridSize; ++x)
                {
                    Vector2 vcand = new(cv.X + x * cs - half, cv.Y + y * cs - half);
                    if (vcand.LengthSquared() > RcMath.Sqr(vmax + cs / 2))
                        continue;

                    float penalty = ProcessSample(new Vector2(vcand.X, vcand.Y), cs, pos, rad, vel, dvel, minPenalty, debug);
                    ns++;
                    if (penalty < minPenalty)
                    {
                        minPenalty = penalty;
                        nvel = new Vector2(vcand.X, vcand.Y);
                    }
                }
            }

            return ns;
        }

        // vector normalization that ignores the y-component.
        static void DtNormalize2D(float[] v)
        {
            float d = (float)Math.Sqrt(v[0] * v[0] + v[2] * v[2]);
            if (d == 0)
                return;
            d = 1.0f / d;
            v[0] *= d;
            v[2] *= d;
        }

        // vector normalization that ignores the y-component.
        static Vector3 DtRotate2D(float[] v, float ang)
        {
            Vector3 dest = new();
            float c = (float)Math.Cos(ang);
            float s = (float)Math.Sin(ang);
            dest.X = v[0] * c - v[2] * s;
            dest.Z = v[0] * s + v[2] * c;
            dest.Y = v[1];
            return dest;
        }

        static readonly float DT_PI = 3.14159265f;

        public int SampleVelocityAdaptive(Vector3 pos, float rad, float vmax, Vector2 vel, Vector2 dvel, out Vector2 nvel,
            DtObstacleAvoidanceParams option,
            DtObstacleAvoidanceDebugData debug)
        {
            Prepare(pos, dvel);
            m_params = option;
            m_invHorizTime = 1.0f / m_params.horizTime;
            m_vmax = vmax;
            m_invVmax = vmax > 0 ? 1.0f / vmax : float.MaxValue;

            debug?.Reset();

            // Build sampling pattern aligned to desired velocity.
            float[] pat = new float[(DT_MAX_PATTERN_DIVS * DT_MAX_PATTERN_RINGS + 1) * 2];
            int npat = 0;

            int ndivs = m_params.adaptiveDivs;
            int nrings = m_params.adaptiveRings;
            int depth = m_params.adaptiveDepth;

            int nd = Math.Clamp(ndivs, 1, DT_MAX_PATTERN_DIVS);
            int nr = Math.Clamp(nrings, 1, DT_MAX_PATTERN_RINGS);
            float da = 1.0f / nd * DT_PI * 2;
            float ca = (float)Math.Cos(da);
            float sa = (float)Math.Sin(da);

            // desired direction
            float[] ddir = new float[6];
            ddir[0] = dvel.X;
            ddir[1] = 0;
            ddir[2] = dvel.Y;
            DtNormalize2D(ddir);
            Vector3 rotated = DtRotate2D(ddir, da * 0.5f); // rotated by da/2
            ddir[3] = rotated.X;
            ddir[4] = 0;
            ddir[5] = rotated.Z;

            // Always add sample at zero
            pat[npat * 2 + 0] = 0;
            pat[npat * 2 + 1] = 0;
            npat++;

            for (int j = 0; j < nr; ++j)
            {
                float r = (nr - j) / (float)nr;
                pat[npat * 2 + 0] = ddir[j % 2 * 3] * r;
                pat[npat * 2 + 1] = ddir[j % 2 * 3 + 2] * r;
                int last1 = npat * 2;
                int last2 = last1;
                npat++;

                for (int i = 1; i < nd - 1; i += 2)
                {
                    // get next point on the "right" (rotate CW)
                    pat[npat * 2 + 0] = pat[last1] * ca + pat[last1 + 1] * sa;
                    pat[npat * 2 + 1] = -pat[last1] * sa + pat[last1 + 1] * ca;
                    // get next point on the "left" (rotate CCW)
                    pat[npat * 2 + 2] = pat[last2] * ca - pat[last2 + 1] * sa;
                    pat[npat * 2 + 3] = pat[last2] * sa + pat[last2 + 1] * ca;

                    last1 = npat * 2;
                    last2 = last1 + 2;
                    npat += 2;
                }

                if ((nd & 1) == 0)
                {
                    pat[npat * 2 + 2] = pat[last2] * ca - pat[last2 + 1] * sa;
                    pat[npat * 2 + 3] = pat[last2] * sa + pat[last2 + 1] * ca;
                    npat++;
                }
            }

            // Start sampling.
            float cr = vmax * (1.0f - m_params.velBias);
            float cs = cr / 10;

            Vector2 res = dvel * m_params.velBias;
            int ns = 0;
            for (int k = 0; k < depth; ++k)
            {
                float minPenalty = float.MaxValue;
                Vector2 bvel = Vector2.Zero;

                for (int i = 0; i < npat; ++i)
                {
                    var vcand = res + new Vector2(pat[i * 2], pat[i * 2 + 1]) * cr;

                    if (vcand.LengthSquared() > RcMath.Sqr(vmax + 0.001f))
                        continue;

                    float penalty = ProcessSample(vcand, cs, pos, rad, vel, dvel, minPenalty, debug);
                    ns++;
                    if (penalty < minPenalty)
                    {
                        minPenalty = penalty;
                        bvel = vcand;
                    }
                }

                res = bvel;

                cr /= 2;
            }

            nvel = res;

            return ns;
        }
    }
}