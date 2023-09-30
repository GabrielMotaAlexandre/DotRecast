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
using System.Buffers;
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

        public void AddCircle(Vector2 pos, float rad, Vector2 vel, Vector2 dvel)
        {
            if (m_ncircles >= m_maxCircles)
                return;

            DtObstacleCircle cir = m_circles[m_ncircles++];
            cir.p = pos;
            cir.rad = rad;
            cir.vel = vel;
            cir.dvel = dvel;
        }

        public void AddSegment(Vector2 p, Vector2 q)
        {
            if (m_nsegments >= m_maxSegments)
                return;

            ref var segment = ref m_segments[m_nsegments++];
            segment.p = p;
            segment.q = q;
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

        private void Prepare(Vector2 pos, Vector2 dvel)
        {
            // Prepare obstacles
            for (int i = 0; i < m_ncircles; ++i)
            {
                DtObstacleCircle cir = m_circles[i];

                // Side
                cir.dp = Vector2.Normalize(cir.p - pos);

                var dv = cir.dvel - dvel;

                float a = dv.X * cir.dp.Y - cir.dp.X * dv.Y;

                if (a < 0.01f)
                {
                    cir.np.X = -cir.dp.Y;
                    cir.np.Y = cir.dp.X;
                }
                else
                {
                    cir.np.X = cir.dp.Y;
                    cir.np.Y = -cir.dp.X;
                }
            }

            for (int i = 0; i < m_nsegments; ++i)
            {
                DtObstacleSegment seg = m_segments[i];

                // Precalc if the agent is really close to the segment.

                var distSqr = DtUtils.DistancePtSegSqr2D(pos, seg.p, seg.q, out _);

                const float r = 0.01f;
                seg.touch = distSqr < r * r;
            }
        }

        private static bool SweepCircleCircle(Vector2 c0, float r0, Vector2 v, Vector2 c1, float r1, out float tmin, out float tmax)
        {
            const float EPS = 0.0001f;

            tmin = 0;
            tmax = 0;

            float a = v.LengthSquared();
            if (a < EPS)
                return false; // not moving

            var r = r0 + r1;
            var s = c1 - c0;
            var c = s.LengthSquared() - r * r;

            // Overlap, calc time to exit.
            var b = Vector2.Dot(v, s);
            var d = b * b - a * c;
            if (d < 0)
                return false; // no intersection.

            float rd = (float)MathF.Sqrt(d);

            tmin = (b - rd) / a;
            tmax = (b + rd) / a;

            return true;
        }

        private static bool IsectRaySeg(Vector2 ap, Vector2 u, Vector2 bp, Vector2 bq, ref float t)
        {
            var v = bq - bp;
            var w = ap - bp;
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
        private float ProcessSample(Vector2 vcand, float cs, Vector2 pos, float rad, Vector2 vel, Vector2 dvel,
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
            var tmin = m_params.horizTime;
            float side = 0;
            var nside = 0;

            for (int i = 0; i != m_ncircles; i++)
            {
                ref readonly DtObstacleCircle cir = ref m_circles.GetUnsafe(i);

                // RVO
                var vab = vcand * 2 - vel - cir.vel;

                // Side
                side += Math.Clamp(Math.Min(Vector2.Dot(cir.dp, vab) * 0.5f + 0.5f, Vector2.Dot(cir.np, vab) * 2), 0.0f, 1.0f);
                nside++;

                if (!SweepCircleCircle(pos, rad, vab, cir.p, cir.rad, out var htmin, out var htmax))
                    continue;

                // Handle overlapping obstacles.
                if (htmin < 0 && htmax > 0)
                {
                    // Avoid more when overlapped.
                    htmin = -htmin / 2;
                }

                if (htmin >= 0)
                {
                    // The closest obstacle is somewhere ahead of us, keep track of nearest obstacle.
                    if (htmin < tmin)
                    {
                        if (htmin < tThresold)
                            return minPenalty;

                        tmin = htmin;
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
                    var sdir = seg.q - seg.p;
                    var snorm = new Vector2(-sdir.Y, sdir.X);
                    // If the velocity is pointing towards the segment, no collision.
                    if (Vector2.Dot(snorm, vcand) < 0.0f)
                        continue;
                    // Else immediate collision.
                    htmin = 0.0f;
                }
                else
                {
                    if (!IsectRaySeg(pos, vcand, seg.p, seg.q, ref htmin))
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

            float spen = nside == 0
                ? 0
                // Normalize side bias, to prevent it dominating too much.
                : m_params.weightSide * (side / nside);

            float tpen = m_params.weightToi * (1f / (0.1f + tmin * m_invHorizTime));

            float penalty = vpen + vcpen + spen + tpen;

            // Store different penalties for debug viewing
            debug?.AddSample(vcand, cs, penalty, vpen, vcpen, spen, tpen);

            return penalty;
        }

        public int SampleVelocityGrid(Vector2 pos, float rad, float vmax, Vector2 vel, Vector2 dvel, out Vector2 nvel,
            DtObstacleAvoidanceParams option, DtObstacleAvoidanceDebugData debug)
        {
            Prepare(pos, dvel);
            m_params = option;
            m_invHorizTime = 1f / m_params.horizTime;
            m_vmax = vmax;
            m_invVmax = vmax > 0 ? 1f / vmax : float.MaxValue;

            nvel = default;

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
        static Vector2 DtRotate2D(float[] v, float ang)
        {
            float c = (float)Math.Cos(ang);
            float s = (float)Math.Sin(ang);
            return new Vector2(v[0] * c - v[2] * s, v[0] * s + v[2] * c);
        }

        static readonly float DT_PI = 3.14159265f;

        public int SampleVelocityAdaptive(Vector2 pos, float rad, float vmax, Vector2 vel, Vector2 dvel, out Vector2 nvel,
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
            float[] pat = ArrayPool<float>.Shared.Rent((DT_MAX_PATTERN_DIVS * DT_MAX_PATTERN_RINGS + 1) * 2);
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
            var rotated = DtRotate2D(ddir, da * 0.5f); // rotated by da/2
            ddir[3] = rotated.X;
            ddir[4] = 0;
            ddir[5] = rotated.Y;

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

            nvel = dvel * m_params.velBias;
            int ns = 0;
            for (int k = 0; k < depth; ++k)
            {
                float minPenalty = float.MaxValue;
                Vector2 bvel = Vector2.Zero;

                for (int i = 0; i < npat; ++i)
                {
                    var vcand = nvel + new Vector2(pat[i * 2], pat[i * 2 + 1]) * cr;

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

                nvel = bvel;

                cr /= 2;
            }

            ArrayPool<float>.Shared.Return(pat);

            return ns;
        }
    }
}