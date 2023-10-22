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
using System.Runtime.CompilerServices;
using DotRecast.Core;
using DotRecast.Detour.Crowd.Tracking;
using UnityEngine;

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

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void AddCircle(in Vector2 pos, float rad, in Vector2 vel, in Vector2 dvel)
        {
            if (m_ncircles >= m_maxCircles)
                return;

            DtObstacleCircle cir = m_circles[m_ncircles++];
            cir.p = pos;
            cir.rad = rad;
            cir.vel = vel;
            cir.dvel = dvel;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void AddSegment(in Vector2 p, in Vector2 q)
        {
            if (m_nsegments >= m_maxSegments)
                return;

            ref var segment = ref m_segments[m_nsegments++];
            segment.p = p;
            segment.q = q;
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

                float a = Vector3Extensions.Perp2D(in cir.dp, in dv);

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
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining | MethodImplOptions.AggressiveOptimization)]
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
        private float ProcessSample(in Vector2 vcand, float cs, in Vector2 pos, float rad, in Vector2 vel, in Vector2 dvel,
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

            var vabPre = vcand * 2 - vel;

            // Find min time of impact and exit amongst all obstacles.
            var tmin = m_params.horizTime;
            float side = 0;

            for (int i = 0; i != m_ncircles; i++)
            {
                DtObstacleCircle cir = m_circles[i];

                // RVO
                var vab = vabPre - cir.vel;

                // Side
                side += Mathf.Clamp01(MathF.Min(Vector2.Dot(cir.dp, vab) / 2 + 0.5f, Vector2.Dot(cir.np, vab) * 2));

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

                //var distSqr = DtUtils.DistancePtSegSqr2D(pos, seg.p, seg.q, out _);
                var sdir = seg.q - seg.p;
                var w = pos - seg.p;
                var t = Vector2.Dot(sdir, w);

                float d = sdir.LengthSquared();
                t = Mathf.Clamp01(t / (d > 0 ? d : 1));

                var distSqr = (t * sdir - w).LengthSquared();


                const float r = 0.01f;
                var touch = distSqr < r * r;

                //if (touch)
                //{
                //    // Special case when the agent is very close to the segment.
                //    var snorm = new Vector2(-sdir.Y, sdir.X);
                //    // If the velocity is pointing towards the segment, no collision.
                //    //if (Vector2.Dot(snorm, vcand) < 0)
                //    //    continue;
                //    //// Else immediate collision.
                //    //htmin = 0;

                //    htmin = Vector2.Dot(snorm, vcand) < 0 ? tmin : 0;
                //}
                //else  // IsectRaySeg                
                //{
                //    float dvcand = Vector3Extensions.Perp2D(vcand, sdir);
                //    //if (MathF.Abs(d) < 1e-6f)
                //    //    continue;

                //    htmin = Vector3Extensions.Perp2D(sdir, w) / dvcand;
                //    //if (htmin < 0 || htmin > 1)
                //    //    continue;

                //    float s = Vector3Extensions.Perp2D(vcand, w) / dvcand;
                //    //if (s < 0 || s > 1)
                //    //    continue;

                //    htmin = MathF.Abs(d) < 1e-6f || htmin < 0 || htmin > 1 || s < 0 || s > 1 ? tmin : htmin;
                //}

                // Avoid less when facing walls.
                const float f = 2;

                var snorm = new Vector2(-sdir.Y, sdir.X);

                float dvcand = Vector3Extensions.Perp2D(vcand, sdir);

                float htmin = Vector3Extensions.Perp2D(sdir, w) / dvcand;

                float s = Vector3Extensions.Perp2D(vcand, w) / dvcand;

                htmin = touch && Vector2.Dot(snorm, vcand) < 0 ? float.MaxValue / f : (!touch && MathF.Abs(d) < 1e-6f || htmin < 0 || htmin > 1 || s < 0 || s > 1 ? float.MaxValue / f : htmin);

                // The closest obstacle is somewhere ahead of us, keep track of nearest obstacle.
                tmin = MathF.Min(tmin, htmin * f);
                if (tmin < tThresold)
                    return minPenalty;
            }

            float spen = m_ncircles == 0
                ? 0
                // Normalize side bias, to prevent it dominating too much.
                : m_params.weightSide * (side / m_ncircles);

            float tpen = m_params.weightToi / (0.1f + tmin * m_invHorizTime);

            float penalty = vpen + vcpen + spen + tpen;

            // Store different penalties for debug viewing
            debug?.AddSample(vcand, cs, penalty, vpen, vcpen, spen, tpen);

            return penalty;
        }

        //public int SampleVelocityGrid(Vector2 pos, float rad, float vmax, Vector2 vel, Vector2 dvel, out Vector2 nvel,
        //    DtObstacleAvoidanceParams option, DtObstacleAvoidanceDebugData debug)
        //{
        //    Prepare(pos, dvel);
        //    m_params = option;
        //    m_invHorizTime = 1f / m_params.horizTime;
        //    m_vmax = vmax;
        //    m_invVmax = vmax > 0 ? 1f / vmax : float.MaxValue;

        //    nvel = default;

        //    debug?.Reset();

        //    var cv = dvel * m_params.velBias;
        //    float cs = vmax * 2 * (1 - m_params.velBias) / (m_params.gridSize - 1);
        //    float half = (m_params.gridSize - 1) * cs * 0.5f;

        //    float minPenalty = float.MaxValue;
        //    int ns = 0;

        //    for (int y = 0; y < m_params.gridSize; ++y)
        //    {
        //        for (int x = 0; x < m_params.gridSize; ++x)
        //        {
        //            Vector2 vcand = new(cv.X + x * cs - half, cv.Y + y * cs - half);
        //            if (vcand.LengthSquared() > RcMath.Sqr(vmax + cs / 2))
        //                continue;

        //            float penalty = ProcessSample(new Vector2(vcand.X, vcand.Y), cs, pos, rad, vel, dvel, minPenalty, debug);
        //            ns++;
        //            if (penalty < minPenalty)
        //            {
        //                minPenalty = penalty;
        //                nvel = new Vector2(vcand.X, vcand.Y);
        //            }
        //        }
        //    }

        //    return ns;
        //}

        public void SampleVelocityAdaptive(Vector2 pos, float rad, float vmax, Vector2 vel, Vector2 dvel, out Vector2 nvel,
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

            int ndivs = m_params.adaptiveDivs;
            int nrings = m_params.adaptiveRings;

            int nd = Math.Clamp(ndivs, 1, DT_MAX_PATTERN_DIVS);
            int nr = Math.Clamp(nrings, 1, DT_MAX_PATTERN_RINGS);
            var da = 1f / nd * float.Pi * 2;
            var ca = MathF.Cos(da);
            var sa = MathF.Sin(da);

            // desired direction
            var di = Vector2.Normalize(dvel);

            // DtRotate by da/2
            var ang = da / 2;

            // rotated by da/2
            var rotc = di * MathF.Cos(ang);
            var rots = di * MathF.Sin(ang);

            ReadOnlySpan<Vector2> ddir = stackalloc System.Numerics.Vector2[] { di, new Vector2(rotc.X - rots.Y, rots.X + rotc.Y) };

            var pat = ArrayPool<float>.Shared.Rent((ndivs * nrings + 1) * 2);

            // Always add sample at zero
            pat.UnsafeAs<float, Vector2>() = default;

            //var preCalculated = (nd % 2) == 0;
            //const int preCalculatedInt = 0; // Unsafe.As<bool, int>(ref preCalculated);

            float nrInverted = 1f / nr;

            int npat = 1;
            for (int j = 0; j < nr; ++j)
            {
                int last1 = npat * 2;
                int last2 = last1;

                var r = (nr - j) * nrInverted;
                pat.UnsafeAs<float, Vector2>(npat) = ddir[j % 2] * r; // npat == 1 + (end + preCalculatedInt + 1) * j

                npat++;

                for (int i = 1; i < nd - 1; i += 2)
                {
                    var npat2 = npat * 2;
                    // get next point on the "right" (rotate CW)
                    pat[npat2] = pat[last1] * ca + pat[last1 + 1] * sa;
                    pat[npat2 + 1] = -pat[last1] * sa + pat[last1 + 1] * ca;
                    // get next point on the "left" (rotate CCW)
                    pat[npat2 + 2] = pat[last2] * ca - pat[last2 + 1] * sa;
                    pat[npat2 + 3] = pat[last2] * sa + pat[last2 + 1] * ca;

                    last1 = npat2;
                    last2 = last1 + 2;
                    npat += 2;
                }

                //if (preCalculated) // nd % 2 != 0
                //{
                //    pat[npat * 2 + 2] = pat[last2] * ca - pat[last2 + 1] * sa;
                //    pat[npat * 2 + 3] = pat[last2] * sa + pat[last2 + 1] * ca;
                //    npat++;
                //}
            }

            // Start sampling.
            float cr = vmax * (1f - m_params.velBias);
            float cs = cr / 10;

            nvel = dvel * m_params.velBias;

            int depth = m_params.adaptiveDepth;

            //int ns = 0;
            for (int k = 0; k < depth; ++k)
            {
                float minPenalty = float.MaxValue;
                Vector2 bvel = default;

                for (int i = 0; i < npat; ++i)
                {
                    var vcand = nvel + pat.UnsafeAs<float, Vector2>(i) * cr;

                    if (vcand.LengthSquared() > RcMath.Sqr(vmax + 0.001f))
                        continue;

                    float penalty = ProcessSample(vcand, cs, pos, rad, vel, dvel, minPenalty, debug);
                    //ns++;
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

            //return ns;
        }
    }
}