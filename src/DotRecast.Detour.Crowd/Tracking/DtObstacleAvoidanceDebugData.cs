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

namespace DotRecast.Detour.Crowd.Tracking
{


    public class DtObstacleAvoidanceDebugData
    {
        private int m_nsamples;
        private readonly int m_maxSamples;
        private readonly float[] m_vel;
        private readonly float[] m_ssize;
        private readonly float[] m_pen;
        private readonly float[] m_vpen;
        private readonly float[] m_vcpen;
        private readonly float[] m_spen;
        private readonly float[] m_tpen;

        public DtObstacleAvoidanceDebugData(int maxSamples)
        {
            m_maxSamples = maxSamples;
            m_vel = new float[2 * m_maxSamples];
            m_pen = new float[m_maxSamples];
            m_ssize = new float[m_maxSamples];
            m_vpen = new float[m_maxSamples];
            m_vcpen = new float[m_maxSamples];
            m_spen = new float[m_maxSamples];
            m_tpen = new float[m_maxSamples];
        }

        public void Reset()
        {
            m_nsamples = 0;
        }

        static void NormalizeArray(float[] arr, int n)
        {
            // Normalize penaly range.
            float minPen = float.MaxValue;
            float maxPen = -float.MaxValue;
            for (int i = 0; i < n; ++i)
            {
                minPen = MathF.Min(minPen, arr[i]);
                maxPen = MathF.Max(maxPen, arr[i]);
            }

            float penRange = maxPen - minPen;
            float s = penRange > 0.001f ? (1f / penRange) : 1;
            for (int i = 0; i < n; ++i)
                arr[i] = Math.Clamp((arr[i] - minPen) * s, 0f, 1f);
        }

        public void NormalizeSamples()
        {
            NormalizeArray(m_pen, m_nsamples);
            NormalizeArray(m_vpen, m_nsamples);
            NormalizeArray(m_vcpen, m_nsamples);
            NormalizeArray(m_spen, m_nsamples);
            NormalizeArray(m_tpen, m_nsamples);
        }

        public void AddSample(Vector2 vel, float ssize, float pen, float vpen, float vcpen, float spen, float tpen)
        {
            if (m_nsamples >= m_maxSamples)
                return;
            m_vel[m_nsamples * 2] = vel.X;
            m_vel[m_nsamples * 2 + 2] = vel.Y;
            m_ssize[m_nsamples] = ssize;
            m_pen[m_nsamples] = pen;
            m_vpen[m_nsamples] = vpen;
            m_vcpen[m_nsamples] = vcpen;
            m_spen[m_nsamples] = spen;
            m_tpen[m_nsamples] = tpen;
            m_nsamples++;
        }

        public int GetSampleCount()
        {
            return m_nsamples;
        }

        public Vector2 GetSampleVelocity(int i)
        {
            Vector2 vel = new()
            {
                X = m_vel[i * 2],
                Y = m_vel[i * 2 + 2]
            };
            return vel;
        }

        public float GetSampleSize(int i)
        {
            return m_ssize[i];
        }

        public float GetSamplePenalty(int i)
        {
            return m_pen[i];
        }

        public float GetSampleDesiredVelocityPenalty(int i)
        {
            return m_vpen[i];
        }

        public float GetSampleCurrentVelocityPenalty(int i)
        {
            return m_vcpen[i];
        }

        public float GetSamplePreferredSidePenalty(int i)
        {
            return m_spen[i];
        }

        public float GetSampleCollisionTimePenalty(int i)
        {
            return m_tpen[i];
        }
    }
}