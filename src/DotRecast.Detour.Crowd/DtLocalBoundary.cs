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

using System.Collections.Generic;
using System.Numerics;
using DotRecast.Core;


namespace DotRecast.Detour.Crowd
{
    public struct DtLocalBoundary
    {
        public const int MAX_LOCAL_SEGS = 8;

        private Vector3 m_center;
        public List<DtSegment> Segments { get; } = new();
        private List<long> m_polys = new();
        private List<long> m_parents = new();

        public DtLocalBoundary()
        {
            m_center.X = m_center.Y = m_center.Z = float.MaxValue;
        }

        public void Reset()
        {
            m_center.X = m_center.Y = m_center.Z = float.MaxValue;
            m_polys.Clear();
            Segments.Clear();
        }

        private void AddSegment(float dist, RcSegmentVert s)
        {
            // Insert neighbour based on the distance.
            DtSegment seg = new()
            {
                Start = s.vmin.AsVector2XZ(),
                End = s.vmax.AsVector2XZ(),
                //Array.Copy(s, seg.s, 6);
                PruningDistance = dist
            };

            if (Segments.Count is 0)
            {
                Segments.Add(seg);
            }
            else if (dist >= Segments[^1].PruningDistance)
            {
                if (Segments.Count >= MAX_LOCAL_SEGS)
                {
                    return;
                }

                Segments.Add(seg);
            }
            else
            {
                // Insert inbetween.
                int i;
                for (i = 0; i < Segments.Count; ++i)
                {
                    if (dist <= Segments[i].PruningDistance)
                    {
                        break;
                    }
                }

                Segments.Insert(i, seg);
            }

            while (Segments.Count > MAX_LOCAL_SEGS)
            {
                Segments.RemoveAt(Segments.Count - 1);
            }
        }

        public void Update(long startRef, Vector3 pos, float collisionQueryRange, DtNavMeshQuery navquery, IDtQueryFilter filter)
        {
            if (startRef is 0)
            {
                Reset();
                return;
            }

            m_center = pos;

            // First query non-overlapping polygons.
            var status = navquery.FindLocalNeighbourhood(startRef, pos, collisionQueryRange, filter, ref m_polys, ref m_parents);
            if (status.Succeeded())
            {
                // Secondly, store all polygon edges.
                Segments.Clear();

                var segmentVerts = new List<RcSegmentVert>();
                var segmentRefs = new List<long>();

                var collisionQueryRangeSqr = RcMath.Sqr(collisionQueryRange);

                for (int j = 0; j < m_polys.Count; ++j)
                {
                    var result = navquery.GetPolyWallSegments(m_polys[j], false, filter, ref segmentVerts, ref segmentRefs);
                    if (result.Succeeded())
                    {
                        for (int k = 0; k < segmentRefs.Count; ++k)
                        {
                            RcSegmentVert s = segmentVerts[k];
                            var s0 = s.vmin;
                            var s3 = s.vmax;


                            // Skip too distant segments.
                            var distSqr = DtUtils.DistancePtSegSqr2D(pos, s0, s3, out _);
                            if (distSqr > collisionQueryRangeSqr)
                            {
                                continue;
                            }

                            AddSegment(distSqr, s);
                        }
                    }
                }
            }
        }

        public readonly bool IsValid(DtNavMeshQuery navquery, IDtQueryFilter filter)
        {
            if (m_polys.Count is 0)
            {
                return false;
            }

            // Check that all polygons still pass query filter.
            foreach (long refs in m_polys)
            {
                if (!navquery.IsValidPolyRef(refs, filter))
                {
                    return false;
                }
            }

            return true;
        }

        public readonly Vector3 GetCenter()
        {
            return m_center;
        }

        public readonly int GetSegmentCount()
        {
            return Segments.Count;
        }
    }
}