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

using System.Diagnostics;
using System;
using System.Numerics;

namespace DotRecast.Recast
{
    /** A compact, static heightfield representing unobstructed space. */
    public struct RcCompactHeightfield
    {
        /** The width of the heightfield. (Along the x-axis in cell units.) */
        public readonly int width;

        /** The height of the heightfield. (Along the z-axis in cell units.) */
        public readonly int height;

        /** The number of spans in the heightfield. */
        public readonly int spanCount;
        /** The minimum bounds in world space. [(x, y, z)] */
        public readonly Vector3 bmin;

        /** The maximum bounds in world space. [(x, y, z)] */
        public readonly Vector3 bmax;

        /** The size of each cell. (On the xz-plane.) */
        public readonly float cs;

        /** The height of each cell. (The minimum increment along the y-axis.) */
        public readonly float ch;

        /** The walkable height used during the build of the field. (See: RecastConfig::walkableHeight) */
        public readonly int walkableHeight;

        /** The walkable climb used during the build of the field. (See: RecastConfig::walkableClimb) */
        public readonly int walkableClimb;

        /** The AABB border size used during the build of the field. (See: RecastConfig::borderSize) */
        public int borderSize;

        /** The maximum distance value of any span within the field. */
        public int maxDistance;

        /** The maximum region id of any span within the field. */
        public int maxRegions;

        /** Array of cells. [Size: #width*#height] */
        public readonly RcCompactCell[] cells;

        /** Array of spans. [Size: #spanCount] */
        public readonly RcCompactSpan[] spans;

        /** Array containing border distance data. [Size: #spanCount] */
        public int[] dist;

        /** Array containing area id data. [Size: #spanCount] */
        public int[] areas;

        private const int MAX_LAYERS = RcConstants.RC_NOT_CONNECTED - 1;
        private const int MAX_HEIGHT = RcConstants.SPAN_MAX_HEIGHT;

        /// @par
        ///
        /// This is just the beginning of the process of fully building a compact heightfield.
        /// Various filters may be applied, then the distance field and regions built.
        /// E.g: #rcBuildDistanceField and #rcBuildRegions
        ///
        /// See the #rcConfig documentation for more information on the configuration parameters.
        ///
        /// @see rcAllocCompactHeightfield, in RcHeightfield, rcCompactHeightfield, rcConfig
        public RcCompactHeightfield(in RcHeightfield heightfield, in RcConfig rcConfig)
        {
            spanCount = GetHeightFieldSpanCount(heightfield);

            // Fill in header.
            width = heightfield.width;
            height = heightfield.height;
            bmin = heightfield.bmin;
            bmax = heightfield.bmax;
            bmax.Y += walkableHeight * heightfield.ch;
            cs = heightfield.cs;
            ch = heightfield.ch;
            cells = new RcCompactCell[width * height];
            spans = new RcCompactSpan[spanCount];
            areas = new int[spanCount];

            walkableHeight = rcConfig.WalkableHeight;
            walkableClimb = rcConfig.WalkableClimb;
            borderSize = heightfield.borderSize;
            maxRegions = 0;

            for (int i = 0; i < spans.Length; i++)
            {
                spans[i] = new RcCompactSpan();
            }

            var w = heightfield.width;
            var h = heightfield.height;

            // Fill in cells and spans.
            int idx = 0;
            for (int y = 0; y < h; ++y)
            {
                for (int x = 0; x < w; ++x)
                {
                    var list = heightfield.spans[x + y * w];
                    // If there are no spans at this cell, just leave the data to index=0, count=0.
                    if (list is null)
                        continue;

                    Debug.Assert(list.Count > 0);

                    int tmpIdx = idx;
                    int tmpCount = 0;
                    for (int i = 0; i < list.Count; i++)
                    {
                        var span = list[i];
                        if (span.area != RcConstants.RC_NULL_AREA)
                        {
                            int bot = span.smax;
                            var hasNext = i + 1 < list.Count;
                            int top = hasNext ? list[i + 1].smin : MAX_HEIGHT;
                            spans[idx].y = Math.Clamp(bot, 0, MAX_HEIGHT);
                            spans[idx].h = Math.Clamp(top - bot, 0, MAX_HEIGHT);
                            areas[idx] = span.area;
                            idx++;
                            tmpCount++;
                        }
                    }

                    cells[x + y * w] = new RcCompactCell(tmpIdx, tmpCount);
                }
            }

            // Find neighbour connections.
            int tooHighNeighbour = 0;
            for (int y = 0; y < h; ++y)
            {
                for (int x = 0; x < w; ++x)
                {
                    RcCompactCell c = cells[x + y * w];
                    for (int i = c.index, ni = c.index + c.count; i < ni; ++i)
                    {
                        ref RcCompactSpan s = ref spans[i];

                        for (int dir = 0; dir < 4; ++dir)
                        {
                            s.SetCon(dir, RcConstants.RC_NOT_CONNECTED);
                            int nx = x + RcCommons.GetDirOffsetX(dir);
                            int ny = y + RcCommons.GetDirOffsetY(dir);
                            // First check that the neighbour cell is in bounds.
                            if (nx < 0 || ny < 0 || nx >= w || ny >= h)
                                continue;

                            // Iterate over all neighbour spans and check if any of the is
                            // accessible from current cell.
                            RcCompactCell nc = cells[nx + ny * w];
                            for (int k = nc.index, nk = nc.index + nc.count; k < nk; ++k)
                            {
                                RcCompactSpan ns = spans[k];
                                int bot = Math.Max(s.y, ns.y);
                                int top = Math.Min(s.y + s.h, ns.y + ns.h);

                                // Check that the gap between the spans is walkable,
                                // and that the climb height between the gaps is not too high.
                                if ((top - bot) >= walkableHeight && Math.Abs(ns.y - s.y) <= walkableClimb)
                                {
                                    // Mark direction as walkable.
                                    int lidx = k - nc.index;
                                    if (lidx < 0 || lidx > MAX_LAYERS)
                                    {
                                        tooHighNeighbour = Math.Max(tooHighNeighbour, lidx);
                                        continue;
                                    }

                                    s.SetCon(dir, lidx);
                                    break;
                                }
                            }
                        }
                    }
                }
            }

            if (tooHighNeighbour > MAX_LAYERS)
            {
                throw new Exception("rcBuildCompactHeightfield: Heightfield has too many layers " + tooHighNeighbour + " (max: " + MAX_LAYERS + ")");
            }
        }

        private static int GetHeightFieldSpanCount(in RcHeightfield hf)
        {
            var w = hf.width;
            var h = hf.height;
            var spanCount = 0;
            for (int y = 0; y < h; ++y)
            {
                for (int x = 0; x < w; ++x)
                {
                    var list = hf.spans[x + y * w];
                    if (list != null)
                        for (int i = 0; i < list.Count; i++)
                        {
                            var span = list[i];
                            if (span.area != RcConstants.RC_NULL_AREA)
                                spanCount++;
                        }
                }
            }

            return spanCount;
        }
    }
}