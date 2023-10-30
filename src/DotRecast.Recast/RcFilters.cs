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
using static DotRecast.Recast.RcConstants;
using static DotRecast.Recast.RcCommons;

namespace DotRecast.Recast
{
    public static class RcFilters
    {
        /// @par
        ///
        /// Allows the formation of walkable regions that will flow over low lying
        /// objects such as curbs, and up structures such as stairways.
        ///
        /// Two neighboring spans are walkable if: <tt>RcAbs(currentSpan.smax - neighborSpan.smax) < walkableClimb</tt>
        ///
        /// @warning Will override the effect of #rcFilterLedgeSpans. So if both filters are used, call
        /// #rcFilterLedgeSpans after calling this filter.
        ///
        /// @see rcHeightfield, rcConfig
        public static void FilterLowHangingWalkableObstacles(int walkableClimb, in RcHeightfield solid)
        {
            int w = solid.width;
            int h = solid.height;

            for (int y = 0; y < h; ++y)
            {
                for (int x = 0; x < w; ++x)
                {
                    bool previousWalkable = false;
                    int previousArea = RC_NULL_AREA;

                    int prevSpanMax = default;

                    var list = solid.spans[x + y * w];
                    if (list != null)
                        for (int i = 0; i < list.Count; i++)
                        {
                            var span = list[i];
                            bool walkable = span.area != RC_NULL_AREA;
                            // If current span is not walkable, but there is walkable
                            // span just below it, mark the span above it walkable too.
                            if (!walkable && previousWalkable)
                            {
                                if (Math.Abs(span.smax - prevSpanMax) <= walkableClimb)
                                    list[i] = new(span.smin, span.smax, previousArea);
                            }

                            // Copy walkable flag so that it cannot propagate
                            // past multiple non-walkable objects.
                            previousWalkable = walkable;
                            previousArea = span.area;
                            prevSpanMax = span.smax;
                        }
                }
            }
        }

        /// @par
        ///
        /// A ledge is a span with one or more neighbors whose maximum is further away than @p walkableClimb
        /// from the current span's maximum.
        /// This method removes the impact of the overestimation of conservative voxelization
        /// so the resulting mesh will not have regions hanging in the air over ledges.
        ///
        /// A span is a ledge if: <tt>RcAbs(currentSpan.smax - neighborSpan.smax) > walkableClimb</tt>
        ///
        /// @see rcHeightfield, rcConfig
        public static void FilterLedgeSpans(int walkableHeight, int walkableClimb, in RcHeightfield solid)
        {
            var w = solid.width;
            var h = solid.height;

            // Mark border spans.
            for (int y = 0; y < h; ++y)
            {
                for (int x = 0; x < w; ++x)
                {
                    var list = solid.spans[x + y * w];
                    if (list != null)
                        for (int i = 0; i < list.Count; i++)
                        {
                            var span = list[i];

                            // Skip non walkable spans.
                            if (span.area == RC_NULL_AREA)
                                continue;

                            var bot = span.smax;
                            var hasNext = i + 1 < list.Count;
                            var top = hasNext ? list[i + 1!].smin : SPAN_MAX_HEIGHT;

                            // Find neighbours minimum height.
                            int minh = SPAN_MAX_HEIGHT;

                            // Min and max height of accessible neighbours.
                            var asmin = span.smax;
                            var asmax = span.smax;

                            for (int dir = 0; dir < 4; ++dir)
                            {
                                int dx = x + GetDirOffsetX(dir);
                                int dy = y + GetDirOffsetY(dir);
                                // Skip neighbours which are out of bounds.
                                if (dx < 0 || dy < 0 || dx >= w || dy >= h)
                                {
                                    minh = Math.Min(minh, -walkableClimb - bot);
                                    continue;
                                }

                                // From minus infinity to the first span.
                                var listMinus = solid.spans[dx + dy * w];
                                var list2HasSpans = listMinus != null;

                                int nbot = -walkableClimb;
                                int ntop = list2HasSpans ? listMinus[0].smin : SPAN_MAX_HEIGHT;

                                // Skip neightbour if the gap between the spans is too small.
                                if (Math.Min(top, ntop) - Math.Max(bot, nbot) > walkableHeight)
                                {
                                    minh = Math.Min(minh, nbot - bot);
                                }

                                // Rest of the spans.
                                if (list2HasSpans)
                                {
                                    for (int i2 = 0; i2 < listMinus.Count; i2++)
                                    {
                                        var ns = listMinus[i2];
                                        nbot = ns.smax;
                                        hasNext = i2 + 1 < listMinus.Count;
                                        ntop = hasNext ? listMinus[i2 + 1].smin : SPAN_MAX_HEIGHT;
                                        // Skip neightbour if the gap between the spans is too small.
                                        if (Math.Min(top, ntop) - Math.Max(bot, nbot) > walkableHeight)
                                        {
                                            minh = Math.Min(minh, nbot - bot);

                                            // Find min/max accessible neighbour height.
                                            if (Math.Abs(nbot - bot) <= walkableClimb)
                                            {
                                                if (nbot < asmin)
                                                    asmin = nbot;
                                                if (nbot > asmax)
                                                    asmax = nbot;
                                            }
                                        }
                                    }
                                }
                            }

                            if (
                                // The current span is close to a ledge if the drop to any
                                // neighbour span is less than the walkableClimb.
                                minh < -walkableClimb
                                // If the difference between all neighbours is too large,
                                // we are at steep slope, mark the span as ledge.
                                || (asmax - asmin) > walkableClimb)
                            {
                                list[i] = new(span.smin, span.smax, RC_NULL_AREA);
                            }

                        }
                }
            }
        }

        /// @par
        ///
        /// For this filter, the clearance above the span is the distance from the span's
        /// maximum to the next higher span's minimum. (Same grid column.)
        ///
        /// @see rcHeightfield, rcConfig
        public static void FilterWalkableLowHeightSpans(int walkableHeight, in RcHeightfield solid)
        {
            int w = solid.width;
            int h = solid.height;

            // Remove walkable flag from spans which do not have enough
            // space above them for the agent to stand there.
            for (int y = 0; y < h; ++y)
            {
                for (int x = 0; x < w; ++x)
                {
                    var list = solid.spans[x + y * w];
                    if (list != null)
                        for (int i = 0; i < list.Count; i++)
                        {
                            var span = list[i];
                            int bot = span.smax;
                            var hasNext = i + 1 != list.Count;
                            int top = hasNext ? list[i + 1].smin : SPAN_MAX_HEIGHT;
                            if ((top - bot) < walkableHeight)
                                list[i] = new(span.smin, span.smax, RC_NULL_AREA);
                        }
                }
            }
        }
    }
}