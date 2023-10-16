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

namespace DotRecast.Recast
{
    /** Represents a group of related contours. */
    public class RcContourSet
    {
        /** A list of the contours in the set. */
        public readonly List<RcContour> Conts;

        /** The minimum bounds in world space. [(x, y, z)] */
        public readonly Vector3 Bmin;

        /** The maximum bounds in world space. [(x, y, z)] */
        public readonly Vector3 Bmax;

        /** The size of each cell. (On the xz-plane.) */
        public readonly float Cs;

        /** The height of each cell. (The minimum increment along the y-axis.) */
        public readonly float Ch;

        /** The width of the set. (Along the x-axis in cell units.) */
        public readonly int Width;

        /** The height of the set. (Along the z-axis in cell units.) */
        public readonly int height;

        /** The AABB border size used to generate the source data from which the contours were derived. */
        public readonly int BorderSize;

        /** The max edge error that this contour set was simplified with. */
        public readonly float MaxSimplificationError;

        public RcContourSet(in RcConfig config, in RcCompactHeightfield compactHeightfield)
        {
            Conts = new();
            Bmin = compactHeightfield.bmin;
            Bmax = compactHeightfield.bmax;
            Cs = config.Cs;
            Ch = config.Ch;
            Width = compactHeightfield.width - config.BorderSize * 2;
            height = compactHeightfield.height - config.BorderSize * 2;
            BorderSize = config.BorderSize;
            MaxSimplificationError = config.MaxSimplificationError;

            if (config.BorderSize > 0)
            {
                // If the heightfield was build with bordersize, remove the offset.
                float pad = config.BorderSize * Cs;
                Bmin.X += pad;
                Bmin.Z += pad;
                Bmax.X -= pad;
                Bmax.Z -= pad;
            }
        }
    }
}