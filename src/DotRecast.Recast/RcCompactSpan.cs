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

namespace DotRecast.Recast
{
    /** Represents a span of unobstructed space within a compact heightfield. */
    public struct RcCompactSpan
    {
        /** The lower extent of the span. (Measured from the heightfield's base.) */
        public int y;

        /** The id of the region the span belongs to. (Or zero if not in a region.) */
        public int reg;

        /** Packed neighbor connection data. */
        public int con;

        /** The height of the span. (Measured from #y.) */
        public int h;

        /// Sets the neighbor connection data for the specified direction.
        /// @param[in]		span			The span to update.
        /// @param[in]		direction		The direction to set. [Limits: 0 <= value < 4]
        /// @param[in]		neighborIndex	The index of the neighbor span.
        public void SetCon(int direction, int neighborIndex)
        {
            int shift = direction * 6;
            con = (con & ~(0x3f << shift)) | ((neighborIndex & 0x3f) << shift);
        }
    }
}