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
using System.Collections.Generic;
using System.Linq;
using DotRecast.Recast;

namespace DotRecast.Detour.Dynamic
{
    public class DtDynamicTileCheckpoint
    {
        public readonly RcHeightfield heightfield;
        public readonly ISet<long> colliders;

        public DtDynamicTileCheckpoint(in RcHeightfield heightfield, ISet<long> colliders)
        {
            this.colliders = colliders;
            this.heightfield = Clone(heightfield);
        }

        private static RcHeightfield Clone(in RcHeightfield source)
        {
            RcHeightfield clone = new(source.width, source.height, source.bmin, source.bmax, source.cs, source.ch, source.borderSize);
            for (int z = 0, pz = 0; z < source.height; z++, pz += source.width)
            {
                for (int x = 0; x < source.width; x++)
                {
                    var list = source.spans[pz + x];
                    if (list != null)
                    {
                        clone.spans[pz + x] = list.ToList();
                    }
                }
            }

            return clone;
        }
    }
}