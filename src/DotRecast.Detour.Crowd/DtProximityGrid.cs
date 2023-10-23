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
using UnityEngine;

namespace DotRecast.Detour.Crowd
{
    public class DtProximityGrid
    {
        public readonly float CellSize;
        private readonly float _invCellSize;
        private readonly (int[] ids, int length)[] _items;
        public Vector2Int Min { get; }
        public Vector2Int Length { get; }

        public DtProximityGrid(Vector4 planeBounds, float cellSize)
        {
            CellSize = cellSize;
            _invCellSize = 1f / cellSize;

            const int collisionQueryRange = 10;

            var iminx = GetSize(planeBounds.X, collisionQueryRange);
            var iminy = GetSize(planeBounds.Y, collisionQueryRange);
            Min = new Vector2Int(iminx, iminy);

            var imaxx = GetSize(planeBounds.Z, -collisionQueryRange);
            var imaxy = GetSize(planeBounds.W, -collisionQueryRange);
            Length = new Vector2Int(imaxx - Min.x, imaxy - Min.y);

            _items = new (int[] ids, int length)[Length.y * Length.x];
        }

        public void Clear()
        {
            for (int i = 0; i != _items.Length; i++)
            {
                _items[i].length = 0;
            }
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        private int GetSize(float v, int offset)
        {
            return (int)MathF.Floor(v * _invCellSize) - offset;
        }

        public void AddItem(DtCrowdAgent agent, float minx, float miny, float maxx, float maxy)
        {
            var iminx = GetSize(minx, Min.x);
            var iminy = GetSize(miny, Min.y);
            var imaxx = GetSize(maxx, Min.x);
            var imaxy = GetSize(maxy, Min.y);

            for (int y = iminy; y <= imaxy; ++y)
            {
                for (int x = iminx; x <= imaxx; ++x)
                {
                    ref var item = ref _items[y * Length.x + x];

                    if (item.ids == null)
                    {
                        item = (ArrayPool<int>.Shared.Rent(8), 0);
                    }

                    if (item.length == item.ids.Length)
                    {
                        var ar = item.ids;
                        item.ids = ArrayPool<int>.Shared.Rent(item.length * 2);

                        ar.CopyTo(item.ids, 0);
                        ArrayPool<int>.Shared.Return(ar);
                    }

                    item.ids[item.length] = agent.idx;
                    item = (item.ids, item.length + 1);
                }
            }
        }

        public void QueryItems(int[] neighbourAgentIds, DtCrowd dtCrowd, float minx, float miny, float maxx, float maxy, DtCrowdAgent skip)
        {
            int count = 0;
            var iminx = GetSize(minx, Min.x);
            var iminy = GetSize(miny, Min.y);
            var imaxx = GetSize(maxx, Min.x);
            var imaxy = GetSize(maxy, Min.y);

            var agentsCount = dtCrowd.GetActiveAgents().Count;

            var neighbourExistArray = ArrayPool<bool>.Shared.Rent(agentsCount);
            Span<bool> neighbourExist = neighbourExistArray;
            neighbourExist.Clear();

            neighbourExist[skip.idx] = true;

            for (int y = iminy; y <= imaxy; ++y)
            {
                for (int x = iminx; x <= imaxx; ++x)
                {
                    var ids = _items[y * Length.x + x];

                    for (int iId = 0; iId != ids.length; iId++)
                    {
                        var agId = ids.ids.GetUnsafe(iId);
                        ref var exist = ref neighbourExist[agId];

                        neighbourAgentIds[exist ? agentsCount : count++] = agId;
                        exist = true;
                    }
                }
            }

            neighbourAgentIds[count] = -1;
            ArrayPool<bool>.Shared.Return(neighbourExistArray);
        }
    }
}