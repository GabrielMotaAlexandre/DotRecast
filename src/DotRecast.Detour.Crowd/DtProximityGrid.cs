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
using System.Collections.Generic;
using System.Linq;
using System.Numerics;
using System.Runtime.CompilerServices;
using System.Runtime.Intrinsics;

namespace DotRecast.Detour.Crowd
{
    public class DtProximityGrid
    {
        public readonly float CellSize;
        private readonly float _invCellSize;
        private readonly Dictionary<long, (DtCrowdAgent[], int length)> _items = new();
        public Vector4 PlaneBounds { get; private set; }

        public DtProximityGrid(Vector4 planeBounds, float cellSize)
        {
            PlaneBounds = planeBounds;
            CellSize = cellSize;
            _invCellSize = 1.0f / cellSize;
        }

        public void Clear()
        {
            foreach(var item in _items.Values)
            {
                ArrayPool<DtCrowdAgent>.Shared.Return(item.Item1);
            }
            _items.Clear();
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static long CombineKey(int x, int y)
        {
            uint ux = (uint)x;
            uint uy = (uint)y;
            return ((long)ux << 32) | uy;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void DecomposeKey(long key, out int x, out int y)
        {
            uint ux = (uint)(key >> 32);
            uint uy = (uint)key;
            x = (int)ux;
            y = (int)uy;
        }

        public void AddItem(DtCrowdAgent agent, float minx, float miny, float maxx, float maxy)
        {
            int iminx = (int)MathF.Floor(minx * _invCellSize);
            int iminy = (int)MathF.Floor(miny * _invCellSize);
            int imaxx = (int)MathF.Floor(maxx * _invCellSize);
            int imaxy = (int)MathF.Floor(maxy * _invCellSize);

            for (int y = iminy; y <= imaxy; ++y)
            {
                for (int x = iminx; x <= imaxx; ++x)
                {
                    long key = CombineKey(x, y);
                    if (!_items.TryGetValue(key, out var ids))
                    {
                        ids = (ArrayPool<DtCrowdAgent>.Shared.Rent(8), 0);
                        _items.Add(key, ids);
                    }

                    if (ids.length == ids.Item1.Length)
                    {
                        var ar = ids.Item1;
                        ids = (ArrayPool<DtCrowdAgent>.Shared.Rent(ids.length * 2), ids.length);

                        ar.CopyTo(ids.Item1, 0);
                        ArrayPool<DtCrowdAgent>.Shared.Return(ar);
                    }

                    ids.Item1[ids.length] = agent;
                    _items[key] = (ids.Item1, ids.length + 1);
                }
            }
        }

        // 해당 셀 사이즈의 크기로 x ~ y 영역을 찾아, 군집 에이전트를 가져오는 코드
        public DtCrowdAgent[] QueryItems(DtCrowd dtCrowd, float minx, float miny, float maxx, float maxy, DtCrowdAgent skip)
        {
            int i = 0;
            int iminx = (int)MathF.Floor(minx * _invCellSize);
            int iminy = (int)MathF.Floor(miny * _invCellSize);
            int imaxx = (int)MathF.Floor(maxx * _invCellSize);
            int imaxy = (int)MathF.Floor(maxy * _invCellSize);
            var result = ArrayPool<DtCrowdAgent>.Shared.Rent(dtCrowd.GetActiveAgents().Count);
            Array.Clear(result);

            var array = ArrayPool<bool>.Shared.Rent(dtCrowd.GetActiveAgents().Count);
            Array.Clear(array);

            array[skip.idx] = true;

            for (int y = iminy; y <= imaxy; ++y)
            {
                for (int x = iminx; x <= imaxx; ++x)
                {
                    long key = CombineKey(x, y);
                    if (_items.TryGetValue(key, out var ids))
                    {
                        for(int iId = 0; iId != ids.length; iId++)
                        {
                            var ag = ids.Item1.GetUnsafe(iId);
                            ref var a = ref array[ag.idx];

                            if (!a)
                            {
                                result[i++] = ag;
                            }

                            a = true;
                        }
                    }
                }
            }
            ArrayPool<bool>.Shared.Return(array);
            return result;
        }

        public IEnumerable<(long, int)> GetItemCounts()
        {
            return _items
                .Where(e => e.Value.length > 0)
                .Select(e => (e.Key, e.Value.length));
        }

        public float GetCellSize()
        {
            return CellSize;
        }
    }
}