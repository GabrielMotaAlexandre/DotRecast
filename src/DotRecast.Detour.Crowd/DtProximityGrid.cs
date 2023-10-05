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
        private readonly (int[] ids, int length)[] _items;
        public Vector2Int Min { get; }
        public Vector2Int Max { get; }
        public Vector2Int Length { get; }

        public DtProximityGrid(Vector4 planeBounds, float cellSize)
        {
            CellSize = cellSize;
            _invCellSize = 1.0f / cellSize;

            const int collisionQueryRange = 10;

            int iminx = (int)MathF.Floor(planeBounds.X * _invCellSize) - collisionQueryRange;
            int iminy = (int)MathF.Floor(planeBounds.Y * _invCellSize) - collisionQueryRange;
            Min = new Vector2Int(iminx, iminy);
            int imaxx = (int)MathF.Floor(planeBounds.Z * _invCellSize) + collisionQueryRange;
            int imaxy = (int)MathF.Floor(planeBounds.W * _invCellSize) + collisionQueryRange;
            Max = new Vector2Int(imaxx, imaxy);
            Length = new Vector2Int(Max.X - Min.X, Max.Y - Min.Y);


            _items = new (int[] ids, int length)[Length.Y * Length.X];
        }

        public void Clear()
        {
            for (int i = 0; i != _items.Length; i++)
            {
                //ArrayPool<int>.Shared.Return(item.ids);
                _items[i].length = 0;

            }
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Vector2 GetSize()
        {

            //int iminx = (int)MathF.Floor(minx * _invCellSize);
            //int iminy = (int)MathF.Floor(miny * _invCellSize);
            //int imaxx = (int)MathF.Floor(maxx * _invCellSize);
            //int imaxy = (int)MathF.Floor(maxy * _invCellSize);

            return new Vector2();
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
            int iminx = (int)MathF.Floor(minx * _invCellSize) - Min.X;
            int iminy = (int)MathF.Floor(miny * _invCellSize) - Min.Y;
            int imaxx = (int)MathF.Floor(maxx * _invCellSize) - Min.X;
            int imaxy = (int)MathF.Floor(maxy * _invCellSize) - Min.Y;

            for (int y = iminy; y <= imaxy; ++y)
            {
                for (int x = iminx; x <= imaxx; ++x)
                {
                    try
                    {
                        ref var item = ref _items[y * Length.X + x];

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
                    catch (Exception)
                    {

                        throw;
                    }
                }
            }
        }

        public void QueryItems(int[] neighbourAgentIds, DtCrowd dtCrowd, float minx, float miny, float maxx, float maxy, DtCrowdAgent skip)
        {
            int i = 0;
            int iminx = (int)MathF.Floor(minx * _invCellSize) - Min.X;
            int iminy = (int)MathF.Floor(miny * _invCellSize) - Min.Y;
            int imaxx = (int)MathF.Floor(maxx * _invCellSize) - Min.X;
            int imaxy = (int)MathF.Floor(maxy * _invCellSize) - Min.Y;

            var count = dtCrowd.GetActiveAgents().Count;

            var array = ArrayPool<bool>.Shared.Rent(count);
            Array.Clear(array);

            array[skip.idx] = true;

            for (int y = iminy; y <= imaxy; ++y)
            {
                for (int x = iminx; x <= imaxx; ++x)
                {
                    var ids = _items[y * Length.X + x];

                    for (int iId = 0; iId != ids.length; iId++)
                    {
                        var agId = ids.ids.GetUnsafe(iId);
                        ref var exist = ref array[agId];

                        neighbourAgentIds[exist ? count : i++] = agId;
                        exist = true;
                    }
                }
            }
            
            neighbourAgentIds[i] = -1;
            ArrayPool<bool>.Shared.Return(array);
        }
    }

    public readonly struct Vector2Int
    {
        public Vector2Int(int x, int y)
        {
            X = x;
            Y = y;
        }

        public readonly int X;

        public readonly int Y;
    }
}