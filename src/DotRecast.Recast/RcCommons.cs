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
using System.Runtime.CompilerServices;

namespace DotRecast.Recast
{
    using static RcConstants;

    public static class RcCommons
    {
        private static readonly int[] DirOffsetX = { -1, 0, 1, 0, };
        private static readonly int[] DirOffsetY = { 0, 1, 0, -1 };
        private static readonly int[] DirForOffset = { 3, 0, -1, 2, 1 };

        /// Gets neighbor connection data for the specified direction.
        /// @param[in]		span		The span to check.
        /// @param[in]		direction	The direction to check. [Limits: 0 <= value < 4]
        /// @return The neighbor connection data for the specified direction, or #RC_NOT_CONNECTED if there is no connection.
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static int GetCon(RcCompactSpan s, int dir)
        {
            int shift = dir * 6;
            return (s.con >> shift) & 0x3f;
        }

        /// Gets the standard width (x-axis) offset for the specified direction.
        /// @param[in]		direction		The direction. [Limits: 0 <= value < 4]
        /// @return The width offset to apply to the current cell position to move in the direction.
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static int GetDirOffsetX(int dir)
        {
            return DirOffsetX[dir & 0x03];
        }

        // TODO (graham): Rename this to rcGetDirOffsetZ
        /// Gets the standard height (z-axis) offset for the specified direction.
        /// @param[in]		direction		The direction. [Limits: 0 <= value < 4]
        /// @return The height offset to apply to the current cell position to move in the direction.
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static int GetDirOffsetY(int dir)
        {
            return DirOffsetY[dir & 0x03];
        }

        /// Gets the direction for the specified offset. One of x and y should be 0.
        /// @param[in]		offsetX		The x offset. [Limits: -1 <= value <= 1]
        /// @param[in]		offsetZ		The z offset. [Limits: -1 <= value <= 1]
        /// @return The direction that represents the offset.
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static int GetDirForOffset(int x, int y)
        {
            return DirForOffset[((y + 1) << 1) + x];
        }

        public static void CalcBounds(float[] verts, int nv, float[] bmin, float[] bmax)
        {
            for (int i = 0; i < 3; i++)
            {
                bmin[i] = verts[i];
                bmax[i] = verts[i];
            }

            for (int i = 1; i < nv; ++i)
            {
                for (int j = 0; j < 3; j++)
                {
                    bmin[j] = Math.Min(bmin[j], verts[i * 3 + j]);
                    bmax[j] = Math.Max(bmax[j], verts[i * 3 + j]);
                }
            }
            // Calculate bounding box.
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void CalcGridSize(Vector3 bmin, Vector3 bmax, float cs, out int sizeX, out int sizeZ)
        {
            sizeX = (int)((bmax.X - bmin.X) / cs + 0.5f);
            sizeZ = (int)((bmax.Z - bmin.Z) / cs + 0.5f);
        }


        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void CalcTileCount(Vector3 bmin, Vector3 bmax, float cs, int tileSizeX, int tileSizeZ, out int tw, out int td)
        {
            CalcGridSize(bmin, bmax, cs, out var gw, out var gd);
            tw = (gw + tileSizeX - 1) / tileSizeX;
            td = (gd + tileSizeZ - 1) / tileSizeZ;
        }

        /// @par
        ///
        /// Modifies the area id of all triangles with a slope below the specified value.
        ///
        /// See the #rcConfig documentation for more information on the configuration parameters.
        ///
        /// @see rcHeightfield, rcClearUnwalkableTriangles, rcRasterizeTriangles
        public static int[] MarkWalkableTriangles(float walkableSlopeAngle, ReadOnlySpan<Vector3> verts, ReadOnlySpan<int> tris, RcAreaModification areaMod)
        {
            int[] areas = new int[tris.Length / 3];
            float walkableThr = (float)Math.Cos(walkableSlopeAngle / 180.0f * Math.PI);
            Vector3 norm = new();
            for (int i = 0; i < areas.Length; ++i)
            {
                int tri = i * 3;
                CalcTriNormal(verts, tris[tri], tris[tri + 1], tris[tri + 2], ref norm);
                // Check if the face is walkable.
                if (norm.Y > walkableThr)
                    areas[i] = areaMod.Apply(areas[i]);
            }

            return areas;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void CalcTriNormal(ReadOnlySpan<Vector3> verts, int v0, int v1, int v2, ref Vector3 norm)
        {
            Vector3 e0 = verts[v1] - verts[v0];
            Vector3 e1 = verts[v2] - verts[v0];

            norm = Vector3.Normalize(Vector3.Cross(e0, e1));
        }

        /// @par
        ///
        /// Only sets the area id's for the unwalkable triangles. Does not alter the
        /// area id's for walkable triangles.
        ///
        /// See the #rcConfig documentation for more information on the configuration parameters.
        ///
        /// @see rcHeightfield, rcClearUnwalkableTriangles, rcRasterizeTriangles
        public static void ClearUnwalkableTriangles(float walkableSlopeAngle, ReadOnlySpan<Vector3> verts, int[] tris, int[] areas)
        {
            float walkableThr = (float)Math.Cos(walkableSlopeAngle / 180.0f * Math.PI);

            Vector3 norm = new();

            for (int i = 0; i < tris.Length / 3; ++i)
            {
                int tri = i * 3;
                CalcTriNormal(verts, tris[tri], tris[tri + 1], tris[tri + 2], ref norm);
                // Check if the face is walkable.
                if (norm.Y <= walkableThr)
                    areas[i] = RC_NULL_AREA;
            }
        }
    }
}