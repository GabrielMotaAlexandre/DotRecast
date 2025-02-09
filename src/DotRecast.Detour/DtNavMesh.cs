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
using System.Collections.Generic;
using System.Numerics;
using System.Runtime.CompilerServices;
using DotRecast.Core;
using UnityEngine;

namespace DotRecast.Detour
{
    public class DtNavMesh
    {
        public const int DT_SALT_BITS = 16;
        public const int DT_TILE_BITS = 28;
        public const int DT_POLY_BITS = 20;
        public const int DT_DETAIL_EDGE_BOUNDARY = 0x01;

        /// A flag that indicates that an entity links to an external entity.
        /// (E.g. A polygon edge is a portal that links to another polygon.)
        public const int DT_EXT_LINK = 0x8000;

        /// A value that indicates the entity does not link to anything.
        public const int DT_NULL_LINK = unchecked((int)0xffffffff);

        /// A flag that indicates that an off-mesh connection can be traversed in
        /// both directions. (Is bidirectional.)
        public const int DT_OFFMESH_CON_BIDIR = 1;

        /// The maximum number of user defined area ids.
        public const int DT_MAX_AREAS = 64;

        /// Limit raycasting during any angle pahfinding
        /// The limit is given as a multiple of the character radius
        public const float DT_RAY_CAST_LIMIT_PROPORTIONS = 50f;

        private readonly DtNavMeshParams m_params;

        /// < Origin of the tile (0,0)
        // float m_orig[3]; ///< Origin of the tile (0,0)
        private readonly float m_tileWidth;

        private readonly float m_tileHeight;

        /**
     * The maximum number of tiles supported by the navigation mesh.
     *
     * @return The maximum number of tiles supported by the navigation mesh.
     */
        /// < Dimensions of each tile.
        public readonly int MaxTiles;

        /// < Max number of tiles.
        private readonly int m_tileLutMask;

        /// < Tile hash lookup mask.
        private readonly Dictionary<int, List<DtMeshTile>> posLookup = new();

        private readonly LinkedList<DtMeshTile> availableTiles = new();
        private readonly DtMeshTile[] m_tiles;

        /// < List of tiles.
        /** The maximum number of vertices per navigation polygon. */
        private readonly int m_maxVertPerPoly;

        private int m_tileCount;

        public DtNavMesh(DtMeshData data, int maxVertsPerPoly, int flags)
            : this(GetNavMeshParams(data), maxVertsPerPoly)
        {
            AddTile(data, flags, 0);
        }

        public DtNavMesh(DtNavMeshParams option, int maxVertsPerPoly)
        {
            m_params = option;
            m_tileWidth = option.tileWidth;
            m_tileHeight = option.tileHeight;
            // Init tiles
            MaxTiles = option.maxTiles;
            m_maxVertPerPoly = maxVertsPerPoly;
            m_tileLutMask = Math.Max(1, DtUtils.NextPow2(option.maxTiles)) - 1;
            m_tiles = new DtMeshTile[MaxTiles];
            for (int i = 0; i < MaxTiles; i++)
            {
                m_tiles[i] = new DtMeshTile(i)
                {
                    salt = 1
                };
                availableTiles.AddLast(m_tiles[i]);
            }
        }

        private static DtNavMeshParams GetNavMeshParams(DtMeshData data)
        {
            DtNavMeshParams option = new()
            {
                orig = data.header.bmin.AsVector2XZ(),
                tileWidth = data.header.bmax.X - data.header.bmin.X,
                tileHeight = data.header.bmax.Z - data.header.bmin.Z,
                maxTiles = 1,
                maxPolys = data.header.polyCount
            };
            return option;
        }

        /**
     * Returns tile in the tile array.
     */
        public DtMeshTile GetTile(int i) => m_tiles[i];

        /**
     * Gets the polygon reference for the tile's base polygon.
     *
     * @param tile
     *            The tile.
     * @return The polygon reference for the base polygon in the specified tile.
     */
        public static long GetPolyRefBase(DtMeshTile tile)
        {
            if (tile is null)
            {
                return 0;
            }

            int it = tile.index;
            return EncodePolyId(tile.salt, it, 0);
        }

        /// @{
        /// @name Encoding and Decoding
        /// These functions are generally meant for internal use only.
        /// Derives a standard polygon reference.
        ///  @note This function is generally meant for internal use only.
        ///  @param[in]	salt	The tile's salt value.
        ///  @param[in]	it		The index of the tile.
        ///  @param[in]	ip		The index of the polygon within the tile.
        public static long EncodePolyId(int salt, int it, int ip)
        {
            return (((long)salt) << (DT_POLY_BITS + DT_TILE_BITS)) | ((long)it << DT_POLY_BITS) | (long)ip;
        }

        /// Decodes a standard polygon reference.
        /// @note This function is generally meant for internal use only.
        /// @param[in] ref The polygon reference to decode.
        /// @param[out] salt The tile's salt value.
        /// @param[out] it The index of the tile.
        /// @param[out] ip The index of the polygon within the tile.
        /// @see #encodePolyId
        static void DecodePolyId(long refs, out int salt, out int it, out int ip)
        {
            long saltMask = (1L << DT_SALT_BITS) - 1;
            long tileMask = (1L << DT_TILE_BITS) - 1;
            long polyMask = (1L << DT_POLY_BITS) - 1;
            salt = (int)((refs >> (DT_POLY_BITS + DT_TILE_BITS)) & saltMask);
            it = (int)((refs >> DT_POLY_BITS) & tileMask);
            ip = (int)(refs & polyMask);
        }

        /// Extracts a tile's salt value from the specified polygon reference.
        /// @note This function is generally meant for internal use only.
        /// @param[in] ref The polygon reference.
        /// @see #encodePolyId
        static int DecodePolyIdSalt(long refs)
        {
            long saltMask = (1L << DT_SALT_BITS) - 1;
            return (int)((refs >> (DT_POLY_BITS + DT_TILE_BITS)) & saltMask);
        }

        /// Extracts the tile's index from the specified polygon reference.
        /// @note This function is generally meant for internal use only.
        /// @param[in] ref The polygon reference.
        /// @see #encodePolyId
        public static int DecodePolyIdTile(long refs)
        {
            long tileMask = (1L << DT_TILE_BITS) - 1;
            return (int)((refs >> DT_POLY_BITS) & tileMask);
        }

        /// Extracts the polygon's index (within its tile) from the specified
        /// polygon reference.
        /// @note This function is generally meant for internal use only.
        /// @param[in] ref The polygon reference.
        /// @see #encodePolyId
        static int DecodePolyIdPoly(long refs)
        {
            long polyMask = (1L << DT_POLY_BITS) - 1;
            return (int)(refs & polyMask);
        }

        private static int AllocLink(DtMeshTile tile)
        {
            if (tile.linksFreeList == DT_NULL_LINK)
            {
                DtLink link = new()
                {
                    next = DT_NULL_LINK
                };
                tile.links.Add(link);
                return tile.links.Count - 1;
            }

            int linkIdx = tile.linksFreeList;
            tile.linksFreeList = tile.links[linkIdx].next;
            return linkIdx;
        }

        private static void FreeLink(DtMeshTile tile, int link)
        {
            tile.links[link].next = tile.linksFreeList;
            tile.linksFreeList = link;
        }

        /**
     * Calculates the tile grid location for the specified world position.
     *
     * @param pos
     *            The world position for the query. [(x, y, z)]
     * @return 2-element int array with (tx,ty) tile location
     */
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void CalcTileLoc(in Vector3 pos, out int tx, out int ty)
        {
            tx = (int)MathF.Floor((pos.X - m_params.orig.X) / m_tileWidth);
            ty = (int)MathF.Floor((pos.Z - m_params.orig.Y) / m_tileHeight);
        }

        /// Gets the tile and polygon for the specified polygon reference.
        ///  @param[in]		ref		The reference for the a polygon.
        ///  @param[out]	tile	The tile containing the polygon.
        ///  @param[out]	poly	The polygon.
        /// @return The status flags for the operation.
        public DtStatus GetTileAndPolyByRef(long refs, out DtMeshTile tile, out DtPoly poly)
        {
            tile = null;
            poly = null;

            if (refs is 0)
            {
                return DtStatus.DT_FAILURE;
            }

            DecodePolyId(refs, out var salt, out var it, out var ip);
            if (it >= MaxTiles)
            {
                return DtStatus.DT_FAILURE | DtStatus.DT_INVALID_PARAM;
            }

            if (m_tiles[it].salt != salt || m_tiles[it].data.header is null)
            {
                return DtStatus.DT_FAILURE | DtStatus.DT_INVALID_PARAM;
            }

            if (ip >= m_tiles[it].data.header.polyCount)
            {
                return DtStatus.DT_FAILURE | DtStatus.DT_INVALID_PARAM;
            }

            tile = m_tiles[it];
            poly = m_tiles[it].data.polys[ip];

            return DtStatus.DT_SUCCSESS;
        }

        /// @par
        ///
        /// @warning Only use this function if it is known that the provided polygon
        /// reference is valid. This function is faster than #getTileAndPolyByRef,
        /// but
        /// it does not validate the reference.
        public void GetTileAndPolyByRefUnsafe(long refs, out DtMeshTile tile, out DtPoly poly)
        {
            DecodePolyId(refs, out _, out var it, out var ip);
            tile = m_tiles[it];
            poly = m_tiles[it].data.polys[ip];
        }

        public bool IsValidPolyRef(long refs)
        {
            if (refs is 0)
            {
                return false;
            }

            DecodePolyId(refs, out var salt, out var it, out var ip);
            if (it >= MaxTiles)
            {
                return false;
            }

            if (m_tiles[it].salt != salt || m_tiles[it].data is null)
            {
                return false;
            }

            if (ip >= m_tiles[it].data.header.polyCount)
            {
                return false;
            }

            return true;
        }

        public ref readonly DtNavMeshParams GetParams()
        {
            return ref m_params;
        }


        // TODO: These methods are duplicates from dtNavMeshQuery, but are needed
        // for off-mesh connection finding.

        static List<long> QueryPolygonsInTile(DtMeshTile tile, Vector3 qmin, Vector3 qmax)
        {
            List<long> polys = new();
            if (tile.data.bvTree != null)
            {
                int nodeIndex = 0;
                var tbmin = tile.data.header.bmin;
                var tbmax = tile.data.header.bmax;
                float qfac = tile.data.header.bvQuantFactor;

                // dtClamp query box to world box quantized
                var min = (Vector3.Clamp(qmin, tbmin, tbmax) - tbmin) * qfac;
                var max = (Vector3.Clamp(qmax, tbmin, tbmax) - tbmin) * qfac;

                // Traverse tree
                long @base = GetPolyRefBase(tile);
                int end = tile.data.header.bvNodeCount;
                while (nodeIndex < end)
                {
                    DtBVNode node = tile.data.bvTree[nodeIndex];
                    bool overlap = DtUtils.OverlapQuantBounds(min, max, node.bmin, node.bmax);
                    bool isLeafNode = node.i >= 0;

                    if (isLeafNode && overlap)
                    {
                        polys.Add(@base | (long)node.i);
                    }

                    if (overlap || isLeafNode)
                    {
                        nodeIndex++;
                    }
                    else
                    {
                        int escapeIndex = -node.i;
                        nodeIndex += escapeIndex;
                    }
                }

                return polys;
            }
            else
            {
                Vector3 bmin = new();
                Vector3 bmax = new();
                long @base = GetPolyRefBase(tile);
                for (int i = 0; i < tile.data.header.polyCount; ++i)
                {
                    DtPoly p = tile.data.polys[i];
                    // Do not return off-mesh connection polygons.
                    if (p.GetPolyType() == DtPoly.DT_POLYTYPE_OFFMESH_CONNECTION)
                    {
                        continue;
                    }

                    // Calc polygon bounds.
                    int v = p.verts[0];
                    bmin.Set(tile.data.verts, v);
                    bmax.Set(tile.data.verts, v);
                    for (int j = 1; j < p.vertCount; ++j)
                    {
                        v = p.verts[j];
                        bmin.Min(tile.data.verts, v * 3);
                        bmax.Max(tile.data.verts, v * 3);
                    }

                    if (DtUtils.OverlapBounds(qmin, qmax, bmin, bmax))
                    {
                        polys.Add(@base | (long)i);
                    }
                }

                return polys;
            }
        }

        public long UpdateTile(DtMeshData data, int flags)
        {
            long refs = GetTileRefAt(data.header.x, data.header.y, data.header.layer);
            refs = RemoveTile(refs);
            return AddTile(data, flags, refs);
        }

        /// Adds a tile to the navigation mesh.
        /// @param[in] data Data for the new tile mesh. (See: #dtCreateNavMeshData)
        /// @param[in] dataSize Data size of the new tile mesh.
        /// @param[in] flags Tile flags. (See: #dtTileFlags)
        /// @param[in] lastRef The desired reference for the tile. (When reloading a
        /// tile.) [opt] [Default: 0]
        /// @param[out] result The tile reference. (If the tile was succesfully
        /// added.) [opt]
        /// @return The status flags for the operation.
        /// @par
        ///
        /// The add operation will fail if the data is in the wrong format, the
        /// allocated tile
        /// space is full, or there is a tile already at the specified reference.
        ///
        /// The lastRef parameter is used to restore a tile with the same tile
        /// reference it had previously used. In this case the #long's for the
        /// tile will be restored to the same values they were before the tile was
        /// removed.
        ///
        /// The nav mesh assumes exclusive access to the data passed and will make
        /// changes to the dynamic portion of the data. For that reason the data
        /// should not be reused in other nav meshes until the tile has been successfully
        /// removed from this nav mesh.
        ///
        /// @see dtCreateNavMeshData, #removeTile
        public long AddTile(DtMeshData data, int flags, long lastRef)
        {
            // Make sure the data is in right format.
            DtMeshHeader header = data.header;

            // Make sure the location is free.
            if (GetTileAt(header.x, header.y, header.layer) != null)
            {
                throw new Exception("Tile already exists");
            }

            // Allocate a tile.
            DtMeshTile tile;
            if (lastRef is 0)
            {
                // Make sure we could allocate a tile.
                if (availableTiles.Count is 0)
                {
                    throw new Exception("Could not allocate a tile");
                }

                tile = availableTiles.First?.Value;
                availableTiles.RemoveFirst();
                m_tileCount++;
            }
            else
            {
                // Try to relocate the tile to specific index with same salt.
                int tileIndex = DecodePolyIdTile(lastRef);
                if (tileIndex >= MaxTiles)
                {
                    throw new Exception("Tile index too high");
                }

                // Try to find the specific tile id from the free list.
                DtMeshTile target = m_tiles[tileIndex];
                // Remove from freelist
                if (!availableTiles.Remove(target))
                {
                    // Could not find the correct location.
                    throw new Exception("Could not find tile");
                }

                tile = target;
                // Restore salt.
                tile.salt = DecodePolyIdSalt(lastRef);
            }

            tile.data = data;
            tile.flags = flags;
            tile.links.Clear();
            tile.polyLinks = new int[data.polys.Length];
            Array.Fill(tile.polyLinks, DT_NULL_LINK);

            // Insert tile into the position lut.
            GetTileListByPos(header.x, header.y).Add(tile);

            // Patch header pointers.

            // If there are no items in the bvtree, reset the tree pointer.
            if (tile.data.bvTree != null && tile.data.bvTree.Length is 0)
            {
                tile.data.bvTree = null;
            }

            // Init tile.

            ConnectIntLinks(tile);
            // Base off-mesh connections to their starting polygons and connect connections inside the tile.
            BaseOffMeshLinks(tile);
            ConnectExtOffMeshLinks(tile, tile, -1);

            // Connect with layers in current tile.
            List<DtMeshTile> neis = GetTilesAt(header.x, header.y);
            for (int j = 0; j < neis.Count; ++j)
            {
                if (neis[j] == tile)
                {
                    continue;
                }

                ConnectExtLinks(tile, neis[j], -1);
                ConnectExtLinks(neis[j], tile, -1);
                ConnectExtOffMeshLinks(tile, neis[j], -1);
                ConnectExtOffMeshLinks(neis[j], tile, -1);
            }

            // Connect with neighbour tiles.
            for (int i = 0; i < 8; ++i)
            {
                neis = GetNeighbourTilesAt(header.x, header.y, i);
                for (int j = 0; j < neis.Count; ++j)
                {
                    ConnectExtLinks(tile, neis[j], i);
                    ConnectExtLinks(neis[j], tile, DtUtils.OppositeTile(i));
                    ConnectExtOffMeshLinks(tile, neis[j], i);
                    ConnectExtOffMeshLinks(neis[j], tile, DtUtils.OppositeTile(i));
                }
            }

            return GetTileRef(tile);
        }

        /// Removes the specified tile from the navigation mesh.
        /// @param[in] ref The reference of the tile to remove.
        /// @param[out] data Data associated with deleted tile.
        /// @param[out] dataSize Size of the data associated with deleted tile.
        ///
        /// This function returns the data for the tile so that, if desired,
        /// it can be added back to the navigation mesh at a later point.
        ///
        /// @see #addTile
        public long RemoveTile(long refs)
        {
            if (refs is 0)
            {
                return 0;
            }

            int tileIndex = DecodePolyIdTile(refs);
            int tileSalt = DecodePolyIdSalt(refs);
            if (tileIndex >= MaxTiles)
            {
                throw new Exception("Invalid tile index");
            }

            DtMeshTile tile = m_tiles[tileIndex];
            if (tile.salt != tileSalt)
            {
                throw new Exception("Invalid tile salt");
            }

            // Remove tile from hash lookup.
            GetTileListByPos(tile.data.header.x, tile.data.header.y).Remove(tile);

            // Remove connections to neighbour tiles.
            // Create connections with neighbour tiles.

            // Disconnect from other layers in current tile.
            List<DtMeshTile> nneis = GetTilesAt(tile.data.header.x, tile.data.header.y);
            foreach (DtMeshTile j in nneis)
            {
                if (j == tile)
                {
                    continue;
                }

                UnconnectLinks(j, tile);
            }

            // Disconnect from neighbour tiles.
            for (int i = 0; i < 8; ++i)
            {
                nneis = GetNeighbourTilesAt(tile.data.header.x, tile.data.header.y, i);
                foreach (DtMeshTile j in nneis)
                {
                    UnconnectLinks(j, tile);
                }
            }

            // Reset tile.
            tile.data = null;

            tile.flags = 0;
            tile.links.Clear();
            tile.linksFreeList = DT_NULL_LINK;

            // Update salt, salt should never be zero.
            tile.salt = (tile.salt + 1) & ((1 << DT_SALT_BITS) - 1);
            if (tile.salt is 0)
            {
                tile.salt++;
            }

            // Add to free list.
            availableTiles.AddFirst(tile);
            m_tileCount--;
            return GetTileRef(tile);
        }

        /// Builds internal polygons links for a tile.
        static void ConnectIntLinks(DtMeshTile tile)
        {
            if (tile is null)
            {
                return;
            }

            long @base = GetPolyRefBase(tile);

            for (int i = 0; i < tile.data.header.polyCount; ++i)
            {
                DtPoly poly = tile.data.polys[i];
                tile.polyLinks[poly.index] = DT_NULL_LINK;

                if (poly.GetPolyType() == DtPoly.DT_POLYTYPE_OFFMESH_CONNECTION)
                {
                    continue;
                }

                // Build edge links backwards so that the links will be
                // in the linked list from lowest index to highest.
                for (int j = poly.vertCount - 1; j >= 0; --j)
                {
                    // Skip hard and non-internal edges.
                    if (poly.neis[j] is 0 || (poly.neis[j] & DT_EXT_LINK) != 0)
                    {
                        continue;
                    }

                    int idx = AllocLink(tile);
                    DtLink link = tile.links[idx];
                    link.refs = @base | (long)(poly.neis[j] - 1);
                    link.edge = j;
                    link.side = 0xff;
                    link.bmin = link.bmax = 0;
                    // Add to linked list.
                    link.next = tile.polyLinks[poly.index];
                    tile.polyLinks[poly.index] = idx;
                }
            }
        }

        static void UnconnectLinks(DtMeshTile tile, DtMeshTile target)
        {
            if (tile is null || target is null)
            {
                return;
            }

            int targetNum = DecodePolyIdTile(GetTileRef(target));

            for (int i = 0; i < tile.data.header.polyCount; ++i)
            {
                DtPoly poly = tile.data.polys[i];
                int j = tile.polyLinks[poly.index];
                int pj = DT_NULL_LINK;
                while (j != DT_NULL_LINK)
                {
                    if (DecodePolyIdTile(tile.links[j].refs) == targetNum)
                    {
                        // Remove link.
                        int nj = tile.links[j].next;
                        if (pj == DT_NULL_LINK)
                        {
                            tile.polyLinks[poly.index] = nj;
                        }
                        else
                        {
                            tile.links[pj].next = nj;
                        }

                        FreeLink(tile, j);
                        j = nj;
                    }
                    else
                    {
                        // Advance
                        pj = j;
                        j = tile.links[j].next;
                    }
                }
            }
        }

        static void ConnectExtLinks(DtMeshTile tile, DtMeshTile target, int side)
        {
            if (tile is null)
            {
                return;
            }

            var connectPolys = new List<DtConnectPoly>();

            // Connect border links.
            for (int i = 0; i < tile.data.header.polyCount; ++i)
            {
                DtPoly poly = tile.data.polys[i];

                // Create new links.
                // short m = DT_EXT_LINK | (short)side;

                int nv = poly.vertCount;
                for (int j = 0; j < nv; ++j)
                {
                    // Skip non-portal edges.
                    if ((poly.neis[j] & DT_EXT_LINK) is 0)
                    {
                        continue;
                    }

                    int dir = poly.neis[j] & 0xff;
                    if (side != -1 && dir != side)
                    {
                        continue;
                    }

                    // Create new links
                    int va = poly.verts[j] * 3;
                    int vb = poly.verts[(j + 1) % nv] * 3;
                    FindConnectingPolys(tile.data.verts, va, vb, target, DtUtils.OppositeTile(dir), ref connectPolys);
                    foreach (var connectPoly in connectPolys)
                    {
                        int idx = AllocLink(tile);
                        DtLink link = tile.links[idx];
                        link.refs = connectPoly.refs;
                        link.edge = j;
                        link.side = dir;

                        link.next = tile.polyLinks[poly.index];
                        tile.polyLinks[poly.index] = idx;

                        // Compress portal limits to a byte value.
                        if (dir is 0 || dir == 4)
                        {
                            float tmin = (connectPoly.tmin - tile.data.verts[va + 2])
                                         / (tile.data.verts[vb + 2] - tile.data.verts[va + 2]);
                            float tmax = (connectPoly.tmax - tile.data.verts[va + 2])
                                         / (tile.data.verts[vb + 2] - tile.data.verts[va + 2]);
                            if (tmin > tmax)
                            {
                                (tmax, tmin) = (tmin, tmax);
                            }

                            link.bmin = (int)MathF.Round(Math.Clamp(tmin, 0f, 1f) * 255.0f);
                            link.bmax = (int)MathF.Round(Math.Clamp(tmax, 0f, 1f) * 255.0f);
                        }
                        else if (dir == 2 || dir == 6)
                        {
                            float tmin = (connectPoly.tmin - tile.data.verts[va])
                                         / (tile.data.verts[vb] - tile.data.verts[va]);
                            float tmax = (connectPoly.tmax - tile.data.verts[va])
                                         / (tile.data.verts[vb] - tile.data.verts[va]);
                            if (tmin > tmax)
                            {
                                (tmax, tmin) = (tmin, tmax);
                            }

                            link.bmin = (int)MathF.Round(Math.Clamp(tmin, 0f, 1f) * 255.0f);
                            link.bmax = (int)MathF.Round(Math.Clamp(tmax, 0f, 1f) * 255.0f);
                        }
                    }
                }
            }
        }

        void ConnectExtOffMeshLinks(DtMeshTile tile, DtMeshTile target, int side)
        {
            if (tile is null)
            {
                return;
            }

            // Connect off-mesh links.
            // We are interested on links which land from target tile to this tile.
            int oppositeSide = (side == -1) ? 0xff : DtUtils.OppositeTile(side);

            for (int i = 0; i < target.data.header.offMeshConCount; ++i)
            {
                DtOffMeshConnection targetCon = target.data.offMeshCons[i];
                if (targetCon.side != oppositeSide)
                {
                    continue;
                }

                DtPoly targetPoly = target.data.polys[targetCon.poly];
                // Skip off-mesh connections which start location could not be
                // connected at all.
                if (target.polyLinks[targetPoly.index] == DT_NULL_LINK)
                {
                    continue;
                }

                var ext = new Vector3()
                {
                    X = targetCon.rad,
                    Y = target.data.header.walkableClimb,
                    Z = targetCon.rad
                };

                // Find polygon to connect to.
                Vector3 p = new()
                {
                    X = targetCon.pos[3],
                    Y = targetCon.pos[4],
                    Z = targetCon.pos[5]
                };
                var refs = FindNearestPolyInTile(tile, p, ext, out var nearestPt);
                if (refs is 0)
                {
                    continue;
                }

                // findNearestPoly may return too optimistic results, further check
                // to make sure.

                if (RcMath.Sqr(nearestPt.X - p.X) + RcMath.Sqr(nearestPt.Z - p.Z) > RcMath.Sqr(targetCon.rad))
                {
                    continue;
                }

                // Make sure the location is on current mesh.
                target.data.verts[targetPoly.verts[1] * 3] = nearestPt.X;
                target.data.verts[targetPoly.verts[1] * 3 + 1] = nearestPt.Y;
                target.data.verts[targetPoly.verts[1] * 3 + 2] = nearestPt.Z;

                // Link off-mesh connection to target poly.
                int idx = AllocLink(target);
                DtLink link = target.links[idx];
                link.refs = refs;
                link.edge = 1;
                link.side = oppositeSide;
                link.bmin = link.bmax = 0;
                // Add to linked list.
                link.next = target.polyLinks[targetPoly.index];
                target.polyLinks[targetPoly.index] = idx;

                // Link target poly to off-mesh connection.
                if ((targetCon.flags & DT_OFFMESH_CON_BIDIR) != 0)
                {
                    int tidx = AllocLink(tile);
                    int landPolyIdx = DecodePolyIdPoly(refs);
                    DtPoly landPoly = tile.data.polys[landPolyIdx];
                    link = tile.links[tidx];
                    link.refs = GetPolyRefBase(target) | (long)targetCon.poly;
                    link.edge = 0xff;
                    link.side = side == -1 ? 0xff : side;
                    link.bmin = link.bmax = 0;
                    // Add to linked list.
                    link.next = tile.polyLinks[landPoly.index];
                    tile.polyLinks[landPoly.index] = tidx;
                }
            }
        }

        private static void FindConnectingPolys(float[] verts, int va, int vb, DtMeshTile tile, int side,
            ref List<DtConnectPoly> cons)
        {
            if (tile is null)
                return;

            cons.Clear();

            Vector2 amin = Vector2.Zero;
            Vector2 amax = Vector2.Zero;
            CalcSlabEndPoints(verts, va, vb, ref amin, ref amax, side);
            float apos = GetSlabCoord(verts, va, side);

            // Remove links pointing to 'side' and compact the links array.
            Vector2 bmin = Vector2.Zero;
            Vector2 bmax = Vector2.Zero;
            int m = DT_EXT_LINK | side;
            long @base = GetPolyRefBase(tile);

            for (int i = 0; i < tile.data.header.polyCount; ++i)
            {
                DtPoly poly = tile.data.polys[i];
                int nv = poly.vertCount;
                for (int j = 0; j < nv; ++j)
                {
                    // Skip edges which do not point to the right side.
                    if (poly.neis[j] != m)
                    {
                        continue;
                    }

                    int vc = poly.verts[j] * 3;
                    int vd = poly.verts[(j + 1) % nv] * 3;
                    float bpos = GetSlabCoord(tile.data.verts, vc, side);
                    // Segments are not close enough.
                    if (Math.Abs(apos - bpos) > 0.01f)
                    {
                        continue;
                    }

                    // Check if the segments touch.
                    CalcSlabEndPoints(tile.data.verts, vc, vd, ref bmin, ref bmax, side);

                    if (!OverlapSlabs(amin, amax, bmin, bmax, 0.01f, tile.data.header.walkableClimb))
                    {
                        continue;
                    }

                    // Add return value.
                    long refs = @base | (long)i;
                    float tmin = Math.Max(amin.X, bmin.X);
                    float tmax = Math.Min(amax.X, bmax.X);
                    cons.Add(new DtConnectPoly(refs, tmin, tmax));
                    break;
                }
            }
        }

        static float GetSlabCoord(float[] verts, int va, int side)
        {
            if (side is 0 || side == 4)
            {
                return verts[va];
            }
            else if (side == 2 || side == 6)
            {
                return verts[va + 2];
            }

            return 0;
        }

        static void CalcSlabEndPoints(float[] verts, int va, int vb, ref Vector2 bmin, ref Vector2 bmax, int side)
        {
            if (side is 0 || side == 4)
            {
                if (verts[va + 2] < verts[vb + 2])
                {
                    bmin.X = verts[va + 2];
                    bmin.Y = verts[va + 1];
                    bmax.X = verts[vb + 2];
                    bmax.Y = verts[vb + 1];
                }
                else
                {
                    bmin.X = verts[vb + 2];
                    bmin.Y = verts[vb + 1];
                    bmax.X = verts[va + 2];
                    bmax.Y = verts[va + 1];
                }
            }
            else if (side == 2 || side == 6)
            {
                var minIsVa = verts[va] < verts[vb];

                bmin = verts.GetUnsafe(minIsVa ? va : vb).UnsafeAs<float, Vector2>();
                bmax = verts.GetUnsafe(minIsVa ? vb : va).UnsafeAs<float, Vector2>();
            }
        }

        static bool OverlapSlabs(Vector2 amin, Vector2 amax, Vector2 bmin, Vector2 bmax, float px, float py)
        {
            // Check for horizontal overlap.
            // The segment is shrunken a little so that slabs which touch
            // at end points are not connected.
            float minx = MathF.Max(amin.X + px, bmin.X + px);
            float maxx = MathF.Min(amax.X - px, bmax.X - px);
            if (minx > maxx)
            {
                return false;
            }

            // Check vertical overlap.
            float ad = (amax.Y - amin.Y) / (amax.X - amin.X);
            float ak = amin.Y - ad * amin.X;
            float bd = (bmax.Y - bmin.Y) / (bmax.X - bmin.X);
            float bk = bmin.Y - bd * bmin.X;
            float aminy = ad * minx + ak;
            float amaxy = ad * maxx + ak;
            float bminy = bd * minx + bk;
            float bmaxy = bd * maxx + bk;
            float dmin = bminy - aminy;
            float dmax = bmaxy - amaxy;

            // Crossing segments always overlap.
            if (dmin * dmax < 0)
            {
                return true;
            }

            // Check for overlap at endpoints.
            float thr = py * 2 * (py * 2);
            if (dmin * dmin <= thr || dmax * dmax <= thr)
            {
                return true;
            }

            return false;
        }

        /**
     * Builds internal polygons links for a tile.
     *
     * @param tile
     */
        void BaseOffMeshLinks(DtMeshTile tile)
        {
            if (tile is null)
            {
                return;
            }

            long @base = GetPolyRefBase(tile);

            // Base off-mesh connection start points.
            for (int i = 0; i < tile.data.header.offMeshConCount; ++i)
            {
                DtOffMeshConnection con = tile.data.offMeshCons[i];
                DtPoly poly = tile.data.polys[con.poly];

                var ext = new Vector3()
                {
                    X = con.rad,
                    Y = tile.data.header.walkableClimb,
                    Z = con.rad,
                };

                // Find polygon to connect to.
                var refs = FindNearestPolyInTile(tile, new Vector3(con.pos[0], con.pos[1], con.pos[2]), ext, out var nearestPt);
                if (refs is 0)
                {
                    continue;
                }

                float[] p = con.pos; // First vertex
                                     // findNearestPoly may return too optimistic results, further check
                                     // to make sure.
                if (RcMath.Sqr(nearestPt.X - p[0]) + RcMath.Sqr(nearestPt.Z - p[2]) > RcMath.Sqr(con.rad))
                {
                    continue;
                }

                // Make sure the location is on current mesh.
                tile.data.verts[poly.verts[0] * 3] = nearestPt.X;
                tile.data.verts[poly.verts[0] * 3 + 1] = nearestPt.Y;
                tile.data.verts[poly.verts[0] * 3 + 2] = nearestPt.Z;

                // Link off-mesh connection to target poly.
                int idx = AllocLink(tile);
                DtLink link = tile.links[idx];
                link.refs = refs;
                link.edge = 0;
                link.side = 0xff;
                link.bmin = link.bmax = 0;
                // Add to linked list.
                link.next = tile.polyLinks[poly.index];
                tile.polyLinks[poly.index] = idx;

                // Start end-point is always connect back to off-mesh connection.
                int tidx = AllocLink(tile);
                int landPolyIdx = DecodePolyIdPoly(refs);
                DtPoly landPoly = tile.data.polys[landPolyIdx];
                link = tile.links[tidx];
                link.refs = @base | (long)con.poly;
                link.edge = 0xff;
                link.side = 0xff;
                link.bmin = link.bmax = 0;
                // Add to linked list.
                link.next = tile.polyLinks[landPoly.index];
                tile.polyLinks[landPoly.index] = tidx;
            }
        }

        /**
     * Returns closest point on polygon.
     *
     * @param ref
     * @param pos
     * @return
     */
        static Vector3 ClosestPointOnDetailEdges(DtMeshTile tile, DtPoly poly, Vector3 pos, bool onlyBoundary)
        {
            int ANY_BOUNDARY_EDGE = (DT_DETAIL_EDGE_BOUNDARY << 0) | (DT_DETAIL_EDGE_BOUNDARY << 2)
                                                                   | (DT_DETAIL_EDGE_BOUNDARY << 4);
            int ip = poly.index;
            float dmin = float.MaxValue;
            float tmin = 0;
            Vector3 pmin = new();
            Vector3 pmax = new();

            if (tile.data.detailMeshes != null)
            {
                DtPolyDetail pd = tile.data.detailMeshes[ip];
                for (int i = 0; i < pd.triCount; i++)
                {
                    int ti = pd.triBase + i;
                    var tris = tile.data.detailTris;
                    if (onlyBoundary && (tris[ti].w & ANY_BOUNDARY_EDGE) is 0)
                    {
                        continue;
                    }

                    Vector3[] v = new Vector3[3];
                    for (int j = 0; j < 3; ++j)
                    {
                        if (tris[ti][j] < poly.vertCount)
                        {
                            int index = poly.verts[tris[ti][j]] * 3;
                            v[j] = new Vector3
                            {
                                X = tile.data.verts[index],
                                Y = tile.data.verts[index + 1],
                                Z = tile.data.verts[index + 2]
                            };
                        }
                        else
                        {
                            int index = pd.vertBase + (tris[ti][j] - poly.vertCount);
                            v[j] = tile.data.detailVerts[index];
                        }
                    }

                    for (int k = 0, j = 2; k < 3; j = k++)
                    {
                        if ((GetDetailTriEdgeFlags(tris[ti].w, j) & DT_DETAIL_EDGE_BOUNDARY) is 0
                            && (onlyBoundary || tris[ti][j] < tris[ti][k]))
                        {
                            // Only looking at boundary edges and this is internal, or
                            // this is an inner edge that we will see again or have already seen.
                            continue;
                        }

                        var d = DtUtils.DistancePtSegSqr2D(pos, v[j], v[k], out var t);
                        if (d < dmin)
                        {
                            dmin = d;
                            tmin = t;
                            pmin = v[j];
                            pmax = v[k];
                        }
                    }
                }
            }
            else
            {
                Vector3[] v = new Vector3[2];
                for (int j = 0; j < poly.vertCount; ++j)
                {
                    int k = (j + 1) % poly.vertCount;
                    v[0].X = tile.data.verts[poly.verts[j] * 3];
                    v[0].Y = tile.data.verts[poly.verts[j] * 3 + 1];
                    v[0].Z = tile.data.verts[poly.verts[j] * 3 + 2];
                    v[1].X = tile.data.verts[poly.verts[k] * 3];
                    v[1].Y = tile.data.verts[poly.verts[k] * 3 + 1];
                    v[1].Z = tile.data.verts[poly.verts[k] * 3 + 2];

                    var d = DtUtils.DistancePtSegSqr2D(pos, v[0], v[1], out var t);
                    if (d < dmin)
                    {
                        dmin = d;
                        tmin = t;
                        pmin = v[0];
                        pmax = v[1];
                    }
                }
            }

            return Vector3.Lerp(pmin, pmax, tmin);
        }

        public bool GetPolyHeight(DtMeshTile tile, DtPoly poly, Vector3 pos, out float height)
        {
            height = 0;

            // Off-mesh connections do not have detail polys and getting height
            // over them does not make sense.
            if (poly.GetPolyType() == DtPoly.DT_POLYTYPE_OFFMESH_CONNECTION)
            {
                return false;
            }

            int ip = poly.index;

            var verts = new Vector3[m_maxVertPerPoly];
            int nv = poly.vertCount;
            for (int i = 0; i < nv; ++i)
            {
                verts[i] = tile.data.verts.UnsafeAs<float, Vector3>(poly.verts[i]);
            }

            if (!DtUtils.PointInPolygon(pos, verts, nv))
            {
                return false;
            }

            // Find height at the location.
            if (tile.data.detailMeshes != null)
            {
                DtPolyDetail pd = tile.data.detailMeshes[ip];
                for (int j = 0; j < pd.triCount; ++j)
                {
                    int t = pd.triBase + j;
                    Vector3[] v = new Vector3[3];
                    for (int k = 0; k < 3; ++k)
                    {
                        var tri = tile.data.detailTris.GetUnsafe(t).UnsafeAs<Vector4Int, int>(k);
                        if (tri < poly.vertCount)
                        {
                            int index = poly.verts[tri] * 3;
                            v[k] = new Vector3
                            {
                                X = tile.data.verts[index],
                                Y = tile.data.verts[index + 1],
                                Z = tile.data.verts[index + 2]
                            };
                        }
                        else
                        {
                            int index = pd.vertBase + (tri - poly.vertCount);
                            v[k] = tile.data.detailVerts[index];
                        }
                    }

                    if (DtUtils.ClosestHeightPointTriangle(pos, v[0], v[1], v[2], out var h))
                    {
                        height = h;
                        return true;
                    }
                }
            }
            else
            {
                Vector3[] v = new Vector3[3];
                v[0].X = tile.data.verts[poly.verts[0] * 3];
                v[0].Y = tile.data.verts[poly.verts[0] * 3 + 1];
                v[0].Z = tile.data.verts[poly.verts[0] * 3 + 2];
                for (int j = 1; j < poly.vertCount - 1; ++j)
                {
                    for (int k = 0; k < 2; ++k)
                    {
                        v[k + 1].X = tile.data.verts[poly.verts[j + k] * 3];
                        v[k + 1].Y = tile.data.verts[poly.verts[j + k] * 3 + 1];
                        v[k + 1].Z = tile.data.verts[poly.verts[j + k] * 3 + 2];
                    }

                    if (DtUtils.ClosestHeightPointTriangle(pos, v[0], v[1], v[2], out var h))
                    {
                        height = h;
                        return true;
                    }
                }
            }

            // If all triangle checks failed above (can happen with degenerate triangles
            // or larger floating point values) the point is on an edge, so just select
            // closest. This should almost never happen so the extra iteration here is
            // ok.
            var closest = ClosestPointOnDetailEdges(tile, poly, pos, false);
            height = closest.Y;
            return true;
        }

        public void ClosestPointOnPoly(long refs, Vector3 pos, out Vector3 closest, out bool posOverPoly)
        {
            GetTileAndPolyByRefUnsafe(refs, out var tile, out var poly);
            closest = pos;

            if (GetPolyHeight(tile, poly, pos, out var h))
            {
                closest.Y = h;
                posOverPoly = true;
                return;
            }

            posOverPoly = false;

            // Off-mesh connections don't have detail polygons.
            if (poly.GetPolyType() == DtPoly.DT_POLYTYPE_OFFMESH_CONNECTION)
            {
                int i = poly.verts[0] * 3;
                var v0 = new Vector3 { X = tile.data.verts[i], Y = tile.data.verts[i + 1], Z = tile.data.verts[i + 2] };
                i = poly.verts[1] * 3;
                var v1 = new Vector3 { X = tile.data.verts[i], Y = tile.data.verts[i + 1], Z = tile.data.verts[i + 2] };
                DtUtils.DistancePtSegSqr2D(pos, v0, v1, out var t);
                closest = Vector3.Lerp(v0, v1, t);
                return;
            }

            // Outside poly that is not an offmesh connection.
            closest = ClosestPointOnDetailEdges(tile, poly, pos, true);
        }

        /// Find nearest polygon within a tile.
        private long FindNearestPolyInTile(DtMeshTile tile, Vector3 center, Vector3 halfExtents, out Vector3 nearestPt)
        {
            nearestPt = Vector3.Zero;

            Vector3 bmin = center - halfExtents;
            Vector3 bmax = center + halfExtents;

            // Get nearby polygons from proximity grid.
            List<long> polys = QueryPolygonsInTile(tile, bmin, bmax);

            // Find nearest polygon amongst the nearby polygons.
            long nearest = 0;
            float nearestDistanceSqr = float.MaxValue;
            for (int i = 0; i < polys.Count; ++i)
            {
                long refs = polys[i];
                float d;
                ClosestPointOnPoly(refs, center, out var closestPtPoly, out var posOverPoly);

                // If a point is directly over a polygon and closer than
                // climb height, favor that instead of straight line nearest point.
                Vector3 diff = center - closestPtPoly;
                if (posOverPoly)
                {
                    d = Math.Abs(diff.Y) - tile.data.header.walkableClimb;
                    d = d > 0 ? d * d : 0;
                }
                else
                {
                    d = diff.LengthSquared();
                }

                if (d < nearestDistanceSqr)
                {
                    nearestPt = closestPtPoly;
                    nearestDistanceSqr = d;
                    nearest = refs;
                }
            }

            return nearest;
        }

        DtMeshTile GetTileAt(int x, int y, int layer)
        {
            foreach (DtMeshTile tile in GetTileListByPos(x, y))
            {
                if (tile.data.header != null && tile.data.header.x == x && tile.data.header.y == y
                    && tile.data.header.layer == layer)
                {
                    return tile;
                }
            }

            return null;
        }

        List<DtMeshTile> GetNeighbourTilesAt(int x, int y, int side)
        {
            int nx = x, ny = y;
            switch (side)
            {
                case 0:
                    nx++;
                    break;
                case 1:
                    nx++;
                    ny++;
                    break;
                case 2:
                    ny++;
                    break;
                case 3:
                    nx--;
                    ny++;
                    break;
                case 4:
                    nx--;
                    break;
                case 5:
                    nx--;
                    ny--;
                    break;
                case 6:
                    ny--;
                    break;
                case 7:
                    nx++;
                    ny--;
                    break;
            }

            return GetTilesAt(nx, ny);
        }

        public List<DtMeshTile> GetTilesAt(int x, int y)
        {
            List<DtMeshTile> tiles = new();
            foreach (DtMeshTile tile in GetTileListByPos(x, y))
            {
                if (tile.data.header?.x == x && tile.data.header.y == y)
                {
                    tiles.Add(tile);
                }
            }

            return tiles;
        }

        public long GetTileRefAt(int x, int y, int layer)
        {
            return GetTileRef(GetTileAt(x, y, layer));
        }

        public DtMeshTile GetTileByRef(long refs)
        {
            if (refs is 0)
            {
                return null;
            }

            int tileIndex = DecodePolyIdTile(refs);
            int tileSalt = DecodePolyIdSalt(refs);
            if (tileIndex >= MaxTiles)
            {
                return null;
            }

            DtMeshTile tile = m_tiles[tileIndex];
            if (tile.salt != tileSalt)
            {
                return null;
            }

            return tile;
        }

        public static long GetTileRef(DtMeshTile tile)
        {
            if (tile is null)
            {
                return 0;
            }

            return EncodePolyId(tile.salt, tile.index, 0);
        }

        public static int ComputeTileHash(int x, int y, int mask)
        {
            uint h1 = 0x8da6b343; // Large multiplicative constants;
            uint h2 = 0xd8163841; // here arbitrarily chosen primes
            uint n = h1 * (uint)x + h2 * (uint)y;
            return (int)(n & mask);
        }

        /// Gets the endpoints for an off-mesh connection, ordered by "direction of travel".
        ///  @param[in]		prevRef		The reference of the polygon before the connection.
        ///  @param[in]		polyRef		The reference of the off-mesh connection polygon.
        ///  @param[out]	startPos	The start position of the off-mesh connection. [(x, y, z)]
        ///  @param[out]	endPos		The end position of the off-mesh connection. [(x, y, z)]
        /// @return The status flags for the operation.
        /// 
        /// @par
        ///
        /// Off-mesh connections are stored in the navigation mesh as special 2-vertex 
        /// polygons with a single edge. At least one of the vertices is expected to be 
        /// inside a normal polygon. So an off-mesh connection is "entered" from a 
        /// normal polygon at one of its endpoints. This is the polygon identified by 
        /// the prevRef parameter.
        public DtStatus GetOffMeshConnectionPolyEndPoints(long prevRef, long polyRef, ref Vector3 startPos, ref Vector3 endPos)
        {
            if (polyRef is 0)
            {
                return DtStatus.DT_FAILURE;
            }

            // Get current polygon
            DecodePolyId(polyRef, out var salt, out var it, out var ip);
            if (it >= MaxTiles)
            {
                return DtStatus.DT_FAILURE | DtStatus.DT_INVALID_PARAM;
            }

            if (m_tiles[it].salt != salt || m_tiles[it].data.header is null)
            {
                return DtStatus.DT_FAILURE | DtStatus.DT_INVALID_PARAM;
            }

            DtMeshTile tile = m_tiles[it];
            if (ip >= tile.data.header.polyCount)
            {
                return DtStatus.DT_FAILURE | DtStatus.DT_INVALID_PARAM;
            }

            DtPoly poly = tile.data.polys[ip];

            // Make sure that the current poly is indeed off-mesh link.
            if (poly.GetPolyType() != DtPoly.DT_POLYTYPE_OFFMESH_CONNECTION)
            {
                return DtStatus.DT_FAILURE;
            }

            // Figure out which way to hand out the vertices.
            int idx0 = 0, idx1 = 1;

            // Find link that points to first vertex.
            for (int i = tile.polyLinks[poly.index]; i != DT_NULL_LINK; i = tile.links[i].next)
            {
                if (tile.links[i].edge is 0)
                {
                    if (tile.links[i].refs != prevRef)
                    {
                        idx0 = 1;
                        idx1 = 0;
                    }

                    break;
                }
            }

            startPos = Vector3Extensions.Of(tile.data.verts, poly.verts[idx0] * 3);
            endPos = Vector3Extensions.Of(tile.data.verts, poly.verts[idx1] * 3);

            return DtStatus.DT_SUCCSESS;
        }

        public int GetMaxVertsPerPoly()
        {
            return m_maxVertPerPoly;
        }

        public int GetTileCount()
        {
            return m_tileCount;
        }

        public int GetAvailableTileCount()
        {
            return availableTiles.Count;
        }

        public DtStatus SetPolyFlags(long refs, int flags)
        {
            if (refs is 0)
            {
                return DtStatus.DT_FAILURE;
            }

            DecodePolyId(refs, out var salt, out var it, out var ip);
            if (it >= MaxTiles)
            {
                return DtStatus.DT_INVALID_PARAM;
            }

            if (m_tiles[it].salt != salt || m_tiles[it].data is null || m_tiles[it].data.header is null)
            {
                return DtStatus.DT_INVALID_PARAM;
            }

            DtMeshTile tile = m_tiles[it];
            if (ip >= tile.data.header.polyCount)
            {
                return DtStatus.DT_INVALID_PARAM;
            }

            DtPoly poly = tile.data.polys[ip];

            // Change flags.
            poly.flags = flags;
            return DtStatus.DT_SUCCSESS;
        }

        /// Gets the user defined flags for the specified polygon.
        ///  @param[in]		ref				The polygon reference.
        ///  @param[out]	resultFlags		The polygon flags.
        /// @return The status flags for the operation.
        public DtStatus GetPolyFlags(long refs, out int resultFlags)
        {
            resultFlags = 0;

            if (refs is 0)
            {
                return DtStatus.DT_FAILURE;
            }

            DecodePolyId(refs, out var salt, out var it, out var ip);
            if (it >= MaxTiles)
            {
                return DtStatus.DT_FAILURE | DtStatus.DT_INVALID_PARAM;
            }

            if (m_tiles[it].salt != salt || m_tiles[it].data is null || m_tiles[it].data.header is null)
            {
                return DtStatus.DT_FAILURE | DtStatus.DT_INVALID_PARAM;
            }

            DtMeshTile tile = m_tiles[it];
            if (ip >= tile.data.header.polyCount)
            {
                return DtStatus.DT_FAILURE | DtStatus.DT_INVALID_PARAM;
            }

            DtPoly poly = tile.data.polys[ip];

            resultFlags = poly.flags;

            return DtStatus.DT_SUCCSESS;
        }

        public DtStatus SetPolyArea(long refs, char area)
        {
            if (refs is 0)
            {
                return DtStatus.DT_FAILURE;
            }

            DecodePolyId(refs, out var salt, out var it, out var ip);
            if (it >= MaxTiles)
            {
                return DtStatus.DT_FAILURE;
            }

            if (m_tiles[it].salt != salt || m_tiles[it].data is null || m_tiles[it].data.header is null)
            {
                return DtStatus.DT_INVALID_PARAM;
            }

            DtMeshTile tile = m_tiles[it];
            if (ip >= tile.data.header.polyCount)
            {
                return DtStatus.DT_INVALID_PARAM;
            }

            DtPoly poly = tile.data.polys[ip];

            poly.SetArea(area);

            return DtStatus.DT_SUCCSESS;
        }

        public DtStatus GetPolyArea(long refs, out int resultArea)
        {
            resultArea = 0;

            if (refs is 0)
            {
                return DtStatus.DT_FAILURE;
            }

            DecodePolyId(refs, out var salt, out var it, out var ip);
            if (it >= MaxTiles)
            {
                return DtStatus.DT_FAILURE | DtStatus.DT_INVALID_PARAM;
            }

            if (m_tiles[it].salt != salt || m_tiles[it].data is null || m_tiles[it].data.header is null)
            {
                return DtStatus.DT_FAILURE | DtStatus.DT_INVALID_PARAM;
            }

            DtMeshTile tile = m_tiles[it];
            if (ip >= tile.data.header.polyCount)
            {
                return DtStatus.DT_FAILURE | DtStatus.DT_INVALID_PARAM;
            }

            DtPoly poly = tile.data.polys[ip];
            resultArea = poly.GetArea();

            return DtStatus.DT_SUCCSESS;
        }

        public Vector3 GetPolyCenter(long refs)
        {
            Vector3 center = Vector3.Zero;

            var status = GetTileAndPolyByRef(refs, out var tile, out var poly);
            if (status.Succeeded())
            {
                for (int i = 0; i < poly.vertCount; ++i)
                {
                    int v = poly.verts[i] * 3;
                    center.X += tile.data.verts[v];
                    center.Y += tile.data.verts[v + 1];
                    center.Z += tile.data.verts[v + 2];
                }

                float s = 1f / poly.vertCount;
                center.X *= s;
                center.Y *= s;
                center.Z *= s;
            }

            return center;
        }

        /**
     * Get flags for edge in detail triangle.
     *
     * @param triFlags
     *            The flags for the triangle (last component of detail vertices above).
     * @param edgeIndex
     *            The index of the first vertex of the edge. For instance, if 0,
     * @return flags for edge AB.
     */
        public static int GetDetailTriEdgeFlags(int triFlags, int edgeIndex)
        {
            return (triFlags >> (edgeIndex * 2)) & 0x3;
        }

        private List<DtMeshTile> GetTileListByPos(int x, int z)
        {
            var tileHash = ComputeTileHash(x, z, m_tileLutMask);
            if (!posLookup.TryGetValue(tileHash, out var tiles))
            {
                tiles = new List<DtMeshTile>();
                posLookup.Add(tileHash, tiles);
            }

            return tiles;
        }

        public void ComputeBounds(out Vector3 bmin, out Vector3 bmax)
        {
            bmin = new Vector3(float.PositiveInfinity, float.PositiveInfinity, float.PositiveInfinity);
            bmax = new Vector3(float.NegativeInfinity, float.NegativeInfinity, float.NegativeInfinity);
            for (int t = 0; t < MaxTiles; ++t)
            {
                DtMeshTile tile = GetTile(t);
                if (tile != null && tile.data != null)
                {
                    for (int i = 0; i < tile.data.verts.Length; i += 3)
                    {
                        bmin.X = MathF.Min(bmin.X, tile.data.verts[i]);
                        bmin.Y = MathF.Min(bmin.Y, tile.data.verts[i + 1]);
                        bmin.Z = MathF.Min(bmin.Z, tile.data.verts[i + 2]);
                        bmax.X = MathF.Max(bmax.X, tile.data.verts[i]);
                        bmax.Y = MathF.Max(bmax.Y, tile.data.verts[i + 1]);
                        bmax.Z = MathF.Max(bmax.Z, tile.data.verts[i + 2]);
                    }
                }
            }
        }
    }
}