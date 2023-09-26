/*
recast4j copyright (c) 2015-2019 Piotr Piastucki piotr@jtilia.org

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
using System.IO.Compression;
using DotRecast.Core;

namespace DotRecast.Detour.Extras.Unity.Astar
{
    class GraphConnectionReader : ZipBinaryReader
    {
        public static List<int[]> Read(ZipArchive file, string filename, Meta meta, int[] indexToNode)
        {
            List<int[]> connections = new();
            RcByteBuffer buffer = ToByteBuffer(file, filename);
            while (buffer.Remaining() > 0)
            {
                int count = buffer.GetInt();
                int[] nodeConnections = new int[count];
                connections.Add(nodeConnections);
                for (int i = 0; i < count; i++)
                {
                    int nodeIndex = buffer.GetInt();
                    nodeConnections[i] = indexToNode[nodeIndex];

                    // XXX: Is there anything we can do with the cost?
                    _ = buffer.GetInt();
                    if (meta.IsVersionAtLeast(Meta.UPDATED_STRUCT_VERSION))
                    {
                        _ = buffer.Get();
                    }
                }
            }

            return connections;
        }
    }
}