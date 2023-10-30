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

namespace DotRecast.Detour
{
    public class DtNodePool
    {
        private readonly Dictionary<long, List<DtNode>> m_map = new();
        private readonly List<DtNode> m_nodes = new();

        public DtNodePool()
        {
        }

        public void Clear()
        {
            m_nodes.Clear();
            m_map.Clear();
        }

        public List<DtNode> FindNodes(long id)
        {
            if (!m_map.TryGetValue(id, out var nodes))
            {
                nodes = new List<DtNode>();
            }

            return nodes;
        }

        public DtNode FindNode(long id)
        {
            if (m_map.TryGetValue(id, out var nodes) && nodes.Count != 0)
            {
                return nodes[0];
            }

            return null;
        }

        public DtNode GetNode(long id, int state)
        {
            m_map.TryGetValue(id, out var nodes);
            if (nodes != null)
            {
                foreach (DtNode node in nodes)
                {
                    if (node.state == state)
                    {
                        return node;
                    }
                }
            }
            else
            {
                nodes = new List<DtNode>();
                m_map.Add(id, nodes);
            }

            return Create(id, state, nodes);
        }

        private DtNode Create(long id, int state, List<DtNode> nodes)
        {
            DtNode node = new(m_nodes.Count + 1)
            {
                id = id,
                state = state
            };
            m_nodes.Add(node);

            nodes.Add(node);
            return node;
        }

        public static int GetNodeIdx(DtNode node)
        {
            return node != null ? node.index : 0;
        }

        public DtNode GetNodeAtIdx(int idx)
        {
            return idx != 0 ? m_nodes[idx - 1] : null;
        }

        public DtNode GetNode(long refs)
        {
            return GetNode(refs, 0);
        }

        public Dictionary<long, List<DtNode>> GetNodeMap()
        {
            return m_map;
        }
    }
}