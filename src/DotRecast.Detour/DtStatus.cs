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

using System.Diagnostics;
using System.Runtime.CompilerServices;

namespace DotRecast.Detour
{
    [DebuggerDisplay("Status: {Check()}")]
    public readonly struct DtStatus
    {
        private string Check()
        {
            string r = "";

            r += (Value & DT_FAILURE.Value) != 0 ? nameof(DT_FAILURE) + " " : "";
            r += (Value & DT_SUCCSESS.Value) != 0 ? nameof(DT_SUCCSESS) + " " : "";
            r += (Value & DT_IN_PROGRESS.Value) != 0 ? nameof(DT_IN_PROGRESS) + " " : "";
            r += (Value & DT_STATUS_DETAIL_MASK.Value) != 0 ? nameof(DT_STATUS_DETAIL_MASK) + " " : "";
            r += (Value & DT_STATUS_NOTHING.Value) != 0 ? nameof(DT_STATUS_NOTHING) + " " : "";
            r += (Value & DT_INVALID_PARAM.Value) != 0 ? nameof(DT_INVALID_PARAM) + " " : "";
            r += (Value & DT_BUFFER_TOO_SMALL.Value) != 0 ? nameof(DT_BUFFER_TOO_SMALL) + " " : "";
            r += (Value & DT_OUT_OF_NODES.Value) != 0 ? nameof(DT_OUT_OF_NODES) + " " : "";
            r += (Value & DT_PARTIAL_RESULT.Value) != 0 ? nameof(DT_PARTIAL_RESULT) : "";

            return r;
        }

        // High level status.
        public static readonly DtStatus DT_FAILURE = new(1u << 31); // Operation failed. 
        public static readonly DtStatus DT_SUCCSESS = new(1u << 30); // Operation succeed. 
        public static readonly DtStatus DT_IN_PROGRESS = new(1u << 29); // Operation still in progress. 

        // Detail information for status.
        public static readonly DtStatus DT_STATUS_DETAIL_MASK = new(0x0ffffff);
        public static readonly DtStatus DT_STATUS_NOTHING = new(0); // nothing
        public static readonly DtStatus DT_INVALID_PARAM = new(1 << 3); // An input parameter was invalid.
        public static readonly DtStatus DT_BUFFER_TOO_SMALL = new(1 << 4); // Result buffer for the query was too small to store all results.
        public static readonly DtStatus DT_OUT_OF_NODES = new(1 << 5); // Query ran out of nodes during search.
        public static readonly DtStatus DT_PARTIAL_RESULT = new(1 << 6); // Query did not reach the end location, returning best guess. 

        public readonly uint Value;

        private DtStatus(uint value)
        {
            Value = value;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public bool IsEmpty()
        {
            return 0 == Value;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public bool Succeeded()
        {
            return 0 != (Value & (DT_SUCCSESS.Value | DT_PARTIAL_RESULT.Value));
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public bool Failed()
        {
            return 0 != (Value & (DT_FAILURE.Value | DT_INVALID_PARAM.Value));
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public bool InProgress()
        {
            return 0 != (Value & DT_IN_PROGRESS.Value);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public bool IsPartial()
        {
            return 0 != (Value & DT_PARTIAL_RESULT.Value);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static DtStatus operator |(DtStatus left, DtStatus right)
        {
            return new DtStatus(left.Value | right.Value);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static DtStatus operator &(DtStatus left, DtStatus right)
        {
            return new DtStatus(left.Value & right.Value);
        }
    }
}