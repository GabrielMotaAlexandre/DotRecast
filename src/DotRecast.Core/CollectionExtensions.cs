using System;
using System.Collections.Generic;

namespace DotRecast.Core
{
    public static class CollectionExtensions
    {
        /// Sorts the given data in-place using insertion sort.
        ///
        /// @param	data		The data to sort
        /// @param	dataLength	The number of elements in @p data
        public static void InsertSort(this int[] data)
        {
            for (int valueIndex = 1; valueIndex < data.Length; valueIndex++)
            {
                int value = data[valueIndex];
                int insertionIndex;
                for (insertionIndex = valueIndex - 1; insertionIndex >= 0 && data[insertionIndex] > value; insertionIndex--)
                {
                    // Shift over values
                    data[insertionIndex + 1] = data[insertionIndex];
                }

                // Insert the value in sorted order.
                data[insertionIndex + 1] = value;
            }
        }

        public static void ForEach<T>(this IEnumerable<T> collection, Action<T> action)
        {
            foreach (var item in collection)
            {
                action.Invoke(item);
            }
        }
    }
}