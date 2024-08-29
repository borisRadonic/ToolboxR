#pragma once

#include <cstdint>
#include <cstddef>

namespace CntrlLibrary
{     
    class MemUtilities
    {
    public:

        MemUtilities() = delete;
            

        /**
        * @brief Safely copies a uint32_t value to an int32_t destination.
        *
        * This inline function copies a uint32_t value from aPtrSrc to aPtrDest, where aPtrDest is an int32_t pointer.
        *
        * @param aPtrDest Pointer to the destination (int32_t).
        * @param aPtrSrc Pointer to the source (uint32_t).
        */
        inline static void safeCopyUint32ToInt32(void* aPtrDest, const std::uint32_t* aPtrSrc)
        {
            if ((aPtrDest != nullptr) && (aPtrSrc != nullptr))
            {
                std::uint32_t* tPtrDest = static_cast<std::uint32_t*>(static_cast<void*>(aPtrDest));
                *tPtrDest = *aPtrSrc;
            }
        }

    };
}

